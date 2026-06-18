/**
 * The MIT License (MIT)
 * Copyright (c) 2026 Advanced Micro Devices, Inc. All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software
 * and associated documentation files (the "Software"), to deal in the Software without restriction,
 * including without limitation the rights to use, copy, modify, merge, publish, distribute,
 * sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
 * NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

/**
 * @file model_client.c
 * @brief Implementation of the daemon-side vpp_sim REQ client (see model_client.h).
 *
 * Hand-rolled minimal JSON: the dialect is tiny and fixed-shape (a handful of
 * objects to emit, three reply shapes to parse: the literal "OK", a bare uint,
 * and a JSON array of byte ints), so we build/parse it directly rather than
 * pulling jsoncpp into a C11 target.  The bytes are byte-exact with sim.cpp /
 * ZmqServer (docs/bridge-protocol.md §2).
 */

#define _GNU_SOURCE

#include "model_client.h"

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <zmq.h>

#include "utils.h"

struct emu_model_client {
    void *ctx;        /* owning ZMQ context */
    void *sock;       /* owning ZMQ_REQ socket */
    bool dead;        /* latched on the first transport error (see header) */
};

/*
 * Map a failed ZMQ call to a negative errno.  zmq sets errno; a receive timeout
 * surfaces as EAGAIN, which we normalise to -ETIMEDOUT so the caller (and the
 * bar/qdma seams) see a clear transport failure rather than a spurious retry
 * hint.  Any transport error latches the client dead.
 */
static int client_fail(struct emu_model_client *c)
{
    int e = errno;
    c->dead = true;
    if (e == EAGAIN) {
        return -ETIMEDOUT;
    }
    return e != 0 ? -e : -EIO;
}

int emu_model_client_connect(const char *endpoint, int timeout_ms,
                             struct emu_model_client **out)
{
    if (endpoint == NULL || out == NULL || timeout_ms <= 0) {
        return -EINVAL;
    }

    struct emu_model_client *c = calloc(1, sizeof(*c));
    if (c == NULL) {
        return -ENOMEM;
    }

    c->ctx = zmq_ctx_new();
    if (c->ctx == NULL) {
        int rc = client_fail(c);
        free(c);
        return rc;
    }

    c->sock = zmq_socket(c->ctx, ZMQ_REQ);
    if (c->sock == NULL) {
        int e = errno;
        zmq_ctx_term(c->ctx);
        free(c);
        return e != 0 ? -e : -EIO;
    }

    int linger = 0;
    (void) zmq_setsockopt(c->sock, ZMQ_LINGER, &linger, sizeof(linger));
    (void) zmq_setsockopt(c->sock, ZMQ_RCVTIMEO, &timeout_ms, sizeof(timeout_ms));
    (void) zmq_setsockopt(c->sock, ZMQ_SNDTIMEO, &timeout_ms, sizeof(timeout_ms));

    if (zmq_connect(c->sock, endpoint) != 0) {
        int e = errno;
        zmq_close(c->sock);
        zmq_ctx_term(c->ctx);
        free(c);
        return e != 0 ? -e : -EIO;
    }

    *out = c;
    return 0;
}

void emu_model_client_close(struct emu_model_client *client)
{
    if (client == NULL) {
        return;
    }
    if (client->sock != NULL) {
        zmq_close(client->sock); /* LINGER=0: never blocks */
    }
    if (client->ctx != NULL) {
        zmq_ctx_term(client->ctx);
    }
    free(client);
}

/* ---- low-level send/recv helpers (REQ side, strict alternation) ---- */

/* Send frame 0 (a JSON command), optionally followed by a binary frame 1. */
static int send_request(struct emu_model_client *c, const char *json,
                        const void *payload, size_t payload_len)
{
    if (c->dead) {
        return -ENODEV;
    }

    size_t jlen = strlen(json);
    int flags = payload != NULL ? ZMQ_SNDMORE : 0;
    if (zmq_send(c->sock, json, jlen, flags) < 0) {
        return client_fail(c);
    }
    if (payload != NULL) {
        if (zmq_send(c->sock, payload, payload_len, 0) < 0) {
            return client_fail(c);
        }
    }
    return 0;
}

/*
 * Receive the single reply frame into a caller buffer.  Returns the reply length
 * on success (>= 0) or a negative errno.  The reply is NUL-terminated in `buf`
 * if it fits (buf_len > received), which the JSON-ish parsers below rely on.
 */
static int recv_reply(struct emu_model_client *c, char *buf, size_t buf_len)
{
    zmq_msg_t msg;
    if (zmq_msg_init(&msg) != 0) {
        return client_fail(c);
    }
    int n = zmq_msg_recv(&msg, c->sock, 0);
    if (n < 0) {
        int rc = client_fail(c);
        zmq_msg_close(&msg);
        return rc;
    }
    size_t got = zmq_msg_size(&msg);
    size_t copy = got < buf_len - 1 ? got : buf_len - 1;
    memcpy(buf, zmq_msg_data(&msg), copy);
    buf[copy] = '\0';
    zmq_msg_close(&msg);
    return (int) got;
}

/* Send a no-payload command and require the literal "OK" reply. */
static int command_ok(struct emu_model_client *c, const char *json)
{
    int rc = send_request(c, json, NULL, 0);
    if (rc != 0) {
        return rc;
    }
    char reply[16];
    int n = recv_reply(c, reply, sizeof(reply));
    if (n < 0) {
        return n;
    }
    if (n == 2 && reply[0] == 'O' && reply[1] == 'K') {
        return 0;
    }
    /* A non-OK reply means the model rejected the command; the socket is still
     * synchronised (we got our one reply), but the bridge treats the model as
     * unusable, so latch dead for fail-fast. */
    c->dead = true;
    return -EPROTO;
}

int emu_model_client_start(struct emu_model_client *client)
{
    if (client == NULL) {
        return -EINVAL;
    }
    return command_ok(client, "{\"command\":\"start\"}");
}

int emu_model_client_exit(struct emu_model_client *client)
{
    if (client == NULL) {
        return -EINVAL;
    }
    return command_ok(client, "{\"command\":\"exit\"}");
}

int emu_model_reg_write(struct emu_model_client *client, uint64_t addr,
                        uint32_t val)
{
    if (client == NULL) {
        return -EINVAL;
    }
    char json[96];
    int n = snprintf(json, sizeof(json),
                     "{\"command\":\"reg\",\"addr\":%llu,\"val\":%lu}",
                     (unsigned long long) addr, (unsigned long) val);
    if (n < 0 || (size_t) n >= sizeof(json)) {
        return -EINVAL;
    }
    return command_ok(client, json);
}

int emu_model_populate(struct emu_model_client *client, uint64_t addr,
                       const void *buf, size_t len)
{
    if (client == NULL || (buf == NULL && len != 0)) {
        return -EINVAL;
    }
    char json[96];
    int n = snprintf(json, sizeof(json),
                     "{\"command\":\"populate\",\"addr\":%llu,\"size\":%llu}",
                     (unsigned long long) addr, (unsigned long long) len);
    if (n < 0 || (size_t) n >= sizeof(json)) {
        return -EINVAL;
    }
    int rc = send_request(client, json, buf, len);
    if (rc != 0) {
        return rc;
    }
    char reply[16];
    int m = recv_reply(client, reply, sizeof(reply));
    if (m < 0) {
        return m;
    }
    if (m == 2 && reply[0] == 'O' && reply[1] == 'K') {
        return 0;
    }
    client->dead = true;
    return -EPROTO;
}

/*
 * Parse a bare unsigned integer out of a reply frame (the scalar-fetch reply is
 * a bare JSON number, possibly with a trailing newline from StreamWriterBuilder).
 * Skips leading whitespace, reads decimal digits.
 */
static int parse_uint_reply(const char *s, uint32_t *out)
{
    while (*s == ' ' || *s == '\t' || *s == '\n' || *s == '\r') {
        s++;
    }
    if (*s < '0' || *s > '9') {
        return -EPROTO;
    }
    uint64_t v = 0;
    while (*s >= '0' && *s <= '9') {
        v = v * 10u + (uint64_t) (*s - '0');
        if (v > 0xFFFFFFFFull) {
            return -EPROTO;
        }
        s++;
    }
    *out = (uint32_t) v;
    return 0;
}

int emu_model_scalar_read(struct emu_model_client *client, uint64_t addr,
                          uint32_t *val)
{
    if (client == NULL || val == NULL) {
        return -EINVAL;
    }
    char json[96];
    int n = snprintf(json, sizeof(json),
                     "{\"command\":\"fetch\",\"type\":\"scalar\",\"addr\":%llu}",
                     (unsigned long long) addr);
    if (n < 0 || (size_t) n >= sizeof(json)) {
        return -EINVAL;
    }
    int rc = send_request(client, json, NULL, 0);
    if (rc != 0) {
        return rc;
    }
    char reply[64];
    int m = recv_reply(client, reply, sizeof(reply));
    if (m < 0) {
        return m;
    }
    rc = parse_uint_reply(reply, val);
    if (rc != 0) {
        client->dead = true;
    }
    return rc;
}

/*
 * Parse a JSON array of byte ints ("[0,12,255]") into dst[0..len).  Requires
 * exactly `len` elements (a short/long array is a protocol error: the model must
 * answer the size we asked for).  The reply may carry a trailing newline.  We
 * receive it into a heap buffer large enough for the worst case ("255," per
 * byte + brackets), so the parse sees the whole frame even for large buffers.
 */
static int parse_byte_array(const char *s, uint8_t *dst, size_t len)
{
    while (*s == ' ' || *s == '\t' || *s == '\n' || *s == '\r') {
        s++;
    }
    if (*s != '[') {
        return -EPROTO;
    }
    s++;
    for (size_t i = 0; i < len; i++) {
        while (*s == ' ' || *s == '\t' || *s == '\n' || *s == '\r' || *s == ',') {
            s++;
        }
        if (*s < '0' || *s > '9') {
            return -EPROTO;
        }
        unsigned v = 0;
        while (*s >= '0' && *s <= '9') {
            v = v * 10u + (unsigned) (*s - '0');
            if (v > 255u) {
                return -EPROTO;
            }
            s++;
        }
        dst[i] = (uint8_t) v;
    }
    /* Trailing must be the closing bracket (after optional whitespace). */
    while (*s == ' ' || *s == '\t' || *s == '\n' || *s == '\r' || *s == ',') {
        s++;
    }
    if (*s != ']') {
        return -EPROTO;
    }
    return 0;
}

int emu_model_fetch(struct emu_model_client *client, uint64_t addr, void *buf,
                    size_t len)
{
    if (client == NULL || (buf == NULL && len != 0)) {
        return -EINVAL;
    }
    if (len == 0) {
        return 0; /* nothing to fetch; do not round-trip */
    }

    char json[96];
    int n = snprintf(
        json, sizeof(json),
        "{\"command\":\"fetch\",\"type\":\"buffer\",\"addr\":%llu,\"size\":%llu}",
        (unsigned long long) addr, (unsigned long long) len);
    if (n < 0 || (size_t) n >= sizeof(json)) {
        return -EINVAL;
    }
    int rc = send_request(client, json, NULL, 0);
    if (rc != 0) {
        return rc;
    }

    /* Worst-case textual size: "[" + len*("255,") + "]" + NL + NUL. */
    size_t cap = 2 + len * 4 + 4;
    _cleanup_(cleanup_free) char *reply = malloc(cap);
    if (reply == NULL) {
        client->dead = true; /* desync risk: we cannot drain the reply safely */
        return -ENOMEM;
    }

    /* Drain the whole reply frame even if it exceeds cap-1 by reading directly. */
    zmq_msg_t msg;
    if (zmq_msg_init(&msg) != 0) {
        return client_fail(client);
    }
    int got = zmq_msg_recv(&msg, client->sock, 0);
    if (got < 0) {
        int frc = client_fail(client);
        zmq_msg_close(&msg);
        return frc;
    }
    size_t copy = (size_t) got < cap - 1 ? (size_t) got : cap - 1;
    memcpy(reply, zmq_msg_data(&msg), copy);
    reply[copy] = '\0';
    zmq_msg_close(&msg);

    rc = parse_byte_array(reply, buf, len);
    if (rc != 0) {
        client->dead = true;
    }
    return rc;
}
