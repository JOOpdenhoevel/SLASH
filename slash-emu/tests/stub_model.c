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
 * @file stub_model.c
 * @brief CI stand-in for vpp_sim: the address-keyed SIM dialect over ipc://.
 *
 * A real synthesized vpp_sim cannot run in CI, so this stub is the single source
 * of truth the daemon's SIM bridge is tested against.  It implements the same
 * wire protocol as sim.cpp (docs/bridge-protocol.md §2) -- byte-exact replies --
 * over a ZMQ_REP socket bound to the endpoint named in SLASH_EMU_ENDPOINT
 * (honouring G5, unlike the real model which hard-codes a fixed tcp port).
 *
 * State: a sparse register map (addr -> u32) and a sparse byte memory map, both
 * answering reads of never-written locations as zero -- so an end-to-end
 * round-trip through the daemon is checkable (write a register / buffer, read it
 * back and get the same bytes).
 *
 * Behaviour is tweakable by env vars so the no-hang / failure paths are testable:
 *   - SLASH_EMU_STUB_NO_BIND=1   : do not bind (model that never comes up).
 *   - SLASH_EMU_STUB_HANG=1      : bind, accept, but never reply (wedge probe).
 *   - SLASH_EMU_STUB_READY_FILE  : touch this path right after bind (spawn proof).
 *   - SLASH_EMU_STUB_CRASH_AFTER_START=1 : answer the start handshake "OK", then
 *                                  SIGSEGV on the NEXT request (model crashes
 *                                  mid-session after connect).
 *   - SLASH_EMU_STUB_EXIT_AFTER_START=1  : answer start "OK", then exit(0) on the
 *                                  next request (model dies between req and reply
 *                                  -> ZMQ desync probe; the client must latch dead
 *                                  rather than reuse a half-consumed REQ socket).
 *   - SLASH_EMU_STUB_GARBAGE=1   : reply with protocol garbage to scalar/buffer
 *                                  fetches (oversized junk, non-numeric, short
 *                                  array) so the client's parsers are fed hostile
 *                                  input -- must be a clean -EPROTO, never UB.
 *   - SLASH_EMU_STUB_IGNORE_EXIT=1 : never honour `exit` (stays alive after the
 *                                  graceful teardown verb) so the bridge's
 *                                  SIGTERM->SIGKILL reap path is exercised.
 *
 * Pure C11 + the C zmq API; no JSON library (the dialect is tiny -- we scan the
 * few integer fields out of the request and emit fixed-shape replies).
 */

#define _GNU_SOURCE

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <zmq.h>

/* ---- tiny field extraction (find "key":<number> in the request frame) ---- */

/* Find the unsigned integer value following the first occurrence of `"key":`. */
static int find_uint(const char *s, const char *key, uint64_t *out)
{
    char pat[32];
    int n = snprintf(pat, sizeof(pat), "\"%s\"", key);
    if (n < 0 || (size_t) n >= sizeof(pat)) {
        return -1;
    }
    const char *p = strstr(s, pat);
    if (p == NULL) {
        return -1;
    }
    p += strlen(pat);
    /* skip ':' and whitespace */
    while (*p == ' ' || *p == ':' || *p == '\t') {
        p++;
    }
    if (*p < '0' || *p > '9') {
        return -1;
    }
    uint64_t v = 0;
    while (*p >= '0' && *p <= '9') {
        v = v * 10u + (uint64_t) (*p - '0');
        p++;
    }
    *out = v;
    return 0;
}

/* Does the request carry "command":"<name>"? */
static int has_command(const char *s, const char *name)
{
    char pat[48];
    int n = snprintf(pat, sizeof(pat), "\"command\":\"%s\"", name);
    if (n < 0 || (size_t) n >= sizeof(pat)) {
        return 0;
    }
    /* tolerate whitespace variants by also checking a spaced form */
    if (strstr(s, pat) != NULL) {
        return 1;
    }
    char pat2[48];
    n = snprintf(pat2, sizeof(pat2), "\"command\": \"%s\"", name);
    if (n > 0 && (size_t) n < sizeof(pat2) && strstr(s, pat2) != NULL) {
        return 1;
    }
    return 0;
}

static int has_type_buffer(const char *s)
{
    return strstr(s, "\"type\":\"buffer\"") != NULL ||
           strstr(s, "\"type\": \"buffer\"") != NULL;
}

/* ---- sparse register + memory maps ---- */

struct reg_ent {
    uint64_t addr;
    uint32_t val;
};

struct mem_ent {
    uint64_t addr;
    uint8_t val;
};

static struct reg_ent *regs;
static size_t regs_len, regs_cap;
static struct mem_ent *mem;
static size_t mem_len, mem_cap;

static void reg_set(uint64_t addr, uint32_t val)
{
    for (size_t i = 0; i < regs_len; i++) {
        if (regs[i].addr == addr) {
            regs[i].val = val;
            return;
        }
    }
    if (regs_len == regs_cap) {
        regs_cap = regs_cap ? regs_cap * 2 : 64;
        regs = realloc(regs, regs_cap * sizeof(*regs));
    }
    regs[regs_len].addr = addr;
    regs[regs_len].val = val;
    regs_len++;
}

static uint32_t reg_get(uint64_t addr)
{
    for (size_t i = 0; i < regs_len; i++) {
        if (regs[i].addr == addr) {
            return regs[i].val;
        }
    }
    return 0;
}

static void mem_set(uint64_t addr, uint8_t val)
{
    for (size_t i = 0; i < mem_len; i++) {
        if (mem[i].addr == addr) {
            mem[i].val = val;
            return;
        }
    }
    if (mem_len == mem_cap) {
        mem_cap = mem_cap ? mem_cap * 2 : 4096;
        mem = realloc(mem, mem_cap * sizeof(*mem));
    }
    mem[mem_len].addr = addr;
    mem[mem_len].val = val;
    mem_len++;
}

static uint8_t mem_get(uint64_t addr)
{
    for (size_t i = 0; i < mem_len; i++) {
        if (mem[i].addr == addr) {
            return mem[i].val;
        }
    }
    return 0;
}

static void send_ok(void *sock)
{
    zmq_send(sock, "OK", 2, 0);
}

int main(void)
{
    const char *endpoint = getenv("SLASH_EMU_ENDPOINT");
    if (endpoint == NULL || endpoint[0] == '\0') {
        fprintf(stderr, "stub_model: SLASH_EMU_ENDPOINT not set\n");
        return 2;
    }

    if (getenv("SLASH_EMU_STUB_NO_BIND") != NULL) {
        /* Model that never comes up: sleep so the daemon's handshake times out,
         * then exit.  The daemon must not wedge. */
        sleep(30);
        return 0;
    }

    void *ctx = zmq_ctx_new();
    void *sock = zmq_socket(ctx, ZMQ_REP);
    if (zmq_bind(sock, endpoint) != 0) {
        fprintf(stderr, "stub_model: bind(%s) failed: %s\n", endpoint,
                zmq_strerror(zmq_errno()));
        return 3;
    }

    const char *ready = getenv("SLASH_EMU_STUB_READY_FILE");
    if (ready != NULL && ready[0] != '\0') {
        FILE *f = fopen(ready, "w");
        if (f != NULL) {
            fputs("ready\n", f);
            fclose(f);
        }
    }

    int hang = getenv("SLASH_EMU_STUB_HANG") != NULL;
    int crash_after_start = getenv("SLASH_EMU_STUB_CRASH_AFTER_START") != NULL;
    int exit_after_start = getenv("SLASH_EMU_STUB_EXIT_AFTER_START") != NULL;
    int garbage = getenv("SLASH_EMU_STUB_GARBAGE") != NULL;
    int ignore_exit = getenv("SLASH_EMU_STUB_IGNORE_EXIT") != NULL;
    int seen_start = 0;

    char reqbuf[1 << 16];
    int running = 1;
    while (running) {
        zmq_msg_t msg;
        zmq_msg_init(&msg);
        int n = zmq_msg_recv(&msg, sock, 0);
        if (n < 0) {
            zmq_msg_close(&msg);
            break;
        }
        size_t got = zmq_msg_size(&msg);
        size_t copy = got < sizeof(reqbuf) - 1 ? got : sizeof(reqbuf) - 1;
        memcpy(reqbuf, zmq_msg_data(&msg), copy);
        reqbuf[copy] = '\0';

        /* Drain a possible binary frame 1 (populate payload) before replying. */
        int more = 0;
        size_t more_sz = sizeof(more);
        zmq_getsockopt(sock, ZMQ_RCVMORE, &more, &more_sz);

        uint64_t addr = 0, size = 0, val = 0;

        /* Hostile lifecycle modes that fire once the start handshake is done: a
         * crash (SIGSEGV) or a clean exit BEFORE answering the current request,
         * modelling a model that dies mid-exchange.  The daemon's REQ client must
         * not wedge: the in-flight recv times out and latches the client dead. */
        if (seen_start && (crash_after_start || exit_after_start)) {
            /* Drain a possible frame-1 payload so we don't trip ZMQ asserts. */
            int more2 = 0;
            size_t ms2 = sizeof(more2);
            zmq_getsockopt(sock, ZMQ_RCVMORE, &more2, &ms2);
            if (more2) {
                zmq_msg_t p;
                zmq_msg_init(&p);
                (void) zmq_msg_recv(&p, sock, 0);
                zmq_msg_close(&p);
            }
            zmq_msg_close(&msg);
            if (crash_after_start) {
                /* Model crashes after connect.  Use abort() (SIGABRT) rather than
                 * a NULL deref so the crash is a genuine fatal signal even under
                 * ASan/UBSan (which would otherwise intercept a SEGV and exit
                 * with a sanitizer status instead of dying by signal). */
                abort();
            }
            _exit(0); /* exit_after_start: die without replying */
        }

        if (has_command(reqbuf, "populate")) {
            (void) find_uint(reqbuf, "addr", &addr);
            (void) find_uint(reqbuf, "size", &size);
            zmq_msg_t payload;
            zmq_msg_init(&payload);
            if (more) {
                (void) zmq_msg_recv(&payload, sock, 0);
                const uint8_t *pb = zmq_msg_data(&payload);
                size_t pl = zmq_msg_size(&payload);
                for (size_t i = 0; i < size && i < pl; i++) {
                    mem_set(addr + i, pb[i]);
                }
            }
            zmq_msg_close(&payload);
            zmq_msg_close(&msg);
            if (!hang) {
                send_ok(sock);
            }
            continue;
        }
        zmq_msg_close(&msg);

        if (hang) {
            /* Never reply: probe that the daemon's recv timeout fires. */
            continue;
        }

        if (has_command(reqbuf, "start")) {
            seen_start = 1;
            send_ok(sock);
        } else if (has_command(reqbuf, "exit")) {
            if (ignore_exit) {
                /* Do not honour exit: stay alive (still answer so no desync) so
                 * the bridge must fall back to SIGTERM/SIGKILL to reap us. */
                send_ok(sock);
            } else {
                send_ok(sock);
                running = 0;
            }
        } else if (has_command(reqbuf, "reg")) {
            (void) find_uint(reqbuf, "addr", &addr);
            (void) find_uint(reqbuf, "val", &val);
            reg_set(addr, (uint32_t) val);
            send_ok(sock);
        } else if (has_command(reqbuf, "fetch")) {
            if (garbage) {
                /* Protocol garbage: a long non-numeric, non-array blob.  The
                 * client's parsers must reject it cleanly (-EPROTO) and latch
                 * dead, never read out of bounds or misparse into UB. */
                char junk[300];
                memset(junk, 'Z', sizeof(junk));
                zmq_send(sock, junk, sizeof(junk), 0);
            } else if (has_type_buffer(reqbuf)) {
                (void) find_uint(reqbuf, "addr", &addr);
                (void) find_uint(reqbuf, "size", &size);
                /* JSON array of byte ints. */
                char *out = malloc(size * 4 + 4);
                size_t o = 0;
                out[o++] = '[';
                for (uint64_t i = 0; i < size; i++) {
                    if (i != 0) {
                        out[o++] = ',';
                    }
                    o += (size_t) sprintf(out + o, "%u", mem_get(addr + i));
                }
                out[o++] = ']';
                zmq_send(sock, out, o, 0);
                free(out);
            } else {
                /* scalar */
                (void) find_uint(reqbuf, "addr", &addr);
                char out[32];
                int m = snprintf(out, sizeof(out), "%u", reg_get(addr));
                zmq_send(sock, out, (size_t) m, 0);
            }
        } else {
            /* Unknown command: reply something non-OK so the client surfaces it. */
            zmq_send(sock, "ERR", 3, 0);
        }
    }

    zmq_close(sock);
    zmq_ctx_term(ctx);
    free(regs);
    free(mem);
    return 0;
}
