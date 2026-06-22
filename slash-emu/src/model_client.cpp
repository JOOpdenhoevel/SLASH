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
 * @file model_client.cpp
 * @brief Implementation of the daemon-side vpp_sim REQ client (see model_client.hpp).
 *
 * Uses the header-only cppzmq C++ API (<zmq.hpp>) for the transport layer and
 * JsonCpp (<json/json.h>) for building and parsing the SIM-dialect wire
 * protocol.  The wire bytes are byte-exact with sim.cpp / ZmqServer
 * (docs/bridge-protocol.md §2).
 *
 * Error model: data-plane — every method returns 0 or a negative errno; no
 * method throws.  A transport error latches the dead_ flag in the Impl so all
 * later calls fail fast with -ENODEV without touching the broken socket.
 *
 * Helper lambdas (clientFail, sendRequest, recvReply, commandOk) are defined
 * inside each method that needs them, capturing the Impl by reference.  This
 * avoids adding private member declarations to the frozen header while keeping
 * the logic readable.
 */

#include "model_client.hpp"

#include <cerrno>
#include <cstdint>
#include <memory>
#include <string>

#include <json/json.h>
#include <zmq.hpp>

namespace slash::emu {

/* ---- Impl: cppzmq RAII context + socket ---------------------------------- */

/**
 * Defined entirely in the .cpp so the header stays dependency-free from
 * zmq.hpp / json/json.h.  ModelClient member functions access the fields
 * via impl_->field (the unique_ptr is private but the member functions are
 * in the same class).
 */
struct ModelClient::Impl {
    zmq::context_t ctx;
    zmq::socket_t sock;
    bool dead = false;  /**< Latched on the first transport error (see header). */

    Impl() : ctx(1), sock(ctx, zmq::socket_type::req) {}
};

ModelClient::ModelClient() : impl_(std::make_unique<Impl>()) {}

ModelClient::~ModelClient() = default;

/* ---- connect ------------------------------------------------------------- */

int ModelClient::connect(const std::string &endpoint, int timeout_ms,
                         std::unique_ptr<ModelClient> &out)
{
    if (endpoint.empty() || timeout_ms <= 0) {
        return -EINVAL;
    }

    std::unique_ptr<ModelClient> c(new ModelClient());

    try {
        /* ZMQ_LINGER=0: destructor never blocks on undelivered frames. */
        c->impl_->sock.set(zmq::sockopt::linger, 0);
        c->impl_->sock.set(zmq::sockopt::rcvtimeo, timeout_ms);
        c->impl_->sock.set(zmq::sockopt::sndtimeo, timeout_ms);
        c->impl_->sock.connect(endpoint);
    } catch (const zmq::error_t &e) {
        int err = e.num();
        return err != 0 ? -err : -EIO;
    }

    out = std::move(c);
    return 0;
}

/* ---- start --------------------------------------------------------------- */

int ModelClient::start()
{
    Impl &im = *impl_;

    /* Map a cppzmq exception to a negative errno and latch dead.
     * EAGAIN (recv timeout) -> -ETIMEDOUT for a clear transport failure. */
    auto clientFail = [&im](const zmq::error_t &e) -> int {
        im.dead = true;
        int err = e.num();
        return (err == EAGAIN) ? -ETIMEDOUT : (err != 0 ? -err : -EIO);
    };

    /* Send frame 0 (JSON command string) with no binary frame 1. */
    auto sendReq = [&im, &clientFail](const std::string &json) -> int {
        if (im.dead) { return -ENODEV; }
        try {
            im.sock.send(zmq::const_buffer(json.data(), json.size()),
                         zmq::send_flags::none);
        } catch (const zmq::error_t &e) { return clientFail(e); }
        return 0;
    };

    /* Receive the single reply frame. */
    auto recvRep = [&im, &clientFail](int &err) -> std::string {
        err = 0;
        zmq::message_t msg;
        try {
            auto res = im.sock.recv(msg, zmq::recv_flags::none);
            /* cppzmq returns an empty result on EAGAIN; under a blocking recv
             * with RCVTIMEO set that means the per-call timeout elapsed, so map
             * it to -ETIMEDOUT (matching the C client's EAGAIN handling) and
             * latch dead — REQ/REP is now desynchronised. */
            if (!res) { im.dead = true; err = -ETIMEDOUT; return {}; }
        } catch (const zmq::error_t &e) { err = clientFail(e); return {}; }
        return std::string(static_cast<const char *>(msg.data()), msg.size());
    };

    /* {"command":"start"} */
    Json::Value req;
    req["command"] = "start";
    Json::StreamWriterBuilder wb;
    wb["indentation"] = "";

    int rc = sendReq(Json::writeString(wb, req));
    if (rc != 0) { return rc; }

    int err = 0;
    std::string reply = recvRep(err);
    if (err != 0) { return err; }
    if (reply.size() == 2 && reply[0] == 'O' && reply[1] == 'K') { return 0; }
    im.dead = true;
    return -EPROTO;
}

/* ---- sendExit ------------------------------------------------------------ */

int ModelClient::sendExit()
{
    Impl &im = *impl_;

    auto clientFail = [&im](const zmq::error_t &e) -> int {
        im.dead = true;
        int err = e.num();
        return (err == EAGAIN) ? -ETIMEDOUT : (err != 0 ? -err : -EIO);
    };
    auto sendReq = [&im, &clientFail](const std::string &json) -> int {
        if (im.dead) { return -ENODEV; }
        try {
            im.sock.send(zmq::const_buffer(json.data(), json.size()),
                         zmq::send_flags::none);
        } catch (const zmq::error_t &e) { return clientFail(e); }
        return 0;
    };
    auto recvRep = [&im, &clientFail](int &err) -> std::string {
        err = 0;
        zmq::message_t msg;
        try {
            auto res = im.sock.recv(msg, zmq::recv_flags::none);
            /* cppzmq returns an empty result on EAGAIN; under a blocking recv
             * with RCVTIMEO set that means the per-call timeout elapsed, so map
             * it to -ETIMEDOUT (matching the C client's EAGAIN handling) and
             * latch dead — REQ/REP is now desynchronised. */
            if (!res) { im.dead = true; err = -ETIMEDOUT; return {}; }
        } catch (const zmq::error_t &e) { err = clientFail(e); return {}; }
        return std::string(static_cast<const char *>(msg.data()), msg.size());
    };

    /* {"command":"exit"} */
    Json::Value req;
    req["command"] = "exit";
    Json::StreamWriterBuilder wb;
    wb["indentation"] = "";

    int rc = sendReq(Json::writeString(wb, req));
    if (rc != 0) { return rc; }

    int err = 0;
    std::string reply = recvRep(err);
    if (err != 0) { return err; }
    if (reply.size() == 2 && reply[0] == 'O' && reply[1] == 'K') { return 0; }
    im.dead = true;
    return -EPROTO;
}

/* ---- regWrite ------------------------------------------------------------ */

int ModelClient::regWrite(uint64_t addr, uint32_t val)
{
    Impl &im = *impl_;

    auto clientFail = [&im](const zmq::error_t &e) -> int {
        im.dead = true;
        int err = e.num();
        return (err == EAGAIN) ? -ETIMEDOUT : (err != 0 ? -err : -EIO);
    };
    auto sendReq = [&im, &clientFail](const std::string &json) -> int {
        if (im.dead) { return -ENODEV; }
        try {
            im.sock.send(zmq::const_buffer(json.data(), json.size()),
                         zmq::send_flags::none);
        } catch (const zmq::error_t &e) { return clientFail(e); }
        return 0;
    };
    auto recvRep = [&im, &clientFail](int &err) -> std::string {
        err = 0;
        zmq::message_t msg;
        try {
            auto res = im.sock.recv(msg, zmq::recv_flags::none);
            /* cppzmq returns an empty result on EAGAIN; under a blocking recv
             * with RCVTIMEO set that means the per-call timeout elapsed, so map
             * it to -ETIMEDOUT (matching the C client's EAGAIN handling) and
             * latch dead — REQ/REP is now desynchronised. */
            if (!res) { im.dead = true; err = -ETIMEDOUT; return {}; }
        } catch (const zmq::error_t &e) { err = clientFail(e); return {}; }
        return std::string(static_cast<const char *>(msg.data()), msg.size());
    };

    /* {"command":"reg","addr":<u64>,"val":<u32>} */
    Json::Value req;
    req["command"] = "reg";
    req["addr"] = static_cast<Json::UInt64>(addr);
    req["val"] = static_cast<Json::UInt>(val);
    Json::StreamWriterBuilder wb;
    wb["indentation"] = "";

    int rc = sendReq(Json::writeString(wb, req));
    if (rc != 0) { return rc; }

    int err = 0;
    std::string reply = recvRep(err);
    if (err != 0) { return err; }
    if (reply.size() == 2 && reply[0] == 'O' && reply[1] == 'K') { return 0; }
    im.dead = true;
    return -EPROTO;
}

/* ---- scalarRead ---------------------------------------------------------- */

int ModelClient::scalarRead(uint64_t addr, uint32_t &val)
{
    Impl &im = *impl_;
    if (im.dead) { return -ENODEV; }

    auto clientFail = [&im](const zmq::error_t &e) -> int {
        im.dead = true;
        int err = e.num();
        return (err == EAGAIN) ? -ETIMEDOUT : (err != 0 ? -err : -EIO);
    };
    auto sendReq = [&im, &clientFail](const std::string &json) -> int {
        if (im.dead) { return -ENODEV; }
        try {
            im.sock.send(zmq::const_buffer(json.data(), json.size()),
                         zmq::send_flags::none);
        } catch (const zmq::error_t &e) { return clientFail(e); }
        return 0;
    };
    auto recvRep = [&im, &clientFail](int &err) -> std::string {
        err = 0;
        zmq::message_t msg;
        try {
            auto res = im.sock.recv(msg, zmq::recv_flags::none);
            /* cppzmq returns an empty result on EAGAIN; under a blocking recv
             * with RCVTIMEO set that means the per-call timeout elapsed, so map
             * it to -ETIMEDOUT (matching the C client's EAGAIN handling) and
             * latch dead — REQ/REP is now desynchronised. */
            if (!res) { im.dead = true; err = -ETIMEDOUT; return {}; }
        } catch (const zmq::error_t &e) { err = clientFail(e); return {}; }
        return std::string(static_cast<const char *>(msg.data()), msg.size());
    };

    /* {"command":"fetch","type":"scalar","addr":<u64>} */
    Json::Value req;
    req["command"] = "fetch";
    req["type"] = "scalar";
    req["addr"] = static_cast<Json::UInt64>(addr);
    Json::StreamWriterBuilder wb;
    wb["indentation"] = "";

    int rc = sendReq(Json::writeString(wb, req));
    if (rc != 0) { return rc; }

    int err = 0;
    std::string reply = recvRep(err);
    if (err != 0) { return err; }

    /* The scalar-fetch reply is a bare JSON number (possibly with a trailing
     * newline from StreamWriterBuilder).  Parse leniently: skip leading
     * whitespace, read decimal digits. */
    const char *s = reply.c_str();
    while (*s == ' ' || *s == '\t' || *s == '\n' || *s == '\r') { s++; }
    if (*s < '0' || *s > '9') { im.dead = true; return -EPROTO; }
    uint64_t v = 0;
    while (*s >= '0' && *s <= '9') {
        v = v * 10u + static_cast<uint64_t>(*s - '0');
        if (v > 0xFFFFFFFFull) { im.dead = true; return -EPROTO; }
        s++;
    }
    val = static_cast<uint32_t>(v);
    return 0;
}

/* ---- populate ------------------------------------------------------------ */

int ModelClient::populate(uint64_t addr, std::span<const std::byte> data)
{
    if (data.empty()) {
        return 0; /* nothing to send; do not round-trip */
    }

    Impl &im = *impl_;

    auto clientFail = [&im](const zmq::error_t &e) -> int {
        im.dead = true;
        int err = e.num();
        return (err == EAGAIN) ? -ETIMEDOUT : (err != 0 ? -err : -EIO);
    };
    auto recvRep = [&im, &clientFail](int &err) -> std::string {
        err = 0;
        zmq::message_t msg;
        try {
            auto res = im.sock.recv(msg, zmq::recv_flags::none);
            /* cppzmq returns an empty result on EAGAIN; under a blocking recv
             * with RCVTIMEO set that means the per-call timeout elapsed, so map
             * it to -ETIMEDOUT (matching the C client's EAGAIN handling) and
             * latch dead — REQ/REP is now desynchronised. */
            if (!res) { im.dead = true; err = -ETIMEDOUT; return {}; }
        } catch (const zmq::error_t &e) { err = clientFail(e); return {}; }
        return std::string(static_cast<const char *>(msg.data()), msg.size());
    };

    if (im.dead) { return -ENODEV; }

    /* {"command":"populate","addr":<u64>,"size":<u64>} + binary frame 1 */
    Json::Value req;
    req["command"] = "populate";
    req["addr"] = static_cast<Json::UInt64>(addr);
    req["size"] = static_cast<Json::UInt64>(data.size());
    Json::StreamWriterBuilder wb;
    wb["indentation"] = "";
    std::string json = Json::writeString(wb, req);

    try {
        im.sock.send(zmq::const_buffer(json.data(), json.size()),
                     zmq::send_flags::sndmore);
        im.sock.send(zmq::const_buffer(data.data(), data.size()),
                     zmq::send_flags::none);
    } catch (const zmq::error_t &e) { return clientFail(e); }

    int err = 0;
    std::string reply = recvRep(err);
    if (err != 0) { return err; }
    if (reply.size() == 2 && reply[0] == 'O' && reply[1] == 'K') { return 0; }
    im.dead = true;
    return -EPROTO;
}

/* ---- fetch --------------------------------------------------------------- */

int ModelClient::fetch(uint64_t addr, std::span<std::byte> buf)
{
    if (buf.empty()) {
        return 0; /* nothing to fetch; do not round-trip */
    }

    Impl &im = *impl_;
    if (im.dead) { return -ENODEV; }

    auto clientFail = [&im](const zmq::error_t &e) -> int {
        im.dead = true;
        int err = e.num();
        return (err == EAGAIN) ? -ETIMEDOUT : (err != 0 ? -err : -EIO);
    };
    auto sendReq = [&im, &clientFail](const std::string &json) -> int {
        if (im.dead) { return -ENODEV; }
        try {
            im.sock.send(zmq::const_buffer(json.data(), json.size()),
                         zmq::send_flags::none);
        } catch (const zmq::error_t &e) { return clientFail(e); }
        return 0;
    };
    auto recvRep = [&im, &clientFail](int &err) -> std::string {
        err = 0;
        zmq::message_t msg;
        try {
            auto res = im.sock.recv(msg, zmq::recv_flags::none);
            /* cppzmq returns an empty result on EAGAIN; under a blocking recv
             * with RCVTIMEO set that means the per-call timeout elapsed, so map
             * it to -ETIMEDOUT (matching the C client's EAGAIN handling) and
             * latch dead — REQ/REP is now desynchronised. */
            if (!res) { im.dead = true; err = -ETIMEDOUT; return {}; }
        } catch (const zmq::error_t &e) { err = clientFail(e); return {}; }
        return std::string(static_cast<const char *>(msg.data()), msg.size());
    };

    /* {"command":"fetch","type":"buffer","addr":<u64>,"size":<u64>} */
    Json::Value req;
    req["command"] = "fetch";
    req["type"] = "buffer";
    req["addr"] = static_cast<Json::UInt64>(addr);
    req["size"] = static_cast<Json::UInt64>(buf.size());
    Json::StreamWriterBuilder wb;
    wb["indentation"] = "";

    int rc = sendReq(Json::writeString(wb, req));
    if (rc != 0) { return rc; }

    int err = 0;
    std::string reply = recvRep(err);
    if (err != 0) { return err; }

    /*
     * Parse a JSON array of byte ints ("[0,12,255]") into buf.  Requires exactly
     * buf.size() elements (a short/long array is a protocol error: the model must
     * answer the size we asked for).  The reply may carry a trailing newline.
     */
    const char *s = reply.c_str();
    while (*s == ' ' || *s == '\t' || *s == '\n' || *s == '\r') { s++; }
    if (*s != '[') { im.dead = true; return -EPROTO; }
    s++;
    for (size_t i = 0; i < buf.size(); i++) {
        while (*s == ' ' || *s == '\t' || *s == '\n' || *s == '\r' || *s == ',') { s++; }
        if (*s < '0' || *s > '9') { im.dead = true; return -EPROTO; }
        unsigned v = 0;
        while (*s >= '0' && *s <= '9') {
            v = v * 10u + static_cast<unsigned>(*s - '0');
            if (v > 255u) { im.dead = true; return -EPROTO; }
            s++;
        }
        buf[i] = static_cast<std::byte>(v);
    }
    /* Trailing must be the closing bracket (after optional whitespace). */
    while (*s == ' ' || *s == '\t' || *s == '\n' || *s == '\r' || *s == ',') { s++; }
    if (*s != ']') { im.dead = true; return -EPROTO; }
    return 0;
}

} // namespace slash::emu
