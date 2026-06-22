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
 * @file model_client.hpp
 * @brief Daemon-side ZMQ REQ client for the vpp_sim address-keyed dialect (C++20).
 *
 * The thin transport + protocol layer the SIM bridge (@ref bridge.hpp) sits on.
 * It connects a @c ZMQ_REQ socket (via the header-only cppzmq API) to the
 * per-device @c ipc:// endpoint the model binds, and builds/parses the dialect
 * with @b JsonCpp.  The wire bytes are byte-exact with the real @c vpp_sim
 * (@c sim.cpp) and VRT's @c ZmqServer; see @c docs/bridge-protocol.md.
 *
 * @section nohang No-hang contract
 *
 * Every call is bounded by @c ZMQ_RCVTIMEO / @c ZMQ_SNDTIMEO (set at connect), so
 * a model that never binds/answers fails a call with a negative errno rather than
 * blocking the single-threaded daemon.  Because @c REQ/REP is strict-alternation,
 * a timeout desynchronises the socket; the client latches a @em dead flag on the
 * first transport error so every later call fails fast.  @c ZMQ_LINGER is 0 so the
 * destructor never blocks.
 *
 * @section errors Error model
 *
 * Data-plane: every method returns @b 0 on success or a @b negative @b errno (the
 * bridge's bar/qdma seams branch on the rc).  No method throws.
 *
 * @section serial Serialisation
 *
 * Not internally locked: the bridge serialises all model I/O for a device behind
 * its own per-model mutex.
 */

#ifndef SLASH_EMU_MODEL_CLIENT_HPP
#define SLASH_EMU_MODEL_CLIENT_HPP

#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <string>

namespace slash::emu {

/** @brief Default per-call ZMQ send/recv timeout, milliseconds (no-hang bound). */
inline constexpr int kModelDefaultTimeoutMs = 2000;

/**
 * @brief A ZMQ REQ client to one running model's @c ipc:// endpoint.
 *
 * Construct via @ref connect; owns its cppzmq context + socket (RAII).
 */
class ModelClient {
public:
    ~ModelClient();

    ModelClient(const ModelClient &) = delete;
    ModelClient &operator=(const ModelClient &) = delete;

    /**
     * @brief Connect a REQ client to a model's @c ipc:// endpoint.
     *
     * Creates a private context + REQ socket, sets the send/recv timeout and
     * @c ZMQ_LINGER=0, and connects to @p endpoint (which succeeds even before the
     * peer binds; readiness is confirmed by @ref start).  Does not block.
     *
     * @param      endpoint   ZMQ endpoint (e.g. @c "ipc:///run/.../model.sock").
     * @param      timeout_ms Per-call send/recv timeout in ms (> 0).
     * @param[out] out        Receives the client on success.
     * @return 0 on success; a negative errno on failure (no client is created).
     */
    static int connect(const std::string &endpoint, int timeout_ms,
                       std::unique_ptr<ModelClient> &out);

    /**
     * @brief Readiness handshake: send @c start, expect literal @c "OK".
     * @return 0 if the model answered @c "OK"; a negative errno on timeout /
     *         transport failure / non-OK reply (the client is then latched dead).
     */
    int start();

    /**
     * @brief Graceful teardown: send @c exit, expect literal @c "OK".
     * @return 0 on @c "OK"; a negative errno otherwise (still proceed to destroy).
     */
    int sendExit();

    /**
     * @brief Forward a register write: @c reg{addr,val}.
     * @return 0 on @c "OK"; a negative errno on transport failure (latches dead).
     */
    int regWrite(uint64_t addr, uint32_t val);

    /**
     * @brief Forward a register read: @c fetch scalar{addr}.
     * @param[out] val Receives the 32-bit register word.
     * @return 0 on success; a negative errno on transport/parse failure (latches
     *         dead).
     */
    int scalarRead(uint64_t addr, uint32_t &val);

    /**
     * @brief Forward a memory write: @c populate{addr,size} + binary payload.
     * @return 0 on @c "OK"; a negative errno on transport failure (latches dead).
     */
    int populate(uint64_t addr, std::span<const std::byte> data);

    /**
     * @brief Forward a memory read: @c fetch buffer{addr,size}.
     * @param buf Destination span; @c buf.size() bytes are requested.
     * @return 0 on success (then @p buf is filled); a negative errno on
     *         transport/parse failure or a short reply (latches dead).
     */
    int fetch(uint64_t addr, std::span<std::byte> buf);

private:
    ModelClient();
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace slash::emu

#endif // SLASH_EMU_MODEL_CLIENT_HPP
