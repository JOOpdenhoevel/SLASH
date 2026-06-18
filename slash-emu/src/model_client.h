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
 * @file model_client.h
 * @brief Daemon-side ZMQ REQ client for the vpp_sim address-keyed dialect (T10).
 *
 * The thin transport + protocol layer the SIM bridge (@ref bridge.h) sits on.
 * It connects a @c ZMQ_REQ socket to the per-device @c ipc:// endpoint the model
 * binds, and exposes the four typed calls the raw data plane needs --
 * @ref emu_model_reg_write, @ref emu_model_scalar_read, @ref emu_model_populate,
 * @ref emu_model_fetch -- plus the @c start handshake and @c exit teardown.  The
 * wire bytes are byte-exact with the real @c vpp_sim (@c sim.cpp) and VRT's
 * @c ZmqServer; see @c docs/bridge-protocol.md for the catalogue.
 *
 * @section nohang No-hang contract
 *
 * Every call is bounded by @c ZMQ_RCVTIMEO / @c ZMQ_SNDTIMEO (set at connect),
 * so a model that never binds or never answers fails a call with a negative
 * errno rather than blocking the single-threaded daemon.  Because @c REQ/REP is
 * strict-alternation, a timeout desynchronises the socket; the client latches a
 * @em dead flag on the first transport error so every later call fails fast with
 * the same negative errno instead of re-using a broken socket.  @c ZMQ_LINGER is
 * 0 so @ref emu_model_client_close never blocks on undelivered frames.
 *
 * @section serial Serialisation
 *
 * The client is @em not internally locked: the bridge serialises all model I/O
 * for a device behind its own per-model mutex (the FUSE session is
 * single-threaded today, but the data model is built for a future MT session).
 */

#ifndef SLASH_EMU_MODEL_CLIENT_H
#define SLASH_EMU_MODEL_CLIENT_H

#include <stddef.h>
#include <stdint.h>

/** @brief Default per-call ZMQ send/recv timeout, milliseconds (no-hang bound). */
#define EMU_MODEL_DEFAULT_TIMEOUT_MS 2000

/** @brief Opaque ZMQ REQ client handle (one per running model). */
struct emu_model_client;

/**
 * @brief Connect a REQ client to a model's @c ipc:// endpoint.
 *
 * Creates a private ZMQ context + @c ZMQ_REQ socket, sets the send/recv timeout
 * and @c ZMQ_LINGER=0, and connects to @p endpoint.  @c zmq_connect on an
 * @c ipc:// endpoint succeeds even before the peer binds; readiness is confirmed
 * separately by @ref emu_model_client_start.  Does @em not block.
 *
 * @param      endpoint   ZMQ endpoint string (e.g. @c "ipc:///run/.../model.sock").
 * @param      timeout_ms Per-call send/recv timeout in ms (use
 *                        @ref EMU_MODEL_DEFAULT_TIMEOUT_MS; must be > 0).
 * @param[out] out        Receives the heap-allocated client on success.
 * @return 0 on success; a negative errno on failure (the client is not created).
 */
int emu_model_client_connect(const char *endpoint, int timeout_ms,
                             struct emu_model_client **out);

/**
 * @brief Close a client: send @c exit (best-effort), then free the socket+ctx.
 *
 * @ref emu_model_client_exit is the explicit graceful teardown; this closer does
 * not itself send @c exit (the bridge sends it before reaping the process).  With
 * @c ZMQ_LINGER=0 the close never blocks.  Safe on NULL.
 *
 * @param client The client to close (may be NULL).
 */
void emu_model_client_close(struct emu_model_client *client);

/**
 * @brief Readiness handshake: send @c start, expect literal @c "OK".
 *
 * The bounded wait here is what turns "model never came up" into a prompt
 * failure instead of a hang: it is the first round-trip after spawn.
 *
 * @param client The client.
 * @return 0 if the model answered @c "OK"; a negative errno on timeout/transport
 *         failure or a non-OK reply (the client is then latched dead).
 */
int emu_model_client_start(struct emu_model_client *client);

/**
 * @brief Graceful teardown: send @c exit, expect literal @c "OK".
 * @param client The client.
 * @return 0 on @c "OK"; a negative errno otherwise (still proceed to close).
 */
int emu_model_client_exit(struct emu_model_client *client);

/**
 * @brief Forward a register write: @c reg{addr,val}.
 * @param client The client.
 * @param addr   Device/AXI address (the BAR-relative byte offset for SIM).
 * @param val    32-bit little-endian register word.
 * @return 0 on @c "OK"; a negative errno on transport failure (latches dead).
 */
int emu_model_reg_write(struct emu_model_client *client, uint64_t addr,
                        uint32_t val);

/**
 * @brief Forward a register read: @c fetch scalar{addr}.
 * @param      client The client.
 * @param      addr   Device/AXI address.
 * @param[out] val    Receives the 32-bit register word.
 * @return 0 on success; a negative errno on transport/parse failure (latches dead).
 */
int emu_model_scalar_read(struct emu_model_client *client, uint64_t addr,
                          uint32_t *val);

/**
 * @brief Forward a memory write: @c populate{addr,size} + binary payload.
 * @param client The client.
 * @param addr   Device address (HBM/DDR).
 * @param buf    Source bytes.
 * @param len    Byte count.
 * @return 0 on @c "OK"; a negative errno on transport failure (latches dead).
 */
int emu_model_populate(struct emu_model_client *client, uint64_t addr,
                       const void *buf, size_t len);

/**
 * @brief Forward a memory read: @c fetch buffer{addr,size}.
 * @param      client The client.
 * @param      addr   Device address (HBM/DDR).
 * @param[out] buf    Destination for @p len bytes.
 * @param      len    Byte count.
 * @return 0 on success (then @p buf is filled); a negative errno on
 *         transport/parse failure or a short reply (latches dead).
 */
int emu_model_fetch(struct emu_model_client *client, uint64_t addr, void *buf,
                    size_t len);

#endif // SLASH_EMU_MODEL_CLIENT_H
