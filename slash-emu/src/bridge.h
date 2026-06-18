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
 * @file bridge.h
 * @brief The SIM data-plane bridge (T10): reconfiguration, model lifecycle, and
 *        the wiring that routes bar/qdma ops to a running @c vpp_sim.
 *
 * This is the module that turns the in-memory endpoints (bars/qdma) into a live
 * SIM data plane.  It owns, per device, a @ref emu_bridge: the spawned model
 * process, the @ref model_client.h REQ client, the runtime scratch dir, the
 * @c ipc:// endpoint, and the bar/qdma backend vtables it attaches onto the
 * endpoints.  See @c docs/bridge-protocol.md for the wire/lifecycle contract.
 *
 * @section flow Flow
 *
 *   1. @ref emu_bridge_attach (at materialize time, per device): allocate the
 *      bridge, register the @em reconfiguration handler on the device's qdma
 *      store (so a write into the reconfig region routes here), and replace the
 *      T9 model-shutdown seam with @ref emu_bridge_shutdown.
 *   2. Reconfiguration: a single @c pwrite of the whole VBIN into the reconfig
 *      region (through a @c qpair<Q> file) calls @ref emu_bridge_reconfigure,
 *      which unpacks the VBIN, forks/execs @c vpp_sim unsandboxed with
 *      @c SLASH_EMU_ENDPOINT set, connects the client, runs the @c start
 *      handshake, and attaches the bar + qdma backends.
 *   3. Data plane: bar register pokes and qpair MM transfers now route to the
 *      model via the attached backends (the @c reg / @c fetch scalar /
 *      @c populate / @c fetch buffer dialect), with the in-memory shadow/store
 *      as the defined-bytes fallback (G7).
 *   4. Teardown: when both PCI functions are removed the spine fires
 *      @ref emu_bridge_shutdown (the seam), which sends @c exit, reaps the
 *      child, closes the client, detaches the backends, and removes the scratch.
 *      Idempotent.  Daemon shutdown also calls it for any still-running model.
 *
 * @section scope Scope (step-1 MVP)
 *
 * SIM-only (no @c vpp_emu), MM-only (no streaming), model run @em unsandboxed
 * directly under the privileged daemon (the hardened systemd transient unit is
 * step 5, out of scope).  The bridge runs the model from a configurable scratch
 * root (@ref emu_bridge_set_scratch_root); tests point it under the repo @c .tmp.
 */

#ifndef SLASH_EMU_BRIDGE_H
#define SLASH_EMU_BRIDGE_H

#include <stddef.h>
#include <stdint.h>

#include "node.h"

/** @brief Opaque per-device SIM bridge handle. */
struct emu_bridge;

/**
 * @brief Attach a SIM bridge to a device (called once at materialize time).
 *
 * Allocates the bridge, wires the reconfiguration handler onto the device's qdma
 * store (so a reconfig-region @c pwrite routes to @ref emu_bridge_reconfigure),
 * and installs @ref emu_bridge_shutdown as the device's model-shutdown seam
 * (replacing the T9 logging default).  No model is spawned yet -- that happens at
 * the first reconfiguration.  The device's @c qdma endpoint must already be
 * attached (@ref emu_qdma_attach).
 *
 * The bridge is owned by the daemon's bridge registry (@p reg); it is torn down
 * either by the model-shutdown seam (both functions removed) or by
 * @ref emu_bridge_registry_free at daemon shutdown.
 *
 * @param reg          The daemon's bridge registry (owns all bridges).
 * @param dev          The device to attach to.
 * @param scratch_root Root dir under which per-device runtime scratch is created
 *                     (the VBIN unpack dir + the @c ipc:// socket live here).
 *                     Borrowed; copied.  NULL uses the built-in default.
 * @return 0 on success; -1 on error (allocation / no qdma endpoint).
 */
struct emu_bridge_registry;
int emu_bridge_attach(struct emu_bridge_registry *reg, struct emu_device *dev,
                      const char *scratch_root);

/**
 * @brief Reconfiguration handler: a (chunk of a) VBIN write landed in the region.
 *
 * Invoked from the qdma store's reconfig hook (qpair write path) with the tree
 * lock @em held.  Because the kernel splits a write larger than its @c max_write
 * into several @c ops->write calls, a multi-MB VBIN arrives as several in-region
 * chunks; this entry point @em reassembles them:
 *
 *   - a write at @c SLASH_RECONFIG_BASE (re)starts the accumulation buffer
 *     (a fresh VBIN -- any prior partial transfer is discarded);
 *   - a write at @c SLASH_RECONFIG_BASE + accumulated_len appends;
 *   - any other (non-contiguous / seeking) reconfig-region write resets the
 *     partial transfer and is rejected with @c -EINVAL.
 *
 * After each chunk the accumulated archive is classified: while @c INCOMPLETE the
 * chunk is accepted (return 0) and more bytes are awaited; a structurally
 * @c INVALID stream is rejected with @c -EINVAL up front; when @c COMPLETE (the
 * ustar terminator is reached) the VBIN is applied -- any running model is torn
 * down, the archive is unpacked to a fresh scratch dir, @c vpp_sim is forked/exec
 * unsandboxed with @c SLASH_EMU_ENDPOINT pointing at the per-device @c ipc://
 * socket, the REQ client connects + runs the @c start handshake, and the bar+qdma
 * backends are attached.  The accumulation is bounded by an internal cap; an
 * over-cap stream is rejected with @c -EFBIG (and reset).
 *
 * Bounded: the readiness handshake uses the client's per-call timeout, so a VBIN
 * whose model never binds fails reconfiguration with a negative errno rather than
 * hanging the daemon.
 *
 * @param bridge The device's bridge (the reconfig hook's ctx).
 * @param addr   The chunk's device address (the qpair write offset).
 * @param vbin   The chunk bytes.
 * @param len    Chunk length in bytes.
 * @return 0 on success (chunk accepted, whether it completed the VBIN -- model up,
 *         backends attached -- or awaits more); a negative errno on a malformed
 *         VBIN, a non-contiguous chunk, an over-cap stream, spawn failure, or
 *         handshake timeout.
 */
int emu_bridge_reconfigure(struct emu_bridge *bridge, uint64_t addr,
                           const void *vbin, size_t len);

/**
 * @brief Tear down a device's running model (the model-shutdown seam body).
 *
 * Installed as the device's @ref emu_model_shutdown_fn, so the spine fires it
 * with the tree lock @em held exactly once when both functions are removed.  Also
 * called directly from @ref emu_bridge_registry_free at daemon shutdown.  Sends
 * @c exit (best-effort, bounded), reaps the child (@c SIGTERM then @c SIGKILL
 * with a bounded wait), closes the client, detaches the bar+qdma backends, and
 * removes the scratch dir.  Idempotent: a second call with no running model is a
 * no-op.  The bridge struct itself survives (freed by the registry) so a later
 * reconfiguration could bring a new model up.
 *
 * @param dev The device whose model to shut down (matches @ref emu_model_shutdown_fn).
 * @param ctx The bridge (the seam's ctx).
 */
void emu_bridge_shutdown(struct emu_device *dev, void *ctx);

/* ------------------------------------------------------------------ */
/* Registry: owns every per-device bridge for the daemon's lifetime    */
/* ------------------------------------------------------------------ */

/**
 * @brief Create an empty bridge registry.
 * @param[out] out Receives the heap-allocated registry (caller owns it).
 * @return 0 on success; -1 on allocation failure.
 */
int emu_bridge_registry_new(struct emu_bridge_registry **out);

/**
 * @brief Tear down every bridge (shutting down any running model) and free.
 *
 * Calls @ref emu_bridge_shutdown on each bridge so no child process / socket /
 * scratch leaks across daemon shutdown, then frees the registry.  Safe on NULL.
 *
 * @param reg The registry to free (may be NULL).
 */
void emu_bridge_registry_free(struct emu_bridge_registry *reg);

/** @brief Cleanup helper for @c __attribute__((cleanup)). */
static inline void cleanup_bridge_registryp(struct emu_bridge_registry **regp)
{
    if (regp == NULL) {
        return;
    }
    emu_bridge_registry_free(*regp);
    *regp = NULL;
}

#endif // SLASH_EMU_BRIDGE_H
