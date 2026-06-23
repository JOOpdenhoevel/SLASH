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
 * @file bridge.hpp
 * @brief The SIM data-plane bridge: reconfiguration, model lifecycle, and the
 *        wiring that routes bar/qdma ops to a running @c vpp_sim (C++20).
 *
 * Turns the in-memory endpoints (bars/qdma) into a live SIM data plane.  Per
 * device a @ref Bridge owns: the spawned model process, the @ref ModelClient REQ
 * client, the runtime scratch dir, the @c ipc:// endpoint, and the bar/qdma
 * backend objects it attaches onto the endpoints.  See
 * @c docs/bridge-protocol.md for the wire/lifecycle contract.
 *
 * @section flow Flow
 *
 *   1. @ref BridgeRegistry::attach (at materialize time, per device): allocate the
 *      bridge, register the reconfiguration handler on the device's qdma store, and
 *      install the bridge's shutdown as the device's model-shutdown seam.
 *   2. Reconfiguration: a single @c pwrite of the whole VBIN into the reconfig
 *      region (through a @c qpair<Q>) drives @ref Bridge::reconfigure, which unpacks
 *      the VBIN, forks/execs @c vpp_sim unsandboxed, connects the client, runs the
 *      @c start handshake, and attaches the bar + qdma backends.
 *   3. Data plane: bar pokes and qpair MM transfers route to the model via the
 *      attached backends, with the in-memory shadow/store as the defined-bytes
 *      fallback.
 *   4. Teardown: when both functions are removed the spine fires
 *      @ref Bridge::shutdown (the seam): send @c exit, reap the child, close the
 *      client, detach the backends, remove the scratch.  Idempotent.  Daemon
 *      shutdown also tears down any still-running model.
 *
 * @section scope Scope (step-1 MVP)
 *
 * SIM-only (no @c vpp_emu), MM-only (no streaming), model run @em unsandboxed under
 * the privileged daemon (the hardened systemd transient unit is a later step).
 */

#ifndef SLASH_EMU_BRIDGE_HPP
#define SLASH_EMU_BRIDGE_HPP

#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <vector>

#include "node.hpp"

namespace slash::emu {

/**
 * @brief A per-device SIM bridge.  Created and owned by @ref BridgeRegistry.
 */
class Bridge {
public:
    ~Bridge();

    Bridge(const Bridge &) = delete;
    Bridge &operator=(const Bridge &) = delete;

    /**
     * @brief Reconfiguration handler: a (chunk of a) VBIN write landed in the region.
     *
     * Invoked from the qdma store's reconfig hook (qpair write path) with the tree
     * lock @em held.  Reassembles contiguous chunks (a write at
     * @c SLASH_RECONFIG_BASE starts/replaces; at @c BASE+accumulated appends; a
     * non-contiguous reconfig write resets and is rejected with @c -EINVAL).  When
     * the archive is @c Complete the VBIN is applied: any running model is torn
     * down, the archive is unpacked to a fresh scratch dir, @c vpp_sim is forked/
     * exec'd unsandboxed with the endpoint passed as @c argv[1], the client connects + runs
     * @c start, and the bar+qdma backends are attached.  The accumulation is
     * bounded; an over-cap stream is rejected with @c -EFBIG.
     *
     * @return 0 on success (chunk accepted, whether it completed the VBIN or awaits
     *         more); a negative errno on a malformed VBIN, a non-contiguous chunk,
     *         an over-cap stream, spawn failure, or handshake timeout.
     */
    int reconfigure(uint64_t addr, std::span<const std::byte> vbin);

    /**
     * @brief Tear down the running model (the model-shutdown seam body).
     *
     * Installed as the device's model-shutdown seam, so the spine fires it with the
     * tree lock @em held exactly once when both functions are removed.  Also called
     * from @ref BridgeRegistry teardown at daemon shutdown.  Sends @c exit
     * (best-effort, bounded), reaps the child (SIGTERM then SIGKILL with a bounded
     * wait), closes the client, detaches the bar+qdma backends, and removes the
     * scratch dir.  Idempotent.  The bridge object survives (the registry owns it)
     * so a later reconfiguration could bring a new model up.
     */
    void shutdown();

    /** @brief The device this bridge serves. */
    Device &device() const;

private:
    friend class BridgeRegistry;
    Bridge();
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

/**
 * @brief Owns every per-device @ref Bridge for the daemon's lifetime.
 */
class BridgeRegistry {
public:
    BridgeRegistry();

    /** @brief Tears down every bridge (shutting down any running model) and frees. */
    ~BridgeRegistry();

    BridgeRegistry(const BridgeRegistry &) = delete;
    BridgeRegistry &operator=(const BridgeRegistry &) = delete;

    /**
     * @brief Attach a SIM bridge to a device (called once at materialize time).
     *
     * Allocates the bridge, wires the reconfiguration handler onto the device's
     * qdma store, and installs the bridge's shutdown as the device's model-shutdown
     * seam.  No model is spawned yet (that happens at the first reconfiguration).
     * The device's @c qdma endpoint must already be attached.
     *
     * @param dev          The device to attach to.
     * @param scratch_root Root dir under which per-device runtime scratch (VBIN
     *                     unpack dir + @c ipc:// socket) is created.  Empty uses the
     *                     built-in default.
     * @return The bridge (non-owning view), or nullptr on error.
     */
    Bridge *attach(Device &dev, const std::string &scratch_root = {});

    /**
     * @brief Re-wire a rediscovered function's data plane to the bridge (RESCAN).
     *
     * After @ref NodeTree::restoreFunction rebuilds a removed function's subtree and
     * the endpoints are re-created, this re-establishes the function's path to the
     * device's model/backend state.  Idempotent / safe when no model is running and
     * when no bridge exists for the device.
     *
     * @return 0 on success (including the no-bridge / no-model no-ops); -1 on a bad
     *         argument or a wiring failure.
     */
    int reattachFunction(Device &dev, DeviceFunction func);

    /**
     * @brief Explicitly tear down every bridge while the node tree is still alive.
     *
     * Intended to be called from @c Fs::Impl::~Impl() before the node tree is
     * destroyed, so that backend detach (which touches the device's endpoint nodes)
     * runs before those nodes are freed.  The registry destructor (@c ~BridgeRegistry)
     * calls this too, making it idempotent; calling it earlier is the correctness
     * constraint, not a functional difference.
     */
    void shutdownAll();

private:
    std::vector<std::unique_ptr<Bridge>> bridges_;
};

} // namespace slash::emu

#endif // SLASH_EMU_BRIDGE_HPP
