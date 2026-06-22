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
 * @file fs.hpp
 * @brief libfuse3 low-level FUSE session for the slash-emu daemon (C++20).
 *
 * slash-emu presents the emulated SLASH device tree as a FUSE filesystem.  We use
 * the libfuse3 @em low-level API (not the high-level path-based one) on purpose:
 * hotplug needs @c fuse_lowlevel_notify_delete / @c notify_inval_entry, available
 * only through the low-level interface.
 *
 * @section node_tree Node tree
 *
 * The filesystem is modelled as a @ref NodeTree (see @ref node.hpp).  The FUSE
 * ops in @c fs.cpp are thin adapters: they translate libfuse @c lookup /
 * @c forget / @c getattr / @c readdir / @c read / @c write / @c ioctl requests
 * into calls on the node model and reply with the result.  No FUSE op hard-codes
 * an inode; all topology lives in the node tree.  At construction @ref Fs
 * materializes one @c <BDF>/ directory per configured-and-available accelerator
 * (each with @c bars/ + @c qdma/), attaches the four endpoints, and mounts.
 *
 * @section revocation Revocation
 *
 * On forced removal the node layer asks this layer, through the @ref Notifier it
 * supplied to the tree, to invalidate the affected dentries via
 * @c fuse_lowlevel_notify_delete so a fresh @c lookup misses.
 *
 * @section lifecycle Lifecycle / error model
 *
 * Construction (allocate session, build tree, mount) is control-plane and throws
 * @ref SystemError on failure; @c main catches it.  The session fd is integrated
 * into the caller's sd-event loop via @ref Fs::attach (not @c fuse_session_loop),
 * so FUSE and daemon events share one loop.  The destructor unmounts and releases
 * everything.
 */

#ifndef SLASH_EMU_FS_HPP
#define SLASH_EMU_FS_HPP

#include <memory>
#include <string>

#include <systemd/sd-event.h>

#include "config.hpp"

namespace slash::emu {

class BridgeRegistry;

/**
 * @brief The FUSE low-level session: owns the mount, the node tree, and wiring.
 */
class Fs {
public:
    /**
     * @brief Create a FUSE low-level session and mount it at @p mountpoint.
     *
     * Allocates the session, builds the node tree (root plus a per-device subtree
     * for each configured, available accelerator), attaches the info/bars/qdma
     * endpoints per device and the global hotplug file, wires the SIM bridge per
     * device through @p bridges, and mounts.  Single-threaded; request dispatch is
     * driven by the caller's sd-event loop via @ref attach.
     *
     * @param mountpoint Directory to mount on (must exist).
     * @param config     Daemon configuration (borrowed for the session's life).
     * @param bridges    The daemon's bridge registry (borrowed; owns all bridges).
     * @throws SystemError on any bring-up failure (logged via sd_journal).
     */
    Fs(const std::string &mountpoint, const Config &config,
       BridgeRegistry &bridges);

    ~Fs();

    Fs(const Fs &) = delete;
    Fs &operator=(const Fs &) = delete;

    /**
     * @brief Attach the session's communication fd to an sd-event loop.
     *
     * Registers the FUSE channel fd as an EPOLLIN I/O source on @p ev; on each
     * request the callback receives it via @c fuse_session_receive_buf and
     * dispatches it with @c fuse_session_process_buf.  If the session is torn
     * down, the event loop is asked to exit.
     *
     * @param ev The sd-event loop to attach to.
     * @throws SystemError on failure (logged via sd_journal).
     */
    void attach(sd_event *ev);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace slash::emu

#endif // SLASH_EMU_FS_HPP
