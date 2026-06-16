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
 * @file fs.h
 * @brief libfuse3 low-level FUSE session for the slash-emu daemon.
 *
 * slash-emu presents the emulated SLASH device tree as a FUSE filesystem.  We
 * use the libfuse3 @em low-level API (not the high-level path-based one) on
 * purpose: later tasks need @c fuse_lowlevel_notify_delete /
 * @c fuse_lowlevel_notify_inval_entry to push hotplug add/remove events to the
 * kernel's dentry cache, and those notifications are only available through the
 * low-level interface.
 *
 * @section node_tree Node tree (scaffold)
 *
 * The filesystem is modelled as a tree of @c struct @c emu_node objects, each
 * identified by a FUSE inode number.  In this scaffold the tree contains only
 * the root directory (inode @c FUSE_ROOT_ID == 1): @c lookup, @c getattr, and
 * @c readdir are implemented for the root and report an empty directory.
 *
 * Later tasks (endpoints: info/bars/qdma/hotplug) extend this by adding child
 * nodes: allocate a new inode, attach it under the root (or a sub-directory),
 * and have @c lookup / @c readdir enumerate it.  The op handlers in @c fs.c are
 * written to dispatch on inode number so that adding nodes does not require
 * touching the session plumbing.
 */

#ifndef SLASH_EMU_FS_H
#define SLASH_EMU_FS_H

#include <systemd/sd-event.h>

#include "config.h"

/** @brief Opaque handle bundling the FUSE session, its mountpoint, and state. */
struct emu_fs;

/**
 * @brief Create a FUSE low-level session and mount it at @p mountpoint.
 *
 * Allocates the session, builds the (root-only) node tree, and mounts it.  The
 * session is created in single-threaded mode; request dispatch is driven by the
 * caller's sd-event loop via @c emu_fs_attach (the session fd is integrated as
 * an I/O event source rather than run through @c fuse_session_loop).
 *
 * @param[out] fsp       On success, receives a heap-allocated @c emu_fs.  The
 *                       caller owns it and must release it with @c cleanup_fs.
 * @param      mountpoint Directory to mount the filesystem on (must exist).
 * @param      config    Daemon configuration (non-owning; borrowed for the
 *                       lifetime of the session).  May be NULL in the scaffold.
 * @return 0 on success, -1 on error (logged via sd_journal).
 */
int emu_fs_create(struct emu_fs **fsp, const char *mountpoint,
                  const struct emu_config *config);

/**
 * @brief Attach the session's communication fd to an sd-event loop.
 *
 * Registers the FUSE channel fd as an EPOLLIN I/O source on @p ev.  When the
 * kernel posts a request, the callback receives it via
 * @c fuse_session_receive_buf and dispatches it with
 * @c fuse_session_process_buf.  If the session is torn down (unmount, or the
 * kernel side closing the channel), the event loop is asked to exit.
 *
 * @param fs The session to attach (must be mounted).
 * @param ev The sd-event loop to attach to.
 * @return 0 on success, -1 on error (logged via sd_journal).
 */
int emu_fs_attach(struct emu_fs *fs, sd_event *ev);

/**
 * @brief Unmount and destroy a FUSE session, releasing all resources.
 * @param fs The session to clean up (may be NULL).
 */
void cleanup_fs(struct emu_fs *fs);

/**
 * @brief Cleanup helper for use with @c __attribute__((cleanup)).
 * @param fsp Address of a @c struct @c emu_fs pointer.
 */
static inline
void cleanup_fsp(struct emu_fs **fsp)
{
    if (fsp == NULL) {
        return;
    }

    cleanup_fs(*fsp);

    *fsp = NULL;
}

#endif // SLASH_EMU_FS_H
