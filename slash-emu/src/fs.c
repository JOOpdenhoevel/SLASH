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
 * @file fs.c
 * @brief libfuse3 low-level session implementation for slash-emu.
 *
 * See fs.h for the design rationale (why low-level, and how the node tree is
 * meant to be extended).  This scaffold serves a single, empty root directory.
 */

#define _GNU_SOURCE

#include "fs.h"

#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <sys/epoll.h>
#include <sys/stat.h>
#include <syslog.h>

#define FUSE_USE_VERSION 314
#include <fuse_lowlevel.h>

#include <systemd/sd-event.h>

#include "config.h"
#include "utils.h"

/*
 * Attribute/entry cache timeout handed to the kernel, in seconds.  The
 * emulated tree only changes in response to hotplug events, which later tasks
 * push explicitly via fuse_lowlevel_notify_*; a modest positive timeout keeps
 * stat/readdir cheap without making invalidation correctness depend on it.
 */
#define EMU_FS_ATTR_TIMEOUT_S 1.0

/**
 * @brief A mounted FUSE low-level session and its associated daemon state.
 *
 * Ownership / lifecycle flags let cleanup_fs() run safely after a partial
 * failure in emu_fs_create(): each resource is only released if its flag says
 * it was actually acquired.
 */
struct emu_fs {
    /** @brief The libfuse session object (owning). */
    struct fuse_session *session; /* owning */

    /** @brief Mountpoint path (heap-allocated, owning). */
    char *mountpoint; /* owning */

    /** @brief Daemon configuration (non-owning; borrowed). */
    const struct emu_config *config; /* non-owning */

    /** @brief sd-event source watching the session fd (owning ref). */
    sd_event_source *source; /* owning */

    /** @brief Reusable receive buffer; grown/managed by libfuse, mem freed by us. */
    struct fuse_buf recv_buf;

    /** @brief True once fuse_session_mount() succeeded (needs unmount on teardown). */
    bool mounted;
};

/*
 * Fill a struct stat for the given inode.  The scaffold knows only the root
 * directory; later tasks extend this to dispatch on the node tree.
 */
static int emu_fs_stat(fuse_ino_t ino, struct stat *st)
{
    *st = (struct stat) {
        .st_ino = ino,
    };

    if (ino == FUSE_ROOT_ID) {
        st->st_mode = S_IFDIR | 0555;
        st->st_nlink = 2;
        return 0;
    }

    /* No other inodes exist yet. */
    return -1;
}

static void emu_op_getattr(fuse_req_t req, fuse_ino_t ino,
                           struct fuse_file_info *fi)
{
    (void) fi;

    struct stat st;
    if (emu_fs_stat(ino, &st) == -1) {
        (void) fuse_reply_err(req, ENOENT);
        return;
    }

    (void) fuse_reply_attr(req, &st, EMU_FS_ATTR_TIMEOUT_S);
}

static void emu_op_lookup(fuse_req_t req, fuse_ino_t parent, const char *name)
{
    /*
     * The root is currently empty, so every lookup below it fails with
     * ENOENT.  Later tasks resolve @name against the parent's children here.
     */
    if (parent != FUSE_ROOT_ID) {
        (void) fuse_reply_err(req, ENOTDIR);
        return;
    }

    (void) name;
    (void) fuse_reply_err(req, ENOENT);
}

/*
 * Append one directory entry to @buf if it fits.  Mirrors the canonical
 * libfuse low-level readdir pattern: size the entry, only emit it when it fits
 * within the kernel-requested window, and always advance the running offset so
 * the next readdir call resumes correctly.
 */
static size_t emu_dirbuf_add(fuse_req_t req, char *buf, size_t bufsize,
                             size_t used, const char *name, fuse_ino_t ino,
                             off_t next_off)
{
    struct stat st = {
        .st_ino = ino,
        .st_mode = S_IFDIR,
    };

    size_t entsize = fuse_add_direntry(req, NULL, 0, name, NULL, 0);
    if (used + entsize > bufsize) {
        return used;
    }

    fuse_add_direntry(req, buf + used, bufsize - used, name, &st, next_off);
    return used + entsize;
}

static void emu_op_readdir(fuse_req_t req, fuse_ino_t ino, size_t size,
                           off_t off, struct fuse_file_info *fi)
{
    (void) fi;

    if (ino != FUSE_ROOT_ID) {
        (void) fuse_reply_err(req, ENOTDIR);
        return;
    }

    _cleanup_(cleanup_free)
    char *buf = calloc(1, size);
    if (buf == NULL && size != 0) {
        (void) fuse_reply_err(req, ENOMEM);
        return;
    }

    /*
     * The root has only "." and ".." for now.  Entry offsets are 1-based and
     * monotonically increasing; the kernel passes the last-seen offset back in
     * @off so we can resume.  Everything fits in a single reply here, but we
     * keep the offset bookkeeping so adding children later is trivial.
     */
    size_t used = 0;
    off_t entry_off = 0;

    if (off <= entry_off) {
        used = emu_dirbuf_add(req, buf, size, used, ".", FUSE_ROOT_ID,
                              entry_off + 1);
    }
    entry_off++;

    if (off <= entry_off) {
        used = emu_dirbuf_add(req, buf, size, used, "..", FUSE_ROOT_ID,
                              entry_off + 1);
    }
    entry_off++;

    (void) fuse_reply_buf(req, buf, used);
}

static const struct fuse_lowlevel_ops emu_fs_ops = {
    .lookup = emu_op_lookup,
    .getattr = emu_op_getattr,
    .readdir = emu_op_readdir,
};

/*
 * sd-event callback: the kernel has a request waiting on the FUSE channel fd.
 * Receive it and hand it to libfuse for dispatch.  If the session has exited
 * (clean unmount or kernel-side close), tear down the event loop.
 */
static int on_fuse_readable(sd_event_source *s, int fd, uint32_t revents,
                            void *userdata)
{
    (void) fd;
    (void) revents;

    struct emu_fs *fs = userdata;

    if (fuse_session_exited(fs->session)) {
        sd_event_exit(sd_event_source_get_event(s), 0);
        return 0;
    }

    int ret = fuse_session_receive_buf(fs->session, &fs->recv_buf);
    if (ret == -EINTR) {
        return 0;
    }
    if (ret <= 0) {
        /*
         * 0 means the kernel closed the channel (unmounted from outside);
         * negative is a real error.  Either way, stop the loop so the daemon
         * shuts down cleanly.
         */
        if (ret < 0) {
            LOG(LOG_ERR, "fuse_session_receive_buf failed: %s",
                strerror(-ret));
        }
        sd_event_exit(sd_event_source_get_event(s), 0);
        return 0;
    }

    fuse_session_process_buf(fs->session, &fs->recv_buf);

    if (fuse_session_exited(fs->session)) {
        sd_event_exit(sd_event_source_get_event(s), 0);
    }

    return 0;
}

int emu_fs_create(struct emu_fs **fsp, const char *mountpoint,
                  const struct emu_config *config)
{
    _cleanup_(cleanup_fsp)
    struct emu_fs *fs = calloc(1, sizeof(*fs));
    PROPAGATE_ERROR_NULL_LOG(fs, LOG_ERR, "Failed to allocate FUSE state");

    *fs = (struct emu_fs) {
        .config = config,
        .recv_buf = {0},
        .mounted = false,
    };

    fs->mountpoint = strdup(mountpoint);
    PROPAGATE_ERROR_NULL_LOG(fs->mountpoint, LOG_ERR,
                             "Failed to duplicate mountpoint");

    /*
     * fuse_session_new() requires a non-empty argv[0]; we pass only the
     * program name and let mount options come from elsewhere (the systemd unit
     * / kernel defaults).  FUSE_ARGS_INIT keeps libfuse's argv conventions.
     */
    char *argv[] = { (char *) "slash-emud", NULL };
    struct fuse_args args = FUSE_ARGS_INIT(1, argv);

    fs->session = fuse_session_new(&args, &emu_fs_ops, sizeof(emu_fs_ops), fs);
    if (fs->session == NULL) {
        LOG(LOG_ERR, "Failed to create FUSE session");
        return -1;
    }

    int ret = fuse_session_mount(fs->session, fs->mountpoint);
    if (ret != 0) {
        LOG(LOG_ERR, "Failed to mount FUSE session at %s", fs->mountpoint);
        return -1;
    }
    fs->mounted = true;

    LOG(LOG_INFO, "Mounted slash-emu filesystem at %s", fs->mountpoint);

    *fsp = fs;
    fs = NULL;

    return 0;
}

int emu_fs_attach(struct emu_fs *fs, sd_event *ev)
{
    int fd = fuse_session_fd(fs->session);
    if (fd < 0) {
        LOG(LOG_ERR, "FUSE session has no valid fd");
        return -1;
    }

    int ret = sd_event_add_io(ev, &fs->source, fd, EPOLLIN, on_fuse_readable,
                              fs);
    PROPAGATE_ERROR_SD_LOG(ret, LOG_ERR, "Failed to watch FUSE channel fd");

    ret = sd_event_source_set_description(fs->source, "FUSE channel");
    PROPAGATE_ERROR_SD_LOG(ret, LOG_ERR,
                           "Failed to set FUSE source description");

    return 0;
}

void cleanup_fs(struct emu_fs *fs)
{
    if (fs == NULL) {
        return;
    }

    /* Drop the event source first so no further callbacks fire mid-teardown. */
    if (fs->source != NULL) {
        (void) sd_event_source_set_enabled(fs->source, SD_EVENT_OFF);
        sd_event_source_unref(fs->source);
        fs->source = NULL;
    }

    if (fs->session != NULL) {
        if (fs->mounted) {
            fuse_session_unmount(fs->session);
            fs->mounted = false;
        }
        fuse_session_destroy(fs->session);
        fs->session = NULL;
    }

    /* libfuse allocates recv_buf.mem lazily inside fuse_session_receive_buf. */
    free(fs->recv_buf.mem);
    fs->recv_buf.mem = NULL;

    free(fs->mountpoint);
    free(fs);
}
