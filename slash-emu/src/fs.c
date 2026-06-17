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
 * structured).  At startup the session materializes a per-device subtree
 * (/<BDF>/ + bars/ + qdma/) for each configured, available accelerator.
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

#include "bars.h"
#include "config.h"
#include "info.h"
#include "node.h"
#include "qdma.h"
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

    /** @brief The emulated device node tree (owning). */
    struct emu_node_tree *tree; /* owning */

    /** @brief True once fuse_session_mount() succeeded (needs unmount on teardown). */
    bool mounted;
};

/* Retrieve the daemon FS state bound to a request. */
static struct emu_fs *fs_of(fuse_req_t req)
{
    return fuse_req_userdata(req);
}

/*
 * Build a fuse_entry_param for a freshly-resolved node.  The node layer has
 * already bumped the kernel lookup count; the entry's nlookup contract (+1 per
 * reply) is balanced by emu_op_forget.
 */
static void fill_entry(struct emu_node_tree *tree, fuse_ino_t ino,
                       struct fuse_entry_param *e)
{
    *e = (struct fuse_entry_param) {
        .ino = ino,
        .attr_timeout = EMU_FS_ATTR_TIMEOUT_S,
        .entry_timeout = EMU_FS_ATTR_TIMEOUT_S,
    };
    (void) emu_node_stat(tree, ino, &e->attr);
}

static void emu_op_getattr(fuse_req_t req, fuse_ino_t ino,
                           struct fuse_file_info *fi)
{
    (void) fi;

    struct stat st;
    int ret = emu_node_stat(fs_of(req)->tree, ino, &st);
    if (ret != 0) {
        (void) fuse_reply_err(req, -ret);
        return;
    }

    (void) fuse_reply_attr(req, &st, EMU_FS_ATTR_TIMEOUT_S);
}

static void emu_op_lookup(fuse_req_t req, fuse_ino_t parent, const char *name)
{
    struct emu_node_tree *tree = fs_of(req)->tree;

    struct emu_node *child = NULL;
    int ret = emu_node_lookup_child(tree, parent, name, &child);
    if (ret != 0) {
        /* -ESTALE on a vanished parent maps to ENOENT for the lookup contract. */
        (void) fuse_reply_err(req, ret == -ESTALE ? ENOENT : -ret);
        return;
    }

    struct fuse_entry_param e;
    fill_entry(tree, child->ino, &e);
    (void) fuse_reply_entry(req, &e);
}

static void emu_op_forget(fuse_req_t req, fuse_ino_t ino, uint64_t nlookup)
{
    emu_node_forget(fs_of(req)->tree, ino, nlookup);
    fuse_reply_none(req);
}

/*
 * Append one directory entry to @buf if it fits.  Mirrors the canonical
 * libfuse low-level readdir pattern: size the entry, only emit it when it fits
 * within the kernel-requested window, and always advance the running offset so
 * the next readdir call resumes correctly.
 */
static size_t emu_dirbuf_add(fuse_req_t req, char *buf, size_t bufsize,
                             size_t used, const char *name, fuse_ino_t ino,
                             mode_t mode, off_t next_off)
{
    struct stat st = {
        .st_ino = ino,
        .st_mode = mode,
    };

    size_t entsize = fuse_add_direntry(req, NULL, 0, name, NULL, 0);
    if (used + entsize > bufsize) {
        return used;
    }

    fuse_add_direntry(req, buf + used, bufsize - used, name, &st, next_off);
    return used + entsize;
}

/*
 * Accumulator threaded through emu_node_readdir: the kernel-supplied window
 * (buf/size), the cursor the kernel handed us (off), the running offset, and how
 * much we have filled.  Each live entry advances entry_off; entries at or before
 * the cursor are skipped so a resumed readdir does not repeat them.
 */
struct readdir_ctx {
    fuse_req_t req;
    char *buf;
    size_t size;
    size_t used;
    off_t off;       /* kernel's resume cursor */
    off_t entry_off; /* running 1-based offset */
};

static bool readdir_emit(void *vctx, const char *name, emu_ino_t ino,
                         enum emu_node_type type)
{
    struct readdir_ctx *ctx = vctx;

    mode_t mode = type == EMU_NODE_DIR ? S_IFDIR : S_IFREG;

    if (ctx->off <= ctx->entry_off) {
        ctx->used = emu_dirbuf_add(ctx->req, ctx->buf, ctx->size, ctx->used,
                                   name, ino, mode, ctx->entry_off + 1);
    }
    ctx->entry_off++;

    return true;
}

static void emu_op_readdir(fuse_req_t req, fuse_ino_t ino, size_t size,
                           off_t off, struct fuse_file_info *fi)
{
    (void) fi;

    _cleanup_(cleanup_free)
    char *buf = calloc(1, size);
    if (buf == NULL && size != 0) {
        (void) fuse_reply_err(req, ENOMEM);
        return;
    }

    struct readdir_ctx ctx = {
        .req = req,
        .buf = buf,
        .size = size,
        .used = 0,
        .off = off,
        .entry_off = 0,
    };

    int ret = emu_node_readdir(fs_of(req)->tree, ino, readdir_emit, &ctx);
    if (ret != 0) {
        (void) fuse_reply_err(req, -ret);
        return;
    }

    (void) fuse_reply_buf(req, buf, ctx.used);
}

/*
 * Serve a positioned read by dispatching to the target node's ops->read hook
 * via emu_node_pread (which enforces the liveness gate: a revoked endpoint
 * returns -ENODEV on an already-open fd).  The node layer implements pread(2)
 * semantics -- a short read near EOF, and a zero-length reply at/after EOF --
 * so we pass libfuse exactly the bytes it produced.  Endpoint files (info,
 * bar<M>, qpair<Q>) supply the read hook; a file without one yields -EIO.
 */
static void emu_op_read(fuse_req_t req, fuse_ino_t ino, size_t size, off_t off,
                        struct fuse_file_info *fi)
{
    (void) fi;

    _cleanup_(cleanup_free)
    char *buf = malloc(size != 0 ? size : 1);
    if (buf == NULL) {
        (void) fuse_reply_err(req, ENOMEM);
        return;
    }

    ssize_t n = emu_node_pread(fs_of(req)->tree, ino, buf, size, off);
    if (n < 0) {
        (void) fuse_reply_err(req, (int) -n);
        return;
    }

    (void) fuse_reply_buf(req, buf, (size_t) n);
}

/*
 * Serve a positioned write by dispatching to the target node's ops->write hook
 * via emu_node_pwrite (the write counterpart of emu_op_read, with the same
 * liveness gate: a revoked endpoint returns -ENODEV on an already-open fd).
 * The node layer enforces the endpoint's range/width policy -- for bar<M> a
 * register poke is width == transfer size, {1,2,4,8}, aligned, in-range, else
 * -EINVAL -- and reports the byte count consumed.  A file without a write hook
 * (e.g. the read-only info file) yields -EIO.
 */
static void emu_op_write(fuse_req_t req, fuse_ino_t ino, const char *buf,
                         size_t size, off_t off, struct fuse_file_info *fi)
{
    (void) fi;

    ssize_t n = emu_node_pwrite(fs_of(req)->tree, ino, buf, size, off);
    if (n < 0) {
        (void) fuse_reply_err(req, (int) -n);
        return;
    }

    (void) fuse_reply_write(req, (size_t) n);
}

/*
 * Honour a node's direct-I/O request at open time.  Register endpoints
 * (bar<M>, later qpair<Q>) require unbuffered access -- the architecture
 * mandates "no buffering, width == transfer size" -- so the kernel must pass
 * each read/write through with the caller's exact size and offset rather than
 * servicing page-sized, page-cached transfers that the width validation would
 * reject.  Files that do not ask for it (e.g. info) open with the default
 * cached path.  We never keep per-fd state, so no fh is set.
 *
 * direct_io also delivers the "no mmap" guarantee, in two parts:
 *   - MAP_SHARED is rejected at mmap() time by the kernel (it cannot provide
 *     shared-mapping coherency for a FOPEN_DIRECT_IO file): -ENODEV.  There is
 *     thus no coherent register window to keep in sync or zap on revocation.
 *   - MAP_PRIVATE cannot be vetoed at mmap() time from a low-level FUSE daemon
 *     (the kernel routes it through generic_file_mmap; libfuse exposes no .mmap
 *     hook, and FOPEN_NONSEEKABLE/STREAM would break pread/pwrite).  The daemon
 *     instead guarantees it never HANGS: the first page-fault issues a
 *     page-sized FUSE_READ, which emu_op_read ALWAYS answers (here, with the
 *     BAR width-validation's -EINVAL), so the fault resolves to a prompt SIGBUS
 *     rather than blocking.  The "answer every read" invariant in emu_op_read
 *     is what makes this hold; see BarsMount.MmapPrivateDerefFaultsPromptly...
 */
static void emu_op_open(fuse_req_t req, fuse_ino_t ino,
                        struct fuse_file_info *fi)
{
    if (emu_node_wants_direct_io(fs_of(req)->tree, ino)) {
        fi->direct_io = 1;
    }

    (void) fuse_reply_open(req, fi);
}

/*
 * Serve an ioctl by dispatching to the target node's ops->ioctl hook via
 * emu_node_ioctl (the ioctl sibling of emu_op_read/emu_op_write, with the same
 * liveness gate: a revoked endpoint returns -ENODEV).  The qdma/ directory's
 * QPAIR_ADD (T8) and, later, the hotplug file's device commands (T9) ride this.
 *
 * FUSE low-level ioctl protocol, and how we handle its quirks for a fixed-size
 * _IOWR struct:
 *
 *   - Restricted (the default): we do NOT set FUSE_IOCTL_UNRESTRICTED, so the
 *     kernel parses the _IOC encoding of `cmd` itself and bounce-buffers the
 *     payload for us.  `in_buf`/`in_bufsz` carry the caller's argument bytes
 *     (for _IOC_WRITE/_IOWR); `out_bufsz` is the room the kernel reserved for
 *     the reply (for _IOC_READ/_IOWR).  Because the size is encoded in `cmd`
 *     and matched to the caller's struct, a single round-trip suffices: there
 *     is no FUSE_IOCTL_RETRY dance (that is only needed when the daemon must
 *     itself describe the in/out iovecs for an unrestricted ioctl, which we
 *     never do, so the kernel never asks us to retry inbound).
 *
 *   - Buffers: the kernel already copied the caller's struct in and will copy
 *     our reply struct out, so the hook works purely on these daemon-side
 *     bounce buffers and never touches the caller's address space.  For the
 *     _IOWR QPAIR_ADD the same fixed-size region is both read (inputs) and
 *     written (the allocated qid), so we hand the hook `in` for inputs and a
 *     separate zeroed reply buffer for outputs, then reply with the reply
 *     buffer clamped to `out_bufsz` (a shorter-struct caller gets the prefix it
 *     asked for -- the same one-directional size-versioning the ABI uses).
 *
 *   - Compat (32-bit caller on a 64-bit kernel): the structs here are fixed
 *     __u32 layouts with identical 32/64-bit representations, but we still
 *     reject FUSE_IOCTL_COMPAT to avoid silently mishandling any future struct
 *     with pointer/long members.
 */
static void emu_op_ioctl(fuse_req_t req, fuse_ino_t ino, unsigned int cmd,
                         void *arg, struct fuse_file_info *fi,
                         unsigned int flags, const void *in_buf,
                         size_t in_bufsz, size_t out_bufsz)
{
    (void) arg;
    (void) fi;

    if (flags & FUSE_IOCTL_COMPAT) {
        (void) fuse_reply_err(req, ENOSYS);
        return;
    }

    /*
     * Reply buffer for the out direction.  Cap it so a malformed/huge out_bufsz
     * cannot make us allocate unboundedly; the real commands are small fixed
     * structs.  We seed it from the in buffer so an _IOWR hook that only fills
     * specific out fields leaves the rest as the caller supplied (then the hook
     * overwrites what it owns).
     */
    enum { EMU_IOCTL_MAX = 4096 };
    if (out_bufsz > EMU_IOCTL_MAX || in_bufsz > EMU_IOCTL_MAX) {
        (void) fuse_reply_err(req, EINVAL);
        return;
    }

    _cleanup_(cleanup_free)
    char *out = out_bufsz != 0 ? calloc(1, out_bufsz) : NULL;
    if (out_bufsz != 0 && out == NULL) {
        (void) fuse_reply_err(req, ENOMEM);
        return;
    }
    if (out != NULL) {
        size_t seed = in_bufsz < out_bufsz ? in_bufsz : out_bufsz;
        memcpy(out, in_buf, seed);
    }

    int ret = emu_node_ioctl(fs_of(req)->tree, ino, cmd, in_buf, in_bufsz, out,
                             out_bufsz);
    if (ret != 0) {
        (void) fuse_reply_err(req, -ret);
        return;
    }

    (void) fuse_reply_ioctl(req, 0, out, out_bufsz);
}

/*
 * Serve an unlink by resolving (parent, name) and unlinking the file node via
 * emu_node_unlink_child (delete-on-last-close).  This is the VRTD pattern's
 * removal step -- unlink the qpair<Q> file while its fd is open so it becomes a
 * nameless orphan that lives until the holder closes it -- and the path by which
 * VRTD prunes leftover named qpairs on startup.  Only files are unlinkable; the
 * endpoint directories are removed by revocation, not user unlink.
 */
static void emu_op_unlink(fuse_req_t req, fuse_ino_t parent, const char *name)
{
    int ret = emu_node_unlink_child(fs_of(req)->tree, parent, name);
    (void) fuse_reply_err(req, ret == 0 ? 0 : -ret);
}

static const struct fuse_lowlevel_ops emu_fs_ops = {
    .lookup = emu_op_lookup,
    .forget = emu_op_forget,
    .unlink = emu_op_unlink,
    .getattr = emu_op_getattr,
    .readdir = emu_op_readdir,
    .open = emu_op_open,
    .read = emu_op_read,
    .write = emu_op_write,
    .ioctl = emu_op_ioctl,
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

/*
 * Notifier callback wired into the node tree: when an endpoint is revoked, the
 * node layer asks us to invalidate the kernel's dentry for it so a subsequent
 * lookup misses (returns ENOENT).  fuse_lowlevel_notify_delete forces the kernel
 * to drop the cached entry; -ENOSYS (old kernel) is harmless (entry_timeout then
 * bounds staleness).  Called with the tree lock NOT held.
 */
static void emu_fs_notify_delete(void *ctx, emu_ino_t parent, emu_ino_t child,
                                 const char *name)
{
    struct emu_fs *fs = ctx;
    if (fs->session == NULL) {
        return;
    }

    int ret = fuse_lowlevel_notify_delete(fs->session, parent, child, name,
                                          strlen(name));
    if (ret != 0 && ret != -ENOSYS) {
        LOG(LOG_WARNING, "notify_delete(%s) failed: %s", name, strerror(-ret));
    }
}

/*
 * Materialize the per-device subtree for every configured accelerator that is
 * not already running.  Reuses the config running-set computation so RESCAN (T9)
 * drives the same path.  The endpoint files (info/bar<M>/qpair<Q>) are attached
 * by later tasks; here we only stand up the <BDF>/, bars/, and qdma/ dirs.
 */
static int emu_fs_materialize(struct emu_fs *fs)
{
    if (fs->config == NULL) {
        return 0;
    }

    _cleanup_(cleanup_running_setp)
    struct emu_running_set *running = NULL;
    if (emu_running_set_new(&running) == -1) {
        return -1;
    }

    struct emu_accelerator_ref_array selected = emu_accelerator_ref_array_init();
    int ret = emu_config_select_new(fs->config, running, &selected);
    if (ret == -1) {
        emu_accelerator_ref_array_free(&selected);
        return -1;
    }

    for (size_t i = 0; i < selected.len; i++) {
        const struct emu_accelerator *acc = selected.d[i];
        struct emu_device *dev = NULL;
        if (emu_node_tree_add_device(fs->tree, acc->bdf, &dev) == -1) {
            LOG(LOG_ERR, "Failed to materialize accelerator '%s'", acc->bdf);
            emu_accelerator_ref_array_free(&selected);
            return -1;
        }

        /*
         * Attach the per-device endpoints under the freshly-built subtree.  The
         * info file is first; bars/qdma/hotplug (T7-T9) attach here too, each
         * via its own emu_<endpoint>_attach following the same pattern.
         */
        if (emu_info_attach(dev) == -1) {
            LOG(LOG_ERR, "Failed to attach info endpoint for '%s'", acc->bdf);
            emu_accelerator_ref_array_free(&selected);
            return -1;
        }

        if (emu_bars_attach(dev) == -1) {
            LOG(LOG_ERR, "Failed to attach bars endpoint for '%s'", acc->bdf);
            emu_accelerator_ref_array_free(&selected);
            return -1;
        }

        if (emu_qdma_attach(dev) == -1) {
            LOG(LOG_ERR, "Failed to attach qdma endpoint for '%s'", acc->bdf);
            emu_accelerator_ref_array_free(&selected);
            return -1;
        }

        LOG(LOG_INFO, "Materialized accelerator '%s'", acc->bdf);
    }

    emu_accelerator_ref_array_free(&selected);

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

    struct emu_notifier notifier = {
        .notify_delete = emu_fs_notify_delete,
        .ctx = fs,
    };
    if (emu_node_tree_new(&fs->tree, &notifier) == -1) {
        LOG(LOG_ERR, "Failed to create node tree");
        return -1;
    }

    if (emu_fs_materialize(fs) == -1) {
        LOG(LOG_ERR, "Failed to materialize device tree");
        return -1;
    }

    /*
     * fuse_session_new() requires a non-empty argv[0]; we pass only the
     * program name and let mount options come from elsewhere (the systemd unit
     * / kernel defaults).  FUSE_ARGS_INIT keeps libfuse's argv conventions.
     */
    char *argv[] = { (char *) "slash-emud", NULL };
    struct fuse_args args = FUSE_ARGS_INIT(1, argv);

    fs->session = fuse_session_new(&args, &emu_fs_ops, sizeof(emu_fs_ops), fs);
    /*
     * fuse_session_new parses @args into its own copy, reallocating @args's argv
     * in place; that internal copy is never reclaimed unless we free @args here.
     * fuse_opt_free_args is the documented owner-frees-its-args contract and
     * fixes the 27-byte ASan leak the T3 audit flagged (a real missing free in
     * our setup, not a libfuse one-time alloc).
     */
    fuse_opt_free_args(&args);
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

    /*
     * Free the node tree only after the session is gone: the notifier callback
     * captures fs->session, and no op can run against the tree once the session
     * is destroyed.
     */
    cleanup_node_tree(fs->tree);
    fs->tree = NULL;

    /* libfuse allocates recv_buf.mem lazily inside fuse_session_receive_buf. */
    free(fs->recv_buf.mem);
    fs->recv_buf.mem = NULL;

    free(fs->mountpoint);
    free(fs);
}
