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
 * @file fs.cpp
 * @brief libfuse3 low-level session implementation for slash-emu (C++20).
 *
 * See fs.hpp for the design rationale (why low-level, and how the node tree is
 * structured).  At startup the session materializes a per-device subtree
 * (/<BDF>/ + bars/ + qdma/) for each configured, available accelerator, attaches
 * the four endpoints + the global hotplug file, wires the SIM bridge per device,
 * and mounts.  This is a faithful port of the C @c fs.c.
 *
 * Error model: construction / mount / attach are control-plane and throw
 * @ref SystemError; the FUSE op thunks use the negative-errno convention
 * (@c fuse_reply_err).
 */

#include "fs.hpp"

#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <sys/epoll.h>
#include <sys/stat.h>

#define FUSE_USE_VERSION 314
#include <fuse_lowlevel.h>

#include "bars.hpp"
#include "bridge.hpp"
#include "config.hpp"
#include "hotplug.hpp"
#include "info.hpp"
#include "node.hpp"
#include "qdma.hpp"
#include "utils.hpp"

namespace slash::emu {

namespace {

/*
 * Attribute/entry cache timeout handed to the kernel, in seconds.  The emulated
 * tree only changes in response to hotplug events, which are pushed explicitly
 * via fuse_lowlevel_notify_*; a modest positive timeout keeps stat/readdir cheap
 * without making invalidation correctness depend on it.
 */
constexpr double kAttrTimeoutS = 1.0;

/* Cap so a malformed/huge ioctl size cannot make us allocate unboundedly; the
 * real commands are small fixed structs. */
constexpr size_t kIoctlMax = 4096;

} // namespace

/**
 * @brief A mounted FUSE low-level session and its associated daemon state.
 *
 * RAII: the destructor drops the event source, unmounts + destroys the session,
 * tears down the bridges (before the tree, which the bridge teardown consults),
 * and releases the receive buffer.  The node tree and the reloaded config are
 * plain members, destroyed in reverse declaration order after the session is
 * gone (the notifier captures the session, so no op can run against the tree once
 * the session is destroyed).
 */
struct Fs::Impl {
    Impl(const std::string &mountpoint, const Config &config,
         BridgeRegistry &bridges);
    ~Impl();

    Impl(const Impl &) = delete;
    Impl &operator=(const Impl &) = delete;

    /* ---- FUSE op cores (member functions; thunks recover `this`) ---- */
    void opLookup(fuse_req_t req, fuse_ino_t parent, const char *name);
    void opForget(fuse_req_t req, fuse_ino_t ino, uint64_t nlookup);
    void opForgetMulti(fuse_req_t req, size_t count,
                       struct fuse_forget_data *forgets);
    void opGetattr(fuse_req_t req, fuse_ino_t ino, struct fuse_file_info *fi);
    void opReaddir(fuse_req_t req, fuse_ino_t ino, size_t size, off_t off,
                   struct fuse_file_info *fi);
    void opOpen(fuse_req_t req, fuse_ino_t ino, struct fuse_file_info *fi);
    void opRead(fuse_req_t req, fuse_ino_t ino, size_t size, off_t off,
                struct fuse_file_info *fi);
    void opWrite(fuse_req_t req, fuse_ino_t ino, const char *buf, size_t size,
                 off_t off, struct fuse_file_info *fi);
    void opIoctl(fuse_req_t req, fuse_ino_t ino, unsigned int cmd, void *arg,
                 struct fuse_file_info *fi, unsigned int flags,
                 const void *in_buf, size_t in_bufsz, size_t out_bufsz);
    void opUnlink(fuse_req_t req, fuse_ino_t parent, const char *name);

    /* ---- bring-up / reload ---- */
    int materialize();
    int rediscover();
    int reload();
    void onFuseReadable(sd_event_source *s);

    void fillEntry(fuse_ino_t ino, struct fuse_entry_param *e);

    /* ---- static thunks: the libfuse callbacks recover `this` via
     *      fuse_req_userdata and forward to the op cores above ---- */
    static Impl *implOf(fuse_req_t req);
    static void thunkLookup(fuse_req_t req, fuse_ino_t parent, const char *name);
    static void thunkForget(fuse_req_t req, fuse_ino_t ino, uint64_t nlookup);
    static void thunkForgetMulti(fuse_req_t req, size_t count,
                                 struct fuse_forget_data *forgets);
    static void thunkGetattr(fuse_req_t req, fuse_ino_t ino,
                             struct fuse_file_info *fi);
    static void thunkReaddir(fuse_req_t req, fuse_ino_t ino, size_t size,
                             off_t off, struct fuse_file_info *fi);
    static void thunkOpen(fuse_req_t req, fuse_ino_t ino,
                          struct fuse_file_info *fi);
    static void thunkRead(fuse_req_t req, fuse_ino_t ino, size_t size, off_t off,
                          struct fuse_file_info *fi);
    static void thunkWrite(fuse_req_t req, fuse_ino_t ino, const char *buf,
                           size_t size, off_t off, struct fuse_file_info *fi);
    static void thunkIoctl(fuse_req_t req, fuse_ino_t ino, unsigned int cmd,
                           void *arg, struct fuse_file_info *fi,
                           unsigned int flags, const void *in_buf,
                           size_t in_bufsz, size_t out_bufsz);
    static void thunkUnlink(fuse_req_t req, fuse_ino_t parent, const char *name);
    static int onFuseReadableThunk(sd_event_source *s, int fd, uint32_t revents,
                                   void *userdata);
    static const struct fuse_lowlevel_ops &ops();

    fuse_session *session = nullptr;     /**< owning */
    std::string mountpoint;
    const Config *config = nullptr;      /**< borrowed (see owned_config) */
    std::optional<Config> owned_config;  /**< config reloaded from disk */
    sd_event_source *source = nullptr;   /**< owning ref */
    fuse_buf recv_buf{};                 /**< grown/managed by libfuse */
    BridgeRegistry &bridges;             /**< borrowed (owns all bridges) */
    std::string scratch_root;            /**< empty => bridge default */
    bool mounted = false;

    /* The tree must be declared AFTER the session so it is destroyed FIRST? No:
     * the C teardown order is session -> bridges -> tree.  We replicate that
     * order explicitly in ~Impl rather than relying on member order, then leave
     * the tree to its own destruction last. */
    NodeTree tree;
};

/* ================================================================== */
/* Static thunks: recover Impl* via fuse_req_userdata.                 */
/* ================================================================== */

Fs::Impl *Fs::Impl::implOf(fuse_req_t req)
{
    return static_cast<Fs::Impl *>(fuse_req_userdata(req));
}

void Fs::Impl::thunkLookup(fuse_req_t req, fuse_ino_t parent, const char *name)
{
    implOf(req)->opLookup(req, parent, name);
}

void Fs::Impl::thunkForget(fuse_req_t req, fuse_ino_t ino, uint64_t nlookup)
{
    implOf(req)->opForget(req, ino, nlookup);
}

void Fs::Impl::thunkForgetMulti(fuse_req_t req, size_t count,
                                struct fuse_forget_data *forgets)
{
    implOf(req)->opForgetMulti(req, count, forgets);
}

void Fs::Impl::thunkGetattr(fuse_req_t req, fuse_ino_t ino,
                            struct fuse_file_info *fi)
{
    implOf(req)->opGetattr(req, ino, fi);
}

void Fs::Impl::thunkReaddir(fuse_req_t req, fuse_ino_t ino, size_t size,
                            off_t off, struct fuse_file_info *fi)
{
    implOf(req)->opReaddir(req, ino, size, off, fi);
}

void Fs::Impl::thunkOpen(fuse_req_t req, fuse_ino_t ino,
                         struct fuse_file_info *fi)
{
    implOf(req)->opOpen(req, ino, fi);
}

void Fs::Impl::thunkRead(fuse_req_t req, fuse_ino_t ino, size_t size, off_t off,
                         struct fuse_file_info *fi)
{
    implOf(req)->opRead(req, ino, size, off, fi);
}

void Fs::Impl::thunkWrite(fuse_req_t req, fuse_ino_t ino, const char *buf,
                          size_t size, off_t off, struct fuse_file_info *fi)
{
    implOf(req)->opWrite(req, ino, buf, size, off, fi);
}

void Fs::Impl::thunkIoctl(fuse_req_t req, fuse_ino_t ino, unsigned int cmd,
                          void *arg, struct fuse_file_info *fi,
                          unsigned int flags, const void *in_buf,
                          size_t in_bufsz, size_t out_bufsz)
{
    implOf(req)->opIoctl(req, ino, cmd, arg, fi, flags, in_buf, in_bufsz,
                         out_bufsz);
}

void Fs::Impl::thunkUnlink(fuse_req_t req, fuse_ino_t parent, const char *name)
{
    implOf(req)->opUnlink(req, parent, name);
}

const struct fuse_lowlevel_ops &Fs::Impl::ops()
{
    static const struct fuse_lowlevel_ops kOps = [] {
        struct fuse_lowlevel_ops o{};
        o.lookup = thunkLookup;
        o.forget = thunkForget;
        o.forget_multi = thunkForgetMulti;
        o.unlink = thunkUnlink;
        o.getattr = thunkGetattr;
        o.readdir = thunkReaddir;
        o.open = thunkOpen;
        o.read = thunkRead;
        o.write = thunkWrite;
        o.ioctl = thunkIoctl;
        return o;
    }();
    return kOps;
}

int Fs::Impl::onFuseReadableThunk(sd_event_source *s, int fd, uint32_t revents,
                                  void *userdata)
{
    (void) fd;
    (void) revents;
    static_cast<Fs::Impl *>(userdata)->onFuseReadable(s);
    return 0;
}

/* ================================================================== */
/* FUSE op cores                                                      */
/* ================================================================== */

/*
 * Build a fuse_entry_param for a freshly-resolved node.  The node layer has
 * already bumped the kernel lookup count; the entry's nlookup contract (+1 per
 * reply) is balanced by opForget.
 */
void Fs::Impl::fillEntry(fuse_ino_t ino, struct fuse_entry_param *e)
{
    *e = (struct fuse_entry_param){};
    e->ino = ino;
    e->attr_timeout = kAttrTimeoutS;
    e->entry_timeout = kAttrTimeoutS;
    (void) tree.stat(ino, &e->attr);
}

void Fs::Impl::opGetattr(fuse_req_t req, fuse_ino_t ino,
                         struct fuse_file_info *fi)
{
    (void) fi;

    struct stat st;
    int ret = tree.stat(ino, &st);
    if (ret != 0) {
        (void) fuse_reply_err(req, -ret);
        return;
    }

    (void) fuse_reply_attr(req, &st, kAttrTimeoutS);
}

void Fs::Impl::opLookup(fuse_req_t req, fuse_ino_t parent, const char *name)
{
    Node *child = nullptr;
    int ret = tree.lookupChild(parent, name, &child);
    if (ret != 0) {
        /* -ESTALE on a vanished parent maps to ENOENT for the lookup contract. */
        (void) fuse_reply_err(req, ret == -ESTALE ? ENOENT : -ret);
        return;
    }

    struct fuse_entry_param e;
    fillEntry(child->ino, &e);
    (void) fuse_reply_entry(req, &e);
}

void Fs::Impl::opForget(fuse_req_t req, fuse_ino_t ino, uint64_t nlookup)
{
    tree.forget(ino, nlookup);
    fuse_reply_none(req);
}

void Fs::Impl::opForgetMulti(fuse_req_t req, size_t count,
                             struct fuse_forget_data *forgets)
{
    for (size_t i = 0; i < count; i++) {
        tree.forget(forgets[i].ino, forgets[i].nlookup);
    }
    fuse_reply_none(req);
}

/*
 * Accumulator threaded through tree.readdir: the kernel-supplied window
 * (buf/size), the cursor the kernel handed us (off), the running offset, and how
 * much we have filled.  Each live entry advances entry_off; entries at or before
 * the cursor are skipped so a resumed readdir does not repeat them.
 */
void Fs::Impl::opReaddir(fuse_req_t req, fuse_ino_t ino, size_t size, off_t off,
                         struct fuse_file_info *fi)
{
    (void) fi;

    std::vector<char> buf(size);

    size_t used = 0;
    off_t entry_off = 0;

    int ret = tree.readdir(
        ino, [&](const std::string &name, Ino child_ino, NodeType type) {
            mode_t mode = type == NodeType::Dir ? S_IFDIR : S_IFREG;

            if (off <= entry_off) {
                struct stat st{};
                st.st_ino = child_ino;
                st.st_mode = mode;

                size_t entsize =
                    fuse_add_direntry(req, nullptr, 0, name.c_str(), nullptr, 0);
                if (used + entsize <= size) {
                    fuse_add_direntry(req, buf.data() + used, size - used,
                                      name.c_str(), &st, entry_off + 1);
                    used += entsize;
                }
            }
            entry_off++;
            return true;
        });
    if (ret != 0) {
        (void) fuse_reply_err(req, -ret);
        return;
    }

    (void) fuse_reply_buf(req, buf.data(), used);
}

/*
 * Honour a node's direct-I/O request at open time.  Register endpoints (bar<M>,
 * qpair<Q>) require unbuffered access so the kernel passes each read/write
 * through with the caller's exact size and offset.  We never keep per-fd state,
 * so no fh is set.
 */
void Fs::Impl::opOpen(fuse_req_t req, fuse_ino_t ino, struct fuse_file_info *fi)
{
    if (tree.wantsDirectIo(ino)) {
        fi->direct_io = 1;
    }

    (void) fuse_reply_open(req, fi);
}

/*
 * Serve a positioned read via tree.pread (which enforces the liveness gate: a
 * revoked endpoint returns -ENODEV on an already-open fd).  The node layer
 * implements pread(2) semantics -- a short read near EOF, a zero-length reply
 * at/after EOF -- so we pass libfuse exactly the bytes it produced.
 */
void Fs::Impl::opRead(fuse_req_t req, fuse_ino_t ino, size_t size, off_t off,
                      struct fuse_file_info *fi)
{
    (void) fi;

    std::vector<char> buf(size != 0 ? size : 1);

    ssize_t n = tree.pread(ino, buf.data(), size, off);
    if (n < 0) {
        (void) fuse_reply_err(req, (int) -n);
        return;
    }

    (void) fuse_reply_buf(req, buf.data(), (size_t) n);
}

/*
 * Serve a positioned write via tree.pwrite (same liveness gate as opRead).  The
 * node layer enforces the endpoint's range/width policy and reports the byte
 * count consumed; a file without a write hook yields -EIO.
 */
void Fs::Impl::opWrite(fuse_req_t req, fuse_ino_t ino, const char *buf,
                       size_t size, off_t off, struct fuse_file_info *fi)
{
    (void) fi;

    ssize_t n = tree.pwrite(ino, buf, size, off);
    if (n < 0) {
        (void) fuse_reply_err(req, (int) -n);
        return;
    }

    (void) fuse_reply_write(req, (size_t) n);
}

/*
 * Serve an ioctl via tree.ioctl (same liveness gate).  We use restricted ioctl
 * (no FUSE_IOCTL_UNRESTRICTED) so the kernel parses the _IOC encoding and
 * bounce-buffers the payload; in_buf carries the argument bytes, out_bufsz the
 * room reserved for the reply.  For an _IOWR command the same fixed-size region
 * is read (inputs) and written (outputs): we hand the hook in_buf for inputs and
 * a separate zeroed reply buffer (seeded from in_buf) for outputs, then reply
 * with the reply buffer clamped to out_bufsz.  FUSE_IOCTL_COMPAT is rejected.
 */
void Fs::Impl::opIoctl(fuse_req_t req, fuse_ino_t ino, unsigned int cmd,
                       void *arg, struct fuse_file_info *fi, unsigned int flags,
                       const void *in_buf, size_t in_bufsz, size_t out_bufsz)
{
    (void) arg;
    (void) fi;

    if (flags & FUSE_IOCTL_COMPAT) {
        (void) fuse_reply_err(req, ENOSYS);
        return;
    }

    if (out_bufsz > kIoctlMax || in_bufsz > kIoctlMax) {
        (void) fuse_reply_err(req, EINVAL);
        return;
    }

    std::vector<char> out(out_bufsz);
    if (out_bufsz != 0) {
        size_t seed = in_bufsz < out_bufsz ? in_bufsz : out_bufsz;
        std::memcpy(out.data(), in_buf, seed);
    }

    int ret = tree.ioctl(ino, cmd, in_buf, in_bufsz,
                         out_bufsz != 0 ? out.data() : nullptr, out_bufsz);
    if (ret != 0) {
        (void) fuse_reply_err(req, -ret);
        return;
    }

    (void) fuse_reply_ioctl(req, 0, out.data(), out_bufsz);
}

/*
 * Serve an unlink by resolving (parent, name) and unlinking the file node via
 * tree.unlinkChild (delete-on-last-close).  Only files marked unlinkable (the
 * qpair<Q> files) may be removed; the endpoint directories are removed by
 * revocation, not user unlink.
 */
void Fs::Impl::opUnlink(fuse_req_t req, fuse_ino_t parent, const char *name)
{
    int ret = tree.unlinkChild(parent, name);
    (void) fuse_reply_err(req, ret == 0 ? 0 : -ret);
}

/* ================================================================== */
/* Event loop integration                                             */
/* ================================================================== */

/*
 * sd-event callback: the kernel has a request waiting on the FUSE channel fd.
 * Receive it and hand it to libfuse for dispatch.  If the session has exited
 * (clean unmount or kernel-side close), tear down the event loop.
 */
void Fs::Impl::onFuseReadable(sd_event_source *s)
{
    if (fuse_session_exited(session)) {
        sd_event_exit(sd_event_source_get_event(s), 0);
        return;
    }

    int ret = fuse_session_receive_buf(session, &recv_buf);
    if (ret == -EINTR) {
        return;
    }
    if (ret <= 0) {
        /*
         * 0 means the kernel closed the channel (unmounted from outside);
         * negative is a real error.  Either way, stop the loop so the daemon
         * shuts down cleanly.
         */
        if (ret < 0) {
            LOG(LOG_ERR, "fuse_session_receive_buf failed: %s",
                std::strerror(-ret));
        }
        sd_event_exit(sd_event_source_get_event(s), 0);
        return;
    }

    fuse_session_process_buf(session, &recv_buf);

    if (fuse_session_exited(session)) {
        sd_event_exit(sd_event_source_get_event(s), 0);
    }
}

/* ================================================================== */
/* Materialize / rediscover / reload                                  */
/* ================================================================== */

/*
 * RESCAN rediscovery restore pass: rebuild any individually-removed function of
 * a still-live device (the hardware "a rescan re-enumerates the function"
 * analogy).  Orthogonal to the config-driven select pass.  For each live BDF and
 * each removable function, restoreFunction rebuilds just that function's dir node
 * (a no-op if it was not removed); when one was actually rebuilt we re-attach its
 * endpoints and re-wire its data plane to the bridge.
 */
int Fs::Impl::rediscover()
{
    static const DeviceFunction kFuncs[] = {
        DeviceFunction::Qdma,
        DeviceFunction::Bars,
    };

    std::vector<std::string> live = tree.collectLiveBdfs();

    for (const std::string &bdf : live) {
        for (DeviceFunction func : kFuncs) {
            bool rebuilt = false;

            if (tree.restoreFunction(bdf, func, &rebuilt) == -1) {
                LOG(LOG_ERR, "Rediscover: failed to restore '%s' function %d",
                    bdf.c_str(), static_cast<int>(func));
                return -1;
            }
            if (!rebuilt) {
                continue; /* function was not removed; nothing to re-attach. */
            }

            Device *dev = tree.findDevice(bdf);
            if (dev == nullptr) {
                /* Raced away (single-threaded today; defensive). */
                continue;
            }

            int aret = func == DeviceFunction::Qdma ? qdmaAttach(*dev)
                                                    : barsAttach(*dev);
            if (aret == -1) {
                LOG(LOG_ERR,
                    "Rediscover: failed to re-attach '%s' function %d endpoint",
                    bdf.c_str(), static_cast<int>(func));
                return -1;
            }

            if (bridges.reattachFunction(*dev, func) == -1) {
                LOG(LOG_ERR,
                    "Rediscover: failed to re-wire '%s' function %d data plane",
                    bdf.c_str(), static_cast<int>(func));
                return -1;
            }

            LOG(LOG_INFO, "Rediscovered '%s' function %d", bdf.c_str(),
                static_cast<int>(func));
        }
    }

    return 0;
}

/*
 * Materialize the per-device subtree for every configured accelerator that is
 * not already running.  Safely RE-INVOCABLE: the running-set is seeded from the
 * devices already live in the tree (via collectLiveBdfs), so a re-invocation only
 * selects newly-configured / re-available BDFs and never reselects an
 * already-running accelerator (the endpoint attach helpers are not idempotent, so
 * seeding the running-set is what keeps RESCAN from double-attaching).
 */
int Fs::Impl::materialize()
{
    if (config == nullptr) {
        return 0;
    }

    /* Seed the running-set from the live devices already in the tree. */
    std::vector<std::string> running = tree.collectLiveBdfs();

    std::vector<const Accelerator *> selected = config->selectNew(running);

    for (const Accelerator *acc : selected) {
        Device *dev = tree.addDevice(acc->bdf);
        if (dev == nullptr) {
            LOG(LOG_ERR, "Failed to materialize accelerator '%s'",
                acc->bdf.c_str());
            return -1;
        }

        /* Attach the per-device endpoints under the freshly-built subtree. */
        if (infoAttach(*dev) == -1) {
            LOG(LOG_ERR, "Failed to attach info endpoint for '%s'",
                acc->bdf.c_str());
            return -1;
        }

        if (barsAttach(*dev) == -1) {
            LOG(LOG_ERR, "Failed to attach bars endpoint for '%s'",
                acc->bdf.c_str());
            return -1;
        }

        if (qdmaAttach(*dev) == -1) {
            LOG(LOG_ERR, "Failed to attach qdma endpoint for '%s'",
                acc->bdf.c_str());
            return -1;
        }

        /*
         * Attach the SIM bridge: wires the reconfiguration handler onto the qdma
         * store and installs the real vpp_sim teardown as the device's
         * model-shutdown seam.  No model is spawned until the first
         * reconfiguration write arrives.
         */
        if (bridges.attach(*dev, scratch_root) == nullptr) {
            LOG(LOG_ERR, "Failed to attach SIM bridge for '%s'",
                acc->bdf.c_str());
            return -1;
        }

        LOG(LOG_INFO, "Materialized accelerator '%s'", acc->bdf.c_str());
    }

    /*
     * RESCAN rediscovery: after the config-driven select pass, run the additive
     * restore pass that rebuilds any individually-removed function of a still-live
     * device.  No-op on the first (startup) materialize.
     */
    if (rediscover() == -1) {
        return -1;
    }

    return 0;
}

/*
 * Reload callback wired into the hotplug endpoint.  Reloads the daemon config
 * from its source path and re-runs the (idempotent) materialize path so
 * RESCAN/SBR/HOTPLUG bring up any newly-configured / re-available accelerator
 * without double-attaching a surviving one.  Called with the tree lock NOT held
 * (the hotplug ioctl hook runs unlocked); materialize takes the lock itself.
 */
int Fs::Impl::reload()
{
    std::optional<std::string> path =
        config != nullptr ? config->sourcePath() : std::nullopt;

    try {
        /* Swap the active config to the freshly-loaded one (replaces any prior
         * reloaded config; the borrowed startup config is never freed here). */
        owned_config = Config::load(path);
    } catch (const std::exception &e) {
        LOG(LOG_ERR, "Reload: failed to load configuration: %s", e.what());
        return -1;
    }
    config = &*owned_config;

    if (materialize() == -1) {
        LOG(LOG_ERR, "Reload: failed to materialize updated device tree");
        return -1;
    }

    return 0;
}

/* ================================================================== */
/* Impl lifecycle (bring-up + teardown)                               */
/* ================================================================== */

Fs::Impl::Impl(const std::string &mountpoint_in, const Config &config_in,
               BridgeRegistry &bridges_in)
    : mountpoint(mountpoint_in),
      config(&config_in),
      bridges(bridges_in),
      tree([this](Ino parent, Ino child, const std::string &name) {
          /* Notifier: invalidate the kernel's dentry so a fresh lookup misses.
           * Called with the tree lock NOT held. */
          if (session == nullptr) {
              return;
          }
          int ret = fuse_lowlevel_notify_delete(session, parent, child,
                                                name.c_str(), name.size());
          if (ret != 0 && ret != -ENOSYS) {
              LOG(LOG_WARNING, "notify_delete(%s) failed: %s", name.c_str(),
                  std::strerror(-ret));
          }
      })
{
    /*
     * Scratch root for per-device model runtime dirs (VBIN unpack + ipc:// socket
     * for the spawned vpp_sim).  Configurable via SLASH_EMU_SCRATCH_ROOT so tests
     * point it under the repo .tmp; empty falls back to the bridge default.
     */
    const char *scratch_env = std::getenv("SLASH_EMU_SCRATCH_ROOT");
    if (scratch_env != nullptr && scratch_env[0] != '\0') {
        scratch_root = scratch_env;
    }

    /*
     * Bring-up failure must tear down any bridges materialize() already attached
     * WHILE THE TREE MEMBER IS STILL ALIVE.  If the ctor throws here, this Impl is
     * only partially constructed: its members (the tree included) are destroyed as
     * the construction unwinds, and the exception then propagates out to main,
     * where ~BridgeRegistry runs LATER -- by which point the tree (and its
     * Devices) are gone, so a bridge's shutdown would touch freed device nodes
     * (the ASan UAF integration found).  Catch every bring-up exception, tear the
     * bridges down now (tree still alive), then rethrow.  shutdownAll() is
     * idempotent, so the eventual ~BridgeRegistry is a clean no-op.
     */
    try {
        if (materialize() == -1) {
            throw SystemError(EIO, "Fs: failed to materialize device tree");
        }

        /*
         * Attach the single global /hotplug file at the mount root, once -- not
         * per device.  Its reload seam re-runs the materialize path for
         * RESCAN/SBR/HOTPLUG.
         */
        if (hotplugAttach(tree, [this] { return reload(); }) == -1) {
            throw SystemError(EIO, "Fs: failed to attach hotplug endpoint");
        }

        /*
         * fuse_session_new() requires a non-empty argv[0]; we pass only the
         * program name and let mount options come from elsewhere.
         * fuse_opt_free_args frees libfuse's internal argv copy (the documented
         * owner-frees-its-args contract).
         */
        char *argv[] = { const_cast<char *>("slash-emud"), nullptr };
        struct fuse_args args = FUSE_ARGS_INIT(1, argv);

        session = fuse_session_new(&args, &ops(), sizeof(ops()), this);
        fuse_opt_free_args(&args);
        if (session == nullptr) {
            throw SystemError(EIO, "Fs: failed to create FUSE session");
        }

        if (fuse_session_mount(session, mountpoint.c_str()) != 0) {
            fuse_session_destroy(session);
            session = nullptr;
            throw SystemError(EIO, "Fs: failed to mount FUSE session at " +
                                       mountpoint);
        }
        mounted = true;
    } catch (...) {
        bridges.shutdownAll();
        throw;
    }

    LOG(LOG_INFO, "Mounted slash-emu filesystem at %s", mountpoint.c_str());
}

Fs::Impl::~Impl()
{
    /*
     * Tear down every SIM bridge FIRST, while the node tree is still alive.
     * Bridge teardown shuts down any still-running vpp_sim (exit + reap + close
     * client + remove scratch) and consults the device's endpoint nodes (which
     * the tree owns) when detaching the bar/qdma backends -- so it must run before
     * the `tree` member is destroyed.  The registry is borrowed (the caller owns
     * it and, in main.cpp, is declared before Fs so it outlives this Impl), which
     * is exactly why we cannot rely on ~BridgeRegistry for the ordering: it runs
     * AFTER ~Fs, when the tree is already gone.  shutdownAll() is idempotent, so
     * the later ~BridgeRegistry is a clean no-op.  This mirrors the C cleanup_fs
     * order (bridges before tree) without reordering main.cpp.
     */
    bridges.shutdownAll();

    /* Drop the event source next so no further callbacks fire mid-teardown. */
    if (source != nullptr) {
        (void) sd_event_source_set_enabled(source, SD_EVENT_OFF);
        sd_event_source_unref(source);
        source = nullptr;
    }

    /*
     * Destroy the session before the tree: the notifier captures `session`, and
     * no FUSE op can run against the tree once the session is gone.
     */
    if (session != nullptr) {
        if (mounted) {
            fuse_session_unmount(session);
            mounted = false;
        }
        fuse_session_destroy(session);
        session = nullptr;
    }

    /* libfuse allocates recv_buf.mem lazily inside fuse_session_receive_buf. */
    std::free(recv_buf.mem);
    recv_buf.mem = nullptr;

    /* `tree` and `owned_config` are destroyed by the implicit member teardown,
     * after every reference into the tree (bridges, session) is already gone. */
}

/* ================================================================== */
/* Fs public surface                                                  */
/* ================================================================== */

Fs::Fs(const std::string &mountpoint, const Config &config,
       BridgeRegistry &bridges)
    : impl_(std::make_unique<Impl>(mountpoint, config, bridges))
{
}

Fs::~Fs() = default;

void Fs::attach(sd_event *ev)
{
    int fd = fuse_session_fd(impl_->session);
    if (fd < 0) {
        throw SystemError(EIO, "Fs: FUSE session has no valid fd");
    }

    int ret = sd_event_add_io(ev, &impl_->source, fd, EPOLLIN,
                              Impl::onFuseReadableThunk, impl_.get());
    if (ret < 0) {
        throw SystemError(-ret, "Fs: failed to watch FUSE channel fd");
    }

    ret = sd_event_source_set_description(impl_->source, "FUSE channel");
    if (ret < 0) {
        throw SystemError(-ret, "Fs: failed to set FUSE source description");
    }
}

} // namespace slash::emu
