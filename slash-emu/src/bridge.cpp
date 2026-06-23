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
 * @file bridge.cpp
 * @brief Implementation of the SIM data-plane bridge (see bridge.hpp).
 *
 * Port of bridge.c to C++20: std::filesystem for scratch dir management,
 * std::vector<std::byte> for the VBIN accumulation buffer, std::mutex for model
 * I/O serialisation, std::unique_ptr<ModelClient> for RAII client ownership, and
 * polymorphic BarBackend / QdmaMemBackend subclasses instead of vtable structs.
 *
 * Design note on accessibility: Bridge::Impl is a private nested struct in the header.
 * To avoid the C++ rule that prevents code outside the Bridge class from naming the
 * private type, all helpers that operate on Impl are declared as static member functions
 * of Bridge::Impl itself, and the backend subclasses are nested inside Bridge::Impl.
 * This keeps the complete definition internal to this TU while remaining standards-correct.
 */

#include "bridge.hpp"

#include <atomic>
#include <cerrno>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <memory>
#include <mutex>
#include <span>
#include <string>
#include <vector>

#include <sys/types.h>
#include <sys/wait.h>
#include <syslog.h>
#include <time.h>
#include <unistd.h>

#include "bars.hpp"
#include "model_client.hpp"
#include "qdma.hpp"
#include "utils.hpp"
#include "vbin.hpp"

#include "slash/uapi/slash_abi.h"

namespace slash::emu {

/* ================================================================== */
/* Constants                                                          */
/* ================================================================== */

/** @brief Default scratch root if none is configured. */
static constexpr const char *kDefaultScratchRoot = "/run/slash_emu/.scratch";

/** @brief Bounded wait for the child to die after SIGTERM before SIGKILL (ms). */
static constexpr int kTermWaitMs = 500;

/**
 * @brief Cap on the reassembled VBIN size (256 MiB).
 *
 * An abandoned / never-terminating reconfig stream cannot grow the accumulation
 * buffer past this; it is rejected with -EFBIG and the buffer is reset.
 */
static constexpr size_t kMaxVbinBytes = 256ull * 1024ull * 1024ull;

/** Monotonic nonce so repeated reconfigurations of the same device do not collide. */
static std::atomic<unsigned long> g_nonce{0};

/* ================================================================== */
/* Bridge::Impl — full definition (private to this TU)               */
/* ================================================================== */

/**
 * The full definition of Bridge::Impl lives entirely in this TU.  All helpers
 * that would otherwise need to name the private type are expressed as static
 * member functions of Impl so they can be called from Bridge's own methods.
 *
 * The two backend subclasses are nested inside Impl so they can freely access
 * Impl's fields without any name-accessibility issue.
 */
struct Bridge::Impl {
    /* ---- data members ---- */

    Device *dev = nullptr;           /**< borrowed (the owning device) */

    std::string scratchRoot;         /**< root under which run dirs are made */
    std::string runDir;              /**< this model's unpack/scratch dir, or "" */
    std::string endpoint;            /**< ipc:// endpoint string, or "" */

    pid_t pid = -1;                  /**< spawned model pid, or -1 if none */
    std::unique_ptr<ModelClient> client; /**< owning; null if no model running */

    /** Serialises all model I/O (the REQ/REP one-in-flight discipline). */
    std::mutex ioMtx;

    /**
     * VBIN reassembly: a write at SLASH_RECONFIG_BASE (re)starts the buffer; a
     * write at BASE+acc.size() appends; anything else resets + -EINVAL.
     */
    std::vector<std::byte> acc;

    /* ---- nested backend subclasses ---- */

    /**
     * BAR read/write backend: forwards validated BAR 0 (user) register ops to
     * the running model; everything else (BAR 2/4, width > 4) falls back to the
     * shadow (rc > 0), which is always kept current, so an in-range read never
     * yields -EIO (G7).
     */
    class BarBackendImpl final : public BarBackend {
    public:
        explicit BarBackendImpl(Impl &impl) : impl_(impl) {}

        int read(uint32_t bar_index, off_t off, size_t width,
                 uint64_t &value) override
        {
            /* Only BAR 0 (user region), width <= 4, forwarded to the SIM model.
             * Any other combination falls back to the shadow (rc > 0). */
            if (bar_index != SLASH_BAR_USER_IDX || width > 4) {
                return 1; /* shadow fallback */
            }

            std::unique_lock<std::mutex> lk(impl_.ioMtx);
            if (!impl_.client) {
                return 1; /* no model yet: shadow fallback */
            }
            uint32_t v = 0;
            int rc = impl_.client->scalarRead(static_cast<uint64_t>(off), v);
            lk.unlock();

            if (rc < 0) {
                return rc; /* transport failure -> -ENODEV at the seam */
            }
            value = v;
            return 0;
        }

        int write(uint32_t bar_index, off_t off, size_t width,
                  uint64_t value) override
        {
            /* Only BAR 0, width <= 4, forwarded as reg{off, val}.  Wider /
             * other-BAR writes are shadow-only (the shadow is updated by the
             * caller regardless). */
            if (bar_index != SLASH_BAR_USER_IDX || width > 4) {
                return 0; /* shadow-only; nothing forwarded */
            }

            std::unique_lock<std::mutex> lk(impl_.ioMtx);
            if (!impl_.client) {
                return 0; /* no model yet: shadow-only */
            }
            int rc = impl_.client->regWrite(static_cast<uint64_t>(off),
                                            static_cast<uint32_t>(value));
            lk.unlock();

            return rc < 0 ? rc : 0;
        }

    private:
        Impl &impl_;
    };

    /**
     * QDMA fetch/populate backend: forwards validated MM transfers to the
     * running model.  rc==0: model answered; rc>0: fall back to the store;
     * rc<0: transport failure.
     */
    class MemBackendImpl final : public QdmaMemBackend {
    public:
        explicit MemBackendImpl(Impl &impl) : impl_(impl) {}

        int fetch(uint64_t addr, void *buf, size_t len) override
        {
            std::unique_lock<std::mutex> lk(impl_.ioMtx);
            if (!impl_.client) {
                return 1; /* no model: fall back to the store */
            }
            auto sp = std::span<std::byte>(static_cast<std::byte *>(buf), len);
            int rc = impl_.client->fetch(addr, sp);
            lk.unlock();

            return rc < 0 ? rc : 0;
        }

        int populate(uint64_t addr, const void *buf, size_t len) override
        {
            std::unique_lock<std::mutex> lk(impl_.ioMtx);
            if (!impl_.client) {
                return 0; /* no model: store-only (caller already wrote the store) */
            }
            auto sp =
                std::span<const std::byte>(static_cast<const std::byte *>(buf), len);
            int rc = impl_.client->populate(addr, sp);
            lk.unlock();

            return rc < 0 ? rc : 0;
        }

    private:
        Impl &impl_;
    };

    /* ---- backend members (after the nested class definitions) ---- */
    BarBackendImpl barBackend;
    MemBackendImpl memBackend;

    /* ---- constructor ---- */
    explicit Impl(Device &d, std::string root)
        : dev(&d)
        , scratchRoot(std::move(root))
        , barBackend(*this)
        , memBackend(*this)
    {
    }

    /* ---- static helpers (operate on Impl& so they can name the type) ---- */

    /**
     * Build a unique per-run scratch dir under the bridge's scratch root and the
     * ipc:// endpoint inside it.  Returns 0 on success, a negative errno on failure.
     */
    static int makeRunDir(Impl &im)
    {
        std::error_code ec;
        char pidstr[32];
        std::snprintf(pidstr, sizeof(pidstr), "%d", static_cast<int>(::getpid()));
        std::string dirName = im.dev->bdf() + "." + pidstr + "." +
                              std::to_string(g_nonce.fetch_add(1));
        std::filesystem::path dir =
            std::filesystem::path(im.scratchRoot) / dirName;

        std::filesystem::create_directories(dir, ec);
        if (ec) {
            LOG(LOG_ERR, "bridge: failed to create run dir %s: %s",
                dir.string().c_str(), ec.message().c_str());
            return -EIO;
        }

        im.runDir = dir.string();
        im.endpoint = "ipc://" + im.runDir + "/model.sock";
        return 0;
    }

    /** Remove the run dir tree (idempotent if already gone). */
    static void removeRunDir(Impl &im)
    {
        if (im.runDir.empty()) {
            return;
        }
        std::error_code ec;
        std::filesystem::remove_all(std::filesystem::path(im.runDir), ec);
        im.runDir.clear();
        im.endpoint.clear();
    }

    /**
     * fork/exec the model executable UNSANDBOXED with SLASH_EMU_ENDPOINT set and
     * cwd at the executable's directory (the real vpp_sim loads sibling .so's by
     * relative path).  Returns 0 on success (im.pid set), a negative errno on failure.
     */
    static int spawnModel(Impl &im, const std::string &execPath)
    {
        std::filesystem::path ep(execPath);
        std::string dir = ep.parent_path().string();
        std::string file = "./" + ep.filename().string();

        pid_t pid = ::fork();
        if (pid < 0) {
            return -errno;
        }
        if (pid == 0) {
            /* Child: detach into its own session so a stray SIGINT/SIGHUP to the
             * daemon's tty does not reach the model.  Then set the endpoint env,
             * cd into the exec dir, and exec.  Any failure exits with 127. */
            (void) ::setsid();
            if (!dir.empty() && ::chdir(dir.c_str()) != 0) {
                ::_exit(127);
            }
            if (::setenv("SLASH_EMU_ENDPOINT", im.endpoint.c_str(), 1) != 0) {
                ::_exit(127);
            }
            char *const argv[] = { const_cast<char *>(file.c_str()), nullptr };
            ::execv(file.c_str(), argv);
            ::_exit(127);
        }

        im.pid = pid;
        return 0;
    }

    /**
     * Reap the child: SIGTERM, bounded poll, then SIGKILL.  Idempotent.
     */
    static void reapModel(Impl &im)
    {
        if (im.pid <= 0) {
            return;
        }

        (void) ::kill(im.pid, SIGTERM);

        struct timespec slice = { .tv_sec = 0, .tv_nsec = 5 * 1000 * 1000 }; /* 5ms */
        int waited_ms = 0;
        bool reaped = false;
        while (waited_ms < kTermWaitMs) {
            int st = 0;
            pid_t r = ::waitpid(im.pid, &st, WNOHANG);
            if (r == im.pid || (r < 0 && errno == ECHILD)) {
                reaped = true;
                break;
            }
            ::nanosleep(&slice, nullptr);
            waited_ms += 5;
        }
        if (!reaped) {
            (void) ::kill(im.pid, SIGKILL);
            (void) ::waitpid(im.pid, nullptr, 0); /* blocking: SIGKILL is prompt */
        }

        im.pid = -1;
    }

    /**
     * Tear down a running model: detach backends, send exit (best-effort), reap
     * the child, close the client, remove the run dir.  Idempotent (no-model is
     * a no-op).  Runs with the tree lock held (seam contract) -- touches only
     * bridge-private state and the borrowed device; never re-enters the spine's
     * public API.
     */
    static void teardownModel(Impl &im)
    {
        /* Detach the backends first so no new op routes to a model we are killing.
         * The backend methods also check impl_.client == nullptr and fall back to
         * the shadow/store, so NULLing the client below is a second line of
         * defence. */
        if (im.dev != nullptr) {
            (void) qdmaSetMemBackend(*im.dev, nullptr);
            (void) barsSetBackend(*im.dev, nullptr);
        }

        if (im.client) {
            std::unique_lock<std::mutex> lk(im.ioMtx);
            (void) im.client->sendExit(); /* best-effort, bounded */
            std::unique_ptr<ModelClient> c = std::move(im.client);
            im.client = nullptr;
            lk.unlock();
            /* c destroyed here (RAII; LINGER=0 so close never blocks) */
        }

        reapModel(im);
        removeRunDir(im);
    }

    /**
     * Unpack a complete VBIN, spawn vpp_sim, connect + handshake, and attach the
     * backends.  Tears down any prior model first.  Returns 0 on success.
     */
    static int applyVbin(Impl &im, std::span<const std::byte> vbin)
    {
        /* A new VBIN replaces any running model (idempotent teardown first). */
        teardownModel(im);

        int rc = makeRunDir(im);
        if (rc != 0) {
            return rc;
        }

        /* Unpack the VBIN and locate the vpp_sim executable. */
        std::string execPath;
        rc = vbinUnpackFindSim(vbin, im.runDir, execPath);
        if (rc != 0) {
            LOG(LOG_ERR, "bridge: VBIN unpack/locate failed for '%s': %s",
                im.dev->bdf().c_str(), std::strerror(-rc));
            removeRunDir(im);
            return rc;
        }

        /* Spawn the model unsandboxed. */
        rc = spawnModel(im, execPath);
        if (rc != 0) {
            LOG(LOG_ERR, "bridge: spawn failed for '%s': %s",
                im.dev->bdf().c_str(), std::strerror(-rc));
            teardownModel(im);
            return rc;
        }

        /* Connect the REQ client and run the bounded readiness handshake. */
        rc = ModelClient::connect(im.endpoint, kModelDefaultTimeoutMs, im.client);
        if (rc != 0) {
            LOG(LOG_ERR, "bridge: client connect failed for '%s': %s",
                im.dev->bdf().c_str(), std::strerror(-rc));
            teardownModel(im);
            return rc;
        }

        rc = im.client->start();
        if (rc != 0) {
            LOG(LOG_ERR, "bridge: model handshake failed for '%s': %s",
                im.dev->bdf().c_str(), std::strerror(-rc));
            teardownModel(im);
            return rc;
        }

        /* Attach the data-plane backends so subsequent ops route to the model. */
        if (qdmaSetMemBackend(*im.dev, &im.memBackend) != 0) {
            LOG(LOG_ERR, "bridge: failed to attach qdma backend for '%s'",
                im.dev->bdf().c_str());
            teardownModel(im);
            return -EIO;
        }
        if (barsSetBackend(*im.dev, &im.barBackend) != 0) {
            LOG(LOG_ERR, "bridge: failed to attach bar backend for '%s'",
                im.dev->bdf().c_str());
            teardownModel(im);
            return -EIO;
        }

        LOG(LOG_INFO, "bridge: model up for '%s' (pid %d, %s)",
            im.dev->bdf().c_str(), static_cast<int>(im.pid), im.endpoint.c_str());
        return 0;
    }
};

/* ================================================================== */
/* Bridge construction / destruction                                  */
/* ================================================================== */

Bridge::Bridge() : impl_(nullptr) {}
Bridge::~Bridge() = default;

Device &Bridge::device() const { return *impl_->dev; }

/* ================================================================== */
/* Bridge public API                                                  */
/* ================================================================== */

/**
 * @brief Reconfiguration handler: reassemble contiguous reconfig-region chunks
 * and apply a complete VBIN.
 */
int Bridge::reconfigure(uint64_t addr, std::span<const std::byte> vbin)
{
    if (vbin.empty()) {
        return -EINVAL;
    }

    Bridge::Impl &im = *impl_;

    /*
     * Reassemble contiguous reconfig-region chunks (the kernel splits a large
     * VBIN write).  A write at the window base (re)starts the buffer; a write at
     * the running BASE+acc.size() offset appends.  Anything else is a
     * non-contiguous / seeking write -- not how a VBIN is delivered -- so we
     * reset any partial transfer and reject it (-EINVAL) rather than silently
     * corrupting the stream.
     */
    if (addr == SLASH_RECONFIG_BASE) {
        im.acc.clear(); /* fresh transfer, replaces any partial one */
    } else if (addr != SLASH_RECONFIG_BASE + static_cast<uint64_t>(im.acc.size())) {
        im.acc.clear();
        LOG(LOG_ERR,
            "bridge: non-contiguous reconfig write for '%s' "
            "(addr 0x%llx, expected 0x%llx)",
            im.dev->bdf().c_str(),
            static_cast<unsigned long long>(addr),
            static_cast<unsigned long long>(SLASH_RECONFIG_BASE + im.acc.size()));
        return -EINVAL;
    }

    /* Cap the accumulation so an abandoned / never-terminating stream cannot
     * grow memory without bound. */
    if (im.acc.size() + vbin.size() > kMaxVbinBytes) {
        im.acc.clear();
        LOG(LOG_ERR, "bridge: VBIN exceeds cap (%zu B) for '%s'",
            kMaxVbinBytes, im.dev->bdf().c_str());
        return -EFBIG;
    }

    /* Append the chunk. */
    im.acc.insert(im.acc.end(), vbin.begin(), vbin.end());

    /*
     * Classify the accumulated archive: keep accepting chunks until the ustar
     * terminator is seen; reject a structurally-invalid stream up front (so a
     * bogus first chunk fails immediately, not after exhausting the cap).
     */
    VbinStatus st = vbinClassify(std::span<const std::byte>(im.acc));
    if (st == VbinStatus::Invalid) {
        im.acc.clear();
        LOG(LOG_ERR, "bridge: malformed VBIN for '%s'", im.dev->bdf().c_str());
        return -EINVAL;
    }
    if (st == VbinStatus::Incomplete) {
        return 0; /* accept the chunk; await the rest */
    }

    /* Complete: apply it, then reset the buffer regardless of outcome. */
    int rc = Bridge::Impl::applyVbin(im, std::span<const std::byte>(im.acc));
    im.acc.clear();
    return rc;
}

/** @brief Tear down the running model (the model-shutdown seam body). */
void Bridge::shutdown()
{
    Bridge::Impl::teardownModel(*impl_);
}

/* ================================================================== */
/* BridgeRegistry                                                     */
/* ================================================================== */

BridgeRegistry::BridgeRegistry() = default;

BridgeRegistry::~BridgeRegistry()
{
    shutdownAll();
}

void BridgeRegistry::shutdownAll()
{
    /* Steal the vector so a re-entrant or second call is a guaranteed no-op
     * that touches nothing.  After the move, bridges_ is empty; ~BridgeRegistry
     * calling shutdownAll() a second time iterates an empty vector and never
     * dereferences a Device that the NodeTree may have already freed. */
    std::vector<std::unique_ptr<Bridge>> local;
    local.swap(bridges_); /* bridges_ is now empty */

    /* Tear down in reverse-construction order (most recently attached first). */
    for (auto it = local.rbegin(); it != local.rend(); ++it) {
        Bridge &b = **it;
        Bridge::Impl &im = *b.impl_;

        /* Clear the model-shutdown seam BEFORE teardown so no dangling callback
         * can fire against a Bridge object that is about to be freed.  The seam
         * clear must happen while im.dev is still valid (i.e. this is the FIRST
         * shutdownAll() call, before the NodeTree destructs). */
        if (im.dev != nullptr) {
            (void) im.dev->setModelShutdown({});
        }

        Bridge::Impl::teardownModel(im);
    }
    /* local is destroyed here, freeing every Bridge object. */
}

/**
 * @brief Attach a SIM bridge to a device (called once at materialize time).
 */
Bridge *BridgeRegistry::attach(Device &dev, const std::string &scratch_root)
{
    if (dev.qdma == nullptr) {
        LOG(LOG_ERR, "bridge attach: device '%s' has no qdma endpoint",
            dev.bdf().c_str());
        return nullptr;
    }

    const std::string &root =
        scratch_root.empty() ? std::string(kDefaultScratchRoot) : scratch_root;

    auto b = std::unique_ptr<Bridge>(new Bridge());
    b->impl_ = std::make_unique<Bridge::Impl>(dev, root);

    /* Wire the reconfig handler so a reconfig-region write routes here.
     * Capture a raw pointer; the registry owns the bridge for its lifetime. */
    Bridge *rawBridge = b.get();
    QdmaReconfigFn reconfigFn = [rawBridge](uint64_t addr, const void *data,
                                            size_t len) -> int {
        auto sp = std::span<const std::byte>(
            static_cast<const std::byte *>(data), len);
        return rawBridge->reconfigure(addr, sp);
    };

    if (qdmaSetReconfigHandler(dev, reconfigFn) != 0) {
        LOG(LOG_ERR, "bridge: failed to wire reconfig handler for '%s'",
            dev.bdf().c_str());
        return nullptr;
    }

    /* Replace the model-shutdown seam default with the real teardown. */
    ModelShutdownFn shutdownFn = [rawBridge](Device &) {
        rawBridge->shutdown();
    };

    if (dev.setModelShutdown(std::move(shutdownFn)) != 0) {
        LOG(LOG_ERR, "bridge: failed to wire model-shutdown seam for '%s'",
            dev.bdf().c_str());
        (void) qdmaSetReconfigHandler(dev, {});
        return nullptr;
    }

    Bridge *kept = rawBridge;
    bridges_.push_back(std::move(b));
    return kept;
}

/**
 * @brief Re-wire a rediscovered function's data plane to the bridge (RESCAN).
 */
int BridgeRegistry::reattachFunction(Device &dev, DeviceFunction func)
{
    /* Find the bridge for this device. */
    Bridge *b = nullptr;
    for (auto &up : bridges_) {
        if (up->impl_->dev == &dev) {
            b = up.get();
            break;
        }
    }
    if (b == nullptr) {
        /* No bridge for this device (e.g. a unit harness that never attached
         * one).  Nothing to re-wire; the rebuilt subtree is a valid bare state. */
        return 0;
    }

    Bridge::Impl &im = *b->impl_;

    if (func == DeviceFunction::Qdma) {
        /* The device-scoped store survives a per-function QDMA remove (its HBM/DDR
         * contents are preserved), but the rebuilt qdma/ node's QdmaDirOps is a
         * fresh co-owner that re-exposes that same store; re-install the reconfig
         * handler unconditionally so the rediscovered endpoint routes reconfig
         * writes back here (the captured bridge survived because only the function,
         * not the whole device, was removed).  If a model is currently running,
         * also re-point the store's mem backend at the live model so a post-restore
         * QDMA transfer round-trips through it again. */
        QdmaReconfigFn reconfigFn = [b](uint64_t addr, const void *data,
                                        size_t len) -> int {
            auto sp = std::span<const std::byte>(
                static_cast<const std::byte *>(data), len);
            return b->reconfigure(addr, sp);
        };
        if (qdmaSetReconfigHandler(dev, reconfigFn) != 0) {
            LOG(LOG_ERR, "Reattach: failed to wire reconfig handler for '%s'",
                dev.bdf().c_str());
            return -1;
        }
        if (im.client) {
            if (qdmaSetMemBackend(dev, &im.memBackend) != 0) {
                LOG(LOG_ERR,
                    "Reattach: failed to re-wire qdma backend for '%s'",
                    dev.bdf().c_str());
                return -1;
            }
        }
    } else if (func == DeviceFunction::Bars) {
        /* The rebuilt bar files carry fresh in-memory shadows; if a model is
         * running, re-attach the bar backend so a post-restore register poke
         * round-trips through it. */
        if (im.client) {
            if (barsSetBackend(dev, &im.barBackend) != 0) {
                LOG(LOG_ERR,
                    "Reattach: failed to re-wire bar backend for '%s'",
                    dev.bdf().c_str());
                return -1;
            }
        }
    } else {
        return -1;
    }

    return 0;
}

} // namespace slash::emu
