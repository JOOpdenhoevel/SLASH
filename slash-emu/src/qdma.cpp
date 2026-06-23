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
 * @file qdma.cpp
 * @brief Implementation of the @c /<BDF>/qdma/ endpoint (see qdma.hpp).
 *
 * The C++20 port of @c qdma.c.  The data model is unchanged; only the ownership
 * mechanics move from hand-rolled C (the @c array.h page array, @c void* node
 * backings, an explicit destroy hook) to RAII:
 *
 *   - The per-device sparse memory store is a @ref QdmaStore held by a
 *     @c std::shared_ptr.  That @c shared_ptr is co-owned by the @c qdma/
 *     directory's @ref QdmaDirOps and by every qpair's @ref QpairOps, so the
 *     store outlives any individual qpair (a qpair torn down via the registry can
 *     still be read through an open fd until its last close) and is freed exactly
 *     when the directory ops and the last qpair ops are gone.
 *   - A qpair file's behaviour lives in @ref QpairOps (a @ref NodeOps subclass);
 *     there is no separate node backing.  The spine frees the ops with the node.
 *   - The @ref Resource teardown ("stop the queue / free the QID") is a
 *     @c std::function the spine runs at most once.
 */

#include "qdma.hpp"

#include <cstring>
#include <memory>

#include "node.hpp"
#include "qdma_store.hpp"
#include "slash/uapi/slash_abi.h"
#include "utils.hpp"

namespace slash::emu {

/* ================================================================== */
/* Device destructor (defined here, where QdmaStore is complete)      */
/* ================================================================== */

/*
 * Out-of-line so the device-scoped shared_ptr<QdmaStore> member (forward-declared
 * in node.hpp) destructs against the complete type defined in qdma_store.hpp.
 */
Device::~Device() = default;

namespace {

/* QDMA ring sizes are CSR table indices, not byte counts: 0..15 inclusive. */
constexpr uint32_t kRingIdxMax = 15u;

/* Queue operating modes (mirrors the legacy slash_qdma_qpair_add semantics). */
constexpr uint32_t kModeMm = 0u;
constexpr uint32_t kModeSt = 1u;

/* dir_mask bits. */
constexpr uint32_t kDirH2c = 0x1u;
constexpr uint32_t kDirC2h = 0x2u;
constexpr uint32_t kDirCmpt = 0x4u;

} // namespace

/* ================================================================== */
/* Address-range + parameter validation (pure, unit-tested)           */
/* ================================================================== */

int qdmaCheckRange(uint64_t addr, size_t len)
{
    /* A zero-length transfer copies nothing and is trivially in range. */
    if (len == 0) {
        return 0;
    }

    /* Guard the end computation against overflow. */
    uint64_t end = addr + static_cast<uint64_t>(len);
    if (end < addr) {
        return -ERANGE;
    }

    /*
     * The whole transfer must lie within ONE valid window.  HBM and DDR are each
     * a single contiguous range here (bank structure is internal to the model);
     * the reconfiguration region is intentionally NOT accepted (the VBIN path).
     * A transfer that straddles a window boundary is out of range.
     */
    if (addr >= SLASH_HBM_BASE && end <= SLASH_HBM_END) {
        return 0;
    }
    if (addr >= SLASH_DDR_BASE && end <= SLASH_DDR_END) {
        return 0;
    }

    return -ERANGE;
}

bool qdmaIsReconfigWrite(uint64_t addr, size_t len)
{
    /* A zero-length write carries no VBIN: not a reconfiguration. */
    if (len == 0) {
        return false;
    }

    /* Guard the end computation against overflow. */
    uint64_t end = addr + static_cast<uint64_t>(len);
    if (end < addr) {
        return false;
    }

    /* The whole write must lie within the reconfiguration region. */
    if (addr >= SLASH_RECONFIG_BASE && end <= SLASH_RECONFIG_END) {
        return true;
    }

    return false;
}

int qdmaCheckQpairAdd(uint32_t mode, uint32_t dir_mask, uint32_t h2c_ring,
                      uint32_t c2h_ring, uint32_t cmpt_ring)
{
    /* Streaming mode is deferred (step-1 decision): MM only. */
    if (mode == kModeSt) {
        return -EOPNOTSUPP;
    }
    if (mode != kModeMm) {
        return -EINVAL;
    }

    /* CMPT direction is not supported. */
    if (dir_mask & kDirCmpt) {
        return -EOPNOTSUPP;
    }

    /* Only H2C/C2H bits are meaningful; at least one direction is required. */
    if (dir_mask & ~(kDirH2c | kDirC2h)) {
        return -EINVAL;
    }
    if ((dir_mask & (kDirH2c | kDirC2h)) == 0) {
        return -EINVAL;
    }

    /* Ring sizes are CSR table indices 0..15. */
    if (h2c_ring > kRingIdxMax || c2h_ring > kRingIdxMax ||
        cmpt_ring > kRingIdxMax) {
        return -EINVAL;
    }

    return 0;
}

namespace {

/* ================================================================== */
/* qpair<Q> data file: ops (size + read/write hooks)                  */
/* ================================================================== */

/*
 * Per-qpair behaviour.  A qpair owns no device memory of its own -- the store is
 * per-device and shared -- so QpairOps holds only:
 *
 *   - `store_`: the device's sparse store (co-owned shared_ptr, so it outlives a
 *               qpair torn down via the registry while an fd is still open);
 *   - `tree_`:  for resourceCheckLocked's liveness probe;
 *   - `res_`:   this qpair's resource (borrowed; the inode ref is held by the
 *               node, so the pointer stays valid for as long as the node lives).
 *
 * The spine destroys QpairOps with the node (the C "destroy hook").  The resource
 * may outlive the node (registry ref still held between a forced teardown and the
 * holder's close); the shared store survives because the dir ops co-own it.
 */
class QpairOps : public NodeOps {
public:
    QpairOps(std::shared_ptr<QdmaStore> store, NodeTree &tree, Resource *res,
             uint32_t qid)
        : store_(std::move(store)), tree_(tree), res_(res), qid_(qid)
    {
    }

    /* A qpair file has no meaningful "size" (it is an address window, not a byte
     * file).  Report 0 -- direct_io reads/writes carry their own length and
     * offset and never consult st_size. */
    off_t size(const Node &) const override { return 0; }

    /* pread hook: serve a validated MM read from device memory. */
    ssize_t read(const Node &, char *buf, size_t size, off_t off) override
    {
        int rc = live();
        if (rc != 0) {
            return rc;
        }

        uint64_t addr = static_cast<uint64_t>(off); /* offset IS device addr */
        rc = qdmaCheckRange(addr, size);
        if (rc != 0) {
            return rc;
        }

        /*
         * SIM bridge seam: ask the model first.  rc == 0 => model answered;
         * rc > 0 => fall back to the sparse store (an in-range read the model
         * cannot answer must still return defined bytes, never -EIO); rc < 0 =>
         * transport failure (-ENODEV).
         */
        if (store_->backend != nullptr) {
            rc = store_->backend->fetch(addr, buf, size);
            if (rc < 0) {
                return -ENODEV;
            }
            if (rc == 0) {
                return static_cast<ssize_t>(size);
            }
            /* rc > 0: fall through to the sparse store. */
        }

        store_->read(addr, buf, size);

        return static_cast<ssize_t>(size);
    }

    /* pwrite hook: store a validated MM write into device memory. */
    ssize_t write(const Node &, const char *buf, size_t size,
                  off_t off) override
    {
        int rc = live();
        if (rc != 0) {
            return rc;
        }

        uint64_t addr = static_cast<uint64_t>(off);

        /*
         * Reconfiguration seam: a write whose whole range lies in the reconfig
         * region is (part of) a VBIN delivery, NOT an ordinary memory transfer.
         * Detect it BEFORE the HBM/DDR range check (which still rejects the
         * region with -ERANGE for reads and for the no-handler case), and route
         * the bytes to the reconfiguration handler, which reassembles chunks the
         * kernel split out of a large VBIN write.  A reconfig write with no
         * handler attached is rejected with -ERANGE, exactly as the region would
         * be without the bridge -- the VBIN has nowhere to go.
         */
        if (qdmaIsReconfigWrite(addr, size)) {
            if (!store_->reconfig) {
                return -ERANGE;
            }
            rc = store_->reconfig(addr, buf, size);
            if (rc != 0) {
                return rc; /* negative errno (malformed VBIN, ...) */
            }
            return static_cast<ssize_t>(size);
        }

        rc = qdmaCheckRange(addr, size);
        if (rc != 0) {
            return rc;
        }

        /*
         * Keep the sparse store current even when a SIM backend is attached, so
         * an in-range read the model cannot answer can fall back to it (never
         * -EIO).
         */
        store_->write(addr, buf, size);

        /* SIM bridge seam: forward the validated transfer to the model. */
        if (store_->backend != nullptr) {
            if (store_->backend->populate(addr, buf, size) < 0) {
                return -ENODEV;
            }
        }

        return static_cast<ssize_t>(size);
    }

private:
    /*
     * Shared liveness gate for the qpair data hooks.  The spine's pread/pwrite
     * already checked node->live under the lock before calling us, but the
     * resource is the authoritative liveness token (it can be torn down via the
     * registry even in paths the node flag does not cover), so consult it too.
     * The hooks run under the tree lock the spine already holds, so use the
     * lock-held variant (the public one would deadlock).
     */
    int live() const { return tree_.resourceCheckLocked(res_); }

    std::shared_ptr<QdmaStore> store_; /* co-owned (device store) */
    NodeTree &tree_;                   /* non-owning */
    Resource *res_;                    /* borrowed */
    uint32_t qid_;
};

/* ================================================================== */
/* qdma/ directory: ops + QPAIR_ADD ioctl                             */
/* ================================================================== */

/*
 * Ops for the qdma/ directory node.  Co-owns the per-device sparse memory store
 * and tracks QID allocation.  Attached onto the existing dir node by qdmaAttach
 * via NodeTree::setOps; the spine owns it (frees it, and with it the directory's
 * shared_ptr to the store, when the node is destroyed).
 */
class QdmaDirOps : public NodeOps {
public:
    QdmaDirOps(Device &dev, std::shared_ptr<QdmaStore> store)
        : dev_(dev), store_(std::move(store))
    {
    }

    /* QPAIR_ADD ioctl handler on the qdma/ directory node. */
    int ioctl(Node &, unsigned int cmd, const void *in, size_t in_size,
              void *out, size_t out_size) override
    {
        if (cmd != SLASH_ABI_QDMA_IOCTL_QPAIR_ADD) {
            return -ENOTTY;
        }

        /*
         * The caller's struct must carry at least the input prefix we read
         * (through cmpt_ring) and the reply must have room for the qid we write.
         * Both buffers are the FUSE layer's fixed-size bounce copies; the qid
         * field sits at the very end of the struct, so requiring the full struct
         * size for both directions is the simplest correct check for this fixed
         * _IOWR layout.
         */
        if (in_size < sizeof(struct slash_abi_qdma_qpair_add) ||
            out_size < sizeof(struct slash_abi_qdma_qpair_add)) {
            return -EINVAL;
        }

        const auto *req = static_cast<const struct slash_abi_qdma_qpair_add *>(in);

        int rc = qdmaCheckQpairAdd(req->mode, req->dir_mask, req->h2c_ring_sz,
                                   req->c2h_ring_sz, req->cmpt_ring_sz);
        if (rc != 0) {
            return rc;
        }

        uint32_t qid = 0;
        if (allocQid(&qid) == -1) {
            return -ENOSPC;
        }

        if (createQpair(qid) == -1) {
            return -ENOMEM;
        }

        /* Write the allocated QID back into the reply struct (the out buffer was
         * seeded from the in buffer by the FUSE layer, so the input fields are
         * preserved and we only set the [out] qid). */
        auto *resp = static_cast<struct slash_abi_qdma_qpair_add *>(out);
        resp->qid = qid;

        return 0;
    }

private:
    /*
     * Allocate a unique per-device QID.  The architecture requires only that the
     * daemon hands out a unique id (not a particular one), so this is a monotonic
     * counter: it returns `next_qid_` and advances, which guarantees uniqueness
     * for the entire practical lifetime of a device (a u32 never wraps in any
     * real run).  "Free" means no resource with that id is currently registered.
     *
     * Two non-monotonic cases keep the invariant at the theoretical edge: if
     * `next_qid_` ever collides with a still-registered id (only reachable after
     * a full u32 wrap), fall back to a linear scan for the lowest free id; do NOT
     * otherwise reuse freed ids below the counter (so the common allocation is
     * strictly increasing, not smallest-free).  QID-space exhaustion (4 billion
     * concurrently-live qpairs) is impossible in practice and returns an error.
     *
     * Runs inside the QPAIR_ADD ioctl hook (tree lock already held), so use the
     * lock-held registry probe.
     */
    int allocQid(uint32_t *out)
    {
        /* Fast path: hand out the monotonic counter if it is genuinely free. */
        if (dev_.findResourceLocked(next_qid_) == nullptr) {
            *out = next_qid_;
            next_qid_++;
            return 0;
        }

        /* Slow path (counter collided with a reused id): linear search from 0. */
        for (uint32_t q = 0; q < UINT32_MAX; q++) {
            if (dev_.findResourceLocked(q) == nullptr) {
                *out = q;
                if (q + 1 > next_qid_) {
                    next_qid_ = q + 1;
                }
                return 0;
            }
        }

        return -1; /* QID space exhausted (4 billion live qpairs -- impossible) */
    }

    /*
     * Create one qpair<Q>: create the file node with QpairOps, register a
     * refcounted resource, attach that resource to the node (the inode ref), and
     * mark it direct_io + unlinkable.  Assumes parameters already validated and
     * `qid` already allocated.
     *
     * Ordering is chosen so every fallible step can be unwound with a public
     * spine primitive that takes no inode/registry ref we cannot release locally:
     *
     *   1. create the node        -> unwind: unlinkLocked (destroys it; the
     *                                node's lookup_count is 0, just made)
     *   2. register the resource  -> unwind: unlinkLocked
     *   3. attach resource->node  -> infallible for a fresh node (a brand-new
     *                                node has no resource); failure can only mean
     *                                a programming error, handled defensively.
     *
     * Because step 3 cannot fail for a fresh node, no path leaves a registered
     * resource without an inode ref; the cooperative/forced teardown machinery
     * then owns the qpair's whole lifetime.
     *
     * Runs inside the QPAIR_ADD ioctl hook, which the spine invokes with the tree
     * lock HELD, so every spine call here is the lock-held (_Locked) variant; the
     * public re-locking ones would deadlock the non-recursive mutex.
     */
    int createQpair(uint32_t qid)
    {
        NodeTree &tree = dev_.tree();

        char name[32];
        int n = std::snprintf(name, sizeof(name), "qpair%u", qid);
        if (n < 0 || static_cast<size_t>(n) >= sizeof(name)) {
            LOG(LOG_ERR, "qpair name overflow for qid %u", qid);
            return -1;
        }

        /*
         * 1. Create the node WITHOUT ops first: QpairOps needs the resource
         *    pointer (step 2) to gate its hooks, and the spine's createChild
         *    wants ops at creation time, so create bare and setOps after the
         *    resource exists.
         */
        Node *node = tree.createChildLocked(dev_.qdma, name, NodeType::File,
                                            0600, nullptr);
        if (node == nullptr) {
            LOG(LOG_ERR, "Failed to create %s node", name);
            return -1;
        }

        /*
         * 2. Register the resource.  The teardown ("stop the queue / free the
         *    QID") is idempotent and the spine guarantees it runs at most once;
         *    nothing external to release in the in-memory model (the QID is freed
         *    implicitly when the resource leaves the registry).
         */
        Resource *res = dev_.registerResourceLocked(
            qid, [qid](Resource &) {
                LOG(LOG_DEBUG, "qpair %u torn down", qid);
            });
        if (res == nullptr) {
            LOG(LOG_ERR, "Failed to register qpair %u resource", qid);
            tree.unlinkLocked(node); /* destroys the just-created node */
            return -1;
        }

        /* Now the resource exists: install the qpair ops (co-own the store). */
        if (tree.setOpsLocked(
                node, std::make_unique<QpairOps>(store_, tree, res, qid)) ==
            -1) {
            LOG(LOG_ERR, "Failed to set ops on %s", name);
            tree.unlinkLocked(node);
            return -1;
        }

        /*
         * 3. Attach the resource to the node (the inode reference).  Obtain the
         *    SECOND co-owning shared_ptr via shared_from_this().  Infallible for
         *    a fresh node; on the impossible failure, unlink to keep things
         *    consistent (the registry ref is then the resource's only ref and is
         *    reaped at device teardown).
         */
        if (tree.attachResourceLocked(node, res->shared_from_this()) == -1) {
            LOG(LOG_ERR, "Failed to attach resource to %s", name);
            tree.unlinkLocked(node);
            return -1;
        }

        /* Memory transfers are unbuffered: open with direct_io so the kernel
         * forwards each pread/pwrite verbatim (exact size + offset) rather than
         * synthesizing page-sized, page-cached transfers. */
        tree.setDirectIoLocked(node);

        /* qpair<Q> files are user-unlinkable: the VRTD delete-on-last-close
         * pattern (ADD -> open -> unlink while open) rides the FUSE unlink op,
         * which is opt-in.  info / bar<M> / hotplug are left non-unlinkable. */
        tree.setUnlinkableLocked(node);

        return 0;
    }

    Device &dev_;                      /* the owning device (non-owning ref) */
    std::shared_ptr<QdmaStore> store_; /* co-owned (the device's per-device store) */
    uint32_t next_qid_ = 0;            /* monotonic allocation hint */
};

} // namespace

/* ================================================================== */
/* Attach + bridge-seam wiring                                        */
/* ================================================================== */

int qdmaAttach(Device &dev)
{
    if (dev.qdma == nullptr) {
        return -1;
    }

    /*
     * The sparse memory store lives at device scope so a per-function QDMA
     * REMOVE+RESCAN preserves HBM/DDR: the first attach allocates it; a re-attach
     * after a per-function remove (revokeFunction destroyed the qdma/ node and its
     * QdmaDirOps co-owner, but left dev.qdmaStore intact) REUSES the same store, so
     * the rediscovered endpoint sees the same memory.  A whole-device revoke clears
     * dev.qdmaStore, so a fresh device after teardown starts with zeroed memory.
     */
    if (dev.qdmaStore == nullptr) {
        dev.qdmaStore = std::make_shared<QdmaStore>();
    }

    /* Attach the ioctl vtable onto the qdma/ dir node, which addDevice already
     * created, co-owning the device's store.  From here the node owns the ops (and,
     * through it, a co-owning shared_ptr to the store) and frees them when the node
     * is destroyed; the device's reference is what keeps the store alive across a
     * per-function remove. */
    if (dev.tree().setOps(dev.qdma,
                          std::make_unique<QdmaDirOps>(dev, dev.qdmaStore)) ==
        -1) {
        LOG(LOG_ERR, "Failed to attach qdma ops for '%s'", dev.bdf().c_str());
        return -1;
    }

    return 0;
}

int qdmaSetMemBackend(Device &dev, QdmaMemBackend *backend)
{
    /* The SIM seams live on the device-scoped store (shared by the qdma/ dir ops
     * and every qpair).  Target the store directly rather than the dir ops: it
     * survives a per-function QDMA remove, so the bridge can still detach a backend
     * (e.g. during model teardown) after the qdma/ node is gone.  A null store
     * means the device was fully revoked -- nothing to wire onto. */
    if (dev.qdmaStore == nullptr) {
        return -1;
    }

    dev.qdmaStore->backend = backend;

    return 0;
}

int qdmaSetReconfigHandler(Device &dev, QdmaReconfigFn handler)
{
    if (dev.qdmaStore == nullptr) {
        return -1;
    }

    dev.qdmaStore->reconfig = std::move(handler);

    return 0;
}

} // namespace slash::emu
