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
 * @file qdma.c
 * @brief Implementation of the @c /<BDF>/qdma/ endpoint (see qdma.h).
 */

#define _GNU_SOURCE

#include "qdma.h"

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <syslog.h>

#include "array.h"
#include "slash/uapi/slash_abi.h"
#include "utils.h"

/* QDMA ring sizes are CSR table indices, not byte counts: 0..15 inclusive. */
#define EMU_QDMA_RING_IDX_MAX 15u

/* Queue operating modes (mirrors the legacy slash_qdma_qpair_add semantics). */
#define EMU_QDMA_MODE_MM 0u
#define EMU_QDMA_MODE_ST 1u

/* dir_mask bits. */
#define EMU_QDMA_DIR_H2C  0x1u
#define EMU_QDMA_DIR_C2H  0x2u
#define EMU_QDMA_DIR_CMPT 0x4u

/*
 * Sparse memory store page size.  Device memory (32 GiB HBM + 32 GiB DDR) is far
 * too large to allocate, so the store is a paged, lazily-populated map: each
 * touched 64 KiB page is allocated on first write and zero-filled.  64 KiB keeps
 * the page table small while not wasting much on a single-byte poke.
 */
#define EMU_QDMA_PAGE_SIZE (64u * 1024u)

/* ================================================================== */
/* Per-device sparse memory store                                     */
/* ================================================================== */

/*
 * One lazily-allocated page of device memory, keyed by its page-aligned device
 * address.  `bytes` is always EMU_QDMA_PAGE_SIZE long and zero-initialised.
 */
struct emu_qdma_page {
    uint64_t base;   /* page-aligned device address */
    uint8_t *bytes;  /* owning, EMU_QDMA_PAGE_SIZE long */
};

static void emu_qdma_page_free(struct emu_qdma_page *p)
{
    if (p == NULL) {
        return;
    }
    free(p->bytes);
    free(p);
}

DECLARE_OWNING_PTR_ARRAY(emu_qdma_page_array, struct emu_qdma_page *,
                         emu_qdma_page_free)

/*
 * The per-device sparse store: the page table plus the optional T10 SIM bridge
 * seam.  Owned by the qdma/ directory node's backing (one store per device,
 * shared by every qpair of that device).  Mutated only from the ioctl/read/write
 * hooks, which the spine invokes with the tree lock held, so it needs no locking
 * of its own.
 */
struct emu_qdma_store {
    struct emu_qdma_page_array pages;

    /* SIM memory-bridge seam (T10).  NULL => in-memory sparse store only. */
    const struct emu_qdma_mem_backend *backend; /* non-owning, static */

    /* Reconfiguration seam (T10): a write into the reconfig region is a VBIN. */
    emu_qdma_reconfig_fn reconfig;     /* NULL => no reconfig handler attached */
    void *reconfig_ctx;                /* borrowed (the device's emu_bridge) */
};

/* Find an existing page covering page-aligned address `base` (or NULL). */
static struct emu_qdma_page *store_find_page(struct emu_qdma_store *store,
                                             uint64_t base)
{
    for (size_t i = 0; i < store->pages.len; i++) {
        if (store->pages.d[i]->base == base) {
            return store->pages.d[i];
        }
    }
    return NULL;
}

/* Get (allocating if needed) the page covering page-aligned address `base`. */
static struct emu_qdma_page *store_get_page(struct emu_qdma_store *store,
                                            uint64_t base)
{
    struct emu_qdma_page *existing = store_find_page(store, base);
    if (existing != NULL) {
        return existing;
    }

    _cleanup_(cleanup_free)
    struct emu_qdma_page *page = calloc(1, sizeof(*page));
    if (page == NULL) {
        return NULL;
    }
    page->base = base;
    page->bytes = calloc(1, EMU_QDMA_PAGE_SIZE);
    if (page->bytes == NULL) {
        return NULL;
    }

    struct emu_qdma_page *ret = page;
    if (emu_qdma_page_array_push(&store->pages, page) == -1) {
        emu_qdma_page_free(page);
        page = NULL;
        return NULL;
    }
    page = NULL; /* ownership transferred to the array */

    return ret;
}

/*
 * Copy `len` bytes out of the store starting at device address `addr` into
 * `dst`, treating never-written pages as zero.  Range is pre-validated.
 */
static void store_read(struct emu_qdma_store *store, uint64_t addr, void *dst,
                       size_t len)
{
    uint8_t *out = dst;
    size_t done = 0;

    while (done < len) {
        uint64_t cur = addr + done;
        uint64_t base = cur - (cur % EMU_QDMA_PAGE_SIZE);
        size_t in_page = (size_t) (cur - base);
        size_t chunk = EMU_QDMA_PAGE_SIZE - in_page;
        if (chunk > len - done) {
            chunk = len - done;
        }

        struct emu_qdma_page *page = store_find_page(store, base);
        if (page == NULL) {
            memset(out + done, 0, chunk); /* never written: defined zero */
        } else {
            memcpy(out + done, page->bytes + in_page, chunk);
        }

        done += chunk;
    }
}

/*
 * Copy `len` bytes from `src` into the store starting at device address `addr`,
 * allocating pages on demand.  Range is pre-validated.  Returns 0, or -ENOMEM if
 * a page allocation failed (a partial write may have occurred -- acceptable for
 * an OOM, which is already a degraded state).
 */
static int store_write(struct emu_qdma_store *store, uint64_t addr,
                       const void *src, size_t len)
{
    const uint8_t *in = src;
    size_t done = 0;

    while (done < len) {
        uint64_t cur = addr + done;
        uint64_t base = cur - (cur % EMU_QDMA_PAGE_SIZE);
        size_t in_page = (size_t) (cur - base);
        size_t chunk = EMU_QDMA_PAGE_SIZE - in_page;
        if (chunk > len - done) {
            chunk = len - done;
        }

        struct emu_qdma_page *page = store_get_page(store, base);
        if (page == NULL) {
            return -ENOMEM;
        }
        memcpy(page->bytes + in_page, in + done, chunk);

        done += chunk;
    }

    return 0;
}

/* ================================================================== */
/* Address-range + parameter validation (pure, unit-tested)           */
/* ================================================================== */

int emu_qdma_check_range(uint64_t addr, size_t len)
{
    /* A zero-length transfer copies nothing and is trivially in range. */
    if (len == 0) {
        return 0;
    }

    /* Guard the end computation against overflow. */
    uint64_t end = addr + (uint64_t) len;
    if (end < addr) {
        return -ERANGE;
    }

    /*
     * The whole transfer must lie within ONE valid window.  HBM and DDR are each
     * a single contiguous range here (bank structure is internal to the model);
     * the reconfiguration region is intentionally NOT accepted (T10's VBIN
     * path).  A transfer that straddles a window boundary is out of range.
     */
    if (addr >= SLASH_HBM_BASE && end <= SLASH_HBM_END) {
        return 0;
    }
    if (addr >= SLASH_DDR_BASE && end <= SLASH_DDR_END) {
        return 0;
    }

    return -ERANGE;
}

int emu_qdma_is_reconfig_write(uint64_t addr, size_t len)
{
    /* A zero-length write carries no VBIN: not a reconfiguration. */
    if (len == 0) {
        return 0;
    }

    /* Guard the end computation against overflow. */
    uint64_t end = addr + (uint64_t) len;
    if (end < addr) {
        return 0;
    }

    /* The whole write must lie within the reconfiguration region. */
    if (addr >= SLASH_RECONFIG_BASE && end <= SLASH_RECONFIG_END) {
        return 1;
    }

    return 0;
}

int emu_qdma_check_qpair_add(uint32_t mode, uint32_t dir_mask,
                             uint32_t h2c_ring, uint32_t c2h_ring,
                             uint32_t cmpt_ring)
{
    /* Streaming mode is deferred (step-1 decision): MM only. */
    if (mode == EMU_QDMA_MODE_ST) {
        return -EOPNOTSUPP;
    }
    if (mode != EMU_QDMA_MODE_MM) {
        return -EINVAL;
    }

    /* CMPT direction is not supported. */
    if (dir_mask & EMU_QDMA_DIR_CMPT) {
        return -EOPNOTSUPP;
    }

    /* Only H2C/C2H bits are meaningful; at least one direction is required. */
    if (dir_mask & ~(EMU_QDMA_DIR_H2C | EMU_QDMA_DIR_C2H)) {
        return -EINVAL;
    }
    if ((dir_mask & (EMU_QDMA_DIR_H2C | EMU_QDMA_DIR_C2H)) == 0) {
        return -EINVAL;
    }

    /* Ring sizes are CSR table indices 0..15. */
    if (h2c_ring > EMU_QDMA_RING_IDX_MAX || c2h_ring > EMU_QDMA_RING_IDX_MAX ||
        cmpt_ring > EMU_QDMA_RING_IDX_MAX) {
        return -EINVAL;
    }

    return 0;
}

/* ================================================================== */
/* qpair<Q> data file: backing + read/write hooks                     */
/* ================================================================== */

/*
 * Per-qpair backing.  The qpair owns no device memory of its own -- the store is
 * per-device and shared -- so it only needs a borrowed pointer to that store
 * plus the bits needed to gate ops on the resource's liveness:
 *
 *   - `store`: the device's sparse store (borrowed; owned by the qdma dir node);
 *   - `tree`:  for emu_resource_check's locking;
 *   - `res`:   this qpair's resource (borrowed; the inode ref is held by the
 *              node, so the pointer stays valid for as long as the node lives).
 *
 * The resource teardown callback NULLs `res->backing` and the spine frees the
 * resource only when both refs drop; this backing (the qpair backing) is freed
 * by the node's destroy hook, which always runs before/with the inode-ref drop.
 */
struct emu_qpair_backing {
    struct emu_qdma_store *store; /* borrowed (device store) */
    struct emu_node_tree *tree;   /* borrowed */
    struct emu_resource *res;     /* borrowed */
    uint32_t qid;
};

/*
 * The qpair "object" the resource frees when its last ref drops.  Distinct from
 * the node backing: the node backing is freed by the node destroy hook, but the
 * resource may outlive the node (registry ref still held during the window
 * between revoke teardown and the holder's close).  Today the qpair has no heavy
 * device-side state (the store is per-device), so the resource backing is just a
 * small tag; it exists so the resource's free_backing has something well-defined
 * to own and so T10 can hang per-qpair model state off it.
 */
struct emu_qpair_resource {
    uint32_t qid;
};

static void emu_qpair_resource_free(void *backing)
{
    free(backing);
}

/* Resource teardown: "stop the queue + free the QID", idempotent (the spine
 * guarantees it runs at most once).  Nothing external to release in the
 * in-memory model; the QID is freed implicitly when the resource leaves the
 * registry.  T10 will stop the per-qpair model state here. */
static void emu_qpair_teardown(struct emu_resource *res, void *backing)
{
    (void) backing;
    LOG(LOG_DEBUG, "qpair %u torn down", res->id);
}

/* getattr size hook: a qpair file has no meaningful "size" (it is an address
 * window, not a byte file).  Report 0 -- direct_io reads/writes carry their own
 * length and offset and never consult st_size. */
static off_t qpair_size_hook(const struct emu_node *node, void *backing)
{
    (void) node;
    (void) backing;
    return 0;
}

/*
 * Shared liveness gate for the qpair data hooks.  The spine's emu_node_pread/
 * pwrite already checked node->live under the lock before calling us, but the
 * resource is the authoritative liveness token (it can be torn down via the
 * registry even in paths the node flag does not cover), so consult it too.
 * Both checks run under the same tree lock the spine holds across the hook.
 */
static int qpair_live(struct emu_qpair_backing *b)
{
    /* The read/write hooks run under the tree lock the spine already holds, so
     * use the lock-held variant (the public one would deadlock). */
    return emu_resource_check_locked(b->tree, b->res);
}

/* pread hook: serve a validated MM read from device memory. */
static ssize_t qpair_read(const struct emu_node *node, void *backing, char *buf,
                          size_t size, off_t off)
{
    (void) node;

    struct emu_qpair_backing *b = backing;

    int rc = qpair_live(b);
    if (rc != 0) {
        return rc;
    }

    uint64_t addr = (uint64_t) off; /* the file offset IS the device address */
    rc = emu_qdma_check_range(addr, size);
    if (rc != 0) {
        return rc;
    }

    /*
     * SIM bridge seam (T10): ask the model first.  rc == 0 => model answered;
     * rc > 0 => fall back to the sparse store (an in-range read the model cannot
     * answer must still return defined bytes, never -EIO); rc < 0 => transport
     * failure (-ENODEV).
     */
    if (b->store->backend != NULL && b->store->backend->fetch != NULL) {
        rc = b->store->backend->fetch(b->store->backend->ctx, addr, buf, size);
        if (rc < 0) {
            return -ENODEV;
        }
        if (rc == 0) {
            return (ssize_t) size;
        }
        /* rc > 0: fall through to the sparse store. */
    }

    store_read(b->store, addr, buf, size);

    return (ssize_t) size;
}

/* pwrite hook: store a validated MM write into device memory. */
static ssize_t qpair_write(const struct emu_node *node, void *backing,
                           const char *buf, size_t size, off_t off)
{
    (void) node;

    struct emu_qpair_backing *b = backing;

    int rc = qpair_live(b);
    if (rc != 0) {
        return rc;
    }

    uint64_t addr = (uint64_t) off;

    /*
     * Reconfiguration seam (T10): a write whose whole range lies in the reconfig
     * region is (part of) a VBIN delivery, NOT an ordinary memory transfer.
     * Detect it BEFORE the HBM/DDR range check (which still rejects the region
     * with -ERANGE for reads and for the no-handler case), and route the bytes to
     * the reconfiguration handler, which reassembles chunks the kernel split out
     * of a large VBIN write.  A reconfig write with no handler attached is
     * rejected with -ERANGE, exactly as the region would be without T10 -- the
     * VBIN has nowhere to go.
     */
    if (emu_qdma_is_reconfig_write(addr, size)) {
        if (b->store->reconfig == NULL) {
            return -ERANGE;
        }
        rc = b->store->reconfig(b->store->reconfig_ctx, addr, buf, size);
        if (rc != 0) {
            return rc; /* negative errno from the handler (malformed VBIN, ...) */
        }
        return (ssize_t) size;
    }

    rc = emu_qdma_check_range(addr, size);
    if (rc != 0) {
        return rc;
    }

    /*
     * Keep the sparse store current even when a SIM backend is attached, so an
     * in-range read the model cannot answer can fall back to it (never -EIO).
     */
    rc = store_write(b->store, addr, buf, size);
    if (rc != 0) {
        return rc; /* -ENOMEM */
    }

    /* SIM bridge seam (T10): forward the validated transfer to the model. */
    if (b->store->backend != NULL && b->store->backend->populate != NULL) {
        if (b->store->backend->populate(b->store->backend->ctx, addr, buf,
                                        size) < 0) {
            return -ENODEV;
        }
    }

    return (ssize_t) size;
}

/* destroy hook: free the qpair node backing.  The device store is owned by the
 * qdma dir node, not by us; the resource is freed by the spine when both refs
 * drop. */
static void qpair_destroy(struct emu_node *node, void *backing)
{
    (void) node;
    free(backing);
}

static const struct emu_node_ops emu_qpair_ops = {
    .size = qpair_size_hook,
    .read = qpair_read,
    .write = qpair_write,
    .destroy = qpair_destroy,
};

/* ================================================================== */
/* qdma/ directory: backing + QPAIR_ADD ioctl                         */
/* ================================================================== */

/*
 * Backing for the qdma/ directory node.  Owns the per-device sparse memory store
 * and tracks QID allocation.  The directory node owns this backing (freed via
 * qdma_dir_destroy).
 */
struct emu_qdma_dir_backing {
    struct emu_device *dev;     /* borrowed (the owning device) */
    struct emu_qdma_store store;
    uint32_t next_qid;          /* monotonic allocation hint */
};

/*
 * Allocate a unique per-device QID.  The architecture requires only that the
 * daemon hands out a unique id (not a particular one), so this is a monotonic
 * counter: it returns `next_qid` and advances, which guarantees uniqueness for
 * the entire practical lifetime of a device (a u32 never wraps in any real run).
 * "Free" means no resource with that id is currently registered.
 *
 * Two non-monotonic cases are handled so the invariant holds even at the
 * theoretical edge: if `next_qid` ever collides with a still-registered id
 * (only reachable after a full u32 wrap), we fall back to a linear scan for the
 * lowest free id; we do NOT otherwise reuse freed ids below the counter (so the
 * common allocation is strictly increasing, not smallest-free).  QID-space
 * exhaustion (4 billion concurrently-live qpairs) is impossible in practice and
 * returns an error rather than blocking.
 */
static int alloc_qid(struct emu_qdma_dir_backing *d, uint32_t *out)
{
    /* Runs inside the QPAIR_ADD ioctl hook (tree lock already held), so use the
     * lock-held registry probe. */
    /* Fast path: hand out the monotonic counter if it is genuinely free. */
    if (emu_device_find_resource_locked(d->dev, d->next_qid) == NULL) {
        *out = d->next_qid;
        d->next_qid++;
        return 0;
    }

    /* Slow path (counter collided with a reused id): linear search from 0. */
    for (uint32_t q = 0; q < UINT32_MAX; q++) {
        if (emu_device_find_resource_locked(d->dev, q) == NULL) {
            *out = q;
            if (q + 1 > d->next_qid) {
                d->next_qid = q + 1;
            }
            return 0;
        }
    }

    return -1; /* QID space exhausted (4 billion live qpairs -- impossible) */
}

/*
 * Create one qpair<Q>: create the file node with a qpair backing, register a
 * refcounted resource, attach that resource to the node, and mark it direct_io.
 * Assumes parameters already validated and `qid` already allocated.
 *
 * Ordering is chosen so every fallible step can be unwound with a public spine
 * primitive that takes no inode/registry ref we cannot release locally:
 *
 *   1. create the node           -> unwind: emu_node_unlink_locked (destroys it;
 *                                   the node's lookup_count is 0, just made)
 *   2. register the resource     -> unwind: emu_node_unlink_locked
 *   3. attach resource to node   -> infallible for a fresh node (a brand-new
 *                                   node has no resource); failure can only mean
 *                                   a programming error, handled defensively.
 *
 * Because step 3 cannot fail for a fresh node, no path leaves a registered
 * resource without an inode ref; the cooperative/forced teardown machinery then
 * owns the qpair's whole lifetime.
 *
 * Runs inside the QPAIR_ADD ioctl hook, which the spine invokes with the tree
 * lock HELD, so every spine call here is the lock-held (_locked) variant; the
 * public re-locking ones would deadlock the non-recursive mutex.
 */
static int create_qpair(struct emu_qdma_dir_backing *d, uint32_t qid)
{
    struct emu_device *dev = d->dev;
    struct emu_node_tree *tree = dev->tree;

    /* The node backing: borrowed pointers into the device store + this res
     * (filled in once the resource exists). */
    _cleanup_(cleanup_free)
    struct emu_qpair_backing *nbacking = calloc(1, sizeof(*nbacking));
    PROPAGATE_ERROR_NULL_LOG(nbacking, LOG_ERR,
                             "Failed to allocate qpair %u node backing", qid);
    *nbacking = (struct emu_qpair_backing) {
        .store = &d->store,
        .tree = tree,
        .res = NULL, /* set after registration */
        .qid = qid,
    };

    /* The resource backing: a small tag the spine frees when both refs drop. */
    _cleanup_(cleanup_free)
    struct emu_qpair_resource *rbacking = calloc(1, sizeof(*rbacking));
    PROPAGATE_ERROR_NULL_LOG(rbacking, LOG_ERR,
                             "Failed to allocate qpair %u resource", qid);
    rbacking->qid = qid;

    char name[32];
    int n = snprintf(name, sizeof(name), "qpair%u", qid);
    if (n < 0 || (size_t) n >= sizeof(name)) {
        LOG(LOG_ERR, "qpair name overflow for qid %u", qid);
        return -1;
    }

    /* 1. Create the node (takes ownership of the node backing on success). */
    struct emu_node *node = NULL;
    if (emu_node_create_child_locked(tree, dev->qdma, name, EMU_NODE_FILE, 0600,
                                     &emu_qpair_ops, nbacking, &node) == -1) {
        LOG(LOG_ERR, "Failed to create %s node", name);
        return -1;
    }
    struct emu_qpair_backing *nbacking_ref = nbacking;
    nbacking = NULL; /* ownership transferred to the node */

    /* 2. Register the resource (takes ownership of the resource backing). */
    struct emu_resource *res = NULL;
    if (emu_device_register_resource_locked(dev, qid, emu_qpair_teardown,
                                            emu_qpair_resource_free, rbacking,
                                            &res) == -1) {
        LOG(LOG_ERR, "Failed to register qpair %u resource", qid);
        emu_node_unlink_locked(tree, node); /* destroys the just-created node */
        return -1;
    }
    rbacking = NULL; /* ownership transferred to the resource */
    nbacking_ref->res = res;

    /* 3. Attach the resource to the node (the inode reference).  Infallible for
     * a fresh node; on the impossible failure, unlink to keep things consistent
     * (the registry ref is then the resource's only ref and is reaped at device
     * teardown). */
    if (emu_node_attach_resource_locked(tree, node, res) == -1) {
        LOG(LOG_ERR, "Failed to attach resource to %s", name);
        emu_node_unlink_locked(tree, node);
        return -1;
    }

    /* Memory transfers are unbuffered: open with direct_io so the kernel
     * forwards each pread/pwrite verbatim (exact size + offset) rather than
     * synthesizing page-sized, page-cached transfers. */
    emu_node_set_direct_io_locked(tree, node);

    /* qpair<Q> files are user-unlinkable: the VRTD delete-on-last-close pattern
     * (ADD -> open -> unlink while open) rides the FUSE unlink op, which is now
     * opt-in.  info / bar<M> / hotplug are left non-unlinkable. */
    emu_node_set_unlinkable_locked(tree, node);

    return 0;
}

/* QPAIR_ADD ioctl handler on the qdma/ directory node. */
static int qdma_dir_ioctl(struct emu_node *node, void *backing,
                          unsigned int cmd, const void *in, size_t in_size,
                          void *out, size_t out_size)
{
    (void) node;

    struct emu_qdma_dir_backing *d = backing;

    if (cmd != SLASH_ABI_QDMA_IOCTL_QPAIR_ADD) {
        return -ENOTTY;
    }

    /*
     * The caller's struct must carry at least the input prefix we read (through
     * cmpt_ring_sz) and the reply must have room for the qid we write.  Both
     * buffers are the FUSE layer's fixed-size bounce copies; the qid field sits
     * at the very end of the struct, so requiring the full struct size for both
     * directions is the simplest correct check for this fixed _IOWR layout.
     */
    if (in_size < sizeof(struct slash_abi_qdma_qpair_add) ||
        out_size < sizeof(struct slash_abi_qdma_qpair_add)) {
        return -EINVAL;
    }

    const struct slash_abi_qdma_qpair_add *req = in;

    int rc = emu_qdma_check_qpair_add(req->mode, req->dir_mask, req->h2c_ring_sz,
                                      req->c2h_ring_sz, req->cmpt_ring_sz);
    if (rc != 0) {
        return rc;
    }

    uint32_t qid = 0;
    if (alloc_qid(d, &qid) == -1) {
        return -ENOSPC;
    }

    if (create_qpair(d, qid) == -1) {
        return -ENOMEM;
    }

    /* Write the allocated QID back into the reply struct (the out buffer was
     * seeded from the in buffer by the FUSE layer, so the input fields are
     * preserved and we only set the [out] qid). */
    struct slash_abi_qdma_qpair_add *resp = out;
    resp->qid = qid;

    return 0;
}

/* destroy hook for the qdma/ directory node: free the per-device sparse store
 * (all pages) and the dir backing. */
static void qdma_dir_destroy(struct emu_node *node, void *backing)
{
    (void) node;

    struct emu_qdma_dir_backing *d = backing;
    if (d == NULL) {
        return;
    }

    emu_qdma_page_array_free(&d->store.pages);
    free(d);
}

static const struct emu_node_ops emu_qdma_dir_ops = {
    .ioctl = qdma_dir_ioctl,
    .destroy = qdma_dir_destroy,
};

int emu_qdma_attach(struct emu_device *dev)
{
    if (dev == NULL || dev->qdma == NULL || dev->tree == NULL) {
        return -1;
    }

    _cleanup_(cleanup_free)
    struct emu_qdma_dir_backing *backing = calloc(1, sizeof(*backing));
    PROPAGATE_ERROR_NULL_LOG(backing, LOG_ERR,
                             "Failed to allocate qdma backing for '%s'",
                             dev->bdf);

    *backing = (struct emu_qdma_dir_backing) {
        .dev = dev,
        .store = {
            .pages = emu_qdma_page_array_init(),
            .backend = NULL, /* T10 attaches the SIM bridge here. */
        },
        .next_qid = 0,
    };

    /* Attach the ioctl/destroy vtable + the store backing onto the qdma/ dir
     * node, which emu_node_tree_add_device already created.  From here the node
     * owns the backing and frees it (store included) via qdma_dir_destroy. */
    if (emu_node_set_ops(dev->tree, dev->qdma, &emu_qdma_dir_ops, backing)
        == -1) {
        LOG(LOG_ERR, "Failed to attach qdma ops for '%s'", dev->bdf);
        return -1;
    }
    backing = NULL; /* ownership transferred to the node */

    return 0;
}

int emu_qdma_set_mem_backend(struct emu_device *dev,
                             const struct emu_qdma_mem_backend *backend)
{
    if (dev == NULL || dev->qdma == NULL) {
        return -1;
    }

    /* The qdma/ dir node owns the store backing; reach it through the node the
     * endpoint attached its ops/backing onto.  NULL means the endpoint was not
     * attached (or already torn down) -- nothing to wire a backend onto. */
    struct emu_qdma_dir_backing *d = dev->qdma->backing;
    if (d == NULL) {
        return -1;
    }

    d->store.backend = backend;

    return 0;
}

int emu_qdma_set_reconfig_handler(struct emu_device *dev,
                                  emu_qdma_reconfig_fn handler, void *ctx)
{
    if (dev == NULL || dev->qdma == NULL) {
        return -1;
    }

    struct emu_qdma_dir_backing *d = dev->qdma->backing;
    if (d == NULL) {
        return -1;
    }

    d->store.reconfig = handler;
    d->store.reconfig_ctx = ctx;

    return 0;
}
