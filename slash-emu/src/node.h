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
 * @file node.h
 * @brief The spine of the slash-emu FUSE daemon: node tree, per-device registry,
 *        refcounted resources, and the revocation state machine.
 *
 * This module owns the data model that the FUSE layer (@ref fs.h) and the four
 * endpoint tasks (info / bars / qdma / hotplug) build on.  It is written to be
 * unit-testable in isolation: it never touches a @c fuse_session or a
 * @c fuse_req directly.  The one effect it must push to the kernel -- dentry
 * invalidation on revocation -- is delegated through a small @ref emu_notifier
 * callback that the FUSE layer supplies (and tests stub).
 *
 * @section model Three layers
 *
 *   1. @b Node @b tree (@ref emu_node, @ref emu_node_tree).  A generic tree of
 *      inodes mirroring the on-disk layout: a root, one @c <BDF>/ directory per
 *      accelerator, and @c bars/ + @c qdma/ subdirectories.  Each node carries a
 *      type, a parent/children topology, a per-node ops hook + opaque backing
 *      (so endpoints attach @c info, @c bar<M>, @c qpair<Q> without reworking the
 *      tree), and a @em live flag driving revocation.  An inode->node table makes
 *      lookup fast.  FUSE lookup/getattr/readdir dispatch against this model.
 *
 *   2. @b Per-device @b registry (@ref emu_device).  One per accelerator,
 *      tracking the live communication resources of that device.  Crucially it
 *      holds @em nameless qpairs: a qpair is unlinked from @c qdma/ while still
 *      open (delete-on-last-close), so it cannot be found by walking the tree.
 *      Forced device removal (@ref emu_device_revoke) reaches every live resource
 *      through this registry.
 *
 *   3. @b Refcounted @b resource (@ref emu_resource).  A primitive whose lifetime
 *      is decoupled from the inode.  It carries two references -- one held by the
 *      registry, one by the inode -- and is freed only when @em both drop.  Its
 *      teardown (stop the queue, free the QID) is @em idempotent and has two
 *      triggers: cooperative inode eviction (last close of an undisturbed
 *      resource) and forced device removal.  The first trigger does the work; the
 *      second is a no-op.  The qpair object (T8) is modelled as the @c backing of
 *      such a resource.
 *
 * @section revocation Revocation semantics (enforced by the conformance suite)
 *
 * On forced removal (@ref emu_device_revoke), for the affected device:
 *   - resources are eagerly torn down (queues stopped, QIDs freed);
 *   - every open handle is marked dead, so any subsequent op returns @c -ENODEV
 *     (@ref emu_handle_check / @ref emu_node_is_live);
 *   - the endpoint names are invalidated via the notifier
 *     (@c fuse_lowlevel_notify_delete / @c notify_inval_entry), so the dentry
 *     cache forgets them and a new @c lookup returns @c -ENOENT;
 *   - @c close always succeeds and remains the holder's responsibility;
 *     @c release is idempotent.
 *
 * @section locking Locking model
 *
 * The whole spine is guarded by a single per-tree mutex (@c emu_node_tree::lock).
 * The FUSE session is single-threaded today, but the data model is built to be
 * safe for a future multi-threaded session:
 *
 *   - All public tree / device / registry mutators and lookups take the lock.
 *   - @ref emu_resource refcounts are maintained under the same lock (so a
 *     get/put never races a teardown decision); they are plain integers, not
 *     atomics, precisely because every touch is already serialised by the lock.
 *   - The lock is @em never held across a notifier call.  Name invalidation can
 *     re-enter the filesystem in the kernel and must not deadlock against an op;
 *     @ref emu_device_revoke therefore collects the names to invalidate under the
 *     lock, drops it, then fires the notifier.
 *   - Functions whose name ends in @c _locked assume the caller already holds the
 *     lock; the unsuffixed public entry points acquire it.
 */

#ifndef SLASH_EMU_NODE_H
#define SLASH_EMU_NODE_H

#include <pthread.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <sys/stat.h>
#include <sys/types.h>

#include "array.h"

/** @brief Root inode number (matches libfuse's @c FUSE_ROOT_ID). */
#define EMU_ROOT_INO 1

/**
 * @brief Inode number type.
 *
 * Kept independent of @c <fuse_lowlevel.h> so this module is testable without a
 * FUSE session.  Layout-compatible with libfuse's @c fuse_ino_t (both
 * @c uint64_t), so the FUSE layer passes inodes through unchanged.
 */
typedef uint64_t emu_ino_t;

/* Forward declarations. */
struct emu_node;
struct emu_node_tree;
struct emu_device;
struct emu_resource;

/** @brief Non-owning array of node pointers (children lists / scratch). */
DECLARE_ARRAY(emu_node_ref_array, struct emu_node *)

/** @brief Non-owning array of device pointers. */
DECLARE_ARRAY(emu_device_ref_array, struct emu_device *)

/** @brief Non-owning array of resource pointers (registry storage). */
DECLARE_ARRAY(emu_resource_ref_array, struct emu_resource *)

/*
 * The tree's owning node/device arrays are declared after the structs so their
 * cleanup functions are visible.  See below (emu_node_owning_array,
 * emu_device_owning_array).
 */

/**
 * @brief Node kind.  Mirrors the only two file types the ABI exposes.
 */
enum emu_node_type {
    EMU_NODE_DIR,   /**< Directory (root, @c <BDF>, @c bars, @c qdma). */
    EMU_NODE_FILE,  /**< Regular file (@c info, @c bar<M>, @c qpair<Q>, ...). */
};

/**
 * @brief Per-node operation hook (filled in by endpoint tasks T6-T9).
 *
 * The spine provides the topology; the endpoints provide behaviour.  All hooks
 * are optional; a NULL hook means "the spine's default applies".  Hooks are
 * invoked with the tree lock @em held unless documented otherwise.
 */
struct emu_node_ops {
    /**
     * @brief Report the size of a file node, in bytes (optional).
     * @param node The node being queried.
     * @param backing The node's @c backing pointer.
     * @return File size in bytes; ignored for directories.
     */
    off_t (*size)(const struct emu_node *node, void *backing);

    /**
     * @brief Release the node's @c backing when the node is destroyed (optional).
     *
     * Called while the tree lock is @em held, exactly once, when the node is
     * finally freed.  For resource-backed nodes the spine drops the inode-side
     * reference on the @ref emu_resource itself (see @ref emu_node_attach_resource);
     * this hook is for any @em additional backing an endpoint owns.
     *
     * @param node The node being destroyed.
     * @param backing The node's @c backing pointer.
     */
    void (*destroy)(struct emu_node *node, void *backing);
};

/**
 * @brief A single inode in the emulated tree.
 *
 * Nodes are owned by their @ref emu_node_tree and addressed by @c ino.  A node
 * may be @em unlinked (removed from its parent's child list, name invalidated)
 * while still present in the inode table because the kernel still holds lookup
 * references; it is destroyed only when its lookup count drops to zero.
 */
struct emu_node {
    /** @brief Inode number (unique, stable for the node's lifetime). */
    emu_ino_t ino;

    /** @brief Entry name within the parent directory (heap, owning). */
    char *name; /* owning */

    /** @brief Node kind. */
    enum emu_node_type type;

    /** @brief POSIX permission bits (the type bits are OR'd in by the FS). */
    mode_t mode;

    /** @brief Parent node, or NULL for the root (non-owning). */
    struct emu_node *parent; /* non-owning */

    /** @brief Child nodes (non-owning refs; the tree table owns the nodes). */
    struct emu_node_ref_array children;

    /** @brief Device this node belongs to, or NULL for the root (non-owning). */
    struct emu_device *device; /* non-owning */

    /** @brief Per-node behaviour hook, or NULL (non-owning, static). */
    const struct emu_node_ops *ops; /* non-owning */

    /** @brief Endpoint-private backing; ownership defined by @c ops->destroy. */
    void *backing;

    /** @brief Resource this node holds an inode-ref on, or NULL (owns 1 ref). */
    struct emu_resource *resource;

    /**
     * @brief Kernel lookup count: the number of @c fuse_reply_entry replies for
     *        this node not yet balanced by a @c forget.  The node is destroyed
     *        when this reaches zero @em and the node has been unlinked.
     */
    uint64_t lookup_count;

    /** @brief False once the endpoint has been revoked (forced removal). */
    bool live;

    /** @brief True once unlinked from its parent (name no longer resolvable). */
    bool unlinked;
};

/**
 * @brief Resource teardown callback (supplied by the resource creator, e.g. T8).
 *
 * Invoked exactly once, the first time the resource is torn down (cooperative or
 * forced), with the tree lock @em held.  It must stop the underlying queue and
 * release any external identifier (the QID).  It must @em not free the resource
 * object itself or its backing -- that happens when the last refcount drops.
 *
 * @param res     The resource being torn down.
 * @param backing The resource's @c backing pointer.
 */
typedef void (*emu_resource_teardown_fn)(struct emu_resource *res, void *backing);

/**
 * @brief Resource backing-free callback (supplied by the creator, e.g. T8).
 *
 * Invoked exactly once, with the tree lock @em held, when the resource's last
 * reference drops and the resource object is about to be freed.  Frees the
 * @c backing (the qpair object and its buffers).
 *
 * @param backing The resource's @c backing pointer.
 */
typedef void (*emu_resource_free_fn)(void *backing);

/**
 * @brief A reference-counted resource whose lifetime is decoupled from any inode.
 *
 * Models the lifetime of a QDMA qpair (T8) but is a reusable primitive.  Two
 * references exist by construction:
 *   - the @em registry reference, dropped by @ref emu_device_unregister or during
 *     forced @ref emu_device_revoke;
 *   - the @em inode reference, dropped when the backing node is destroyed.
 * The teardown callback fires on the first of {cooperative eviction, forced
 * removal}; the resource object is freed when @em both references are gone.
 */
struct emu_resource {
    /** @brief Owning device's registry this resource is registered in. */
    struct emu_device *device; /* non-owning */

    /** @brief Stable per-device identifier (e.g. the QID). */
    uint32_t id;

    /** @brief Total live references (registry + inode); freed when it hits 0. */
    unsigned int refcount;

    /** @brief True once teardown has run (makes the second trigger a no-op). */
    bool torn_down;

    /** @brief False once revoked/torn-down: ops on open handles return -ENODEV. */
    bool live;

    /** @brief Teardown callback, or NULL. */
    emu_resource_teardown_fn teardown;

    /** @brief Backing-free callback, or NULL. */
    emu_resource_free_fn free_backing;

    /** @brief Endpoint-private backing (the qpair object), or NULL. */
    void *backing;
};

/**
 * @brief A per-accelerator device: its directory subtree and resource registry.
 *
 * The registry is the authoritative list of live resources for forced teardown,
 * including nameless (unlinked-while-open) qpairs that the tree can no longer
 * reach.
 */
struct emu_device {
    /** @brief Owning tree (non-owning back-pointer, for locking). */
    struct emu_node_tree *tree; /* non-owning */

    /** @brief Normalized board-level BDF "DDDD:BB:DD" (NUL-terminated). */
    char bdf[16];

    /** @brief The @c <BDF>/ directory node (non-owning; owned by the tree). */
    struct emu_node *dir; /* non-owning */

    /** @brief The @c <BDF>/bars/ directory node (non-owning). */
    struct emu_node *bars; /* non-owning */

    /** @brief The @c <BDF>/qdma/ directory node (non-owning). */
    struct emu_node *qdma; /* non-owning */

    /** @brief Live resources registered to this device (non-owning refs). */
    struct emu_resource_ref_array registry;

    /** @brief False once the device has been revoked (forced removal). */
    bool live;
};

/**
 * @brief Notifier callbacks the FUSE layer supplies for kernel dentry control.
 *
 * Lets @c node.c invalidate names on revocation without depending on
 * @c <fuse_lowlevel.h>.  Tests pass a stub that records calls.  Both callbacks
 * are invoked with the tree lock @em not held.
 */
struct emu_notifier {
    /**
     * @brief Invalidate the dentry @p name under directory inode @p parent.
     * @param ctx    Opaque context (the FUSE session wrapper).
     * @param parent Parent directory inode.
     * @param child  Child inode being deleted (for @c notify_delete).
     * @param name   Entry name to invalidate.
     */
    void (*notify_delete)(void *ctx, emu_ino_t parent, emu_ino_t child,
                          const char *name);

    /** @brief Opaque context passed back to the callbacks. */
    void *ctx;
};

/**
 * @brief Free a node's owned shell (name + children-array storage), then itself.
 *
 * Frees only the node's own allocations -- not its children (the tree owns those
 * separately) and not its resource reference (dropped explicitly during ordered
 * teardown).  Exposed only so the owning array below can name it.
 *
 * @param node The node to free (may be NULL).
 */
void emu_node_free_shell(struct emu_node *node);

/**
 * @brief Free a device's owned storage (its registry array), then itself.
 * @param dev The device to free (may be NULL).
 */
void emu_device_free_shell(struct emu_device *dev);

/** @brief Owning array of nodes (frees each shell on destruction). */
DECLARE_OWNING_PTR_ARRAY(emu_node_owning_array, struct emu_node *,
                         emu_node_free_shell)

/** @brief Owning array of devices (frees each shell on destruction). */
DECLARE_OWNING_PTR_ARRAY(emu_device_owning_array, struct emu_device *,
                         emu_device_free_shell)

/**
 * @brief The inode table + topology root + lock + notifier.
 *
 * Owns every @ref emu_node and every @ref emu_device.  Opaque to callers other
 * than via the API below; the FUSE layer holds one and threads it through ops.
 */
struct emu_node_tree {
    /** @brief All live nodes, indexed by position (owning). */
    struct emu_node_owning_array nodes;

    /** @brief All devices (owning). */
    struct emu_device_owning_array devices;

    /** @brief The root directory node (non-owning; element of @c nodes). */
    struct emu_node *root; /* non-owning */

    /** @brief Next inode number to hand out (monotonic). */
    emu_ino_t next_ino;

    /** @brief Guards the whole structure (see @ref locking). */
    pthread_mutex_t lock;

    /** @brief Kernel dentry notifier (borrowed). */
    struct emu_notifier notifier; /* non-owning ctx */
};

/* ------------------------------------------------------------------ */
/* Lifecycle                                                          */
/* ------------------------------------------------------------------ */

/**
 * @brief Allocate a node tree with just the root directory.
 * @param[out] treep  Receives the heap-allocated tree (caller owns it; free with
 *                    @ref cleanup_node_tree).
 * @param notifier    Notifier callbacks (copied by value; @c ctx borrowed).  May
 *                    be all-NULL for tests that do not exercise invalidation.
 * @return 0 on success, -1 on error.
 */
int emu_node_tree_new(struct emu_node_tree **treep,
                      const struct emu_notifier *notifier);

/**
 * @brief Destroy a node tree, freeing all nodes, devices, and resources.
 * @param tree The tree to destroy (may be NULL).
 */
void cleanup_node_tree(struct emu_node_tree *tree);

/**
 * @brief Cleanup helper for @c __attribute__((cleanup)).
 * @param treep Address of a @c struct @c emu_node_tree pointer.
 */
static inline
void cleanup_node_treep(struct emu_node_tree **treep)
{
    if (treep == NULL) {
        return;
    }

    cleanup_node_tree(*treep);

    *treep = NULL;
}

/* ------------------------------------------------------------------ */
/* Tree construction (used at startup and by RESCAN/hotplug add)       */
/* ------------------------------------------------------------------ */

/**
 * @brief Materialize the @c <BDF>/ + @c bars/ + @c qdma/ subtree for one device.
 *
 * Idempotent at the BDF level: if a @em live device with @p bdf already exists,
 * it is returned without creating a duplicate.  Endpoints (T6-T9) then attach
 * their files/dirs under @c dev->bars / @c dev->qdma and the @c info file under
 * @c dev->dir.
 *
 * @param tree     The tree.
 * @param bdf      Normalized board-level BDF "DDDD:BB:DD".
 * @param[out] devp Receives the (new or existing) device (non-owning).  May be
 *                  NULL if the caller does not need it.
 * @return 0 on success, -1 on error.
 */
int emu_node_tree_add_device(struct emu_node_tree *tree, const char *bdf,
                             struct emu_device **devp);

/**
 * @brief Find a live device by normalized BDF.
 * @param tree The tree.
 * @param bdf  Normalized board-level BDF.
 * @return The device (non-owning), or NULL if absent or already revoked.
 */
struct emu_device *emu_node_tree_find_device(struct emu_node_tree *tree,
                                             const char *bdf);

/**
 * @brief Create a child node under @p parent and register it in the inode table.
 *
 * For use by endpoint tasks attaching @c info / @c bar<M> / @c qpair<Q>.  The
 * new node is linked into @p parent's children and inherits @p parent's device.
 *
 * @param tree     The tree.
 * @param parent   Parent directory node.
 * @param name     Entry name (copied).
 * @param type     Node kind.
 * @param mode     POSIX permission bits.
 * @param ops      Per-node ops hook (borrowed, may be NULL).
 * @param backing  Endpoint backing (ownership per @p ops->destroy, may be NULL).
 * @param[out] nodep Receives the new node (non-owning).  May be NULL.
 * @return 0 on success, -1 on error.
 */
int emu_node_create_child(struct emu_node_tree *tree, struct emu_node *parent,
                          const char *name, enum emu_node_type type, mode_t mode,
                          const struct emu_node_ops *ops, void *backing,
                          struct emu_node **nodep);

/**
 * @brief Unlink a single node from its parent (the delete-on-last-close primitive).
 *
 * Removes @p node from its parent's directory listing so it is no longer
 * resolvable by name, then reaps it immediately if the kernel holds no lookups.
 * If the kernel still references it (it is open), the node survives as a nameless
 * orphan and is destroyed on the final @ref emu_node_forget -- the cooperative
 * teardown trigger.  This is exactly the QDMA "unlink the qpair<Q> file while its
 * fd is open" pattern (T8): the node becomes nameless but stays reachable via the
 * per-device registry until torn down.
 *
 * The node stays @em live (ops still succeed) -- unlinking is not revocation; a
 * normally-unlinked qpair keeps working until its fd is closed.  This is distinct
 * from @ref emu_device_revoke, which additionally marks the node dead (@c -ENODEV)
 * and is the forced trigger.
 *
 * Does not fire the kernel-dentry notifier: an explicit @c unlink/@c rmdir issued
 * by the holder already updates the kernel's view, unlike a daemon-initiated
 * revocation.
 *
 * @param tree The tree.
 * @param node The node to unlink (must belong to @p tree).
 */
void emu_node_unlink(struct emu_node_tree *tree, struct emu_node *node);

/* ------------------------------------------------------------------ */
/* FUSE op support (called from fs.c with inode numbers)              */
/* ------------------------------------------------------------------ */

/**
 * @brief Resolve an inode number to its node.
 * @param tree The tree.
 * @param ino  Inode number.
 * @return The node (non-owning), or NULL if no such inode.
 */
struct emu_node *emu_node_lookup_ino(struct emu_node_tree *tree, emu_ino_t ino);

/**
 * @brief Resolve a name within a directory to a child node, for FUSE @c lookup.
 *
 * Honours revocation: an unlinked or dead child is @em not resolvable, so a
 * removed endpoint yields "not found" (the caller maps this to @c -ENOENT).  On
 * success the child's kernel lookup count is incremented (balanced by
 * @ref emu_node_forget).
 *
 * @param tree     The tree.
 * @param parent   Parent directory inode.
 * @param name     Entry name to resolve.
 * @param[out] out Receives the resolved child (non-owning) on success.
 * @return 0 on success; -ENOENT if no such name; -ENOTDIR if @p parent is not a
 *         directory; -ESTALE if @p parent does not exist.
 */
int emu_node_lookup_child(struct emu_node_tree *tree, emu_ino_t parent,
                          const char *name, struct emu_node **out);

/**
 * @brief Decrement a node's kernel lookup count, reaping it if appropriate.
 *
 * Called from the FUSE @c forget op.  When the count reaches zero and the node
 * has been unlinked, the node is destroyed (dropping any inode-side resource
 * reference, which may trigger cooperative teardown).
 *
 * @param tree    The tree.
 * @param ino     Inode being forgotten.
 * @param nlookup Amount to subtract from the lookup count.
 */
void emu_node_forget(struct emu_node_tree *tree, emu_ino_t ino, uint64_t nlookup);

/**
 * @brief Snapshot a node's attributes into @p st, for FUSE @c getattr.
 * @param tree     The tree.
 * @param ino      Inode to stat.
 * @param[out] st  Receives the attributes (zeroed then filled).
 * @return 0 on success; -ENOENT if the inode does not exist.
 */
int emu_node_stat(struct emu_node_tree *tree, emu_ino_t ino, struct stat *st);

/**
 * @brief Callback for @ref emu_node_readdir, one per directory entry.
 * @param ctx   Opaque caller context.
 * @param name  Entry name.
 * @param ino   Entry inode.
 * @param type  Entry node kind.
 * @return true to continue enumeration, false to stop early.
 */
typedef bool (*emu_readdir_cb)(void *ctx, const char *name, emu_ino_t ino,
                               enum emu_node_type type);

/**
 * @brief Enumerate a directory's entries (".", "..", then live children).
 *
 * Unlinked/dead children are skipped, so a removed endpoint vanishes from
 * @c readdir.  Enumeration is snapshot-consistent under the lock.
 *
 * @param tree The tree.
 * @param ino  Directory inode to enumerate.
 * @param cb   Per-entry callback.
 * @param ctx  Opaque context passed to @p cb.
 * @return 0 on success; -ENOENT if the inode does not exist; -ENOTDIR if it is
 *         not a directory.
 */
int emu_node_readdir(struct emu_node_tree *tree, emu_ino_t ino,
                     emu_readdir_cb cb, void *ctx);

/**
 * @brief Liveness check for an op arriving on an inode.
 *
 * Endpoints call this at the top of read/write/ioctl.  Returns 0 if the node is
 * live, @c -ENODEV if it has been revoked (forced removal of an already-open
 * fd).  Per the ABI, @c close/release must @em not consult this and always
 * succeed.
 *
 * @param tree The tree.
 * @param ino  Inode the op targets.
 * @return 0 if live; -ENODEV if revoked; -ENOENT if the inode does not exist.
 */
int emu_node_is_live(struct emu_node_tree *tree, emu_ino_t ino);

/* ------------------------------------------------------------------ */
/* Per-device registry + refcounted resources (qpair lifetime, T8)    */
/* ------------------------------------------------------------------ */

/**
 * @brief Create a refcounted resource and register it with a device.
 *
 * The resource starts with a single reference -- the @em registry reference.
 * Attaching it to an inode (@ref emu_node_attach_resource) adds the inode
 * reference.  T8 uses this for a qpair: register it, attach to the @c qpair<Q>
 * node, then (per the VRTD pattern) unlink the node while the fd is open so the
 * qpair becomes nameless but stays reachable via the registry.
 *
 * @param dev          The owning device.
 * @param id           Stable per-device id (the QID).
 * @param teardown     Teardown callback (stop queue, free QID), or NULL.
 * @param free_backing Backing-free callback, or NULL.
 * @param backing      Endpoint backing (the qpair object), or NULL.
 * @param[out] resp    Receives the resource (non-owning) on success.
 * @return 0 on success, -1 on error (e.g. the device has been revoked).
 */
int emu_device_register_resource(struct emu_device *dev, uint32_t id,
                                 emu_resource_teardown_fn teardown,
                                 emu_resource_free_fn free_backing,
                                 void *backing, struct emu_resource **resp);

/**
 * @brief Find a registered resource by id.
 * @param dev The device.
 * @param id  The resource id (QID).
 * @return The resource (non-owning), or NULL if absent.
 */
struct emu_resource *emu_device_find_resource(struct emu_device *dev,
                                              uint32_t id);

/**
 * @brief Attach a resource to a node, taking an inode-side reference.
 *
 * The node then holds one reference on @p res; it is dropped automatically when
 * the node is destroyed (cooperative eviction).  A node may hold at most one
 * resource.
 *
 * @param tree The tree (for locking).
 * @param node The node to attach to.
 * @param res  The resource (already registered).
 * @return 0 on success, -1 on error (node already has a resource).
 */
int emu_node_attach_resource(struct emu_node_tree *tree, struct emu_node *node,
                             struct emu_resource *res);

/**
 * @brief Liveness check for an op on an open resource handle (e.g. qpair fd).
 * @param tree The tree (for locking).
 * @param res  The resource the op targets.
 * @return 0 if live; -ENODEV if torn down / revoked.
 */
int emu_resource_check(struct emu_node_tree *tree, struct emu_resource *res);

/* ------------------------------------------------------------------ */
/* Revocation (forced removal -- the keystone for T9)                 */
/* ------------------------------------------------------------------ */

/**
 * @brief Eagerly revoke a whole device (forced removal / REMOVE / SBR / HOTPLUG).
 *
 * For the device identified by @p bdf, atomically under the lock:
 *   - mark the device, its endpoint nodes, and every registered resource dead
 *     (so subsequent ops on already-open handles return @c -ENODEV);
 *   - tear down every registered resource (idempotent; queues stopped, QIDs
 *     freed) and drop the registry references;
 *   - unlink the endpoint nodes from the tree so new lookups return @c -ENOENT.
 * Then, with the lock dropped, invalidate the endpoint names via the notifier so
 * the kernel dentry cache forgets them.  Open fds survive as orphans until their
 * holders @c close them (release is idempotent).
 *
 * Idempotent: revoking an already-revoked BDF is a no-op success.
 *
 * @param tree The tree.
 * @param bdf  Normalized board-level BDF of the device to revoke.
 * @return 0 on success (including the already-revoked / absent no-op case).
 */
int emu_device_revoke(struct emu_node_tree *tree, const char *bdf);

#endif // SLASH_EMU_NODE_H
