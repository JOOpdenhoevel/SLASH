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
 * @file node.c
 * @brief Implementation of the slash-emu spine (see node.h for the design).
 *
 * Conventions used throughout:
 *   - Functions suffixed @c _locked assume @c tree->lock is held; the public
 *     entry points acquire it via @ref tree_lock / @ref tree_unlock.
 *   - The lock is never held across a notifier callback (see @ref locking in
 *     node.h); @ref emu_device_revoke collects names under the lock and fires the
 *     notifier after dropping it.
 */

#define _GNU_SOURCE

#include "node.h"

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <syslog.h>

#include "utils.h"

/* ================================================================== */
/* Locking helpers                                                    */
/* ================================================================== */

static void tree_lock(struct emu_node_tree *tree)
{
    (void) pthread_mutex_lock(&tree->lock);
}

static void tree_unlock(struct emu_node_tree *tree)
{
    (void) pthread_mutex_unlock(&tree->lock);
}

/*
 * Scope-guard unlock for __attribute__((cleanup)): set a
 * `_cleanup_(tree_unlockp) struct emu_node_tree *guard = tree;` right after
 * locking so every return path releases the lock.
 */
static void tree_unlockp(struct emu_node_tree **treep)
{
    if (treep == NULL || *treep == NULL) {
        return;
    }

    tree_unlock(*treep);
    *treep = NULL;
}

/* ================================================================== */
/* Node / device shell allocation + lookup                            */
/* ================================================================== */

void emu_node_free_shell(struct emu_node *node)
{
    if (node == NULL) {
        return;
    }

    free(node->name);
    emu_node_ref_array_free(&node->children);
    free(node);
}

void emu_device_free_shell(struct emu_device *dev)
{
    if (dev == NULL) {
        return;
    }

    emu_resource_ref_array_free(&dev->registry);
    free(dev);
}

/* Resolve an inode to its node (assumes lock held). */
static struct emu_node *find_ino_locked(struct emu_node_tree *tree,
                                        emu_ino_t ino)
{
    for (size_t i = 0; i < tree->nodes.len; i++) {
        if (tree->nodes.d[i]->ino == ino) {
            return tree->nodes.d[i];
        }
    }

    return NULL;
}

/* Resolve a live child by name within a directory (assumes lock held). */
static struct emu_node *find_child_locked(const struct emu_node *parent,
                                          const char *name)
{
    for (size_t i = 0; i < parent->children.len; i++) {
        struct emu_node *child = parent->children.d[i];
        if (!child->unlinked && strcmp(child->name, name) == 0) {
            return child;
        }
    }

    return NULL;
}

/*
 * Allocate a node, assign it a fresh inode, and register it in the inode table.
 * The node is not yet linked into any parent.  Assumes the lock is held.
 */
static int node_alloc_locked(struct emu_node_tree *tree, const char *name,
                             enum emu_node_type type, mode_t mode,
                             struct emu_node **out)
{
    _cleanup_(cleanup_free)
    char *name_copy = strdup(name);
    PROPAGATE_ERROR_NULL_LOG(name_copy, LOG_ERR, "Failed to copy node name");

    struct emu_node *node = calloc(1, sizeof(*node));
    PROPAGATE_ERROR_NULL_LOG(node, LOG_ERR, "Failed to allocate node");

    *node = (struct emu_node) {
        .ino = tree->next_ino,
        .name = name_copy,
        .type = type,
        .mode = mode,
        .children = emu_node_ref_array_init(),
        .live = true,
        .unlinked = false,
        .lookup_count = 0,
    };
    name_copy = NULL;

    if (emu_node_owning_array_push(&tree->nodes, node) == -1) {
        emu_node_free_shell(node);
        LOG(LOG_ERR, "Failed to register node in inode table");
        return -1;
    }

    tree->next_ino++;
    *out = node;

    return 0;
}

/*
 * Link an already-allocated node under a parent directory.  Assumes lock held.
 */
static int node_link_locked(struct emu_node *parent, struct emu_node *child)
{
    int ret = emu_node_ref_array_push(&parent->children, child);
    PROPAGATE_ERROR_LOG(ret, LOG_ERR, "Failed to link node '%s'", child->name);

    child->parent = parent;
    child->device = parent->device;

    return 0;
}

/* ================================================================== */
/* Lifecycle                                                          */
/* ================================================================== */

int emu_node_tree_new(struct emu_node_tree **treep,
                      const struct emu_notifier *notifier)
{
    _cleanup_(cleanup_node_treep)
    struct emu_node_tree *tree = calloc(1, sizeof(*tree));
    PROPAGATE_ERROR_NULL_LOG(tree, LOG_ERR, "Failed to allocate node tree");

    *tree = (struct emu_node_tree) {
        .nodes = emu_node_owning_array_init(),
        .devices = emu_device_owning_array_init(),
        .next_ino = EMU_ROOT_INO,
        .notifier = notifier != NULL ? *notifier : (struct emu_notifier) {0},
    };

    int pret = pthread_mutex_init(&tree->lock, NULL);
    if (pret != 0) {
        LOG(LOG_ERR, "Failed to init tree mutex: %s", strerror(pret));
        /* No mutex to destroy; free directly to avoid cleanup touching it. */
        emu_node_owning_array_free(&tree->nodes);
        emu_device_owning_array_free(&tree->devices);
        free(tree);
        tree = NULL;
        return -1;
    }

    /* The root directory: inode EMU_ROOT_INO, name "/", world-readable+exec. */
    struct emu_node *root = NULL;
    if (node_alloc_locked(tree, "/", EMU_NODE_DIR, 0555, &root) == -1) {
        return -1;
    }
    tree->root = root;

    *treep = tree;
    tree = NULL;

    return 0;
}

void cleanup_node_tree(struct emu_node_tree *tree)
{
    if (tree == NULL) {
        return;
    }

    /*
     * Ordered teardown: drop every resource first (registry + any inode refs),
     * so no emu_resource outlives the tree.  We do not run teardown callbacks on
     * a plain destroy -- the daemon is going away -- but we must free backings,
     * each EXACTLY once.  A resource can be reachable from two roots at once:
     *
     *   - the registry of a still-live device (res->device != NULL), and/or
     *   - an inode that still references it (some node->resource == res).
     *
     * To make "free exactly once" airtight regardless of how many roots point at
     * a resource, we do a single ownership pass per resource that frees it AND
     * severs BOTH references in lock-step: the registry pass below frees every
     * registered resource and, for each, clears any node back-pointer to it
     * before moving on.  The orphan pass that follows then only ever sees
     * resources whose registry ref was already dropped (by an earlier revoke) and
     * whose node back-pointer is therefore the sole remaining reference -- true
     * inode-only orphans.  No freed resource is read or freed twice; none is
     * missed.
     */
    for (size_t i = 0; i < tree->devices.len; i++) {
        struct emu_device *dev = tree->devices.d[i];
        for (size_t j = 0; j < dev->registry.len; j++) {
            struct emu_resource *res = dev->registry.d[j];

            /* Sever the inode back-reference(s) BEFORE freeing, so the orphan
             * pass below never dereferences this soon-to-be-freed pointer. */
            for (size_t k = 0; k < tree->nodes.len; k++) {
                if (tree->nodes.d[k]->resource == res) {
                    tree->nodes.d[k]->resource = NULL;
                }
            }

            if (res->free_backing != NULL) {
                res->free_backing(res->backing);
            }
            free(res);
        }
        dev->registry.len = 0;
    }

    /*
     * Reclaim inode-only orphans: resources a prior forced revoke unregistered
     * (resource_unregister_locked NULLs res->device) while an open handle kept
     * the inode ref, with no intervening forget before shutdown.  After the
     * registry pass cleared every still-registered resource's node back-pointer,
     * any node->resource still set here is necessarily such an orphan.
     */
    for (size_t i = 0; i < tree->nodes.len; i++) {
        struct emu_resource *res = tree->nodes.d[i]->resource;
        if (res == NULL) {
            continue;
        }
        if (res->free_backing != NULL) {
            res->free_backing(res->backing);
        }
        free(res);
        tree->nodes.d[i]->resource = NULL;
    }

    emu_node_owning_array_free(&tree->nodes);
    emu_device_owning_array_free(&tree->devices);

    (void) pthread_mutex_destroy(&tree->lock);

    free(tree);
}

/* ================================================================== */
/* Tree construction                                                  */
/* ================================================================== */

/* Find a live device by BDF (assumes lock held). */
static struct emu_device *find_device_locked(struct emu_node_tree *tree,
                                             const char *bdf)
{
    for (size_t i = 0; i < tree->devices.len; i++) {
        struct emu_device *dev = tree->devices.d[i];
        if (dev->live && strcmp(dev->bdf, bdf) == 0) {
            return dev;
        }
    }

    return NULL;
}

/*
 * Create a child directory under `parent` and return it.  Assumes lock held.
 */
static int make_dir_locked(struct emu_node_tree *tree, struct emu_node *parent,
                           const char *name, struct emu_node **out)
{
    struct emu_node *node = NULL;
    int ret = node_alloc_locked(tree, name, EMU_NODE_DIR, 0555, &node);
    PROPAGATE_ERROR(ret);

    ret = node_link_locked(parent, node);
    PROPAGATE_ERROR(ret);

    *out = node;
    return 0;
}

int emu_node_tree_add_device(struct emu_node_tree *tree, const char *bdf,
                             struct emu_device **devp)
{
    if (tree == NULL || bdf == NULL) {
        return -1;
    }

    tree_lock(tree);
    _cleanup_(tree_unlockp) struct emu_node_tree *guard = tree;

    /* Idempotent at the BDF level. */
    struct emu_device *existing = find_device_locked(tree, bdf);
    if (existing != NULL) {
        if (devp != NULL) {
            *devp = existing;
        }
        return 0;
    }

    struct emu_device *dev = calloc(1, sizeof(*dev));
    PROPAGATE_ERROR_NULL_LOG(dev, LOG_ERR, "Failed to allocate device");

    *dev = (struct emu_device) {
        .tree = tree,
        .registry = emu_resource_ref_array_init(),
        .live = true,
    };
    int n = snprintf(dev->bdf, sizeof(dev->bdf), "%s", bdf);
    if (n < 0 || (size_t) n >= sizeof(dev->bdf)) {
        emu_device_free_shell(dev);
        LOG(LOG_ERR, "BDF '%s' too long for device", bdf);
        return -1;
    }

    if (emu_device_owning_array_push(&tree->devices, dev) == -1) {
        emu_device_free_shell(dev);
        LOG(LOG_ERR, "Failed to register device '%s'", bdf);
        return -1;
    }

    /*
     * Build the subtree: /<BDF>/, /<BDF>/bars/, /<BDF>/qdma/.  The device's node
     * pointers are wired up before children inherit dev->* via node_link_locked,
     * so set dev->dir first, then attach bars/qdma under it.  On any failure the
     * partially-built nodes remain in the inode table but unreferenced as a
     * device subtree; cleanup_node_tree still frees them.  We unwind the device
     * registration to keep find_device_locked consistent.
     */
    struct emu_node *dir = NULL;
    if (make_dir_locked(tree, tree->root, bdf, &dir) == -1) {
        goto fail_unwind_device;
    }
    dir->device = dev;
    dev->dir = dir;

    if (make_dir_locked(tree, dir, "bars", &dev->bars) == -1) {
        goto fail_unwind_device;
    }
    if (make_dir_locked(tree, dir, "qdma", &dev->qdma) == -1) {
        goto fail_unwind_device;
    }

    if (devp != NULL) {
        *devp = dev;
    }

    return 0;

fail_unwind_device:
    /* Drop the device from the registry list (its nodes are owned by the tree
     * table and will be freed at tree destruction; mark device dead so it is no
     * longer found). */
    dev->live = false;
    return -1;
}

struct emu_device *emu_node_tree_find_device(struct emu_node_tree *tree,
                                             const char *bdf)
{
    if (tree == NULL || bdf == NULL) {
        return NULL;
    }

    tree_lock(tree);
    _cleanup_(tree_unlockp) struct emu_node_tree *guard = tree;

    return find_device_locked(tree, bdf);
}

int emu_node_create_child(struct emu_node_tree *tree, struct emu_node *parent,
                          const char *name, enum emu_node_type type, mode_t mode,
                          const struct emu_node_ops *ops, void *backing,
                          struct emu_node **nodep)
{
    if (tree == NULL || parent == NULL || name == NULL) {
        return -1;
    }

    tree_lock(tree);
    _cleanup_(tree_unlockp) struct emu_node_tree *guard = tree;

    if (parent->type != EMU_NODE_DIR) {
        LOG(LOG_ERR, "Parent of '%s' is not a directory", name);
        return -1;
    }

    struct emu_node *node = NULL;
    int ret = node_alloc_locked(tree, name, type, mode, &node);
    PROPAGATE_ERROR(ret);

    node->ops = ops;
    node->backing = backing;

    ret = node_link_locked(parent, node);
    PROPAGATE_ERROR(ret);

    if (nodep != NULL) {
        *nodep = node;
    }

    return 0;
}

/* ================================================================== */
/* Resource refcounting + teardown                                    */
/* ================================================================== */

/*
 * Drop one reference on a resource.  If it was the last, free the backing and
 * the object.  Assumes lock held.
 */
static void resource_unref_locked(struct emu_resource *res)
{
    if (res == NULL) {
        return;
    }

    if (res->refcount > 0) {
        res->refcount--;
    }

    if (res->refcount == 0) {
        if (res->free_backing != NULL) {
            res->free_backing(res->backing);
        }
        free(res);
    }
}

/*
 * Run a resource's teardown exactly once (idempotent).  Stops the queue and
 * frees the external id via the callback; does not touch refcounts.  Assumes
 * lock held.
 */
static void resource_teardown_locked(struct emu_resource *res)
{
    if (res == NULL || res->torn_down) {
        return;
    }

    res->torn_down = true;
    res->live = false;

    if (res->teardown != NULL) {
        res->teardown(res, res->backing);
    }
}

/*
 * Remove a resource from its device's registry, dropping the registry ref.
 * No-op if already unregistered.  Assumes lock held.
 */
static void resource_unregister_locked(struct emu_resource *res)
{
    struct emu_device *dev = res->device;
    if (dev == NULL) {
        return;
    }

    for (size_t i = 0; i < dev->registry.len; i++) {
        if (dev->registry.d[i] != res) {
            continue;
        }

        /* Remove from the registry array (order-independent). */
        dev->registry.d[i] = dev->registry.d[dev->registry.len - 1];
        dev->registry.len--;
        res->device = NULL;

        /* Drop the registry reference (may free the resource). */
        resource_unref_locked(res);
        return;
    }
}

int emu_device_register_resource(struct emu_device *dev, uint32_t id,
                                 emu_resource_teardown_fn teardown,
                                 emu_resource_free_fn free_backing,
                                 void *backing, struct emu_resource **resp)
{
    if (dev == NULL || dev->tree == NULL) {
        return -1;
    }

    tree_lock(dev->tree);
    _cleanup_(tree_unlockp) struct emu_node_tree *guard = dev->tree;

    if (!dev->live) {
        LOG(LOG_ERR, "Cannot register resource on revoked device '%s'",
            dev->bdf);
        return -1;
    }

    struct emu_resource *res = calloc(1, sizeof(*res));
    PROPAGATE_ERROR_NULL_LOG(res, LOG_ERR, "Failed to allocate resource");

    *res = (struct emu_resource) {
        .device = dev,
        .id = id,
        .refcount = 1, /* the registry reference */
        .torn_down = false,
        .live = true,
        .teardown = teardown,
        .free_backing = free_backing,
        .backing = backing,
    };

    if (emu_resource_ref_array_push(&dev->registry, res) == -1) {
        free(res);
        LOG(LOG_ERR, "Failed to register resource %u on '%s'", id, dev->bdf);
        return -1;
    }

    if (resp != NULL) {
        *resp = res;
    }

    return 0;
}

struct emu_resource *emu_device_find_resource(struct emu_device *dev,
                                              uint32_t id)
{
    if (dev == NULL || dev->tree == NULL) {
        return NULL;
    }

    tree_lock(dev->tree);
    _cleanup_(tree_unlockp) struct emu_node_tree *guard = dev->tree;

    for (size_t i = 0; i < dev->registry.len; i++) {
        if (dev->registry.d[i]->id == id) {
            return dev->registry.d[i];
        }
    }

    return NULL;
}

int emu_node_attach_resource(struct emu_node_tree *tree, struct emu_node *node,
                             struct emu_resource *res)
{
    if (tree == NULL || node == NULL || res == NULL) {
        return -1;
    }

    tree_lock(tree);
    _cleanup_(tree_unlockp) struct emu_node_tree *guard = tree;

    if (node->resource != NULL) {
        LOG(LOG_ERR, "Node '%s' already has a resource", node->name);
        return -1;
    }

    node->resource = res;
    res->refcount++; /* the inode reference */

    return 0;
}

int emu_resource_check(struct emu_node_tree *tree, struct emu_resource *res)
{
    if (tree == NULL || res == NULL) {
        return -ENODEV;
    }

    tree_lock(tree);
    _cleanup_(tree_unlockp) struct emu_node_tree *guard = tree;

    return res->live ? 0 : -ENODEV;
}

/* ================================================================== */
/* Node destruction (cooperative teardown trigger)                    */
/* ================================================================== */

/*
 * Destroy a node: run its destroy hook, drop its inode-side resource reference
 * (triggering cooperative teardown of an undisturbed resource), detach it from
 * the inode table, and free the shell.  Assumes the node is unlinked and has a
 * zero lookup count.  Assumes lock held.
 */
static void node_destroy_locked(struct emu_node_tree *tree,
                                struct emu_node *node)
{
    /* Detach from the parent's child list so no live array dangles at us. */
    if (node->parent != NULL) {
        emu_node_ref_array_rm_by_value(&node->parent->children, node);
        node->parent = NULL;
    }

    if (node->ops != NULL && node->ops->destroy != NULL) {
        node->ops->destroy(node, node->backing);
    }

    if (node->resource != NULL) {
        struct emu_resource *res = node->resource;
        node->resource = NULL;

        /*
         * Cooperative teardown: last close of this (possibly already-unlinked)
         * resource.  Teardown is idempotent -- if a forced revoke already ran it,
         * this is a no-op.  Then unregister (drops the registry ref if still
         * present) and finally drop the inode ref.  Both refs gone => freed.
         */
        resource_teardown_locked(res);
        resource_unregister_locked(res);
        resource_unref_locked(res);
    }

    /* Remove from the inode table (frees the shell via the owning array). */
    emu_node_owning_array_rm_by_reference(&tree->nodes, node);
}

void emu_node_unlink(struct emu_node_tree *tree, struct emu_node *node)
{
    if (tree == NULL || node == NULL) {
        return;
    }

    tree_lock(tree);
    _cleanup_(tree_unlockp) struct emu_node_tree *guard = tree;

    if (node->unlinked) {
        return;
    }

    node->unlinked = true;

    /* Detach from the parent so the name no longer resolves. */
    if (node->parent != NULL) {
        emu_node_ref_array_rm_by_value(&node->parent->children, node);
    }

    /*
     * Reap now if the kernel holds no lookups; otherwise the node survives as a
     * nameless orphan and node_destroy_locked runs on the final forget -- the
     * cooperative teardown trigger.  node_destroy_locked re-checks node->parent,
     * so leaving it set here is harmless.
     */
    if (node->lookup_count == 0) {
        node_destroy_locked(tree, node);
    } else {
        node->parent = NULL;
    }
}

/* ================================================================== */
/* FUSE op support                                                    */
/* ================================================================== */

struct emu_node *emu_node_lookup_ino(struct emu_node_tree *tree, emu_ino_t ino)
{
    if (tree == NULL) {
        return NULL;
    }

    tree_lock(tree);
    _cleanup_(tree_unlockp) struct emu_node_tree *guard = tree;

    return find_ino_locked(tree, ino);
}

int emu_node_lookup_child(struct emu_node_tree *tree, emu_ino_t parent,
                          const char *name, struct emu_node **out)
{
    if (tree == NULL || name == NULL) {
        return -EINVAL;
    }

    tree_lock(tree);
    _cleanup_(tree_unlockp) struct emu_node_tree *guard = tree;

    struct emu_node *pnode = find_ino_locked(tree, parent);
    if (pnode == NULL) {
        return -ESTALE;
    }
    if (pnode->type != EMU_NODE_DIR) {
        return -ENOTDIR;
    }

    struct emu_node *child = find_child_locked(pnode, name);
    if (child == NULL || !child->live) {
        return -ENOENT;
    }

    child->lookup_count++;
    if (out != NULL) {
        *out = child;
    }

    return 0;
}

void emu_node_forget(struct emu_node_tree *tree, emu_ino_t ino, uint64_t nlookup)
{
    if (tree == NULL) {
        return;
    }

    tree_lock(tree);
    _cleanup_(tree_unlockp) struct emu_node_tree *guard = tree;

    struct emu_node *node = find_ino_locked(tree, ino);
    if (node == NULL) {
        return;
    }

    if (nlookup >= node->lookup_count) {
        node->lookup_count = 0;
    } else {
        node->lookup_count -= nlookup;
    }

    /* Reap a node the kernel has fully forgotten if it is already unlinked. */
    if (node->lookup_count == 0 && node->unlinked) {
        node_destroy_locked(tree, node);
    }
}

int emu_node_stat(struct emu_node_tree *tree, emu_ino_t ino, struct stat *st)
{
    if (tree == NULL || st == NULL) {
        return -EINVAL;
    }

    tree_lock(tree);
    _cleanup_(tree_unlockp) struct emu_node_tree *guard = tree;

    struct emu_node *node = find_ino_locked(tree, ino);
    if (node == NULL) {
        return -ENOENT;
    }

    *st = (struct stat) {
        .st_ino = node->ino,
    };

    if (node->type == EMU_NODE_DIR) {
        st->st_mode = S_IFDIR | (node->mode & 07777);
        st->st_nlink = 2;
    } else {
        st->st_mode = S_IFREG | (node->mode & 07777);
        st->st_nlink = 1;
        if (node->ops != NULL && node->ops->size != NULL) {
            st->st_size = node->ops->size(node, node->backing);
        }
    }

    return 0;
}

int emu_node_readdir(struct emu_node_tree *tree, emu_ino_t ino,
                     emu_readdir_cb cb, void *ctx)
{
    if (tree == NULL || cb == NULL) {
        return -EINVAL;
    }

    tree_lock(tree);
    _cleanup_(tree_unlockp) struct emu_node_tree *guard = tree;

    struct emu_node *node = find_ino_locked(tree, ino);
    if (node == NULL) {
        return -ENOENT;
    }
    if (node->type != EMU_NODE_DIR) {
        return -ENOTDIR;
    }

    emu_ino_t parent_ino = node->parent != NULL ? node->parent->ino : node->ino;

    if (!cb(ctx, ".", node->ino, EMU_NODE_DIR)) {
        return 0;
    }
    if (!cb(ctx, "..", parent_ino, EMU_NODE_DIR)) {
        return 0;
    }

    for (size_t i = 0; i < node->children.len; i++) {
        struct emu_node *child = node->children.d[i];
        if (child->unlinked || !child->live) {
            continue;
        }
        if (!cb(ctx, child->name, child->ino, child->type)) {
            return 0;
        }
    }

    return 0;
}

int emu_node_is_live(struct emu_node_tree *tree, emu_ino_t ino)
{
    if (tree == NULL) {
        return -ENOENT;
    }

    tree_lock(tree);
    _cleanup_(tree_unlockp) struct emu_node_tree *guard = tree;

    struct emu_node *node = find_ino_locked(tree, ino);
    if (node == NULL) {
        return -ENOENT;
    }

    return node->live ? 0 : -ENODEV;
}

/* ================================================================== */
/* Revocation (forced removal)                                        */
/* ================================================================== */

/*
 * A name to invalidate after the lock is dropped: parent inode, child inode, and
 * the entry name (owned copy).
 */
struct pending_invalidation {
    emu_ino_t parent;
    emu_ino_t child;
    char *name; /* owning */
};

/* Free a pending invalidation including its owned name. */
static void pending_inval_free(struct pending_invalidation *p)
{
    if (p == NULL) {
        return;
    }
    free(p->name);
    free(p);
}

DECLARE_OWNING_PTR_ARRAY(pending_inval_array, struct pending_invalidation *,
                         pending_inval_free)

/* Record one (parent, child, name) invalidation.  Assumes lock held. */
static int record_invalidation_locked(struct emu_node *node,
                                      struct pending_inval_array *pending)
{
    if (node->parent == NULL || node->unlinked) {
        return 0;
    }

    struct pending_invalidation *p = calloc(1, sizeof(*p));
    PROPAGATE_ERROR_NULL_LOG(p, LOG_ERR, "Failed to allocate invalidation");

    p->parent = node->parent->ino;
    p->child = node->ino;
    p->name = strdup(node->name);
    if (p->name == NULL) {
        free(p);
        LOG(LOG_ERR, "Failed to copy invalidation name");
        return -1;
    }

    if (pending_inval_array_push(pending, p) == -1) {
        pending_inval_free(p);
        LOG(LOG_ERR, "Failed to record invalidation");
        return -1;
    }

    return 0;
}

/*
 * Revoke an entire subtree rooted at @p node: record names to invalidate, mark
 * each node dead + unlinked, sever parent/child links, and reap nodes the kernel
 * no longer references.  Nodes with outstanding lookups survive as dead orphans
 * (parent cleared) until forgotten.  Returns -1 only if recording an
 * invalidation name failed (revocation itself still completes).  Lock held.
 */
static int revoke_subtree_locked(struct emu_node_tree *tree,
                                 struct emu_node *node,
                                 struct pending_inval_array *pending)
{
    int rc = 0;

    /* Snapshot children: destruction below mutates the children array. */
    struct emu_node_ref_array kids = emu_node_ref_array_init();
    for (size_t i = 0; i < node->children.len; i++) {
        if (emu_node_ref_array_push(&kids, node->children.d[i]) == -1) {
            rc = -1; /* best-effort: fall through and revoke what we can */
            break;
        }
    }

    for (size_t i = 0; i < kids.len; i++) {
        if (revoke_subtree_locked(tree, kids.d[i], pending) == -1) {
            rc = -1;
        }
    }
    emu_node_ref_array_free(&kids);

    if (record_invalidation_locked(node, pending) == -1) {
        rc = -1;
    }

    node->live = false;
    node->unlinked = true;

    /*
     * All children have been revoked (and possibly freed) above, so drop our
     * references to them.  This prevents a surviving (still-looked-up) dead node
     * from ever iterating dangling child pointers in readdir.
     */
    node->children.len = 0;

    if (node->lookup_count == 0) {
        /* node_destroy_locked detaches us from our parent's child list. */
        node_destroy_locked(tree, node);
    } else {
        /*
         * The kernel still references this node; it survives as a dead orphan
         * until forgotten.  Detach it from its parent now so a later reap (after
         * the parent may itself be gone) never touches a stale parent.
         */
        if (node->parent != NULL) {
            emu_node_ref_array_rm_by_value(&node->parent->children, node);
            node->parent = NULL;
        }
    }

    return rc;
}

int emu_device_revoke(struct emu_node_tree *tree, const char *bdf)
{
    if (tree == NULL || bdf == NULL) {
        return -1;
    }

    struct pending_inval_array pending = pending_inval_array_init();

    tree_lock(tree);

    struct emu_device *dev = find_device_locked(tree, bdf);
    if (dev == NULL) {
        /* Idempotent: already revoked or never existed. */
        tree_unlock(tree);
        return 0;
    }

    dev->live = false;

    /*
     * 1. Tear down every registered resource (idempotent).  Iterate from the end
     *    because resource_unregister_locked compacts the array in place and may
     *    free the resource if no inode ref remains.
     */
    while (dev->registry.len > 0) {
        struct emu_resource *res = dev->registry.d[dev->registry.len - 1];
        resource_teardown_locked(res);
        resource_unregister_locked(res); /* drops registry ref, compacts array */
    }

    /*
     * 2. Revoke the endpoint subtree: mark every node dead + unlinked, sever the
     *    links, reap nodes the kernel has already forgotten, and gather the names
     *    to invalidate.  On allocation failure the revocation still completes
     *    (best-effort); -ENODEV/-ENOENT correctness depends only on the
     *    dead/unlinked flags, not on the kernel notification.
     *
     *    revoke_subtree_locked may free dev->dir (and dev->bars/qdma); the device
     *    keeps its (now stale) node pointers but is already marked dead, so it is
     *    never found again.  Clear them defensively.
     */
    if (revoke_subtree_locked(tree, dev->dir, &pending) == -1) {
        LOG(LOG_WARNING, "Revoke of '%s': name-invalidation list incomplete",
            bdf);
    }
    dev->dir = NULL;
    dev->bars = NULL;
    dev->qdma = NULL;

    tree_unlock(tree);

    /*
     * 4. Fire the notifier with the lock dropped.  notify_delete forces the
     *    kernel dentry cache to forget the names so a fresh lookup misses.
     */
    if (tree->notifier.notify_delete != NULL) {
        for (size_t i = 0; i < pending.len; i++) {
            struct pending_invalidation *p = pending.d[i];
            tree->notifier.notify_delete(tree->notifier.ctx, p->parent,
                                         p->child, p->name);
        }
    }

    pending_inval_array_free(&pending);

    return 0;
}
