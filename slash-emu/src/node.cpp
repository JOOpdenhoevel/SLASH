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
 * @file node.cpp
 * @brief Implementation of the slash-emu spine (see node.hpp for the design).
 *
 * Conventions used throughout:
 *   - Methods suffixed @c Locked assume @c mutex_ is held; the public entry
 *     points acquire it via @c std::scoped_lock.
 *   - The lock is never held across a @ref Notifier callback (see @ref locking in
 *     node.hpp); @ref NodeTree::revokeDevice collects names under the lock and
 *     fires the notifier after dropping it.
 *
 * @section refcount Refcount model (port note)
 *
 * The C daemon hand-rolled an integer @c refcount with a registry ref and an
 * inode ref.  The C++ port maps the @em free to @c std::shared_ptr ownership: the
 * @ref Device registry holds one @c shared_ptr (the registry reference) and the
 * @ref Node holds the other (the inode reference, @c Node::resource).  The
 * @ref Resource object is destroyed exactly when both drop -- and, crucially,
 * destruction is automatic, so the elaborate ordered-teardown pass the C
 * @c cleanup_node_tree needed collapses into ordinary member destruction:
 * @c devices_ destructs before @c nodes_ (reverse declaration order), so a
 * resource reachable from both roots is freed exactly once when the surviving
 * (inode) reference finally drops.  The @em teardown (stop queue / free QID)
 * stays an explicit, idempotent @ref Resource::teardown fired on the first of
 * {cooperative eviction, forced removal}; a plain shutdown does @em not run it.
 */

#include "node.hpp"

#include <algorithm>
#include <utility>

#include "utils.hpp"

namespace slash::emu {

/* ================================================================== */
/* Resource                                                           */
/* ================================================================== */

Resource::Resource(Device &dev, uint32_t id, ResourceTeardownFn teardown)
    : device_(dev), id_(id), teardown_(std::move(teardown))
{
}

void Resource::teardown()
{
    if (torn_down_) {
        return;
    }

    torn_down_ = true;
    live_ = false;

    if (teardown_) {
        teardown_(*this);
    }
}

/* ================================================================== */
/* Device                                                             */
/* ================================================================== */

Device::Device(NodeTree &tree, std::string bdf)
    : tree_(tree), bdf_(std::move(bdf))
{
}

Resource *Device::registerResource(uint32_t id, ResourceTeardownFn teardown)
{
    std::scoped_lock lock(tree_.mutex_);
    return registerResourceLocked(id, std::move(teardown));
}

Resource *Device::registerResourceLocked(uint32_t id, ResourceTeardownFn teardown)
{
    if (!live) {
        LOG(LOG_ERR, "Cannot register resource on revoked device '%s'",
            bdf_.c_str());
        return nullptr;
    }

    /* The registry holds the registry reference (one of the two shared_ptrs). */
    auto res = std::make_shared<Resource>(*this, id, std::move(teardown));
    registry_.push_back(res);

    return res.get();
}

Resource *Device::findResource(uint32_t id)
{
    std::scoped_lock lock(tree_.mutex_);
    return findResourceLocked(id);
}

Resource *Device::findResourceLocked(uint32_t id)
{
    for (const auto &res : registry_) {
        if (res->id() == id) {
            return res.get();
        }
    }

    return nullptr;
}

int Device::setModelShutdown(ModelShutdownFn fn)
{
    std::scoped_lock lock(tree_.mutex_);

    if (!live) {
        return -1;
    }

    model_shutdown_ = std::move(fn);
    return 0;
}

/* ================================================================== */
/* NodeTree: lifecycle                                                */
/* ================================================================== */

NodeTree::NodeTree(Notifier notifier) : notifier_(std::move(notifier))
{
    /* The root directory: inode kRootIno, name "/", world-readable+exec. */
    root_ = allocNodeLocked("/", NodeType::Dir, 0555, nullptr);
}

NodeTree::~NodeTree() = default;

/* ================================================================== */
/* Node / device allocation + lookup helpers                          */
/* ================================================================== */

Node *NodeTree::allocNodeLocked(const std::string &name, NodeType type,
                                mode_t mode, std::unique_ptr<NodeOps> ops)
{
    auto node = std::make_unique<Node>();
    node->ino = next_ino_;
    node->name = name;
    node->type = type;
    node->mode = mode;
    node->ops = std::move(ops);

    Node *raw = node.get();
    nodes_.push_back(std::move(node));
    next_ino_++;

    return raw;
}

Node *NodeTree::findInoLocked(Ino ino) const
{
    for (const auto &node : nodes_) {
        if (node->ino == ino) {
            return node.get();
        }
    }

    return nullptr;
}

Node *NodeTree::findChildLocked(const Node *parent, const std::string &name) const
{
    for (Node *child : parent->children) {
        if (!child->unlinked && child->name == name) {
            return child;
        }
    }

    return nullptr;
}

void NodeTree::linkChildLocked(Node *parent, Node *child)
{
    parent->children.push_back(child);
    child->parent = parent;
    child->device = parent->device;
}

Device *NodeTree::findDeviceLocked(const std::string &bdf) const
{
    for (const auto &dev : devices_) {
        if (dev->live && dev->bdf_ == bdf) {
            return dev.get();
        }
    }

    return nullptr;
}

Node *NodeTree::makeDirLocked(Node *parent, const std::string &name)
{
    Node *node = allocNodeLocked(name, NodeType::Dir, 0555, nullptr);
    linkChildLocked(parent, node);
    return node;
}

/* ================================================================== */
/* Tree construction                                                  */
/* ================================================================== */

Device *NodeTree::addDevice(const std::string &bdf)
{
    std::scoped_lock lock(mutex_);

    /* Idempotent at the BDF level. */
    Device *existing = findDeviceLocked(bdf);
    if (existing != nullptr) {
        return existing;
    }

    auto dev = std::make_unique<Device>(*this, bdf);
    Device *raw = dev.get();
    devices_.push_back(std::move(dev));

    /*
     * Build the subtree: /<BDF>/, /<BDF>/bars/, /<BDF>/qdma/.  The device's node
     * pointers are wired up before children inherit dev->* via linkChildLocked, so
     * set raw->dir first, then attach bars/qdma under it.
     */
    Node *dir = makeDirLocked(root_, bdf);
    dir->device = raw;
    raw->dir = dir;

    raw->bars = makeDirLocked(dir, "bars");
    raw->qdma = makeDirLocked(dir, "qdma");

    return raw;
}

Device *NodeTree::findDevice(const std::string &bdf)
{
    std::scoped_lock lock(mutex_);
    return findDeviceLocked(bdf);
}

Node *NodeTree::createChild(Node *parent, const std::string &name, NodeType type,
                           mode_t mode, std::unique_ptr<NodeOps> ops)
{
    std::scoped_lock lock(mutex_);
    return createChildLocked(parent, name, type, mode, std::move(ops));
}

Node *NodeTree::createChildLocked(Node *parent, const std::string &name,
                                  NodeType type, mode_t mode,
                                  std::unique_ptr<NodeOps> ops)
{
    if (parent == nullptr) {
        return nullptr;
    }

    if (parent->type != NodeType::Dir) {
        LOG(LOG_ERR, "Parent of '%s' is not a directory", name.c_str());
        return nullptr;
    }

    Node *node = allocNodeLocked(name, type, mode, std::move(ops));
    linkChildLocked(parent, node);

    return node;
}

void NodeTree::setDirectIo(Node *node)
{
    std::scoped_lock lock(mutex_);
    setDirectIoLocked(node);
}

void NodeTree::setDirectIoLocked(Node *node)
{
    if (node == nullptr) {
        return;
    }
    node->direct_io = true;
}

void NodeTree::setUnlinkable(Node *node)
{
    std::scoped_lock lock(mutex_);
    setUnlinkableLocked(node);
}

void NodeTree::setUnlinkableLocked(Node *node)
{
    if (node == nullptr) {
        return;
    }
    node->unlinkable = true;
}

void NodeTree::setIoctlUnlocked(Node *node)
{
    std::scoped_lock lock(mutex_);
    if (node == nullptr) {
        return;
    }
    node->ioctl_unlocked = true;
}

int NodeTree::setOps(Node *node, std::unique_ptr<NodeOps> ops)
{
    std::scoped_lock lock(mutex_);
    return setOpsLocked(node, std::move(ops));
}

int NodeTree::setOpsLocked(Node *node, std::unique_ptr<NodeOps> ops)
{
    if (node == nullptr) {
        return -1;
    }

    if (node->ops) {
        LOG(LOG_ERR, "Node '%s' already has ops", node->name.c_str());
        return -1;
    }

    node->ops = std::move(ops);

    return 0;
}

bool NodeTree::wantsDirectIo(Ino ino)
{
    std::scoped_lock lock(mutex_);
    Node *node = findInoLocked(ino);
    return node != nullptr && node->direct_io;
}

/* ================================================================== */
/* Resource attach (inode reference)                                  */
/* ================================================================== */

int NodeTree::attachResource(Node *node, std::shared_ptr<Resource> res)
{
    std::scoped_lock lock(mutex_);
    return attachResourceLocked(node, std::move(res));
}

int NodeTree::attachResourceLocked(Node *node, std::shared_ptr<Resource> res)
{
    if (node == nullptr || res == nullptr) {
        return -1;
    }

    if (node->resource != nullptr) {
        LOG(LOG_ERR, "Node '%s' already has a resource", node->name.c_str());
        return -1;
    }

    /* The node takes the inode reference (the second of the two shared_ptrs). */
    node->resource = std::move(res);

    return 0;
}

int NodeTree::resourceCheck(Resource *res)
{
    std::scoped_lock lock(mutex_);
    return resourceCheckLocked(res);
}

int NodeTree::resourceCheckLocked(Resource *res)
{
    if (res == nullptr) {
        return -ENODEV;
    }
    return res->live() ? 0 : -ENODEV;
}

/* ================================================================== */
/* Node destruction (cooperative teardown trigger)                    */
/* ================================================================== */

void NodeTree::unregisterResourceLocked(Resource *res)
{
    if (res == nullptr) {
        return;
    }

    Device &dev = res->device();
    auto &reg = dev.registry_;
    auto it = std::find_if(reg.begin(), reg.end(),
                           [res](const std::shared_ptr<Resource> &r) {
                               return r.get() == res;
                           });
    if (it != reg.end()) {
        reg.erase(it);
    }
}

void NodeTree::destroyNodeLocked(Node *node)
{
    /* Detach from the parent's child list so no live array dangles at us. */
    if (node->parent != nullptr) {
        auto &kids = node->parent->children;
        kids.erase(std::remove(kids.begin(), kids.end(), node), kids.end());
        node->parent = nullptr;
    }

    /*
     * Cooperative teardown: last close of this (possibly already-unlinked)
     * resource.  Teardown is idempotent -- if a forced revoke already ran it,
     * this is a no-op.  Then unregister (drops the registry reference if still
     * present, matching the C node_destroy_locked) and finally drop the inode
     * reference.  Both refs gone => the Resource object is freed (and its backing
     * with it).  Dropping the registry ref here makes the two drop orders for a
     * cooperatively-evicted node collapse: there is no reachable "inode dropped
     * but registry ref still held" intermediate state.
     *
     * The per-node ops hook is owned by the node (unique_ptr<NodeOps>); its
     * destructor -- the C "destroy hook" -- runs automatically when the node's
     * shell is freed below.
     */
    if (node->resource != nullptr) {
        Resource *res = node->resource.get();
        res->teardown();
        unregisterResourceLocked(res);
        node->resource.reset();
    }

    /* Remove from the inode table (frees the shell, running ops's destructor). */
    auto it = std::find_if(nodes_.begin(), nodes_.end(),
                           [node](const std::unique_ptr<Node> &n) {
                               return n.get() == node;
                           });
    if (it != nodes_.end()) {
        nodes_.erase(it);
    }
}

void NodeTree::unlink(Node *node)
{
    std::scoped_lock lock(mutex_);
    unlinkLocked(node);
}

void NodeTree::unlinkLocked(Node *node)
{
    if (node == nullptr || node->unlinked) {
        return;
    }

    node->unlinked = true;

    /* Detach from the parent so the name no longer resolves. */
    if (node->parent != nullptr) {
        auto &kids = node->parent->children;
        kids.erase(std::remove(kids.begin(), kids.end(), node), kids.end());
    }

    /*
     * Reap now if the kernel holds no lookups; otherwise the node survives as a
     * nameless orphan and destroyNodeLocked runs on the final forget -- the
     * cooperative teardown trigger.  destroyNodeLocked re-checks node->parent, so
     * leaving it set here is harmless; clear it otherwise so a later reap never
     * touches a stale parent.
     */
    if (node->lookup_count == 0) {
        destroyNodeLocked(node);
    } else {
        node->parent = nullptr;
    }
}

int NodeTree::unlinkChild(Ino parent, const std::string &name)
{
    std::scoped_lock lock(mutex_);

    Node *pnode = findInoLocked(parent);
    if (pnode == nullptr) {
        return -ENOENT;
    }
    if (pnode->type != NodeType::Dir) {
        return -ENOTDIR;
    }

    Node *child = findChildLocked(pnode, name);
    if (child == nullptr) {
        return -ENOENT;
    }

    /* The endpoint directories are removed via revocation, not user unlink. */
    if (child->type != NodeType::File) {
        return -EISDIR;
    }

    /* Unlinkability is opt-in: only the qpair<Q> files are user-removable (the
     * VRTD delete-on-last-close pattern).  info / bar<M> / the global hotplug
     * control file are NOT, so a stray unlink cannot free the control surface or
     * an endpoint out from under the model. */
    if (!child->unlinkable) {
        return -EPERM;
    }

    unlinkLocked(child);

    return 0;
}

/* ================================================================== */
/* FUSE op support                                                    */
/* ================================================================== */

Node *NodeTree::lookupIno(Ino ino)
{
    std::scoped_lock lock(mutex_);
    return findInoLocked(ino);
}

int NodeTree::lookupChild(Ino parent, const std::string &name, Node **out)
{
    std::scoped_lock lock(mutex_);

    Node *pnode = findInoLocked(parent);
    if (pnode == nullptr) {
        return -ESTALE;
    }
    if (pnode->type != NodeType::Dir) {
        return -ENOTDIR;
    }

    Node *child = findChildLocked(pnode, name);
    if (child == nullptr || !child->live) {
        return -ENOENT;
    }

    child->lookup_count++;
    if (out != nullptr) {
        *out = child;
    }

    return 0;
}

void NodeTree::forget(Ino ino, uint64_t nlookup)
{
    std::scoped_lock lock(mutex_);

    Node *node = findInoLocked(ino);
    if (node == nullptr) {
        return;
    }

    if (nlookup >= node->lookup_count) {
        node->lookup_count = 0;
    } else {
        node->lookup_count -= nlookup;
    }

    /* Reap a node the kernel has fully forgotten if it is already unlinked. */
    if (node->lookup_count == 0 && node->unlinked) {
        destroyNodeLocked(node);
    }
}

int NodeTree::stat(Ino ino, struct stat *st)
{
    if (st == nullptr) {
        return -EINVAL;
    }

    std::scoped_lock lock(mutex_);

    Node *node = findInoLocked(ino);
    if (node == nullptr) {
        return -ENOENT;
    }

    *st = (struct stat){};
    st->st_ino = node->ino;

    if (node->type == NodeType::Dir) {
        st->st_mode = S_IFDIR | (node->mode & 07777);
        st->st_nlink = 2;
    } else {
        st->st_mode = S_IFREG | (node->mode & 07777);
        st->st_nlink = 1;
        if (node->ops) {
            st->st_size = node->ops->size(*node);
        }
    }

    return 0;
}

int NodeTree::readdir(Ino ino, const ReaddirCb &cb)
{
    if (!cb) {
        return -EINVAL;
    }

    std::scoped_lock lock(mutex_);

    Node *node = findInoLocked(ino);
    if (node == nullptr) {
        return -ENOENT;
    }
    if (node->type != NodeType::Dir) {
        return -ENOTDIR;
    }

    Ino parent_ino = node->parent != nullptr ? node->parent->ino : node->ino;

    if (!cb(".", node->ino, NodeType::Dir)) {
        return 0;
    }
    if (!cb("..", parent_ino, NodeType::Dir)) {
        return 0;
    }

    for (Node *child : node->children) {
        if (child->unlinked || !child->live) {
            continue;
        }
        if (!cb(child->name, child->ino, child->type)) {
            return 0;
        }
    }

    return 0;
}

int NodeTree::isLive(Ino ino)
{
    std::scoped_lock lock(mutex_);

    Node *node = findInoLocked(ino);
    if (node == nullptr) {
        return -ENOENT;
    }

    return node->live ? 0 : -ENODEV;
}

ssize_t NodeTree::pread(Ino ino, char *buf, size_t size, off_t off)
{
    if (buf == nullptr || off < 0) {
        return -EINVAL;
    }

    std::scoped_lock lock(mutex_);

    Node *node = findInoLocked(ino);
    if (node == nullptr) {
        return -ENOENT;
    }

    /* Liveness gate: a revoked endpoint returns -ENODEV on an open fd. */
    if (!node->live) {
        return -ENODEV;
    }

    if (!node->ops) {
        return -EIO;
    }

    return node->ops->read(*node, buf, size, off);
}

ssize_t NodeTree::pwrite(Ino ino, const char *buf, size_t size, off_t off)
{
    if (buf == nullptr || off < 0) {
        return -EINVAL;
    }

    std::scoped_lock lock(mutex_);

    Node *node = findInoLocked(ino);
    if (node == nullptr) {
        return -ENOENT;
    }

    /* Liveness gate: a revoked endpoint returns -ENODEV on an open fd. */
    if (!node->live) {
        return -ENODEV;
    }

    if (!node->ops) {
        return -EIO;
    }

    return node->ops->write(*node, buf, size, off);
}

int NodeTree::ioctl(Ino ino, unsigned int cmd, const void *in, size_t in_size,
                    void *out, size_t out_size)
{
    std::unique_lock<std::mutex> lock(mutex_);

    Node *node = findInoLocked(ino);
    if (node == nullptr) {
        return -ENOENT;
    }

    /* Liveness gate: a revoked endpoint returns -ENODEV on an open fd. */
    if (!node->live) {
        return -ENODEV;
    }

    if (!node->ops) {
        /* No command vtable: the canonical "inappropriate ioctl" errno. */
        return -ENOTTY;
    }

    /*
     * The hotplug file's command IS the self-locking revoke/reload machinery, so
     * its hook must run with the lock dropped (it would otherwise deadlock the
     * non-recursive mutex).  Resolving + gating the node under the lock and only
     * then dropping it is safe because such a node (the global hotplug file) is
     * never unlinked, so its pointer + ops outlive the unlocked call.
     */
    if (node->ioctl_unlocked) {
        Node *n = node;
        NodeOps *ops = node->ops.get();
        lock.unlock();
        return ops->ioctl(*n, cmd, in, in_size, out, out_size);
    }

    return node->ops->ioctl(*node, cmd, in, in_size, out, out_size);
}

/* ================================================================== */
/* Revocation (forced removal)                                        */
/* ================================================================== */

void NodeTree::recordInvalidationLocked(Node *node,
                                        std::vector<PendingInvalidation> &pending)
{
    if (node->parent == nullptr || node->unlinked) {
        return;
    }

    pending.push_back(
        PendingInvalidation{node->parent->ino, node->ino, node->name});
}

void NodeTree::revokeSubtreeLocked(Node *node,
                                   std::vector<PendingInvalidation> &pending)
{
    /* Snapshot children: destruction below mutates the children array. */
    std::vector<Node *> kids = node->children;
    for (Node *child : kids) {
        revokeSubtreeLocked(child, pending);
    }

    recordInvalidationLocked(node, pending);

    node->live = false;
    node->unlinked = true;

    /*
     * All children have been revoked (and possibly freed) above, so drop our
     * references to them.  This prevents a surviving (still-looked-up) dead node
     * from ever iterating dangling child pointers in readdir.
     */
    node->children.clear();

    if (node->lookup_count == 0) {
        /* destroyNodeLocked detaches us from our parent's child list. */
        destroyNodeLocked(node);
    } else if (node->parent != nullptr) {
        /*
         * The kernel still references this node; it survives as a dead orphan
         * until forgotten.  Detach it from its parent now so a later reap (after
         * the parent may itself be gone) never touches a stale parent.
         */
        auto &pk = node->parent->children;
        pk.erase(std::remove(pk.begin(), pk.end(), node), pk.end());
        node->parent = nullptr;
    }
}

void NodeTree::markFunctionRemovedLocked(Device &dev, DeviceFunction func)
{
    dev.removed_functions_ |= deviceFunctionMask(func);

    if ((dev.removed_functions_ & kAllDeviceFunctions) != kAllDeviceFunctions) {
        return;
    }

    if (dev.model_shutdown_fired_) {
        return;
    }
    dev.model_shutdown_fired_ = true;

    if (dev.model_shutdown_) {
        dev.model_shutdown_(dev);
    }
}

int NodeTree::revokeDevice(const std::string &bdf)
{
    std::vector<PendingInvalidation> pending;

    {
        std::scoped_lock lock(mutex_);

        Device *dev = findDeviceLocked(bdf);
        if (dev == nullptr) {
            /* Idempotent: already revoked or never existed. */
            return 0;
        }

        dev->live = false;

        /*
         * 1. Tear down every registered resource (idempotent) and drop the
         *    registry references.  Clearing registry_ drops each registry-side
         *    shared_ptr; a resource with no surviving inode ref is freed here,
         *    one still inode-held survives as a dead orphan.
         */
        for (const auto &res : dev->registry_) {
            res->teardown();
        }
        dev->registry_.clear();

        /*
         * 2. Revoke the endpoint subtree: mark every node dead + unlinked, sever
         *    the links, reap nodes the kernel has already forgotten, and gather
         *    the names to invalidate.  revokeSubtreeLocked may free dev->dir (and
         *    dev->bars/qdma); the device keeps its (now stale) node pointers but
         *    is already marked dead, so it is never found again.  Clear them
         *    defensively.
         */
        if (dev->dir != nullptr) {
            revokeSubtreeLocked(dev->dir, pending);
        }
        dev->dir = nullptr;
        dev->bars = nullptr;
        dev->qdma = nullptr;

        /*
         * 3. A whole-device revoke removes both functions at once; mark them
         *    removed and fire the model-shutdown seam exactly once (TOGGLE_SBR /
         *    HOTPLUG / full teardown all funnel through here).
         */
        markFunctionRemovedLocked(*dev, DeviceFunction::Qdma);
        markFunctionRemovedLocked(*dev, DeviceFunction::Bars);
    }

    /*
     * 4. Fire the notifier with the lock dropped.  notify_delete forces the
     *    kernel dentry cache to forget the names so a fresh lookup misses.
     */
    if (notifier_) {
        for (const auto &p : pending) {
            notifier_(p.parent, p.child, p.name);
        }
    }

    return 0;
}

int NodeTree::revokeFunction(const std::string &bdf, DeviceFunction func)
{
    if (func != DeviceFunction::Qdma && func != DeviceFunction::Bars) {
        return -1;
    }

    std::vector<PendingInvalidation> pending;

    {
        std::scoped_lock lock(mutex_);

        Device *dev = findDeviceLocked(bdf);
        if (dev == nullptr) {
            /* Idempotent: absent or already fully revoked. */
            return 0;
        }

        /* Idempotent: this function is already removed. */
        if (dev->removed_functions_ & deviceFunctionMask(func)) {
            return 0;
        }

        /* The subtree node for this function (may already be NULL on a
         * torn-down device, in which case there is nothing left to revoke). */
        Node *subtree =
            func == DeviceFunction::Qdma ? dev->qdma : dev->bars;

        /*
         * QDMA owns the registered resources (the qpairs); tear them down on a
         * function-1 removal and leave them untouched on a function-2 removal.
         */
        if (func == DeviceFunction::Qdma) {
            for (const auto &res : dev->registry_) {
                res->teardown();
            }
            dev->registry_.clear();
        }

        /*
         * Revoke just this function's subtree: mark every node dead + unlinked,
         * sever the links, reap nodes the kernel has already forgotten, and
         * gather the names to invalidate.
         */
        if (subtree != nullptr) {
            revokeSubtreeLocked(subtree, pending);
        }

        /* Clear the device's now-stale pointer to the revoked subtree (it may
         * have been freed by revokeSubtreeLocked). */
        if (func == DeviceFunction::Qdma) {
            dev->qdma = nullptr;
        } else {
            dev->bars = nullptr;
        }

        /* Record the removal and (if both gone) fire the model-shutdown seam. */
        markFunctionRemovedLocked(*dev, func);
    }

    if (notifier_) {
        for (const auto &p : pending) {
            notifier_(p.parent, p.child, p.name);
        }
    }

    return 0;
}

int NodeTree::setModelShutdown(const std::string &bdf, ModelShutdownFn fn)
{
    std::scoped_lock lock(mutex_);

    Device *dev = findDeviceLocked(bdf);
    if (dev == nullptr) {
        return -1;
    }

    dev->model_shutdown_ = std::move(fn);
    return 0;
}

std::vector<std::string> NodeTree::collectLiveBdfs()
{
    std::scoped_lock lock(mutex_);

    std::vector<std::string> out;
    for (const auto &dev : devices_) {
        if (dev->live) {
            out.push_back(dev->bdf_);
        }
    }

    return out;
}

int NodeTree::restoreFunction(const std::string &bdf, DeviceFunction func,
                              bool *rebuilt)
{
    if (rebuilt != nullptr) {
        *rebuilt = false;
    }
    if (func != DeviceFunction::Qdma && func != DeviceFunction::Bars) {
        return -1;
    }

    std::scoped_lock lock(mutex_);

    Device *dev = findDeviceLocked(bdf);
    if (dev == nullptr) {
        /* Absent or fully revoked: nothing to rediscover (the additive restore
         * pass leaves config-driven re-add to the select_new pass). */
        return 0;
    }

    /* Idempotent: only a function whose subtree was removed gets rebuilt. */
    if ((dev->removed_functions_ & deviceFunctionMask(func)) == 0) {
        return 0;
    }

    /*
     * The device must still own its <BDF>/ dir to re-anchor the subtree under it.
     * A live device always does (a whole-device revoke clears dev->dir and marks
     * it dead, so findDeviceLocked would not have returned it).  Defensive.
     */
    if (dev->dir == nullptr) {
        LOG(LOG_ERR, "Restore of '%s' function %d: device has no dir",
            bdf.c_str(), static_cast<int>(func));
        return -1;
    }

    const char *name = func == DeviceFunction::Qdma ? "qdma" : "bars";
    Node **slot = func == DeviceFunction::Qdma ? &dev->qdma : &dev->bars;

    /* The slot was NULLed by revokeFunction; rebuild the dir node. */
    Node *dir = makeDirLocked(dev->dir, name);
    *slot = dir;

    /*
     * Clear the removed bit so the function is live again, and re-arm the
     * model-shutdown seam: the device no longer has both functions gone, so a
     * future both-removed transition must fire the seam again.
     */
    dev->removed_functions_ &= ~deviceFunctionMask(func);
    dev->model_shutdown_fired_ = false;

    if (rebuilt != nullptr) {
        *rebuilt = true;
    }

    return 0;
}

} // namespace slash::emu
