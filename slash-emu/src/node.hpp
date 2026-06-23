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
 * @file node.hpp
 * @brief The spine of the slash-emu FUSE daemon: node tree, per-device registry,
 *        refcounted resources, and the revocation state machine (C++20).
 *
 * This module owns the data model that the FUSE layer (@ref fs.hpp) and the four
 * endpoint tasks (info / bars / qdma / hotplug) build on.  It is written to be
 * unit-testable in isolation: it never touches a @c fuse_session or a
 * @c fuse_req directly.  The one effect it must push to the kernel -- dentry
 * invalidation on revocation -- is delegated through a small @ref Notifier
 * callback that the FUSE layer supplies (and tests stub).
 *
 * @section model Three layers
 *
 *   1. @b Node @b tree (@ref Node, @ref NodeTree).  A generic tree of inodes
 *      mirroring the on-disk layout: a root, one @c <BDF>/ directory per
 *      accelerator, and @c bars/ + @c qdma/ subdirectories.  Each node carries a
 *      type, a parent/children topology, a per-node ops hook (@ref NodeOps) plus
 *      opaque backing (so endpoints attach @c info, @c bar<M>, @c qpair<Q>
 *      behaviour without reworking the tree), and a @em live flag driving
 *      revocation.  An inode->node table makes lookup fast.
 *
 *   2. @b Per-device @b registry (@ref Device).  One per accelerator, tracking
 *      the live communication resources of that device.  Crucially it holds
 *      @em nameless qpairs: a qpair is unlinked from @c qdma/ while still open
 *      (delete-on-last-close), so it cannot be found by walking the tree.  Forced
 *      device removal (@ref NodeTree::revokeDevice) reaches every live resource
 *      through this registry.
 *
 *   3. @b Refcounted @b resource (@ref Resource).  A primitive whose lifetime is
 *      decoupled from the inode.  Two @c std::shared_ptr references exist by
 *      construction -- one held by the registry, one by the inode -- and the
 *      object is freed only when @em both drop.  Its teardown (stop the queue,
 *      free the QID) is @em idempotent and has two triggers: cooperative inode
 *      eviction (last close of an undisturbed resource) and forced device
 *      removal.  The first trigger does the work; the second is a no-op.
 *
 * @section refcount Refcount model (port note)
 *
 * The C daemon hand-rolled an integer @c refcount with a registry ref and an
 * inode ref.  The C++ port maps the @em free to @c std::shared_ptr ownership
 * (the @ref Device registry holds one @c shared_ptr; the @ref Node holds the
 * other), so the @ref Resource object is destroyed exactly when both drop.  The
 * @em teardown (stop queue / free QID) stays an explicit, idempotent
 * @ref Resource::teardown call fired on the first of {cooperative eviction,
 * forced removal} -- behaviour identical to the C version, only the free
 * mechanism changed.
 *
 * @section revocation Revocation semantics (enforced by the conformance suite)
 *
 * On forced removal (@ref NodeTree::revokeDevice), for the affected device:
 *   - resources are eagerly torn down (queues stopped, QIDs freed);
 *   - every open handle is marked dead, so any subsequent op returns @c -ENODEV
 *     (@ref NodeTree::isLive);
 *   - the endpoint names are invalidated via the @ref Notifier
 *     (@c fuse_lowlevel_notify_delete), so the dentry cache forgets them and a
 *     new @c lookup returns @c -ENOENT;
 *   - @c close always succeeds and remains the holder's responsibility;
 *     @c release is idempotent.
 *
 * @section locking Locking model
 *
 * The whole spine is guarded by a single per-tree mutex (@ref NodeTree::mutex_).
 * The FUSE session is single-threaded today, but the data model is built to be
 * safe for a future multi-threaded session:
 *
 *   - All public tree / device / registry mutators and lookups take the lock.
 *   - @ref Resource refcounts (the @c shared_ptr copies the spine itself holds)
 *     are maintained under the same lock, so a get/put never races a teardown
 *     decision.
 *   - The lock is @em never held across a @ref Notifier call.  Name invalidation
 *     can re-enter the filesystem in the kernel and must not deadlock against an
 *     op; @ref NodeTree::revokeDevice therefore collects the names to invalidate
 *     under the lock, drops it, then fires the notifier.
 *   - Methods whose name ends in @c Locked assume the caller already holds the
 *     lock; the unsuffixed public entry points acquire it.
 */

#ifndef SLASH_EMU_NODE_HPP
#define SLASH_EMU_NODE_HPP

#include <cerrno>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <sys/stat.h>
#include <sys/types.h>

namespace slash::emu {

/** @brief Root inode number (matches libfuse's @c FUSE_ROOT_ID). */
inline constexpr uint64_t kRootIno = 1;

/**
 * @brief Inode number type.
 *
 * Layout-compatible with libfuse's @c fuse_ino_t (both @c uint64_t), so the FUSE
 * layer passes inodes through unchanged, while this module stays testable without
 * a FUSE session.
 */
using Ino = uint64_t;

class Node;
class Device;
class Resource;
class NodeTree;

/**
 * @brief The per-device QDMA sparse memory store (defined in @ref qdma_store.hpp).
 *
 * Forward-declared here so a @ref Device can own one at device scope via a
 * @c std::shared_ptr without the spine depending on the QDMA endpoint's internals.
 * The store survives a single-function QDMA REMOVE+RESCAN (it is owned by the
 * device, not the destroyed @c qdma/ node) and is freed on a whole-device revoke.
 */
struct QdmaStore;

/**
 * @brief A removable PCI function of an accelerator (hotplug REMOVE granularity).
 *
 * The ABI maps PCI function 1 to the QDMA endpoint subtree (@c qdma/) and
 * function 2 to the control-register BAR subtree (@c bars/).  REMOVE
 * (@ref NodeTree::revokeFunction) operates at this granularity -- one function at
 * a time -- whereas SBR/HOTPLUG/teardown remove the whole device
 * (@ref NodeTree::revokeDevice).
 */
enum class DeviceFunction : unsigned {
    Pf0 = 0,  /**< PCI function 0: board management, owned by the @c ami driver.
                   The daemon does not emulate it and exposes no subtree for it; a
                   REMOVE of @c <BDF>.0 is tolerated as a no-op (it changes nothing
                   in the tree).  Never passed to the spine's per-function revoke /
                   restore, which accept only @ref Qdma / @ref Bars. */
    Qdma = 1, /**< PCI function 1: the @c qdma/ subtree. */
    Bars = 2, /**< PCI function 2: the @c bars/ subtree. */
};

/** @brief Bit for @p func in a device's removed-functions mask. */
constexpr unsigned deviceFunctionMask(DeviceFunction func)
{
    return 1u << static_cast<unsigned>(func);
}

/** @brief Mask with both removable functions set (device fully removed). */
inline constexpr unsigned kAllDeviceFunctions =
    deviceFunctionMask(DeviceFunction::Qdma) |
    deviceFunctionMask(DeviceFunction::Bars);

/**
 * @brief Per-device model-shutdown seam (the vpp_emu/vpp_sim teardown hook).
 *
 * Wired onto a device via @ref Device::setModelShutdown.  The spine invokes it
 * exactly once, with the tree lock @em held, the first time @em both functions of
 * the device have been removed (whether via two per-function REMOVEs or a single
 * whole-device revoke) -- modelling "shut down the model only once both Function
 * 1 and Function 2 are gone".  It must not free the device or re-enter the
 * spine's public (re-locking) API.
 */
using ModelShutdownFn = std::function<void(Device &dev)>;

/** @brief Node kind.  Mirrors the only two file types the ABI exposes. */
enum class NodeType {
    Dir,  /**< Directory (root, @c <BDF>, @c bars, @c qdma). */
    File, /**< Regular file (@c info, @c bar<M>, @c qpair<Q>, ...). */
};

/**
 * @brief Per-node behaviour hook (implemented by endpoint tasks).
 *
 * The spine provides the topology; endpoints provide behaviour by subclassing
 * this interface and attaching an instance to a @ref Node (the node owns it).
 * All methods are optional (the base implementations are the spine defaults);
 * an endpoint overrides only what it needs.  Methods are invoked with the tree
 * lock @em held unless documented otherwise, so an override may consult its
 * (immutable) state and call the spine's @c *Locked cores without re-locking.
 */
class NodeOps {
public:
    virtual ~NodeOps() = default;

    /**
     * @brief Report the size of a file node, in bytes (default 0).
     * @param node The node being queried.
     */
    virtual off_t size(const Node &node) const
    {
        (void) node;
        return 0;
    }

    /**
     * @brief Serve a positioned read against a file node (default: @c -EIO).
     *
     * Implements @c pread(2) semantics: copy up to @p size bytes starting at byte
     * offset @p off into @p buf and return the count.  A read wholly at/after EOF
     * returns 0; a straddling read returns the available prefix (short read).
     * Invoked with the tree lock @em held, after the liveness gate has passed.
     *
     * @return Number of bytes copied (0 at/after EOF), or a negative errno.
     */
    virtual ssize_t read(const Node &node, char *buf, size_t size, off_t off)
    {
        (void) node;
        (void) buf;
        (void) size;
        (void) off;
        return -EIO;
    }

    /**
     * @brief Serve a positioned write against a file node (default: @c -EIO).
     *
     * Implements @c pwrite(2) semantics for the endpoint.  Invoked with the tree
     * lock @em held, after the liveness gate has passed.
     *
     * @return Number of bytes consumed, or a negative errno.
     */
    virtual ssize_t write(const Node &node, const char *buf, size_t size,
                          off_t off)
    {
        (void) node;
        (void) buf;
        (void) size;
        (void) off;
        return -EIO;
    }

    /**
     * @brief Serve an ioctl against a node (default: @c -ENOTTY).
     *
     * The generic ioctl seam.  @p in and @p out alias the @em same fixed-size
     * region for an @c _IOWR command; the hook reads inputs from @p in, writes
     * outputs into @p out, and returns 0.  Buffers are the FUSE layer's already
     * bounce-buffered fixed-size copies.  Invoked with the tree lock @em held
     * (unless the node opted into @ref Node::ioctlUnlocked), after the liveness
     * gate has passed.
     *
     * @return 0 on success (then @p out holds the reply), or a negative errno.
     */
    virtual int ioctl(Node &node, unsigned int cmd, const void *in,
                      size_t in_size, void *out, size_t out_size)
    {
        (void) node;
        (void) cmd;
        (void) in;
        (void) in_size;
        (void) out;
        (void) out_size;
        return -ENOTTY;
    }
};

/**
 * @brief Resource teardown callback (supplied by the resource creator).
 *
 * Invoked exactly once, the first time the resource is torn down (cooperative or
 * forced), with the tree lock @em held.  It must stop the underlying queue and
 * release any external identifier (the QID).  It must @em not free the resource
 * object itself or its backing -- the object's destructor (last @c shared_ptr
 * drop) does that.
 */
using ResourceTeardownFn = std::function<void(Resource &res)>;

/**
 * @brief A reference-counted resource whose lifetime is decoupled from any inode.
 *
 * Models the lifetime of a QDMA qpair but is a reusable primitive.  Two
 * @c std::shared_ptr references exist by construction: the @em registry reference
 * (held by @ref Device, dropped by @ref Device::unregisterResource or forced
 * revoke) and the @em inode reference (held by @ref Node, dropped when the node
 * is destroyed).  The @ref teardown callback fires on the first of {cooperative
 * eviction, forced removal}; the object is destroyed when @em both references are
 * gone.
 *
 * Created and manipulated only through @ref Device / @ref Node / @ref NodeTree
 * under the tree lock.
 */
class Resource : public std::enable_shared_from_this<Resource> {
public:
    /**
     * @brief Construct a resource (use @ref Device::registerResource instead).
     * @param dev      Owning device.
     * @param id       Stable per-device identifier (the QID).
     * @param teardown Teardown callback (stop queue, free QID), or empty.
     *
     * Resources are always heap-allocated via @c std::make_shared (the @ref
     * Device registry holds the first @c shared_ptr).  Deriving from
     * @c std::enable_shared_from_this lets a holder of a raw @ref Resource* (as
     * @ref Device::registerResource hands back) obtain the @em second co-owning
     * reference -- the inode reference -- via @c shared_from_this() to pass to
     * @ref NodeTree::attachResource, without exposing the registry's @c shared_ptr.
     */
    Resource(Device &dev, uint32_t id, ResourceTeardownFn teardown);

    Resource(const Resource &) = delete;
    Resource &operator=(const Resource &) = delete;

    /** @brief Stable per-device identifier (e.g. the QID). */
    uint32_t id() const { return id_; }

    /** @brief Owning device. */
    Device &device() const { return device_; }

    /** @brief False once revoked/torn-down: ops on open handles return -ENODEV. */
    bool live() const { return live_; }

    /**
     * @brief Run the idempotent teardown (first trigger does the work).
     *
     * Called with the tree lock @em held by the cooperative (inode eviction) and
     * forced (device revoke) paths.  Marks the resource dead and invokes the
     * teardown callback exactly once; a second call is a no-op.
     */
    void teardown();

    /** @brief Endpoint-private backing (the qpair object); ownership is the
     *         creator's, kept alive by the resource. */
    std::shared_ptr<void> backing;

private:
    friend class Device;
    friend class Node;
    friend class NodeTree;

    Device &device_;             /**< non-owning */
    uint32_t id_;
    bool torn_down_ = false;     /**< makes the second teardown a no-op */
    bool live_ = true;
    ResourceTeardownFn teardown_;
};

/**
 * @brief A single inode in the emulated tree.
 *
 * Nodes are owned by their @ref NodeTree and addressed by @ref ino.  A node may
 * be @em unlinked (removed from its parent's child list, name invalidated) while
 * still present in the inode table because the kernel still holds lookup
 * references; it is destroyed only when its lookup count drops to zero.
 *
 * The node fields are public for the spine and endpoint code that operates on the
 * tree under the lock (mirroring the C @c struct), but a node is only ever
 * created/destroyed through @ref NodeTree.
 */
class Node {
public:
    Node() = default;
    Node(const Node &) = delete;
    Node &operator=(const Node &) = delete;

    /** @brief Inode number (unique, stable for the node's lifetime). */
    Ino ino = 0;

    /** @brief Entry name within the parent directory. */
    std::string name;

    /** @brief Node kind. */
    NodeType type = NodeType::Dir;

    /** @brief POSIX permission bits (the type bits are OR'd in by the FS). */
    mode_t mode = 0;

    /** @brief Parent node, or nullptr for the root (non-owning). */
    Node *parent = nullptr;

    /** @brief Child nodes (non-owning refs; the tree owns the nodes). */
    std::vector<Node *> children;

    /** @brief Device this node belongs to, or nullptr for the root (non-owning). */
    Device *device = nullptr;

    /** @brief Per-node behaviour hook, or nullptr (owning). */
    std::unique_ptr<NodeOps> ops;

    /** @brief Resource this node holds an inode-ref on, or nullptr. */
    std::shared_ptr<Resource> resource;

    /**
     * @brief Kernel lookup count: the number of @c fuse_reply_entry replies for
     *        this node not yet balanced by a @c forget.  The node is destroyed
     *        when this reaches zero @em and the node has been unlinked.
     */
    uint64_t lookup_count = 0;

    /** @brief False once the endpoint has been revoked (forced removal). */
    bool live = true;

    /** @brief True once unlinked from its parent (name no longer resolvable). */
    bool unlinked = false;

    /**
     * @brief Whether a user @c unlink(2) may remove this file (opt-in).
     *
     * The ABI makes only the @c qpair<Q> files user-unlinkable -- the VRTD
     * delete-on-last-close nameless pattern.  Every other endpoint (@c info,
     * @c bar<M>, the global @c hotplug control file) must @em not be removable by
     * @c unlink.  Default false; the qdma endpoint sets it true per qpair.
     */
    bool unlinkable = false;

    /**
     * @brief Invoke this node's @c ops->ioctl with the tree lock @em dropped.
     *
     * Set only on the global @c hotplug file, whose command @em is the
     * self-locking revoke/reload machinery and would deadlock the non-recursive
     * mutex if re-entered under the lock.  Safe only for a node whose lifetime
     * spans the tree's and that is never unlinked/revoked (the hotplug file).
     */
    bool ioctl_unlocked = false;

    /**
     * @brief Require unbuffered (direct) I/O for this file (set by the endpoint).
     *
     * When true the FUSE layer opens the file with @c direct_io so the kernel
     * passes every read/write through with the caller's exact size and offset.
     * Register endpoints (@c bar<M>, @c qpair<Q>) need this.
     */
    bool direct_io = false;
};

/**
 * @brief Notifier callback the FUSE layer supplies for kernel dentry control.
 *
 * Lets the spine invalidate names on revocation without depending on
 * @c <fuse_lowlevel.h>.  Tests pass a callable that records calls.  Invoked with
 * the tree lock @em not held.  Signature mirrors @c fuse_lowlevel_notify_delete:
 * @c (parent_ino, child_ino, name).
 */
using Notifier = std::function<void(Ino parent, Ino child, const std::string &name)>;

/**
 * @brief Per-entry callback for @ref NodeTree::readdir.
 * @return true to continue enumeration, false to stop early.
 */
using ReaddirCb =
    std::function<bool(const std::string &name, Ino ino, NodeType type)>;

/**
 * @brief A per-accelerator device: its directory subtree and resource registry.
 *
 * The registry is the authoritative list of live resources for forced teardown,
 * including nameless (unlinked-while-open) qpairs that the tree can no longer
 * reach.  Owned by its @ref NodeTree; manipulated under the tree lock.
 */
class Device {
public:
    /**
     * @brief Construct a device (use @ref NodeTree::addDevice instead).
     * @param tree Owning tree (for locking).
     * @param bdf  Normalized board-level BDF "DDDD:BB:DD".
     */
    Device(NodeTree &tree, std::string bdf);

    /* Out-of-line (defined in qdma.cpp, where QdmaStore is complete) so the
     * device-scoped shared_ptr<QdmaStore> member can destruct against a complete
     * type despite QdmaStore being only forward-declared here. */
    ~Device();

    Device(const Device &) = delete;
    Device &operator=(const Device &) = delete;

    /** @brief Normalized board-level BDF "DDDD:BB:DD". */
    const std::string &bdf() const { return bdf_; }

    /** @brief Owning tree. */
    NodeTree &tree() const { return tree_; }

    /** @brief The @c <BDF>/ directory node (non-owning; owned by the tree). */
    Node *dir = nullptr;
    /** @brief The @c <BDF>/bars/ directory node (non-owning). */
    Node *bars = nullptr;
    /** @brief The @c <BDF>/qdma/ directory node (non-owning). */
    Node *qdma = nullptr;

    /**
     * @brief The per-device QDMA sparse memory store (HBM/DDR contents).
     *
     * Owned at @em device scope so it survives a single-function QDMA
     * REMOVE+RESCAN: @ref NodeTree::revokeFunction tears down the @c qdma/ node
     * (and its @ref QdmaDirOps co-owner) but leaves this reference intact, so the
     * rediscovered endpoint (@ref qdmaAttach) reuses the same memory.  Allocated
     * lazily by @ref qdmaAttach (the first attach creates it; a re-attach after a
     * per-function remove reuses it).  Freed on a whole-device revoke
     * (@ref NodeTree::revokeDevice clears it).  The @c qdma/ ops and every qpair
     * also co-own it through their own @c shared_ptr copies, so it additionally
     * outlives an open qpair fd across the remove.
     */
    std::shared_ptr<QdmaStore> qdmaStore;

    /** @brief False once the device has been revoked (forced removal). */
    bool live = true;

    /* ---- registry + refcounted resources (qpair lifetime) ---- */

    /**
     * @brief Create a refcounted resource and register it with this device.
     *
     * The resource starts with the @em registry reference (this device holds a
     * @c shared_ptr).  Attaching it to an inode (@ref Node::attachResource /
     * @ref NodeTree::attachResource) adds the inode reference.  Takes the tree
     * lock.
     *
     * @param id       Stable per-device id (the QID).
     * @param teardown Teardown callback (stop queue, free QID), or empty.
     * @return The resource (non-owning view), or nullptr if the device is revoked.
     */
    Resource *registerResource(uint32_t id, ResourceTeardownFn teardown);

    /** @brief Lock-held variant of @ref registerResource. */
    Resource *registerResourceLocked(uint32_t id, ResourceTeardownFn teardown);

    /** @brief Find a registered resource by id (takes the lock). */
    Resource *findResource(uint32_t id);
    /** @brief Lock-held variant of @ref findResource. */
    Resource *findResourceLocked(uint32_t id);

    /**
     * @brief Wire the model-shutdown seam (fires once both functions removed).
     * @param fn The model-shutdown callback, or empty to clear it.
     * @return 0 on success; -1 if the device is not live.
     */
    int setModelShutdown(ModelShutdownFn fn);

private:
    friend class NodeTree;
    friend class Resource;

    NodeTree &tree_;             /**< non-owning */
    std::string bdf_;

    /** @brief Live resources registered to this device (the registry refs). */
    std::vector<std::shared_ptr<Resource>> registry_;

    /** @brief Functions removed so far (OR of @ref deviceFunctionMask). */
    unsigned removed_functions_ = 0;
    /** @brief True once the model-shutdown seam has fired (fire-once guard). */
    bool model_shutdown_fired_ = false;
    /** @brief Model-shutdown seam, or empty. */
    ModelShutdownFn model_shutdown_;
};

/**
 * @brief The inode table + topology root + lock + notifier.
 *
 * Owns every @ref Node and every @ref Device.  The FUSE layer holds one and
 * threads it through its ops; endpoints attach behaviour through the API below.
 */
class NodeTree {
public:
    /**
     * @brief Construct a node tree with just the root directory.
     * @param notifier Kernel dentry notifier (copied; may be empty for tests).
     */
    explicit NodeTree(Notifier notifier = {});
    ~NodeTree();

    NodeTree(const NodeTree &) = delete;
    NodeTree &operator=(const NodeTree &) = delete;

    /* ---- tree construction (startup, RESCAN/hotplug add) ---- */

    /**
     * @brief Materialize the @c <BDF>/ + @c bars/ + @c qdma/ subtree for a device.
     *
     * Idempotent at the BDF level: if a @em live device with @p bdf already
     * exists, it is returned without creating a duplicate.  Endpoints then attach
     * their files under @c dev->bars / @c dev->qdma and @c info under @c dev->dir.
     *
     * @param bdf Normalized board-level BDF "DDDD:BB:DD".
     * @return The (new or existing) device (non-owning), or nullptr on error.
     */
    Device *addDevice(const std::string &bdf);

    /**
     * @brief Find a live device by normalized BDF.
     * @return The device (non-owning), or nullptr if absent or revoked.
     */
    Device *findDevice(const std::string &bdf);

    /**
     * @brief Create a child node under @p parent and register it in the table.
     * @param parent  Parent directory node.
     * @param name    Entry name.
     * @param type    Node kind.
     * @param mode    POSIX permission bits.
     * @param ops     Per-node ops hook (owning, may be nullptr).
     * @return The new node (non-owning), or nullptr on error.
     */
    Node *createChild(Node *parent, const std::string &name, NodeType type,
                      mode_t mode, std::unique_ptr<NodeOps> ops = nullptr);

    /** @brief Lock-held variant of @ref createChild (for ioctl hooks). */
    Node *createChildLocked(Node *parent, const std::string &name, NodeType type,
                            mode_t mode, std::unique_ptr<NodeOps> ops = nullptr);

    /**
     * @brief Unlink a single node from its parent (delete-on-last-close primitive).
     *
     * Removes @p node from its parent's listing so it is no longer resolvable by
     * name, then reaps it immediately if the kernel holds no lookups; otherwise
     * the node survives as a nameless orphan until the final @ref forget.  The
     * node stays @em live (ops still succeed) -- unlinking is not revocation.
     * Does not fire the notifier (the holder's explicit unlink already updates the
     * kernel's view).
     */
    void unlink(Node *node);
    /** @brief Lock-held variant of @ref unlink (for ioctl error unwinding). */
    void unlinkLocked(Node *node);

    /**
     * @brief Resolve a child by name within a directory and unlink it (FUSE unlink).
     *
     * Unlinkability is opt-in (@ref Node::unlinkable): only nodes the endpoint
     * marked unlinkable (the @c qpair<Q> files) may be removed this way.
     *
     * @return 0 on success; @c -ENOENT if parent/name absent; @c -ENOTDIR if
     *         @p parent is not a directory; @c -EISDIR if the target is a
     *         directory; @c -EPERM if the target file is not unlinkable.
     */
    int unlinkChild(Ino parent, const std::string &name);

    /** @brief Mark a file node as requiring unbuffered (direct) I/O (takes lock). */
    void setDirectIo(Node *node);
    /** @brief Lock-held variant of @ref setDirectIo. */
    void setDirectIoLocked(Node *node);

    /** @brief Mark a file node as user-unlinkable (opt-in; takes lock). */
    void setUnlinkable(Node *node);
    /** @brief Lock-held variant of @ref setUnlinkable. */
    void setUnlinkableLocked(Node *node);

    /** @brief Mark a node's ioctl hook to run with the tree lock dropped. */
    void setIoctlUnlocked(Node *node);

    /**
     * @brief Attach an ops handler to an already-created node that has none.
     *
     * @ref createChild wires ops at creation time, but the per-device @c bars/
     * and @c qdma/ directory nodes are materialized by @ref addDevice before any
     * endpoint exists.  This lets an endpoint (the qdma endpoint's attach)
     * attach a command vtable -- e.g. the @c QPAIR_ADD ioctl hook -- onto the
     * @c qdma/ directory after the fact.  The node must not already own ops (one
     * owner only); ownership of @p ops then follows the node.
     *
     * @param node The node to attach to (must belong to this tree).
     * @param ops  Per-node ops handler (owning, may be nullptr).
     * @return 0 on success, -1 on error (node is null or already has ops).
     */
    int setOps(Node *node, std::unique_ptr<NodeOps> ops);
    /** @brief Lock-held variant of @ref setOps (for ioctl/reload hooks). */
    int setOpsLocked(Node *node, std::unique_ptr<NodeOps> ops);

    /** @brief Query whether a file node requested direct I/O (FUSE open). */
    bool wantsDirectIo(Ino ino);

    /* ---- FUSE op support (called from fs.cpp with inode numbers) ---- */

    /** @brief Resolve an inode number to its node (non-owning), or nullptr. */
    Node *lookupIno(Ino ino);

    /**
     * @brief Resolve a name within a directory to a child node, for FUSE @c lookup.
     *
     * Honours revocation: an unlinked/dead child is not resolvable.  On success
     * the child's kernel lookup count is incremented (balanced by @ref forget).
     *
     * @param[out] out Receives the resolved child (non-owning) on success.
     * @return 0 on success; -ENOENT if no such name; -ENOTDIR if @p parent is not
     *         a directory; -ESTALE if @p parent does not exist.
     */
    int lookupChild(Ino parent, const std::string &name, Node **out);

    /**
     * @brief Decrement a node's kernel lookup count, reaping it if appropriate.
     *
     * When the count reaches zero and the node has been unlinked, the node is
     * destroyed (dropping its inode-side resource reference, which may trigger
     * cooperative teardown).
     */
    void forget(Ino ino, uint64_t nlookup);

    /**
     * @brief Snapshot a node's attributes into @p st, for FUSE @c getattr.
     * @return 0 on success; -ENOENT if the inode does not exist.
     */
    int stat(Ino ino, struct stat *st);

    /**
     * @brief Enumerate a directory's entries (".", "..", then live children).
     * @return 0 on success; -ENOENT if absent; -ENOTDIR if not a directory.
     */
    int readdir(Ino ino, const ReaddirCb &cb);

    /**
     * @brief Liveness check for an op arriving on an inode.
     * @return 0 if live; -ENODEV if revoked; -ENOENT if the inode does not exist.
     */
    int isLive(Ino ino);

    /**
     * @brief Dispatch a positioned read to a file node's ops (FUSE @c read).
     *
     * Under the tree lock resolves @p ino, enforces the liveness gate, and
     * forwards to the node's @ref NodeOps::read.
     *
     * @return Bytes copied (0 at/after EOF); -ENOENT if absent; -ENODEV if
     *         revoked; -EINVAL on a bad argument / negative offset; any negative
     *         errno the hook returns.
     */
    ssize_t pread(Ino ino, char *buf, size_t size, off_t off);

    /**
     * @brief Dispatch a positioned write to a file node's ops (FUSE @c write).
     * @return Bytes consumed; -ENOENT if absent; -ENODEV if revoked; -EINVAL on a
     *         bad argument; -EIO if the node has no write hook; any negative errno
     *         the hook returns.
     */
    ssize_t pwrite(Ino ino, const char *buf, size_t size, off_t off);

    /**
     * @brief Dispatch an ioctl to a node's ops (FUSE @c ioctl).
     *
     * @p in and @p out may alias for an @c _IOWR command.
     * @return 0 on success; -ENOENT if absent; -ENODEV if revoked; -ENOTTY if the
     *         node has no ioctl hook; any negative errno the hook returns.
     */
    int ioctl(Ino ino, unsigned int cmd, const void *in, size_t in_size,
              void *out, size_t out_size);

    /* ---- resource attach (inode reference) ---- */

    /**
     * @brief Attach a resource to a node, taking an inode-side reference (lock).
     * @return 0 on success, -1 on error (node already has a resource).
     */
    int attachResource(Node *node, std::shared_ptr<Resource> res);
    /** @brief Lock-held variant of @ref attachResource. */
    int attachResourceLocked(Node *node, std::shared_ptr<Resource> res);

    /**
     * @brief Liveness check for an op on an open resource handle (takes lock).
     * @return 0 if live; -ENODEV if torn down / revoked.
     */
    int resourceCheck(Resource *res);
    /** @brief Lock-held variant of @ref resourceCheck. */
    int resourceCheckLocked(Resource *res);

    /* ---- revocation (forced removal) ---- */

    /**
     * @brief Eagerly revoke a whole device (REMOVE / SBR / HOTPLUG / teardown).
     *
     * Atomically under the lock marks the device, its endpoint nodes, and every
     * registered resource dead; tears down every resource (idempotent) and drops
     * the registry references; unlinks the endpoint nodes so new lookups return
     * @c -ENOENT.  Then, lock dropped, invalidates the endpoint names via the
     * notifier.  Idempotent: revoking an already-revoked BDF is a no-op success.
     *
     * @return 0 on success (including the already-revoked / absent no-op case).
     */
    int revokeDevice(const std::string &bdf);

    /**
     * @brief Eagerly revoke a single function (subtree) of a device (REMOVE).
     *
     * Function 1 (@ref DeviceFunction::Qdma) revokes the @c qdma/ subtree and the
     * device's registered resources; function 2 (@ref DeviceFunction::Bars)
     * revokes the @c bars/ subtree.  The other function stays live.  The device is
     * @em not marked dead until both functions are removed, at which point the
     * model-shutdown seam fires exactly once.  Idempotent.
     *
     * @return 0 on success (including idempotent no-op); -1 on a bad argument.
     */
    int revokeFunction(const std::string &bdf, DeviceFunction func);

    /**
     * @brief Wire the model-shutdown seam onto a device by BDF.
     * @return 0 on success; -1 if no live device with @p bdf exists.
     */
    int setModelShutdown(const std::string &bdf, ModelShutdownFn fn);

    /**
     * @brief Collect the BDFs of every live device (RESCAN seeding).
     * @return The board-level BDFs of all currently-live devices.
     */
    std::vector<std::string> collectLiveBdfs();

    /**
     * @brief Rediscover (restore) a previously-removed function of a live device.
     *
     * Rebuilds just that function's directory node under the device's surviving
     * @c <BDF>/ dir and clears the removed bit so the function is live again; the
     * caller then re-attaches the endpoint files and re-wires the data plane.
     * Re-arms the model-shutdown seam.  Idempotent for a never-removed/already-
     * restored function and for an absent/fully-revoked BDF.
     *
     * @param[out] rebuilt Set true iff the subtree was actually rebuilt (may be
     *                     nullptr).
     * @return 0 on success (including idempotent no-ops); -1 on a bad argument or
     *         an allocation failure.
     */
    int restoreFunction(const std::string &bdf, DeviceFunction func,
                        bool *rebuilt);

    /** @brief The root directory node (non-owning). */
    Node *root() const { return root_; }

private:
    friend class Device;
    friend class Resource;

    /**
     * @brief A name to invalidate once the lock is dropped: parent inode, child
     *        inode, and the entry name.
     *
     * @ref revokeDevice / @ref revokeFunction collect these under the lock, then
     * fire the @ref Notifier with the lock dropped (see @ref locking).
     */
    struct PendingInvalidation {
        Ino parent;
        Ino child;
        std::string name;
    };

    /* ---- node / device allocation + lookup (assume the lock is held) ---- */

    /** @brief Allocate a node, assign a fresh inode, register it in the table;
     *         not yet linked into any parent. */
    Node *allocNodeLocked(const std::string &name, NodeType type, mode_t mode,
                          std::unique_ptr<NodeOps> ops);
    /** @brief Resolve an inode to its node, or nullptr. */
    Node *findInoLocked(Ino ino) const;
    /** @brief Resolve a live (non-unlinked) child by name within a directory. */
    Node *findChildLocked(const Node *parent, const std::string &name) const;
    /** @brief Link an already-allocated node under a parent directory. */
    void linkChildLocked(Node *parent, Node *child);
    /** @brief Find a live device by BDF, or nullptr. */
    Device *findDeviceLocked(const std::string &bdf) const;
    /** @brief Create a child directory under @p parent and return it. */
    Node *makeDirLocked(Node *parent, const std::string &name);

    /* ---- destruction + revocation (assume the lock is held) ---- */

    /** @brief Remove a resource from its device's registry, dropping the
     *         registry-side @c shared_ptr (no-op if already unregistered). */
    void unregisterResourceLocked(Resource *res);
    /** @brief Destroy a node: run cooperative teardown, drop both its registry
     *         and inode resource refs, detach from the table, free the shell
     *         (runs the ops destructor). */
    void destroyNodeLocked(Node *node);
    /** @brief Record one (parent, child, name) invalidation for the notifier. */
    void recordInvalidationLocked(Node *node,
                                  std::vector<PendingInvalidation> &pending);
    /** @brief Revoke an entire subtree rooted at @p node (mark dead+unlinked,
     *         sever links, reap forgotten nodes, gather invalidation names). */
    void revokeSubtreeLocked(Node *node,
                             std::vector<PendingInvalidation> &pending);
    /** @brief Mark @p func removed on @p dev; fire the model-shutdown seam once
     *         both functions are gone. */
    void markFunctionRemovedLocked(Device &dev, DeviceFunction func);

    /* The lock is mutable so const-looking lookups can still take it; the public
     * API is not const, so this is just for internal helpers. */
    std::mutex mutex_;
    Notifier notifier_;

    std::vector<std::unique_ptr<Node>> nodes_;     /**< owns every node */
    std::vector<std::unique_ptr<Device>> devices_; /**< owns every device */
    Node *root_ = nullptr;                         /**< element of nodes_ */
    Ino next_ino_ = kRootIno;
};

} // namespace slash::emu

#endif // SLASH_EMU_NODE_HPP
