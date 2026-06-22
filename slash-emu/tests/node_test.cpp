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
 * @file node_test.cpp
 * @brief Unit tests for the slash-emu spine (node.hpp): the node tree, the
 *        per-device registry, refcounted resources, and the revocation state
 *        machine.
 *
 * These exercise the C++ data model directly against slash_emu_core -- no FUSE
 * session is needed because node.cpp never touches one.  The kernel-dentry
 * notifier is replaced by a recording lambda so name-invalidation can be
 * asserted.
 *
 * Coverage:
 *   - Node-tree lookup / readdir over a materialized multi-accelerator tree.
 *   - Registry register / find / iterate.
 *   - Refcount transitions: freed only when BOTH the registry and inode refs drop.
 *   - The two idempotent teardown triggers (cooperative eviction; forced removal).
 *   - The revocation state machine: live -> removed yields ENOENT on lookup,
 *     ENODEV on ops against an already-open handle, idempotent close/teardown,
 *     and name-invalidation invoked.
 *
 * Port note (refcount model): the resource object is destroyed when BOTH the
 * registry @c shared_ptr and the inode @c shared_ptr drop.  The test counts that
 * "free" via a custom deleter installed on the resource's @c backing shared_ptr
 * (it fires exactly when the Resource object is destroyed), and counts the
 * idempotent "teardown" via the @ref slash::emu::ResourceTeardownFn.
 */

#include <gtest/gtest.h>

#include <cerrno>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "node.hpp"

using namespace slash::emu;

namespace {

// ---------------------------------------------------------------------------
// Recording notifier: captures every notify the spine fires.
// ---------------------------------------------------------------------------
struct NotifyRecord {
    Ino parent;
    Ino child;
    std::string name;
};

std::vector<NotifyRecord> g_notifications;

Notifier make_recording_notifier()
{
    g_notifications.clear();
    return [](Ino parent, Ino child, const std::string &name) {
        g_notifications.push_back({parent, child, name});
    };
}

// ---------------------------------------------------------------------------
// Recording resource callbacks: count teardown and free events per resource.
//
// teardown is the idempotent ResourceTeardownFn; free is the destruction of the
// Resource object, observed via a custom deleter on its `backing` shared_ptr.
// ---------------------------------------------------------------------------
struct ResourceProbe {
    int teardown_calls = 0;
    int free_calls = 0;
};

// Register a resource whose teardown / free both land on `probe`.  Returns the
// resource (non-owning); the inode/registry shared_ptrs are held by the spine.
Resource *register_probed(Device *dev, uint32_t id, ResourceProbe *probe)
{
    Resource *res = dev->registerResource(
        id, probe != nullptr ? [probe](Resource &) { probe->teardown_calls++; }
                             : ResourceTeardownFn{});
    if (res != nullptr && probe != nullptr) {
        // The custom deleter fires when the Resource object is destroyed (both
        // refs dropped) -- our "free" event.  The void payload is unused.
        res->backing = std::shared_ptr<void>(
            probe, [](void *p) { static_cast<ResourceProbe *>(p)->free_calls++; });
    }
    return res;
}

// RAII wrapper around a tree so each test cleans up.
class Tree {
public:
    explicit Tree(bool with_notifier = false)
        : tree_(with_notifier ? make_recording_notifier() : Notifier{})
    {
    }

    NodeTree *get() { return &tree_; }

private:
    NodeTree tree_;
};

// Collect a directory's child names (excluding "." / "..") via readdir.
std::set<std::string> readdir_names(NodeTree *tree, Ino ino)
{
    std::set<std::string> names;
    EXPECT_EQ(tree->readdir(ino,
                            [&](const std::string &name, Ino, NodeType) {
                                if (name != "." && name != "..") {
                                    names.insert(name);
                                }
                                return true;
                            }),
              0);
    return names;
}

}  // namespace

// ===========================================================================
// Node tree: materialization, lookup, readdir
// ===========================================================================

TEST(NodeTree, RootExistsAndIsEmptyInitially)
{
    Tree t;
    struct stat st{};
    ASSERT_EQ(t.get()->stat(kRootIno, &st), 0);
    EXPECT_TRUE(S_ISDIR(st.st_mode));
    EXPECT_TRUE(readdir_names(t.get(), kRootIno).empty());
}

TEST(NodeTree, AddDeviceMaterializesBarsAndQdma)
{
    Tree t;
    Device *dev = t.get()->addDevice("0000:61:00");
    ASSERT_NE(dev, nullptr);

    // Root now lists the BDF dir.
    auto root_kids = readdir_names(t.get(), kRootIno);
    EXPECT_EQ(root_kids.count("0000:61:00"), 1u);

    // <BDF>/ lists bars and qdma.
    struct stat dir_st{};
    ASSERT_EQ(t.get()->stat(dev->dir->ino, &dir_st), 0);
    EXPECT_TRUE(S_ISDIR(dir_st.st_mode));

    auto dev_kids = readdir_names(t.get(), dev->dir->ino);
    EXPECT_EQ(dev_kids.count("bars"), 1u);
    EXPECT_EQ(dev_kids.count("qdma"), 1u);
    EXPECT_EQ(dev_kids.size(), 2u);
}

TEST(NodeTree, AddDeviceIsIdempotentPerBdf)
{
    Tree t;
    Device *a = t.get()->addDevice("0000:61:00");
    Device *b = t.get()->addDevice("0000:61:00");
    EXPECT_EQ(a, b);
    EXPECT_EQ(readdir_names(t.get(), kRootIno).size(), 1u);
}

TEST(NodeTree, MultipleAcceleratorsCoexist)
{
    Tree t;
    ASSERT_NE(t.get()->addDevice("0000:61:00"), nullptr);
    ASSERT_NE(t.get()->addDevice("0000:62:00"), nullptr);
    ASSERT_NE(t.get()->addDevice("0000:63:00"), nullptr);

    auto kids = readdir_names(t.get(), kRootIno);
    EXPECT_EQ(kids.size(), 3u);
    EXPECT_EQ(kids.count("0000:61:00"), 1u);
    EXPECT_EQ(kids.count("0000:62:00"), 1u);
    EXPECT_EQ(kids.count("0000:63:00"), 1u);
}

TEST(NodeTree, LookupResolvesChildAndBumpsLookupCount)
{
    Tree t;
    Device *dev = t.get()->addDevice("0000:61:00");
    ASSERT_NE(dev, nullptr);

    Node *child = nullptr;
    ASSERT_EQ(t.get()->lookupChild(kRootIno, "0000:61:00", &child), 0);
    ASSERT_NE(child, nullptr);
    EXPECT_EQ(child->ino, dev->dir->ino);

    // bars under the device.
    Node *bars = nullptr;
    ASSERT_EQ(t.get()->lookupChild(dev->dir->ino, "bars", &bars), 0);
    EXPECT_EQ(bars->ino, dev->bars->ino);
}

TEST(NodeTree, LookupMissReturnsEnoent)
{
    Tree t;
    ASSERT_NE(t.get()->addDevice("0000:61:00"), nullptr);
    Node *child = nullptr;
    EXPECT_EQ(t.get()->lookupChild(kRootIno, "nope", &child), -ENOENT);
}

TEST(NodeTree, LookupUnderFileReturnsEnotdir)
{
    Tree t;
    Device *dev = t.get()->addDevice("0000:61:00");
    ASSERT_NE(dev, nullptr);

    // Attach a file under <BDF>/ and try to look up inside it.
    Node *file = t.get()->createChild(dev->dir, "info", NodeType::File, 0444);
    ASSERT_NE(file, nullptr);
    Node *child = nullptr;
    EXPECT_EQ(t.get()->lookupChild(file->ino, "x", &child), -ENOTDIR);
}

TEST(NodeTree, CreateChildFileReportsSize)
{
    Tree t;
    Device *dev = t.get()->addDevice("0000:61:00");
    ASSERT_NE(dev, nullptr);

    class SizeOps : public NodeOps {
    public:
        off_t size(const Node &) const override { return 4242; }
    };
    Node *file = t.get()->createChild(dev->dir, "info", NodeType::File, 0444,
                                      std::make_unique<SizeOps>());
    ASSERT_NE(file, nullptr);

    struct stat st{};
    ASSERT_EQ(t.get()->stat(file->ino, &st), 0);
    EXPECT_TRUE(S_ISREG(st.st_mode));
    EXPECT_EQ(st.st_size, 4242);
}

TEST(NodeTree, SetOpsAttachesToBareNodeAndRejectsDoubleAttach)
{
    // The qdma endpoint attaches an ioctl handler onto the EXISTING qdma/ dir
    // node (materialized by addDevice without ops), exactly like the C
    // emu_node_set_ops did.  Attaching once succeeds; a second attach is rejected
    // (one owner only).
    Tree t;
    Device *dev = t.get()->addDevice("0000:61:00");
    ASSERT_NE(dev, nullptr);
    ASSERT_NE(dev->qdma, nullptr);

    // A handler whose ioctl is observable so we can prove dispatch reaches it.
    class CmdOps : public NodeOps {
    public:
        int ioctl(Node &, unsigned int, const void *, size_t, void *,
                  size_t) override
        {
            return 0;
        }
    };

    // The bare qdma/ dir node has no ops yet: dispatch yields -ENOTTY.
    Node *qd = nullptr;
    ASSERT_EQ(t.get()->lookupChild(dev->dir->ino, "qdma", &qd), 0);
    EXPECT_EQ(t.get()->ioctl(qd->ino, 0, nullptr, 0, nullptr, 0), -ENOTTY);

    // Attach succeeds, and the handler now serves the ioctl.
    ASSERT_EQ(t.get()->setOps(dev->qdma, std::make_unique<CmdOps>()), 0);
    EXPECT_EQ(t.get()->ioctl(qd->ino, 0, nullptr, 0, nullptr, 0), 0);

    // A second attach is rejected (the node already owns ops) and does not
    // disturb the installed handler.
    EXPECT_EQ(t.get()->setOps(dev->qdma, std::make_unique<CmdOps>()), -1);
    EXPECT_EQ(t.get()->ioctl(qd->ino, 0, nullptr, 0, nullptr, 0), 0);

    // A null node is a clean error, not a crash.
    EXPECT_EQ(t.get()->setOps(nullptr, std::make_unique<CmdOps>()), -1);

    t.get()->forget(qd->ino, 1);
}

// ===========================================================================
// Registry: register / find / iterate
// ===========================================================================

TEST(Registry, RegisterAndFind)
{
    Tree t;
    Device *dev = t.get()->addDevice("0000:61:00");
    ASSERT_NE(dev, nullptr);

    Resource *r0 = dev->registerResource(0, {});
    Resource *r1 = dev->registerResource(7, {});
    ASSERT_NE(r0, nullptr);
    ASSERT_NE(r1, nullptr);

    EXPECT_EQ(dev->findResource(0), r0);
    EXPECT_EQ(dev->findResource(7), r1);
    EXPECT_EQ(dev->findResource(99), nullptr);
}

TEST(Registry, FindsNamelessResourceNotInTree)
{
    // A registered resource that is NOT attached to any tree node (the qpair is
    // unlinked-while-open and nameless) is still reachable via the registry.
    Tree t;
    Device *dev = t.get()->addDevice("0000:61:00");
    ASSERT_NE(dev, nullptr);

    Resource *r = dev->registerResource(3, {});
    ASSERT_NE(r, nullptr);
    // qdma/ has no child for it -> truly nameless.
    EXPECT_TRUE(readdir_names(t.get(), dev->qdma->ino).empty());
    // ... yet the registry finds it.
    EXPECT_EQ(dev->findResource(3), r);
}

TEST(Registry, TreeDestructionFreesLiveResources)
{
    // A registered, never-closed resource (e.g. a leaked qpair from a crashed
    // client) must have its backing freed when the daemon tears down the tree.
    ResourceProbe probe;
    {
        Tree t;
        Device *dev = t.get()->addDevice("0000:61:00");
        ASSERT_NE(dev, nullptr);
        ASSERT_NE(register_probed(dev, 0, &probe), nullptr);
        // Leave it registered and open; t goes out of scope here.
    }
    EXPECT_EQ(probe.free_calls, 1) << "tree destruction must free the backing";
}

TEST(Registry, RegisterOnRevokedDeviceFails)
{
    Tree t;
    Device *dev = t.get()->addDevice("0000:61:00");
    ASSERT_NE(dev, nullptr);
    ASSERT_EQ(t.get()->revokeDevice("0000:61:00"), 0);

    EXPECT_EQ(dev->registerResource(0, {}), nullptr);
}

// ===========================================================================
// Refcount: freed only when BOTH registry and inode refs drop
// ===========================================================================

TEST(Refcount, FreedOnlyWhenBothRefsDrop_CooperativeLast)
{
    // registry ref + inode ref; drop registry first (revoke), then inode (forget)
    // -> freed exactly once on the second drop.
    ResourceProbe probe;
    Tree t;
    Device *dev = t.get()->addDevice("0000:61:00");
    ASSERT_NE(dev, nullptr);

    Resource *res = register_probed(dev, 0, &probe);
    ASSERT_NE(res, nullptr);

    // Attach to a qpair node and look it up (kernel ref); the still-looked-up
    // node holds the inode ref across the revoke.
    Node *qp = t.get()->createChild(dev->qdma, "qpair0", NodeType::File, 0644);
    ASSERT_NE(qp, nullptr);
    ASSERT_EQ(t.get()->attachResource(qp, res->shared_from_this()), 0);

    Node *looked = nullptr;
    ASSERT_EQ(t.get()->lookupChild(dev->qdma->ino, "qpair0", &looked), 0);

    // Forced removal: drops the registry ref + tears down, but the inode ref
    // (held by the still-looked-up node) keeps the object alive.
    ASSERT_EQ(t.get()->revokeDevice("0000:61:00"), 0);
    EXPECT_EQ(probe.teardown_calls, 1);
    EXPECT_EQ(probe.free_calls, 0) << "freed too early: inode ref still held";

    // The holder finally lets go (forget drops the inode ref) -> freed once.
    t.get()->forget(qp->ino, 1);
    EXPECT_EQ(probe.free_calls, 1);
    EXPECT_EQ(probe.teardown_calls, 1) << "teardown must not run twice";
}

TEST(Refcount, FreedOnlyWhenBothRefsDrop_RegistryLast)
{
    // Drop the inode ref first (cooperative eviction), then leave the registry
    // ref to be dropped by revoke -> teardown runs on the FIRST trigger
    // (cooperative), free happens when the registry ref also drops.
    ResourceProbe probe;
    Tree t;
    Device *dev = t.get()->addDevice("0000:61:00");
    ASSERT_NE(dev, nullptr);

    Resource *res = register_probed(dev, 0, &probe);
    ASSERT_NE(res, nullptr);

    Node *qp = t.get()->createChild(dev->qdma, "qpair0", NodeType::File, 0644);
    ASSERT_NE(qp, nullptr);
    ASSERT_EQ(t.get()->attachResource(qp, res->shared_from_this()), 0);

    Node *looked = nullptr;
    ASSERT_EQ(t.get()->lookupChild(dev->qdma->ino, "qpair0", &looked), 0);
    // Drop the kernel lookup; node is still linked, so it is NOT destroyed yet.
    t.get()->forget(qp->ino, 1);
    EXPECT_EQ(probe.teardown_calls, 0);
    EXPECT_EQ(probe.free_calls, 0);

    // Now forced removal drops both the inode ref (node destroyed) and the
    // registry ref -> teardown once, freed once.
    ASSERT_EQ(t.get()->revokeDevice("0000:61:00"), 0);
    EXPECT_EQ(probe.teardown_calls, 1);
    EXPECT_EQ(probe.free_calls, 1);
}

// ===========================================================================
// Idempotent teardown triggers
// ===========================================================================

TEST(Teardown, CooperativeEvictionRunsTeardownOnce)
{
    // Pure cooperative trigger: a qpair is registered, attached, then unlinked
    // while its fd is open (nameless), and finally closed.  Teardown runs once on
    // the last forget and the object is freed (both refs gone) -- no forced
    // revoke involved.
    ResourceProbe probe;
    Tree t;
    Device *dev = t.get()->addDevice("0000:61:00");
    ASSERT_NE(dev, nullptr);

    Resource *res = register_probed(dev, 0, &probe);
    ASSERT_NE(res, nullptr);
    Node *qp = t.get()->createChild(dev->qdma, "qpair0", NodeType::File, 0644);
    ASSERT_NE(qp, nullptr);
    ASSERT_EQ(t.get()->attachResource(qp, res->shared_from_this()), 0);

    // Open it (kernel lookup), then unlink-while-open: it becomes nameless but
    // is still found via the registry, and is NOT yet torn down.
    Node *looked = nullptr;
    ASSERT_EQ(t.get()->lookupChild(dev->qdma->ino, "qpair0", &looked), 0);
    Ino qp_ino = qp->ino;
    t.get()->unlink(qp);
    EXPECT_TRUE(readdir_names(t.get(), dev->qdma->ino).empty());
    EXPECT_EQ(dev->findResource(0), res) << "still in registry";
    EXPECT_EQ(probe.teardown_calls, 0);
    EXPECT_EQ(probe.free_calls, 0);

    // Last close -> cooperative teardown: once, then freed.
    t.get()->forget(qp_ino, 1);
    EXPECT_EQ(probe.teardown_calls, 1);
    EXPECT_EQ(probe.free_calls, 1);
}

TEST(Teardown, UnlinkedQpairStaysLiveUntilClosed)
{
    // A normally-unlinked qpair (not revoked) keeps working until its fd closes:
    // ops still succeed (it is live), unlike the forced-revoke case.
    Tree t;
    Device *dev = t.get()->addDevice("0000:61:00");
    ASSERT_NE(dev, nullptr);

    Resource *res = dev->registerResource(0, {});
    ASSERT_NE(res, nullptr);
    Node *qp = t.get()->createChild(dev->qdma, "qpair0", NodeType::File, 0644);
    ASSERT_NE(qp, nullptr);
    ASSERT_EQ(t.get()->attachResource(qp, res->shared_from_this()), 0);
    Node *looked = nullptr;
    ASSERT_EQ(t.get()->lookupChild(dev->qdma->ino, "qpair0", &looked), 0);

    t.get()->unlink(qp);
    // Still live: a normal unlink is delete-on-last-close, not revocation.
    EXPECT_EQ(t.get()->resourceCheck(res), 0);

    t.get()->forget(qp->ino, 1);  // close -> freed
}

TEST(Teardown, BothTriggersAreIdempotent)
{
    // Fire BOTH teardown triggers on one resource -- forced removal (registry
    // ref) while the qpair fd is still open, then cooperative eviction (inode
    // ref) on forget.  Teardown must run exactly once and the object be freed
    // exactly once.
    ResourceProbe probe;
    Tree t;
    Device *dev = t.get()->addDevice("0000:61:00");
    ASSERT_NE(dev, nullptr);

    Resource *res = register_probed(dev, 0, &probe);
    ASSERT_NE(res, nullptr);
    Node *qp = t.get()->createChild(dev->qdma, "qpair0", NodeType::File, 0644);
    ASSERT_NE(qp, nullptr);
    ASSERT_EQ(t.get()->attachResource(qp, res->shared_from_this()), 0);

    Node *looked = nullptr;
    ASSERT_EQ(t.get()->lookupChild(dev->qdma->ino, "qpair0", &looked), 0);
    ASSERT_EQ(t.get()->revokeDevice("0000:61:00"), 0);  // trigger 1 (forced)
    t.get()->forget(qp->ino, 1);                        // trigger 2 (coop)
    EXPECT_EQ(probe.teardown_calls, 1) << "teardown must be idempotent";
    EXPECT_EQ(probe.free_calls, 1);
}

TEST(Teardown, RevokeIsIdempotent)
{
    Tree t;
    ASSERT_NE(t.get()->addDevice("0000:61:00"), nullptr);
    EXPECT_EQ(t.get()->revokeDevice("0000:61:00"), 0);
    EXPECT_EQ(t.get()->revokeDevice("0000:61:00"), 0);  // no-op
    EXPECT_EQ(t.get()->revokeDevice("0000:99:00"), 0);  // never existed
}

// ===========================================================================
// Revocation state machine
// ===========================================================================

TEST(Revocation, LookupOfRemovedEndpointReturnsEnoent)
{
    Tree t;
    ASSERT_NE(t.get()->addDevice("0000:61:00"), nullptr);

    // Present before revoke.
    Node *child = nullptr;
    ASSERT_EQ(t.get()->lookupChild(kRootIno, "0000:61:00", &child), 0);
    t.get()->forget(child->ino, 1);

    ASSERT_EQ(t.get()->revokeDevice("0000:61:00"), 0);

    // New lookup misses.
    EXPECT_EQ(t.get()->lookupChild(kRootIno, "0000:61:00", &child), -ENOENT);
    // ... and it is gone from readdir.
    EXPECT_TRUE(readdir_names(t.get(), kRootIno).empty());
}

TEST(Revocation, OpOnOpenHandleOfRemovedEndpointReturnsEnodev)
{
    Tree t;
    Device *dev = t.get()->addDevice("0000:61:00");
    ASSERT_NE(dev, nullptr);

    // A node-backed endpoint (e.g. bars/bar0) with an outstanding open handle.
    Node *bar = t.get()->createChild(dev->bars, "bar0", NodeType::File, 0644);
    ASSERT_NE(bar, nullptr);
    Node *opened = nullptr;
    ASSERT_EQ(t.get()->lookupChild(dev->bars->ino, "bar0", &opened), 0);
    // Live before revoke.
    EXPECT_EQ(t.get()->isLive(bar->ino), 0);

    ASSERT_EQ(t.get()->revokeDevice("0000:61:00"), 0);

    // The fd is still open (lookup not yet forgotten): ops must see ENODEV.
    EXPECT_EQ(t.get()->isLive(bar->ino), -ENODEV);
}

TEST(Revocation, ResourceOpReturnsEnodevAfterRevoke)
{
    Tree t;
    Device *dev = t.get()->addDevice("0000:61:00");
    ASSERT_NE(dev, nullptr);

    Resource *res = dev->registerResource(0, {});
    ASSERT_NE(res, nullptr);
    Node *qp = t.get()->createChild(dev->qdma, "qpair0", NodeType::File, 0644);
    ASSERT_NE(qp, nullptr);
    ASSERT_EQ(t.get()->attachResource(qp, res->shared_from_this()), 0);
    Node *looked = nullptr;
    ASSERT_EQ(t.get()->lookupChild(dev->qdma->ino, "qpair0", &looked), 0);

    EXPECT_EQ(t.get()->resourceCheck(res), 0);
    ASSERT_EQ(t.get()->revokeDevice("0000:61:00"), 0);
    // res is still alive (inode ref) but dead -> ENODEV.
    EXPECT_EQ(t.get()->resourceCheck(res), -ENODEV);

    t.get()->forget(qp->ino, 1);  // release the orphan
}

TEST(Revocation, NameInvalidationIsInvoked)
{
    Tree t(/*with_notifier=*/true);
    Device *dev = t.get()->addDevice("0000:61:00");
    ASSERT_NE(dev, nullptr);
    Ino bdf_ino = dev->dir->ino;

    ASSERT_EQ(t.get()->revokeDevice("0000:61:00"), 0);

    // The <BDF> dir (and its bars/qdma children) must have been invalidated.
    bool saw_bdf = false;
    for (const auto &rec : g_notifications) {
        if (rec.name == "0000:61:00" && rec.parent == kRootIno &&
            rec.child == bdf_ino) {
            saw_bdf = true;
        }
    }
    EXPECT_TRUE(saw_bdf) << "the BDF entry was not invalidated";
    // bars + qdma children invalidated too.
    std::set<std::string> invalidated;
    for (const auto &rec : g_notifications) {
        invalidated.insert(rec.name);
    }
    EXPECT_EQ(invalidated.count("bars"), 1u);
    EXPECT_EQ(invalidated.count("qdma"), 1u);
}

TEST(Revocation, StaleParentLookupReturnsEnoent)
{
    // After a device is revoked, a lookup using the (now stale) <BDF> inode as a
    // parent must not crash; the inode may be gone -> ESTALE, mapped to ENOENT by
    // the FUSE layer.  Here we assert it does not resolve a live child.
    Tree t;
    Device *dev = t.get()->addDevice("0000:61:00");
    ASSERT_NE(dev, nullptr);
    Ino bars_ino = dev->bars->ino;
    ASSERT_EQ(t.get()->revokeDevice("0000:61:00"), 0);

    Node *child = nullptr;
    int ret = t.get()->lookupChild(bars_ino, "bar0", &child);
    EXPECT_TRUE(ret == -ESTALE || ret == -ENOENT) << "unexpected ret=" << ret;
}
