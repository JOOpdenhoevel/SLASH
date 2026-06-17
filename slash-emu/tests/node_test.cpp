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
 * @brief Unit tests for the slash-emu spine (node.h): the node tree, the
 *        per-device registry, refcounted resources, and the revocation state
 *        machine.
 *
 * These exercise the pure C data structures directly against slash_emu_core --
 * no FUSE session is needed because node.c never touches one.  The kernel-dentry
 * notifier is replaced by a recording stub so name-invalidation can be asserted.
 *
 * Coverage:
 *   - Node-tree lookup / readdir over a materialized multi-accelerator tree.
 *   - Registry register / find / iterate.
 *   - Refcount transitions: freed only when BOTH the registry and inode refs drop.
 *   - The two idempotent teardown triggers (cooperative eviction; forced removal).
 *   - The revocation state machine: live -> removed yields ENOENT on lookup,
 *     ENODEV on ops against an already-open handle, idempotent close/teardown,
 *     and name-invalidation invoked.
 */

#include <gtest/gtest.h>

#include <cerrno>
#include <set>
#include <string>
#include <vector>

extern "C" {
#include "node.h"
}

namespace {

// ---------------------------------------------------------------------------
// Recording notifier: captures every notify_delete the spine fires.
// ---------------------------------------------------------------------------
struct NotifyRecord {
    emu_ino_t parent;
    emu_ino_t child;
    std::string name;
};

std::vector<NotifyRecord> g_notifications;

void recording_notify_delete(void *ctx, emu_ino_t parent, emu_ino_t child,
                             const char *name)
{
    (void) ctx;
    g_notifications.push_back({parent, child, std::string(name)});
}

emu_notifier make_recording_notifier()
{
    g_notifications.clear();
    emu_notifier n{};
    n.notify_delete = recording_notify_delete;
    n.ctx = nullptr;
    return n;
}

// ---------------------------------------------------------------------------
// Recording resource callbacks: count teardown and free events per resource.
// ---------------------------------------------------------------------------
struct ResourceProbe {
    int teardown_calls = 0;
    int free_calls = 0;
};

void probe_teardown(struct emu_resource *res, void *backing)
{
    (void) res;
    static_cast<ResourceProbe *>(backing)->teardown_calls++;
}

void probe_free(void *backing)
{
    static_cast<ResourceProbe *>(backing)->free_calls++;
}

// RAII wrapper around a tree so each test cleans up.
class Tree {
public:
    explicit Tree(bool with_notifier = false)
    {
        emu_notifier n = with_notifier ? make_recording_notifier()
                                       : emu_notifier{};
        EXPECT_EQ(emu_node_tree_new(&tree_, &n), 0);
    }
    ~Tree() { cleanup_node_tree(tree_); }

    emu_node_tree *get() { return tree_; }

private:
    emu_node_tree *tree_ = nullptr;
};

// Collect a directory's child names (excluding "." / "..") via readdir.
struct NameCollector {
    std::set<std::string> names;
};

bool collect_names(void *ctx, const char *name, emu_ino_t ino,
                   enum emu_node_type type)
{
    (void) ino;
    (void) type;
    auto *c = static_cast<NameCollector *>(ctx);
    if (std::string(name) != "." && std::string(name) != "..") {
        c->names.insert(name);
    }
    return true;
}

std::set<std::string> readdir_names(emu_node_tree *tree, emu_ino_t ino)
{
    NameCollector c;
    EXPECT_EQ(emu_node_readdir(tree, ino, collect_names, &c), 0);
    return c.names;
}

}  // namespace

// ===========================================================================
// Node tree: materialization, lookup, readdir
// ===========================================================================

TEST(NodeTree, RootExistsAndIsEmptyInitially)
{
    Tree t;
    struct stat st{};
    ASSERT_EQ(emu_node_stat(t.get(), EMU_ROOT_INO, &st), 0);
    EXPECT_TRUE(S_ISDIR(st.st_mode));
    EXPECT_TRUE(readdir_names(t.get(), EMU_ROOT_INO).empty());
}

TEST(NodeTree, AddDeviceMaterializesBarsAndQdma)
{
    Tree t;
    emu_device *dev = nullptr;
    ASSERT_EQ(emu_node_tree_add_device(t.get(), "0000:61:00", &dev), 0);
    ASSERT_NE(dev, nullptr);

    // Root now lists the BDF dir.
    auto root_kids = readdir_names(t.get(), EMU_ROOT_INO);
    EXPECT_EQ(root_kids.count("0000:61:00"), 1u);

    // <BDF>/ lists bars and qdma.
    struct stat dir_st{};
    ASSERT_EQ(emu_node_stat(t.get(), dev->dir->ino, &dir_st), 0);
    EXPECT_TRUE(S_ISDIR(dir_st.st_mode));

    auto dev_kids = readdir_names(t.get(), dev->dir->ino);
    EXPECT_EQ(dev_kids.count("bars"), 1u);
    EXPECT_EQ(dev_kids.count("qdma"), 1u);
    EXPECT_EQ(dev_kids.size(), 2u);
}

TEST(NodeTree, AddDeviceIsIdempotentPerBdf)
{
    Tree t;
    emu_device *a = nullptr;
    emu_device *b = nullptr;
    ASSERT_EQ(emu_node_tree_add_device(t.get(), "0000:61:00", &a), 0);
    ASSERT_EQ(emu_node_tree_add_device(t.get(), "0000:61:00", &b), 0);
    EXPECT_EQ(a, b);
    EXPECT_EQ(readdir_names(t.get(), EMU_ROOT_INO).size(), 1u);
}

TEST(NodeTree, MultipleAcceleratorsCoexist)
{
    Tree t;
    ASSERT_EQ(emu_node_tree_add_device(t.get(), "0000:61:00", nullptr), 0);
    ASSERT_EQ(emu_node_tree_add_device(t.get(), "0000:62:00", nullptr), 0);
    ASSERT_EQ(emu_node_tree_add_device(t.get(), "0000:63:00", nullptr), 0);

    auto kids = readdir_names(t.get(), EMU_ROOT_INO);
    EXPECT_EQ(kids.size(), 3u);
    EXPECT_EQ(kids.count("0000:61:00"), 1u);
    EXPECT_EQ(kids.count("0000:62:00"), 1u);
    EXPECT_EQ(kids.count("0000:63:00"), 1u);
}

TEST(NodeTree, LookupResolvesChildAndBumpsLookupCount)
{
    Tree t;
    emu_device *dev = nullptr;
    ASSERT_EQ(emu_node_tree_add_device(t.get(), "0000:61:00", &dev), 0);

    emu_node *child = nullptr;
    ASSERT_EQ(emu_node_lookup_child(t.get(), EMU_ROOT_INO, "0000:61:00", &child),
              0);
    ASSERT_NE(child, nullptr);
    EXPECT_EQ(child->ino, dev->dir->ino);

    // bars under the device.
    emu_node *bars = nullptr;
    ASSERT_EQ(emu_node_lookup_child(t.get(), dev->dir->ino, "bars", &bars), 0);
    EXPECT_EQ(bars->ino, dev->bars->ino);
}

TEST(NodeTree, LookupMissReturnsEnoent)
{
    Tree t;
    ASSERT_EQ(emu_node_tree_add_device(t.get(), "0000:61:00", nullptr), 0);
    emu_node *child = nullptr;
    EXPECT_EQ(emu_node_lookup_child(t.get(), EMU_ROOT_INO, "nope", &child),
              -ENOENT);
}

TEST(NodeTree, LookupUnderFileReturnsEnotdir)
{
    Tree t;
    emu_device *dev = nullptr;
    ASSERT_EQ(emu_node_tree_add_device(t.get(), "0000:61:00", &dev), 0);

    // Attach a file under <BDF>/ and try to look up inside it.
    emu_node *file = nullptr;
    ASSERT_EQ(emu_node_create_child(t.get(), dev->dir, "info", EMU_NODE_FILE,
                                    0444, nullptr, nullptr, &file),
              0);
    emu_node *child = nullptr;
    EXPECT_EQ(emu_node_lookup_child(t.get(), file->ino, "x", &child), -ENOTDIR);
}

TEST(NodeTree, CreateChildFileReportsSize)
{
    Tree t;
    emu_device *dev = nullptr;
    ASSERT_EQ(emu_node_tree_add_device(t.get(), "0000:61:00", &dev), 0);

    static const emu_node_ops kOps = {
        /* size */ [](const emu_node *, void *) -> off_t { return 4242; },
        /* destroy */ nullptr,
    };
    emu_node *file = nullptr;
    ASSERT_EQ(emu_node_create_child(t.get(), dev->dir, "info", EMU_NODE_FILE,
                                    0444, &kOps, nullptr, &file),
              0);

    struct stat st{};
    ASSERT_EQ(emu_node_stat(t.get(), file->ino, &st), 0);
    EXPECT_TRUE(S_ISREG(st.st_mode));
    EXPECT_EQ(st.st_size, 4242);
}

// ===========================================================================
// Registry: register / find / iterate
// ===========================================================================

TEST(Registry, RegisterAndFind)
{
    Tree t;
    emu_device *dev = nullptr;
    ASSERT_EQ(emu_node_tree_add_device(t.get(), "0000:61:00", &dev), 0);

    emu_resource *r0 = nullptr;
    emu_resource *r1 = nullptr;
    ASSERT_EQ(emu_device_register_resource(dev, 0, nullptr, nullptr, nullptr,
                                           &r0),
              0);
    ASSERT_EQ(emu_device_register_resource(dev, 7, nullptr, nullptr, nullptr,
                                           &r1),
              0);

    EXPECT_EQ(emu_device_find_resource(dev, 0), r0);
    EXPECT_EQ(emu_device_find_resource(dev, 7), r1);
    EXPECT_EQ(emu_device_find_resource(dev, 99), nullptr);
}

TEST(Registry, FindsNamelessResourceNotInTree)
{
    // A registered resource that is NOT attached to any tree node (the qpair is
    // unlinked-while-open and nameless) is still reachable via the registry.
    Tree t;
    emu_device *dev = nullptr;
    ASSERT_EQ(emu_node_tree_add_device(t.get(), "0000:61:00", &dev), 0);

    emu_resource *r = nullptr;
    ASSERT_EQ(emu_device_register_resource(dev, 3, nullptr, nullptr, nullptr,
                                           &r),
              0);
    // qdma/ has no child for it -> truly nameless.
    EXPECT_TRUE(readdir_names(t.get(), dev->qdma->ino).empty());
    // ... yet the registry finds it.
    EXPECT_EQ(emu_device_find_resource(dev, 3), r);
}

TEST(Registry, TreeDestructionFreesLiveResources)
{
    // A registered, never-closed resource (e.g. a leaked qpair from a crashed
    // client) must have its backing freed when the daemon tears down the tree.
    ResourceProbe probe;
    {
        Tree t;
        emu_device *dev = nullptr;
        ASSERT_EQ(emu_node_tree_add_device(t.get(), "0000:61:00", &dev), 0);
        emu_resource *res = nullptr;
        ASSERT_EQ(emu_device_register_resource(dev, 0, probe_teardown,
                                               probe_free, &probe, &res),
                  0);
        // Leave it registered and open; t goes out of scope here.
    }
    EXPECT_EQ(probe.free_calls, 1) << "tree destruction must free the backing";
}

TEST(Registry, RegisterOnRevokedDeviceFails)
{
    Tree t;
    emu_device *dev = nullptr;
    ASSERT_EQ(emu_node_tree_add_device(t.get(), "0000:61:00", &dev), 0);
    ASSERT_EQ(emu_device_revoke(t.get(), "0000:61:00"), 0);

    emu_resource *r = nullptr;
    EXPECT_EQ(emu_device_register_resource(dev, 0, nullptr, nullptr, nullptr,
                                           &r),
              -1);
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
    emu_device *dev = nullptr;
    ASSERT_EQ(emu_node_tree_add_device(t.get(), "0000:61:00", &dev), 0);

    emu_resource *res = nullptr;
    ASSERT_EQ(emu_device_register_resource(dev, 0, probe_teardown, probe_free,
                                           &probe, &res),
              0);

    // Attach to a qpair node and look it up (kernel ref), then unlink-while-open
    // is modelled by leaving the node looked-up while we revoke.
    emu_node *qp = nullptr;
    ASSERT_EQ(emu_node_create_child(t.get(), dev->qdma, "qpair0", EMU_NODE_FILE,
                                    0644, nullptr, nullptr, &qp),
              0);
    ASSERT_EQ(emu_node_attach_resource(t.get(), qp, res), 0);

    // Kernel holds a lookup on the qpair node.
    emu_node *looked = nullptr;
    ASSERT_EQ(emu_node_lookup_child(t.get(), dev->qdma->ino, "qpair0", &looked),
              0);

    // Forced removal: drops the registry ref + tears down, but the inode ref
    // (held by the still-looked-up node) keeps the object alive.
    ASSERT_EQ(emu_device_revoke(t.get(), "0000:61:00"), 0);
    EXPECT_EQ(probe.teardown_calls, 1);
    EXPECT_EQ(probe.free_calls, 0) << "freed too early: inode ref still held";

    // The holder finally lets go (forget drops the inode ref) -> freed once.
    emu_node_forget(t.get(), qp->ino, 1);
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
    emu_device *dev = nullptr;
    ASSERT_EQ(emu_node_tree_add_device(t.get(), "0000:61:00", &dev), 0);

    emu_resource *res = nullptr;
    ASSERT_EQ(emu_device_register_resource(dev, 0, probe_teardown, probe_free,
                                           &probe, &res),
              0);

    emu_node *qp = nullptr;
    ASSERT_EQ(emu_node_create_child(t.get(), dev->qdma, "qpair0", EMU_NODE_FILE,
                                    0644, nullptr, nullptr, &qp),
              0);
    ASSERT_EQ(emu_node_attach_resource(t.get(), qp, res), 0);

    // Cooperative teardown path: node has no outstanding lookups, so creating it
    // then forgetting/destroying it must run teardown AND unregister.
    // First give it a lookup and unlink via revoke would conflate paths; instead
    // simulate "last close of an undisturbed qpair": the node is reaped because
    // lookup_count is 0 once we mark it unlinked.  We use forget on a node with
    // count 0 after an explicit lookup+forget pair.
    emu_node *looked = nullptr;
    ASSERT_EQ(emu_node_lookup_child(t.get(), dev->qdma->ino, "qpair0", &looked),
              0);
    // Drop the kernel lookup; node is still linked, so it is NOT destroyed yet.
    emu_node_forget(t.get(), qp->ino, 1);
    EXPECT_EQ(probe.teardown_calls, 0);
    EXPECT_EQ(probe.free_calls, 0);

    // Now forced removal drops both the inode ref (node destroyed) and the
    // registry ref -> teardown once, freed once.
    ASSERT_EQ(emu_device_revoke(t.get(), "0000:61:00"), 0);
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
    emu_device *dev = nullptr;
    ASSERT_EQ(emu_node_tree_add_device(t.get(), "0000:61:00", &dev), 0);

    emu_resource *res = nullptr;
    ASSERT_EQ(emu_device_register_resource(dev, 0, probe_teardown, probe_free,
                                           &probe, &res),
              0);
    emu_node *qp = nullptr;
    ASSERT_EQ(emu_node_create_child(t.get(), dev->qdma, "qpair0", EMU_NODE_FILE,
                                    0644, nullptr, nullptr, &qp),
              0);
    ASSERT_EQ(emu_node_attach_resource(t.get(), qp, res), 0);

    // Open it (kernel lookup), then unlink-while-open: it becomes nameless but
    // is still found via the registry, and is NOT yet torn down.
    emu_node *looked = nullptr;
    ASSERT_EQ(emu_node_lookup_child(t.get(), dev->qdma->ino, "qpair0", &looked),
              0);
    emu_ino_t qp_ino = qp->ino;
    emu_node_unlink(t.get(), qp);
    EXPECT_TRUE(readdir_names(t.get(), dev->qdma->ino).empty());
    EXPECT_EQ(emu_device_find_resource(dev, 0), res) << "still in registry";
    EXPECT_EQ(probe.teardown_calls, 0);
    EXPECT_EQ(probe.free_calls, 0);

    // Last close -> cooperative teardown: once, then freed.
    emu_node_forget(t.get(), qp_ino, 1);
    EXPECT_EQ(probe.teardown_calls, 1);
    EXPECT_EQ(probe.free_calls, 1);
}

TEST(Teardown, UnlinkedQpairStaysLiveUntilClosed)
{
    // A normally-unlinked qpair (not revoked) keeps working until its fd closes:
    // ops still succeed (it is live), unlike the forced-revoke case.
    Tree t;
    emu_device *dev = nullptr;
    ASSERT_EQ(emu_node_tree_add_device(t.get(), "0000:61:00", &dev), 0);

    emu_resource *res = nullptr;
    ASSERT_EQ(emu_device_register_resource(dev, 0, nullptr, nullptr, nullptr,
                                           &res),
              0);
    emu_node *qp = nullptr;
    ASSERT_EQ(emu_node_create_child(t.get(), dev->qdma, "qpair0", EMU_NODE_FILE,
                                    0644, nullptr, nullptr, &qp),
              0);
    ASSERT_EQ(emu_node_attach_resource(t.get(), qp, res), 0);
    emu_node *looked = nullptr;
    ASSERT_EQ(emu_node_lookup_child(t.get(), dev->qdma->ino, "qpair0", &looked),
              0);

    emu_node_unlink(t.get(), qp);
    // Still live: a normal unlink is delete-on-last-close, not revocation.
    EXPECT_EQ(emu_resource_check(t.get(), res), 0);

    emu_node_forget(t.get(), qp->ino, 1);  // close -> freed
}

TEST(Teardown, BothTriggersAreIdempotent)
{
    // Fire BOTH teardown triggers on one resource -- forced removal (registry
    // ref) while the qpair fd is still open, then cooperative eviction (inode
    // ref) on forget.  Teardown must run exactly once and the object be freed
    // exactly once.
    ResourceProbe probe;
    Tree t;
    emu_device *dev = nullptr;
    ASSERT_EQ(emu_node_tree_add_device(t.get(), "0000:61:00", &dev), 0);

    emu_resource *res = nullptr;
    ASSERT_EQ(emu_device_register_resource(dev, 0, probe_teardown, probe_free,
                                           &probe, &res),
              0);
    emu_node *qp = nullptr;
    ASSERT_EQ(emu_node_create_child(t.get(), dev->qdma, "qpair0", EMU_NODE_FILE,
                                    0644, nullptr, nullptr, &qp),
              0);
    ASSERT_EQ(emu_node_attach_resource(t.get(), qp, res), 0);

    emu_node *looked = nullptr;
    ASSERT_EQ(emu_node_lookup_child(t.get(), dev->qdma->ino, "qpair0", &looked),
              0);
    ASSERT_EQ(emu_device_revoke(t.get(), "0000:61:00"), 0);  // trigger 1 (forced)
    emu_node_forget(t.get(), qp->ino, 1);                    // trigger 2 (coop)
    EXPECT_EQ(probe.teardown_calls, 1) << "teardown must be idempotent";
    EXPECT_EQ(probe.free_calls, 1);
}

TEST(Teardown, RevokeIsIdempotent)
{
    Tree t;
    ASSERT_EQ(emu_node_tree_add_device(t.get(), "0000:61:00", nullptr), 0);
    EXPECT_EQ(emu_device_revoke(t.get(), "0000:61:00"), 0);
    EXPECT_EQ(emu_device_revoke(t.get(), "0000:61:00"), 0);  // no-op
    EXPECT_EQ(emu_device_revoke(t.get(), "0000:99:00"), 0);  // never existed
}

// ===========================================================================
// Revocation state machine
// ===========================================================================

TEST(Revocation, LookupOfRemovedEndpointReturnsEnoent)
{
    Tree t;
    ASSERT_EQ(emu_node_tree_add_device(t.get(), "0000:61:00", nullptr), 0);

    // Present before revoke.
    emu_node *child = nullptr;
    ASSERT_EQ(emu_node_lookup_child(t.get(), EMU_ROOT_INO, "0000:61:00", &child),
              0);
    emu_node_forget(t.get(), child->ino, 1);

    ASSERT_EQ(emu_device_revoke(t.get(), "0000:61:00"), 0);

    // New lookup misses.
    EXPECT_EQ(emu_node_lookup_child(t.get(), EMU_ROOT_INO, "0000:61:00", &child),
              -ENOENT);
    // ... and it is gone from readdir.
    EXPECT_TRUE(readdir_names(t.get(), EMU_ROOT_INO).empty());
}

TEST(Revocation, OpOnOpenHandleOfRemovedEndpointReturnsEnodev)
{
    Tree t;
    emu_device *dev = nullptr;
    ASSERT_EQ(emu_node_tree_add_device(t.get(), "0000:61:00", &dev), 0);

    // A node-backed endpoint (e.g. bars/bar0) with an outstanding open handle.
    emu_node *bar = nullptr;
    ASSERT_EQ(emu_node_create_child(t.get(), dev->bars, "bar0", EMU_NODE_FILE,
                                    0644, nullptr, nullptr, &bar),
              0);
    emu_node *opened = nullptr;
    ASSERT_EQ(emu_node_lookup_child(t.get(), dev->bars->ino, "bar0", &opened),
              0);
    // Live before revoke.
    EXPECT_EQ(emu_node_is_live(t.get(), bar->ino), 0);

    ASSERT_EQ(emu_device_revoke(t.get(), "0000:61:00"), 0);

    // The fd is still open (lookup not yet forgotten): ops must see ENODEV.
    EXPECT_EQ(emu_node_is_live(t.get(), bar->ino), -ENODEV);
}

TEST(Revocation, ResourceOpReturnsEnodevAfterRevoke)
{
    Tree t;
    emu_device *dev = nullptr;
    ASSERT_EQ(emu_node_tree_add_device(t.get(), "0000:61:00", &dev), 0);

    emu_resource *res = nullptr;
    ASSERT_EQ(emu_device_register_resource(dev, 0, nullptr, nullptr, nullptr,
                                           &res),
              0);
    emu_node *qp = nullptr;
    ASSERT_EQ(emu_node_create_child(t.get(), dev->qdma, "qpair0", EMU_NODE_FILE,
                                    0644, nullptr, nullptr, &qp),
              0);
    ASSERT_EQ(emu_node_attach_resource(t.get(), qp, res), 0);
    emu_node *looked = nullptr;
    ASSERT_EQ(emu_node_lookup_child(t.get(), dev->qdma->ino, "qpair0", &looked),
              0);

    EXPECT_EQ(emu_resource_check(t.get(), res), 0);
    ASSERT_EQ(emu_device_revoke(t.get(), "0000:61:00"), 0);
    // res is still alive (inode ref) but dead -> ENODEV.
    EXPECT_EQ(emu_resource_check(t.get(), res), -ENODEV);

    emu_node_forget(t.get(), qp->ino, 1);  // release the orphan
}

TEST(Revocation, NameInvalidationIsInvoked)
{
    Tree t(/*with_notifier=*/true);
    emu_device *dev = nullptr;
    ASSERT_EQ(emu_node_tree_add_device(t.get(), "0000:61:00", &dev), 0);
    emu_ino_t bdf_ino = dev->dir->ino;

    ASSERT_EQ(emu_device_revoke(t.get(), "0000:61:00"), 0);

    // The <BDF> dir (and its bars/qdma children) must have been invalidated.
    bool saw_bdf = false;
    for (const auto &rec : g_notifications) {
        if (rec.name == "0000:61:00" && rec.parent == EMU_ROOT_INO &&
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
    emu_device *dev = nullptr;
    ASSERT_EQ(emu_node_tree_add_device(t.get(), "0000:61:00", &dev), 0);
    emu_ino_t bars_ino = dev->bars->ino;
    ASSERT_EQ(emu_device_revoke(t.get(), "0000:61:00"), 0);

    emu_node *child = nullptr;
    int ret = emu_node_lookup_child(t.get(), bars_ino, "bar0", &child);
    EXPECT_TRUE(ret == -ESTALE || ret == -ENOENT)
        << "unexpected ret=" << ret;
}
