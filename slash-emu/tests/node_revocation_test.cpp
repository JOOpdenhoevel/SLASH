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
 * @file node_revocation_test.cpp
 * @brief Adversarial conformance tests for the slash-emu spine (T5 / task #3).
 *
 * This suite exists to break the refcount / teardown / revocation machinery in
 * node.c -- the semantics the ABI conformance suite (T11) will enforce.  It is
 * deliberately exhaustive about drop orders, idempotency of the two teardown
 * triggers, the dead-orphan state machine, nameless-qpair lifetime, and OOM /
 * error-path cleanup (the latter leaning on ASan).  Each test corresponds to a
 * numbered hammer item in the T5 adversarial brief.
 *
 * node.c is intentionally FUSE-free, so everything here is a pure unit test
 * driven directly against slash_emu_core; the kernel-dentry notifier is a
 * recording stub.
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
// Recording notifier: captures every notify_delete the spine fires, and (for
// the re-entry test) can call back INTO the tree to prove the lock is dropped.
// ---------------------------------------------------------------------------
struct NotifyRecord {
    emu_ino_t parent;
    emu_ino_t child;
    std::string name;
};

struct Notifier {
    std::vector<NotifyRecord> records;
    // Optional re-entry probe: if set, the notifier calls this with the tree so
    // it can issue a locking operation from inside notify_delete.  If the lock
    // were held across the notifier, this would deadlock.
    emu_node_tree *reentry_tree = nullptr;
    int reentry_calls = 0;
    int reentry_ok = 0;
};

Notifier *g_notifier = nullptr;

void recording_notify_delete(void *ctx, emu_ino_t parent, emu_ino_t child,
                             const char *name)
{
    auto *n = static_cast<Notifier *>(ctx);
    n->records.push_back({parent, child, std::string(name)});

    if (n->reentry_tree != nullptr) {
        n->reentry_calls++;
        // A lookup takes tree->lock.  Reaching here at all (no deadlock) proves
        // the spine dropped the lock before firing the notifier.
        emu_node *out = nullptr;
        (void) emu_node_lookup_child(n->reentry_tree, EMU_ROOT_INO, "nope",
                                     &out);
        n->reentry_ok++;
    }
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

// RAII wrapper around a tree.  Optionally installs a recording notifier whose
// records live in `notifier`.
class Tree {
public:
    Tree()
    {
        emu_notifier n{};
        EXPECT_EQ(emu_node_tree_new(&tree_, &n), 0);
    }
    explicit Tree(Notifier *notifier)
    {
        g_notifier = notifier;
        emu_notifier n{};
        n.notify_delete = recording_notify_delete;
        n.ctx = notifier;
        EXPECT_EQ(emu_node_tree_new(&tree_, &n), 0);
    }
    ~Tree() { cleanup_node_tree(tree_); }

    emu_node_tree *get() { return tree_; }

private:
    emu_node_tree *tree_ = nullptr;
};

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

// Helper: stand up a device with one registered + attached + looked-up qpair.
// Returns the resource and writes the qpair node out.  The qpair has
// lookup_count == 1 (a held kernel reference / open fd).
emu_resource *make_open_qpair(emu_node_tree *tree, emu_device *dev,
                              ResourceProbe *probe, const char *name,
                              uint32_t id, emu_node **qp_out)
{
    emu_resource *res = nullptr;
    // When no probe is supplied the callbacks are NULL too -- the spine must
    // tolerate teardown/free with no callbacks (and the test isn't counting).
    EXPECT_EQ(emu_device_register_resource(
                  dev, id, probe != nullptr ? probe_teardown : nullptr,
                  probe != nullptr ? probe_free : nullptr, probe, &res),
              0);
    emu_node *qp = nullptr;
    EXPECT_EQ(emu_node_create_child(tree, dev->qdma, name, EMU_NODE_FILE, 0644,
                                    nullptr, nullptr, &qp),
              0);
    EXPECT_EQ(emu_node_attach_resource(tree, qp, res), 0);
    emu_node *looked = nullptr;
    EXPECT_EQ(emu_node_lookup_child(tree, dev->qdma->ino, name, &looked), 0);
    if (qp_out != nullptr) {
        *qp_out = qp;
    }
    return res;
}

}  // namespace

// ===========================================================================
// 1. REFCOUNT: freed only when BOTH refs drop; free_backing runs exactly once.
//    Exhaustive over single-ref-only paths and drop orders.
// ===========================================================================

// A resource that holds ONLY the registry reference (never attached to an inode)
// is freed exactly once when the registry ref drops via revoke -- no inode ref to
// keep it alive, no double free.
TEST(Refcount, RegistryOnly_FreedOnRevoke)
{
    ResourceProbe probe;
    Tree t;
    emu_device *dev = nullptr;
    ASSERT_EQ(emu_node_tree_add_device(t.get(), "0000:61:00", &dev), 0);

    emu_resource *res = nullptr;
    ASSERT_EQ(emu_device_register_resource(dev, 5, probe_teardown, probe_free,
                                           &probe, &res),
              0);
    // Never attached to any inode: registry ref only.
    ASSERT_EQ(emu_device_revoke(t.get(), "0000:61:00"), 0);
    EXPECT_EQ(probe.teardown_calls, 1);
    EXPECT_EQ(probe.free_calls, 1) << "registry-only resource must free at revoke";
}

// A resource that holds ONLY the registry reference, dropped via the cooperative
// path (the node it would attach to is destroyed) -- but here it is never
// attached, so the ONLY way to drop the registry ref is revoke or tree
// destruction.  Confirm tree destruction frees a registry-only resource exactly
// once (no inode ref, no leak, no double free).
TEST(Refcount, RegistryOnly_FreedAtTreeDestruction)
{
    ResourceProbe probe;
    {
        Tree t;
        emu_device *dev = nullptr;
        ASSERT_EQ(emu_node_tree_add_device(t.get(), "0000:61:00", &dev), 0);
        emu_resource *res = nullptr;
        ASSERT_EQ(emu_device_register_resource(dev, 5, probe_teardown,
                                               probe_free, &probe, &res),
                  0);
    }
    EXPECT_EQ(probe.free_calls, 1);
}

// Drop order A: registry-then-inode (revoke first, forget second).  Free fires
// on the SECOND drop (inode), exactly once; teardown exactly once on the first.
TEST(Refcount, DropOrder_RegistryThenInode)
{
    ResourceProbe probe;
    Tree t;
    emu_device *dev = nullptr;
    ASSERT_EQ(emu_node_tree_add_device(t.get(), "0000:61:00", &dev), 0);
    emu_node *qp = nullptr;
    emu_resource *res = make_open_qpair(t.get(), dev, &probe, "qpair0", 0, &qp);

    ASSERT_EQ(emu_device_revoke(t.get(), "0000:61:00"), 0);  // registry ref drops
    EXPECT_EQ(probe.teardown_calls, 1);
    EXPECT_EQ(probe.free_calls, 0) << "inode ref still held";

    emu_node_forget(t.get(), qp->ino, 1);                  // inode ref drops
    EXPECT_EQ(probe.free_calls, 1);
    EXPECT_EQ(probe.teardown_calls, 1);
    (void) res;
}

// Drop order B: the cooperative inode-drop path ALSO unregisters.  Closing an
// undisturbed (unlinked) qpair runs node_destroy_locked, which tears down,
// unregisters (drops the registry ref), and then drops the inode ref -- so both
// refs go in one atomic step and the object frees immediately.  There is, by
// design, no reachable "inode dropped but registry ref still held" intermediate
// state for a cooperatively-evicted node: the two drop orders collapse.  Codify
// this so a future refactor that splits the cooperative path (re-introducing a
// registry-only-after-inode window) is caught.
TEST(Refcount, CooperativeDropAlsoUnregisters)
{
    ResourceProbe probe;
    Tree t;
    emu_device *dev = nullptr;
    ASSERT_EQ(emu_node_tree_add_device(t.get(), "0000:61:00", &dev), 0);
    emu_node *qp = nullptr;
    (void) make_open_qpair(t.get(), dev, &probe, "qpair0", 0, &qp);

    // Sanity: it is registry-findable before close.
    EXPECT_NE(emu_device_find_resource(dev, 0), nullptr);

    // Unlink-while-open then close: cooperative teardown.
    emu_ino_t ino = qp->ino;
    emu_node_unlink(t.get(), qp);
    emu_node_forget(t.get(), ino, 1);

    // Both refs dropped in one shot: freed immediately, teardown once, and the
    // resource is gone from the registry (no dangling registry entry for revoke
    // to trip over).
    EXPECT_EQ(probe.teardown_calls, 1);
    EXPECT_EQ(probe.free_calls, 1) << "cooperative close drops BOTH refs";
    EXPECT_EQ(emu_device_find_resource(dev, 0), nullptr)
        << "cooperative teardown must unregister";

    // A subsequent revoke is a clean no-op -- no double free / use-after-free.
    ASSERT_EQ(emu_device_revoke(t.get(), "0000:61:00"), 0);
    EXPECT_EQ(probe.teardown_calls, 1);
    EXPECT_EQ(probe.free_calls, 1);
}

// LEAK HUNT: a resource left inode-only-referenced (registry ref already
// dropped) at tree destruction.  Reachable when a device is revoked while a
// qpair fd is still open: revoke drops the registry ref but the orphan node
// keeps the inode ref; if the daemon then shuts down WITHOUT a forget, the
// resource backing must still be freed exactly once.  emu_node_free_shell does
// not drop node->resource, so this is the canonical leak path for the tree
// teardown.
TEST(Refcount, InodeOnlyResourceFreedAtTreeDestruction)
{
    ResourceProbe probe;
    {
        Tree t;
        emu_device *dev = nullptr;
        ASSERT_EQ(emu_node_tree_add_device(t.get(), "0000:61:00", &dev), 0);
        emu_node *qp = nullptr;
        (void) make_open_qpair(t.get(), dev, &probe, "qpair0", 0, &qp);

        // Forced removal drops the registry ref; the still-open node keeps the
        // inode ref, so the resource survives as a dead orphan.
        ASSERT_EQ(emu_device_revoke(t.get(), "0000:61:00"), 0);
        EXPECT_EQ(probe.teardown_calls, 1);
        EXPECT_EQ(probe.free_calls, 0) << "inode ref keeps it alive";

        // No forget: the daemon shuts down with the orphan still inode-held.
        // Tree destruction must reclaim the backing.
    }
    EXPECT_EQ(probe.free_calls, 1)
        << "inode-only orphan leaked at tree destruction (free_backing skipped)";
}

// ===========================================================================
// 2. IDEMPOTENT TEARDOWN: first of {cooperative, forced} does the work; the
//    second is a true no-op.  Both orders + double-forced.
// ===========================================================================

// forced-then-cooperative (covered conceptually in node_test, re-asserted with
// the side-effect counters split out).
TEST(IdempotentTeardown, ForcedThenCooperative)
{
    ResourceProbe probe;
    Tree t;
    emu_device *dev = nullptr;
    ASSERT_EQ(emu_node_tree_add_device(t.get(), "0000:61:00", &dev), 0);
    emu_node *qp = nullptr;
    (void) make_open_qpair(t.get(), dev, &probe, "qpair0", 0, &qp);

    ASSERT_EQ(emu_device_revoke(t.get(), "0000:61:00"), 0);  // forced
    emu_node_forget(t.get(), qp->ino, 1);                  // cooperative
    EXPECT_EQ(probe.teardown_calls, 1);
    EXPECT_EQ(probe.free_calls, 1);
}

// cooperative-then-forced: close the open (unlinked) qpair first (cooperative
// teardown + free of the inode ref), THEN revoke the device.  Revoke must not
// touch the freed resource (it was unregistered on cooperative teardown) and
// teardown stays at one.
TEST(IdempotentTeardown, CooperativeThenForced)
{
    ResourceProbe probe;
    Tree t;
    emu_device *dev = nullptr;
    ASSERT_EQ(emu_node_tree_add_device(t.get(), "0000:61:00", &dev), 0);
    emu_node *qp = nullptr;
    (void) make_open_qpair(t.get(), dev, &probe, "qpair0", 0, &qp);

    // Cooperative: unlink + close -> teardown + unregister + free (both refs
    // gone because attach added the inode ref and register the registry ref, but
    // cooperative teardown unregisters too).
    emu_ino_t ino = qp->ino;
    emu_node_unlink(t.get(), qp);
    emu_node_forget(t.get(), ino, 1);
    EXPECT_EQ(probe.teardown_calls, 1);
    EXPECT_EQ(probe.free_calls, 1) << "cooperative close frees an undisturbed qpair";

    // Forced removal afterwards must be a clean no-op (resource already gone from
    // the registry; no double teardown, no use-after-free).
    ASSERT_EQ(emu_device_revoke(t.get(), "0000:61:00"), 0);
    EXPECT_EQ(probe.teardown_calls, 1);
    EXPECT_EQ(probe.free_calls, 1);
}

// Double-forced: revoke the same BDF twice while an orphan still holds the inode
// ref.  The second revoke is a no-op (device already gone from the live set);
// teardown stays at one, no double free.
TEST(IdempotentTeardown, DoubleForced)
{
    ResourceProbe probe;
    Tree t;
    emu_device *dev = nullptr;
    ASSERT_EQ(emu_node_tree_add_device(t.get(), "0000:61:00", &dev), 0);
    emu_node *qp = nullptr;
    (void) make_open_qpair(t.get(), dev, &probe, "qpair0", 0, &qp);

    ASSERT_EQ(emu_device_revoke(t.get(), "0000:61:00"), 0);
    ASSERT_EQ(emu_device_revoke(t.get(), "0000:61:00"), 0);  // no-op
    EXPECT_EQ(probe.teardown_calls, 1);
    EXPECT_EQ(probe.free_calls, 0) << "orphan inode ref still held";

    emu_node_forget(t.get(), qp->ino, 1);
    EXPECT_EQ(probe.free_calls, 1);
    EXPECT_EQ(probe.teardown_calls, 1);
}

// ===========================================================================
// 3. REVOCATION STATE MACHINE: ENOENT on lookup, ENODEV on open handles,
//    idempotent, notify_delete fires for each revoked name.
// ===========================================================================

// Revoke is a success no-op on an absent BDF and on an already-revoked BDF, and
// does not fire any notification for the absent case.
TEST(RevocationStateMachine, IdempotentAndAbsentNoNotify)
{
    Notifier notifier;
    Tree t(&notifier);
    ASSERT_EQ(emu_node_tree_add_device(t.get(), "0000:61:00", nullptr), 0);

    ASSERT_EQ(emu_device_revoke(t.get(), "0000:61:00"), 0);
    size_t after_first = notifier.records.size();
    EXPECT_GT(after_first, 0u);

    // Re-revoking the same BDF: no device found -> no new notifications.
    ASSERT_EQ(emu_device_revoke(t.get(), "0000:61:00"), 0);
    EXPECT_EQ(notifier.records.size(), after_first);

    // A BDF that never existed: success, no notifications.
    ASSERT_EQ(emu_device_revoke(t.get(), "0000:99:00"), 0);
    EXPECT_EQ(notifier.records.size(), after_first);
}

// notify_delete must fire for EVERY revoked name: the <BDF> dir, bars/, qdma/,
// and any endpoint files under them.
TEST(RevocationStateMachine, NotifyDeleteFiresForEveryName)
{
    Notifier notifier;
    Tree t(&notifier);
    emu_device *dev = nullptr;
    ASSERT_EQ(emu_node_tree_add_device(t.get(), "0000:61:00", &dev), 0);

    // Attach endpoint files so revocation must invalidate them too.
    emu_node *info = nullptr;
    ASSERT_EQ(emu_node_create_child(t.get(), dev->dir, "info", EMU_NODE_FILE,
                                    0444, nullptr, nullptr, &info),
              0);
    emu_node *bar0 = nullptr;
    ASSERT_EQ(emu_node_create_child(t.get(), dev->bars, "bar0", EMU_NODE_FILE,
                                    0644, nullptr, nullptr, &bar0),
              0);

    ASSERT_EQ(emu_device_revoke(t.get(), "0000:61:00"), 0);

    std::set<std::string> names;
    for (const auto &r : notifier.records) {
        names.insert(r.name);
    }
    EXPECT_EQ(names.count("0000:61:00"), 1u);
    EXPECT_EQ(names.count("bars"), 1u);
    EXPECT_EQ(names.count("qdma"), 1u);
    EXPECT_EQ(names.count("info"), 1u);
    EXPECT_EQ(names.count("bar0"), 1u);
}

// emu_resource_check / emu_node_is_live report -ENODEV after revoke on a handle
// still held; -ENOENT semantics (lookup miss) for the name.
TEST(RevocationStateMachine, EnodevOnHandleEnoentOnName)
{
    Tree t;
    emu_device *dev = nullptr;
    ASSERT_EQ(emu_node_tree_add_device(t.get(), "0000:61:00", &dev), 0);
    emu_node *qp = nullptr;
    emu_resource *res = make_open_qpair(t.get(), dev, nullptr, "qpair0", 0, &qp);

    EXPECT_EQ(emu_resource_check(t.get(), res), 0);
    EXPECT_EQ(emu_node_is_live(t.get(), qp->ino), 0);

    ASSERT_EQ(emu_device_revoke(t.get(), "0000:61:00"), 0);

    // Op on the open handle: -ENODEV.
    EXPECT_EQ(emu_resource_check(t.get(), res), -ENODEV);
    EXPECT_EQ(emu_node_is_live(t.get(), qp->ino), -ENODEV);

    // New lookup of any removed name: not found.
    emu_node *out = nullptr;
    EXPECT_EQ(emu_node_lookup_child(t.get(), EMU_ROOT_INO, "0000:61:00", &out),
              -ENOENT);

    emu_node_forget(t.get(), qp->ino, 1);
}

// ===========================================================================
// 4. NAMELESS QPAIR LIFETIME: unlink != revoke.  Stays live, gone from readdir
//    / by-name lookup, still registry-findable by qid, torn down on final close.
// ===========================================================================

TEST(NamelessQpair, UnlinkKeepsLiveAndRegistryFindable)
{
    ResourceProbe probe;
    Tree t;
    emu_device *dev = nullptr;
    ASSERT_EQ(emu_node_tree_add_device(t.get(), "0000:61:00", &dev), 0);
    emu_node *qp = nullptr;
    emu_resource *res = make_open_qpair(t.get(), dev, &probe, "qpair7", 7, &qp);
    emu_ino_t ino = qp->ino;

    emu_node_unlink(t.get(), qp);

    // Still live -- unlink is delete-on-last-close, not revocation.
    EXPECT_EQ(emu_resource_check(t.get(), res), 0);
    EXPECT_EQ(emu_node_is_live(t.get(), ino), 0);
    EXPECT_EQ(probe.teardown_calls, 0);

    // Gone from the directory listing and by-name lookup.
    EXPECT_TRUE(readdir_names(t.get(), dev->qdma->ino).empty());
    emu_node *byname = nullptr;
    EXPECT_EQ(emu_node_lookup_child(t.get(), dev->qdma->ino, "qpair7", &byname),
              -ENOENT);

    // Still reachable through the registry by qid.
    EXPECT_EQ(emu_device_find_resource(dev, 7), res);

    // Final close -> cooperative teardown exactly once, then freed.
    emu_node_forget(t.get(), ino, 1);
    EXPECT_EQ(probe.teardown_calls, 1);
    EXPECT_EQ(probe.free_calls, 1);
}

// Unlink while open then forced revoke of the device: the nameless qpair is
// reached through the registry (the tree can't see it), torn down once, and
// freed once on final close.
TEST(NamelessQpair, RevokeReachesNamelessViaRegistry)
{
    ResourceProbe probe;
    Tree t;
    emu_device *dev = nullptr;
    ASSERT_EQ(emu_node_tree_add_device(t.get(), "0000:61:00", &dev), 0);
    emu_node *qp = nullptr;
    emu_resource *res = make_open_qpair(t.get(), dev, &probe, "qpair7", 7, &qp);
    emu_ino_t ino = qp->ino;

    emu_node_unlink(t.get(), qp);  // nameless, still registered + live
    ASSERT_EQ(emu_device_find_resource(dev, 7), res);

    ASSERT_EQ(emu_device_revoke(t.get(), "0000:61:00"), 0);
    EXPECT_EQ(probe.teardown_calls, 1) << "registry reached the nameless qpair";
    EXPECT_EQ(emu_resource_check(t.get(), res), -ENODEV);
    EXPECT_EQ(probe.free_calls, 0) << "orphan inode ref still held";

    emu_node_forget(t.get(), ino, 1);
    EXPECT_EQ(probe.free_calls, 1);
    EXPECT_EQ(probe.teardown_calls, 1);
}

// ===========================================================================
// 5. forget / lookup_count: a looked-up node revoked must survive as a DEAD
//    orphan (ops -ENODEV) until forgotten, then be reaped.  Regression for the
//    self-caught use-after-free: parent child-list integrity after reap.
// ===========================================================================

TEST(DeadOrphan, SurvivesUntilForgottenThenReaped)
{
    Tree t;
    emu_device *dev = nullptr;
    ASSERT_EQ(emu_node_tree_add_device(t.get(), "0000:61:00", &dev), 0);

    emu_node *bar = nullptr;
    ASSERT_EQ(emu_node_create_child(t.get(), dev->bars, "bar0", EMU_NODE_FILE,
                                    0644, nullptr, nullptr, &bar),
              0);
    emu_node *opened = nullptr;
    ASSERT_EQ(emu_node_lookup_child(t.get(), dev->bars->ino, "bar0", &opened), 0);
    emu_ino_t ino = bar->ino;

    ASSERT_EQ(emu_device_revoke(t.get(), "0000:61:00"), 0);

    // Dead orphan: still resolvable by inode (it exists), but dead.
    EXPECT_EQ(emu_node_is_live(t.get(), ino), -ENODEV);
    EXPECT_NE(emu_node_lookup_ino(t.get(), ino), nullptr);

    // After forget, reaped: the inode no longer exists.
    emu_node_forget(t.get(), ino, 1);
    EXPECT_EQ(emu_node_lookup_ino(t.get(), ino), nullptr);
    EXPECT_EQ(emu_node_is_live(t.get(), ino), -ENOENT);
}

// REGRESSION for the self-caught use-after-free: a revoked-then-reaped <BDF>
// node must not leave a dangling pointer in root's children.  Here the <BDF>
// node is looked up (held), revoked (survives as orphan, detached from root),
// then forgotten (reaped).  Afterward root's child list must be intact: a fresh
// add of a DIFFERENT device and a root readdir must not touch freed memory.
TEST(DeadOrphan, ReapDoesNotDangleInParentChildList)
{
    Tree t;
    emu_device *dev = nullptr;
    ASSERT_EQ(emu_node_tree_add_device(t.get(), "0000:61:00", &dev), 0);

    // Hold the <BDF> dir itself (kernel lookup on the device directory).
    emu_node *held = nullptr;
    ASSERT_EQ(emu_node_lookup_child(t.get(), EMU_ROOT_INO, "0000:61:00", &held),
              0);
    emu_ino_t bdf_ino = held->ino;

    ASSERT_EQ(emu_device_revoke(t.get(), "0000:61:00"), 0);
    // The orphan is detached from root immediately (parent cleared) so root's
    // child list holds no reference to it even while it lives.
    EXPECT_TRUE(readdir_names(t.get(), EMU_ROOT_INO).empty());

    // Reap the orphan.
    emu_node_forget(t.get(), bdf_ino, 1);

    // Root's child list must be intact: adding another device and listing root
    // must work and show only the new one (ASan would flag any dangling ref).
    ASSERT_EQ(emu_node_tree_add_device(t.get(), "0000:62:00", nullptr), 0);
    auto kids = readdir_names(t.get(), EMU_ROOT_INO);
    EXPECT_EQ(kids.size(), 1u);
    EXPECT_EQ(kids.count("0000:62:00"), 1u);
}

// A revoked subtree with the <BDF> dir held but a child (qdma) NOT separately
// held: revoke must reap the child immediately (lookup_count 0) yet keep the
// held parent as an orphan, with no dangling child pointer left in the orphan's
// child list (revoke zeroes children.len).  A readdir on the dead orphan dir
// must return no children and not touch freed child nodes.
TEST(DeadOrphan, OrphanDirHasNoDanglingChildrenAfterRevoke)
{
    Tree t;
    emu_device *dev = nullptr;
    ASSERT_EQ(emu_node_tree_add_device(t.get(), "0000:61:00", &dev), 0);

    emu_node *held = nullptr;
    ASSERT_EQ(emu_node_lookup_child(t.get(), EMU_ROOT_INO, "0000:61:00", &held),
              0);
    emu_ino_t bdf_ino = held->ino;

    ASSERT_EQ(emu_device_revoke(t.get(), "0000:61:00"), 0);

    // The held <BDF> dir is a dead orphan.  Its children (bars/qdma) were reaped
    // (no separate lookups), and its child list was cleared, so a readdir over
    // the orphan must enumerate nothing and never dereference a freed child.
    NameCollector c;
    int ret = emu_node_readdir(t.get(), bdf_ino, collect_names, &c);
    EXPECT_EQ(ret, 0);
    EXPECT_TRUE(c.names.empty());

    emu_node_forget(t.get(), bdf_ino, 1);
}

// A partial-forget (nlookup < lookup_count) must keep a revoked node alive as an
// orphan; only the final balancing forget reaps it.
TEST(DeadOrphan, PartialForgetKeepsOrphanAlive)
{
    Tree t;
    emu_device *dev = nullptr;
    ASSERT_EQ(emu_node_tree_add_device(t.get(), "0000:61:00", &dev), 0);

    emu_node *bar = nullptr;
    ASSERT_EQ(emu_node_create_child(t.get(), dev->bars, "bar0", EMU_NODE_FILE,
                                    0644, nullptr, nullptr, &bar),
              0);
    // Two outstanding lookups.
    emu_node *o1 = nullptr;
    emu_node *o2 = nullptr;
    ASSERT_EQ(emu_node_lookup_child(t.get(), dev->bars->ino, "bar0", &o1), 0);
    ASSERT_EQ(emu_node_lookup_child(t.get(), dev->bars->ino, "bar0", &o2), 0);
    emu_ino_t ino = bar->ino;

    ASSERT_EQ(emu_device_revoke(t.get(), "0000:61:00"), 0);

    // Drop one of two: still alive as an orphan.
    emu_node_forget(t.get(), ino, 1);
    EXPECT_NE(emu_node_lookup_ino(t.get(), ino), nullptr);
    EXPECT_EQ(emu_node_is_live(t.get(), ino), -ENODEV);

    // Drop the last: reaped.
    emu_node_forget(t.get(), ino, 1);
    EXPECT_EQ(emu_node_lookup_ino(t.get(), ino), nullptr);
}

// ===========================================================================
// 6. LOCKING / RE-ENTRY: the tree mutex must NEVER be held across notify_delete.
//    The notifier re-enters the tree with a locking op; if the lock were held
//    this would self-deadlock (the test would hang and CTest would time out).
// ===========================================================================

TEST(Locking, NotifierMayReenterTreeWithoutDeadlock)
{
    Notifier notifier;
    Tree t(&notifier);
    notifier.reentry_tree = t.get();  // notify_delete will call back in

    emu_device *dev = nullptr;
    ASSERT_EQ(emu_node_tree_add_device(t.get(), "0000:61:00", &dev), 0);

    ASSERT_EQ(emu_device_revoke(t.get(), "0000:61:00"), 0);

    // Every notification re-entered the tree successfully (lock was dropped).
    EXPECT_GT(notifier.reentry_calls, 0);
    EXPECT_EQ(notifier.reentry_calls, notifier.reentry_ok);
}

// ===========================================================================
// 7. TREE / MATERIALIZE: idempotent re-add (RESCAN), multi-accelerator, and
//    re-add of a BDF AFTER it was revoked (it can be materialized fresh).
// ===========================================================================

TEST(Materialize, ReAddAfterRevokeCreatesFreshDevice)
{
    Tree t;
    emu_device *dev = nullptr;
    ASSERT_EQ(emu_node_tree_add_device(t.get(), "0000:61:00", &dev), 0);
    ASSERT_EQ(emu_device_revoke(t.get(), "0000:61:00"), 0);

    // After revoke, find_device must miss (it filters on live).
    EXPECT_EQ(emu_node_tree_find_device(t.get(), "0000:61:00"), nullptr);

    // RESCAN re-adds it: a NEW live device, and the name resolves again.
    emu_device *dev2 = nullptr;
    ASSERT_EQ(emu_node_tree_add_device(t.get(), "0000:61:00", &dev2), 0);
    EXPECT_NE(dev2, nullptr);
    EXPECT_NE(dev2, dev) << "must be a fresh device, not the revoked one";

    emu_node *out = nullptr;
    EXPECT_EQ(emu_node_lookup_child(t.get(), EMU_ROOT_INO, "0000:61:00", &out),
              0);
    emu_node_forget(t.get(), out->ino, 1);

    // Root shows exactly one live BDF dir (the dead one is unlinked).
    auto kids = readdir_names(t.get(), EMU_ROOT_INO);
    EXPECT_EQ(kids.size(), 1u);
}

// Re-add of a still-live device is idempotent and returns the same device with
// no duplicate subtree.
TEST(Materialize, ReAddLiveIsIdempotent)
{
    Tree t;
    emu_device *a = nullptr;
    emu_device *b = nullptr;
    ASSERT_EQ(emu_node_tree_add_device(t.get(), "0000:61:00", &a), 0);

    // Attach a file, then re-add: the existing subtree (and the file) survives.
    emu_node *info = nullptr;
    ASSERT_EQ(emu_node_create_child(t.get(), a->dir, "info", EMU_NODE_FILE, 0444,
                                    nullptr, nullptr, &info),
              0);
    ASSERT_EQ(emu_node_tree_add_device(t.get(), "0000:61:00", &b), 0);
    EXPECT_EQ(a, b);

    auto kids = readdir_names(t.get(), a->dir->ino);
    EXPECT_EQ(kids.count("info"), 1u);
    EXPECT_EQ(kids.count("bars"), 1u);
    EXPECT_EQ(kids.count("qdma"), 1u);
    EXPECT_EQ(kids.size(), 3u);
}

// ===========================================================================
// 8. Multi-resource revoke: many registered resources (some inode-attached,
//    some nameless) all torn down exactly once; inode-held ones freed on forget,
//    registry-only ones freed at revoke.  Mixed bag stress under ASan.
// ===========================================================================

TEST(MultiResource, RevokeTearsDownAllExactlyOnce)
{
    constexpr int kN = 8;
    std::vector<ResourceProbe> probes(kN);
    Tree t;
    emu_device *dev = nullptr;
    ASSERT_EQ(emu_node_tree_add_device(t.get(), "0000:61:00", &dev), 0);

    std::vector<emu_node *> held_nodes;
    for (int i = 0; i < kN; i++) {
        emu_resource *res = nullptr;
        ASSERT_EQ(emu_device_register_resource(dev, (uint32_t) i, probe_teardown,
                                               probe_free, &probes[i], &res),
                  0);
        if (i % 2 == 0) {
            // Even: attach + hold open (inode ref outlives revoke).
            std::string name = "qpair" + std::to_string(i);
            emu_node *qp = nullptr;
            ASSERT_EQ(emu_node_create_child(t.get(), dev->qdma, name.c_str(),
                                            EMU_NODE_FILE, 0644, nullptr, nullptr,
                                            &qp),
                      0);
            ASSERT_EQ(emu_node_attach_resource(t.get(), qp, res), 0);
            emu_node *looked = nullptr;
            ASSERT_EQ(emu_node_lookup_child(t.get(), dev->qdma->ino, name.c_str(),
                                            &looked),
                      0);
            held_nodes.push_back(qp);
        }
        // Odd: registry-only nameless resource.
    }

    ASSERT_EQ(emu_device_revoke(t.get(), "0000:61:00"), 0);

    for (int i = 0; i < kN; i++) {
        EXPECT_EQ(probes[i].teardown_calls, 1) << "resource " << i;
        if (i % 2 == 0) {
            EXPECT_EQ(probes[i].free_calls, 0) << "inode-held " << i;
        } else {
            EXPECT_EQ(probes[i].free_calls, 1) << "registry-only " << i;
        }
    }

    // Forget the held ones -> they free now.
    for (auto *qp : held_nodes) {
        emu_node_forget(t.get(), qp->ino, 1);
    }
    for (int i = 0; i < kN; i += 2) {
        EXPECT_EQ(probes[i].free_calls, 1) << "freed on forget " << i;
        EXPECT_EQ(probes[i].teardown_calls, 1);
    }
}

// DELTA RE-VERIFICATION of the cleanup_node_tree fix: force BOTH teardown loops
// (the registry loop and the inode-only-orphan node walk) to run in a single
// tree destruction, with three resource states coexisting at shutdown, and prove
// each backing is freed EXACTLY once -- no double free (the guard), no miss (the
// leak).  ASan is the witness: a double free aborts, a miss reports a leak.
//
//   (a) STILL REGISTERED + INODE-ATTACHED (device != NULL, also node->resource):
//       reachable from BOTH roots.  The registry loop must free it; the node
//       walk must SKIP it (device != NULL).  This is the double-free trap.
//   (b) STILL REGISTERED, registry-only (device != NULL, no inode):
//       reachable only from the registry.  Registry loop frees it.
//   (c) INODE-ONLY ORPHAN (device == NULL, node->resource only):
//       a prior revoke of a SECOND device dropped its registry ref while an open
//       handle kept the inode ref.  Only the node walk can reclaim it.
TEST(Shutdown, MixedRegisteredAndOrphanResourcesFreedExactlyOnce)
{
    ResourceProbe pa;  // (a) registered + inode-attached, never forgotten
    ResourceProbe pb;  // (b) registered, registry-only
    ResourceProbe pc;  // (c) inode-only orphan from a revoked device
    {
        Tree t;

        // Device 1 stays LIVE to shutdown: holds (a) and (b).
        emu_device *d1 = nullptr;
        ASSERT_EQ(emu_node_tree_add_device(t.get(), "0000:61:00", &d1), 0);

        // (a) registered + attached to a held-open node, NEVER forgotten -> at
        //     shutdown it is BOTH in d1's registry and pointed to by the node.
        emu_node *qp_a = nullptr;
        emu_resource *ra = nullptr;
        ASSERT_EQ(emu_device_register_resource(d1, 1, probe_teardown, probe_free,
                                               &pa, &ra),
                  0);
        ASSERT_EQ(emu_node_create_child(t.get(), d1->qdma, "qpairA",
                                        EMU_NODE_FILE, 0644, nullptr, nullptr,
                                        &qp_a),
                  0);
        ASSERT_EQ(emu_node_attach_resource(t.get(), qp_a, ra), 0);
        emu_node *la = nullptr;
        ASSERT_EQ(emu_node_lookup_child(t.get(), d1->qdma->ino, "qpairA", &la), 0);

        // (b) registered, never attached -> registry-only at shutdown.
        emu_resource *rb = nullptr;
        ASSERT_EQ(emu_device_register_resource(d1, 2, probe_teardown, probe_free,
                                               &pb, &rb),
                  0);

        // Device 2 is REVOKED before shutdown, leaving (c) as an inode-only
        // orphan (registry ref dropped by revoke, inode ref held by open fd).
        emu_device *d2 = nullptr;
        ASSERT_EQ(emu_node_tree_add_device(t.get(), "0000:62:00", &d2), 0);
        emu_node *qp_c = nullptr;
        (void) make_open_qpair(t.get(), d2, &pc, "qpairC", 3, &qp_c);
        ASSERT_EQ(emu_device_revoke(t.get(), "0000:62:00"), 0);
        EXPECT_EQ(pc.teardown_calls, 1);
        EXPECT_EQ(pc.free_calls, 0) << "orphan inode ref still held pre-shutdown";

        // Sanity on the discriminant the fix relies on:
        //  (a)/(b) still registered -> device != NULL; (c) revoked -> device NULL.
        EXPECT_NE(ra->device, nullptr);
        EXPECT_NE(rb->device, nullptr);
        // ra is reachable from both the registry and the node at this point.
        EXPECT_EQ(emu_device_find_resource(d1, 1), ra);
        EXPECT_EQ(qp_a->resource, ra);

        // No forgets for (a) or (c): the daemon shuts down here with all three
        // states live.  Tree destruction runs both cleanup loops.
    }

    // Each backing freed exactly once.  (free_calls==1 each; ASan would abort on
    // a double free of (a) and report a leak on a missed (c).)
    EXPECT_EQ(pa.free_calls, 1) << "(a) registered+attached double-freed or missed";
    EXPECT_EQ(pb.free_calls, 1) << "(b) registry-only";
    EXPECT_EQ(pc.free_calls, 1) << "(c) inode-only orphan leaked";
}
