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
 * node.cpp -- the semantics the ABI conformance suite (T11) will enforce.  It is
 * deliberately exhaustive about drop orders, idempotency of the two teardown
 * triggers, the dead-orphan state machine, nameless-qpair lifetime, and OOM /
 * error-path cleanup (the latter leaning on ASan).  Each test corresponds to a
 * numbered hammer item in the T5 adversarial brief.
 *
 * node.cpp is intentionally FUSE-free, so everything here is a pure unit test
 * driven directly against slash_emu_core; the kernel-dentry notifier is a
 * recording lambda.
 *
 * Port note (refcount model): a Resource is destroyed when BOTH its registry
 * @c shared_ptr and its inode @c shared_ptr drop.  The test counts that "free"
 * via a custom deleter installed on the resource's @c backing shared_ptr (it
 * fires exactly when the Resource object is destroyed), and the idempotent
 * "teardown" via the @ref slash::emu::ResourceTeardownFn.  The inode reference is
 * obtained from the registry's raw @c Resource* via @c shared_from_this().
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
// Recording notifier: captures every notify the spine fires, and (for the
// re-entry test) can call back INTO the tree to prove the lock is dropped.
// ---------------------------------------------------------------------------
struct NotifyRecord {
    Ino parent;
    Ino child;
    std::string name;
};

struct Recorder {
    std::vector<NotifyRecord> records;
    // Optional re-entry probe: if set, the notifier calls a locking op on this
    // tree from inside the notify callback.  If the lock were held across the
    // notifier, this would deadlock.
    NodeTree *reentry_tree = nullptr;
    int reentry_calls = 0;
    int reentry_ok = 0;
};

// ---------------------------------------------------------------------------
// Recording resource callbacks: count teardown and free events per resource.
// ---------------------------------------------------------------------------
struct ResourceProbe {
    int teardown_calls = 0;
    int free_calls = 0;
};

// Register a resource whose teardown / free both land on `probe` (NULL -> no
// callbacks, exercising the no-callback teardown/free path).
Resource *register_probed(Device *dev, uint32_t id, ResourceProbe *probe)
{
    Resource *res = dev->registerResource(
        id, probe != nullptr ? [probe](Resource &) { probe->teardown_calls++; }
                             : ResourceTeardownFn{});
    if (res != nullptr && probe != nullptr) {
        res->backing = std::shared_ptr<void>(
            probe, [](void *p) { static_cast<ResourceProbe *>(p)->free_calls++; });
    }
    return res;
}

// RAII wrapper around a tree.  Optionally installs a recording notifier whose
// records live in `recorder`.
class Tree {
public:
    Tree() : tree_(Notifier{}) {}

    explicit Tree(Recorder *recorder)
        : tree_([recorder](Ino parent, Ino child, const std::string &name) {
              recorder->records.push_back({parent, child, name});
              if (recorder->reentry_tree != nullptr) {
                  recorder->reentry_calls++;
                  // A lookup takes the tree lock.  Reaching here at all (no
                  // deadlock) proves the spine dropped the lock before firing
                  // the notifier.
                  Node *out = nullptr;
                  (void) recorder->reentry_tree->lookupChild(kRootIno, "nope",
                                                             &out);
                  recorder->reentry_ok++;
              }
          })
    {
    }

    NodeTree *get() { return &tree_; }

private:
    NodeTree tree_;
};

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

// Helper: stand up a device with one registered + attached + looked-up qpair.
// Returns the resource and writes the qpair node out.  The qpair has
// lookup_count == 1 (a held kernel reference / open fd).
Resource *make_open_qpair(NodeTree *tree, Device *dev, ResourceProbe *probe,
                          const char *name, uint32_t id, Node **qp_out)
{
    Resource *res = register_probed(dev, id, probe);
    EXPECT_NE(res, nullptr);
    Node *qp = tree->createChild(dev->qdma, name, NodeType::File, 0644);
    EXPECT_NE(qp, nullptr);
    EXPECT_EQ(tree->attachResource(qp, res->shared_from_this()), 0);
    Node *looked = nullptr;
    EXPECT_EQ(tree->lookupChild(dev->qdma->ino, name, &looked), 0);
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
    Device *dev = t.get()->addDevice("0000:61:00");
    ASSERT_NE(dev, nullptr);

    ASSERT_NE(register_probed(dev, 5, &probe), nullptr);
    // Never attached to any inode: registry ref only.
    ASSERT_EQ(t.get()->revokeDevice("0000:61:00"), 0);
    EXPECT_EQ(probe.teardown_calls, 1);
    EXPECT_EQ(probe.free_calls, 1) << "registry-only resource must free at revoke";
}

// A resource that holds ONLY the registry reference at tree destruction is freed
// exactly once (no inode ref, no leak, no double free).
TEST(Refcount, RegistryOnly_FreedAtTreeDestruction)
{
    ResourceProbe probe;
    {
        Tree t;
        Device *dev = t.get()->addDevice("0000:61:00");
        ASSERT_NE(dev, nullptr);
        ASSERT_NE(register_probed(dev, 5, &probe), nullptr);
    }
    EXPECT_EQ(probe.free_calls, 1);
}

// Drop order A: registry-then-inode (revoke first, forget second).  Free fires
// on the SECOND drop (inode), exactly once; teardown exactly once on the first.
TEST(Refcount, DropOrder_RegistryThenInode)
{
    ResourceProbe probe;
    Tree t;
    Device *dev = t.get()->addDevice("0000:61:00");
    ASSERT_NE(dev, nullptr);
    Node *qp = nullptr;
    Resource *res = make_open_qpair(t.get(), dev, &probe, "qpair0", 0, &qp);

    ASSERT_EQ(t.get()->revokeDevice("0000:61:00"), 0);  // registry ref drops
    EXPECT_EQ(probe.teardown_calls, 1);
    EXPECT_EQ(probe.free_calls, 0) << "inode ref still held";

    t.get()->forget(qp->ino, 1);  // inode ref drops
    EXPECT_EQ(probe.free_calls, 1);
    EXPECT_EQ(probe.teardown_calls, 1);
    (void) res;
}

// Drop order B: the cooperative inode-drop path ALSO unregisters.  Closing an
// undisturbed (unlinked) qpair runs destroyNodeLocked, which tears down,
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
    Device *dev = t.get()->addDevice("0000:61:00");
    ASSERT_NE(dev, nullptr);
    Node *qp = nullptr;
    (void) make_open_qpair(t.get(), dev, &probe, "qpair0", 0, &qp);

    // Sanity: it is registry-findable before close.
    EXPECT_NE(dev->findResource(0), nullptr);

    // Unlink-while-open then close: cooperative teardown.
    Ino ino = qp->ino;
    t.get()->unlink(qp);
    t.get()->forget(ino, 1);

    // Both refs dropped in one shot: freed immediately, teardown once, and the
    // resource is gone from the registry (no dangling registry entry for revoke
    // to trip over).
    EXPECT_EQ(probe.teardown_calls, 1);
    EXPECT_EQ(probe.free_calls, 1) << "cooperative close drops BOTH refs";
    EXPECT_EQ(dev->findResource(0), nullptr)
        << "cooperative teardown must unregister";

    // A subsequent revoke is a clean no-op -- no double free / use-after-free.
    ASSERT_EQ(t.get()->revokeDevice("0000:61:00"), 0);
    EXPECT_EQ(probe.teardown_calls, 1);
    EXPECT_EQ(probe.free_calls, 1);
}

// LEAK HUNT: a resource left inode-only-referenced (registry ref already
// dropped) at tree destruction.  Reachable when a device is revoked while a
// qpair fd is still open: revoke drops the registry ref but the orphan node
// keeps the inode ref; if the daemon then shuts down WITHOUT a forget, the
// resource backing must still be freed exactly once when the node table is
// destroyed.
TEST(Refcount, InodeOnlyResourceFreedAtTreeDestruction)
{
    ResourceProbe probe;
    {
        Tree t;
        Device *dev = t.get()->addDevice("0000:61:00");
        ASSERT_NE(dev, nullptr);
        Node *qp = nullptr;
        (void) make_open_qpair(t.get(), dev, &probe, "qpair0", 0, &qp);

        // Forced removal drops the registry ref; the still-open node keeps the
        // inode ref, so the resource survives as a dead orphan.
        ASSERT_EQ(t.get()->revokeDevice("0000:61:00"), 0);
        EXPECT_EQ(probe.teardown_calls, 1);
        EXPECT_EQ(probe.free_calls, 0) << "inode ref keeps it alive";

        // No forget: the daemon shuts down with the orphan still inode-held.
        // Tree destruction must reclaim the backing.
    }
    EXPECT_EQ(probe.free_calls, 1)
        << "inode-only orphan leaked at tree destruction";
}

// ===========================================================================
// 2. IDEMPOTENT TEARDOWN: first of {cooperative, forced} does the work; the
//    second is a true no-op.  Both orders + double-forced.
// ===========================================================================

TEST(IdempotentTeardown, ForcedThenCooperative)
{
    ResourceProbe probe;
    Tree t;
    Device *dev = t.get()->addDevice("0000:61:00");
    ASSERT_NE(dev, nullptr);
    Node *qp = nullptr;
    (void) make_open_qpair(t.get(), dev, &probe, "qpair0", 0, &qp);

    ASSERT_EQ(t.get()->revokeDevice("0000:61:00"), 0);  // forced
    t.get()->forget(qp->ino, 1);                        // cooperative
    EXPECT_EQ(probe.teardown_calls, 1);
    EXPECT_EQ(probe.free_calls, 1);
}

// cooperative-then-forced: close the open (unlinked) qpair first (cooperative
// teardown + free of both refs), THEN revoke the device.  Revoke must not touch
// the freed resource (it was unregistered on cooperative teardown) and teardown
// stays at one.
TEST(IdempotentTeardown, CooperativeThenForced)
{
    ResourceProbe probe;
    Tree t;
    Device *dev = t.get()->addDevice("0000:61:00");
    ASSERT_NE(dev, nullptr);
    Node *qp = nullptr;
    (void) make_open_qpair(t.get(), dev, &probe, "qpair0", 0, &qp);

    // Cooperative: unlink + close -> teardown + unregister + free (both refs
    // gone because the cooperative path unregisters too).
    Ino ino = qp->ino;
    t.get()->unlink(qp);
    t.get()->forget(ino, 1);
    EXPECT_EQ(probe.teardown_calls, 1);
    EXPECT_EQ(probe.free_calls, 1) << "cooperative close frees an undisturbed qpair";

    // Forced removal afterwards must be a clean no-op (resource already gone from
    // the registry; no double teardown, no use-after-free).
    ASSERT_EQ(t.get()->revokeDevice("0000:61:00"), 0);
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
    Device *dev = t.get()->addDevice("0000:61:00");
    ASSERT_NE(dev, nullptr);
    Node *qp = nullptr;
    (void) make_open_qpair(t.get(), dev, &probe, "qpair0", 0, &qp);

    ASSERT_EQ(t.get()->revokeDevice("0000:61:00"), 0);
    ASSERT_EQ(t.get()->revokeDevice("0000:61:00"), 0);  // no-op
    EXPECT_EQ(probe.teardown_calls, 1);
    EXPECT_EQ(probe.free_calls, 0) << "orphan inode ref still held";

    t.get()->forget(qp->ino, 1);
    EXPECT_EQ(probe.free_calls, 1);
    EXPECT_EQ(probe.teardown_calls, 1);
}

// ===========================================================================
// 3. REVOCATION STATE MACHINE: ENOENT on lookup, ENODEV on open handles,
//    idempotent, notify fires for each revoked name.
// ===========================================================================

// Revoke is a success no-op on an absent BDF and on an already-revoked BDF, and
// does not fire any notification for the absent case.
TEST(RevocationStateMachine, IdempotentAndAbsentNoNotify)
{
    Recorder recorder;
    Tree t(&recorder);
    ASSERT_NE(t.get()->addDevice("0000:61:00"), nullptr);

    ASSERT_EQ(t.get()->revokeDevice("0000:61:00"), 0);
    size_t after_first = recorder.records.size();
    EXPECT_GT(after_first, 0u);

    // Re-revoking the same BDF: no device found -> no new notifications.
    ASSERT_EQ(t.get()->revokeDevice("0000:61:00"), 0);
    EXPECT_EQ(recorder.records.size(), after_first);

    // A BDF that never existed: success, no notifications.
    ASSERT_EQ(t.get()->revokeDevice("0000:99:00"), 0);
    EXPECT_EQ(recorder.records.size(), after_first);
}

// notify must fire for EVERY revoked name: the <BDF> dir, bars/, qdma/, and any
// endpoint files under them.
TEST(RevocationStateMachine, NotifyDeleteFiresForEveryName)
{
    Recorder recorder;
    Tree t(&recorder);
    Device *dev = t.get()->addDevice("0000:61:00");
    ASSERT_NE(dev, nullptr);

    // Attach endpoint files so revocation must invalidate them too.
    ASSERT_NE(t.get()->createChild(dev->dir, "info", NodeType::File, 0444),
              nullptr);
    ASSERT_NE(t.get()->createChild(dev->bars, "bar0", NodeType::File, 0644),
              nullptr);

    ASSERT_EQ(t.get()->revokeDevice("0000:61:00"), 0);

    std::set<std::string> names;
    for (const auto &r : recorder.records) {
        names.insert(r.name);
    }
    EXPECT_EQ(names.count("0000:61:00"), 1u);
    EXPECT_EQ(names.count("bars"), 1u);
    EXPECT_EQ(names.count("qdma"), 1u);
    EXPECT_EQ(names.count("info"), 1u);
    EXPECT_EQ(names.count("bar0"), 1u);
}

// resourceCheck / isLive report -ENODEV after revoke on a handle still held;
// -ENOENT semantics (lookup miss) for the name.
TEST(RevocationStateMachine, EnodevOnHandleEnoentOnName)
{
    Tree t;
    Device *dev = t.get()->addDevice("0000:61:00");
    ASSERT_NE(dev, nullptr);
    Node *qp = nullptr;
    Resource *res = make_open_qpair(t.get(), dev, nullptr, "qpair0", 0, &qp);

    EXPECT_EQ(t.get()->resourceCheck(res), 0);
    EXPECT_EQ(t.get()->isLive(qp->ino), 0);

    ASSERT_EQ(t.get()->revokeDevice("0000:61:00"), 0);

    // Op on the open handle: -ENODEV.
    EXPECT_EQ(t.get()->resourceCheck(res), -ENODEV);
    EXPECT_EQ(t.get()->isLive(qp->ino), -ENODEV);

    // New lookup of any removed name: not found.
    Node *out = nullptr;
    EXPECT_EQ(t.get()->lookupChild(kRootIno, "0000:61:00", &out), -ENOENT);

    t.get()->forget(qp->ino, 1);
}

// ===========================================================================
// 4. NAMELESS QPAIR LIFETIME: unlink != revoke.  Stays live, gone from readdir
//    / by-name lookup, still registry-findable by qid, torn down on final close.
// ===========================================================================

TEST(NamelessQpair, UnlinkKeepsLiveAndRegistryFindable)
{
    ResourceProbe probe;
    Tree t;
    Device *dev = t.get()->addDevice("0000:61:00");
    ASSERT_NE(dev, nullptr);
    Node *qp = nullptr;
    Resource *res = make_open_qpair(t.get(), dev, &probe, "qpair7", 7, &qp);
    Ino ino = qp->ino;

    t.get()->unlink(qp);

    // Still live -- unlink is delete-on-last-close, not revocation.
    EXPECT_EQ(t.get()->resourceCheck(res), 0);
    EXPECT_EQ(t.get()->isLive(ino), 0);
    EXPECT_EQ(probe.teardown_calls, 0);

    // Gone from the directory listing and by-name lookup.
    EXPECT_TRUE(readdir_names(t.get(), dev->qdma->ino).empty());
    Node *byname = nullptr;
    EXPECT_EQ(t.get()->lookupChild(dev->qdma->ino, "qpair7", &byname), -ENOENT);

    // Still reachable through the registry by qid.
    EXPECT_EQ(dev->findResource(7), res);

    // Final close -> cooperative teardown exactly once, then freed.
    t.get()->forget(ino, 1);
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
    Device *dev = t.get()->addDevice("0000:61:00");
    ASSERT_NE(dev, nullptr);
    Node *qp = nullptr;
    Resource *res = make_open_qpair(t.get(), dev, &probe, "qpair7", 7, &qp);
    Ino ino = qp->ino;

    t.get()->unlink(qp);  // nameless, still registered + live
    ASSERT_EQ(dev->findResource(7), res);

    ASSERT_EQ(t.get()->revokeDevice("0000:61:00"), 0);
    EXPECT_EQ(probe.teardown_calls, 1) << "registry reached the nameless qpair";
    EXPECT_EQ(t.get()->resourceCheck(res), -ENODEV);
    EXPECT_EQ(probe.free_calls, 0) << "orphan inode ref still held";

    t.get()->forget(ino, 1);
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
    Device *dev = t.get()->addDevice("0000:61:00");
    ASSERT_NE(dev, nullptr);

    Node *bar = t.get()->createChild(dev->bars, "bar0", NodeType::File, 0644);
    ASSERT_NE(bar, nullptr);
    Node *opened = nullptr;
    ASSERT_EQ(t.get()->lookupChild(dev->bars->ino, "bar0", &opened), 0);
    Ino ino = bar->ino;

    ASSERT_EQ(t.get()->revokeDevice("0000:61:00"), 0);

    // Dead orphan: still resolvable by inode (it exists), but dead.
    EXPECT_EQ(t.get()->isLive(ino), -ENODEV);
    EXPECT_NE(t.get()->lookupIno(ino), nullptr);

    // After forget, reaped: the inode no longer exists.
    t.get()->forget(ino, 1);
    EXPECT_EQ(t.get()->lookupIno(ino), nullptr);
    EXPECT_EQ(t.get()->isLive(ino), -ENOENT);
}

// REGRESSION for the self-caught use-after-free: a revoked-then-reaped <BDF>
// node must not leave a dangling pointer in root's children.  Here the <BDF>
// node is looked up (held), revoked (survives as orphan, detached from root),
// then forgotten (reaped).  Afterward root's child list must be intact: a fresh
// add of a DIFFERENT device and a root readdir must not touch freed memory.
TEST(DeadOrphan, ReapDoesNotDangleInParentChildList)
{
    Tree t;
    Device *dev = t.get()->addDevice("0000:61:00");
    ASSERT_NE(dev, nullptr);

    // Hold the <BDF> dir itself (kernel lookup on the device directory).
    Node *held = nullptr;
    ASSERT_EQ(t.get()->lookupChild(kRootIno, "0000:61:00", &held), 0);
    Ino bdf_ino = held->ino;

    ASSERT_EQ(t.get()->revokeDevice("0000:61:00"), 0);
    // The orphan is detached from root immediately (parent cleared) so root's
    // child list holds no reference to it even while it lives.
    EXPECT_TRUE(readdir_names(t.get(), kRootIno).empty());

    // Reap the orphan.
    t.get()->forget(bdf_ino, 1);

    // Root's child list must be intact: adding another device and listing root
    // must work and show only the new one (ASan would flag any dangling ref).
    ASSERT_NE(t.get()->addDevice("0000:62:00"), nullptr);
    auto kids = readdir_names(t.get(), kRootIno);
    EXPECT_EQ(kids.size(), 1u);
    EXPECT_EQ(kids.count("0000:62:00"), 1u);
}

// A revoked subtree with the <BDF> dir held but a child (qdma) NOT separately
// held: revoke must reap the child immediately (lookup_count 0) yet keep the
// held parent as an orphan, with no dangling child pointer left in the orphan's
// child list (revoke clears children).  A readdir on the dead orphan dir must
// return no children and not touch freed child nodes.
TEST(DeadOrphan, OrphanDirHasNoDanglingChildrenAfterRevoke)
{
    Tree t;
    Device *dev = t.get()->addDevice("0000:61:00");
    ASSERT_NE(dev, nullptr);

    Node *held = nullptr;
    ASSERT_EQ(t.get()->lookupChild(kRootIno, "0000:61:00", &held), 0);
    Ino bdf_ino = held->ino;

    ASSERT_EQ(t.get()->revokeDevice("0000:61:00"), 0);

    // The held <BDF> dir is a dead orphan.  Its children (bars/qdma) were reaped
    // (no separate lookups), and its child list was cleared, so a readdir over
    // the orphan must enumerate nothing and never dereference a freed child.
    std::set<std::string> names = readdir_names(t.get(), bdf_ino);
    EXPECT_TRUE(names.empty());

    t.get()->forget(bdf_ino, 1);
}

// A partial-forget (nlookup < lookup_count) must keep a revoked node alive as an
// orphan; only the final balancing forget reaps it.
TEST(DeadOrphan, PartialForgetKeepsOrphanAlive)
{
    Tree t;
    Device *dev = t.get()->addDevice("0000:61:00");
    ASSERT_NE(dev, nullptr);

    Node *bar = t.get()->createChild(dev->bars, "bar0", NodeType::File, 0644);
    ASSERT_NE(bar, nullptr);
    // Two outstanding lookups.
    Node *o1 = nullptr;
    Node *o2 = nullptr;
    ASSERT_EQ(t.get()->lookupChild(dev->bars->ino, "bar0", &o1), 0);
    ASSERT_EQ(t.get()->lookupChild(dev->bars->ino, "bar0", &o2), 0);
    Ino ino = bar->ino;

    ASSERT_EQ(t.get()->revokeDevice("0000:61:00"), 0);

    // Drop one of two: still alive as an orphan.
    t.get()->forget(ino, 1);
    EXPECT_NE(t.get()->lookupIno(ino), nullptr);
    EXPECT_EQ(t.get()->isLive(ino), -ENODEV);

    // Drop the last: reaped.
    t.get()->forget(ino, 1);
    EXPECT_EQ(t.get()->lookupIno(ino), nullptr);
}

// ===========================================================================
// 6. LOCKING / RE-ENTRY: the tree mutex must NEVER be held across notify.  The
//    notifier re-enters the tree with a locking op; if the lock were held this
//    would self-deadlock (the test would hang and CTest would time out).
// ===========================================================================

TEST(Locking, NotifierMayReenterTreeWithoutDeadlock)
{
    Recorder recorder;
    Tree t(&recorder);
    recorder.reentry_tree = t.get();  // notify will call back in

    ASSERT_NE(t.get()->addDevice("0000:61:00"), nullptr);

    ASSERT_EQ(t.get()->revokeDevice("0000:61:00"), 0);

    // Every notification re-entered the tree successfully (lock was dropped).
    EXPECT_GT(recorder.reentry_calls, 0);
    EXPECT_EQ(recorder.reentry_calls, recorder.reentry_ok);
}

// ===========================================================================
// 7. TREE / MATERIALIZE: idempotent re-add (RESCAN), multi-accelerator, and
//    re-add of a BDF AFTER it was revoked (it can be materialized fresh).
// ===========================================================================

TEST(Materialize, ReAddAfterRevokeCreatesFreshDevice)
{
    Tree t;
    Device *dev = t.get()->addDevice("0000:61:00");
    ASSERT_NE(dev, nullptr);
    ASSERT_EQ(t.get()->revokeDevice("0000:61:00"), 0);

    // After revoke, find_device must miss (it filters on live).
    EXPECT_EQ(t.get()->findDevice("0000:61:00"), nullptr);

    // RESCAN re-adds it: a NEW live device, and the name resolves again.
    Device *dev2 = t.get()->addDevice("0000:61:00");
    ASSERT_NE(dev2, nullptr);
    EXPECT_NE(dev2, dev) << "must be a fresh device, not the revoked one";

    Node *out = nullptr;
    EXPECT_EQ(t.get()->lookupChild(kRootIno, "0000:61:00", &out), 0);
    t.get()->forget(out->ino, 1);

    // Root shows exactly one live BDF dir (the dead one is unlinked).
    auto kids = readdir_names(t.get(), kRootIno);
    EXPECT_EQ(kids.size(), 1u);
}

// Re-add of a still-live device is idempotent and returns the same device with
// no duplicate subtree.
TEST(Materialize, ReAddLiveIsIdempotent)
{
    Tree t;
    Device *a = t.get()->addDevice("0000:61:00");
    ASSERT_NE(a, nullptr);

    // Attach a file, then re-add: the existing subtree (and the file) survives.
    ASSERT_NE(t.get()->createChild(a->dir, "info", NodeType::File, 0444),
              nullptr);
    Device *b = t.get()->addDevice("0000:61:00");
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
    Device *dev = t.get()->addDevice("0000:61:00");
    ASSERT_NE(dev, nullptr);

    std::vector<Node *> held_nodes;
    for (int i = 0; i < kN; i++) {
        Resource *res = register_probed(dev, (uint32_t) i, &probes[i]);
        ASSERT_NE(res, nullptr);
        if (i % 2 == 0) {
            // Even: attach + hold open (inode ref outlives revoke).
            std::string name = "qpair" + std::to_string(i);
            Node *qp = t.get()->createChild(dev->qdma, name, NodeType::File,
                                            0644);
            ASSERT_NE(qp, nullptr);
            ASSERT_EQ(t.get()->attachResource(qp, res->shared_from_this()), 0);
            Node *looked = nullptr;
            ASSERT_EQ(t.get()->lookupChild(dev->qdma->ino, name, &looked), 0);
            held_nodes.push_back(qp);
        }
        // Odd: registry-only nameless resource.
    }

    ASSERT_EQ(t.get()->revokeDevice("0000:61:00"), 0);

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
        t.get()->forget(qp->ino, 1);
    }
    for (int i = 0; i < kN; i += 2) {
        EXPECT_EQ(probes[i].free_calls, 1) << "freed on forget " << i;
        EXPECT_EQ(probes[i].teardown_calls, 1);
    }
}

// DELTA RE-VERIFICATION of the shutdown teardown: force three resource states to
// coexist at tree destruction and prove each backing is freed EXACTLY once -- no
// double free (the guard), no miss (the leak).  ASan is the witness: a double
// free aborts, a miss reports a leak.  In the C++ model the shared_ptr ownership
// makes this automatic (devices_ destructs before nodes_, so a both-roots
// resource is freed exactly once on the surviving inode ref), but the states are
// codified so a regression that breaks the invariant is caught.
//
//   (a) STILL REGISTERED + INODE-ATTACHED: reachable from BOTH roots.
//   (b) STILL REGISTERED, registry-only: reachable only from the registry.
//   (c) INODE-ONLY ORPHAN: a prior revoke of a SECOND device dropped its
//       registry ref while an open handle kept the inode ref.
TEST(Shutdown, MixedRegisteredAndOrphanResourcesFreedExactlyOnce)
{
    ResourceProbe pa;  // (a) registered + inode-attached, never forgotten
    ResourceProbe pb;  // (b) registered, registry-only
    ResourceProbe pc;  // (c) inode-only orphan from a revoked device
    {
        Tree t;

        // Device 1 stays LIVE to shutdown: holds (a) and (b).
        Device *d1 = t.get()->addDevice("0000:61:00");
        ASSERT_NE(d1, nullptr);

        // (a) registered + attached to a held-open node, NEVER forgotten -> at
        //     shutdown it is BOTH in d1's registry and pointed to by the node.
        Resource *ra = register_probed(d1, 1, &pa);
        ASSERT_NE(ra, nullptr);
        Node *qp_a =
            t.get()->createChild(d1->qdma, "qpairA", NodeType::File, 0644);
        ASSERT_NE(qp_a, nullptr);
        ASSERT_EQ(t.get()->attachResource(qp_a, ra->shared_from_this()), 0);
        Node *la = nullptr;
        ASSERT_EQ(t.get()->lookupChild(d1->qdma->ino, "qpairA", &la), 0);

        // (b) registered, never attached -> registry-only at shutdown.
        Resource *rb = register_probed(d1, 2, &pb);
        ASSERT_NE(rb, nullptr);

        // Device 2 is REVOKED before shutdown, leaving (c) as an inode-only
        // orphan (registry ref dropped by revoke, inode ref held by open fd).
        Device *d2 = t.get()->addDevice("0000:62:00");
        ASSERT_NE(d2, nullptr);
        Node *qp_c = nullptr;
        (void) make_open_qpair(t.get(), d2, &pc, "qpairC", 3, &qp_c);
        ASSERT_EQ(t.get()->revokeDevice("0000:62:00"), 0);
        EXPECT_EQ(pc.teardown_calls, 1);
        EXPECT_EQ(pc.free_calls, 0) << "orphan inode ref still held pre-shutdown";

        // Sanity: (a)/(b) still registered; ra reachable from both roots.
        EXPECT_EQ(d1->findResource(1), ra);
        EXPECT_EQ(qp_a->resource.get(), ra);

        // No forgets for (a) or (c): the daemon shuts down here with all three
        // states live.  Tree destruction reclaims every backing.
    }

    // Each backing freed exactly once.
    EXPECT_EQ(pa.free_calls, 1) << "(a) registered+attached double-freed or missed";
    EXPECT_EQ(pb.free_calls, 1) << "(b) registry-only";
    EXPECT_EQ(pc.free_calls, 1) << "(c) inode-only orphan leaked";
}
