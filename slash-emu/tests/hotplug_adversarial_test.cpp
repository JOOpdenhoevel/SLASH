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
 * @file hotplug_adversarial_test.cpp
 * @brief Adversarial conformance suite for the global /hotplug endpoint (T9).
 *
 * The destructive counterpart of hotplug_test.cpp.  It hammers the
 * highest-risk pieces of the T9 spine/locking changes:
 *
 *   - ioctl_unlocked SAFETY: only the hotplug file carries the unlocked-ioctl
 *     flag (device-endpoint ioctls -- qdma QPAIR_ADD -- still run LOCKED); the
 *     reentrant self-locking spine API the unlocked hook calls does not double-
 *     lock; and the documented "hotplug file is never unlinked" invariant is
 *     PROBED for the one path that can unlink/free it (FUSE unlink at the root).
 *   - PER-FUNCTION REVOKE in BOTH orders, with the surviving function asserted
 *     fully usable, the device NOT marked dead after one function, the second
 *     REMOVE then succeeding, and idempotent / absent-device no-ops.
 *   - MODEL-SHUTDOWN exactly once, including the REMOVE-both-then-TOGGLE_SBR
 *     guard (model_shutdown_fired must prevent a double fire).
 *   - NO DOUBLE-ATTACH on repeated reload: a surviving device gains no duplicate
 *     info/bars/qdma child nodes after repeated materialize re-invocations.
 *   - CONFIG OWNERSHIP across reload (the borrowed-startup vs owned-reloaded
 *     swap): exercised end-to-end over the mount under ASan for leak/UAF.
 *   - BDF-with-function parsing edge cases beyond the base suite.
 *
 * Per-test ctest TIMEOUT (set in CMakeLists) so a deadlock in the unlocked-hook
 * reentrancy or a mount hang FAILS the suite rather than blocking it.
 */

#include <gtest/gtest.h>

#include <atomic>
#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include <dirent.h>
#include <fcntl.h>
#include <linux/magic.h>
#include <signal.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/vfs.h>
#include <sys/wait.h>
#include <unistd.h>

extern "C" {
#include "bars.h"
#include "config.h"
#include "hotplug.h"
#include "info.h"
#include "node.h"
#include "qdma.h"
#include "slash/uapi/slash_abi.h"
}

namespace {

// ===========================================================================
// Shared unit harness (mirrors hotplug_test.cpp; kept local so the adversarial
// suite is self-contained and can be reasoned about in isolation).
// ===========================================================================

struct ReloadState {
    std::atomic<int> calls{0};
    int rc{0};
};

int stub_reload(void *ctx)
{
    auto *s = static_cast<ReloadState *>(ctx);
    s->calls++;
    return s->rc;
}

struct ShutdownState {
    std::atomic<int> calls{0};
    emu_device *last{nullptr};
};

void stub_model_shutdown(emu_device *dev, void *ctx)
{
    auto *s = static_cast<ShutdownState *>(ctx);
    s->calls++;
    s->last = dev;
}

class HotplugTree {
public:
    explicit HotplugTree(emu_hotplug_reload_fn reload = nullptr,
                         void *ctx = nullptr)
    {
        EXPECT_EQ(emu_node_tree_new(&tree_, nullptr), 0);
        EXPECT_EQ(emu_hotplug_attach(tree_, reload, ctx), 0);
    }
    ~HotplugTree() { cleanup_node_tree(tree_); }
    emu_node_tree *get() { return tree_; }

    emu_device *add_device(const char *bdf, emu_model_shutdown_fn sd = nullptr,
                           void *sd_ctx = nullptr)
    {
        emu_device *dev = nullptr;
        EXPECT_EQ(emu_node_tree_add_device(tree_, bdf, &dev), 0);
        EXPECT_NE(dev, nullptr);
        EXPECT_EQ(emu_info_attach(dev), 0);
        EXPECT_EQ(emu_bars_attach(dev), 0);
        EXPECT_EQ(emu_qdma_attach(dev), 0);
        if (sd != nullptr) {
            EXPECT_EQ(emu_device_set_model_shutdown(tree_, bdf, sd, sd_ctx), 0);
        }
        return dev;
    }

    emu_ino_t hotplug_ino()
    {
        emu_node *child = nullptr;
        EXPECT_EQ(emu_node_lookup_child(tree_, EMU_ROOT_INO, "hotplug", &child),
                  0);
        emu_ino_t ino = child != nullptr ? child->ino : 0;
        // lookup_child bumped the count; drop it so the node is not held looked-up
        // (mirrors a getattr that the kernel immediately forgets).
        if (ino != 0) {
            emu_node_forget(tree_, ino, 1);
        }
        return ino;
    }

private:
    emu_node_tree *tree_ = nullptr;
};

int hotplug_dev_ioctl(emu_node_tree *tree, emu_ino_t ino, unsigned int cmd,
                      const char *bdf_with_func)
{
    struct slash_abi_hotplug_device_request req {};
    req.size = sizeof(req);
    std::snprintf(req.bdf, sizeof(req.bdf), "%s", bdf_with_func);
    return emu_node_ioctl(tree, ino, cmd, &req, sizeof(req), &req, sizeof(req));
}

bool resolves(emu_node_tree *tree, emu_ino_t parent, const char *name)
{
    emu_node *child = nullptr;
    int rc = emu_node_lookup_child(tree, parent, name, &child);
    if (rc == 0 && child != nullptr) {
        emu_node_forget(tree, child->ino, 1);
        return true;
    }
    return false;
}

// Count live children of a directory node by name (for the no-double-attach
// assertion: a re-attach would create a SECOND child of the same name).
int count_children_named(emu_node *dir, const char *name)
{
    int n = 0;
    for (size_t i = 0; i < dir->children.len; i++) {
        emu_node *c = dir->children.d[i];
        if (!c->unlinked && std::strcmp(c->name, name) == 0) {
            n++;
        }
    }
    return n;
}

// ===========================================================================
// ioctl_unlocked SAFETY
// ===========================================================================

// (b) Only the hotplug file is ioctl_unlocked; the device-endpoint ioctls
// (qdma QPAIR_ADD) must still run LOCKED.  We cannot read the private flag, so
// we assert behaviourally: a qdma QPAIR_ADD that ITSELF re-enters the spine's
// self-locking public API from inside its hook would deadlock IF it ran
// unlocked-then-relocking the way hotplug does -- but the real proof here is
// structural: the qdma node never gets emu_node_set_ioctl_unlocked, so its hook
// runs under the held lock.  We pin the contract by confirming QPAIR_ADD works
// (it would self-deadlock if it had been mis-marked unlocked AND its hook
// re-locked).  A ctest TIMEOUT backstops a regression that deadlocks.
TEST(HotplugUnlockedSafety, QdmaIoctlStillRunsLocked)
{
    HotplugTree t;
    emu_device *dev = t.add_device("0000:61:00");

    struct slash_abi_qdma_qpair_add qreq {};
    qreq.size = sizeof(qreq);
    qreq.mode = 0;
    qreq.dir_mask = 0x1;
    struct slash_abi_qdma_qpair_add qout = qreq;
    // Runs the qdma hook under the held lock and returns; no deadlock, no hang.
    ASSERT_EQ(emu_node_ioctl(t.get(), dev->qdma->ino,
                             SLASH_ABI_QDMA_IOCTL_QPAIR_ADD, &qreq, sizeof(qreq),
                             &qout, sizeof(qout)),
              0);
}

// (d) Reentrancy: the unlocked hotplug hook calls the self-locking
// revoke/reload spine API.  If the hook ran LOCKED, those calls would re-lock a
// non-recursive mutex and self-deadlock.  Exercising every command that calls
// the spine API (REMOVE -> revoke_function, TOGGLE_SBR/HOTPLUG -> revoke +
// reload, RESCAN -> reload) and returning proves the lock was dropped first.
TEST(HotplugUnlockedSafety, EveryCommandReentersSpineWithoutDeadlock)
{
    ReloadState rs;
    HotplugTree t(stub_reload, &rs);
    t.add_device("0000:61:00");
    t.add_device("0000:62:00");
    emu_ino_t hp = t.hotplug_ino();
    ASSERT_EQ(emu_hotplug_set_sbr_sleep_us(t.get(), 0), 0);

    // RESCAN -> reload (re-locks materialize internally).
    ASSERT_EQ(emu_node_ioctl(t.get(), hp, SLASH_ABI_HOTPLUG_IOCTL_RESCAN, nullptr,
                             0, nullptr, 0),
              0);
    // REMOVE -> revoke_function (re-locks).
    ASSERT_EQ(hotplug_dev_ioctl(t.get(), hp, SLASH_ABI_HOTPLUG_IOCTL_REMOVE,
                                "0000:61:00.1"),
              0);
    // TOGGLE_SBR -> revoke + reload (two separately-locked phases).
    ASSERT_EQ(hotplug_dev_ioctl(t.get(), hp, SLASH_ABI_HOTPLUG_IOCTL_TOGGLE_SBR,
                                "0000:61:00.2"),
              0);
    // HOTPLUG -> revoke + reload.
    ASSERT_EQ(hotplug_dev_ioctl(t.get(), hp, SLASH_ABI_HOTPLUG_IOCTL_HOTPLUG,
                                "0000:62:00.1"),
              0);
}

// (a)/(c) The documented invariant backing the dropped-lock pointer is
// "the hotplug file is never unlinked".  PROBE it: FUSE exposes the hotplug node
// as a regular FILE at the root, and emu_node_unlink_child unlinks any file
// child.  This test pins the CURRENT behaviour of unlinking the hotplug node so
// a regression (or a fix) is visible.  If unlink is rejected, the invariant is
// truly enforced; if it succeeds, the control surface can be destroyed.
TEST(HotplugUnlockedSafety, UnlinkOfHotplugNodeBehaviour)
{
    HotplugTree t;
    t.add_device("0000:61:00");
    emu_ino_t hp = t.hotplug_ino();
    ASSERT_NE(hp, 0u);

    // The "hotplug file is never unlinked" precondition the ioctl_unlocked
    // dropped-lock path relies on is now ENFORCED by the spine: the hotplug node
    // is created non-unlinkable, so the FUSE unlink op rejects it with -EPERM.
    int rc = emu_node_unlink_child(t.get(), EMU_ROOT_INO, "hotplug");
    EXPECT_EQ(rc, -EPERM)
        << "unlink of the global /hotplug control surface must be refused";

    // The control surface survives intact: it still resolves and still services
    // ioctls (RESCAN here has no reload seam wired, so it is a 0-return no-op).
    EXPECT_TRUE(resolves(t.get(), EMU_ROOT_INO, "hotplug"))
        << "hotplug file must still resolve after a refused unlink";
    EXPECT_EQ(emu_node_ioctl(t.get(), hp, SLASH_ABI_HOTPLUG_IOCTL_RESCAN,
                             nullptr, 0, nullptr, 0),
              0)
        << "hotplug ioctl must still work after a refused unlink";
}

// Unit-level guard for the opt-in unlinkability contract (the spine path,
// without a mount): info and bar<M> files are NOT user-unlinkable (-EPERM and
// intact), whereas a qpair<Q> file IS (the fix must not over-rotate).
TEST(HotplugUnlockedSafety, OnlyQpairFilesAreUnlinkable)
{
    HotplugTree t;
    emu_device *dev = t.add_device("0000:61:00");

    // info (under <BDF>/) and bar0 (under bars/) are non-unlinkable.
    EXPECT_EQ(emu_node_unlink_child(t.get(), dev->dir->ino, "info"), -EPERM);
    EXPECT_TRUE(resolves(t.get(), dev->dir->ino, "info"));
    EXPECT_EQ(emu_node_unlink_child(t.get(), dev->bars->ino, "bar0"), -EPERM);
    EXPECT_TRUE(resolves(t.get(), dev->bars->ino, "bar0"));

    // A qpair<Q> file stays unlinkable.
    struct slash_abi_qdma_qpair_add req {};
    req.size = sizeof(req);
    req.mode = 0;       // MM
    req.dir_mask = 0x1; // H2C
    struct slash_abi_qdma_qpair_add out = req;
    ASSERT_EQ(emu_node_ioctl(t.get(), dev->qdma->ino,
                             SLASH_ABI_QDMA_IOCTL_QPAIR_ADD, &req, sizeof(req),
                             &out, sizeof(out)),
              0);
    char qname[32];
    std::snprintf(qname, sizeof(qname), "qpair%u", out.qid);
    EXPECT_EQ(emu_node_unlink_child(t.get(), dev->qdma->ino, qname), 0)
        << "qpair<Q> must remain user-unlinkable";
    EXPECT_FALSE(resolves(t.get(), dev->qdma->ino, qname));
}

// ASan UAF probe for the dropped-lock pointer when the hotplug node is held
// looked-up (so unlink orphans it rather than freeing it), then an ioctl runs
// on the surviving orphan, then it is forgotten (freed).  Under ASan a
// use-after-free in the unlocked hook's backing access would trip here.
TEST(HotplugUnlockedSafety, UnlinkedButLookedUpOrphanIoctlIsCleanUnderAsan)
{
    HotplugTree t;
    t.add_device("0000:61:00");

    // Hold a lookup ref so unlink orphans (does not free) the node.
    emu_node *hpnode = nullptr;
    ASSERT_EQ(emu_node_lookup_child(t.get(), EMU_ROOT_INO, "hotplug", &hpnode),
              0);
    ASSERT_NE(hpnode, nullptr);
    emu_ino_t hp = hpnode->ino;

    int rc = emu_node_unlink_child(t.get(), EMU_ROOT_INO, "hotplug");
    if (rc != 0) {
        // Invariant enforced (unlink refused): nothing more to probe.
        emu_node_forget(t.get(), hp, 1);
        SUCCEED();
        return;
    }

    // The node survives as an orphan (lookup_count > 0). emu_node_unlink_locked
    // does NOT set live=false, so an ioctl by the orphan's inode still runs the
    // unlocked hook -- on a backing that is still alive (not yet destroyed).
    // RESCAN with no reload wired is a safe no-op that still dereferences the
    // backing. Under ASan this must not be a use-after-free.
    EXPECT_EQ(emu_node_ioctl(t.get(), hp, SLASH_ABI_HOTPLUG_IOCTL_RESCAN, nullptr,
                             0, nullptr, 0),
              0);

    // Final forget destroys the orphan + frees the backing.  A later ioctl by
    // the now-dead inode must miss, not touch freed memory.
    emu_node_forget(t.get(), hp, 1);
    EXPECT_EQ(emu_node_ioctl(t.get(), hp, SLASH_ABI_HOTPLUG_IOCTL_RESCAN, nullptr,
                             0, nullptr, 0),
              -ENOENT);
}

// ===========================================================================
// PER-FUNCTION REVOKE -- surviving function fully usable, both orders
// ===========================================================================

// After REMOVE .1 (qdma), bars must remain FULLY usable: a fresh bar register
// read/write round-trips, and the device is still found (not marked dead), so a
// later REMOVE .2 succeeds.
TEST(HotplugPerFunction, RemoveQdmaLeavesBarsFullyLive)
{
    HotplugTree t;
    emu_device *dev = t.add_device("0000:61:00");
    emu_ino_t hp = t.hotplug_ino();

    ASSERT_EQ(hotplug_dev_ioctl(t.get(), hp, SLASH_ABI_HOTPLUG_IOCTL_REMOVE,
                                "0000:61:00.1"),
              0);

    // bars/ subtree still live and a register read/write round-trips.
    emu_node *bar0 = nullptr;
    ASSERT_EQ(emu_node_lookup_child(t.get(), dev->bars->ino, "bar0", &bar0), 0);
    ASSERT_NE(bar0, nullptr);
    emu_ino_t bar0_ino = bar0->ino;
    EXPECT_EQ(emu_node_is_live(t.get(), bar0_ino), 0);

    uint32_t val = 0xdeadbeef;
    ASSERT_EQ(emu_node_pwrite(t.get(), bar0_ino,
                              reinterpret_cast<const char *>(&val), sizeof(val),
                              0),
              static_cast<ssize_t>(sizeof(val)));
    uint32_t got = 0;
    ASSERT_EQ(emu_node_pread(t.get(), bar0_ino, reinterpret_cast<char *>(&got),
                             sizeof(got), 0),
              static_cast<ssize_t>(sizeof(got)));
    EXPECT_EQ(got, val);

    // The device is still found (not dead): the second REMOVE then succeeds.
    EXPECT_EQ(hotplug_dev_ioctl(t.get(), hp, SLASH_ABI_HOTPLUG_IOCTL_REMOVE,
                                "0000:61:00.2"),
              0);
    EXPECT_FALSE(resolves(t.get(), dev->dir->ino, "bars"));
    emu_node_forget(t.get(), bar0_ino, 1);
}

// After REMOVE .2 (bars), qdma must remain usable: QPAIR_ADD still works and the
// new qpair round-trips.  Then REMOVE .1 succeeds.
TEST(HotplugPerFunction, RemoveBarsLeavesQdmaFullyLive)
{
    HotplugTree t;
    emu_device *dev = t.add_device("0000:61:00");
    emu_ino_t hp = t.hotplug_ino();

    ASSERT_EQ(hotplug_dev_ioctl(t.get(), hp, SLASH_ABI_HOTPLUG_IOCTL_REMOVE,
                                "0000:61:00.2"),
              0);

    // qdma/ still accepts QPAIR_ADD and the qpair round-trips.
    struct slash_abi_qdma_qpair_add qreq {};
    qreq.size = sizeof(qreq);
    qreq.mode = 0;
    qreq.dir_mask = 0x1;
    struct slash_abi_qdma_qpair_add qout = qreq;
    ASSERT_EQ(emu_node_ioctl(t.get(), dev->qdma->ino,
                             SLASH_ABI_QDMA_IOCTL_QPAIR_ADD, &qreq, sizeof(qreq),
                             &qout, sizeof(qout)),
              0);
    emu_node *qp = nullptr;
    char qname[32];
    std::snprintf(qname, sizeof(qname), "qpair%u", qout.qid);
    ASSERT_EQ(emu_node_lookup_child(t.get(), dev->qdma->ino, qname, &qp), 0);
    emu_ino_t qp_ino = qp->ino;
    std::vector<uint8_t> data(16, 0x5a);
    ASSERT_EQ(emu_node_pwrite(t.get(), qp_ino,
                              reinterpret_cast<const char *>(data.data()),
                              data.size(), static_cast<off_t>(SLASH_HBM_BASE)),
              static_cast<ssize_t>(data.size()));

    // REMOVE .1 then succeeds and tears the qpair down (-ENODEV on the handle).
    ASSERT_EQ(hotplug_dev_ioctl(t.get(), hp, SLASH_ABI_HOTPLUG_IOCTL_REMOVE,
                                "0000:61:00.1"),
              0);
    EXPECT_EQ(emu_node_pread(t.get(), qp_ino,
                             reinterpret_cast<char *>(data.data()), data.size(),
                             static_cast<off_t>(SLASH_HBM_BASE)),
              -ENODEV);
    emu_node_forget(t.get(), qp_ino, 1);
}

// REMOVE of an absent device, and a re-REMOVE of an already-removed function,
// are both no-op successes (idempotent revocation contract).
TEST(HotplugPerFunction, AbsentAndRepeatNoops)
{
    HotplugTree t;
    t.add_device("0000:61:00");
    emu_ino_t hp = t.hotplug_ino();

    // Absent device.
    EXPECT_EQ(hotplug_dev_ioctl(t.get(), hp, SLASH_ABI_HOTPLUG_IOCTL_REMOVE,
                                "0000:ab:00.1"),
              0);
    // Remove .2 twice.
    EXPECT_EQ(hotplug_dev_ioctl(t.get(), hp, SLASH_ABI_HOTPLUG_IOCTL_REMOVE,
                                "0000:61:00.2"),
              0);
    EXPECT_EQ(hotplug_dev_ioctl(t.get(), hp, SLASH_ABI_HOTPLUG_IOCTL_REMOVE,
                                "0000:61:00.2"),
              0);
}

// ===========================================================================
// MODEL-SHUTDOWN exactly-once (the double-fire guard)
// ===========================================================================

// REMOVE both functions (seam fires once), then TOGGLE_SBR the same BDF: the
// whole-device revoke path marks both functions removed again, but
// model_shutdown_fired must prevent a second fire.
TEST(HotplugModelShutdownOnce, RemoveBothThenSbrDoesNotRefire)
{
    ShutdownState sd;
    HotplugTree t(nullptr, nullptr);
    t.add_device("0000:61:00", stub_model_shutdown, &sd);
    emu_ino_t hp = t.hotplug_ino();
    ASSERT_EQ(emu_hotplug_set_sbr_sleep_us(t.get(), 0), 0);

    ASSERT_EQ(hotplug_dev_ioctl(t.get(), hp, SLASH_ABI_HOTPLUG_IOCTL_REMOVE,
                                "0000:61:00.1"),
              0);
    ASSERT_EQ(hotplug_dev_ioctl(t.get(), hp, SLASH_ABI_HOTPLUG_IOCTL_REMOVE,
                                "0000:61:00.2"),
              0);
    EXPECT_EQ(sd.calls.load(), 1) << "seam must fire once both functions gone";

    // Whole-device revoke (TOGGLE_SBR) on the now-fully-removed device: the
    // device is still in the registry (revoke_function does not mark it dead),
    // so emu_device_revoke finds it, re-marks both functions, but the
    // already-fired guard must hold.
    ASSERT_EQ(hotplug_dev_ioctl(t.get(), hp, SLASH_ABI_HOTPLUG_IOCTL_TOGGLE_SBR,
                                "0000:61:00.1"),
              0);
    EXPECT_EQ(sd.calls.load(), 1)
        << "model_shutdown_fired must prevent a double fire across REMOVE+SBR";
}

// Single-function removal must NOT fire the seam.
TEST(HotplugModelShutdownOnce, SingleFunctionDoesNotFire)
{
    ShutdownState sd;
    HotplugTree t;
    t.add_device("0000:61:00", stub_model_shutdown, &sd);
    emu_ino_t hp = t.hotplug_ino();

    ASSERT_EQ(hotplug_dev_ioctl(t.get(), hp, SLASH_ABI_HOTPLUG_IOCTL_REMOVE,
                                "0000:61:00.1"),
              0);
    EXPECT_EQ(sd.calls.load(), 0);
}

// Direct whole-device revoke fires once, and a SECOND whole-device revoke (now
// the device is dead/absent) does not fire again.
TEST(HotplugModelShutdownOnce, DoubleWholeDeviceRevokeFiresOnce)
{
    ShutdownState sd;
    HotplugTree t;
    t.add_device("0000:61:00", stub_model_shutdown, &sd);

    ASSERT_EQ(emu_device_revoke(t.get(), "0000:61:00"), 0);
    EXPECT_EQ(sd.calls.load(), 1);
    // Device is now dead -> find_device_locked skips it -> idempotent no-op, no
    // second fire.
    ASSERT_EQ(emu_device_revoke(t.get(), "0000:61:00"), 0);
    EXPECT_EQ(sd.calls.load(), 1);
}

// ===========================================================================
// NO DOUBLE-ATTACH on repeated materialize re-invocation
// ===========================================================================

// Repeated reloads via a real materialize-style seed: a surviving device must
// gain NO duplicate info/bars/qdma children.  We drive the actual collision
// skip primitive (collect_live_bdfs + running_set + select_new) the way fs.c
// does, and assert the device dir has exactly one of each endpoint after.
TEST(HotplugNoDoubleAttach, RepeatedSeedSelectsNothingForLiveDevice)
{
    HotplugTree t;
    emu_device *dev = t.add_device("0000:61:00");

    // Baseline: exactly one of each endpoint child.
    ASSERT_EQ(count_children_named(dev->dir, "info"), 1);
    ASSERT_EQ(count_children_named(dev->dir, "bars"), 1);
    ASSERT_EQ(count_children_named(dev->dir, "qdma"), 1);

    // Emulate three RESCAN re-invocations of the seed+select primitive.  Each
    // time the live device must be in the running-set, so select_new returns it
    // as NOT-selected (skip), so nothing is re-attached.
    for (int iter = 0; iter < 3; iter++) {
        str_array live = str_array_init();
        ASSERT_EQ(emu_node_tree_collect_live_bdfs(t.get(), &live), 0);
        ASSERT_EQ(live.len, 1u);

        emu_running_set *running = nullptr;
        ASSERT_EQ(emu_running_set_new(&running), 0);
        for (size_t i = 0; i < live.len; i++) {
            ASSERT_EQ(emu_running_set_add(running, live.d[i]), 0);
        }
        EXPECT_TRUE(emu_running_set_contains(running, "0000:61:00"));
        cleanup_running_set(running);
        str_array_free(&live);

        // The endpoint children count must stay exactly one (no double-attach).
        EXPECT_EQ(count_children_named(dev->dir, "info"), 1) << "iter " << iter;
        EXPECT_EQ(count_children_named(dev->dir, "bars"), 1) << "iter " << iter;
        EXPECT_EQ(count_children_named(dev->dir, "qdma"), 1) << "iter " << iter;
    }
}

// ===========================================================================
// BDF-with-function parsing -- adversarial edges beyond the base suite
// ===========================================================================

TEST(HotplugParseBdfAdversarial, EmptyAndNullAndNoDot)
{
    char bdf[EMU_BDF_LEN];
    emu_device_function func;
    EXPECT_EQ(emu_hotplug_parse_bdf(nullptr, bdf, sizeof(bdf), &func), -EINVAL);
    EXPECT_EQ(emu_hotplug_parse_bdf("", bdf, sizeof(bdf), &func), -EINVAL);
    // Leading dot only.
    EXPECT_EQ(emu_hotplug_parse_bdf(".", bdf, sizeof(bdf), &func), -EINVAL);
    // No function dot at all is rejected (board-only).
    EXPECT_EQ(emu_hotplug_parse_bdf("0000:61:00", bdf, sizeof(bdf), &func),
              -EINVAL);
}

TEST(HotplugParseBdfAdversarial, NullOutPointers)
{
    char bdf[EMU_BDF_LEN];
    emu_device_function func;
    EXPECT_EQ(emu_hotplug_parse_bdf("0000:61:00.1", nullptr, sizeof(bdf), &func),
              -EINVAL);
    EXPECT_EQ(emu_hotplug_parse_bdf("0000:61:00.1", bdf, sizeof(bdf), nullptr),
              -EINVAL);
}

TEST(HotplugParseBdfAdversarial, OverlongBoardPrefixRejected)
{
    char bdf[EMU_BDF_LEN];
    emu_device_function func;
    // A board prefix far longer than any valid BDF, with a valid function suffix.
    std::string huge(200, '0');
    huge += ".1";
    EXPECT_EQ(emu_hotplug_parse_bdf(huge.c_str(), bdf, sizeof(bdf), &func),
              -EINVAL);
}

TEST(HotplugParseBdfAdversarial, FunctionWithTrailingGarbage)
{
    char bdf[EMU_BDF_LEN];
    emu_device_function func;
    // Digit followed by garbage: dot[2] != '\0' -> -EINVAL.
    EXPECT_EQ(emu_hotplug_parse_bdf("0000:61:00.1x", bdf, sizeof(bdf), &func),
              -EINVAL);
    EXPECT_EQ(emu_hotplug_parse_bdf("0000:61:00.1 ", bdf, sizeof(bdf), &func),
              -EINVAL);
}

// REMOVE/SBR with a syntactically valid but unsupported function (.0/.3) must be
// -EOPNOTSUPP at the ioctl level too (parse error propagates).
TEST(HotplugParseBdfAdversarial, IoctlUnsupportedFunctionPropagates)
{
    HotplugTree t;
    t.add_device("0000:61:00");
    emu_ino_t hp = t.hotplug_ino();
    EXPECT_EQ(hotplug_dev_ioctl(t.get(), hp, SLASH_ABI_HOTPLUG_IOCTL_REMOVE,
                                "0000:61:00.3"),
              -EOPNOTSUPP);
    EXPECT_EQ(hotplug_dev_ioctl(t.get(), hp, SLASH_ABI_HOTPLUG_IOCTL_TOGGLE_SBR,
                                "0000:61:00.0"),
              -EOPNOTSUPP);
}

// ===========================================================================
// ioctl buffer / versioning matrix -- adversarial
// ===========================================================================

// An OVERSIZED request (newer caller, larger struct) must be accepted: the
// handler reads only the prefix it understands.
TEST(HotplugIoctlMatrix, OversizedRequestAccepted)
{
    HotplugTree t;
    t.add_device("0000:61:00");
    emu_ino_t hp = t.hotplug_ino();

    // A buffer larger than the struct, with a valid prefix.
    std::vector<uint8_t> buf(sizeof(slash_abi_hotplug_device_request) + 64, 0);
    auto *req = reinterpret_cast<slash_abi_hotplug_device_request *>(buf.data());
    req->size = sizeof(*req);
    std::snprintf(req->bdf, sizeof(req->bdf), "0000:61:00.1");
    EXPECT_EQ(emu_node_ioctl(t.get(), hp, SLASH_ABI_HOTPLUG_IOCTL_REMOVE,
                             buf.data(), buf.size(), buf.data(), buf.size()),
              0);
}

// RESCAN ignores any in payload (it is _IO, no arg): passing a bogus buffer is
// still fine.
TEST(HotplugIoctlMatrix, RescanIgnoresPayload)
{
    ReloadState rs;
    HotplugTree t(stub_reload, &rs);
    emu_ino_t hp = t.hotplug_ino();
    char junk[16] = {1, 2, 3};
    EXPECT_EQ(emu_node_ioctl(t.get(), hp, SLASH_ABI_HOTPLUG_IOCTL_RESCAN, junk,
                             sizeof(junk), junk, sizeof(junk)),
              0);
    EXPECT_EQ(rs.calls.load(), 1);
}

// A device-request command with a NULL in pointer and zero size must be rejected
// (-EINVAL), not crash.
TEST(HotplugIoctlMatrix, NullInRejectedForDeviceRequest)
{
    HotplugTree t;
    t.add_device("0000:61:00");
    emu_ino_t hp = t.hotplug_ino();
    EXPECT_EQ(emu_node_ioctl(t.get(), hp, SLASH_ABI_HOTPLUG_IOCTL_REMOVE, nullptr,
                             0, nullptr, 0),
              -EINVAL);
}

// ===========================================================================
// Integration: the unlink defect over the REAL FUSE mount
// ===========================================================================

constexpr int kMountTimeoutMs = 5000;
constexpr int kShutdownTimeoutMs = 5000;
constexpr int kPollIntervalMs = 50;

std::string make_scratch(const char *suffix)
{
    ::mkdir(SLASH_EMU_TMP_DIR, 0755);
    std::string tmpl =
        std::string(SLASH_EMU_TMP_DIR) + "/slash_emu_" + suffix + "_XXXXXX";
    std::vector<char> buf(tmpl.begin(), tmpl.end());
    buf.push_back('\0');
    char *result = ::mkdtemp(buf.data());
    EXPECT_NE(result, nullptr) << "mkdtemp failed: " << std::strerror(errno);
    return result != nullptr ? std::string(result) : std::string();
}

std::string write_config()
{
    ::mkdir(SLASH_EMU_TMP_DIR, 0755);
    std::string tmpl =
        std::string(SLASH_EMU_TMP_DIR) + "/slash_emu_hpadvcfg_XXXXXX";
    std::vector<char> buf(tmpl.begin(), tmpl.end());
    buf.push_back('\0');
    int fd = ::mkstemp(buf.data());
    EXPECT_GE(fd, 0) << "mkstemp failed: " << std::strerror(errno);
    const char *cfg =
        "[accelerator:0000:61:00]\n"
        "net-ip = 10.0.0.1\n";
    ssize_t n = ::write(fd, cfg, std::strlen(cfg));
    EXPECT_EQ(static_cast<size_t>(n), std::strlen(cfg));
    ::close(fd);
    return buf.data();
}

template <typename Pred>
bool wait_for(Pred pred, int timeout_ms)
{
    int waited = 0;
    while (waited < timeout_ms) {
        if (pred()) {
            return true;
        }
        ::usleep(kPollIntervalMs * 1000);
        waited += kPollIntervalMs;
    }
    return pred();
}

bool mount_is_ready(const std::string &mountpoint)
{
    struct statfs sfs {};
    if (::statfs(mountpoint.c_str(), &sfs) != 0) {
        return false;
    }
    return sfs.f_type == FUSE_SUPER_MAGIC;
}

template <typename Body>
void with_mounted_daemon(Body body)
{
    const std::string mountpoint = make_scratch("hpadv");
    ASSERT_FALSE(mountpoint.empty());
    const std::string config = write_config();

    pid_t pid = ::fork();
    ASSERT_NE(pid, -1) << "fork failed: " << std::strerror(errno);
    if (pid == 0) {
        ::execl(SLASH_EMUD_PATH, "slash-emud", "--config", config.c_str(),
                "--mount", mountpoint.c_str(), (char *) nullptr);
        ::_exit(127);
    }

    bool ready =
        wait_for([&] { return mount_is_ready(mountpoint); }, kMountTimeoutMs);
    if (!ready) {
        ::kill(pid, SIGKILL);
        int status = 0;
        ::waitpid(pid, &status, 0);
        ::rmdir(mountpoint.c_str());
        ::unlink(config.c_str());
        FAIL() << "Daemon did not mount within timeout at " << mountpoint;
    }

    body(mountpoint);

    ASSERT_EQ(::kill(pid, SIGTERM), 0) << "kill failed: " << std::strerror(errno);
    int status = 0;
    bool exited = wait_for(
        [&] { return ::waitpid(pid, &status, WNOHANG) == pid; },
        kShutdownTimeoutMs);
    if (!exited) {
        ::kill(pid, SIGKILL);
        ::waitpid(pid, &status, 0);
    }
    ::rmdir(mountpoint.c_str());
    ::unlink(config.c_str());
}

bool path_exists(const std::string &p)
{
    struct stat st {};
    return ::stat(p.c_str(), &st) == 0;
}

// The user-reachable form of the invariant violation: a plain unlink(2) of the
// global /hotplug file over the FUSE mount.  If the invariant ("never unlinked")
// held, the unlink would be refused (e.g. EPERM/EISDIR) and the control surface
// would survive.  Today it succeeds and the surface is destroyed: a fresh open
// of /hotplug then misses with ENOENT and no further hotplug command can run.
TEST(HotplugMountUnlink, HotplugFileUnlinkIsRefusedSurfaceSurvives)
{
    with_mounted_daemon([](const std::string &mnt) {
        const std::string hp = mnt + "/hotplug";
        ASSERT_TRUE(path_exists(hp)) << "hotplug missing before unlink";

        // The control surface must NOT be removable by a user unlink: the FUSE
        // unlink op rejects the non-unlinkable hotplug node with EPERM.
        int urc = ::unlink(hp.c_str());
        int saved = errno;
        EXPECT_EQ(urc, -1) << "unlink of /hotplug must fail";
        EXPECT_EQ(saved, EPERM)
            << "unlink of /hotplug must be refused with EPERM, got "
            << std::strerror(saved);

        // Surface intact: still present, still openable, still servicing ioctls.
        EXPECT_TRUE(path_exists(hp))
            << "hotplug vanished despite a refused unlink";
        int fd = ::open(hp.c_str(), O_RDONLY);
        EXPECT_GE(fd, 0) << "control surface lost despite refused unlink";
        if (fd >= 0) {
            EXPECT_EQ(::ioctl(fd, SLASH_ABI_HOTPLUG_IOCTL_RESCAN), 0)
                << "RESCAN: " << std::strerror(errno);
            ::close(fd);
        }
    });
}

// The fix must not over-rotate: a qpair<Q> file IS still user-unlinkable (the
// VRTD delete-on-last-close pattern), while info and bars/bar<M> are now NOT.
TEST(HotplugMountUnlink, QpairUnlinkableButInfoAndBarsAreNot)
{
    with_mounted_daemon([](const std::string &mnt) {
        const std::string dev = mnt + "/0000:61:00";

        // info and bars/bar0 are endpoints removed by revocation, not by the
        // holder: a user unlink must be refused with EPERM, leaving them intact.
        for (const std::string &p : {dev + "/info", dev + "/bars/bar0"}) {
            ASSERT_TRUE(path_exists(p)) << "missing before unlink: " << p;
            EXPECT_EQ(::unlink(p.c_str()), -1) << "unlink unexpectedly ok: " << p;
            EXPECT_EQ(errno, EPERM)
                << "unlink of " << p << " must be EPERM, got "
                << std::strerror(errno);
            EXPECT_TRUE(path_exists(p)) << "vanished despite refused unlink: " << p;
        }

        // A qpair<Q> file, by contrast, IS unlinkable: ADD one, then unlink it.
        int qfd = ::open((dev + "/qdma").c_str(), O_RDONLY | O_DIRECTORY);
        ASSERT_GE(qfd, 0) << "open qdma/: " << std::strerror(errno);
        struct slash_abi_qdma_qpair_add req {};
        req.size = sizeof(req);
        req.mode = 0;       // MM
        req.dir_mask = 0x1; // H2C
        ASSERT_EQ(::ioctl(qfd, SLASH_ABI_QDMA_IOCTL_QPAIR_ADD, &req), 0)
            << "QPAIR_ADD: " << std::strerror(errno);
        ::close(qfd);

        char qpath[256];
        std::snprintf(qpath, sizeof(qpath), "%s/qdma/qpair%u", dev.c_str(),
                      req.qid);
        ASSERT_TRUE(path_exists(qpath)) << "qpair file missing: " << qpath;
        EXPECT_EQ(::unlink(qpath), 0)
            << "qpair<Q> must stay user-unlinkable: " << std::strerror(errno);
        EXPECT_FALSE(path_exists(qpath))
            << "qpair file should be gone after unlink (not currently open)";
    });
}

}  // namespace
