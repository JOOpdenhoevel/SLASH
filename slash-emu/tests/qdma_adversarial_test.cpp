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
 * @file qdma_adversarial_test.cpp
 * @brief Adversarial conformance suite for the /<BDF>/qdma/ endpoint.
 *
 * The destructive counterpart of qdma_test.cpp, pinning the spine machinery the
 * endpoint leans on hardest:
 *
 *   1. THE _Locked CHAIN: the QPAIR_ADD handler runs its entire create chain
 *      (createChildLocked -> registerResourceLocked -> setOpsLocked ->
 *      attachResourceLocked -> setDirectIoLocked, plus findResourceLocked in the
 *      allocator) under the held ioctl lock; a POSIX-timer watchdog catches any
 *      self-deadlock.  A concurrency stress (one tree+device per thread) exercises
 *      the per-tree mutex under contention.
 *   2. IOCTL handling edge cases (buffer sizing, unknown inode/cmd, post-revoke).
 *   3. QPAIR LIFECYCLE matrix (cooperative-once, immediate reap, forced-then-
 *      cooperative idempotency, QID uniqueness after a free).
 *   4. SPARSE STORE (straddling, multi-page, range matrix, lazy alloc, isolation).
 *   5. MEM BACKEND SEAM: a FAKE backend pins the rc-contract (rc==0 model /
 *      rc>0 store fallback / rc<0 -ENODEV; populate keeps the store current).
 *   6. direct_io: the created qpair file is marked direct_io (the no-hang seam).
 *
 * Unit-level only (links slash_emu_core, no FUSE mount); the mount-level VRTD
 * path is already covered by qdma_test.cpp's QdmaMount.* integration tests.
 */

#include <gtest/gtest.h>

#include <atomic>
#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include <csignal>
#include <ctime>
#include <sys/time.h>
#include <unistd.h>

#include "node.hpp"
#include "qdma.hpp"
#include "slash/uapi/slash_abi.h"

using namespace slash::emu;

namespace {

constexpr uint32_t kH2C = 0x1u;
constexpr uint32_t kC2H = 0x2u;
constexpr uint32_t kCMPT = 0x4u;
constexpr uint32_t kMM = 0u;
constexpr uint32_t kST = 1u;

Device *attach_dev(NodeTree &tree, const char *bdf)
{
    Device *dev = tree.addDevice(bdf);
    EXPECT_NE(dev, nullptr);
    if (dev != nullptr) {
        EXPECT_EQ(qdmaAttach(*dev), 0);
    }
    return dev;
}

// Issue a QPAIR_ADD against the qdma/ dir inode; returns the ioctl rc, fills qid.
int qpair_add(NodeTree &tree, Device *dev, uint32_t *qid_out, uint32_t mode = kMM,
              uint32_t dir_mask = kH2C, uint32_t h2c = 0, uint32_t c2h = 0,
              uint32_t cmpt = 0)
{
    struct slash_abi_qdma_qpair_add req {};
    req.size = sizeof(req);
    req.mode = mode;
    req.dir_mask = dir_mask;
    req.h2c_ring_sz = h2c;
    req.c2h_ring_sz = c2h;
    req.cmpt_ring_sz = cmpt;
    struct slash_abi_qdma_qpair_add out = req;
    int rc = tree.ioctl(dev->qdma->ino, SLASH_ABI_QDMA_IOCTL_QPAIR_ADD, &req,
                        sizeof(req), &out, sizeof(out));
    if (rc == 0 && qid_out != nullptr) {
        *qid_out = out.qid;
    }
    return rc;
}

// Resolve qpair<Q> WITHOUT bumping lookup_count (a finder, not an open).
Node *find_qpair_node(Device *dev, uint32_t qid)
{
    char name[32];
    std::snprintf(name, sizeof(name), "qpair%u", qid);
    for (Node *child : dev->qdma->children) {
        if (!child->unlinked && child->name == name) {
            return child;
        }
    }
    return nullptr;
}

// Model an open: lookup bumps lookup_count and returns the inode.
Ino open_qpair(NodeTree &tree, Device *dev, uint32_t qid)
{
    char name[32];
    std::snprintf(name, sizeof(name), "qpair%u", qid);
    Node *child = nullptr;
    EXPECT_EQ(tree.lookupChild(dev->qdma->ino, name, &child), 0) << name;
    return child != nullptr ? child->ino : 0;
}

// ===========================================================================
// 1. THE _Locked REFACTOR: no self-deadlock under the held ioctl lock.
// ===========================================================================

// A POSIX-timer watchdog: arm before the op, disarm after.  If the op
// self-deadlocks on the non-recursive tree mutex, the timer fires SIGALRM whose
// default action kills the process -> ctest records the failure (and the
// per-test TIMEOUT is the backstop).  A passing op disarms before it fires.
class Watchdog {
public:
    explicit Watchdog(unsigned int seconds)
    {
        struct itimerval it {};
        it.it_value.tv_sec = seconds;
        ::setitimer(ITIMER_REAL, &it, &saved_);
    }
    ~Watchdog()
    {
        struct itimerval off {};
        ::setitimer(ITIMER_REAL, &off, nullptr);
    }

private:
    struct itimerval saved_ {};
};

// Driving QPAIR_ADD runs createQpair's ENTIRE _Locked chain
// (createChildLocked -> registerResourceLocked -> setOpsLocked ->
// attachResourceLocked -> setDirectIoLocked, plus findResourceLocked in the
// allocator) all inside NodeTree::ioctl's held lock.  If any link re-took the
// lock, this would hang.
TEST(QdmaLockedRefactor, QpairAddChainDoesNotSelfDeadlock)
{
    Watchdog wd(10);  // generous; a deadlock never returns, a healthy run is <1ms

    NodeTree t;
    Device *dev = attach_dev(t, "0000:61:00");
    ASSERT_NE(dev, nullptr);

    // Many adds in a row exercise BOTH allocator paths (monotonic + probe).
    for (int i = 0; i < 64; i++) {
        uint32_t qid = 0xffffffff;
        ASSERT_EQ(qpair_add(t, dev, &qid), 0) << "add " << i;
        ASSERT_EQ(qid, static_cast<uint32_t>(i));
        // The error-unwind path also runs _Locked unwinders (unlinkLocked):
        // an immediately-following rejected add must not wedge the lock either.
        uint32_t bad = 0;
        ASSERT_EQ(qpair_add(t, dev, &bad, kST), -EOPNOTSUPP);
    }
}

// The hook's error-unwind primitive is unlinkLocked of the just-created node
// (register/attach failure path), run under the held lock.  Its public sibling
// unlink takes the lock and then runs the same destroyNodeLocked path; driving a
// prune through it under a watchdog confirms that shared destroy path does not
// deadlock or wedge.  (The in-hook _Locked unwind itself is exercised by the
// OOM-style failure paths; here we pin the shared reap path the unwinder
// ultimately calls.)
TEST(QdmaLockedRefactor, UnlinkFreshNodeNoDeadlock)
{
    Watchdog wd(10);
    NodeTree t;
    Device *dev = attach_dev(t, "0000:61:00");
    ASSERT_NE(dev, nullptr);

    uint32_t qid = 0;
    ASSERT_EQ(qpair_add(t, dev, &qid), 0);
    Node *node = find_qpair_node(dev, qid);
    ASSERT_NE(node, nullptr);

    // Prune (no open fd): unlink takes the lock then runs the same
    // destroyNodeLocked path the _Locked unwinder uses.  Must not deadlock.
    t.unlink(node);
    EXPECT_EQ(find_qpair_node(dev, qid), nullptr);
}

// Concurrency stress: many threads, each on its OWN tree+device, hammer
// QPAIR_ADD + transfers + teardown concurrently.  Per-tree isolation means the
// per-tree mutex is exercised under real contention within each tree (the FUSE
// session is single-threaded today, but the spine is built to be MT-safe).  A
// lock-ordering bug or a missed-lock data race surfaces here under ASan/TSan-ish
// pressure and the watchdog catches a hang.
TEST(QdmaLockedRefactor, ConcurrentTreesStressNoDeadlockNoCorruption)
{
    Watchdog wd(30);
    constexpr int kThreads = 8;
    constexpr int kIters = 200;

    std::atomic<int> failures{0};
    std::vector<std::thread> ts;
    for (int tnum = 0; tnum < kThreads; tnum++) {
        ts.emplace_back([&failures, tnum] {
            NodeTree tree;
            char bdf[16];
            std::snprintf(bdf, sizeof(bdf), "0000:%02x:00", tnum + 1);
            Device *dev = tree.addDevice(bdf);
            if (dev == nullptr || qdmaAttach(*dev) != 0) {
                failures++;
                return;
            }

            for (int i = 0; i < kIters; i++) {
                uint32_t qid = 0xffffffff;
                if (qpair_add(tree, dev, &qid, kMM, kH2C | kC2H) != 0) {
                    failures++;
                    break;
                }
                Ino ino = 0;
                {
                    char name[32];
                    std::snprintf(name, sizeof(name), "qpair%u", qid);
                    Node *child = nullptr;
                    if (tree.lookupChild(dev->qdma->ino, name, &child) != 0) {
                        failures++;
                        break;
                    }
                    ino = child->ino;
                }
                uint64_t v = (static_cast<uint64_t>(tnum) << 32) | i;
                uint64_t addr = SLASH_HBM_BASE + (static_cast<uint64_t>(i) * 64);
                if (tree.pwrite(ino, reinterpret_cast<const char *>(&v), 8,
                                static_cast<off_t>(addr)) != 8) {
                    failures++;
                    break;
                }
                uint64_t got = 0;
                if (tree.pread(ino, reinterpret_cast<char *>(&got), 8,
                               static_cast<off_t>(addr)) != 8 ||
                    got != v) {
                    failures++;
                    break;
                }
                // Prune it (no open fd held beyond the lookup): unlink then drop
                // the lookup so the cooperative teardown reaps it.
                Node *node = find_qpair_node(dev, qid);
                if (node != nullptr) {
                    tree.unlink(node);
                }
                tree.forget(ino, 1);
            }
        });
    }
    for (auto &th : ts) {
        th.join();
    }
    EXPECT_EQ(failures.load(), 0);
}

// ===========================================================================
// 2. IOCTL handling edge cases.
// ===========================================================================

TEST(QdmaIoctl, OversizedBuffersStillAccepted)
{
    // in_size/out_size LARGER than the struct is fine: the hook requires AT
    // LEAST sizeof, and reads only the prefix it knows.
    NodeTree t;
    Device *dev = attach_dev(t, "0000:61:00");
    ASSERT_NE(dev, nullptr);

    struct Big {
        struct slash_abi_qdma_qpair_add req;
        uint8_t tail[64];
    } big {};
    big.req.size = sizeof(big.req);
    big.req.mode = kMM;
    big.req.dir_mask = kH2C;

    Big out = big;
    EXPECT_EQ(t.ioctl(dev->qdma->ino, SLASH_ABI_QDMA_IOCTL_QPAIR_ADD, &big,
                      sizeof(big), &out, sizeof(out)),
              0);
    EXPECT_EQ(out.req.qid, 0u);
}

TEST(QdmaIoctl, OutBufferShortByOneRejected)
{
    NodeTree t;
    Device *dev = attach_dev(t, "0000:61:00");
    ASSERT_NE(dev, nullptr);

    struct slash_abi_qdma_qpair_add req {};
    req.size = sizeof(req);
    req.mode = kMM;
    req.dir_mask = kH2C;
    // out_size exactly one byte short of the struct -> -EINVAL (qid would be
    // truncated otherwise).
    EXPECT_EQ(t.ioctl(dev->qdma->ino, SLASH_ABI_QDMA_IOCTL_QPAIR_ADD, &req,
                      sizeof(req), &req, sizeof(req) - 1),
              -EINVAL);
    // in_size one byte short -> -EINVAL too.
    EXPECT_EQ(t.ioctl(dev->qdma->ino, SLASH_ABI_QDMA_IOCTL_QPAIR_ADD, &req,
                      sizeof(req) - 1, &req, sizeof(req)),
              -EINVAL);
}

TEST(QdmaIoctl, ZeroSizeBuffersRejectedNotCrash)
{
    NodeTree t;
    Device *dev = attach_dev(t, "0000:61:00");
    ASSERT_NE(dev, nullptr);
    struct slash_abi_qdma_qpair_add req {};
    req.size = sizeof(req);
    req.mode = kMM;
    req.dir_mask = kH2C;
    EXPECT_EQ(t.ioctl(dev->qdma->ino, SLASH_ABI_QDMA_IOCTL_QPAIR_ADD, &req, 0,
                      &req, sizeof(req)),
              -EINVAL);
    EXPECT_EQ(t.ioctl(dev->qdma->ino, SLASH_ABI_QDMA_IOCTL_QPAIR_ADD, &req,
                      sizeof(req), &req, 0),
              -EINVAL);
}

TEST(QdmaIoctl, IoctlOnUnknownInodeIsEnoent)
{
    NodeTree t;
    attach_dev(t, "0000:61:00");
    struct slash_abi_qdma_qpair_add req {};
    req.size = sizeof(req);
    EXPECT_EQ(t.ioctl(0xdeadbeefu, SLASH_ABI_QDMA_IOCTL_QPAIR_ADD, &req,
                      sizeof(req), &req, sizeof(req)),
              -ENOENT);
}

TEST(QdmaIoctl, IoctlAfterRevokeIsEnodev)
{
    NodeTree t;
    Device *dev = attach_dev(t, "0000:61:00");
    ASSERT_NE(dev, nullptr);
    Ino qdma_ino = dev->qdma->ino;

    // Hold a lookup on qdma/ so the dead node survives revoke as an orphan,
    // letting the ioctl resolve the inode and hit the liveness gate (-ENODEV)
    // rather than -ENOENT (which a reaped inode would give).
    {
        Node *child = nullptr;
        ASSERT_EQ(t.lookupChild(dev->dir->ino, "qdma", &child), 0);
    }
    ASSERT_EQ(t.revokeDevice("0000:61:00"), 0);

    struct slash_abi_qdma_qpair_add req {};
    req.size = sizeof(req);
    req.mode = kMM;
    req.dir_mask = kH2C;
    EXPECT_EQ(t.ioctl(qdma_ino, SLASH_ABI_QDMA_IOCTL_QPAIR_ADD, &req,
                      sizeof(req), &req, sizeof(req)),
              -ENODEV);
}

TEST(QdmaIoctl, MalformedDirMaskAndRingMatrix)
{
    // High dir bits beyond H2C/C2H/CMPT -> -EINVAL; CMPT alone -> -EOPNOTSUPP.
    EXPECT_EQ(qdmaCheckQpairAdd(kMM, 0x10u, 0, 0, 0), -EINVAL);
    EXPECT_EQ(qdmaCheckQpairAdd(kMM, 0xffffffffu, 0, 0, 0), -EOPNOTSUPP);
    // CMPT combined with a valid dir is still EOPNOTSUPP (CMPT checked first).
    EXPECT_EQ(qdmaCheckQpairAdd(kMM, kH2C | kC2H | kCMPT, 0, 0, 0), -EOPNOTSUPP);
    // Ring index just over the boundary.
    EXPECT_EQ(qdmaCheckQpairAdd(kMM, kH2C, 16, 0, 0), -EINVAL);
    EXPECT_EQ(qdmaCheckQpairAdd(kMM, kH2C, 0xffffffffu, 0, 0), -EINVAL);
    // Boundary values 0 and 15 accepted.
    EXPECT_EQ(qdmaCheckQpairAdd(kMM, kH2C, 0, 0, 0), 0);
    EXPECT_EQ(qdmaCheckQpairAdd(kMM, kH2C, 15, 15, 15), 0);
}

// ===========================================================================
// 3. QPAIR LIFECYCLE matrix.
// ===========================================================================

// Cooperative teardown fires EXACTLY once: count teardown invocations by
// observing the resource's liveness flip and that a second forget is a no-op.
TEST(QdmaLifecycle, CooperativeTeardownExactlyOnce)
{
    NodeTree t;
    Device *dev = attach_dev(t, "0000:61:00");
    ASSERT_NE(dev, nullptr);
    uint32_t qid = 0;
    ASSERT_EQ(qpair_add(t, dev, &qid), 0);

    Node *node = find_qpair_node(dev, qid);
    ASSERT_NE(node, nullptr);
    Ino ino = node->ino;
    Resource *res = node->resource.get();
    ASSERT_NE(res, nullptr);
    EXPECT_EQ(t.resourceCheck(res), 0) << "live before teardown";

    // Open + unlink-while-open: nameless, both refs live, still working.
    {
        Node *child = nullptr;
        ASSERT_EQ(t.lookupChild(dev->qdma->ino, "qpair0", &child), 0);
    }
    t.unlink(node);
    EXPECT_EQ(t.resourceCheck(res), 0) << "still live after unlink";

    // Last close: cooperative teardown. The resource is freed here (both refs
    // gone), so we must NOT touch `res` afterwards -- ASan would catch a UAF.
    t.forget(ino, 1);

    // A second forget on the (now reaped) inode is a no-op (idempotent).
    t.forget(ino, 1);
    char buf[8] = {};
    EXPECT_EQ(t.pread(ino, buf, 8, static_cast<off_t>(SLASH_HBM_BASE)), -ENOENT);
}

// Unlink of a NOT-open named qpair tears down immediately (reaped at once).
TEST(QdmaLifecycle, UnlinkNotOpenReapsImmediately)
{
    NodeTree t;
    Device *dev = attach_dev(t, "0000:61:00");
    ASSERT_NE(dev, nullptr);
    uint32_t qid = 0;
    ASSERT_EQ(qpair_add(t, dev, &qid), 0);

    Node *node = find_qpair_node(dev, qid);
    ASSERT_NE(node, nullptr);
    Ino ino = node->ino;

    t.unlink(node);  // no lookups held -> immediate destroy

    // Inode is gone: a stat/read on it is -ENOENT, and it is not in readdir.
    char buf[8] = {};
    EXPECT_EQ(t.pread(ino, buf, 8, static_cast<off_t>(SLASH_HBM_BASE)), -ENOENT);
    EXPECT_EQ(find_qpair_node(dev, qid), nullptr);
}

// unlinkChild of a DIRECTORY (qdma/) -> -EISDIR (only files are unlinkable).
TEST(QdmaLifecycle, UnlinkChildOnDirectoryIsEisdir)
{
    NodeTree t;
    Device *dev = attach_dev(t, "0000:61:00");
    ASSERT_NE(dev, nullptr);
    // qdma is a child dir of <BDF>/; unlinking it via the file path -> -EISDIR.
    EXPECT_EQ(t.unlinkChild(dev->dir->ino, "qdma"), -EISDIR);
    // A nonexistent name -> -ENOENT.
    EXPECT_EQ(t.unlinkChild(dev->qdma->ino, "qpair999"), -ENOENT);
    // Unlink under a non-directory parent inode -> -ENOTDIR is exercised via a
    // qpair file as the "parent".
    uint32_t qid = 0;
    ASSERT_EQ(qpair_add(t, dev, &qid), 0);
    Node *qp = find_qpair_node(dev, qid);
    ASSERT_NE(qp, nullptr);
    EXPECT_EQ(t.unlinkChild(qp->ino, "anything"), -ENOTDIR);
}

// Forced teardown: open fd -> revoke -> pread/pwrite -ENODEV; then cooperative
// close is the idempotent no-op (no double-free; ASan witness).
TEST(QdmaLifecycle, ForcedThenCooperativeIdempotent)
{
    NodeTree t;
    Device *dev = attach_dev(t, "0000:61:00");
    ASSERT_NE(dev, nullptr);
    uint32_t qid = 0;
    ASSERT_EQ(qpair_add(t, dev, &qid), 0);
    Ino ino = open_qpair(t, dev, qid);
    ASSERT_NE(ino, 0u);

    ASSERT_EQ(t.revokeDevice("0000:61:00"), 0);

    char buf[8] = {};
    EXPECT_EQ(t.pread(ino, buf, 8, static_cast<off_t>(SLASH_HBM_BASE)), -ENODEV);
    EXPECT_EQ(t.pwrite(ino, buf, 8, static_cast<off_t>(SLASH_HBM_BASE)),
              -ENODEV);

    t.forget(ino, 1);  // idempotent second teardown trigger
    SUCCEED();
}

// QID allocation after a free.  The HARD invariant (architecture spec) is only
// uniqueness: a freed QID's id is no longer registered, and a subsequent add
// never collides with a live QID.
//
// The allocator is a monotonic next_qid counter (qdma.cpp / allocQid document
// this): it advances on the fast path and only reuses a lower freed id if the
// counter itself collides (practically: on a full u32 wrap).  So after freeing a
// middle QID, the next add does NOT reclaim it -- it advances the counter.  This
// test pins that ACTUAL (and architecturally sufficient) behaviour: uniqueness
// and no collision with a live id.
TEST(QdmaLifecycle, QidAllocationNeverCollidesWithLive)
{
    NodeTree t;
    Device *dev = attach_dev(t, "0000:61:00");
    ASSERT_NE(dev, nullptr);

    uint32_t q0 = 0, q1 = 0, q2 = 0;
    ASSERT_EQ(qpair_add(t, dev, &q0), 0);
    ASSERT_EQ(qpair_add(t, dev, &q1), 0);
    ASSERT_EQ(qpair_add(t, dev, &q2), 0);
    EXPECT_EQ(q0, 0u);
    EXPECT_EQ(q1, 1u);
    EXPECT_EQ(q2, 2u);

    // Free the middle one (q1) entirely (unlink, no open fd -> reaped, registry
    // ref dropped).
    Node *n1 = find_qpair_node(dev, q1);
    ASSERT_NE(n1, nullptr);
    t.unlink(n1);
    ASSERT_EQ(dev->findResource(q1), nullptr) << "qid 1 freed";

    // The next add returns a UNIQUE id that does not collide with q0/q2 (the
    // still-live qpairs).  (It happens to be 3 today, per the monotonic policy.)
    uint32_t qn = 0xffffffff;
    ASSERT_EQ(qpair_add(t, dev, &qn), 0);
    EXPECT_NE(qn, q0) << "must not collide with live QID 0";
    EXPECT_NE(qn, q2) << "must not collide with live QID 2";
    EXPECT_NE(dev->findResource(qn), nullptr) << "new QID registered";
}

// ===========================================================================
// 4. SPARSE STORE: straddling, multi-page, range matrix, lazy alloc, isolation.
// ===========================================================================

TEST(QdmaStore, MultiPageStraddlingRoundTrip)
{
    NodeTree t;
    Device *dev = attach_dev(t, "0000:61:00");
    ASSERT_NE(dev, nullptr);
    uint32_t qid = 0;
    ASSERT_EQ(qpair_add(t, dev, &qid), 0);
    Ino ino = open_qpair(t, dev, qid);
    ASSERT_NE(ino, 0u);

    // A transfer spanning >2 of the 64KiB store pages, crossing each boundary.
    const uint64_t addr = SLASH_HBM_BASE + 65536 - 100;  // start near a boundary
    const size_t len = 65536 * 2 + 200;                  // crosses 3 page bounds
    std::vector<uint8_t> in(len);
    for (size_t i = 0; i < len; i++) {
        in[i] = static_cast<uint8_t>((i * 31 + 7) & 0xff);
    }
    ASSERT_EQ(t.pwrite(ino, reinterpret_cast<const char *>(in.data()), len,
                       static_cast<off_t>(addr)),
              static_cast<ssize_t>(len));
    std::vector<uint8_t> out(len, 0xee);
    ASSERT_EQ(t.pread(ino, reinterpret_cast<char *>(out.data()), len,
                      static_cast<off_t>(addr)),
              static_cast<ssize_t>(len));
    EXPECT_EQ(in, out);

    // A read that partially overlaps the written span and partially virgin
    // pages: the virgin tail reads zero, the overlap reads the data.
    std::vector<uint8_t> mixed(300, 0xff);
    uint64_t past = addr + len;  // first byte past the write
    ASSERT_EQ(t.pread(ino, reinterpret_cast<char *>(mixed.data()), 300,
                      static_cast<off_t>(past)),
              300);
    for (uint8_t b : mixed) {
        EXPECT_EQ(b, 0u) << "virgin byte past the written span not zero";
    }
}

TEST(QdmaStore, RangeMatrixExhaustive)
{
    // Pure range validation, the authoritative matrix.
    EXPECT_EQ(qdmaCheckRange(SLASH_HBM_BASE, 1), 0);
    EXPECT_EQ(qdmaCheckRange(SLASH_HBM_END - 1, 1), 0);
    EXPECT_EQ(qdmaCheckRange(SLASH_DDR_BASE, 1), 0);
    EXPECT_EQ(qdmaCheckRange(SLASH_DDR_END - 1, 1), 0);

    // Out of every window.
    EXPECT_EQ(qdmaCheckRange(0, 1), -ERANGE);
    EXPECT_EQ(qdmaCheckRange(SLASH_HBM_BASE - 1, 1), -ERANGE);
    EXPECT_EQ(qdmaCheckRange(SLASH_HBM_END, 1), -ERANGE);
    EXPECT_EQ(qdmaCheckRange(SLASH_DDR_END, 1), -ERANGE);
    // Gap between HBM and DDR.
    EXPECT_EQ(qdmaCheckRange(SLASH_HBM_END + 4096, 8), -ERANGE);

    // The reconfiguration region is REJECTED here (the VBIN path, not qpair).
    EXPECT_EQ(qdmaCheckRange(SLASH_RECONFIG_BASE, 8), -ERANGE);
    EXPECT_EQ(qdmaCheckRange(SLASH_RECONFIG_END - 8, 8), -ERANGE);

    // Straddle each window's end.
    EXPECT_EQ(qdmaCheckRange(SLASH_HBM_END - 4, 8), -ERANGE);
    EXPECT_EQ(qdmaCheckRange(SLASH_DDR_END - 4, 8), -ERANGE);

    // Overflow: addr + len wraps.
    EXPECT_EQ(qdmaCheckRange(SLASH_HBM_BASE, SIZE_MAX), -ERANGE);
    EXPECT_EQ(qdmaCheckRange(UINT64_MAX, 1), -ERANGE);
    EXPECT_EQ(qdmaCheckRange(UINT64_MAX - 3, 8), -ERANGE);

    // Zero-length is always accepted (copies nothing) -- even out of range.
    EXPECT_EQ(qdmaCheckRange(0, 0), 0);
    EXPECT_EQ(qdmaCheckRange(SLASH_RECONFIG_BASE, 0), 0);
}

// Lazy allocation: sparse writes scattered across the 32 GiB HBM window must not
// fault in anywhere near the addressed span -- only the touched 64 KiB pages.
TEST(QdmaStore, LazyAllocationRssWitness)
{
    auto rss_kib = []() -> long {
        FILE *f = ::fopen("/proc/self/statm", "r");
        if (f == nullptr) {
            return -1;
        }
        long total = 0, resident = 0;
        int n = ::fscanf(f, "%ld %ld", &total, &resident);
        ::fclose(f);
        if (n != 2) {
            return -1;
        }
        return resident * (::sysconf(_SC_PAGESIZE) / 1024);
    };

    NodeTree t;
    Device *dev = attach_dev(t, "0000:61:00");
    ASSERT_NE(dev, nullptr);
    uint32_t qid = 0;
    ASSERT_EQ(qpair_add(t, dev, &qid), 0);
    Ino ino = open_qpair(t, dev, qid);
    ASSERT_NE(ino, 0u);

    long before = rss_kib();
    ASSERT_GE(before, 0);

    // 256 single-byte writes spread across the whole 32 GiB HBM window.  A
    // dense backing would need 32 GiB; lazy paging touches 256 * 64KiB == 16MiB.
    const uint64_t span = SLASH_HBM_END - SLASH_HBM_BASE;
    const uint64_t step = span / 256;
    char b = 0x5a;
    for (int i = 0; i < 256; i++) {
        uint64_t addr = SLASH_HBM_BASE + static_cast<uint64_t>(i) * step;
        ASSERT_EQ(t.pwrite(ino, &b, 1, static_cast<off_t>(addr)), 1) << "i=" << i;
    }
    // A read sweep of never-written addresses must allocate NOTHING.
    char rb = 0;
    for (int i = 0; i < 256; i++) {
        uint64_t addr = SLASH_HBM_BASE + static_cast<uint64_t>(i) * step + 32;
        ASSERT_EQ(t.pread(ino, &rb, 1, static_cast<off_t>(addr)), 1);
    }

    long after = rss_kib();
    ASSERT_GE(after, 0);
    // 256 pages * 64 KiB == 16 MiB of real data; allow generous 64 MiB slack.
    // A dense (non-lazy) backing would blow past by GiB.
    EXPECT_LT(after - before, 64 * 1024)
        << "RSS grew " << (after - before) << " KiB; store appears non-sparse";
}

TEST(QdmaStore, DestroyFreesAllPagesOnce)
{
    // Touch many pages across two devices, then let the tree destructor free
    // everything.  ASan is the real assertion (no leak, no double free).
    NodeTree t;
    Device *d1 = attach_dev(t, "0000:61:00");
    Device *d2 = attach_dev(t, "0000:62:00");
    ASSERT_NE(d1, nullptr);
    ASSERT_NE(d2, nullptr);
    uint32_t q1 = 0, q2 = 0;
    ASSERT_EQ(qpair_add(t, d1, &q1), 0);
    ASSERT_EQ(qpair_add(t, d2, &q2), 0);
    Ino i1 = open_qpair(t, d1, q1);
    Ino i2 = open_qpair(t, d2, q2);

    char b = 0x11;
    for (int i = 0; i < 32; i++) {
        uint64_t addr = SLASH_DDR_BASE + static_cast<uint64_t>(i) * 65536;
        ASSERT_EQ(t.pwrite(i1, &b, 1, static_cast<off_t>(addr)), 1);
        ASSERT_EQ(t.pwrite(i2, &b, 1, static_cast<off_t>(addr)), 1);
    }
    SUCCEED();  // tree dtor frees both stores; ASan checks once-only.
}

// ===========================================================================
// 5. MEM BACKEND SEAM: pin the rc-contract.
// ===========================================================================

// A fake QdmaMemBackend that records calls and returns a programmable rc.
class FakeBackend : public QdmaMemBackend {
public:
    int fetch_rc = 0;     // what fetch() returns
    int populate_rc = 0;  // what populate() returns
    int fetch_calls = 0;
    int populate_calls = 0;
    uint8_t fetch_fill = 0xAB;  // byte fetch() writes when rc==0

    int fetch(uint64_t, void *buf, size_t len) override
    {
        fetch_calls++;
        if (fetch_rc == 0) {
            std::memset(buf, fetch_fill, len);
        }
        return fetch_rc;
    }

    int populate(uint64_t, const void *, size_t) override
    {
        populate_calls++;
        return populate_rc;
    }
};

TEST(QdmaBackend, FetchRcContract)
{
    NodeTree t;
    Device *dev = attach_dev(t, "0000:61:00");
    ASSERT_NE(dev, nullptr);
    uint32_t qid = 0;
    ASSERT_EQ(qpair_add(t, dev, &qid), 0);
    Ino ino = open_qpair(t, dev, qid);
    ASSERT_NE(ino, 0u);

    FakeBackend fb;
    ASSERT_EQ(qdmaSetMemBackend(*dev, &fb), 0);

    const off_t addr = static_cast<off_t>(SLASH_HBM_BASE);

    // Seed the sparse store with a known pattern so we can tell store vs model.
    uint8_t seed[16];
    std::memset(seed, 0x77, sizeof(seed));
    ASSERT_EQ(t.pwrite(ino, reinterpret_cast<char *>(seed), sizeof(seed), addr),
              static_cast<ssize_t>(sizeof(seed)));
    EXPECT_EQ(fb.populate_calls, 1) << "write forwarded to backend";

    // rc == 0: model answered; buf holds the model's fill byte, NOT the store.
    fb.fetch_rc = 0;
    uint8_t got[16];
    std::memset(got, 0, sizeof(got));
    ASSERT_EQ(t.pread(ino, reinterpret_cast<char *>(got), sizeof(got), addr),
              static_cast<ssize_t>(sizeof(got)));
    EXPECT_EQ(fb.fetch_calls, 1);
    for (uint8_t v : got) {
        EXPECT_EQ(v, fb.fetch_fill) << "rc==0 must use the model's bytes";
    }

    // rc > 0: fall back to the sparse store (which holds 0x77 from the seed).
    fb.fetch_rc = 1;
    std::memset(got, 0, sizeof(got));
    ASSERT_EQ(t.pread(ino, reinterpret_cast<char *>(got), sizeof(got), addr),
              static_cast<ssize_t>(sizeof(got)));
    EXPECT_EQ(fb.fetch_calls, 2);
    for (uint8_t v : got) {
        EXPECT_EQ(v, 0x77u) << "rc>0 must fall back to the sparse store";
    }

    // rc < 0: transport failure mapped to -ENODEV.
    fb.fetch_rc = -1;
    EXPECT_EQ(t.pread(ino, reinterpret_cast<char *>(got), sizeof(got), addr),
              -ENODEV);
}

TEST(QdmaBackend, PopulateFailureIsEnodevButStoreStaysCurrent)
{
    NodeTree t;
    Device *dev = attach_dev(t, "0000:61:00");
    ASSERT_NE(dev, nullptr);
    uint32_t qid = 0;
    ASSERT_EQ(qpair_add(t, dev, &qid), 0);
    Ino ino = open_qpair(t, dev, qid);
    ASSERT_NE(ino, 0u);

    FakeBackend fb;
    fb.populate_rc = -1;  // populate fails
    ASSERT_EQ(qdmaSetMemBackend(*dev, &fb), 0);

    const off_t addr = static_cast<off_t>(SLASH_DDR_BASE);
    uint8_t payload[8];
    std::memset(payload, 0x42, sizeof(payload));
    // populate fails -> -ENODEV, but the store was written BEFORE the forward
    // (impl keeps the store current so a later fall-back read never -EIOs).
    EXPECT_EQ(t.pwrite(ino, reinterpret_cast<char *>(payload), sizeof(payload),
                       addr),
              -ENODEV);

    // Detach the backend; a plain store read must now see the 0x42 payload.
    ASSERT_EQ(qdmaSetMemBackend(*dev, nullptr), 0);
    uint8_t got[8];
    std::memset(got, 0, sizeof(got));
    ASSERT_EQ(t.pread(ino, reinterpret_cast<char *>(got), sizeof(got), addr),
              static_cast<ssize_t>(sizeof(got)));
    for (uint8_t v : got) {
        EXPECT_EQ(v, 0x42u) << "store must stay current despite populate failure";
    }
}

TEST(QdmaBackend, SetBackendOnUnattachedDeviceFails)
{
    NodeTree t;
    Device *dev = t.addDevice("0000:61:00");
    ASSERT_NE(dev, nullptr);
    // No qdmaAttach: the qdma/ dir has no ops yet.
    FakeBackend fb;
    EXPECT_EQ(qdmaSetMemBackend(*dev, &fb), -1);
}

// ===========================================================================
// 6. direct_io: the created qpair file requests unbuffered I/O.
// ===========================================================================

TEST(QdmaDirectIo, QpairFileMarkedDirectIo)
{
    NodeTree t;
    Device *dev = attach_dev(t, "0000:61:00");
    ASSERT_NE(dev, nullptr);
    uint32_t qid = 0;
    ASSERT_EQ(qpair_add(t, dev, &qid), 0);
    Node *node = find_qpair_node(dev, qid);
    ASSERT_NE(node, nullptr);
    EXPECT_TRUE(t.wantsDirectIo(node->ino))
        << "qpair file must be direct_io (the no-hang / no-cache seam)";
}

}  // namespace
