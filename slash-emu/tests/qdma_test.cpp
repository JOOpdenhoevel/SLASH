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
 * @file qdma_test.cpp
 * @brief Unit + integration tests for the /<BDF>/qdma/ endpoint (T8).
 *
 * Two layers, matching the build rules (CMake+CTest, GTest, scratch under .tmp):
 *
 *   - Unit, against slash_emu_core directly (no FUSE mount): the pure validation
 *     matrices (emu_qdma_check_range, emu_qdma_check_qpair_add), and the
 *     end-to-end qpair lifecycle through the spine (emu_qdma_attach -> QPAIR_ADD
 *     via emu_node_ioctl -> qpair pread/pwrite via emu_node_pread/pwrite),
 *     including QID allocation/uniqueness, MM round-trip into HBM and DDR,
 *     out-of-range rejection, unwritten-reads-zero, the full nameless-qpair
 *     lifecycle (open -> unlink-while-open -> still works -> last close ->
 *     cooperative teardown exactly once), forced teardown via revoke (-ENODEV),
 *     double-teardown idempotency, multiple qpairs, and per-device isolation.
 *
 *   - Integration, over the real FUSE mount (fork+exec the freshly-built daemon,
 *     the bars_test.cpp pattern): QPAIR_ADD ioctl on the qdma/ dir fd, open the
 *     created qpair<Q>, MM round-trip, readdir visibility before unlink, the
 *     unlink-while-open VRTD pattern (fd keeps working after unlink), and that
 *     QPAIR_ADD with an unsupported mode is rejected.
 */

#include <gtest/gtest.h>

#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <set>
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
#include "node.h"
#include "qdma.h"
#include "slash/uapi/slash_abi.h"
}

namespace {

// ===========================================================================
// Unit: pure range-validation matrix (emu_qdma_check_range)
// ===========================================================================

TEST(QdmaCheckRange, ZeroLengthAlwaysInRange)
{
    // A zero-length transfer copies nothing; accepted everywhere, even at an
    // address outside any window (no bytes are touched).
    EXPECT_EQ(emu_qdma_check_range(0, 0), 0);
    EXPECT_EQ(emu_qdma_check_range(SLASH_HBM_BASE, 0), 0);
    EXPECT_EQ(emu_qdma_check_range(0xdeadbeefULL, 0), 0);
}

TEST(QdmaCheckRange, HbmWindowAccepted)
{
    EXPECT_EQ(emu_qdma_check_range(SLASH_HBM_BASE, 8), 0);
    EXPECT_EQ(emu_qdma_check_range(SLASH_HBM_BASE, SLASH_HBM_END - SLASH_HBM_BASE),
              0);
    // Last valid byte.
    EXPECT_EQ(emu_qdma_check_range(SLASH_HBM_END - 1, 1), 0);
}

TEST(QdmaCheckRange, DdrWindowAccepted)
{
    EXPECT_EQ(emu_qdma_check_range(SLASH_DDR_BASE, 4096), 0);
    EXPECT_EQ(emu_qdma_check_range(SLASH_DDR_END - 8, 8), 0);
}

TEST(QdmaCheckRange, OutsideWindowsRejected)
{
    // Below HBM, between HBM and DDR, and above DDR.
    EXPECT_EQ(emu_qdma_check_range(0, 8), -ERANGE);
    EXPECT_EQ(emu_qdma_check_range(SLASH_HBM_BASE - 8, 8), -ERANGE);
    EXPECT_EQ(emu_qdma_check_range(SLASH_HBM_END, 8), -ERANGE);
    EXPECT_EQ(emu_qdma_check_range(SLASH_DDR_END, 8), -ERANGE);
    // The reconfiguration region is NOT accepted here (T10's path).
    EXPECT_EQ(emu_qdma_check_range(SLASH_RECONFIG_BASE, 8), -ERANGE);
}

TEST(QdmaCheckRange, StraddlingWindowEndRejected)
{
    // A transfer that starts in-window but runs past the end is rejected,
    // not clamped.
    EXPECT_EQ(emu_qdma_check_range(SLASH_HBM_END - 4, 8), -ERANGE);
    EXPECT_EQ(emu_qdma_check_range(SLASH_DDR_END - 4, 8), -ERANGE);
}

TEST(QdmaCheckRange, OverflowRejected)
{
    EXPECT_EQ(emu_qdma_check_range(SLASH_HBM_BASE, SIZE_MAX), -ERANGE);
    EXPECT_EQ(emu_qdma_check_range(UINT64_MAX, 2), -ERANGE);
}

// ===========================================================================
// Unit: QPAIR_ADD parameter-validation matrix (emu_qdma_check_qpair_add)
// ===========================================================================

constexpr uint32_t kH2C = 0x1u;
constexpr uint32_t kC2H = 0x2u;
constexpr uint32_t kCMPT = 0x4u;
constexpr uint32_t kMM = 0u;
constexpr uint32_t kST = 1u;

TEST(QdmaCheckQpairAdd, ValidMmAccepted)
{
    EXPECT_EQ(emu_qdma_check_qpair_add(kMM, kH2C, 0, 0, 0), 0);
    EXPECT_EQ(emu_qdma_check_qpair_add(kMM, kC2H, 15, 15, 15), 0);
    EXPECT_EQ(emu_qdma_check_qpair_add(kMM, kH2C | kC2H, 7, 8, 0), 0);
}

TEST(QdmaCheckQpairAdd, StreamingModeUnsupported)
{
    EXPECT_EQ(emu_qdma_check_qpair_add(kST, kH2C, 0, 0, 0), -EOPNOTSUPP);
}

TEST(QdmaCheckQpairAdd, BadModeRejected)
{
    EXPECT_EQ(emu_qdma_check_qpair_add(2, kH2C, 0, 0, 0), -EINVAL);
}

TEST(QdmaCheckQpairAdd, CmptDirectionUnsupported)
{
    EXPECT_EQ(emu_qdma_check_qpair_add(kMM, kH2C | kCMPT, 0, 0, 0), -EOPNOTSUPP);
    EXPECT_EQ(emu_qdma_check_qpair_add(kMM, kCMPT, 0, 0, 0), -EOPNOTSUPP);
}

TEST(QdmaCheckQpairAdd, EmptyOrUnknownDirMaskRejected)
{
    EXPECT_EQ(emu_qdma_check_qpair_add(kMM, 0, 0, 0, 0), -EINVAL);
    EXPECT_EQ(emu_qdma_check_qpair_add(kMM, 0x8u, 0, 0, 0), -EINVAL);
}

TEST(QdmaCheckQpairAdd, OutOfRangeRingIndexRejected)
{
    EXPECT_EQ(emu_qdma_check_qpair_add(kMM, kH2C, 16, 0, 0), -EINVAL);
    EXPECT_EQ(emu_qdma_check_qpair_add(kMM, kH2C, 0, 16, 0), -EINVAL);
    EXPECT_EQ(emu_qdma_check_qpair_add(kMM, kH2C, 0, 0, 16), -EINVAL);
}

// ===========================================================================
// Unit: qpair lifecycle through the spine (no FUSE mount)
// ===========================================================================

// RAII tree wrapper (mirrors node_test.cpp / bars_test.cpp).
class Tree {
public:
    Tree() { EXPECT_EQ(emu_node_tree_new(&tree_, nullptr), 0); }
    ~Tree() { cleanup_node_tree(tree_); }
    emu_node_tree *get() { return tree_; }

private:
    emu_node_tree *tree_ = nullptr;
};

// Issue a QPAIR_ADD ioctl against the qdma/ dir inode and return the new QID.
// `mode`/`dir_mask` default to a valid MM/H2C qpair.
int qpair_add(emu_node_tree *tree, emu_device *dev, uint32_t *qid_out,
              uint32_t mode = kMM, uint32_t dir_mask = kH2C)
{
    struct slash_abi_qdma_qpair_add req {};
    req.size = sizeof(req);
    req.mode = mode;
    req.dir_mask = dir_mask;
    req.h2c_ring_sz = 0;
    req.c2h_ring_sz = 0;
    req.cmpt_ring_sz = 0;

    // _IOWR: in and out alias the same fixed-size region (read-modify-write),
    // exactly as the FUSE layer presents it.
    struct slash_abi_qdma_qpair_add out = req;
    int rc = emu_node_ioctl(tree, dev->qdma->ino, SLASH_ABI_QDMA_IOCTL_QPAIR_ADD,
                            &req, sizeof(req), &out, sizeof(out));
    if (rc == 0 && qid_out != nullptr) {
        *qid_out = out.qid;
    }
    return rc;
}

// Resolve qdma/qpair<Q> to its inode (bumps lookup_count, modelling an open).
emu_ino_t open_qpair(emu_node_tree *tree, emu_device *dev, uint32_t qid)
{
    char name[32];
    std::snprintf(name, sizeof(name), "qpair%u", qid);
    emu_node *child = nullptr;
    EXPECT_EQ(emu_node_lookup_child(tree, dev->qdma->ino, name, &child), 0)
        << name;
    return child != nullptr ? child->ino : 0;
}

// Find the qpair<Q> node WITHOUT bumping its lookup_count (a plain finder, not
// an "open"): walk the qdma/ directory's children directly.  emu_node_lookup_child
// would increment lookup_count, which would skew the cooperative-teardown tests.
emu_node *find_qpair_node(emu_node_tree *, emu_device *dev, uint32_t qid)
{
    char name[32];
    std::snprintf(name, sizeof(name), "qpair%u", qid);
    for (size_t i = 0; i < dev->qdma->children.len; i++) {
        emu_node *child = dev->qdma->children.d[i];
        if (!child->unlinked && std::strcmp(child->name, name) == 0) {
            return child;
        }
    }
    return nullptr;
}

TEST(QdmaEndpoint, AttachAddsIoctlToQdmaDir)
{
    Tree t;
    emu_device *dev = nullptr;
    ASSERT_EQ(emu_node_tree_add_device(t.get(), "0000:61:00", &dev), 0);
    ASSERT_EQ(emu_qdma_attach(dev), 0);

    uint32_t qid = 0xffffffff;
    ASSERT_EQ(qpair_add(t.get(), dev, &qid), 0);
    EXPECT_EQ(qid, 0u) << "first allocated QID is 0";

    // The qpair<0> file now resolves under qdma/.
    emu_node *child = find_qpair_node(t.get(), dev, 0);
    EXPECT_NE(child, nullptr);
}

TEST(QdmaEndpoint, IoctlOnNonQdmaNodeIsEnotty)
{
    // A node without an ioctl hook (a bar file) yields -ENOTTY.
    Tree t;
    emu_device *dev = nullptr;
    ASSERT_EQ(emu_node_tree_add_device(t.get(), "0000:61:00", &dev), 0);
    ASSERT_EQ(emu_qdma_attach(dev), 0);

    struct slash_abi_qdma_qpair_add req {};
    req.size = sizeof(req);
    // dev->dir has no ioctl hook.
    EXPECT_EQ(emu_node_ioctl(t.get(), dev->dir->ino,
                             SLASH_ABI_QDMA_IOCTL_QPAIR_ADD, &req, sizeof(req),
                             &req, sizeof(req)),
              -ENOTTY);
}

TEST(QdmaEndpoint, QidsAreUniqueAndMonotonic)
{
    Tree t;
    emu_device *dev = nullptr;
    ASSERT_EQ(emu_node_tree_add_device(t.get(), "0000:61:00", &dev), 0);
    ASSERT_EQ(emu_qdma_attach(dev), 0);

    std::set<uint32_t> seen;
    for (int i = 0; i < 8; i++) {
        uint32_t qid = 0;
        ASSERT_EQ(qpair_add(t.get(), dev, &qid), 0) << "add " << i;
        EXPECT_EQ(qid, static_cast<uint32_t>(i)) << "monotonic";
        EXPECT_TRUE(seen.insert(qid).second) << "unique";
    }
}

TEST(QdmaEndpoint, UnsupportedModeRejectedNoQpairCreated)
{
    Tree t;
    emu_device *dev = nullptr;
    ASSERT_EQ(emu_node_tree_add_device(t.get(), "0000:61:00", &dev), 0);
    ASSERT_EQ(emu_qdma_attach(dev), 0);

    uint32_t qid = 0;
    EXPECT_EQ(qpair_add(t.get(), dev, &qid, kST, kH2C), -EOPNOTSUPP);
    EXPECT_EQ(qpair_add(t.get(), dev, &qid, kMM, kCMPT), -EOPNOTSUPP);

    // No qpair node was created by a rejected add: the first VALID add still
    // gets QID 0 (the allocator did not advance for the rejected attempts).
    ASSERT_EQ(qpair_add(t.get(), dev, &qid, kMM, kH2C), 0);
    EXPECT_EQ(qid, 0u);
}

TEST(QdmaEndpoint, ShortIoctlBufferRejected)
{
    Tree t;
    emu_device *dev = nullptr;
    ASSERT_EQ(emu_node_tree_add_device(t.get(), "0000:61:00", &dev), 0);
    ASSERT_EQ(emu_qdma_attach(dev), 0);

    struct slash_abi_qdma_qpair_add req {};
    req.size = sizeof(req);
    req.mode = kMM;
    req.dir_mask = kH2C;
    // in_size too small to hold the input prefix.
    EXPECT_EQ(emu_node_ioctl(t.get(), dev->qdma->ino,
                             SLASH_ABI_QDMA_IOCTL_QPAIR_ADD, &req, 4, &req,
                             sizeof(req)),
              -EINVAL);
    // out_size too small to hold the qid.
    EXPECT_EQ(emu_node_ioctl(t.get(), dev->qdma->ino,
                             SLASH_ABI_QDMA_IOCTL_QPAIR_ADD, &req, sizeof(req),
                             &req, 4),
              -EINVAL);
}

TEST(QdmaEndpoint, UnknownIoctlCmdIsEnotty)
{
    Tree t;
    emu_device *dev = nullptr;
    ASSERT_EQ(emu_node_tree_add_device(t.get(), "0000:61:00", &dev), 0);
    ASSERT_EQ(emu_qdma_attach(dev), 0);

    char buf[64] = {};
    EXPECT_EQ(emu_node_ioctl(t.get(), dev->qdma->ino, 0xdeadbeef, buf,
                             sizeof(buf), buf, sizeof(buf)),
              -ENOTTY);
}

TEST(QdmaEndpoint, MmRoundTripHbmAndDdr)
{
    Tree t;
    emu_device *dev = nullptr;
    ASSERT_EQ(emu_node_tree_add_device(t.get(), "0000:61:00", &dev), 0);
    ASSERT_EQ(emu_qdma_attach(dev), 0);

    uint32_t qid = 0;
    ASSERT_EQ(qpair_add(t.get(), dev, &qid), 0);
    emu_ino_t ino = open_qpair(t.get(), dev, qid);
    ASSERT_NE(ino, 0u);

    struct Case {
        uint64_t addr;
        size_t len;
    };
    const Case cases[] = {
        { SLASH_HBM_BASE, 16 },
        { SLASH_HBM_BASE + 4096, 1024 },          // spans within a page
        { SLASH_HBM_BASE + 65536 - 8, 32 },       // straddles a 64KiB page
        { SLASH_DDR_BASE, 256 },
        { SLASH_DDR_END - 128, 128 },             // last bytes of DDR
    };

    for (const auto &c : cases) {
        std::vector<uint8_t> in(c.len);
        for (size_t i = 0; i < c.len; i++) {
            in[i] = static_cast<uint8_t>((c.addr + i) & 0xff);
        }
        ASSERT_EQ(emu_node_pwrite(t.get(), ino,
                                  reinterpret_cast<const char *>(in.data()),
                                  c.len, static_cast<off_t>(c.addr)),
                  static_cast<ssize_t>(c.len))
            << "addr " << std::hex << c.addr;

        std::vector<uint8_t> out(c.len, 0xcc);
        ASSERT_EQ(emu_node_pread(t.get(), ino,
                                 reinterpret_cast<char *>(out.data()), c.len,
                                 static_cast<off_t>(c.addr)),
                  static_cast<ssize_t>(c.len))
            << "addr " << std::hex << c.addr;
        EXPECT_EQ(in, out) << "addr " << std::hex << c.addr;
    }
}

TEST(QdmaEndpoint, UnwrittenReadsZero)
{
    Tree t;
    emu_device *dev = nullptr;
    ASSERT_EQ(emu_node_tree_add_device(t.get(), "0000:61:00", &dev), 0);
    ASSERT_EQ(emu_qdma_attach(dev), 0);

    uint32_t qid = 0;
    ASSERT_EQ(qpair_add(t.get(), dev, &qid), 0);
    emu_ino_t ino = open_qpair(t.get(), dev, qid);
    ASSERT_NE(ino, 0u);

    std::vector<uint8_t> out(512, 0xff);
    ASSERT_EQ(emu_node_pread(t.get(), ino, reinterpret_cast<char *>(out.data()),
                             out.size(), static_cast<off_t>(SLASH_HBM_BASE + 1)),
              static_cast<ssize_t>(out.size()));
    for (uint8_t b : out) {
        EXPECT_EQ(b, 0u);
    }
}

TEST(QdmaEndpoint, OutOfRangeRejected)
{
    Tree t;
    emu_device *dev = nullptr;
    ASSERT_EQ(emu_node_tree_add_device(t.get(), "0000:61:00", &dev), 0);
    ASSERT_EQ(emu_qdma_attach(dev), 0);

    uint32_t qid = 0;
    ASSERT_EQ(qpair_add(t.get(), dev, &qid), 0);
    emu_ino_t ino = open_qpair(t.get(), dev, qid);
    ASSERT_NE(ino, 0u);

    char buf[16] = {};
    // Below any window.
    EXPECT_EQ(emu_node_pwrite(t.get(), ino, buf, 8, 0), -ERANGE);
    EXPECT_EQ(emu_node_pread(t.get(), ino, buf, 8, 0), -ERANGE);
    // Straddling the HBM end.
    EXPECT_EQ(emu_node_pwrite(t.get(), ino, buf, 8,
                              static_cast<off_t>(SLASH_HBM_END - 4)),
              -ERANGE);
}

TEST(QdmaEndpoint, PerDeviceMemoryIsolation)
{
    Tree t;
    emu_device *d1 = nullptr;
    emu_device *d2 = nullptr;
    ASSERT_EQ(emu_node_tree_add_device(t.get(), "0000:61:00", &d1), 0);
    ASSERT_EQ(emu_node_tree_add_device(t.get(), "0000:62:00", &d2), 0);
    ASSERT_EQ(emu_qdma_attach(d1), 0);
    ASSERT_EQ(emu_qdma_attach(d2), 0);

    uint32_t q1 = 0;
    uint32_t q2 = 0;
    ASSERT_EQ(qpair_add(t.get(), d1, &q1), 0);
    ASSERT_EQ(qpair_add(t.get(), d2, &q2), 0);
    emu_ino_t i1 = open_qpair(t.get(), d1, q1);
    emu_ino_t i2 = open_qpair(t.get(), d2, q2);
    ASSERT_NE(i1, 0u);
    ASSERT_NE(i2, 0u);

    uint64_t v = 0xA5A5A5A5A5A5A5A5ull;
    ASSERT_EQ(emu_node_pwrite(t.get(), i1, reinterpret_cast<const char *>(&v), 8,
                              static_cast<off_t>(SLASH_HBM_BASE)),
              8);

    // Device 2's memory at the same address is untouched (zero).
    uint64_t got = 0xdeadbeef;
    ASSERT_EQ(emu_node_pread(t.get(), i2, reinterpret_cast<char *>(&got), 8,
                             static_cast<off_t>(SLASH_HBM_BASE)),
              8);
    EXPECT_EQ(got, 0u);
}

// The headline lifecycle: ADD -> open -> unlink-while-open -> STILL works via
// the open inode -> last close (forget) -> cooperative teardown. The qpair is
// nameless after unlink (gone from readdir, not resolvable) but the open inode
// keeps working until the final forget.
TEST(QdmaEndpoint, NamelessQpairLifecycle)
{
    Tree t;
    emu_device *dev = nullptr;
    ASSERT_EQ(emu_node_tree_add_device(t.get(), "0000:61:00", &dev), 0);
    ASSERT_EQ(emu_qdma_attach(dev), 0);

    uint32_t qid = 0;
    ASSERT_EQ(qpair_add(t.get(), dev, &qid), 0);

    // Open (lookup bumps lookup_count to 1, modelling the held fd).
    emu_node *node = find_qpair_node(t.get(), dev, qid);
    ASSERT_NE(node, nullptr);
    emu_ino_t ino = node->ino;
    {
        emu_node *child = nullptr;
        ASSERT_EQ(emu_node_lookup_child(t.get(), dev->qdma->ino, "qpair0",
                                        &child),
                  0);
    }

    // Unlink while open: nameless now -- not resolvable, gone from readdir.
    emu_node_unlink(t.get(), node);
    {
        emu_node *child = nullptr;
        EXPECT_EQ(emu_node_lookup_child(t.get(), dev->qdma->ino, "qpair0",
                                        &child),
                  -ENOENT);
    }

    // The open inode STILL works (a live, merely-unlinked qpair).
    uint64_t v = 0x1122334455667788ull;
    ASSERT_EQ(emu_node_pwrite(t.get(), ino, reinterpret_cast<const char *>(&v),
                              8, static_cast<off_t>(SLASH_HBM_BASE)),
              8);
    uint64_t got = 0;
    ASSERT_EQ(emu_node_pread(t.get(), ino, reinterpret_cast<char *>(&got), 8,
                             static_cast<off_t>(SLASH_HBM_BASE)),
              8);
    EXPECT_EQ(got, v);

    // Last close: cooperative teardown destroys the node + frees the resource.
    emu_node_forget(t.get(), ino, 1);

    // The inode is gone now; an op on the (stale) ino is -ENOENT.
    char buf[8] = {};
    EXPECT_EQ(emu_node_pread(t.get(), ino, buf, 8,
                             static_cast<off_t>(SLASH_HBM_BASE)),
              -ENOENT);
}

// Named-but-not-unlinked qpairs are visible in readdir until unlinked: this is
// how VRTD prunes leftovers from a previous crash.
TEST(QdmaEndpoint, NamedQpairVisibleInReaddirUntilUnlinked)
{
    Tree t;
    emu_device *dev = nullptr;
    ASSERT_EQ(emu_node_tree_add_device(t.get(), "0000:61:00", &dev), 0);
    ASSERT_EQ(emu_qdma_attach(dev), 0);

    uint32_t qid = 0;
    ASSERT_EQ(qpair_add(t.get(), dev, &qid), 0);

    struct Ctx {
        std::set<std::string> names;
    } ctx;
    auto cb = [](void *vctx, const char *name, emu_ino_t, enum emu_node_type) {
        static_cast<Ctx *>(vctx)->names.insert(name);
        return true;
    };
    ASSERT_EQ(emu_node_readdir(t.get(), dev->qdma->ino, cb, &ctx), 0);
    EXPECT_TRUE(ctx.names.count("qpair0")) << "named qpair visible in readdir";

    // Prune it (unlink with no open fd -> immediate destroy).
    emu_node *node = find_qpair_node(t.get(), dev, qid);
    ASSERT_NE(node, nullptr);
    emu_node_unlink(t.get(), node);

    Ctx ctx2;
    ASSERT_EQ(emu_node_readdir(t.get(), dev->qdma->ino, cb, &ctx2), 0);
    EXPECT_FALSE(ctx2.names.count("qpair0")) << "pruned qpair gone from readdir";
}

// Forced teardown via device revoke: an op on a still-open fd returns -ENODEV.
TEST(QdmaEndpoint, ForcedTeardownEnodevOnOpenFd)
{
    Tree t;
    emu_device *dev = nullptr;
    ASSERT_EQ(emu_node_tree_add_device(t.get(), "0000:61:00", &dev), 0);
    ASSERT_EQ(emu_qdma_attach(dev), 0);

    uint32_t qid = 0;
    ASSERT_EQ(qpair_add(t.get(), dev, &qid), 0);
    emu_ino_t ino = open_qpair(t.get(), dev, qid); // bumps lookup_count

    // Forced removal: the lookup above keeps the node alive as a dead orphan.
    ASSERT_EQ(emu_device_revoke(t.get(), "0000:61:00"), 0);

    char buf[8] = {};
    EXPECT_EQ(emu_node_pread(t.get(), ino, buf, 8,
                             static_cast<off_t>(SLASH_HBM_BASE)),
              -ENODEV);
    EXPECT_EQ(emu_node_pwrite(t.get(), ino, buf, 8,
                              static_cast<off_t>(SLASH_HBM_BASE)),
              -ENODEV);

    // Cooperative close after a forced teardown is a no-op (idempotent): the
    // forget must not crash or double-free.
    emu_node_forget(t.get(), ino, 1);
}

// Double-teardown idempotency: forced revoke then cooperative forget, both on a
// qpair that was unlinked-while-open. Neither path double-frees (ASan witness).
TEST(QdmaEndpoint, DoubleTeardownIdempotent)
{
    Tree t;
    emu_device *dev = nullptr;
    ASSERT_EQ(emu_node_tree_add_device(t.get(), "0000:61:00", &dev), 0);
    ASSERT_EQ(emu_qdma_attach(dev), 0);

    uint32_t qid = 0;
    ASSERT_EQ(qpair_add(t.get(), dev, &qid), 0);
    emu_node *node = find_qpair_node(t.get(), dev, qid);
    ASSERT_NE(node, nullptr);
    emu_ino_t ino = node->ino;

    // Open + unlink-while-open: nameless, registry+inode refs both live.
    {
        emu_node *child = nullptr;
        ASSERT_EQ(emu_node_lookup_child(t.get(), dev->qdma->ino, "qpair0",
                                        &child),
                  0);
    }
    emu_node_unlink(t.get(), node);

    // Forced teardown drops the registry ref + runs teardown once.
    ASSERT_EQ(emu_device_revoke(t.get(), "0000:61:00"), 0);
    // Cooperative trigger drops the inode ref; teardown is a no-op the 2nd time.
    emu_node_forget(t.get(), ino, 1);
    // No crash / no double free => pass (ASan build is the real assertion).
    SUCCEED();
}

TEST(QdmaEndpoint, MultipleQpairsShareDeviceMemory)
{
    // Two qpairs of the same device address the same per-device store: a write
    // through one is visible through the other (shared HBM/DDR).
    Tree t;
    emu_device *dev = nullptr;
    ASSERT_EQ(emu_node_tree_add_device(t.get(), "0000:61:00", &dev), 0);
    ASSERT_EQ(emu_qdma_attach(dev), 0);

    uint32_t qa = 0;
    uint32_t qb = 0;
    ASSERT_EQ(qpair_add(t.get(), dev, &qa), 0);
    ASSERT_EQ(qpair_add(t.get(), dev, &qb), 0);
    ASSERT_NE(qa, qb);
    emu_ino_t ia = open_qpair(t.get(), dev, qa);
    emu_ino_t ib = open_qpair(t.get(), dev, qb);

    uint64_t v = 0xCAFEF00DDEADBEEFull;
    ASSERT_EQ(emu_node_pwrite(t.get(), ia, reinterpret_cast<const char *>(&v), 8,
                              static_cast<off_t>(SLASH_DDR_BASE + 4096)),
              8);
    uint64_t got = 0;
    ASSERT_EQ(emu_node_pread(t.get(), ib, reinterpret_cast<char *>(&got), 8,
                             static_cast<off_t>(SLASH_DDR_BASE + 4096)),
              8);
    EXPECT_EQ(got, v);
}

// ===========================================================================
// Integration: QPAIR_ADD + qpair MM transfers over a real FUSE mount
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
        std::string(SLASH_EMU_TMP_DIR) + "/slash_emu_qdmacfg_XXXXXX";
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

// Fork+exec the daemon, run body() against the mount, then SIGTERM + reap.
template <typename Body>
void with_mounted_daemon(Body body)
{
    const std::string mountpoint = make_scratch("qdma");
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

    ASSERT_EQ(::kill(pid, SIGTERM), 0)
        << "kill failed: " << std::strerror(errno);
    int status = 0;
    bool exited = wait_for(
        [&] { return ::waitpid(pid, &status, WNOHANG) == pid; },
        kShutdownTimeoutMs);
    if (!exited) {
        ::kill(pid, SIGKILL);
        ::waitpid(pid, &status, 0);
    }
    EXPECT_TRUE(exited);

    ::rmdir(mountpoint.c_str());
    ::unlink(config.c_str());
}

// Issue QPAIR_ADD on the qdma/ dir fd; returns 0 and fills qid, or -1 (errno).
int mount_qpair_add(const std::string &qdma_dir, uint32_t *qid)
{
    int fd = ::open(qdma_dir.c_str(), O_RDONLY | O_DIRECTORY);
    if (fd < 0) {
        return -1;
    }
    struct slash_abi_qdma_qpair_add req {};
    req.size = sizeof(req);
    req.mode = 0;       // MM
    req.dir_mask = 0x1; // H2C
    int rc = ::ioctl(fd, SLASH_ABI_QDMA_IOCTL_QPAIR_ADD, &req);
    int saved = errno;
    ::close(fd);
    if (rc != 0) {
        errno = saved;
        return -1;
    }
    if (qid != nullptr) {
        *qid = req.qid;
    }
    return 0;
}

TEST(QdmaMount, QpairAddCreatesOpenableFile)
{
    with_mounted_daemon([](const std::string &mountpoint) {
        const std::string qdma = mountpoint + "/0000:61:00/qdma";

        uint32_t qid = 0xffffffff;
        ASSERT_EQ(mount_qpair_add(qdma, &qid), 0) << std::strerror(errno);
        EXPECT_EQ(qid, 0u);

        // The qpair<0> file is visible and openable.
        const std::string qpath = qdma + "/qpair0";
        struct stat st {};
        ASSERT_EQ(::stat(qpath.c_str(), &st), 0) << std::strerror(errno);
        EXPECT_TRUE(S_ISREG(st.st_mode));

        int fd = ::open(qpath.c_str(), O_RDWR);
        ASSERT_GE(fd, 0) << std::strerror(errno);
        ::close(fd);
    });
}

TEST(QdmaMount, QpairMmRoundTrip)
{
    with_mounted_daemon([](const std::string &mountpoint) {
        const std::string qdma = mountpoint + "/0000:61:00/qdma";
        uint32_t qid = 0;
        ASSERT_EQ(mount_qpair_add(qdma, &qid), 0) << std::strerror(errno);

        const std::string qpath = qdma + "/qpair" + std::to_string(qid);
        int fd = ::open(qpath.c_str(), O_RDWR);
        ASSERT_GE(fd, 0) << std::strerror(errno);

        // MM write + read back at an HBM address (the file offset IS the addr).
        std::vector<uint8_t> in(4096);
        for (size_t i = 0; i < in.size(); i++) {
            in[i] = static_cast<uint8_t>(i & 0xff);
        }
        ASSERT_EQ(::pwrite(fd, in.data(), in.size(),
                           static_cast<off_t>(SLASH_HBM_BASE)),
                  static_cast<ssize_t>(in.size()))
            << std::strerror(errno);

        std::vector<uint8_t> out(in.size(), 0xcc);
        ASSERT_EQ(::pread(fd, out.data(), out.size(),
                          static_cast<off_t>(SLASH_HBM_BASE)),
                  static_cast<ssize_t>(out.size()))
            << std::strerror(errno);
        EXPECT_EQ(in, out);

        // An out-of-range access is rejected with ERANGE.
        errno = 0;
        EXPECT_EQ(::pread(fd, out.data(), 8, 0), -1);
        EXPECT_EQ(errno, ERANGE);

        ::close(fd);
    });
}

// The VRTD pattern: ADD -> open -> unlink while open -> the fd keeps working.
TEST(QdmaMount, UnlinkWhileOpenKeepsWorking)
{
    with_mounted_daemon([](const std::string &mountpoint) {
        const std::string qdma = mountpoint + "/0000:61:00/qdma";
        uint32_t qid = 0;
        ASSERT_EQ(mount_qpair_add(qdma, &qid), 0) << std::strerror(errno);

        const std::string qpath = qdma + "/qpair" + std::to_string(qid);
        int fd = ::open(qpath.c_str(), O_RDWR);
        ASSERT_GE(fd, 0) << std::strerror(errno);

        // Unlink the qpair file while the fd is open (delete-on-last-close).
        ASSERT_EQ(::unlink(qpath.c_str()), 0) << std::strerror(errno);

        // The path is gone, but the open fd keeps working.
        struct stat st {};
        EXPECT_NE(::stat(qpath.c_str(), &st), 0);
        EXPECT_EQ(errno, ENOENT);

        uint64_t v = 0x0123456789abcdefull;
        ASSERT_EQ(::pwrite(fd, &v, sizeof(v),
                           static_cast<off_t>(SLASH_DDR_BASE)),
                  static_cast<ssize_t>(sizeof(v)))
            << std::strerror(errno);
        uint64_t got = 0;
        ASSERT_EQ(::pread(fd, &got, sizeof(got),
                          static_cast<off_t>(SLASH_DDR_BASE)),
                  static_cast<ssize_t>(sizeof(got)))
            << std::strerror(errno);
        EXPECT_EQ(got, v);

        ::close(fd);  // last close -> cooperative teardown
    });
}

TEST(QdmaMount, QpairVisibleInReaddirBeforeUnlink)
{
    with_mounted_daemon([](const std::string &mountpoint) {
        const std::string qdma = mountpoint + "/0000:61:00/qdma";
        uint32_t qid = 0;
        ASSERT_EQ(mount_qpair_add(qdma, &qid), 0) << std::strerror(errno);
        const std::string entry = "qpair" + std::to_string(qid);

        DIR *d = ::opendir(qdma.c_str());
        ASSERT_NE(d, nullptr) << std::strerror(errno);
        bool found = false;
        struct dirent *ent;
        while ((ent = ::readdir(d)) != nullptr) {
            if (entry == ent->d_name) {
                found = true;
            }
        }
        ::closedir(d);
        EXPECT_TRUE(found) << "named qpair should be visible in readdir(qdma/)";
    });
}

TEST(QdmaMount, UnsupportedModeIoctlRejected)
{
    with_mounted_daemon([](const std::string &mountpoint) {
        const std::string qdma = mountpoint + "/0000:61:00/qdma";
        int fd = ::open(qdma.c_str(), O_RDONLY | O_DIRECTORY);
        ASSERT_GE(fd, 0) << std::strerror(errno);

        struct slash_abi_qdma_qpair_add req {};
        req.size = sizeof(req);
        req.mode = 1;       // ST: unsupported
        req.dir_mask = 0x1; // H2C
        errno = 0;
        EXPECT_EQ(::ioctl(fd, SLASH_ABI_QDMA_IOCTL_QPAIR_ADD, &req), -1);
        EXPECT_EQ(errno, EOPNOTSUPP);

        ::close(fd);
    });
}

}  // namespace
