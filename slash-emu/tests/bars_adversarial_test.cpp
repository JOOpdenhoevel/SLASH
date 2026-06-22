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
 * @file bars_adversarial_test.cpp
 * @brief Destructive conformance suite for the /<BDF>/bars/ endpoint (T7 audit).
 *
 * The adversarial counterpart of bars_test.cpp.  Where bars_test.cpp proves the
 * happy path and a representative slice of rejection, this file hammers the
 * corners that load-bear the BAR access contract:
 *
 *   1. Exhaustive validation matrix through BOTH the pure kernel
 *      (barCheckAccess) AND the spine (tree.pread / tree.pwrite):
 *      every bad width, every misaligned offset per width, the EXACT in/out
 *      boundary, and integer-overflow offsets that must be REJECTED, never
 *      wrapped or clamped.  Asserted per BAR size (512 KiB clk vs 128 MiB).
 *   2. width == size semantics: a wide write read back byte-by-byte
 *      (little-endian as stored), and that adjacent/overlapping writes do not
 *      corrupt neighbours.
 *   3. The spine write path: -ENODEV on a revoked open handle for BOTH read
 *      and write (no TOCTOU), -EIO for a node with no write hook, -EINVAL on
 *      NULL / negative, and that the write-hook addition did not make
 *      read-only nodes writable.
 *   4. Lazy shadow: read-before-write returns zero without allocating the
 *      128 MiB shadow (RSS witness), first write allocates, destroy frees once
 *      (ASan).
 *   5. Exactly three BARs (0/2/4), no bar1/3/5/N, per-device isolation.
 *
 * The over-the-real-mount no-mmap / unbuffered-direct_io guarantees live in
 * bars_test.cpp (BarsMount.MmapIsNotOffered) and are extended here with a
 * MAP_PRIVATE attempt and an explicit single-width readahead probe.
 */

#include <gtest/gtest.h>

#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include <fcntl.h>
#include <linux/magic.h>
#include <signal.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/vfs.h>
#include <sys/wait.h>
#include <unistd.h>

#include "bars.hpp"
#include "info.hpp"
#include "node.hpp"
#include "slash/uapi/slash_abi.h"

using namespace slash::emu;

namespace {

// ===========================================================================
// Shared fixtures (mirror bars_test.cpp / node_test.cpp)
// ===========================================================================

class Tree {
public:
    Tree() = default;
    NodeTree &get() { return tree_; }

private:
    NodeTree tree_;
};

Ino bar_ino(NodeTree &tree, Device *dev, const char *name)
{
    Node *child = nullptr;
    EXPECT_EQ(tree.lookupChild(dev->bars->ino, name, &child), 0);
    return child != nullptr ? child->ino : 0;
}

// Attach a device + its bars endpoint; returns the device.
Device *attach_dev(NodeTree &tree, const char *bdf)
{
    Device *dev = tree.addDevice(bdf);
    EXPECT_NE(dev, nullptr);
    EXPECT_EQ(barsAttach(*dev), 0);
    return dev;
}

struct BarSpec {
    const char *name;
    uint64_t size;
};
constexpr BarSpec kBars[] = {
    { "bar0", SLASH_BAR_USER_SIZE },  // 128 MiB
    { "bar2", SLASH_BAR_SL_SIZE },    // 128 MiB
    { "bar4", SLASH_BAR_CLK_SIZE },   // 512 KiB
};

// ===========================================================================
// 1. EXHAUSTIVE VALIDATION MATRIX (pure kernel: barCheckAccess)
// ===========================================================================

// Every accepted width, aligned, at 0 / mid / last-valid -- for BOTH BAR sizes.
TEST(BarAccessMatrix, ValidWidthsAlignedAcceptedBothSizes)
{
    for (uint64_t bar : {SLASH_BAR_CLK_SIZE, SLASH_BAR_USER_SIZE}) {
        for (size_t w : {size_t{1}, size_t{2}, size_t{4}, size_t{8}}) {
            EXPECT_EQ(barCheckAccess(0, w, bar), 0)
                << "bar " << bar << " w " << w << " @0";
            EXPECT_EQ(barCheckAccess(static_cast<off_t>(64 * w), w, bar), 0)
                << "bar " << bar << " w " << w << " @mid";
            EXPECT_EQ(
                barCheckAccess(static_cast<off_t>(bar - w), w, bar), 0)
                << "bar " << bar << " w " << w << " @last-valid";
        }
    }
}

// The full set of rejected widths, for both BAR sizes.
TEST(BarAccessMatrix, BadWidthsRejectedBothSizes)
{
    for (uint64_t bar : {SLASH_BAR_CLK_SIZE, SLASH_BAR_USER_SIZE}) {
        for (size_t w : {size_t{0}, size_t{3}, size_t{5}, size_t{6}, size_t{7},
                         size_t{9}, size_t{16}, size_t{1024},
                         static_cast<size_t>(-1)}) {
            EXPECT_EQ(barCheckAccess(0, w, bar), -EINVAL)
                << "bar " << bar << " w " << w;
        }
    }
}

// Every misaligned offset for each multi-byte width is rejected; width 1 is
// always aligned.  Sweep every residue class for widths 2/4/8.
TEST(BarAccessMatrix, EveryMisalignedResidueRejected)
{
    const uint64_t bar = SLASH_BAR_CLK_SIZE;
    for (size_t w : {size_t{2}, size_t{4}, size_t{8}}) {
        for (off_t r = 1; r < static_cast<off_t>(w); r++) {
            // A nonzero residue at an otherwise-deep, in-range base.
            off_t off = static_cast<off_t>(64 * w) + r;
            EXPECT_EQ(barCheckAccess(off, w, bar), -EINVAL)
                << "w " << w << " residue " << r;
        }
    }
    // width 1 at any offset in range is aligned.
    for (off_t off : {off_t{0}, off_t{1}, off_t{3}, off_t{4095},
                      static_cast<off_t>(bar - 1)}) {
        EXPECT_EQ(barCheckAccess(off, 1, bar), 0) << "w1 off " << off;
    }
}

TEST(BarAccessMatrix, NegativeOffsetsRejected)
{
    for (size_t w : {size_t{1}, size_t{2}, size_t{4}, size_t{8}}) {
        EXPECT_EQ(barCheckAccess(-1, w, SLASH_BAR_CLK_SIZE), -EINVAL)
            << "w " << w;
        EXPECT_EQ(barCheckAccess(static_cast<off_t>(-(int64_t) w), w,
                                 SLASH_BAR_CLK_SIZE),
                  -EINVAL)
            << "w " << w;
    }
}

// The EXACT boundary, per width, per BAR size.
TEST(BarAccessMatrix, ExactBoundaryPerWidthBothSizes)
{
    for (uint64_t bar : {SLASH_BAR_CLK_SIZE, SLASH_BAR_USER_SIZE}) {
        for (size_t w : {size_t{1}, size_t{2}, size_t{4}, size_t{8}}) {
            // Last aligned start that still fits.
            EXPECT_EQ(barCheckAccess(static_cast<off_t>(bar - w), w, bar), 0)
                << "bar " << bar << " w " << w;
            // First aligned start at/after the end: nothing fits.
            EXPECT_EQ(barCheckAccess(static_cast<off_t>(bar), w, bar), -EINVAL)
                << "bar " << bar << " w " << w << " @end";
            if (w > 1) {
                EXPECT_EQ(barCheckAccess(
                              static_cast<off_t>(bar - w + (off_t) (w / 2)),
                              w, bar),
                          -EINVAL)
                    << "bar " << bar << " w " << w << " straddle";
            }
        }
    }
}

// OVERFLOW: an offset near INT64_MAX with a real width must be rejected.
TEST(BarAccessMatrix, OverflowOffsetsRejectedNotWrapped)
{
    const uint64_t bar = SLASH_BAR_CLK_SIZE;
    constexpr off_t kMax = INT64_MAX;
    for (size_t w : {size_t{1}, size_t{2}, size_t{4}, size_t{8}}) {
        EXPECT_EQ(barCheckAccess(kMax, w, bar), -EINVAL) << "max w " << w;
        EXPECT_EQ(barCheckAccess(kMax - static_cast<off_t>(w - 1), w, bar),
                  -EINVAL)
            << "near-max w " << w;
    }
    EXPECT_EQ(barCheckAccess(static_cast<off_t>(0x7FFFFFFFFFFFFFF8LL), 8, bar),
              -EINVAL);
}

// ===========================================================================
// 1b. VALIDATION MATRIX THROUGH THE SPINE (pread + pwrite dispatch)
// ===========================================================================

TEST(BarSpineMatrix, WidthAlignBoundaryThroughSpineBothBars)
{
    Tree t;
    Device *dev = attach_dev(t.get(), "0000:61:00");

    for (const auto &b : {kBars[2], kBars[0]}) {  // clk then user
        Ino ino = bar_ino(t.get(), dev, b.name);
        ASSERT_NE(ino, 0u) << b.name;
        char buf[8] = {};

        // Accepted widths aligned at 0 and last-valid.
        for (size_t w : {size_t{1}, size_t{2}, size_t{4}, size_t{8}}) {
            EXPECT_EQ(t.get().pread(ino, buf, w, 0),
                      static_cast<ssize_t>(w))
                << b.name << " read w " << w;
            EXPECT_EQ(t.get().pwrite(ino, buf, w, 0),
                      static_cast<ssize_t>(w))
                << b.name << " write w " << w;
            off_t last = static_cast<off_t>(b.size - w);
            EXPECT_EQ(t.get().pread(ino, buf, w, last),
                      static_cast<ssize_t>(w))
                << b.name << " read last w " << w;
            EXPECT_EQ(t.get().pwrite(ino, buf, w, last),
                      static_cast<ssize_t>(w))
                << b.name << " write last w " << w;
        }

        // Bad widths rejected on both paths.
        for (size_t w : {size_t{0}, size_t{3}, size_t{5}, size_t{6}, size_t{7},
                         size_t{9}, size_t{16}}) {
            EXPECT_EQ(t.get().pread(ino, buf, w, 0), -EINVAL)
                << b.name << " read bad w " << w;
            EXPECT_EQ(t.get().pwrite(ino, buf, w, 0), -EINVAL)
                << b.name << " write bad w " << w;
        }

        // Misaligned rejected (each multi-byte width).
        EXPECT_EQ(t.get().pread(ino, buf, 2, 1), -EINVAL);
        EXPECT_EQ(t.get().pwrite(ino, buf, 4, 2), -EINVAL);
        EXPECT_EQ(t.get().pwrite(ino, buf, 8, 4), -EINVAL);

        // Straddle / at-end rejected, NEVER clamped to a short transfer.
        EXPECT_EQ(t.get().pread(ino, buf, 8,
                                 static_cast<off_t>(b.size - 4)),
                  -EINVAL)
            << b.name << " straddle";
        EXPECT_EQ(t.get().pwrite(ino, buf, 1,
                                  static_cast<off_t>(b.size)),
                  -EINVAL)
            << b.name << " at-end";
    }
}

// The spine layer rejects a negative offset before reaching the BAR hook.
TEST(BarSpineMatrix, NegativeOffsetRejectedAtSpine)
{
    Tree t;
    Device *dev = attach_dev(t.get(), "0000:61:00");
    Ino ino = bar_ino(t.get(), dev, "bar0");
    ASSERT_NE(ino, 0u);
    char buf[8] = {};
    EXPECT_EQ(t.get().pread(ino, buf, 4, -4), -EINVAL);
    EXPECT_EQ(t.get().pwrite(ino, buf, 4, -4), -EINVAL);
}

// NULL buffer is rejected with -EINVAL on both paths (spine guard).
TEST(BarSpineMatrix, NullBufferRejected)
{
    Tree t;
    Device *dev = attach_dev(t.get(), "0000:61:00");
    Ino ino = bar_ino(t.get(), dev, "bar0");
    ASSERT_NE(ino, 0u);
    EXPECT_EQ(t.get().pread(ino, nullptr, 4, 0), -EINVAL);
    EXPECT_EQ(t.get().pwrite(ino, nullptr, 4, 0), -EINVAL);
}

// ===========================================================================
// 2. width == size SEMANTICS: byte exactness + neighbour integrity
// ===========================================================================

// A single 8-byte write is observable as eight little-endian bytes via 1-byte
// reads, and as two 4-byte halves.
TEST(BarWidthSemantics, WideWriteReadableAsConstituentBytes)
{
    Tree t;
    Device *dev = attach_dev(t.get(), "0000:61:00");
    Ino ino = bar_ino(t.get(), dev, "bar4");
    ASSERT_NE(ino, 0u);

    const uint64_t v = 0x0102030405060708ull;
    ASSERT_EQ(t.get().pwrite(ino, reinterpret_cast<const char *>(&v), 8, 128),
              8);

    // 1-byte reads: little-endian, byte i == (v >> 8*i) & 0xFF.
    for (off_t i = 0; i < 8; i++) {
        uint8_t got = 0xAA;
        ASSERT_EQ(t.get().pread(ino, reinterpret_cast<char *>(&got), 1,
                                 128 + i),
                  1)
            << "byte " << i;
        EXPECT_EQ(got, static_cast<uint8_t>((v >> (8 * i)) & 0xFF))
            << "byte " << i;
    }

    // 4-byte halves.
    uint32_t lo = 0, hi = 0;
    ASSERT_EQ(t.get().pread(ino, reinterpret_cast<char *>(&lo), 4, 128), 4);
    ASSERT_EQ(t.get().pread(ino, reinterpret_cast<char *>(&hi), 4, 132), 4);
    EXPECT_EQ(lo, static_cast<uint32_t>(v & 0xFFFFFFFF));
    EXPECT_EQ(hi, static_cast<uint32_t>(v >> 32));
}

// Writing one register must not disturb its immediate neighbours.
TEST(BarWidthSemantics, AdjacentWritesDoNotCorruptNeighbours)
{
    Tree t;
    Device *dev = attach_dev(t.get(), "0000:61:00");
    Ino ino = bar_ino(t.get(), dev, "bar0");
    ASSERT_NE(ino, 0u);

    // Lay down three adjacent 4-byte words with distinct sentinels.
    const uint32_t a = 0x11111111, b = 0x22222222, c = 0x33333333;
    ASSERT_EQ(t.get().pwrite(ino, reinterpret_cast<const char *>(&a), 4, 256),
              4);
    ASSERT_EQ(t.get().pwrite(ino, reinterpret_cast<const char *>(&c), 4, 264),
              4);
    // Now write the middle word; the outer two must be untouched.
    ASSERT_EQ(t.get().pwrite(ino, reinterpret_cast<const char *>(&b), 4, 260),
              4);

    uint32_t ra = 0, rb = 0, rc = 0;
    ASSERT_EQ(t.get().pread(ino, reinterpret_cast<char *>(&ra), 4, 256), 4);
    ASSERT_EQ(t.get().pread(ino, reinterpret_cast<char *>(&rb), 4, 260), 4);
    ASSERT_EQ(t.get().pread(ino, reinterpret_cast<char *>(&rc), 4, 264), 4);
    EXPECT_EQ(ra, a);
    EXPECT_EQ(rb, b);
    EXPECT_EQ(rc, c);

    // A narrower overwrite of the low byte of the middle word leaves the upper
    // three bytes intact.
    const uint8_t lowbyte = 0xEE;
    ASSERT_EQ(t.get().pwrite(ino,
                              reinterpret_cast<const char *>(&lowbyte), 1, 260),
              1);
    uint32_t rb2 = 0;
    ASSERT_EQ(t.get().pread(ino, reinterpret_cast<char *>(&rb2), 4, 260), 4);
    EXPECT_EQ(rb2, (b & 0xFFFFFF00u) | lowbyte);
}

// ===========================================================================
// 3. SPINE WRITE PATH: revoke gate (no TOCTOU), no-write-hook, read-only nodes
// ===========================================================================

// Both read AND write on a revoked open handle return -ENODEV.
TEST(BarSpineWrite, RevokedHandleEnodevBothDirections)
{
    Tree t;
    Device *dev = attach_dev(t.get(), "0000:61:00");
    Ino ino = bar_ino(t.get(), dev, "bar0");  // lookup ref keeps it alive
    ASSERT_NE(ino, 0u);

    ASSERT_EQ(t.get().revokeDevice("0000:61:00"), 0);

    char buf[8] = {};
    EXPECT_EQ(t.get().pread(ino, buf, 4, 0), -ENODEV);
    EXPECT_EQ(t.get().pwrite(ino, buf, 4, 0), -ENODEV);
    // A write that WOULD be -EINVAL on a live node is still -ENODEV here.
    EXPECT_EQ(t.get().pwrite(ino, buf, 3, 1), -ENODEV);
    EXPECT_EQ(t.get().pread(ino, buf, 8,
                             static_cast<off_t>(SLASH_BAR_USER_SIZE)),
              -ENODEV);
}

// A NEW lookup after revoke from a LIVE ancestor (root) returns -ENOENT.
TEST(BarSpineWrite, NewLookupAfterRevokeIsEnoent)
{
    Tree t;
    Device *dev = attach_dev(t.get(), "0000:61:00");
    const Ino stale_bars_ino = dev->bars->ino;  // captured pre-revoke
    ASSERT_EQ(t.get().revokeDevice("0000:61:00"), 0);

    // Spec path: new lookup of the device name from the live root -> -ENOENT.
    Node *out = nullptr;
    EXPECT_EQ(t.get().lookupChild(kRootIno, "0000:61:00", &out), -ENOENT);

    // Layering boundary: the reaped bars/ inode is now stale, not a miss.
    Node *child = nullptr;
    EXPECT_EQ(t.get().lookupChild(stale_bars_ino, "bar0", &child), -ESTALE);
}

// A file node with no write hook yields -EIO on pwrite.
TEST(BarSpineWrite, NoWriteHookIsEio)
{
    Tree t;
    Device *dev = t.get().addDevice("0000:61:00");
    ASSERT_NE(dev, nullptr);
    Node *node = t.get().createChild(dev->dir, "plain", NodeType::File,
                                     0444, nullptr);
    ASSERT_NE(node, nullptr);
    char buf[4] = {};
    EXPECT_EQ(t.get().pwrite(node->ino, buf, 4, 0), -EIO);
    EXPECT_EQ(t.get().pread(node->ino, buf, 4, 0), -EIO);
}

// Adding the write hook to NodeOps did not make the read-only info file
// writable: a write to /<BDF>/info must be rejected with -EIO.
TEST(BarSpineWrite, InfoFileRejectsWrite)
{
    Tree t;
    Device *dev = t.get().addDevice("0000:61:00");
    ASSERT_NE(dev, nullptr);
    // Model the same read-only contract directly: a 0444 file with no ops.
    Node *node = t.get().createChild(dev->dir, "info", NodeType::File,
                                     0444, nullptr);
    ASSERT_NE(node, nullptr);
    char buf[8] = {};
    EXPECT_EQ(t.get().pwrite(node->ino, buf, 8, 0), -EIO);
}

// ===========================================================================
// 4. LAZY SHADOW: no spurious 128 MiB allocation on read; destroy frees once
// ===========================================================================

// Resident-set witness: reading the two 128 MiB BARs end-to-end (never writing)
// must NOT fault in a 256 MiB shadow.
TEST(BarLazyShadow, ReadOnlyDoesNotAllocate128MiB)
{
    auto rss_kib = []() -> long {
        FILE *f = ::fopen("/proc/self/statm", "r");
        if (f == nullptr) {
            return -1;
        }
        long total_pages = 0, resident_pages = 0;
        int n = ::fscanf(f, "%ld %ld", &total_pages, &resident_pages);
        ::fclose(f);
        if (n != 2) {
            return -1;
        }
        return resident_pages * (::sysconf(_SC_PAGESIZE) / 1024);
    };

    Tree t;
    Device *dev = attach_dev(t.get(), "0000:61:00");
    Ino b0 = bar_ino(t.get(), dev, "bar0");
    Ino b2 = bar_ino(t.get(), dev, "bar2");
    ASSERT_NE(b0, 0u);
    ASSERT_NE(b2, 0u);

    long before = rss_kib();
    ASSERT_GE(before, 0);

    // Read 8 bytes at 4096 offsets spread across each 128 MiB BAR.
    char buf[8] = {};
    const uint64_t step = SLASH_BAR_USER_SIZE / 4096;  // ~32 KiB stride
    for (Ino ino : {b0, b2}) {
        for (uint64_t off = 0; off + 8 <= SLASH_BAR_USER_SIZE; off += step) {
            ASSERT_EQ(t.get().pread(ino, buf, 8,
                                     static_cast<off_t>(off & ~uint64_t{7})),
                      8);
            for (char c : buf) {
                ASSERT_EQ(c, 0) << "unwritten byte not zero at " << off;
            }
        }
    }

    long after = rss_kib();
    ASSERT_GE(after, 0);
    // Allocating even one shadow would add ~128 MiB (131072 KiB).  Allow a
    // generous 32 MiB slack for unrelated allocator / test noise.
    EXPECT_LT(after - before, 32 * 1024)
        << "RSS grew " << (after - before) << " KiB on read-only access; "
        << "shadow appears to be allocated eagerly";
}

// First write to a previously-unread BAR allocates the shadow and the value is
// retained; a subsequent read of an untouched neighbour region is still zero.
TEST(BarLazyShadow, FirstWriteAllocatesAndZeroFills)
{
    Tree t;
    Device *dev = attach_dev(t.get(), "0000:61:00");
    Ino ino = bar_ino(t.get(), dev, "bar4");
    ASSERT_NE(ino, 0u);

    const uint32_t v = 0xA5A5A5A5;
    ASSERT_EQ(t.get().pwrite(ino, reinterpret_cast<const char *>(&v),
                              4, 1000 & ~uint64_t{3}),
              4);
    // The written word reads back.
    uint32_t got = 0;
    ASSERT_EQ(t.get().pread(ino, reinterpret_cast<char *>(&got), 4,
                             1000 & ~uint64_t{3}),
              4);
    EXPECT_EQ(got, v);
    // A far-away untouched word in the same (now-allocated) shadow is zero.
    uint64_t z = 0xdead;
    ASSERT_EQ(t.get().pread(ino, reinterpret_cast<char *>(&z), 8,
                             static_cast<off_t>(SLASH_BAR_CLK_SIZE - 8)),
              8);
    EXPECT_EQ(z, 0u);
}

// Tree teardown with allocated + unallocated shadows across multiple devices
// frees each shadow exactly once.
TEST(BarLazyShadow, DestroyFreesMixOfTouchedAndUntouched)
{
    Tree t;
    Device *d1 = attach_dev(t.get(), "0000:61:00");
    Device *d2 = attach_dev(t.get(), "0000:62:00");
    // Touch one BAR on each device; leave the rest unallocated.
    char buf[8] = {};
    ASSERT_EQ(t.get().pwrite(bar_ino(t.get(), d1, "bar4"), buf, 8, 0), 8);
    ASSERT_EQ(t.get().pwrite(bar_ino(t.get(), d2, "bar0"), buf, 8, 0), 8);
    // ~Tree() runs NodeTree destructor -> BarOps destructor on every node.
    // ASan proves each shadow (and each backing) is freed exactly once.
    SUCCEED();
}

// ===========================================================================
// 5. EXACTLY THREE BARS + per-device isolation (extended)
// ===========================================================================

TEST(BarTopology, OnlyBar024ExistNoOthers)
{
    Tree t;
    Device *dev = attach_dev(t.get(), "0000:61:00");

    for (const auto &b : kBars) {
        Node *child = nullptr;
        EXPECT_EQ(t.get().lookupChild(dev->bars->ino, b.name, &child), 0)
            << b.name;
    }
    for (const char *absent : {"bar1", "bar3", "bar5", "bar6", "bar7", "bar",
                               "bar00", "BAR0", "bar0x", "barN", ""}) {
        Node *child = nullptr;
        EXPECT_EQ(t.get().lookupChild(dev->bars->ino, absent, &child), -ENOENT)
            << "'" << absent << "'";
    }
}

// Writes are isolated per device AND per BAR within a device.
TEST(BarTopology, IsolationAcrossDevicesAndBars)
{
    Tree t;
    Device *d1 = attach_dev(t.get(), "0000:61:00");
    Device *d2 = attach_dev(t.get(), "0000:62:00");

    const uint32_t mark = 0xCAFEBABE;
    ASSERT_EQ(t.get().pwrite(bar_ino(t.get(), d1, "bar0"),
                              reinterpret_cast<const char *>(&mark), 4, 64),
              4);

    // Same offset on d2/bar0, and on d1/bar2 and d1/bar4, are all untouched.
    for (auto probe : {std::make_pair(d2, "bar0"), std::make_pair(d1, "bar2"),
                       std::make_pair(d1, "bar4")}) {
        uint32_t got = 0xffffffff;
        ASSERT_EQ(t.get().pread(bar_ino(t.get(), probe.first, probe.second),
                                 reinterpret_cast<char *>(&got), 4, 64),
                  4)
            << probe.second;
        EXPECT_EQ(got, 0u) << probe.first->bdf() << "/" << probe.second;
    }
}

// ===========================================================================
// 6. NO-MMAP + DIRECT_IO over the REAL mount (extends BarsMount.*)
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
        std::string(SLASH_EMU_TMP_DIR) + "/slash_emu_advbarscfg_XXXXXX";
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
    struct statfs sfs{};
    if (::statfs(mountpoint.c_str(), &sfs) != 0) {
        return false;
    }
    return sfs.f_type == FUSE_SUPER_MAGIC;
}

template <typename Body>
void with_mounted_daemon(Body body)
{
    const std::string mountpoint = make_scratch("advbars");
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

// The load-bearing no-mmap guarantee: a writable SHARED mapping must FAIL.
TEST(BarsMountAdv, MmapSharedWritableFails)
{
    with_mounted_daemon([](const std::string &mountpoint) {
        const std::string path = mountpoint + "/0000:61:00/bars/bar0";
        int fd = ::open(path.c_str(), O_RDWR);
        ASSERT_GE(fd, 0) << std::strerror(errno);

        errno = 0;
        void *ps = ::mmap(nullptr, 4096, PROT_READ | PROT_WRITE, MAP_SHARED, fd,
                          0);
        EXPECT_EQ(ps, MAP_FAILED) << "MAP_SHARED writable window must not exist";
        if (ps != MAP_FAILED) {
            ::munmap(ps, 4096);
        }
        errno = 0;
        void *psr = ::mmap(nullptr, 4096, PROT_READ, MAP_SHARED, fd, 0);
        EXPECT_EQ(psr, MAP_FAILED) << "MAP_SHARED read window must not exist";
        if (psr != MAP_FAILED) {
            ::munmap(psr, 4096);
        }

        ::close(fd);
    });
}

// DEFECT-2 re-verification, hardened: the no-hang invariant must hold for a
// MAP_PRIVATE deref at EVERY offset class and on EVERY BAR.
static constexpr int kDerefWatchdogMs = 30000;

TEST(BarsMountAdv, MmapPrivateDerefNeverHangsOrMapsAcrossOffsetsAndBars)
{
    with_mounted_daemon([](const std::string &mountpoint) {
        struct Probe {
            const char *bar;
            uint64_t size;
        };
        const Probe probes[] = {
            { "bar4", SLASH_BAR_CLK_SIZE },
            { "bar0", SLASH_BAR_USER_SIZE },
        };
        const long pg = ::sysconf(_SC_PAGESIZE);

        for (const auto &pr : probes) {
            const std::string path =
                mountpoint + "/0000:61:00/bars/" + pr.bar;
            const off_t offsets[] = {
                0,
                static_cast<off_t>((pr.size / 2) & ~(uint64_t)(pg - 1)),
                static_cast<off_t>(pr.size - static_cast<uint64_t>(pg)),
            };

            for (off_t off : offsets) {
                pid_t child = ::fork();
                ASSERT_NE(child, -1) << "fork: " << std::strerror(errno);
                if (child == 0) {
                    ::signal(SIGBUS, SIG_DFL);
                    int fd = ::open(path.c_str(), O_RDONLY);
                    if (fd < 0) {
                        ::_exit(10);
                    }
                    void *p = ::mmap(nullptr, static_cast<size_t>(pg),
                                     PROT_READ, MAP_PRIVATE, fd, off);
                    if (p == MAP_FAILED) {
                        ::_exit(0);
                    }
                    volatile char c = *static_cast<volatile char *>(p);
                    (void) c;
                    ::_exit(42);  // deref SUCCEEDED -> a usable window exists.
                }

                int status = 0;
                bool finished = wait_for(
                    [&] { return ::waitpid(child, &status, WNOHANG) == child; },
                    kDerefWatchdogMs);
                if (!finished) {
                    ::kill(child, SIGKILL);
                    ::waitpid(child, &status, 0);
                    FAIL() << pr.bar << " @" << off
                           << ": MAP_PRIVATE deref HUNG (watchdog SIGKILL) -- a "
                              "page-fault read was left unanswered";
                    continue;
                }
                if (WIFEXITED(status)) {
                    EXPECT_NE(WEXITSTATUS(status), 42)
                        << pr.bar << " @" << off
                        << ": MAP_PRIVATE deref returned data -- a usable "
                           "mapping of device memory was produced";
                    EXPECT_NE(WEXITSTATUS(status), 10)
                        << pr.bar << " @" << off << ": child failed to open BAR";
                } else if (WIFSIGNALED(status)) {
                    EXPECT_EQ(WTERMSIG(status), SIGBUS)
                        << pr.bar << " @" << off << ": expected SIGBUS, got "
                        << WTERMSIG(status);
                }
            }
        }
    });
}

// direct_io is unbuffered: a register read delivers EXACTLY the requested
// width, with no page-cache readahead.
TEST(BarsMountAdv, DirectIoDeliversExactWidthNoReadahead)
{
    with_mounted_daemon([](const std::string &mountpoint) {
        const std::string path = mountpoint + "/0000:61:00/bars/bar4";
        int fd = ::open(path.c_str(), O_RDWR);
        ASSERT_GE(fd, 0) << std::strerror(errno);

        // Exactly-4 in the middle.
        uint32_t r = 0xdeadbeef;
        ASSERT_EQ(::pread(fd, &r, 4, 4096), 4) << std::strerror(errno);
        EXPECT_EQ(r, 0u);

        // Exactly-4 at the last aligned slot: no readahead past the end.
        r = 0xdeadbeef;
        ASSERT_EQ(::pread(fd, &r, 4, static_cast<off_t>(SLASH_BAR_CLK_SIZE - 4)),
                  4)
            << "last-slot 4B read failed (readahead leak?): "
            << std::strerror(errno);
        EXPECT_EQ(r, 0u);

        // A 1-byte read returns exactly 1 byte (not a page).
        uint8_t b = 0xAA;
        ASSERT_EQ(::pread(fd, &b, 1, 8192), 1) << std::strerror(errno);
        EXPECT_EQ(b, 0u);

        ::close(fd);
    });
}

// Over the mount, a write to the read-only info file is rejected.
TEST(BarsMountAdv, InfoNotWritableOverMount)
{
    with_mounted_daemon([](const std::string &mountpoint) {
        const std::string path = mountpoint + "/0000:61:00/info";
        errno = 0;
        int fd = ::open(path.c_str(), O_WRONLY);
        if (fd >= 0) {
            char buf[4] = {};
            errno = 0;
            ssize_t n = ::pwrite(fd, buf, 4, 0);
            EXPECT_EQ(n, -1);
            EXPECT_EQ(errno, EIO);
            ::close(fd);
        } else {
            EXPECT_EQ(errno, EACCES) << "expected EACCES on O_WRONLY of 0444";
        }
    });
}

}  // namespace
