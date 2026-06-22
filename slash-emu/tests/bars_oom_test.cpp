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
 * @file bars_oom_test.cpp
 * @brief Deterministic OOM-injection coverage for the BAR shadow allocation
 *        (T7 defect-1 re-verification).
 *
 * The bug: bar_write's first-write shadow calloc used PROPAGATE_ERROR_NULL_*
 * (C era), which returns -1 (== -EPERM) instead of a real errno; the fix
 * returns -ENOMEM.  That path is only taken on allocation failure, which is not
 * reachable through the public API.  This file injects the failure via a
 * link-time `--wrap=calloc` interposer (set up by tests/CMakeLists.txt for THIS
 * executable only) so the -ENOMEM arm is exercised for real and pinned.
 *
 * The interposer is deliberately narrow: it fails a calloc ONLY when a test has
 * armed it AND the requested size matches a BAR size (128 MiB / 512 KiB).  Every
 * other allocation (GoogleTest, the node tree, the BarOps backing structs) passes
 * straight through to __real_calloc, so arming the failure cannot perturb
 * unrelated machinery.
 */

#include <gtest/gtest.h>

#include <atomic>
#include <cerrno>
#include <cstddef>
#include <cstdint>

#include <sys/stat.h>

#include "bars.hpp"
#include "node.hpp"
#include "slash/uapi/slash_abi.h"

using namespace slash::emu;

extern "C" {

// --- calloc interposer (linked via -Wl,--wrap=calloc) ------------------------
// __real_calloc is provided by the linker; it is the genuine libc calloc.
void *__real_calloc(size_t nmemb, size_t size);

// Armed state: when g_fail_bar_calloc is true, a calloc whose total byte count
// equals one of the BAR sizes fails (returns NULL).  Narrow by design.
static std::atomic<bool> g_fail_bar_calloc{false};

void *__wrap_calloc(size_t nmemb, size_t size)
{
    if (g_fail_bar_calloc.load(std::memory_order_acquire)) {
        // BarOps::write calls calloc(1, bar_size); match the BAR sizes only.
        size_t total = nmemb * size;
        if (total == SLASH_BAR_USER_SIZE || total == SLASH_BAR_SL_SIZE ||
            total == SLASH_BAR_CLK_SIZE) {
            return nullptr;
        }
    }
    return __real_calloc(nmemb, size);
}

}  // extern "C"

namespace {

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

// A first write whose shadow allocation fails returns -ENOMEM (NOT -1/-EPERM),
// and leaves the BAR in a clean state: no shadow allocated, a subsequent read
// still returns defined zero, and a later write (allocation now succeeding)
// works -- i.e. the failure is transient and non-corrupting.
TEST(BarOom, FirstWriteShadowAllocFailureIsEnomem)
{
    Tree t;
    Device *dev = t.get().addDevice("0000:61:00");
    ASSERT_NE(dev, nullptr);
    ASSERT_EQ(barsAttach(*dev), 0);
    Ino ino = bar_ino(t.get(), dev, "bar4");
    ASSERT_NE(ino, 0u);

    char buf[8] = { 1, 2, 3, 4, 5, 6, 7, 8 };

    // Arm the failure: the first write's calloc(1, 512 KiB) returns NULL.
    g_fail_bar_calloc.store(true, std::memory_order_release);
    ssize_t rc = t.get().pwrite(ino, buf, 8, 0);
    g_fail_bar_calloc.store(false, std::memory_order_release);

    // The load-bearing assertion: -ENOMEM, never -1 (-EPERM) and never a
    // short or "successful" write.
    EXPECT_EQ(rc, -ENOMEM) << "shadow OOM must surface as -ENOMEM, got " << rc;

    // The BAR is uncorrupted: the failed write allocated nothing, so a read
    // still returns defined zero (the never-written value), never -EIO.
    uint64_t got = 0xdeadbeef;
    ASSERT_EQ(t.get().pread(ino, reinterpret_cast<char *>(&got), 8, 0), 8);
    EXPECT_EQ(got, 0u);

    // And the failure was transient: with calloc working again the same write
    // now succeeds and round-trips.
    ASSERT_EQ(t.get().pwrite(ino, buf, 8, 0), 8);
    got = 0;
    ASSERT_EQ(t.get().pread(ino, reinterpret_cast<char *>(&got), 8, 0), 8);
    EXPECT_EQ(got, 0x0807060504030201ull);
}

// Sanity: the interposer truly intercepts (a guard against the test silently
// no-op'ing if the --wrap flag is ever dropped).  Armed, a direct BAR-sized
// calloc returns NULL; a non-BAR-sized one still succeeds.
TEST(BarOom, InterposerIsActiveAndNarrow)
{
    g_fail_bar_calloc.store(true, std::memory_order_release);
    void *bar_sized = ::calloc(1, SLASH_BAR_CLK_SIZE);
    void *small = ::calloc(1, 64);
    g_fail_bar_calloc.store(false, std::memory_order_release);

    EXPECT_EQ(bar_sized, nullptr)
        << "--wrap=calloc not active: BAR-sized calloc was not intercepted";
    EXPECT_NE(small, nullptr) << "interposer too broad: failed a non-BAR calloc";
    ::free(small);
    ::free(bar_sized);  // free(NULL) is a no-op
}

}  // namespace
