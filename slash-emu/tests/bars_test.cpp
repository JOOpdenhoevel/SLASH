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
 * @file bars_test.cpp
 * @brief Unit + integration tests for the /<BDF>/bars/ endpoint (T7).
 *
 * Two layers, matching the build rules (CMake+CTest, GTest, scratch under .tmp):
 *
 *   - Unit, against slash_emu_core directly (no FUSE mount): the pure access
 *     validation matrix (emu_bar_check_access), and the end-to-end register
 *     round-trip + geometry through the spine (emu_bars_attach -> emu_node_stat /
 *     emu_node_pread / emu_node_pwrite), including the -ENODEV liveness gate
 *     after a forced revoke (reachable here before T9 wires revoke to a
 *     mount-visible trigger).
 *
 *   - Integration, over the real FUSE mount (fork+exec the freshly-built daemon,
 *     the info_test.cpp pattern): open /<BDF>/bars/bar{0,2,4}, round-trip
 *     register writes/reads, assert the file sizes, reject misaligned/bad-width
 *     and beyond-BAR transfers, and assert mmap is NOT offered.
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

extern "C" {
#include "bars.h"
#include "node.h"
#include "slash/uapi/slash_abi.h"
}

namespace {

// ===========================================================================
// Unit: pure access validation matrix (emu_bar_check_access)
// ===========================================================================

TEST(BarCheckAccess, AllValidWidthsAlignedAccepted)
{
    const uint64_t bar = SLASH_BAR_CLK_SIZE;  // 512 KiB
    for (size_t w : {1u, 2u, 4u, 8u}) {
        // Aligned at 0 and at a width-multiple offset deep in the BAR.
        EXPECT_EQ(emu_bar_check_access(0, w, bar), 0) << "width " << w;
        EXPECT_EQ(emu_bar_check_access(static_cast<off_t>(8 * w), w, bar), 0)
            << "width " << w;
    }
}

TEST(BarCheckAccess, BadWidthsRejected)
{
    const uint64_t bar = SLASH_BAR_CLK_SIZE;
    for (size_t w : {0u, 3u, 5u, 6u, 7u, 9u, 16u, 1024u}) {
        EXPECT_EQ(emu_bar_check_access(0, w, bar), -EINVAL) << "width " << w;
    }
}

TEST(BarCheckAccess, MisalignedOffsetsRejected)
{
    const uint64_t bar = SLASH_BAR_CLK_SIZE;
    // width 2 at odd offset; width 4 at offset 2; width 8 at offset 4.
    EXPECT_EQ(emu_bar_check_access(1, 2, bar), -EINVAL);
    EXPECT_EQ(emu_bar_check_access(2, 4, bar), -EINVAL);
    EXPECT_EQ(emu_bar_check_access(4, 8, bar), -EINVAL);
    EXPECT_EQ(emu_bar_check_access(3, 1, bar), 0);  // width 1 is always aligned
}

TEST(BarCheckAccess, NegativeOffsetRejected)
{
    EXPECT_EQ(emu_bar_check_access(-1, 1, SLASH_BAR_CLK_SIZE), -EINVAL);
    EXPECT_EQ(emu_bar_check_access(-8, 8, SLASH_BAR_CLK_SIZE), -EINVAL);
}

TEST(BarCheckAccess, BeyondBarRejected)
{
    const uint64_t bar = SLASH_BAR_CLK_SIZE;
    // Last valid 8-byte read starts at bar - 8.
    EXPECT_EQ(emu_bar_check_access(static_cast<off_t>(bar - 8), 8, bar), 0);
    // A transfer that straddles the end is invalid, not clamped.
    EXPECT_EQ(emu_bar_check_access(static_cast<off_t>(bar - 4), 8, bar),
              -EINVAL);
    // A transfer starting exactly at the end.
    EXPECT_EQ(emu_bar_check_access(static_cast<off_t>(bar), 1, bar), -EINVAL);
    // The last valid single byte.
    EXPECT_EQ(emu_bar_check_access(static_cast<off_t>(bar - 1), 1, bar), 0);
}

// ===========================================================================
// Unit: round-trip + geometry through the spine
// ===========================================================================

// RAII tree wrapper (mirrors node_test.cpp / info_test.cpp).
class Tree {
public:
    Tree() { EXPECT_EQ(emu_node_tree_new(&tree_, nullptr), 0); }
    ~Tree() { cleanup_node_tree(tree_); }
    emu_node_tree *get() { return tree_; }

private:
    emu_node_tree *tree_ = nullptr;
};

// Resolve a bars/<name> inode under a device.
emu_ino_t bar_ino(emu_node_tree *tree, emu_device *dev, const char *name)
{
    emu_node *child = nullptr;
    EXPECT_EQ(emu_node_lookup_child(tree, dev->bars->ino, name, &child), 0);
    return child != nullptr ? child->ino : 0;
}

struct BarSpec {
    const char *name;
    uint64_t size;
};

constexpr BarSpec kBars[] = {
    { "bar0", SLASH_BAR_USER_SIZE },
    { "bar2", SLASH_BAR_SL_SIZE },
    { "bar4", SLASH_BAR_CLK_SIZE },
};

TEST(BarsEndpoint, AttachCreatesExactlyThreeBars)
{
    Tree t;
    emu_device *dev = nullptr;
    ASSERT_EQ(emu_node_tree_add_device(t.get(), "0000:61:00", &dev), 0);
    ASSERT_EQ(emu_bars_attach(dev), 0);

    // bar0, bar2, bar4 resolve; bar1/bar3/bar5 do not.
    for (const auto &b : kBars) {
        emu_node *child = nullptr;
        EXPECT_EQ(emu_node_lookup_child(t.get(), dev->bars->ino, b.name, &child),
                  0)
            << b.name;
    }
    for (const char *absent : {"bar1", "bar3", "bar5"}) {
        emu_node *child = nullptr;
        EXPECT_EQ(emu_node_lookup_child(t.get(), dev->bars->ino, absent, &child),
                  -ENOENT)
            << absent;
    }
}

TEST(BarsEndpoint, GetattrReportsBarSize)
{
    Tree t;
    emu_device *dev = nullptr;
    ASSERT_EQ(emu_node_tree_add_device(t.get(), "0000:61:00", &dev), 0);
    ASSERT_EQ(emu_bars_attach(dev), 0);

    for (const auto &b : kBars) {
        emu_ino_t ino = bar_ino(t.get(), dev, b.name);
        ASSERT_NE(ino, 0u) << b.name;
        struct stat st {};
        ASSERT_EQ(emu_node_stat(t.get(), ino, &st), 0) << b.name;
        EXPECT_TRUE(S_ISREG(st.st_mode)) << b.name;
        EXPECT_EQ(st.st_size, static_cast<off_t>(b.size)) << b.name;
    }
}

TEST(BarsEndpoint, RoundTripWriteThenReadAtVariousOffsets)
{
    Tree t;
    emu_device *dev = nullptr;
    ASSERT_EQ(emu_node_tree_add_device(t.get(), "0000:61:00", &dev), 0);
    ASSERT_EQ(emu_bars_attach(dev), 0);
    emu_ino_t ino = bar_ino(t.get(), dev, "bar0");
    ASSERT_NE(ino, 0u);

    struct Case {
        off_t off;
        size_t width;
        uint64_t value;
    };
    const Case cases[] = {
        { 0, 1, 0xABull },
        { 2, 2, 0xBEEFull },
        { 8, 4, 0xDEADBEEFull },
        { 16, 8, 0x0123456789ABCDEFull },
        { static_cast<off_t>(SLASH_BAR_USER_SIZE - 8), 8, 0xFEEDFACECAFEB00Eull },
    };

    for (const auto &c : cases) {
        uint64_t in = c.value;
        ASSERT_EQ(emu_node_pwrite(t.get(), ino,
                                  reinterpret_cast<const char *>(&in), c.width,
                                  c.off),
                  static_cast<ssize_t>(c.width))
            << "off " << c.off << " width " << c.width;

        uint64_t out = 0;
        ASSERT_EQ(emu_node_pread(t.get(), ino, reinterpret_cast<char *>(&out),
                                 c.width, c.off),
                  static_cast<ssize_t>(c.width))
            << "off " << c.off << " width " << c.width;
        // Compare only the low `width` bytes.
        uint64_t mask = c.width == 8 ? ~0ull : ((1ull << (8 * c.width)) - 1);
        EXPECT_EQ(out & mask, c.value & mask)
            << "off " << c.off << " width " << c.width;
    }
}

TEST(BarsEndpoint, ReadOfUnwrittenIsZero)
{
    Tree t;
    emu_device *dev = nullptr;
    ASSERT_EQ(emu_node_tree_add_device(t.get(), "0000:61:00", &dev), 0);
    ASSERT_EQ(emu_bars_attach(dev), 0);
    emu_ino_t ino = bar_ino(t.get(), dev, "bar4");
    ASSERT_NE(ino, 0u);

    // No write has touched this BAR; every aligned read returns zero, never EIO.
    for (off_t off : {static_cast<off_t>(0), static_cast<off_t>(4096),
                      static_cast<off_t>(SLASH_BAR_CLK_SIZE - 8)}) {
        uint64_t out = 0xdeadbeef;
        ASSERT_EQ(emu_node_pread(t.get(), ino, reinterpret_cast<char *>(&out), 8,
                                 off),
                  8)
            << "off " << off;
        EXPECT_EQ(out, 0u) << "off " << off;
    }
}

TEST(BarsEndpoint, ValidationMatrixThroughSpine)
{
    Tree t;
    emu_device *dev = nullptr;
    ASSERT_EQ(emu_node_tree_add_device(t.get(), "0000:61:00", &dev), 0);
    ASSERT_EQ(emu_bars_attach(dev), 0);
    emu_ino_t ino = bar_ino(t.get(), dev, "bar0");
    ASSERT_NE(ino, 0u);

    char buf[8] = {};

    // Valid widths aligned: accepted (both read and write).
    for (size_t w : {1u, 2u, 4u, 8u}) {
        EXPECT_EQ(emu_node_pread(t.get(), ino, buf, w, 0),
                  static_cast<ssize_t>(w));
        EXPECT_EQ(emu_node_pwrite(t.get(), ino, buf, w, 0),
                  static_cast<ssize_t>(w));
    }

    // Bad widths: rejected.
    for (size_t w : {0u, 3u, 5u, 6u, 7u}) {
        EXPECT_EQ(emu_node_pread(t.get(), ino, buf, w, 0), -EINVAL)
            << "read width " << w;
        EXPECT_EQ(emu_node_pwrite(t.get(), ino, buf, w, 0), -EINVAL)
            << "write width " << w;
    }

    // Misaligned offsets: rejected.
    EXPECT_EQ(emu_node_pread(t.get(), ino, buf, 4, 2), -EINVAL);
    EXPECT_EQ(emu_node_pwrite(t.get(), ino, buf, 4, 2), -EINVAL);

    // Beyond BAR: rejected.
    EXPECT_EQ(emu_node_pread(t.get(), ino, buf, 8,
                             static_cast<off_t>(SLASH_BAR_USER_SIZE - 4)),
              -EINVAL);
    EXPECT_EQ(emu_node_pwrite(t.get(), ino, buf, 8,
                              static_cast<off_t>(SLASH_BAR_USER_SIZE - 4)),
              -EINVAL);
}

TEST(BarsEndpoint, PerDeviceIsolation)
{
    Tree t;
    emu_device *d1 = nullptr;
    emu_device *d2 = nullptr;
    ASSERT_EQ(emu_node_tree_add_device(t.get(), "0000:61:00", &d1), 0);
    ASSERT_EQ(emu_node_tree_add_device(t.get(), "0000:62:00", &d2), 0);
    ASSERT_EQ(emu_bars_attach(d1), 0);
    ASSERT_EQ(emu_bars_attach(d2), 0);

    emu_ino_t i1 = bar_ino(t.get(), d1, "bar0");
    emu_ino_t i2 = bar_ino(t.get(), d2, "bar0");
    ASSERT_NE(i1, 0u);
    ASSERT_NE(i2, 0u);
    ASSERT_NE(i1, i2);

    uint32_t v = 0xCAFEBABE;
    ASSERT_EQ(emu_node_pwrite(t.get(), i1, reinterpret_cast<const char *>(&v), 4,
                              64),
              4);

    // Device 2's BAR0 at the same offset is untouched (still zero).
    uint32_t got = 0xffffffff;
    ASSERT_EQ(
        emu_node_pread(t.get(), i2, reinterpret_cast<char *>(&got), 4, 64), 4);
    EXPECT_EQ(got, 0u);
}

TEST(BarsEndpoint, AccessAfterRevokeIsEnodev)
{
    Tree t;
    emu_device *dev = nullptr;
    ASSERT_EQ(emu_node_tree_add_device(t.get(), "0000:61:00", &dev), 0);
    ASSERT_EQ(emu_bars_attach(dev), 0);
    emu_ino_t ino = bar_ino(t.get(), dev, "bar0");  // bumps lookup_count
    ASSERT_NE(ino, 0u);

    // The lookup above holds the node alive as a dead orphan across revoke.
    ASSERT_EQ(emu_device_revoke(t.get(), "0000:61:00"), 0);

    char buf[8] = {};
    EXPECT_EQ(emu_node_pread(t.get(), ino, buf, 4, 0), -ENODEV);
    EXPECT_EQ(emu_node_pwrite(t.get(), ino, buf, 4, 0), -ENODEV);
}

TEST(BarsEndpoint, WriteToReadOnlyNodeIsEio)
{
    // A node with no write hook (the spine's read-only default) yields -EIO on
    // pwrite: confirms the write dispatch mirrors the read dispatch.
    Tree t;
    emu_device *dev = nullptr;
    ASSERT_EQ(emu_node_tree_add_device(t.get(), "0000:61:00", &dev), 0);

    emu_node *node = nullptr;
    ASSERT_EQ(emu_node_create_child(t.get(), dev->dir, "plain", EMU_NODE_FILE,
                                    0444, nullptr, nullptr, &node),
              0);
    char buf[4] = {};
    EXPECT_EQ(emu_node_pwrite(t.get(), node->ino, buf, 4, 0), -EIO);
}

// ===========================================================================
// Integration: register access over a real FUSE mount
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
        std::string(SLASH_EMU_TMP_DIR) + "/slash_emu_barscfg_XXXXXX";
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
    const std::string mountpoint = make_scratch("bars");
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

TEST(BarsMount, FileSizesMatchGeometry)
{
    with_mounted_daemon([](const std::string &mountpoint) {
        const std::string base = mountpoint + "/0000:61:00/bars/";
        struct {
            const char *name;
            off_t size;
        } expect[] = {
            { "bar0", static_cast<off_t>(SLASH_BAR_USER_SIZE) },
            { "bar2", static_cast<off_t>(SLASH_BAR_SL_SIZE) },
            { "bar4", static_cast<off_t>(SLASH_BAR_CLK_SIZE) },
        };
        for (const auto &e : expect) {
            struct stat st {};
            ASSERT_EQ(::stat((base + e.name).c_str(), &st), 0)
                << e.name << ": " << std::strerror(errno);
            EXPECT_TRUE(S_ISREG(st.st_mode)) << e.name;
            EXPECT_EQ(st.st_size, e.size) << e.name;
        }
        // The absent BARs do not exist.
        for (const char *absent : {"bar1", "bar3", "bar5"}) {
            struct stat st {};
            EXPECT_NE(::stat((base + absent).c_str(), &st), 0) << absent;
            EXPECT_EQ(errno, ENOENT) << absent;
        }
    });
}

TEST(BarsMount, RegisterRoundTripOnEachBar)
{
    with_mounted_daemon([](const std::string &mountpoint) {
        const std::string base = mountpoint + "/0000:61:00/bars/";
        for (const char *name : {"bar0", "bar2", "bar4"}) {
            int fd = ::open((base + name).c_str(), O_RDWR);
            ASSERT_GE(fd, 0) << name << ": " << std::strerror(errno);

            // 4-byte register poke + read-back.
            uint32_t w = 0xDEADBEEF;
            ASSERT_EQ(::pwrite(fd, &w, sizeof(w), 16),
                      static_cast<ssize_t>(sizeof(w)))
                << name << ": " << std::strerror(errno);
            uint32_t r = 0;
            ASSERT_EQ(::pread(fd, &r, sizeof(r), 16),
                      static_cast<ssize_t>(sizeof(r)))
                << name << ": " << std::strerror(errno);
            EXPECT_EQ(r, w) << name;

            // An unwritten location reads back zero.
            uint64_t z = 0xff;
            ASSERT_EQ(::pread(fd, &z, sizeof(z), 1024),
                      static_cast<ssize_t>(sizeof(z)))
                << name;
            EXPECT_EQ(z, 0u) << name;

            ::close(fd);
        }
    });
}

TEST(BarsMount, BadWidthAndAlignmentRejected)
{
    with_mounted_daemon([](const std::string &mountpoint) {
        const std::string path = mountpoint + "/0000:61:00/bars/bar0";
        int fd = ::open(path.c_str(), O_RDWR);
        ASSERT_GE(fd, 0) << std::strerror(errno);

        char buf[16] = {};

        // 3-byte transfer: bad width.
        errno = 0;
        EXPECT_EQ(::pwrite(fd, buf, 3, 0), -1);
        EXPECT_EQ(errno, EINVAL);
        errno = 0;
        EXPECT_EQ(::pread(fd, buf, 3, 0), -1);
        EXPECT_EQ(errno, EINVAL);

        // 4-byte transfer at offset 2: misaligned.
        errno = 0;
        EXPECT_EQ(::pwrite(fd, buf, 4, 2), -1);
        EXPECT_EQ(errno, EINVAL);
        errno = 0;
        EXPECT_EQ(::pread(fd, buf, 4, 2), -1);
        EXPECT_EQ(errno, EINVAL);

        ::close(fd);
    });
}

TEST(BarsMount, BeyondBarRejected)
{
    with_mounted_daemon([](const std::string &mountpoint) {
        const std::string path = mountpoint + "/0000:61:00/bars/bar4";
        int fd = ::open(path.c_str(), O_RDWR);
        ASSERT_GE(fd, 0) << std::strerror(errno);

        char buf[8] = {};
        // 8-byte read straddling the end of the 512 KiB BAR is rejected.
        errno = 0;
        EXPECT_EQ(::pread(fd, buf, 8, static_cast<off_t>(SLASH_BAR_CLK_SIZE - 4)),
                  -1);
        EXPECT_EQ(errno, EINVAL);

        // The last valid 8-byte read succeeds.
        EXPECT_EQ(::pread(fd, buf, 8, static_cast<off_t>(SLASH_BAR_CLK_SIZE - 8)),
                  8);

        ::close(fd);
    });
}

// A SHARED mapping -- the only kind that could ever be a coherent register
// window the daemon would have to keep in sync and zap on revocation -- is
// rejected at mmap() time.  With FOPEN_DIRECT_IO the kernel cannot provide
// MAP_SHARED coherency and returns -ENODEV (fs/fuse/file.c fuse_file_mmap),
// for both writable and read-only SHARED maps.
TEST(BarsMount, MmapSharedIsRejected)
{
    with_mounted_daemon([](const std::string &mountpoint) {
        const std::string path = mountpoint + "/0000:61:00/bars/bar0";
        int fd = ::open(path.c_str(), O_RDWR);
        ASSERT_GE(fd, 0) << std::strerror(errno);

        errno = 0;
        void *prw = ::mmap(nullptr, 4096, PROT_READ | PROT_WRITE, MAP_SHARED, fd,
                           0);
        EXPECT_EQ(prw, MAP_FAILED);
        EXPECT_EQ(errno, ENODEV);
        if (prw != MAP_FAILED) {
            ::munmap(prw, 4096);
        }

        errno = 0;
        void *pr = ::mmap(nullptr, 4096, PROT_READ, MAP_SHARED, fd, 0);
        EXPECT_EQ(pr, MAP_FAILED);
        EXPECT_EQ(errno, ENODEV);
        if (pr != MAP_FAILED) {
            ::munmap(pr, 4096);
        }

        ::close(fd);
    });
}

// The "no mmap" guarantee, defect-2 arm: a MAP_PRIVATE mapping is a kernel-side
// page-cache COW snapshot that no userspace FUSE op set can refuse at mmap()
// time (with FOPEN_DIRECT_IO the kernel routes MAP_PRIVATE through
// generic_file_mmap; there is no low-level .mmap hook to veto it).  What the
// daemon MUST guarantee is that touching such a mapping never HANGS the client:
// the first page-fault issues a page-sized FUSE_READ, which the BAR read hook
// rejects (size is not a {1,2,4,8} register width) and the daemon answers
// PROMPTLY with -EINVAL, so the fault resolves to a prompt SIGBUS rather than
// blocking forever.  We verify this by dereferencing in a CHILD under a
// watchdog: the child must terminate quickly (by SIGBUS, or cleanly if a future
// kernel rejects the map outright) -- it must never be killed by the watchdog
// for hanging.
TEST(BarsMount, MmapPrivateDerefFaultsPromptlyNeverHangs)
{
    with_mounted_daemon([](const std::string &mountpoint) {
        const std::string path = mountpoint + "/0000:61:00/bars/bar0";

        pid_t child = ::fork();
        ASSERT_NE(child, -1) << "fork failed: " << std::strerror(errno);
        if (child == 0) {
            // Child: map MAP_PRIVATE and touch the first page.  Either the map
            // is refused (clean exit 0) or the deref faults (SIGBUS).  Either
            // way we must reach a terminal state fast; we never loop/block.
            //
            // Restore the default SIGBUS disposition first: under the ASan/UBSan
            // build the sanitizer installs its own SIGBUS handler that prints a
            // DEADLYSIGNAL report and _exit()s with a nonzero code, which would
            // otherwise mask the clean signal-death we are characterizing.  With
            // SIG_DFL the fault terminates the child by SIGBUS in every build.
            ::signal(SIGBUS, SIG_DFL);
            int fd = ::open(path.c_str(), O_RDONLY);
            if (fd < 0) {
                ::_exit(10);
            }
            void *p = ::mmap(nullptr, 4096, PROT_READ, MAP_PRIVATE, fd, 0);
            if (p == MAP_FAILED) {
                ::_exit(0);  // map refused outright -- the ideal end state.
            }
            volatile char c = *static_cast<volatile char *>(p);
            (void) c;        // deref returned without faulting (e.g. served 0s).
            ::_exit(0);
        }

        // Parent: watchdog.  The load-bearing assertion is that the child
        // reaches a terminal state PROMPTLY; a hang (watchdog must SIGKILL) is
        // the defect-2 failure and the only failure this test cares about.
        int status = 0;
        bool finished = wait_for(
            [&] { return ::waitpid(child, &status, WNOHANG) == child; }, 4000);
        if (!finished) {
            ::kill(child, SIGKILL);
            ::waitpid(child, &status, 0);
            FAIL() << "MAP_PRIVATE deref HUNG (watchdog had to SIGKILL the "
                      "child) -- a page-fault read was left unanswered";
        }

        // Any prompt terminal state is acceptable: a clean exit (map refused or
        // deref served zeros) or death by SIGBUS (the prompt fault, possibly via
        // the sanitizer's interposed handler under the ASan build).  We only
        // reject a missing/forced terminal state, already handled above.  When
        // the child died by a signal, require it to be SIGBUS specifically.
        SUCCEED() << "MAP_PRIVATE deref reached a terminal state promptly "
                     "(no hang)";
        if (WIFSIGNALED(status)) {
            EXPECT_EQ(WTERMSIG(status), SIGBUS)
                << "expected a prompt SIGBUS fault, got signal "
                << WTERMSIG(status);
        }
    });
}

}  // namespace
