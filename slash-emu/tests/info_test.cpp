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
 * @file info_test.cpp
 * @brief Unit + integration tests for the /<BDF>/info endpoint (T6).
 *
 * Two layers, matching the build rules (CMake+CTest, GTest, scratch under .tmp):
 *
 *   - Unit, against slash_emu_core directly (no FUSE mount): the pure pread/
 *     offset math (emu_info_pread_buf) and the end-to-end struct population +
 *     read dispatch through the spine (emu_info_attach -> emu_node_stat /
 *     emu_node_pread), including the -ENODEV liveness gate after a forced
 *     revoke (the read path's revocation contract, reachable here before T9
 *     wires revoke to a mount-visible trigger).
 *
 *   - Integration, over the real FUSE mount (fork+exec the freshly-built
 *     daemon, the smoke_test.cpp pattern): open /<BDF>/info and assert the
 *     struct contents, a full read, a short read, and an offset read.
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
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/vfs.h>
#include <sys/wait.h>
#include <unistd.h>

extern "C" {
#include "info.h"
#include "node.h"
#include "slash/uapi/slash_abi.h"
}

namespace {

// ===========================================================================
// Unit: pure pread/offset math (emu_info_pread_buf)
// ===========================================================================

TEST(InfoPreadBuf, FullReadCopiesEverything)
{
    const char src[] = {1, 2, 3, 4, 5, 6, 7, 8};
    char dst[sizeof(src)] = {};

    ssize_t n = emu_info_pread_buf(dst, sizeof(dst), 0, src, sizeof(src));
    ASSERT_EQ(n, static_cast<ssize_t>(sizeof(src)));
    EXPECT_EQ(std::memcmp(dst, src, sizeof(src)), 0);
}

TEST(InfoPreadBuf, ShortReadClampsToAvailable)
{
    const char src[] = {1, 2, 3, 4, 5, 6, 7, 8};
    char dst[64] = {};

    // Ask for more than exists: get exactly src_len bytes (a short read).
    ssize_t n = emu_info_pread_buf(dst, sizeof(dst), 0, src, sizeof(src));
    ASSERT_EQ(n, static_cast<ssize_t>(sizeof(src)));
    EXPECT_EQ(std::memcmp(dst, src, sizeof(src)), 0);
}

TEST(InfoPreadBuf, OffsetReadReturnsTail)
{
    const char src[] = {10, 11, 12, 13, 14, 15};
    char dst[64] = {};

    ssize_t n = emu_info_pread_buf(dst, sizeof(dst), 2, src, sizeof(src));
    ASSERT_EQ(n, 4);
    EXPECT_EQ(dst[0], 12);
    EXPECT_EQ(dst[3], 15);
}

TEST(InfoPreadBuf, OffsetPlusSizeStraddlingEofIsShort)
{
    const char src[] = {0, 1, 2, 3, 4, 5, 6, 7};
    char dst[64] = {};

    // off=6, want 100 bytes, only 2 remain.
    ssize_t n = emu_info_pread_buf(dst, 100, 6, src, sizeof(src));
    ASSERT_EQ(n, 2);
    EXPECT_EQ(dst[0], 6);
    EXPECT_EQ(dst[1], 7);
}

TEST(InfoPreadBuf, ReadAtEofReturnsZero)
{
    const char src[] = {1, 2, 3};
    char dst[8] = {};
    EXPECT_EQ(emu_info_pread_buf(dst, sizeof(dst), 3, src, sizeof(src)), 0);
}

TEST(InfoPreadBuf, ReadPastEofReturnsZero)
{
    const char src[] = {1, 2, 3};
    char dst[8] = {};
    EXPECT_EQ(emu_info_pread_buf(dst, sizeof(dst), 100, src, sizeof(src)), 0);
}

TEST(InfoPreadBuf, ZeroSizeReturnsZero)
{
    const char src[] = {1, 2, 3};
    char dst[8] = {};
    EXPECT_EQ(emu_info_pread_buf(dst, 0, 0, src, sizeof(src)), 0);
}

TEST(InfoPreadBuf, NegativeOffsetIsEinval)
{
    const char src[] = {1, 2, 3};
    char dst[8] = {};
    EXPECT_EQ(emu_info_pread_buf(dst, sizeof(dst), -1, src, sizeof(src)),
              -EINVAL);
}

// ===========================================================================
// Unit: struct population + read dispatch through the spine
// ===========================================================================

// RAII tree wrapper (mirrors node_test.cpp).
class Tree {
public:
    Tree() { EXPECT_EQ(emu_node_tree_new(&tree_, nullptr), 0); }
    ~Tree() { cleanup_node_tree(tree_); }
    emu_node_tree *get() { return tree_; }

private:
    emu_node_tree *tree_ = nullptr;
};

// Resolve the info node's inode under a device (its dir's only child).
emu_ino_t info_ino(emu_node_tree *tree, emu_ino_t parent)
{
    emu_node *child = nullptr;
    EXPECT_EQ(emu_node_lookup_child(tree, parent, "info", &child), 0);
    return child != nullptr ? child->ino : 0;
}

TEST(InfoEndpoint, AttachPopulatesStruct)
{
    Tree t;
    emu_device *dev = nullptr;
    ASSERT_EQ(emu_node_tree_add_device(t.get(), "0000:61:00", &dev), 0);
    ASSERT_EQ(emu_info_attach(dev), 0);

    emu_ino_t ino = info_ino(t.get(), dev->dir->ino);
    ASSERT_NE(ino, 0u);

    // getattr reports exactly sizeof(struct slash_info).
    struct stat st {};
    ASSERT_EQ(emu_node_stat(t.get(), ino, &st), 0);
    EXPECT_TRUE(S_ISREG(st.st_mode));
    EXPECT_EQ(st.st_size, static_cast<off_t>(sizeof(struct slash_info)));

    // A full read yields a struct with the expected fields.
    struct slash_info got {};
    ssize_t n = emu_node_pread(t.get(), ino, reinterpret_cast<char *>(&got),
                               sizeof(got), 0);
    ASSERT_EQ(n, static_cast<ssize_t>(sizeof(struct slash_info)));
    EXPECT_EQ(got.size, sizeof(struct slash_info));
    EXPECT_EQ(got.acc_type, SLASH_ACC_TYPE_SYSTEM_EMULATED);
    EXPECT_STREQ(got.bdf, "0000:61:00");
}

TEST(InfoEndpoint, PreadOffsetAndShortReadThroughSpine)
{
    Tree t;
    emu_device *dev = nullptr;
    ASSERT_EQ(emu_node_tree_add_device(t.get(), "0000:62:00", &dev), 0);
    ASSERT_EQ(emu_info_attach(dev), 0);
    emu_ino_t ino = info_ino(t.get(), dev->dir->ino);
    ASSERT_NE(ino, 0u);

    // Offset read: the bytes at offset 4 are the start of acc_type (u32 after
    // the u32 size), little-endian on the test host.
    uint32_t acc = 0;
    ssize_t n = emu_node_pread(t.get(), ino, reinterpret_cast<char *>(&acc),
                               sizeof(acc), offsetof(struct slash_info, acc_type));
    ASSERT_EQ(n, static_cast<ssize_t>(sizeof(acc)));
    EXPECT_EQ(acc, SLASH_ACC_TYPE_SYSTEM_EMULATED);

    // Short read: only the leading size word.
    uint32_t sz = 0;
    n = emu_node_pread(t.get(), ino, reinterpret_cast<char *>(&sz), sizeof(sz),
                       0);
    ASSERT_EQ(n, static_cast<ssize_t>(sizeof(sz)));
    EXPECT_EQ(sz, sizeof(struct slash_info));

    // Read at EOF: zero bytes.
    char b = 0;
    EXPECT_EQ(emu_node_pread(t.get(), ino, &b, 1,
                             static_cast<off_t>(sizeof(struct slash_info))),
              0);
}

TEST(InfoEndpoint, ReadAfterRevokeIsEnodev)
{
    Tree t;
    emu_device *dev = nullptr;
    ASSERT_EQ(emu_node_tree_add_device(t.get(), "0000:61:00", &dev), 0);
    ASSERT_EQ(emu_info_attach(dev), 0);
    emu_ino_t ino = info_ino(t.get(), dev->dir->ino);
    ASSERT_NE(ino, 0u);

    // Hold a kernel lookup so the node survives revoke as a dead orphan (the
    // "op on an already-open fd of a removed endpoint" case): info_ino above
    // already bumped lookup_count via emu_node_lookup_child.
    ASSERT_EQ(emu_device_revoke(t.get(), "0000:61:00"), 0);

    struct slash_info got {};
    EXPECT_EQ(emu_node_pread(t.get(), ino, reinterpret_cast<char *>(&got),
                             sizeof(got), 0),
              -ENODEV);
}

// ===========================================================================
// Integration: read /<BDF>/info over a real FUSE mount
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
        std::string(SLASH_EMU_TMP_DIR) + "/slash_emu_infocfg_XXXXXX";
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
    const std::string mountpoint = make_scratch("info");
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

TEST(InfoMount, ReadStructFullShortAndOffset)
{
    with_mounted_daemon([](const std::string &mountpoint) {
        const std::string path = mountpoint + "/0000:61:00/info";

        // The file stats as a read-only regular file of sizeof(slash_info).
        struct stat st {};
        ASSERT_EQ(::stat(path.c_str(), &st), 0)
            << "stat " << path << ": " << std::strerror(errno);
        EXPECT_TRUE(S_ISREG(st.st_mode));
        EXPECT_EQ(st.st_size, static_cast<off_t>(sizeof(struct slash_info)));

        int fd = ::open(path.c_str(), O_RDONLY);
        ASSERT_GE(fd, 0) << "open " << path << ": " << std::strerror(errno);

        // Full read: the whole struct, with the expected contents.
        struct slash_info info {};
        ssize_t n = ::pread(fd, &info, sizeof(info), 0);
        EXPECT_EQ(n, static_cast<ssize_t>(sizeof(info)));
        EXPECT_EQ(info.size, sizeof(struct slash_info));
        EXPECT_EQ(info.acc_type, SLASH_ACC_TYPE_SYSTEM_EMULATED);
        EXPECT_STREQ(info.bdf, "0000:61:00");

        // Short read: just the leading size word.
        uint32_t sz = 0;
        n = ::pread(fd, &sz, sizeof(sz), 0);
        EXPECT_EQ(n, static_cast<ssize_t>(sizeof(sz)));
        EXPECT_EQ(sz, sizeof(struct slash_info));

        // Offset read: the acc_type word at its offset.
        uint32_t acc = 0;
        n = ::pread(fd, &acc, sizeof(acc), offsetof(struct slash_info, acc_type));
        EXPECT_EQ(n, static_cast<ssize_t>(sizeof(acc)));
        EXPECT_EQ(acc, SLASH_ACC_TYPE_SYSTEM_EMULATED);

        // Read at EOF: zero bytes.
        char b = 0;
        n = ::pread(fd, &b, 1, static_cast<off_t>(sizeof(struct slash_info)));
        EXPECT_EQ(n, 0);

        ::close(fd);
    });
}

}  // namespace
