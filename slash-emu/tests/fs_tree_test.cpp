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
 * @file fs_tree_test.cpp
 * @brief End-to-end test that the spine materializes the configured device tree
 *        over the real FUSE mount.
 *
 * Launches the freshly-built daemon with a two-accelerator config and asserts
 * that, through the kernel, the mount shows one <BDF>/ directory per accelerator,
 * each containing bars/ and qdma/.  This exercises the node model wired into the
 * FUSE lookup / getattr / readdir ops (the data-structure logic itself is
 * unit-tested in node_test.cpp).  Follows smoke_test.cpp's fork+exec pattern.
 */

#include <gtest/gtest.h>

#include <cerrno>
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
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/vfs.h>
#include <sys/wait.h>
#include <unistd.h>

namespace {

constexpr int kMountTimeoutMs = 5000;
constexpr int kShutdownTimeoutMs = 5000;
constexpr int kPollIntervalMs = 50;

std::string make_scratch(const char *suffix)
{
    ::mkdir(SLASH_EMU_TMP_DIR, 0755);
    std::string tmpl = std::string(SLASH_EMU_TMP_DIR) + "/slash_emu_" + suffix +
                       "_XXXXXX";
    std::vector<char> buf(tmpl.begin(), tmpl.end());
    buf.push_back('\0');
    char *result = ::mkdtemp(buf.data());
    EXPECT_NE(result, nullptr) << "mkdtemp failed: " << std::strerror(errno);
    return result != nullptr ? std::string(result) : std::string();
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

std::set<std::string> list_dir(const std::string &path)
{
    std::set<std::string> names;
    DIR *dir = ::opendir(path.c_str());
    if (dir == nullptr) {
        return names;
    }
    struct dirent *ent;
    while ((ent = ::readdir(dir)) != nullptr) {
        std::string n = ent->d_name;
        if (n != "." && n != "..") {
            names.insert(n);
        }
    }
    ::closedir(dir);
    return names;
}

// Write a two-accelerator config to a throwaway file under .tmp.
std::string write_config()
{
    ::mkdir(SLASH_EMU_TMP_DIR, 0755);
    std::string tmpl = std::string(SLASH_EMU_TMP_DIR) + "/slash_emu_treecfg_XXXXXX";
    std::vector<char> buf(tmpl.begin(), tmpl.end());
    buf.push_back('\0');
    int fd = ::mkstemp(buf.data());
    EXPECT_GE(fd, 0) << "mkstemp failed: " << std::strerror(errno);

    const char *cfg =
        "[accelerator:0000:61:00]\n"
        "net-ip = 10.0.0.1\n"
        "[accelerator:0000:62:00]\n"
        "net-ip = 10.0.0.2\n";
    ssize_t n = ::write(fd, cfg, std::strlen(cfg));
    EXPECT_EQ(static_cast<size_t>(n), std::strlen(cfg));
    ::close(fd);
    return buf.data();
}

}  // namespace

TEST(SlashEmuFsTree, MaterializesConfiguredAccelerators)
{
    const std::string mountpoint = make_scratch("tree");
    ASSERT_FALSE(mountpoint.empty());
    const std::string config = write_config();

    pid_t pid = ::fork();
    ASSERT_NE(pid, -1) << "fork failed: " << std::strerror(errno);
    if (pid == 0) {
        ::execl(SLASH_EMUD_PATH, "slash-emud", "--config", config.c_str(),
                "--mount", mountpoint.c_str(), (char *) nullptr);
        ::_exit(127);
    }

    bool ready = wait_for([&] { return mount_is_ready(mountpoint); },
                          kMountTimeoutMs);
    if (!ready) {
        ::kill(pid, SIGKILL);
        int status = 0;
        ::waitpid(pid, &status, 0);
        ::rmdir(mountpoint.c_str());
        ::unlink(config.c_str());
        FAIL() << "Daemon did not mount within timeout at " << mountpoint;
    }

    // Root lists the two configured BDF directories plus the global hotplug
    // control file (a sibling of the per-device dirs).
    auto root = list_dir(mountpoint);
    EXPECT_EQ(root.size(), 3u);
    EXPECT_EQ(root.count("0000:61:00"), 1u);
    EXPECT_EQ(root.count("0000:62:00"), 1u);
    EXPECT_EQ(root.count("hotplug"), 1u);

    // Each <BDF>/ stats as a dir and contains bars/ + qdma/.
    for (const std::string &bdf : {"0000:61:00", "0000:62:00"}) {
        std::string devdir = mountpoint + "/" + bdf;
        struct stat st{};
        EXPECT_EQ(::stat(devdir.c_str(), &st), 0)
            << "stat " << devdir << ": " << std::strerror(errno);
        EXPECT_TRUE(S_ISDIR(st.st_mode));

        auto kids = list_dir(devdir);
        EXPECT_EQ(kids.count("bars"), 1u) << "missing bars/ in " << bdf;
        EXPECT_EQ(kids.count("qdma"), 1u) << "missing qdma/ in " << bdf;

        // bars/ and qdma/ are directories.
        struct stat bst{};
        EXPECT_EQ(::stat((devdir + "/bars").c_str(), &bst), 0);
        EXPECT_TRUE(S_ISDIR(bst.st_mode));
        struct stat qst{};
        EXPECT_EQ(::stat((devdir + "/qdma").c_str(), &qst), 0);
        EXPECT_TRUE(S_ISDIR(qst.st_mode));
    }

    // A nonexistent BDF lookup fails with ENOENT.
    struct stat gst{};
    EXPECT_NE(::stat((mountpoint + "/0000:99:00").c_str(), &gst), 0);
    EXPECT_EQ(errno, ENOENT);

    ASSERT_EQ(::kill(pid, SIGTERM), 0) << "kill failed: " << std::strerror(errno);
    int status = 0;
    bool exited = wait_for(
        [&] { return ::waitpid(pid, &status, WNOHANG) == pid; },
        kShutdownTimeoutMs);
    if (!exited) {
        ::kill(pid, SIGKILL);
        ::waitpid(pid, &status, 0);
    }
    EXPECT_TRUE(exited);
    EXPECT_TRUE(WIFEXITED(status));
    if (WIFEXITED(status)) {
        EXPECT_EQ(WEXITSTATUS(status), 0) << "daemon exited non-zero";
    }

    ::rmdir(mountpoint.c_str());
    ::unlink(config.c_str());
}
