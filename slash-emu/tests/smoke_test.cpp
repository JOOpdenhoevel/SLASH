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
 * @file smoke_test.cpp
 * @brief End-to-end smoke test for the slash-emud FUSE scaffold.
 *
 * Launches the freshly-built daemon as a child process, mounting an empty root
 * filesystem under a temp directory in the repo's .tmp scratch area.  It then
 * verifies the mount is browsable (stat + readdir of the root) and shuts the
 * daemon down with SIGTERM, asserting a clean exit and that the mountpoint is
 * empty again afterwards.
 *
 * The daemon path and scratch dir are injected as compile definitions by CMake
 * (SLASH_EMUD_PATH / SLASH_EMU_TMP_DIR) so the test never hand-runs or guesses
 * a binary location.
 */

#include <gtest/gtest.h>

#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <set>
#include <string>

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

// Create a unique mountpoint directory under the repo .tmp scratch area.
std::string make_mountpoint()
{
    std::string tmpl = std::string(SLASH_EMU_TMP_DIR) + "/slash_emu_smoke_XXXXXX";
    std::vector<char> buf(tmpl.begin(), tmpl.end());
    buf.push_back('\0');

    // Ensure .tmp exists (mkdtemp requires the parent).
    ::mkdir(SLASH_EMU_TMP_DIR, 0755);

    char *result = ::mkdtemp(buf.data());
    EXPECT_NE(result, nullptr) << "mkdtemp failed: " << std::strerror(errno);
    return result != nullptr ? std::string(result) : std::string();
}

// Poll a predicate until it returns true or the timeout elapses.
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

// True once the mountpoint is actually a FUSE filesystem (not the bare temp
// dir mkdtemp created). statfs's f_type reliably distinguishes the two; merely
// stat-ing or opendir-ing the path would also succeed on the unmounted temp
// dir, racing the daemon's signal setup.
bool mount_is_ready(const std::string &mountpoint)
{
    struct statfs sfs {};
    if (::statfs(mountpoint.c_str(), &sfs) != 0) {
        return false;
    }
    return sfs.f_type == FUSE_SUPER_MAGIC;
}

}  // namespace

TEST(SlashEmuSmoke, MountBrowseAndUnmount)
{
    const std::string mountpoint = make_mountpoint();
    ASSERT_FALSE(mountpoint.empty());

    pid_t pid = ::fork();
    ASSERT_NE(pid, -1) << "fork failed: " << std::strerror(errno);

    if (pid == 0) {
        // Child: exec the daemon mounting our temp dir.
        ::execl(SLASH_EMUD_PATH, "slash-emud", "--mount", mountpoint.c_str(),
                (char *) nullptr);
        // Only reached on exec failure.
        ::_exit(127);
    }

    // Parent: wait for the mount to come up.
    bool ready = wait_for([&] { return mount_is_ready(mountpoint); },
                          kMountTimeoutMs);

    if (!ready) {
        // Best-effort teardown before failing so we don't leak the child/mount.
        ::kill(pid, SIGKILL);
        int status = 0;
        ::waitpid(pid, &status, 0);
        ::rmdir(mountpoint.c_str());
        FAIL() << "Daemon did not mount within timeout at " << mountpoint;
    }

    // The root must stat as a directory.
    struct stat st {};
    EXPECT_EQ(::stat(mountpoint.c_str(), &st), 0)
        << "stat root failed: " << std::strerror(errno);
    EXPECT_TRUE(S_ISDIR(st.st_mode));

    // readdir must succeed.  With no --config there are no per-device <BDF>/
    // dirs, but the global "hotplug" control file is always present at root (a
    // sibling of the per-device dirs), so the only entries are it plus "."/"..".
    DIR *dir = ::opendir(mountpoint.c_str());
    ASSERT_NE(dir, nullptr) << "opendir failed: " << std::strerror(errno);

    std::set<std::string> entries;
    errno = 0;
    struct dirent *ent;
    while ((ent = ::readdir(dir)) != nullptr) {
        entries.insert(ent->d_name);
    }
    EXPECT_EQ(errno, 0) << "readdir failed: " << std::strerror(errno);
    ::closedir(dir);

    // "." and ".." may or may not be surfaced by libc/kernel for FUSE; the only
    // non-dot entry permitted in a config-less root is the global hotplug file.
    for (const auto &name : entries) {
        EXPECT_TRUE(name == "." || name == ".." || name == "hotplug")
            << "unexpected entry in empty root: " << name;
    }
    EXPECT_EQ(entries.count("hotplug"), 1u)
        << "the global hotplug file must be present at the mount root";

    // Looking up a nonexistent name must fail with ENOENT (lookup is wired).
    std::string ghost = mountpoint + "/does_not_exist";
    EXPECT_NE(::stat(ghost.c_str(), &st), 0);
    EXPECT_EQ(errno, ENOENT);

    // Graceful shutdown: SIGTERM should unmount and exit cleanly.
    ASSERT_EQ(::kill(pid, SIGTERM), 0) << "kill failed: " << std::strerror(errno);

    int status = 0;
    bool exited = wait_for(
        [&] { return ::waitpid(pid, &status, WNOHANG) == pid; },
        kShutdownTimeoutMs);

    if (!exited) {
        ::kill(pid, SIGKILL);
        ::waitpid(pid, &status, 0);
        ::rmdir(mountpoint.c_str());
        FAIL() << "Daemon did not exit within timeout after SIGTERM";
    }

    EXPECT_TRUE(WIFEXITED(status))
        << "daemon did not exit normally (status=" << status << ")";
    if (WIFEXITED(status)) {
        EXPECT_EQ(WEXITSTATUS(status), 0) << "daemon exited non-zero";
    }

    // After clean shutdown the FUSE mount is gone; the now-empty temp dir
    // should be removable.
    EXPECT_EQ(::rmdir(mountpoint.c_str()), 0)
        << "mountpoint not cleanly unmounted/removable: "
        << std::strerror(errno);
}
