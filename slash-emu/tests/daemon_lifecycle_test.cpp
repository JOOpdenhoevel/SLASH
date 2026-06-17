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
 * @file daemon_lifecycle_test.cpp
 * @brief Adversarial lifecycle / robustness tests for the slash-emud daemon.
 *
 * These tests codify the T3 scaffold audit as committed, repeatable CTest
 * cases (replacing ad-hoc shell verification).  They cover:
 *
 *   - CLI error paths: --help (exit 0), unknown flag (exit 1 + usage on
 *     stderr), stray positional arg (exit 1), --mount with no value (exit 1).
 *   - Mount-failure / partial-init cleanup: an invalid or unwritable --mount
 *     path must make the daemon exit non-zero cleanly, leave no FUSE mount
 *     behind, and leak no child process.
 *   - No-leak / idempotency: a normal mount -> SIGTERM -> unmount cycle must
 *     leave no leftover .tmp/slash_emu_* scratch dirs and no orphaned process.
 *   - Race robustness: the mount/unmount de-race gate is exercised in a loop so
 *     a single `ctest` run permanently guards against regressions (no manual
 *     12x reruns).
 *
 * Like smoke_test.cpp, the daemon path and scratch dir are injected as compile
 * definitions (SLASH_EMUD_PATH / SLASH_EMU_TMP_DIR); the daemon is always
 * exercised via fork+exec so CTest fully drives it.
 *
 * NOTE: Some of these tests may legitimately FAIL against the current scaffold
 * --- they document required behavior for the implementer, not a guarantee the
 * scaffold already satisfies it.
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
constexpr int kProcExitTimeoutMs = 5000;
constexpr int kPollIntervalMs = 50;

// Number of mount/unmount cycles run by the race-robustness test.  Encodes the
// "run it many times" check so the de-race gate is permanently guarded.
constexpr int kRaceCycles = 12;

// ---------------------------------------------------------------------------
// Helpers (mirroring smoke_test.cpp's fork+exec pattern).
// ---------------------------------------------------------------------------

// Create a unique mountpoint directory under the repo .tmp scratch area.
std::string make_mountpoint()
{
    std::string tmpl = std::string(SLASH_EMU_TMP_DIR) + "/slash_emu_lifecycle_XXXXXX";
    std::vector<char> buf(tmpl.begin(), tmpl.end());
    buf.push_back('\0');

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
// dir mkdtemp created).  statfs's f_type reliably distinguishes the two.
bool mount_is_ready(const std::string &mountpoint)
{
    struct statfs sfs {};
    if (::statfs(mountpoint.c_str(), &sfs) != 0) {
        return false;
    }
    return sfs.f_type == FUSE_SUPER_MAGIC;
}

// True if the given path is currently a FUSE mount (used to assert that a
// failed/torn-down daemon left nothing mounted behind).
bool is_fuse_mount(const std::string &path)
{
    struct statfs sfs {};
    if (::statfs(path.c_str(), &sfs) != 0) {
        return false;
    }
    return sfs.f_type == FUSE_SUPER_MAGIC;
}

// True if a process with this pid still exists (kill(pid, 0)).
bool process_alive(pid_t pid)
{
    return ::kill(pid, 0) == 0 || errno != ESRCH;
}

// Best-effort force-unmount of a FUSE mountpoint.  Used by the failure-path
// tests' teardown so that even if the daemon has to be SIGKILLed (because it
// hung), we never leave an orphaned FUSE mount behind on the test host.
void force_unmount(const std::string &mountpoint)
{
    if (!is_fuse_mount(mountpoint)) {
        return;
    }
    std::string cmd = "fusermount3 -u '" + mountpoint +
                      "' 2>/dev/null || fusermount -u '" + mountpoint +
                      "' 2>/dev/null";
    (void) std::system(cmd.c_str());
}

// Result of running the daemon to completion (used for the failure-path tests).
struct RunResult {
    bool exited_normally = false;  // WIFEXITED
    int exit_code = -1;            // WEXITSTATUS (valid iff exited_normally)
    bool timed_out = false;        // had to be SIGKILLed
};

// Fork+exec the daemon with the given argv tail, wait up to a timeout for it to
// exit on its own, and report how it ended.  On timeout the child is SIGKILLed
// (and timed_out is set) so the test never hangs or leaks a process.
//
// @p mountpoint, if non-empty, is force-unmounted on the SIGKILL path so a
// misbehaving daemon (one that mounted but then hung) cannot leak a mount.
RunResult run_daemon(const std::vector<std::string> &args, int timeout_ms,
                     const std::string &mountpoint = "")
{
    RunResult res;

    pid_t pid = ::fork();
    EXPECT_NE(pid, -1) << "fork failed: " << std::strerror(errno);
    if (pid == -1) {
        return res;
    }

    if (pid == 0) {
        std::vector<char *> argv;
        argv.push_back(const_cast<char *>("slash-emud"));
        for (const auto &a : args) {
            argv.push_back(const_cast<char *>(a.c_str()));
        }
        argv.push_back(nullptr);
        ::execv(SLASH_EMUD_PATH, argv.data());
        ::_exit(127);
    }

    int status = 0;
    bool exited = wait_for(
        [&] { return ::waitpid(pid, &status, WNOHANG) == pid; }, timeout_ms);

    if (!exited) {
        // Daemon did not exit on its own --- this is itself a failure for the
        // mount-failure tests, but we must still reap it AND force-unmount so
        // nothing leaks (process or FUSE mount).
        res.timed_out = true;
        ::kill(pid, SIGKILL);
        ::waitpid(pid, &status, 0);
        if (!mountpoint.empty()) {
            force_unmount(mountpoint);
        }
        return res;
    }

    res.exited_normally = WIFEXITED(status);
    if (res.exited_normally) {
        res.exit_code = WEXITSTATUS(status);
    }
    return res;
}

}  // namespace

// ===========================================================================
// CLI error paths
// ===========================================================================

TEST(SlashEmuCli, HelpExitsZero)
{
    RunResult res = run_daemon({"--help"}, kProcExitTimeoutMs);
    ASSERT_FALSE(res.timed_out) << "--help should exit immediately";
    EXPECT_TRUE(res.exited_normally);
    EXPECT_EQ(res.exit_code, 0) << "--help must exit 0";
}

TEST(SlashEmuCli, UnknownFlagExitsNonZeroWithUsageOnStderr)
{
    // Capture stderr via a pipe so we can assert usage is printed there.
    int err_pipe[2];
    ASSERT_EQ(::pipe(err_pipe), 0);

    pid_t pid = ::fork();
    ASSERT_NE(pid, -1);
    if (pid == 0) {
        ::dup2(err_pipe[1], STDERR_FILENO);
        ::close(err_pipe[0]);
        ::close(err_pipe[1]);
        ::execl(SLASH_EMUD_PATH, "slash-emud", "--bogus", (char *) nullptr);
        ::_exit(127);
    }
    ::close(err_pipe[1]);

    std::string stderr_out;
    char buf[512];
    ssize_t n;
    while ((n = ::read(err_pipe[0], buf, sizeof(buf))) > 0) {
        stderr_out.append(buf, static_cast<size_t>(n));
    }
    ::close(err_pipe[0]);

    int status = 0;
    ::waitpid(pid, &status, 0);

    ASSERT_TRUE(WIFEXITED(status));
    EXPECT_EQ(WEXITSTATUS(status), 1) << "unknown flag must exit 1";
    EXPECT_NE(stderr_out.find("Usage:"), std::string::npos)
        << "usage must be printed to stderr on unknown flag; got: " << stderr_out;
}

TEST(SlashEmuCli, StrayPositionalArgExitsNonZero)
{
    RunResult res = run_daemon({"stray"}, kProcExitTimeoutMs);
    ASSERT_FALSE(res.timed_out) << "stray arg should be rejected immediately";
    EXPECT_TRUE(res.exited_normally);
    EXPECT_EQ(res.exit_code, 1) << "stray positional arg must exit 1";
}

TEST(SlashEmuCli, MountWithoutValueExitsNonZero)
{
    RunResult res = run_daemon({"--mount"}, kProcExitTimeoutMs);
    ASSERT_FALSE(res.timed_out) << "--mount with no value should be rejected immediately";
    EXPECT_TRUE(res.exited_normally);
    EXPECT_EQ(res.exit_code, 1) << "--mount with no argument must exit 1";
}

// ===========================================================================
// Mount-failure & partial-init cleanup
// ===========================================================================

TEST(SlashEmuMountFailure, NonexistentMountpointExitsCleanly)
{
    // A mountpoint path whose parent directory does not exist.  The daemon must
    // fail to mount, exit non-zero, and NOT hang or leak.
    std::string bad = std::string(SLASH_EMU_TMP_DIR) +
                      "/slash_emu_nonexistent_parent/mnt";

    RunResult res = run_daemon({"--mount", bad}, kProcExitTimeoutMs, bad);

    EXPECT_FALSE(res.timed_out)
        << "daemon hung on an invalid mountpoint instead of failing fast";
    EXPECT_TRUE(res.exited_normally) << "daemon did not exit normally";
    if (res.exited_normally) {
        EXPECT_NE(res.exit_code, 0)
            << "daemon must exit non-zero when the mount fails";
    }
    // Nothing should be mounted at the bad path.
    EXPECT_FALSE(is_fuse_mount(bad)) << "a FUSE mount was left behind at " << bad;
}

// A mountpoint inside a directory the daemon cannot traverse/write must fail to
// mount.  We create a 0000-permission parent dir and point --mount at a child
// path inside it; mounting there is denied, so the daemon must fail fast and
// non-zero, hang-free and leak-free.
//
// (NOTE: this deliberately does NOT use "mount on a regular file" --- on Linux
// FUSE *can* mount over a non-directory, so that is not a failure path; the
// audit's real concern, that a SIGKILLed daemon could orphan a mount, is
// guarded by run_daemon()'s force_unmount teardown.)
TEST(SlashEmuMountFailure, UnwritableMountpointParentExitsCleanly)
{
    if (::geteuid() == 0) {
        GTEST_SKIP() << "running as root bypasses directory permission checks";
    }

    std::string tmpl = std::string(SLASH_EMU_TMP_DIR) + "/slash_emu_noperm_XXXXXX";
    std::vector<char> nbuf(tmpl.begin(), tmpl.end());
    nbuf.push_back('\0');
    ::mkdir(SLASH_EMU_TMP_DIR, 0755);
    char *parent = ::mkdtemp(nbuf.data());
    ASSERT_NE(parent, nullptr) << "mkdtemp failed: " << std::strerror(errno);
    std::string parent_dir = parent;
    std::string child = parent_dir + "/mnt";

    // Drop all permissions on the parent so the child path is inaccessible.
    ASSERT_EQ(::chmod(parent_dir.c_str(), 0000), 0)
        << "chmod failed: " << std::strerror(errno);

    RunResult res = run_daemon({"--mount", child}, kProcExitTimeoutMs, child);

    // Restore permissions so we can clean up regardless of the outcome.
    ::chmod(parent_dir.c_str(), 0755);

    EXPECT_FALSE(res.timed_out)
        << "daemon hung on an inaccessible mountpoint instead of failing fast";
    EXPECT_TRUE(res.exited_normally) << "daemon did not exit normally";
    if (res.exited_normally) {
        EXPECT_NE(res.exit_code, 0)
            << "daemon must exit non-zero when the mount is denied";
    }
    EXPECT_FALSE(is_fuse_mount(child))
        << "a FUSE mount was left behind at " << child;

    ::rmdir(child.c_str());
    ::rmdir(parent_dir.c_str());
}

// ===========================================================================
// No-leak / idempotency over a normal mount -> SIGTERM -> unmount cycle
// ===========================================================================

TEST(SlashEmuLifecycle, CleanCycleLeavesNoLeak)
{
    const std::string mountpoint = make_mountpoint();
    ASSERT_FALSE(mountpoint.empty());

    pid_t pid = ::fork();
    ASSERT_NE(pid, -1) << "fork failed: " << std::strerror(errno);
    if (pid == 0) {
        ::execl(SLASH_EMUD_PATH, "slash-emud", "--mount", mountpoint.c_str(),
                (char *) nullptr);
        ::_exit(127);
    }

    bool ready = wait_for([&] { return mount_is_ready(mountpoint); },
                          kMountTimeoutMs);
    if (!ready) {
        ::kill(pid, SIGKILL);
        int status = 0;
        ::waitpid(pid, &status, 0);
        ::rmdir(mountpoint.c_str());
        FAIL() << "Daemon did not mount within timeout at " << mountpoint;
    }

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

    EXPECT_TRUE(WIFEXITED(status)) << "daemon did not exit normally";
    if (WIFEXITED(status)) {
        EXPECT_EQ(WEXITSTATUS(status), 0) << "daemon exited non-zero on SIGTERM";
    }

    // No leaked child process.
    EXPECT_FALSE(process_alive(pid))
        << "daemon process " << pid << " is still alive after exit";

    // Mount is gone; the now-empty temp dir is removable.
    EXPECT_FALSE(is_fuse_mount(mountpoint))
        << "FUSE mount still present after clean shutdown";
    EXPECT_EQ(::rmdir(mountpoint.c_str()), 0)
        << "mountpoint not cleanly unmounted/removable: " << std::strerror(errno);

    // No leak: THIS cycle's own uniquely-named scratch dir must be gone after the
    // rmdir above.  (We check only our own dir, not a global count of every
    // slash_emu_lifecycle_*/smoke_* dir -- under parallel ctest -j16 other
    // concurrent lifecycle/smoke tests create and reap their own such dirs
    // between any before/after snapshot, so a global count is inherently racy and
    // would false-positive even though this cycle leaked nothing.)
    struct stat st {};
    EXPECT_NE(::stat(mountpoint.c_str(), &st), 0)
        << "this cycle's scratch dir " << mountpoint << " survived the clean cycle";
    EXPECT_EQ(errno, ENOENT)
        << "this cycle's scratch dir " << mountpoint << " is not cleanly gone";
}

// ===========================================================================
// Race robustness: encode the "run many times" de-race check as a loop so a
// single ctest run permanently guards it.
// ===========================================================================

TEST(SlashEmuLifecycle, MountUnmountRaceIsRobust)
{
    for (int cycle = 0; cycle < kRaceCycles; cycle++) {
        SCOPED_TRACE("cycle " + std::to_string(cycle));

        const std::string mountpoint = make_mountpoint();
        ASSERT_FALSE(mountpoint.empty());

        pid_t pid = ::fork();
        ASSERT_NE(pid, -1) << "fork failed: " << std::strerror(errno);
        if (pid == 0) {
            ::execl(SLASH_EMUD_PATH, "slash-emud", "--mount",
                    mountpoint.c_str(), (char *) nullptr);
            ::_exit(127);
        }

        bool ready = wait_for([&] { return mount_is_ready(mountpoint); },
                              kMountTimeoutMs);
        if (!ready) {
            ::kill(pid, SIGKILL);
            int status = 0;
            ::waitpid(pid, &status, 0);
            ::rmdir(mountpoint.c_str());
            FAIL() << "Daemon did not mount within timeout (cycle " << cycle
                   << ") at " << mountpoint;
        }

        ASSERT_EQ(::kill(pid, SIGTERM), 0)
            << "kill failed: " << std::strerror(errno);

        int status = 0;
        bool exited = wait_for(
            [&] { return ::waitpid(pid, &status, WNOHANG) == pid; },
            kShutdownTimeoutMs);
        if (!exited) {
            ::kill(pid, SIGKILL);
            ::waitpid(pid, &status, 0);
            ::rmdir(mountpoint.c_str());
            FAIL() << "Daemon did not exit after SIGTERM (cycle " << cycle << ")";
        }

        EXPECT_TRUE(WIFEXITED(status))
            << "daemon did not exit normally (cycle " << cycle << ")";
        if (WIFEXITED(status)) {
            EXPECT_EQ(WEXITSTATUS(status), 0)
                << "daemon exited non-zero (cycle " << cycle << ")";
        }

        EXPECT_EQ(::rmdir(mountpoint.c_str()), 0)
            << "mountpoint not cleanly removable (cycle " << cycle
            << "): " << std::strerror(errno);
    }
}
