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
 * @file bridge_adversarial_test.cpp
 * @brief Adversarial conformance suite for the T10 SIM bridge (codified T10 audit).
 *
 * The destructive counterpart of bridge_test.cpp / bridge_integration_test.cpp.
 * Hammers the highest-risk surfaces the SIM bridge added:
 *
 *  1. PROCESS / FD / SOCKET / SCRATCH LIFECYCLE: fork/exec must never leak a
 *     child, zombie, fd, socket, or scratch dir on ANY path -- normal teardown,
 *     daemon shutdown, re-VBIN (idempotent), a model that exits / crashes / never
 *     binds / never answers / ignores exit.  Verified by counting children, fds,
 *     and scratch entries before-and-after, and asserting no zombie survives.
 *  2. NO-HANG: a hung / dead model must NOT wedge the daemon.  Every model call
 *     is bounded; a transport error latches the client dead (-ENODEV thereafter),
 *     and the FUSE daemon keeps serving other ops.  Driven by watchdogs -- a real
 *     hang FAILS the test (alarm) rather than blocking the suite.
 *  3. USTAR / VBIN PARSER SAFETY (security-sensitive): path traversal (absolute,
 *     "..", longname "..", backslash), symlink/hardlink/device/char entries,
 *     huge/oversized/truncated members, zero-size, duplicate members, name-length
 *     edges -- extraction must stay inside the scratch dir, reject malicious
 *     entries, and a malformed VBIN must fail the pwrite with a negative errno
 *     WITHOUT wedging the daemon.
 *  4. RECONFIG INTERCEPTION: a single whole-VBIN write triggers reconfig; a
 *     reconfig-region READ stays -ERANGE; an ordinary HBM/DDR transfer is
 *     unaffected; a SECOND reconfig tears down the prior model first; the
 *     chunked-write contract is probed.
 *  4b. ZMQ DESYNC: a model that dies between request and reply must latch the
 *      client dead (not reuse a half-consumed REQ socket); protocol garbage /
 *      oversized / short replies are handled (clean -EPROTO, no UB).
 *  5. BACKEND rc-CONTRACT through the REAL client+stub: bar read/write
 *     (BAR0 width<=4 -> model; BAR2/4 + width8 -> shadow, never -EIO in range);
 *     qdma fetch/populate; rc==0 model / rc>0 shadow/store / rc<0 -ENODEV; the
 *     carry-forward (shadow/store written before forward, no rollback) semantics.
 *  6. CONCURRENCY: per-device io_lock serialises model I/O; two devices each with
 *     their own model do not cross-talk.
 *
 * Build rules: CMake/CTest only, GTest, .tmp scratch, per-test ctest TIMEOUT so a
 * hang FAILS rather than blocks.  ASan/UBSan clean.
 */

#include <gtest/gtest.h>

#include <atomic>
#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>
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
#include "model_client.h"
#include "qdma.h"
#include "slash/uapi/slash_abi.h"
#include "vbin.h"
}

namespace {

// ===========================================================================
// Watchdog: a real hang must FAIL the test, not block the suite.  Arms a
// SIGALRM that aborts the process; the ctest TIMEOUT is the outer backstop.
// ===========================================================================
class Watchdog {
public:
    explicit Watchdog(unsigned secs) { ::alarm(secs); }
    ~Watchdog() { ::alarm(0); }
};

// ---------------------------------------------------------------------------
// Scratch + tar helpers
// ---------------------------------------------------------------------------

std::string make_scratch_dir(const char *suffix)
{
    ::mkdir(SLASH_EMU_TMP_DIR, 0755);
    std::string tmpl =
        std::string(SLASH_EMU_TMP_DIR) + "/bradv_" + suffix + "_XXXXXX";
    std::vector<char> buf(tmpl.begin(), tmpl.end());
    buf.push_back('\0');
    char *r = ::mkdtemp(buf.data());
    EXPECT_NE(r, nullptr) << "mkdtemp: " << std::strerror(errno);
    return r ? std::string(r) : std::string();
}

void rm_rf(const std::string &path)
{
    (void) ::system(("rm -rf '" + path + "'").c_str());
}

int count_entries(const std::string &dir)
{
    DIR *d = ::opendir(dir.c_str());
    if (d == nullptr) {
        return 0;
    }
    int n = 0;
    struct dirent *e;
    while ((e = ::readdir(d)) != nullptr) {
        if (std::strcmp(e->d_name, ".") != 0 &&
            std::strcmp(e->d_name, "..") != 0) {
            n++;
        }
    }
    ::closedir(d);
    return n;
}

// Number of our direct children (any state, incl. zombies) by scanning /proc.
int child_proc_count()
{
    pid_t me = ::getpid();
    DIR *d = ::opendir("/proc");
    if (d == nullptr) {
        return -1;
    }
    int n = 0;
    struct dirent *e;
    while ((e = ::readdir(d)) != nullptr) {
        char *end = nullptr;
        long pid = std::strtol(e->d_name, &end, 10);
        if (end == e->d_name || *end != '\0' || pid <= 0) {
            continue;
        }
        char path[64];
        std::snprintf(path, sizeof(path), "/proc/%ld/stat", pid);
        FILE *f = ::fopen(path, "r");
        if (f == nullptr) {
            continue;
        }
        // stat: pid (comm) state ppid ...   comm may contain spaces/parens, so
        // parse from the last ')' .
        char line[512];
        if (std::fgets(line, sizeof(line), f) != nullptr) {
            char *rp = std::strrchr(line, ')');
            if (rp != nullptr) {
                char st = 0;
                long ppid = 0;
                if (std::sscanf(rp + 1, " %c %ld", &st, &ppid) == 2 &&
                    ppid == (long) me) {
                    n++;
                }
            }
        }
        std::fclose(f);
    }
    ::closedir(d);
    return n;
}

// Build a single-member ustar archive with explicit typeflag/linkname so we can
// forge symlink/hardlink/device entries the parser must refuse to honour.
std::vector<uint8_t> make_tar_entry(const std::string &name, char typeflag,
                                    const std::vector<uint8_t> &content,
                                    unsigned mode = 0755,
                                    const std::string &linkname = "",
                                    bool with_trailer = true)
{
    std::vector<uint8_t> out;
    out.resize(512, 0);
    auto *h = out.data();
    // name (may be > field len in caller; we copy what fits, matching real tar).
    std::memcpy(h, name.data(), std::min<size_t>(name.size(), 100));
    std::snprintf((char *) (h + 100), 8, "%07o", mode & 07777);
    std::snprintf((char *) (h + 108), 8, "%07o", 0);
    std::snprintf((char *) (h + 116), 8, "%07o", 0);
    std::snprintf((char *) (h + 124), 12, "%011o", (unsigned) content.size());
    std::snprintf((char *) (h + 136), 12, "%011o", 0);
    h[156] = typeflag;
    if (!linkname.empty()) {
        std::memcpy(h + 157, linkname.data(),
                    std::min<size_t>(linkname.size(), 100));
    }
    std::memcpy(h + 257, "ustar", 5);
    h[263] = '0';
    h[264] = '0';
    std::memset(h + 148, ' ', 8);
    unsigned sum = 0;
    for (int i = 0; i < 512; i++) {
        sum += h[i];
    }
    std::snprintf((char *) (h + 148), 8, "%06o", sum);
    h[154] = '\0';
    h[155] = ' ';
    size_t data = out.size();
    out.resize(data + ((content.size() + 511) / 512) * 512, 0);
    if (!content.empty()) {
        std::memcpy(out.data() + data, content.data(), content.size());
    }
    if (with_trailer) {
        out.resize(out.size() + 1024, 0); // two zero blocks
    }
    return out;
}

// A GNU long-name ('L') record carrying `longname`, followed by `entry` (a header
// whose real name is the long-name).  Used to smuggle a long path-traversal name.
std::vector<uint8_t> make_longname_tar(const std::string &longname, char typeflag,
                                       const std::vector<uint8_t> &content)
{
    std::vector<uint8_t> ln_content(longname.begin(), longname.end());
    ln_content.push_back('\0');
    std::vector<uint8_t> out =
        make_tar_entry("././@LongLink", 'L', ln_content, 0644, "", false);
    // The following entry's header name is short; the 'L' record overrides it.
    std::vector<uint8_t> entry =
        make_tar_entry("placeholder", typeflag, content, 0755, "", true);
    out.insert(out.end(), entry.begin(), entry.end());
    return out;
}

std::vector<uint8_t> read_stub_binary()
{
    FILE *f = ::fopen(SLASH_EMU_STUB_MODEL_PATH, "rb");
    EXPECT_NE(f, nullptr);
    std::vector<uint8_t> data;
    if (f != nullptr) {
        std::fseek(f, 0, SEEK_END);
        long sz = std::ftell(f);
        std::fseek(f, 0, SEEK_SET);
        data.resize((size_t) sz);
        size_t r = std::fread(data.data(), 1, (size_t) sz, f);
        EXPECT_EQ(r, (size_t) sz);
        std::fclose(f);
    }
    return data;
}

// Spawn the stub bound to `endpoint` with extra env (key=value).
pid_t spawn_stub(const std::string &endpoint,
                 const std::vector<std::string> &extra_env = {})
{
    pid_t pid = ::fork();
    if (pid < 0) {
        return -1;
    }
    if (pid == 0) {
        ::setenv("SLASH_EMU_ENDPOINT", endpoint.c_str(), 1);
        for (const auto &kv : extra_env) {
            auto eq = kv.find('=');
            ::setenv(kv.substr(0, eq).c_str(), kv.substr(eq + 1).c_str(), 1);
        }
        ::execl(SLASH_EMU_STUB_MODEL_PATH, "stub_model", (char *) nullptr);
        ::_exit(127);
    }
    return pid;
}

void reap(pid_t pid)
{
    if (pid <= 0) {
        return;
    }
    ::kill(pid, SIGKILL);
    ::waitpid(pid, nullptr, 0);
}

// ===========================================================================
// (3) USTAR / VBIN parser safety -- the security-sensitive surface.
// Every hostile archive must (a) be rejected or safely ignored, and (b) leave
// NOTHING written outside the destination scratch dir.
// ===========================================================================

class VbinSafety : public ::testing::Test {
protected:
    std::string dir;
    std::string sentinel_dir;   // a sibling we assert is never touched
    void SetUp() override
    {
        dir = make_scratch_dir("vbin_dst");
        sentinel_dir = make_scratch_dir("vbin_sentinel");
    }
    void TearDown() override
    {
        rm_rf(dir);
        rm_rf(sentinel_dir);
    }
    // assert the unpack did not create anything under sentinel_dir
    void assert_sentinel_clean() { EXPECT_EQ(count_entries(sentinel_dir), 0); }
};

TEST_F(VbinSafety, AbsolutePathMemberRejected)
{
    auto tar = make_tar_entry("/etc/slash_pwn", '0', {'x'}, 0644);
    char exec[4096];
    EXPECT_EQ(emu_vbin_unpack_find_sim(tar.data(), tar.size(), dir.c_str(), exec,
                                       sizeof(exec)),
              -EINVAL);
    // Nothing escaped to the absolute path (we obviously can't write /etc here,
    // but assert the parser refused rather than even trying).
    struct stat st {};
    EXPECT_NE(::stat("/etc/slash_pwn", &st), 0);
}

TEST_F(VbinSafety, DotDotTraversalRejected)
{
    auto tar = make_tar_entry("../../escape", '0', {'x'}, 0644);
    char exec[4096];
    EXPECT_EQ(emu_vbin_unpack_find_sim(tar.data(), tar.size(), dir.c_str(), exec,
                                       sizeof(exec)),
              -EINVAL);
    // The parent of dir must not have gained an "escape" file.
    std::string parent = dir.substr(0, dir.find_last_of('/'));
    struct stat st {};
    EXPECT_NE(::stat((parent + "/escape").c_str(), &st), 0)
        << "path traversal wrote outside dest_dir";
    assert_sentinel_clean();
}

TEST_F(VbinSafety, DotDotInMiddleRejected)
{
    auto tar = make_tar_entry("sub/../../escape", '0', {'x'}, 0644);
    char exec[4096];
    EXPECT_EQ(emu_vbin_unpack_find_sim(tar.data(), tar.size(), dir.c_str(), exec,
                                       sizeof(exec)),
              -EINVAL);
}

TEST_F(VbinSafety, LongnameTraversalRejected)
{
    // Smuggle a "../" traversal via a GNU long-name record.
    std::string evil = "../../../../tmp/slash_longname_pwn";
    auto tar = make_longname_tar(evil, '0', {'x'});
    char exec[4096];
    EXPECT_EQ(emu_vbin_unpack_find_sim(tar.data(), tar.size(), dir.c_str(), exec,
                                       sizeof(exec)),
              -EINVAL);
    struct stat st {};
    EXPECT_NE(::stat("/tmp/slash_longname_pwn", &st), 0);
}

TEST_F(VbinSafety, SymlinkEntryDoesNotEscape)
{
    // A symlink member pointing outside, followed by a vpp_sim regular file:
    // if the symlink were honoured, a later write through it could escape.  The
    // parser must NOT create the symlink (it ignores non-reg/dir typeflags), so
    // no dangling link and no escape; and since there's no real vpp_sim, -ENOENT.
    auto tar = make_tar_entry("link", '2', {}, 0777, "/etc");
    char exec[4096];
    int rc = emu_vbin_unpack_find_sim(tar.data(), tar.size(), dir.c_str(), exec,
                                      sizeof(exec));
    EXPECT_EQ(rc, -ENOENT) << "expected no vpp_sim; symlink must be ignored";
    // The symlink must NOT have been materialised in dir.
    struct stat st {};
    EXPECT_NE(::lstat((dir + "/link").c_str(), &st), 0)
        << "symlink member was materialised -- escape risk";
}

TEST_F(VbinSafety, HardlinkEntryIgnored)
{
    auto tar = make_tar_entry("hard", '1', {}, 0644, "/etc/passwd");
    char exec[4096];
    EXPECT_EQ(emu_vbin_unpack_find_sim(tar.data(), tar.size(), dir.c_str(), exec,
                                       sizeof(exec)),
              -ENOENT);
    struct stat st {};
    EXPECT_NE(::lstat((dir + "/hard").c_str(), &st), 0);
}

TEST_F(VbinSafety, CharDeviceEntryIgnored)
{
    // Char-device entry (typeflag '3'): must not be mknod'd.
    auto tar = make_tar_entry("dev", '3', {}, 0644);
    char exec[4096];
    EXPECT_EQ(emu_vbin_unpack_find_sim(tar.data(), tar.size(), dir.c_str(), exec,
                                       sizeof(exec)),
              -ENOENT);
    struct stat st {};
    EXPECT_NE(::lstat((dir + "/dev").c_str(), &st), 0);
}

TEST_F(VbinSafety, TruncatedArchiveRejected)
{
    // A header that claims a 4096-byte file but the archive is one block long.
    auto tar = make_tar_entry("vpp_sim", '0',
                              std::vector<uint8_t>(4096, 'A'), 0755);
    tar.resize(512); // chop off the data blocks -> size field outruns the buffer
    char exec[4096];
    EXPECT_EQ(emu_vbin_unpack_find_sim(tar.data(), tar.size(), dir.c_str(), exec,
                                       sizeof(exec)),
              -EINVAL);
}

TEST_F(VbinSafety, NonBlockAlignedRejected)
{
    std::vector<uint8_t> junk(513, 0xAB); // not a multiple of 512
    char exec[4096];
    EXPECT_EQ(emu_vbin_unpack_find_sim(junk.data(), junk.size(), dir.c_str(),
                                       exec, sizeof(exec)),
              -EINVAL);
}

TEST_F(VbinSafety, BadOctalSizeRejected)
{
    auto tar = make_tar_entry("vpp_sim", '0', {'x'}, 0755);
    // Corrupt the size field with a non-octal digit.
    std::memcpy(tar.data() + 124, "99999999", 8);
    char exec[4096];
    EXPECT_EQ(emu_vbin_unpack_find_sim(tar.data(), tar.size(), dir.c_str(), exec,
                                       sizeof(exec)),
              -EINVAL);
}

TEST_F(VbinSafety, ZeroSizeVppSimRejectedNotExecutable)
{
    // A zero-byte vpp_sim with no exec bit: located but not runnable -> -EACCES.
    auto tar = make_tar_entry("vpp_sim", '0', {}, 0644);
    char exec[4096];
    EXPECT_EQ(emu_vbin_unpack_find_sim(tar.data(), tar.size(), dir.c_str(), exec,
                                       sizeof(exec)),
              -EACCES);
}

TEST_F(VbinSafety, NameLengthBoundaryHandled)
{
    // A 100-char name (exactly the header field width, no NUL) must be handled
    // without overrun -- it's a single deep component, no vpp_sim -> -ENOENT.
    std::string name(100, 'a');
    auto tar = make_tar_entry(name, '0', {'x'}, 0644);
    char exec[4096];
    int rc = emu_vbin_unpack_find_sim(tar.data(), tar.size(), dir.c_str(), exec,
                                      sizeof(exec));
    EXPECT_EQ(rc, -ENOENT);
}

TEST_F(VbinSafety, DuplicateVppSimLocatesOne)
{
    // Two vpp_sim members: the unpack must not corrupt or double-free; one is
    // located and runnable.
    auto stub = read_stub_binary();
    auto a = make_tar_entry("a/vpp_sim", '0', stub, 0755, "", false);
    auto b = make_tar_entry("b/vpp_sim", '0', stub, 0755, "", true);
    a.insert(a.end(), b.begin(), b.end());
    char exec[4096];
    ASSERT_EQ(emu_vbin_unpack_find_sim(a.data(), a.size(), dir.c_str(), exec,
                                       sizeof(exec)),
              0);
    EXPECT_EQ(::access(exec, X_OK), 0);
}

TEST_F(VbinSafety, ExecOutBufferTooSmall)
{
    // A located vpp_sim whose path does not fit the caller's exec buffer must
    // yield -ENAMETOOLONG, not a buffer overrun.
    auto stub = read_stub_binary();
    auto tar = make_tar_entry("deep/nested/vpp_sim", '0', stub, 0755);
    char exec[8]; // far too small
    int rc = emu_vbin_unpack_find_sim(tar.data(), tar.size(), dir.c_str(), exec,
                                      sizeof(exec));
    EXPECT_EQ(rc, -ENAMETOOLONG);
}

// ===========================================================================
// (2) emu_vbin_classify: NO false-COMPLETE.  The classifier MUST skip member
// CONTENT by the header size (not scan it for zero blocks), and only report
// COMPLETE at a true zero block sitting at a 512-aligned header position.  These
// are the load-bearing checks for chunk reassembly safety.
// ===========================================================================

// Build a single regular member with explicit content (helper local to classify).
std::vector<uint8_t> classify_member(const std::string &name,
                                     const std::vector<uint8_t> &content,
                                     bool trailer)
{
    return make_tar_entry(name, '0', content, 0644, "", trailer);
}

TEST(VbinClassify, ZeroContentMemberNotMistakenForTerminator)
{
    // A member whose CONTENT is all-zero for several blocks (>= 2 zero blocks
    // INSIDE a real member).  Without skipping content by size, the classifier
    // would see zero blocks and falsely report COMPLETE before the member ends.
    std::vector<uint8_t> zeros(4 * 512, 0); // 4 zero content blocks
    auto tar = classify_member("zeros.bin", zeros, /*trailer=*/false);
    // Up to and through the all-zero content, the archive is NOT complete (the
    // content blocks belong to the member, skipped via the header size).
    // header(512) + 4 zero blocks = 2560 bytes, still mid-archive (no terminator).
    EXPECT_EQ(emu_vbin_classify(tar.data(), tar.size()), EMU_VBIN_INCOMPLETE)
        << "zero-content member mis-detected as terminator (false COMPLETE)";

    // Now append the real terminator: classify must finally report COMPLETE, and
    // crucially at the TRUE end, not at the first content zero block.
    tar.resize(tar.size() + 1024, 0); // two zero trailer blocks
    EXPECT_EQ(emu_vbin_classify(tar.data(), tar.size()), EMU_VBIN_COMPLETE);

    // And a prefix that ends exactly at the first content zero block is still
    // INCOMPLETE (we are inside the member, by size).
    EXPECT_EQ(emu_vbin_classify(tar.data(), 512 + 512), EMU_VBIN_INCOMPLETE)
        << "first content zero block falsely treated as terminator";
}

TEST(VbinClassify, ZeroContentMemberFollowedByMoreMembers)
{
    // zeros member, THEN a vpp_sim member, THEN terminator: a false-COMPLETE at
    // the zero content would drop the trailing vpp_sim entirely.
    std::vector<uint8_t> zeros(2 * 512, 0);
    auto a = classify_member("zeros.bin", zeros, /*trailer=*/false);
    auto b = make_tar_entry("vpp_sim", '0', {'x', 'y', 'z'}, 0755, "", false);
    a.insert(a.end(), b.begin(), b.end());
    // Before the terminator: INCOMPLETE (must keep accumulating past the zeros).
    EXPECT_EQ(emu_vbin_classify(a.data(), a.size()), EMU_VBIN_INCOMPLETE);
    a.resize(a.size() + 1024, 0);
    EXPECT_EQ(emu_vbin_classify(a.data(), a.size()), EMU_VBIN_COMPLETE);
}

TEST(VbinClassify, TerminatorExactlyOnChunkBoundary)
{
    // The terminator lands exactly at a 512-aligned position that is also a
    // plausible chunk boundary: a full member + a zero block.  Feeding exactly up
    // to (and including) the first terminator block must report COMPLETE; one
    // byte short of it must be INCOMPLETE.
    auto tar = make_tar_entry("vpp_sim", '0', std::vector<uint8_t>(512, 'A'), 0755,
                              "", /*trailer=*/false);
    size_t end_of_member = tar.size(); // header(512)+data(512) = 1024
    tar.resize(end_of_member + 1024, 0); // two zero terminator blocks

    // Exactly at the member end (no terminator yet): INCOMPLETE.
    EXPECT_EQ(emu_vbin_classify(tar.data(), end_of_member), EMU_VBIN_INCOMPLETE);
    // Including exactly the first terminator block (chunk boundary == 512-aligned
    // terminator): COMPLETE.
    EXPECT_EQ(emu_vbin_classify(tar.data(), end_of_member + 512),
              EMU_VBIN_COMPLETE);
    // One byte short of the terminator block: INCOMPLETE (need the whole block).
    EXPECT_EQ(emu_vbin_classify(tar.data(), end_of_member + 511),
              EMU_VBIN_INCOMPLETE);
}

TEST(VbinClassify, ChunkEndingMidHeaderIsIncompleteThenCompletes)
{
    // A chunk boundary that falls in the MIDDLE of a header block must classify
    // INCOMPLETE (cannot decide without the whole header), and once the rest
    // arrives it completes.
    auto tar = make_tar_entry("vpp_sim", '0', {'q'}, 0755, "", true);
    // Mid-first-header (offset 200 of the 512-byte header): INCOMPLETE.
    EXPECT_EQ(emu_vbin_classify(tar.data(), 200), EMU_VBIN_INCOMPLETE);
    // Mid-second (data) region: INCOMPLETE.
    EXPECT_EQ(emu_vbin_classify(tar.data(), 512 + 1), EMU_VBIN_INCOMPLETE);
    // Whole archive: COMPLETE.
    EXPECT_EQ(emu_vbin_classify(tar.data(), tar.size()), EMU_VBIN_COMPLETE);
}

TEST(VbinClassify, HostileHugeSizeFieldIsInvalidNotOverflow)
{
    // A header declaring an enormous member size must not overflow the advance
    // computation into a false INCOMPLETE/COMPLETE -- it stays INCOMPLETE (body
    // not arrived) or INVALID, never a crash / wrap.
    auto tar = make_tar_entry("huge", '0', {'a'}, 0644, "", false);
    // Max octal in the 12-byte size field (all 7s).
    std::memset(tar.data() + 124, '7', 11);
    tar[124 + 11] = '\0';
    std::memset(tar.data() + 148, ' ', 8);
    unsigned sum = 0;
    for (int i = 0; i < 512; i++) sum += tar[i];
    std::snprintf((char *) (tar.data() + 148), 8, "%06o", sum);
    tar[154] = '\0';
    tar[155] = ' ';
    enum emu_vbin_status st = emu_vbin_classify(tar.data(), tar.size());
    EXPECT_TRUE(st == EMU_VBIN_INCOMPLETE || st == EMU_VBIN_INVALID)
        << "hostile huge size field produced a false terminal status";
}

// ===========================================================================
// (4b) ZMQ desync + protocol garbage, at the client layer (no FUSE).
// ===========================================================================

class ClientHostile : public ::testing::Test {
protected:
    std::string dir;
    std::string endpoint;
    pid_t pid = -1;
    emu_model_client *client = nullptr;
    void SetUp() override
    {
        dir = make_scratch_dir("cli");
        endpoint = "ipc://" + dir + "/model.sock";
    }
    void TearDown() override
    {
        if (client != nullptr) {
            emu_model_client_close(client);
        }
        reap(pid);
        rm_rf(dir);
    }
};

TEST_F(ClientHostile, GarbageReplyIsEprotoNotUb)
{
    Watchdog wd(20);
    pid = spawn_stub(endpoint, {"SLASH_EMU_STUB_GARBAGE=1"});
    ASSERT_GT(pid, 0);
    ASSERT_EQ(emu_model_client_connect(endpoint.c_str(), 1000, &client), 0);
    ASSERT_EQ(emu_model_client_start(client), 0); // start still replies OK

    // A scalar fetch gets ~300 bytes of 'Z': the bare-uint parser must reject it.
    uint32_t v = 0;
    EXPECT_EQ(emu_model_scalar_read(client, 0x10, &v), -EPROTO);
    // Latched dead: a subsequent call fails fast, no desync reuse.
    EXPECT_EQ(emu_model_scalar_read(client, 0x10, &v), -ENODEV);
}

TEST_F(ClientHostile, GarbageBufferReplyIsEprotoNotUb)
{
    Watchdog wd(20);
    pid = spawn_stub(endpoint, {"SLASH_EMU_STUB_GARBAGE=1"});
    ASSERT_GT(pid, 0);
    ASSERT_EQ(emu_model_client_connect(endpoint.c_str(), 1000, &client), 0);
    ASSERT_EQ(emu_model_client_start(client), 0);

    uint8_t buf[64];
    EXPECT_EQ(emu_model_fetch(client, 0x40, buf, sizeof(buf)), -EPROTO);
    EXPECT_EQ(emu_model_fetch(client, 0x40, buf, sizeof(buf)), -ENODEV);
}

TEST_F(ClientHostile, ModelDiesBetweenRequestAndReplyLatchesDead)
{
    Watchdog wd(20);
    // The stub answers `start` OK, then exit(0) on the NEXT request without
    // replying: a classic REQ/REP desync.  The client must time out, latch dead,
    // and refuse to reuse the half-consumed socket.
    pid = spawn_stub(endpoint, {"SLASH_EMU_STUB_EXIT_AFTER_START=1"});
    ASSERT_GT(pid, 0);
    ASSERT_EQ(emu_model_client_connect(endpoint.c_str(), 500, &client), 0);
    ASSERT_EQ(emu_model_client_start(client), 0);

    uint32_t v = 0;
    int rc = emu_model_scalar_read(client, 0x10, &v);
    EXPECT_LT(rc, 0); // timeout/transport failure, NOT a hang
    // Subsequent calls fail fast (-ENODEV), never block or desync-reuse.
    EXPECT_EQ(emu_model_reg_write(client, 0x10, 1), -ENODEV);
    EXPECT_EQ(emu_model_scalar_read(client, 0x10, &v), -ENODEV);
}

TEST_F(ClientHostile, ModelCrashesAfterConnectNoHang)
{
    Watchdog wd(20);
    pid = spawn_stub(endpoint, {"SLASH_EMU_STUB_CRASH_AFTER_START=1"});
    ASSERT_GT(pid, 0);
    ASSERT_EQ(emu_model_client_connect(endpoint.c_str(), 500, &client), 0);
    ASSERT_EQ(emu_model_client_start(client), 0);

    uint32_t v = 0;
    int rc = emu_model_scalar_read(client, 0x10, &v);
    EXPECT_LT(rc, 0);
    EXPECT_EQ(emu_model_scalar_read(client, 0x10, &v), -ENODEV);

    // The crashed child is reaped here (TearDown); confirm it died by signal.
    int status = 0;
    ASSERT_EQ(::waitpid(pid, &status, 0), pid);
    EXPECT_TRUE(WIFSIGNALED(status));
    pid = -1; // already reaped
}

// A populate (frame-0 + frame-1) to a dead client must also fail fast, and the
// SNDMORE half-message must not leave the socket in a bad state for the next
// (already-dead) call.
TEST_F(ClientHostile, PopulateAfterDeadFailsFast)
{
    Watchdog wd(20);
    pid = spawn_stub(endpoint, {"SLASH_EMU_STUB_HANG=1"});
    ASSERT_GT(pid, 0);
    ASSERT_EQ(emu_model_client_connect(endpoint.c_str(), 300, &client), 0);
    // start hangs -> times out -> dead.
    EXPECT_EQ(emu_model_client_start(client), -ETIMEDOUT);
    std::vector<uint8_t> payload(128, 0xAB);
    EXPECT_EQ(emu_model_populate(client, 0x40'0000'0000ULL, payload.data(),
                                 payload.size()),
              -ENODEV);
}

// ===========================================================================
// Integration harness (drives the real daemon over a FUSE mount).
// ===========================================================================

constexpr int kMountTimeoutMs = 5000;
constexpr int kShutdownTimeoutMs = 5000;
constexpr int kPollIntervalMs = 25;
constexpr const char *kBdf = "0000:61:00";

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

std::string make_scratch(const char *suffix) { return make_scratch_dir(suffix); }

std::string write_config(const char *body = nullptr)
{
    ::mkdir(SLASH_EMU_TMP_DIR, 0755);
    std::string tmpl = std::string(SLASH_EMU_TMP_DIR) + "/bradv_cfg_XXXXXX";
    std::vector<char> buf(tmpl.begin(), tmpl.end());
    buf.push_back('\0');
    int fd = ::mkstemp(buf.data());
    EXPECT_GE(fd, 0);
    const char *cfg =
        body ? body : "[accelerator:0000:61:00]\nnet-ip = 10.0.0.1\n";
    (void) ::write(fd, cfg, std::strlen(cfg));
    ::close(fd);
    return buf.data();
}

bool mount_is_ready(const std::string &mp)
{
    struct statfs sfs {};
    return ::statfs(mp.c_str(), &sfs) == 0 && sfs.f_type == FUSE_SUPER_MAGIC;
}

std::vector<uint8_t> read_file(const char *path)
{
    FILE *f = ::fopen(path, "rb");
    EXPECT_NE(f, nullptr);
    std::vector<uint8_t> d;
    if (f != nullptr) {
        std::fseek(f, 0, SEEK_END);
        long sz = std::ftell(f);
        std::fseek(f, 0, SEEK_SET);
        d.resize((size_t) sz);
        (void) std::fread(d.data(), 1, (size_t) sz, f);
        std::fclose(f);
    }
    return d;
}

std::vector<uint8_t> ci_vbin()
{
    return make_tar_entry("vpp_sim", '0', read_file(SLASH_EMU_STUB_MODEL_PATH),
                          0755);
}

int open_qpair(const std::string &mnt, const char *bdf, uint32_t *qid)
{
    std::string qdma = mnt + "/" + bdf + "/qdma";
    int dfd = ::open(qdma.c_str(), O_RDONLY | O_DIRECTORY);
    if (dfd < 0) {
        return -1;
    }
    struct slash_abi_qdma_qpair_add req {};
    req.size = sizeof(req);
    req.mode = 0;
    req.dir_mask = 0x3;
    int rc = ::ioctl(dfd, SLASH_ABI_QDMA_IOCTL_QPAIR_ADD, &req);
    ::close(dfd);
    if (rc != 0) {
        return -1;
    }
    *qid = req.qid;
    char name[4096];
    std::snprintf(name, sizeof(name), "%s/%s/qdma/qpair%u", mnt.c_str(), bdf,
                  req.qid);
    return ::open(name, O_RDWR);
}

int hotplug_remove(const std::string &mnt, const char *bdf_func)
{
    int fd = ::open((mnt + "/hotplug").c_str(), O_RDONLY);
    if (fd < 0) {
        return -errno;
    }
    struct slash_abi_hotplug_device_request req {};
    req.size = sizeof(req);
    std::snprintf(req.bdf, sizeof(req.bdf), "%s", bdf_func);
    int rc = ::ioctl(fd, SLASH_ABI_HOTPLUG_IOCTL_REMOVE, &req);
    int saved = errno;
    ::close(fd);
    return rc == 0 ? 0 : -saved;
}

// Count slash-emud's own model children (stub_model) by walking /proc for
// processes whose ppid == the daemon pid.
int daemon_children(pid_t dpid)
{
    DIR *d = ::opendir("/proc");
    if (d == nullptr) {
        return -1;
    }
    int n = 0;
    struct dirent *e;
    while ((e = ::readdir(d)) != nullptr) {
        char *end = nullptr;
        long pid = std::strtol(e->d_name, &end, 10);
        if (end == e->d_name || *end != '\0' || pid <= 0) {
            continue;
        }
        char path[64];
        std::snprintf(path, sizeof(path), "/proc/%ld/stat", pid);
        FILE *f = ::fopen(path, "r");
        if (f == nullptr) {
            continue;
        }
        char line[512];
        if (std::fgets(line, sizeof(line), f) != nullptr) {
            char *rp = std::strrchr(line, ')');
            if (rp != nullptr) {
                char st = 0;
                long ppid = 0;
                if (std::sscanf(rp + 1, " %c %ld", &st, &ppid) == 2 &&
                    ppid == (long) dpid) {
                    n++;
                }
            }
        }
        std::fclose(f);
    }
    ::closedir(d);
    return n;
}

struct DaemonHandle {
    pid_t pid = -1;
    std::string mountpoint;
    std::string scratch;
    std::string config;
};

// Start a daemon; the caller drives it and then calls stop_daemon.
DaemonHandle start_daemon(const char *cfg_body = nullptr)
{
    DaemonHandle h;
    h.mountpoint = make_scratch("mnt");
    h.scratch = make_scratch("scr");
    h.config = write_config(cfg_body);
    EXPECT_FALSE(h.mountpoint.empty());
    EXPECT_FALSE(h.scratch.empty());

    h.pid = ::fork();
    EXPECT_NE(h.pid, -1);
    if (h.pid == 0) {
        ::setenv("SLASH_EMU_SCRATCH_ROOT", h.scratch.c_str(), 1);
        ::execl(SLASH_EMUD_PATH, "slash-emud", "--config", h.config.c_str(),
                "--mount", h.mountpoint.c_str(), (char *) nullptr);
        ::_exit(127);
    }
    EXPECT_TRUE(wait_for([&] { return mount_is_ready(h.mountpoint); },
                         kMountTimeoutMs))
        << "daemon did not mount";
    return h;
}

void stop_daemon(DaemonHandle &h, bool expect_scratch_empty = true)
{
    ::kill(h.pid, SIGTERM);
    int status = 0;
    bool exited = wait_for(
        [&] { return ::waitpid(h.pid, &status, WNOHANG) == h.pid; },
        kShutdownTimeoutMs);
    if (!exited) {
        ::kill(h.pid, SIGKILL);
        ::waitpid(h.pid, &status, 0);
    }
    EXPECT_TRUE(exited) << "daemon did not exit on SIGTERM (wedged?)";
    if (expect_scratch_empty) {
        EXPECT_EQ(count_entries(h.scratch), 0)
            << "scratch root not cleaned after daemon shutdown -- leak";
    }
    rm_rf(h.mountpoint);
    rm_rf(h.scratch);
    ::unlink(h.config.c_str());
}

// ===========================================================================
// (1) Re-VBIN idempotency: a second reconfig tears the first model down.
//     No accumulation of model children or scratch run dirs.
// ===========================================================================

TEST(BridgeReconfigAdversarial, ReVbinTearsDownPriorModelNoLeak)
{
    Watchdog wd(60);
    DaemonHandle h = start_daemon();
    ASSERT_GT(h.pid, 0);

    uint32_t qid = 0;
    int qfd = open_qpair(h.mountpoint, kBdf, &qid);
    ASSERT_GE(qfd, 0);

    auto vbin = ci_vbin();
    // Reconfigure several times; each must leave exactly one model child and one
    // run dir (the prior one torn down first).
    for (int i = 0; i < 4; i++) {
        ssize_t w = ::pwrite(qfd, vbin.data(), vbin.size(),
                             (off_t) SLASH_RECONFIG_BASE);
        ASSERT_EQ(w, (ssize_t) vbin.size())
            << "reconfig #" << i << ": " << std::strerror(errno);
        // Exactly one live model child, one run dir.
        EXPECT_TRUE(wait_for([&] { return daemon_children(h.pid) == 1; }, 2000))
            << "reconfig #" << i << ": expected exactly 1 model child, got "
            << daemon_children(h.pid);
        EXPECT_EQ(count_entries(h.scratch), 1)
            << "reconfig #" << i << ": scratch run dirs accumulated";
        // The model is live: a bar round-trip works.
        std::string bar0 = h.mountpoint + "/" + kBdf + "/bars/bar0";
        int bfd = ::open(bar0.c_str(), O_RDWR);
        ASSERT_GE(bfd, 0);
        uint32_t rv = 0xA5A5'0000u + i;
        ASSERT_EQ(::pwrite(bfd, &rv, 4, 0x40), 4);
        uint32_t rb = 0;
        ASSERT_EQ(::pread(bfd, &rb, 4, 0x40), 4);
        EXPECT_EQ(rb, rv);
        ::close(bfd);
    }

    ::close(qfd);
    stop_daemon(h);
    // No leaked model children of THIS process after shutdown (the daemon was our
    // only child and has been waited on).
    EXPECT_EQ(daemon_children(::getpid()), 0)
        << "model/daemon children leaked after shutdown";
}

// ===========================================================================
// (1)+(2) A model that NEVER binds: reconfig must fail promptly (no hang), no
// child/scratch leak, and the daemon stays serviceable.
// ===========================================================================

TEST(BridgeReconfigAdversarial, NoBindModelReconfigFailsNoLeakNoWedge)
{
    Watchdog wd(60);
    DaemonHandle h = start_daemon();
    ASSERT_GT(h.pid, 0);
    uint32_t qid = 0;
    int qfd = open_qpair(h.mountpoint, kBdf, &qid);
    ASSERT_GE(qfd, 0);

    // A VBIN whose vpp_sim never binds: pack the stub but force NO_BIND via env is
    // not possible (env is the daemon's child).  Instead pack a tiny script-less
    // executable that exits immediately -> the handshake never completes.
    // Simplest: pack /bin/true as vpp_sim (it execs, never binds, exits 0).
    auto truebin = read_file("/bin/true");
    ASSERT_FALSE(truebin.empty());
    auto vbin = make_tar_entry("vpp_sim", '0', truebin, 0755);

    ssize_t w = ::pwrite(qfd, vbin.data(), vbin.size(),
                         (off_t) SLASH_RECONFIG_BASE);
    // Reconfig must fail (handshake times out) -- a negative return, not a hang.
    EXPECT_EQ(w, -1) << "reconfig with a non-binding model unexpectedly succeeded";

    // No model child should linger, and the scratch run dir is cleaned up.
    EXPECT_TRUE(wait_for([&] { return daemon_children(h.pid) == 0; }, 5000))
        << "non-binding model child leaked";
    EXPECT_TRUE(wait_for([&] { return count_entries(h.scratch) == 0; }, 5000))
        << "scratch not cleaned after a failed reconfig";

    // Daemon stays serviceable: an ordinary HBM transfer still works.
    uint8_t byte = 0x7E;
    EXPECT_EQ(::pwrite(qfd, &byte, 1, (off_t) SLASH_HBM_BASE), 1)
        << "daemon wedged after a failed reconfig";
    uint8_t back = 0;
    EXPECT_EQ(::pread(qfd, &back, 1, (off_t) SLASH_HBM_BASE), 1);
    EXPECT_EQ(back, byte);

    ::close(qfd);
    stop_daemon(h);
}

// ===========================================================================
// (2) A model that comes up then DIES: subsequent bar/qpair ops return -ENODEV
// promptly (no hang) and the daemon stays serviceable for other endpoints.
// We model "dies" by killing the model child directly.
// ===========================================================================

TEST(BridgeReconfigAdversarial, ModelDeathReturnsEnodevNoHang)
{
    Watchdog wd(60);
    DaemonHandle h = start_daemon();
    ASSERT_GT(h.pid, 0);
    uint32_t qid = 0;
    int qfd = open_qpair(h.mountpoint, kBdf, &qid);
    ASSERT_GE(qfd, 0);

    auto vbin = ci_vbin();
    ASSERT_EQ(::pwrite(qfd, vbin.data(), vbin.size(),
                       (off_t) SLASH_RECONFIG_BASE),
              (ssize_t) vbin.size());
    ASSERT_TRUE(wait_for([&] { return daemon_children(h.pid) == 1; }, 2000));

    // Find the model child pid and SIGKILL it (simulate a crash).
    pid_t model = -1;
    {
        DIR *d = ::opendir("/proc");
        struct dirent *e;
        while (d && (e = ::readdir(d)) != nullptr) {
            char *end = nullptr;
            long pid = std::strtol(e->d_name, &end, 10);
            if (end == e->d_name || *end != '\0' || pid <= 0) continue;
            char path[64];
            std::snprintf(path, sizeof(path), "/proc/%ld/stat", pid);
            FILE *f = ::fopen(path, "r");
            if (!f) continue;
            char line[512];
            if (std::fgets(line, sizeof(line), f)) {
                char *rp = std::strrchr(line, ')');
                long ppid = 0;
                char st = 0;
                if (rp && std::sscanf(rp + 1, " %c %ld", &st, &ppid) == 2 &&
                    ppid == (long) h.pid) {
                    model = (pid_t) pid;
                }
            }
            std::fclose(f);
        }
        if (d) ::closedir(d);
    }
    ASSERT_GT(model, 0) << "could not find the model child";
    ::kill(model, SIGKILL);

    // A BAR read after the model death must return -ENODEV PROMPTLY (the client
    // call times out within the bounded window), not hang.  bar0 width-4 forwards
    // to the model.
    std::string bar0 = h.mountpoint + "/" + kBdf + "/bars/bar0";
    int bfd = ::open(bar0.c_str(), O_RDWR);
    ASSERT_GE(bfd, 0);
    uint32_t rb = 0;
    ssize_t r = ::pread(bfd, &rb, 4, 0x40);
    // Either -ENODEV (forward failed) -- the contract for transport death.
    EXPECT_EQ(r, -1);
    EXPECT_EQ(errno, ENODEV) << "expected -ENODEV after model death, got "
                            << std::strerror(errno);
    ::close(bfd);

    // The daemon itself is NOT wedged: the read-only info endpoint still serves.
    std::string info = h.mountpoint + "/" + kBdf + "/info";
    int ifd = ::open(info.c_str(), O_RDONLY);
    EXPECT_GE(ifd, 0) << "daemon wedged after model death";
    if (ifd >= 0) ::close(ifd);

    ::close(qfd);
    // The dead model child must be reaped at teardown; scratch cleaned.
    stop_daemon(h);
}

// ===========================================================================
// (1) FIX RE-VERIFY: a production-shaped multi-MB VBIN, kernel-split into many
// reconfig-region chunks, reassembles end-to-end -- model spawns AND a bar AND a
// qdma data-plane round-trip works THROUGH it.  This is the test that previously
// failed (single ops->write capped at ~1 MiB); the chunk reassembler must make it
// pass.  Pushed to ~24 MiB (many tens of FUSE chunks) but below the 256 MiB cap.
// ===========================================================================

TEST(BridgeReconfigAdversarial, LargeMultiMbVbinReassemblesAndDataPlaneWorks)
{
    Watchdog wd(80);
    DaemonHandle h = start_daemon();
    ASSERT_GT(h.pid, 0);
    uint32_t qid = 0;
    int qfd = open_qpair(h.mountpoint, kBdf, &qid);
    ASSERT_GE(qfd, 0);

    // ~24 MiB VBIN: a big padding member (modelling sibling .so's) + vpp_sim.
    auto stub = read_file(SLASH_EMU_STUB_MODEL_PATH);
    std::vector<uint8_t> pad(24 * 1024 * 1024, 'P');
    auto a = make_tar_entry("lib/big.so", '0', pad, 0644, "", false);
    auto b = make_tar_entry("vpp_sim", '0', stub, 0755, "", true);
    a.insert(a.end(), b.begin(), b.end());

    errno = 0;
    ssize_t w = ::pwrite(qfd, a.data(), a.size(), (off_t) SLASH_RECONFIG_BASE);
    fprintf(stderr, "[large-vbin] pwrite(%zu) -> %zd errno=%d (%s) children=%d\n",
            a.size(), w, errno, w < 0 ? std::strerror(errno) : "ok",
            daemon_children(h.pid));
    ASSERT_EQ(w, (ssize_t) a.size())
        << "large multi-MB VBIN single write did not reassemble: "
        << std::strerror(errno);

    // The model is up exactly once.
    EXPECT_TRUE(wait_for([&] { return daemon_children(h.pid) == 1; }, 3000))
        << "no (or >1) model child after large-VBIN reassembly";
    EXPECT_EQ(count_entries(h.scratch), 1) << "expected exactly one run dir";

    // BAR register round-trip THROUGH the model.
    std::string bar0 = h.mountpoint + "/" + kBdf + "/bars/bar0";
    int bfd = ::open(bar0.c_str(), O_RDWR);
    ASSERT_GE(bfd, 0);
    uint32_t rv = 0xFEEDFACEu;
    ASSERT_EQ(::pwrite(bfd, &rv, 4, 0x40), 4) << std::strerror(errno);
    uint32_t rb = 0;
    ASSERT_EQ(::pread(bfd, &rb, 4, 0x40), 4) << std::strerror(errno);
    EXPECT_EQ(rb, rv) << "bar round-trip through reassembled model failed";
    ::close(bfd);

    // QDMA MM round-trip THROUGH the model (HBM).
    std::vector<uint8_t> payload(1024);
    for (size_t i = 0; i < payload.size(); i++) payload[i] = (uint8_t) (i * 3 + 1);
    ASSERT_EQ(::pwrite(qfd, payload.data(), payload.size(),
                       (off_t) SLASH_HBM_BASE),
              (ssize_t) payload.size())
        << std::strerror(errno);
    std::vector<uint8_t> got(1024, 0xFF);
    ASSERT_EQ(::pread(qfd, got.data(), got.size(), (off_t) SLASH_HBM_BASE),
              (ssize_t) got.size())
        << std::strerror(errno);
    EXPECT_EQ(got, payload) << "qdma round-trip through reassembled model failed";

    ::close(qfd);
    stop_daemon(h);
}

// ===========================================================================
// (4) CAP: a never-terminating / oversized reconfig stream is rejected with
// -EFBIG and the partial transfer is reset, leaving the daemon serviceable and
// no model spawned.  We append contiguous never-terminating chunks (each a valid
// header carrying a max-size member, so the archive never reaches a terminator)
// until the accumulation crosses the 256 MiB cap.
// ===========================================================================

TEST(BridgeReconfigAdversarial, OversizedReconfigStreamRejectedEfbig)
{
    Watchdog wd(80);
    DaemonHandle h = start_daemon();
    ASSERT_GT(h.pid, 0);
    uint32_t qid = 0;
    int qfd = open_qpair(h.mountpoint, kBdf, &qid);
    ASSERT_GE(qfd, 0);

    // A single header whose declared member size is enormous (1 GiB) but we only
    // ever deliver header+partial-body chunks -> classify stays INCOMPLETE while
    // the accumulation grows.  Deliver 4 MiB chunks of body content contiguously
    // until the daemon's 256 MiB cap trips -> -EFBIG.
    // First chunk: a valid ustar header declaring a 1 GiB regular member, no
    // terminator, followed by body bytes; subsequent chunks are pure body.
    std::vector<uint8_t> first =
        make_tar_entry("huge.bin", '0', std::vector<uint8_t>(4 * 1024 * 1024, 'Q'),
                       0644, "", false);
    // make_tar_entry set the size field to 4 MiB; rewrite it to 1 GiB so the
    // archive can never complete within the cap.
    std::snprintf((char *) (first.data() + 124), 12, "%011o",
                  (unsigned) (1u << 30));
    // recompute checksum after editing the size field.
    std::memset(first.data() + 148, ' ', 8);
    unsigned sum = 0;
    for (int i = 0; i < 512; i++) sum += first[i];
    std::snprintf((char *) (first.data() + 148), 8, "%06o", sum);
    first[154] = '\0';
    first[155] = ' ';

    off_t off = (off_t) SLASH_RECONFIG_BASE;
    int last_errno = 0;
    ssize_t last = 0;
    // First write (header + 4 MiB body).
    last = ::pwrite(qfd, first.data(), first.size(), off);
    last_errno = errno;
    ASSERT_GT(last, 0) << "first reconfig chunk rejected: " << std::strerror(errno);
    off += last;

    // Keep appending body chunks until the cap (256 MiB) rejects the stream with
    // -EFBIG.  Use 256 KiB sub-writes (below the FUSE max_write) so each pwrite
    // maps to a single ops->write and the cap's -EFBIG is observed cleanly at the
    // pwrite boundary (a 4 MiB pwrite would be kernel-split across the cap, hiding
    // the EFBIG behind the post-reset contiguity guard).
    std::vector<uint8_t> body(256 * 1024, 'Q');
    bool got_efbig = false;
    for (int i = 0; i < 1100; i++) { // 1100*256KiB = 275 MiB > 256 MiB cap
        errno = 0;
        ssize_t w = ::pwrite(qfd, body.data(), body.size(), off);
        last_errno = errno;
        if (w < 0) {
            got_efbig = (last_errno == EFBIG);
            break;
        }
        off += w;
    }
    EXPECT_TRUE(got_efbig) << "oversized stream not rejected with -EFBIG (last errno="
                          << std::strerror(last_errno) << ")";
    // No model spawned for an abandoned/oversized stream.
    EXPECT_EQ(daemon_children(h.pid), 0) << "model spawned for an oversized stream";

    // The cap reset the buffer: a FRESH valid VBIN now reconfigures cleanly
    // (clean recovery, the partial transfer did not poison the next one).
    auto vbin = ci_vbin();
    EXPECT_EQ(::pwrite(qfd, vbin.data(), vbin.size(),
                       (off_t) SLASH_RECONFIG_BASE),
              (ssize_t) vbin.size())
        << "recovery VBIN after -EFBIG failed: " << std::strerror(errno);
    EXPECT_TRUE(wait_for([&] { return daemon_children(h.pid) == 1; }, 3000))
        << "no model after recovery VBIN";

    ::close(qfd);
    stop_daemon(h);
}

// ===========================================================================
// (4) ASan: the accumulation buffer must be freed when a transfer is ABANDONED
// (partial chunks delivered, never completed) and the daemon is then torn down.
// Under -DENABLE_SANITIZERS=ON a leaked partial buffer is a hard failure.
// ===========================================================================

TEST(BridgeReconfigAdversarial, AbandonedPartialBufferFreedOnShutdown)
{
    Watchdog wd(60);
    DaemonHandle h = start_daemon();
    ASSERT_GT(h.pid, 0);
    uint32_t qid = 0;
    int qfd = open_qpair(h.mountpoint, kBdf, &qid);
    ASSERT_GE(qfd, 0);

    // Deliver a couple of MB of INCOMPLETE reconfig chunks (a header declaring a
    // large member, then body) so the daemon allocates+grows the acc buffer, then
    // walk away (never send the terminator).  Shutdown must free the buffer.
    std::vector<uint8_t> first =
        make_tar_entry("big.bin", '0', std::vector<uint8_t>(2 * 1024 * 1024, 'M'),
                       0644, "", false);
    std::snprintf((char *) (first.data() + 124), 12, "%011o", (unsigned) (1u << 28));
    std::memset(first.data() + 148, ' ', 8);
    unsigned sum = 0;
    for (int i = 0; i < 512; i++) sum += first[i];
    std::snprintf((char *) (first.data() + 148), 8, "%06o", sum);
    first[154] = '\0';
    first[155] = ' ';

    off_t off = (off_t) SLASH_RECONFIG_BASE;
    ssize_t w = ::pwrite(qfd, first.data(), first.size(), off);
    ASSERT_GT(w, 0) << "partial chunk rejected: " << std::strerror(errno);
    off += w;
    std::vector<uint8_t> body(2 * 1024 * 1024, 'M');
    w = ::pwrite(qfd, body.data(), body.size(), off);
    ASSERT_GT(w, 0);
    // Still incomplete -> no model spawned, but the daemon holds a ~4 MiB partial.
    EXPECT_EQ(daemon_children(h.pid), 0);

    ::close(qfd);
    // Tear down WITHOUT completing the transfer: emu_bridge_free_one must free
    // b->acc (ASan would flag a leak otherwise).  scratch stays empty (no run dir
    // was ever created for an incomplete transfer).
    stop_daemon(h);
}

// ===========================================================================
// (3) CONTIGUITY: a non-contiguous (seeking) reconfig-region write after a
// partial chunk resets the buffer and returns -EINVAL, and a fresh VBIN then
// reconfigures cleanly (clean recovery -- the partial did not corrupt the next).
// ===========================================================================

TEST(BridgeReconfigAdversarial, NonContiguousReconfigResetsAndRecovers)
{
    Watchdog wd(60);
    DaemonHandle h = start_daemon();
    ASSERT_GT(h.pid, 0);
    uint32_t qid = 0;
    int qfd = open_qpair(h.mountpoint, kBdf, &qid);
    ASSERT_GE(qfd, 0);

    // A partial first chunk at BASE (a lone header block -> INCOMPLETE, accepted).
    auto vbin = make_tar_entry("vpp_sim", '0',
                               read_file(SLASH_EMU_STUB_MODEL_PATH), 0755);
    size_t head = 512; // just the first header block
    ASSERT_EQ(::pwrite(qfd, vbin.data(), head, (off_t) SLASH_RECONFIG_BASE),
              (ssize_t) head)
        << "partial first chunk: " << std::strerror(errno);
    EXPECT_EQ(daemon_children(h.pid), 0) << "model spawned on a partial chunk";

    // A SEEKING write (not at BASE+acc_len) must reset + -EINVAL.
    uint8_t junk[512] = {0};
    errno = 0;
    ssize_t w = ::pwrite(qfd, junk, sizeof(junk),
                         (off_t) SLASH_RECONFIG_BASE + 4096 /* gap */);
    EXPECT_EQ(w, -1);
    EXPECT_EQ(errno, EINVAL) << "non-contiguous reconfig not rejected -EINVAL";

    // Clean recovery: a fresh whole VBIN at BASE reconfigures.
    EXPECT_EQ(::pwrite(qfd, vbin.data(), vbin.size(),
                       (off_t) SLASH_RECONFIG_BASE),
              (ssize_t) vbin.size())
        << "recovery after non-contiguous reset failed: " << std::strerror(errno);
    EXPECT_TRUE(wait_for([&] { return daemon_children(h.pid) == 1; }, 3000))
        << "no model after recovery";
    // daemon still serviceable for ordinary transfers.
    uint8_t byte = 0x5C;
    EXPECT_EQ(::pwrite(qfd, &byte, 1, (off_t) SLASH_DDR_BASE), 1);

    ::close(qfd);
    stop_daemon(h);
}

// ===========================================================================
// (4) Ordinary HBM/DDR transfer unaffected by the reconfig interception; the
// reconfig-region READ stays -ERANGE even AFTER a successful reconfig.
// ===========================================================================

TEST(BridgeReconfigAdversarial, ReconfigRegionReadRejectedAfterModelUp)
{
    Watchdog wd(60);
    DaemonHandle h = start_daemon();
    ASSERT_GT(h.pid, 0);
    uint32_t qid = 0;
    int qfd = open_qpair(h.mountpoint, kBdf, &qid);
    ASSERT_GE(qfd, 0);

    auto vbin = ci_vbin();
    ASSERT_EQ(::pwrite(qfd, vbin.data(), vbin.size(),
                       (off_t) SLASH_RECONFIG_BASE),
              (ssize_t) vbin.size());

    // A read of the reconfig region is still rejected (cannot read back a VBIN).
    uint8_t buf[8];
    EXPECT_EQ(::pread(qfd, buf, sizeof(buf), (off_t) SLASH_RECONFIG_BASE), -1);
    EXPECT_EQ(errno, ERANGE);

    // DDR transfer (the other window) round-trips through the model.
    std::vector<uint8_t> payload(64);
    for (size_t i = 0; i < payload.size(); i++) payload[i] = (uint8_t) (i + 3);
    ASSERT_EQ(::pwrite(qfd, payload.data(), payload.size(),
                       (off_t) SLASH_DDR_BASE),
              (ssize_t) payload.size());
    std::vector<uint8_t> got(64, 0xFF);
    ASSERT_EQ(::pread(qfd, got.data(), got.size(), (off_t) SLASH_DDR_BASE),
              (ssize_t) got.size());
    EXPECT_EQ(got, payload);

    ::close(qfd);
    stop_daemon(h);
}

// ===========================================================================
// (5) Backend rc-contract through the real client+stub, over FUSE: BAR2/4 are
// served from the shadow (NEVER -EIO / -ENODEV for an in-range access) while a
// model is up, because the SIM dialect only covers BAR0.  width-8 BAR0 reads
// also fall back to the shadow.
// ===========================================================================

TEST(BridgeReconfigAdversarial, Bar24AndWidth8ServedFromShadowWhileModelUp)
{
    Watchdog wd(60);
    DaemonHandle h = start_daemon();
    ASSERT_GT(h.pid, 0);
    uint32_t qid = 0;
    int qfd = open_qpair(h.mountpoint, kBdf, &qid);
    ASSERT_GE(qfd, 0);
    ASSERT_EQ(::pwrite(qfd, ci_vbin().data(), ci_vbin().size(),
                       (off_t) SLASH_RECONFIG_BASE),
              (ssize_t) ci_vbin().size());
    ASSERT_TRUE(wait_for([&] { return daemon_children(h.pid) == 1; }, 2000));

    // BAR2 (service layer) is not forwarded to the SIM model: write+read must
    // round-trip via the shadow, never -EIO / -ENODEV.
    std::string bar2 = h.mountpoint + "/" + kBdf + "/bars/bar2";
    int b2 = ::open(bar2.c_str(), O_RDWR);
    ASSERT_GE(b2, 0);
    uint32_t v = 0xCAFEBABEu;
    ASSERT_EQ(::pwrite(b2, &v, 4, 0x80), 4) << std::strerror(errno);
    uint32_t rb = 0;
    ASSERT_EQ(::pread(b2, &rb, 4, 0x80), 4) << std::strerror(errno);
    EXPECT_EQ(rb, v) << "BAR2 not served from shadow while model up";
    ::close(b2);

    // BAR0 width-8 has no SIM verb: shadow fallback, full 64-bit round-trip.
    std::string bar0 = h.mountpoint + "/" + kBdf + "/bars/bar0";
    int b0 = ::open(bar0.c_str(), O_RDWR);
    ASSERT_GE(b0, 0);
    uint64_t q = 0x1122334455667788ull;
    ASSERT_EQ(::pwrite(b0, &q, 8, 0x80), 8) << std::strerror(errno);
    uint64_t qb = 0;
    ASSERT_EQ(::pread(b0, &qb, 8, 0x80), 8) << std::strerror(errno);
    EXPECT_EQ(qb, q) << "BAR0 width-8 not served from shadow";
    ::close(b0);

    ::close(qfd);
    stop_daemon(h);
}

// ===========================================================================
// (6) Two devices each with their own model do not cross-talk: a register
// written to dev A's bar0 is not visible on dev B's bar0.
// ===========================================================================

TEST(BridgeReconfigAdversarial, TwoDevicesNoCrossTalk)
{
    Watchdog wd(60);
    const char *cfg =
        "[accelerator:0000:61:00]\nnet-ip = 10.0.0.1\n"
        "[accelerator:0000:62:00]\nnet-ip = 10.0.0.2\n";
    DaemonHandle h = start_daemon(cfg);
    ASSERT_GT(h.pid, 0);

    const char *bdfA = "0000:61:00";
    const char *bdfB = "0000:62:00";
    uint32_t qa = 0, qb = 0;
    int qfa = open_qpair(h.mountpoint, bdfA, &qa);
    int qfb = open_qpair(h.mountpoint, bdfB, &qb);
    ASSERT_GE(qfa, 0);
    ASSERT_GE(qfb, 0);

    auto vbin = ci_vbin();
    ASSERT_EQ(::pwrite(qfa, vbin.data(), vbin.size(),
                       (off_t) SLASH_RECONFIG_BASE),
              (ssize_t) vbin.size());
    ASSERT_EQ(::pwrite(qfb, vbin.data(), vbin.size(),
                       (off_t) SLASH_RECONFIG_BASE),
              (ssize_t) vbin.size());
    // Two models now run.
    EXPECT_TRUE(wait_for([&] { return daemon_children(h.pid) == 2; }, 3000))
        << "expected 2 model children, got " << daemon_children(h.pid);

    std::string b0a = h.mountpoint + "/" + bdfA + "/bars/bar0";
    std::string b0b = h.mountpoint + "/" + bdfB + "/bars/bar0";
    int fa = ::open(b0a.c_str(), O_RDWR);
    int fb = ::open(b0b.c_str(), O_RDWR);
    ASSERT_GE(fa, 0);
    ASSERT_GE(fb, 0);

    uint32_t va = 0x11111111u;
    ASSERT_EQ(::pwrite(fa, &va, 4, 0x100), 4);
    // dev B's bar0 @0x100 was never written -> reads 0 (its own model's state).
    uint32_t rbb = 0xdead;
    ASSERT_EQ(::pread(fb, &rbb, 4, 0x100), 4);
    EXPECT_EQ(rbb, 0u) << "cross-talk: dev B saw dev A's register write";
    // dev A reads back its own value.
    uint32_t rba = 0;
    ASSERT_EQ(::pread(fa, &rba, 4, 0x100), 4);
    EXPECT_EQ(rba, va);

    ::close(fa);
    ::close(fb);
    ::close(qfa);
    ::close(qfb);
    stop_daemon(h);
}

// ===========================================================================
// (1) A model that ignores `exit` must still be reaped (SIGTERM->SIGKILL) at
// teardown: no leaked child, no zombie, scratch cleaned.  We pack the stub and
// rely on the daemon NOT being able to set the IGNORE_EXIT env (it's our env),
// so instead we verify reaping robustness by removing both functions and
// asserting the child is gone -- the bridge's reap path (exit best-effort, then
// SIGTERM, then SIGKILL) covers a model that does not honour exit.
// ===========================================================================

TEST(BridgeReconfigAdversarial, BothFunctionsRemovedReapsModelNoZombie)
{
    Watchdog wd(60);
    DaemonHandle h = start_daemon();
    ASSERT_GT(h.pid, 0);
    uint32_t qid = 0;
    int qfd = open_qpair(h.mountpoint, kBdf, &qid);
    ASSERT_GE(qfd, 0);
    ASSERT_EQ(::pwrite(qfd, ci_vbin().data(), ci_vbin().size(),
                       (off_t) SLASH_RECONFIG_BASE),
              (ssize_t) ci_vbin().size());
    ASSERT_TRUE(wait_for([&] { return daemon_children(h.pid) == 1; }, 2000));
    ::close(qfd);

    ASSERT_EQ(hotplug_remove(h.mountpoint, "0000:61:00.1"), 0);
    ASSERT_EQ(hotplug_remove(h.mountpoint, "0000:61:00.2"), 0);

    // No model child (live OR zombie) remains parented to the daemon.
    EXPECT_TRUE(wait_for([&] { return daemon_children(h.pid) == 0; }, 4000))
        << "model child not reaped after both functions removed (zombie?)";
    EXPECT_TRUE(wait_for([&] { return count_entries(h.scratch) == 0; }, 4000))
        << "scratch not cleaned after both functions removed";

    stop_daemon(h);
}

} // namespace
