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
 * @file bridge_integration_test.cpp
 * @brief End-to-end T10 bridge over a real FUSE mount.
 *
 * Drives the freshly-built daemon (the qdma_test.cpp fork+exec pattern) with its
 * model scratch root pointed under .tmp (SLASH_EMU_SCRATCH_ROOT).  The flow:
 *
 *   1. Write a CI VBIN (a ustar tar carrying the stub model as vpp_sim) into the
 *      reconfiguration region through a qpair<Q> -- a single pwrite at offset
 *      SLASH_RECONFIG_BASE -- which spawns the model and attaches the backends.
 *   2. Round-trip BAR registers through the model (write bar0 @ off, read back).
 *   3. Round-trip qpair MM transfers through the model (populate HBM, fetch).
 *   4. REMOVE both functions; assert the model child and its ipc:// socket and
 *      scratch dir are gone (no leaked process / socket / scratch).
 *
 * Also: a malformed VBIN fails the reconfig write without spawning anything and
 * without wedging the daemon; the daemon stays serviceable afterwards.
 */

#include <gtest/gtest.h>

#include <algorithm>
#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
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

#include "slash/uapi/slash_abi.h"

namespace {

constexpr int kMountTimeoutMs = 5000;
constexpr int kShutdownTimeoutMs = 5000;
constexpr int kPollIntervalMs = 50;
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

std::string make_scratch(const char *suffix)
{
    ::mkdir(SLASH_EMU_TMP_DIR, 0755);
    std::string tmpl =
        std::string(SLASH_EMU_TMP_DIR) + "/slash_emu_" + suffix + "_XXXXXX";
    std::vector<char> buf(tmpl.begin(), tmpl.end());
    buf.push_back('\0');
    char *r = ::mkdtemp(buf.data());
    EXPECT_NE(r, nullptr) << "mkdtemp: " << std::strerror(errno);
    return r ? std::string(r) : std::string();
}

std::string write_config()
{
    ::mkdir(SLASH_EMU_TMP_DIR, 0755);
    std::string tmpl = std::string(SLASH_EMU_TMP_DIR) + "/slash_emu_brcfg_XXXXXX";
    std::vector<char> buf(tmpl.begin(), tmpl.end());
    buf.push_back('\0');
    int fd = ::mkstemp(buf.data());
    EXPECT_GE(fd, 0);
    const char *cfg = "[accelerator:0000:61:00]\nnet-ip = 10.0.0.1\n";
    (void) ::write(fd, cfg, std::strlen(cfg));
    ::close(fd);
    return buf.data();
}

bool mount_is_ready(const std::string &mp)
{
    struct statfs sfs {};
    return ::statfs(mp.c_str(), &sfs) == 0 && sfs.f_type == FUSE_SUPER_MAGIC;
}

bool path_exists(const std::string &p)
{
    struct stat st {};
    return ::stat(p.c_str(), &st) == 0;
}

void rm_rf(const std::string &p) { (void) ::system(("rm -rf '" + p + "'").c_str()); }

// Count entries (excluding . / ..) under a directory; 0 if it doesn't exist.
int count_entries(const std::string &dir)
{
    DIR *d = ::opendir(dir.c_str());
    if (d == nullptr) {
        return 0;
    }
    int n = 0;
    struct dirent *e;
    while ((e = ::readdir(d)) != nullptr) {
        if (std::strcmp(e->d_name, ".") != 0 && std::strcmp(e->d_name, "..") != 0) {
            n++;
        }
    }
    ::closedir(d);
    return n;
}

// Append one ustar member (header + padded body) onto `out`.
void tar_append_member(std::vector<uint8_t> &out, const std::string &member,
                       const std::vector<uint8_t> &content, unsigned mode)
{
    size_t hoff = out.size();
    out.resize(hoff + 512, 0);
    auto *h = out.data() + hoff;
    std::snprintf((char *) h, 100, "%s", member.c_str());
    std::snprintf((char *) (h + 100), 8, "%07o", mode & 07777);
    std::snprintf((char *) (h + 108), 8, "%07o", 0);
    std::snprintf((char *) (h + 116), 8, "%07o", 0);
    std::snprintf((char *) (h + 124), 12, "%011o", (unsigned) content.size());
    std::snprintf((char *) (h + 136), 12, "%011o", 0);
    h[156] = '0';
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
    std::memcpy(out.data() + data, content.data(), content.size());
}

// Two trailing zero blocks = end-of-archive.
void tar_finalize(std::vector<uint8_t> &out) { out.resize(out.size() + 1024, 0); }

// Build a single-member ustar archive (used by the small / error-path tests).
std::vector<uint8_t> make_tar(const std::string &member,
                              const std::vector<uint8_t> &content, unsigned mode)
{
    std::vector<uint8_t> out;
    tar_append_member(out, member, content, mode);
    tar_finalize(out);
    return out;
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
    return make_tar("vpp_sim", read_file(SLASH_EMU_STUB_MODEL_PATH), 0755);
}

// A production-shaped large VBIN: the stub as vpp_sim plus a filler member so the
// whole archive is at least `min_bytes` (forces the kernel to split a single
// pwrite into several reconfig-region chunks, mirroring a real multi-MB VBIN).
std::vector<uint8_t> large_ci_vbin(size_t min_bytes)
{
    std::vector<uint8_t> out;
    tar_append_member(out, "vpp_sim", read_file(SLASH_EMU_STUB_MODEL_PATH), 0755);
    if (out.size() < min_bytes) {
        // Deterministic filler content (so any accidental truncation shows up).
        std::vector<uint8_t> filler(min_bytes - out.size());
        for (size_t i = 0; i < filler.size(); i++) {
            filler[i] = (uint8_t) (i * 31 + 7);
        }
        tar_append_member(out, "padding.bin", filler, 0644);
    }
    tar_finalize(out);
    return out;
}

// Open the qdma/ dir and QPAIR_ADD; return the opened qpair fd (or -1) + qid.
int open_qpair(const std::string &mnt, uint32_t *qid)
{
    std::string qdma = mnt + "/" + kBdf + "/qdma";
    int dfd = ::open(qdma.c_str(), O_RDONLY | O_DIRECTORY);
    if (dfd < 0) {
        return -1;
    }
    struct slash_abi_qdma_qpair_add req {};
    req.size = sizeof(req);
    req.mode = 0;
    req.dir_mask = 0x3; // H2C | C2H
    int rc = ::ioctl(dfd, SLASH_ABI_QDMA_IOCTL_QPAIR_ADD, &req);
    ::close(dfd);
    if (rc != 0) {
        return -1;
    }
    *qid = req.qid;
    char name[4096];
    std::snprintf(name, sizeof(name), "%s/%s/qdma/qpair%u", mnt.c_str(), kBdf,
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

// Fork+exec the daemon with SLASH_EMU_SCRATCH_ROOT set, run body(mnt, scratch),
// then SIGTERM+reap.  body returns nothing; failures use GTest macros.
template <typename Body>
void with_daemon(Body body)
{
    const std::string mountpoint = make_scratch("brmnt");
    const std::string scratch = make_scratch("brscratch");
    ASSERT_FALSE(mountpoint.empty());
    ASSERT_FALSE(scratch.empty());
    const std::string config = write_config();

    pid_t pid = ::fork();
    ASSERT_NE(pid, -1);
    if (pid == 0) {
        ::setenv("SLASH_EMU_SCRATCH_ROOT", scratch.c_str(), 1);
        ::execl(SLASH_EMUD_PATH, "slash-emud", "--config", config.c_str(),
                "--mount", mountpoint.c_str(), (char *) nullptr);
        ::_exit(127);
    }

    bool ready =
        wait_for([&] { return mount_is_ready(mountpoint); }, kMountTimeoutMs);
    if (!ready) {
        ::kill(pid, SIGKILL);
        ::waitpid(pid, nullptr, 0);
        rm_rf(mountpoint);
        rm_rf(scratch);
        ::unlink(config.c_str());
        FAIL() << "daemon did not mount";
    }

    body(mountpoint, scratch);

    ::kill(pid, SIGTERM);
    int status = 0;
    bool exited = wait_for(
        [&] { return ::waitpid(pid, &status, WNOHANG) == pid; },
        kShutdownTimeoutMs);
    if (!exited) {
        ::kill(pid, SIGKILL);
        ::waitpid(pid, &status, 0);
    }
    EXPECT_TRUE(exited);

    // After daemon shutdown the scratch root must be empty (every model run dir
    // was cleaned up by the bridge teardown).
    EXPECT_EQ(count_entries(scratch), 0)
        << "scratch root not cleaned after daemon shutdown";

    ::rmdir((mountpoint).c_str());
    rm_rf(mountpoint);
    rm_rf(scratch);
    ::unlink(config.c_str());
}

// ===========================================================================

TEST(BridgeIntegration, ReconfigSpawnRoundTripTeardown)
{
    with_daemon([](const std::string &mnt, const std::string &scratch) {
        uint32_t qid = 0;
        int qfd = open_qpair(mnt, &qid);
        ASSERT_GE(qfd, 0) << "QPAIR_ADD/open: " << std::strerror(errno);

        // 1. Deliver the whole VBIN in one write to the reconfig region.
        std::vector<uint8_t> vbin = ci_vbin();
        ssize_t w = ::pwrite(qfd, vbin.data(), vbin.size(),
                             (off_t) SLASH_RECONFIG_BASE);
        ASSERT_EQ(w, (ssize_t) vbin.size())
            << "reconfig write: " << std::strerror(errno);

        // The model is up: a run dir (with model.sock) exists under scratch.
        EXPECT_GT(count_entries(scratch), 0) << "no model run dir after reconfig";

        // 2. BAR register round-trip THROUGH the model.
        std::string bar0 = mnt + "/" + kBdf + "/bars/bar0";
        int bfd = ::open(bar0.c_str(), O_RDWR);
        ASSERT_GE(bfd, 0);
        uint32_t regval = 0x1234abcdu;
        ASSERT_EQ(::pwrite(bfd, &regval, 4, 0x40), 4)
            << "bar write: " << std::strerror(errno);
        uint32_t readback = 0;
        ASSERT_EQ(::pread(bfd, &readback, 4, 0x40), 4)
            << "bar read: " << std::strerror(errno);
        EXPECT_EQ(readback, regval);
        ::close(bfd);

        // 3. QPAIR MM round-trip THROUGH the model (HBM).
        std::vector<uint8_t> payload(512);
        for (size_t i = 0; i < payload.size(); i++) {
            payload[i] = (uint8_t) (i ^ 0x5A);
        }
        ASSERT_EQ(::pwrite(qfd, payload.data(), payload.size(),
                           (off_t) SLASH_HBM_BASE),
                  (ssize_t) payload.size())
            << "qpair write: " << std::strerror(errno);
        std::vector<uint8_t> got(512, 0xFF);
        ASSERT_EQ(::pread(qfd, got.data(), got.size(), (off_t) SLASH_HBM_BASE),
                  (ssize_t) got.size())
            << "qpair read: " << std::strerror(errno);
        EXPECT_EQ(got, payload);

        ::close(qfd);

        // 4. REMOVE both functions -> the model is torn down.
        ASSERT_EQ(hotplug_remove(mnt, "0000:61:00.1"), 0);
        ASSERT_EQ(hotplug_remove(mnt, "0000:61:00.2"), 0);

        // The model run dir (and its socket) must be gone after teardown.
        EXPECT_TRUE(wait_for([&] { return count_entries(scratch) == 0; }, 3000))
            << "model run dir not cleaned after both functions removed";
    });
}

TEST(BridgeIntegration, MalformedVbinFailsWriteNoSpawnNoWedge)
{
    with_daemon([](const std::string &mnt, const std::string &scratch) {
        uint32_t qid = 0;
        int qfd = open_qpair(mnt, &qid);
        ASSERT_GE(qfd, 0);

        // A full block of non-zero bytes without the ustar magic is structurally
        // INVALID (no further bytes can fix it) -> the write fails up front, and
        // no model is spawned (scratch stays empty).
        std::vector<uint8_t> junk(512, 0xAB);
        ssize_t w = ::pwrite(qfd, junk.data(), junk.size(),
                             (off_t) SLASH_RECONFIG_BASE);
        EXPECT_EQ(w, -1);
        EXPECT_EQ(errno, EINVAL);
        EXPECT_EQ(count_entries(scratch), 0) << "model spawned for a bad VBIN";

        // The daemon stays serviceable: an ordinary HBM MM transfer still works.
        uint8_t byte = 0x99;
        EXPECT_EQ(::pwrite(qfd, &byte, 1, (off_t) SLASH_HBM_BASE), 1)
            << "daemon wedged after a bad VBIN: " << std::strerror(errno);
        uint8_t back = 0;
        EXPECT_EQ(::pread(qfd, &back, 1, (off_t) SLASH_HBM_BASE), 1);
        EXPECT_EQ(back, byte);

        ::close(qfd);
    });
}

TEST(BridgeIntegration, ReconfigRegionReadStillRejected)
{
    with_daemon([](const std::string &mnt, const std::string &scratch) {
        (void) scratch;
        uint32_t qid = 0;
        int qfd = open_qpair(mnt, &qid);
        ASSERT_GE(qfd, 0);

        // A READ of the reconfig region is not a VBIN op; it stays -ERANGE.
        uint8_t buf[8];
        EXPECT_EQ(::pread(qfd, buf, sizeof(buf), (off_t) SLASH_RECONFIG_BASE), -1);
        EXPECT_EQ(errno, ERANGE);

        ::close(qfd);
    });
}

// Drive a BAR + HBM round-trip through the model (used by the spawn tests).
void assert_data_plane_works(const std::string &mnt, int qfd)
{
    std::string bar0 = mnt + "/" + kBdf + "/bars/bar0";
    int bfd = ::open(bar0.c_str(), O_RDWR);
    ASSERT_GE(bfd, 0);
    uint32_t v = 0xfeedface, r = 0;
    ASSERT_EQ(::pwrite(bfd, &v, 4, 0x80), 4);
    ASSERT_EQ(::pread(bfd, &r, 4, 0x80), 4);
    EXPECT_EQ(r, v);
    ::close(bfd);

    std::vector<uint8_t> out(1024), in(1024, 0);
    for (size_t i = 0; i < out.size(); i++) {
        out[i] = (uint8_t) (i * 13 + 5);
    }
    ASSERT_EQ(::pwrite(qfd, out.data(), out.size(), (off_t) SLASH_DDR_BASE),
              (ssize_t) out.size());
    ASSERT_EQ(::pread(qfd, in.data(), in.size(), (off_t) SLASH_DDR_BASE),
              (ssize_t) in.size());
    EXPECT_EQ(in, out);
}

// The flipped bug-marker: a VBIN delivered in MANY explicit contiguous chunks
// (each a separate pwrite at BASE + running offset) must reassemble, spawn the
// model, and round-trip the data plane.  This deterministically exercises the
// append path regardless of how the kernel happens to split writes.
TEST(BridgeIntegration, MultiChunkVbinReassemblesAndSpawns)
{
    with_daemon([](const std::string &mnt, const std::string &scratch) {
        uint32_t qid = 0;
        int qfd = open_qpair(mnt, &qid);
        ASSERT_GE(qfd, 0);

        std::vector<uint8_t> vbin = ci_vbin();
        ASSERT_GT(vbin.size(), 4096u);

        // Write in 4 KiB contiguous chunks.
        const size_t chunk = 4096;
        size_t off = 0;
        while (off < vbin.size()) {
            size_t n = std::min(chunk, vbin.size() - off);
            ssize_t w = ::pwrite(qfd, vbin.data() + off, n,
                                 (off_t) (SLASH_RECONFIG_BASE + off));
            ASSERT_EQ(w, (ssize_t) n)
                << "chunk @" << off << ": " << std::strerror(errno);
            off += n;
        }

        EXPECT_GT(count_entries(scratch), 0) << "model not spawned after chunks";
        assert_data_plane_works(mnt, qfd);

        ::close(qfd);
        ASSERT_EQ(hotplug_remove(mnt, "0000:61:00.1"), 0);
        ASSERT_EQ(hotplug_remove(mnt, "0000:61:00.2"), 0);
        EXPECT_TRUE(wait_for([&] { return count_entries(scratch) == 0; }, 3000));
    });
}

// Production-shaped: a genuinely large (>max_write) VBIN written in a SINGLE
// pwrite, which the kernel splits into several reconfig-region chunks.  This is
// the case the tiny CI stub previously hid; it must spawn and round-trip.
TEST(BridgeIntegration, LargeSingleWriteVbinSplitByKernelReassembles)
{
    with_daemon([](const std::string &mnt, const std::string &scratch) {
        uint32_t qid = 0;
        int qfd = open_qpair(mnt, &qid);
        ASSERT_GE(qfd, 0);

        // ~12 MiB: well above any FUSE max_write, so the kernel WILL split it.
        std::vector<uint8_t> vbin = large_ci_vbin(12u * 1024u * 1024u);
        ASSERT_GT(vbin.size(), 8u * 1024u * 1024u);

        ssize_t w = ::pwrite(qfd, vbin.data(), vbin.size(),
                             (off_t) SLASH_RECONFIG_BASE);
        ASSERT_EQ(w, (ssize_t) vbin.size())
            << "large reconfig write: " << std::strerror(errno);

        EXPECT_GT(count_entries(scratch), 0)
            << "model not spawned for a large multi-chunk VBIN";
        assert_data_plane_works(mnt, qfd);

        ::close(qfd);
        ASSERT_EQ(hotplug_remove(mnt, "0000:61:00.1"), 0);
        ASSERT_EQ(hotplug_remove(mnt, "0000:61:00.2"), 0);
        EXPECT_TRUE(wait_for([&] { return count_entries(scratch) == 0; }, 3000));
    });
}

// A non-contiguous (seeking) write into the reconfig region after a partial chunk
// is rejected (-EINVAL) and resets the transfer; the daemon stays serviceable and
// a fresh full VBIN at BASE afterwards still spawns.
TEST(BridgeIntegration, NonContiguousReconfigWriteRejectedThenRecovers)
{
    with_daemon([](const std::string &mnt, const std::string &scratch) {
        uint32_t qid = 0;
        int qfd = open_qpair(mnt, &qid);
        ASSERT_GE(qfd, 0);

        std::vector<uint8_t> vbin = ci_vbin();
        ASSERT_GT(vbin.size(), 8192u);

        // First chunk at BASE (incomplete -> accepted).
        ASSERT_EQ(::pwrite(qfd, vbin.data(), 4096, (off_t) SLASH_RECONFIG_BASE),
                  4096);
        // A seeking write (gap) -> rejected, transfer reset.
        ssize_t w = ::pwrite(qfd, vbin.data() + 4096, 4096,
                             (off_t) (SLASH_RECONFIG_BASE + 8192));
        EXPECT_EQ(w, -1);
        EXPECT_EQ(errno, EINVAL);
        EXPECT_EQ(count_entries(scratch), 0) << "model spawned despite reset";

        // Recovery: a fresh full VBIN starting at BASE spawns normally.
        ASSERT_EQ(::pwrite(qfd, vbin.data(), vbin.size(),
                           (off_t) SLASH_RECONFIG_BASE),
                  (ssize_t) vbin.size());
        EXPECT_GT(count_entries(scratch), 0) << "recovery reconfig did not spawn";
        assert_data_plane_works(mnt, qfd);

        ::close(qfd);
        ASSERT_EQ(hotplug_remove(mnt, "0000:61:00.1"), 0);
        ASSERT_EQ(hotplug_remove(mnt, "0000:61:00.2"), 0);
        EXPECT_TRUE(wait_for([&] { return count_entries(scratch) == 0; }, 3000));
    });
}

// A fresh VBIN at BASE after a completed reconfiguration is a NEW reconfiguration:
// the prior model is torn down and a new one comes up.  No process/scratch leak.
TEST(BridgeIntegration, ReVbinAfterCompletionReplacesModel)
{
    with_daemon([](const std::string &mnt, const std::string &scratch) {
        uint32_t qid = 0;
        int qfd = open_qpair(mnt, &qid);
        ASSERT_GE(qfd, 0);

        std::vector<uint8_t> vbin = ci_vbin();
        ASSERT_EQ(::pwrite(qfd, vbin.data(), vbin.size(),
                           (off_t) SLASH_RECONFIG_BASE),
                  (ssize_t) vbin.size());
        EXPECT_EQ(count_entries(scratch), 1) << "first model run dir";
        assert_data_plane_works(mnt, qfd);

        // Second reconfiguration: replaces the model; still exactly one run dir.
        ASSERT_EQ(::pwrite(qfd, vbin.data(), vbin.size(),
                           (off_t) SLASH_RECONFIG_BASE),
                  (ssize_t) vbin.size());
        EXPECT_EQ(count_entries(scratch), 1)
            << "re-VBIN must replace, not accumulate, model run dirs";
        assert_data_plane_works(mnt, qfd);

        ::close(qfd);
        ASSERT_EQ(hotplug_remove(mnt, "0000:61:00.1"), 0);
        ASSERT_EQ(hotplug_remove(mnt, "0000:61:00.2"), 0);
        EXPECT_TRUE(wait_for([&] { return count_entries(scratch) == 0; }, 3000));
    });
}

} // namespace
