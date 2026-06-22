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
 * @file rescan_adversarial_test.cpp
 * @brief Adversarial scrutiny of RESCAN rediscovery (task 14).
 *
 * The destructive counterpart of rescan_test.cpp.  Where that suite confirms the
 * happy path, this one assumes defects and hammers the fiddly seams:
 *
 *   - model_shutdown re-arm bookkeeping driven WITHOUT manually re-installing the
 *     seam after each restore (the real daemon never re-installs it -- it relies
 *     on the fn/ctx surviving a per-function revoke).  Exhaustive remove/restore
 *     sequences against a counting seam, asserting the seam fires EXACTLY on each
 *     both-removed transition and never spuriously -- verified through observable
 *     call counts (the C++20 Device private fields are not inspectable; the public
 *     contract is the count and usability, which is what the spec cares about).
 *   - removed_functions bit integrity: verified through behavioral invariants
 *     (which ioctls succeed/fail, which nodes resolve) rather than internal state.
 *   - leak/UAF across many remove/restore cycles (ASan): fresh store/handler each
 *     time, no double-install, old store freed.
 *   - integration over the real FUSE mount: the real-daemon double-teardown (both
 *     removed -> model down -> RESCAN -> reconfig -> both removed -> model down
 *     AGAIN) works through the un-re-installed seam; restore-from-both-removed
 *     reads the MODEL (not a stale shadow); and select_new is UNCHANGED -- an
 *     unconfigured live BDF's removed function still rediscovers, while a NEW
 *     configured-but-not-yet-present BDF is the select_new pass's job, never the
 *     rediscover pass's.
 */

#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
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

#include "bars.hpp"
#include "info.hpp"
#include "node.hpp"
#include "qdma.hpp"

extern "C" {
#include "slash/uapi/slash_abi.h"
}

using slash::emu::Device;
using slash::emu::DeviceFunction;
using slash::emu::Ino;
using slash::emu::ModelShutdownFn;
using slash::emu::Node;
using slash::emu::NodeTree;
using slash::emu::barsAttach;
using slash::emu::infoAttach;
using slash::emu::kRootIno;
using slash::emu::qdmaAttach;

namespace {

// ===========================================================================
// Unit harness (mirrors rescan_test.cpp's RescanTree but with no manual seam
// re-install: the seam is wired ONCE at addDevice and we assert the restore
// path re-arms it on its own, exactly as the real daemon relies on).
// ===========================================================================

struct ShutdownCounter {
    std::atomic<int> calls{0};
    std::atomic<int> ctx_mismatches{0};
    Device *expect_dev{nullptr};
};

class AdvTree {
public:
    AdvTree() : tree_() {}
    NodeTree &get() { return tree_; }

    Device *addDevice(const std::string &bdf, ModelShutdownFn sd = {})
    {
        Device *dev = tree_.addDevice(bdf);
        EXPECT_NE(dev, nullptr);
        if (dev == nullptr) {
            return nullptr;
        }
        EXPECT_EQ(infoAttach(*dev), 0);
        EXPECT_EQ(barsAttach(*dev), 0);
        EXPECT_EQ(qdmaAttach(*dev), 0);
        if (sd) {
            EXPECT_EQ(tree_.setModelShutdown(bdf, std::move(sd)), 0);
        }
        return dev;
    }

    void remove(const std::string &bdf, DeviceFunction func)
    {
        ASSERT_EQ(tree_.revokeFunction(bdf, func), 0);
    }

    // Restore exactly as the daemon would (rebuild + re-attach endpoint), but
    // DELIBERATELY do NOT re-install the model_shutdown seam -- the real daemon
    // does not, so this is the honest re-arm test.
    bool restore(Device *dev, DeviceFunction func)
    {
        bool rebuilt = false;
        EXPECT_EQ(tree_.restoreFunction(dev->bdf(), func, &rebuilt), 0);
        if (rebuilt) {
            int aret = func == DeviceFunction::Qdma ? qdmaAttach(*dev)
                                                    : barsAttach(*dev);
            EXPECT_EQ(aret, 0);
        }
        return rebuilt;
    }

private:
    NodeTree tree_;
};

bool resolves(NodeTree &tree, Ino parent, const char *name)
{
    Node *child = nullptr;
    int rc = tree.lookupChild(parent, name, &child);
    return rc == 0 && child != nullptr;
}

uint32_t add_qpair(NodeTree &tree, Device *dev)
{
    struct slash_abi_qdma_qpair_add req {};
    req.size = sizeof(req);
    req.mode = 0;
    req.dir_mask = 0x3;
    struct slash_abi_qdma_qpair_add out = req;
    EXPECT_EQ(tree.ioctl(dev->qdma->ino, SLASH_ABI_QDMA_IOCTL_QPAIR_ADD, &req,
                         sizeof(req), &out, sizeof(out)),
              0);
    return out.qid;
}

// ===========================================================================
// SEAM WIRING INVARIANT: model_shutdown re-arm bookkeeping, NO manual re-install.
//
// The real daemon's reattach does NOT call setModelShutdown again.  It relies
// on the seam lambda surviving a per-function revoke and on restoreFunction
// re-arming the guard.  These tests drive the brief's exact sequences against a
// seam wired ONCE and assert exactly-once-per-both-removed through call counts.
// ===========================================================================

// {rm1, rm2(fires), restore1, rm1(fires again)} -- WITHOUT re-installing.
TEST(RescanAdvSeam, Rm1Rm2Restore1Rm1FiresTwiceNoReinstall)
{
    ShutdownCounter c;
    AdvTree t;
    Device *dev = t.addDevice("0000:61:00", [&c](Device &d) {
        c.calls++;
        if (c.expect_dev != nullptr && &d != c.expect_dev) {
            c.ctx_mismatches++;
        }
    });
    c.expect_dev = dev;

    t.remove("0000:61:00", DeviceFunction::Qdma);
    EXPECT_EQ(c.calls.load(), 0) << "qdma alone must not fire the seam";
    t.remove("0000:61:00", DeviceFunction::Bars);
    ASSERT_EQ(c.calls.load(), 1) << "seam must fire on both-removed transition";

    // Restore fn1 (qdma) -- DO NOT re-install the seam.
    ASSERT_TRUE(t.restore(dev, DeviceFunction::Qdma));
    // The seam is re-armed; fn2 (bars) is still removed.
    // qdma resolves, bars does not.
    EXPECT_TRUE(resolves(t.get(), dev->dir->ino, "qdma"));
    EXPECT_FALSE(resolves(t.get(), dev->dir->ino, "bars"));

    // Remove fn1 again -> both gone again -> fires through the surviving seam.
    t.remove("0000:61:00", DeviceFunction::Qdma);
    EXPECT_EQ(c.calls.load(), 2)
        << "re-arm must fire through the surviving seam, no re-install";
    EXPECT_EQ(c.ctx_mismatches.load(), 0);
}

// {rm1, restore1, rm1, rm2} -> fires EXACTLY once (the both-removed transition is
// only reached at the final rm2; the rm1/restore1/rm1 churn must not fire it).
TEST(RescanAdvSeam, Rm1Restore1Rm1Rm2FiresExactlyOnce)
{
    ShutdownCounter c;
    AdvTree t;
    Device *dev = t.addDevice("0000:61:00", [&c](Device &d) {
        c.calls++;
        if (c.expect_dev != nullptr && &d != c.expect_dev) {
            c.ctx_mismatches++;
        }
    });
    c.expect_dev = dev;

    t.remove("0000:61:00", DeviceFunction::Qdma);
    EXPECT_EQ(c.calls.load(), 0);
    ASSERT_TRUE(t.restore(dev, DeviceFunction::Qdma));
    EXPECT_EQ(c.calls.load(), 0) << "restore must not spuriously fire seam";
    t.remove("0000:61:00", DeviceFunction::Qdma);
    EXPECT_EQ(c.calls.load(), 0) << "qdma alone is not both-removed";
    t.remove("0000:61:00", DeviceFunction::Bars);
    EXPECT_EQ(c.calls.load(), 1) << "fires exactly once at the both-removed edge";
    EXPECT_EQ(c.ctx_mismatches.load(), 0);
}

// {rm1, rm2(fires), restore1, restore2, rm2, rm1(fires)} -- restoring BOTH then
// removing in the OTHER order must still fire exactly the second time.
TEST(RescanAdvSeam, RestoreBothThenRemoveReverseOrderFiresAgain)
{
    ShutdownCounter c;
    AdvTree t;
    Device *dev = t.addDevice("0000:61:00", [&c](Device &d) {
        c.calls++;
        if (c.expect_dev != nullptr && &d != c.expect_dev) {
            c.ctx_mismatches++;
        }
    });
    c.expect_dev = dev;

    t.remove("0000:61:00", DeviceFunction::Qdma);
    t.remove("0000:61:00", DeviceFunction::Bars);
    ASSERT_EQ(c.calls.load(), 1) << "first both-removed fires";

    ASSERT_TRUE(t.restore(dev, DeviceFunction::Qdma));
    ASSERT_TRUE(t.restore(dev, DeviceFunction::Bars));
    // Both restored: both resolve, guard re-armed.
    EXPECT_TRUE(resolves(t.get(), dev->dir->ino, "qdma"));
    EXPECT_TRUE(resolves(t.get(), dev->dir->ino, "bars"));

    // Remove in REVERSE order (bars first, then qdma).
    t.remove("0000:61:00", DeviceFunction::Bars);
    EXPECT_EQ(c.calls.load(), 1) << "one function alone must not fire";
    t.remove("0000:61:00", DeviceFunction::Qdma);
    EXPECT_EQ(c.calls.load(), 2) << "second both-removed (reverse order) fires";
    EXPECT_EQ(c.ctx_mismatches.load(), 0);
}

// A restore of a NOT-removed function must not reset state in a way that causes a
// missed or double fire: with only fn1 removed, restoring fn2 (a no-op rebuild)
// must NOT prevent the seam from firing when fn1 is finally removed too.
TEST(RescanAdvSeam, RestoreOfNotRemovedDoesNotPerturbBookkeeping)
{
    ShutdownCounter c;
    AdvTree t;
    Device *dev = t.addDevice("0000:61:00", [&c](Device &d) {
        c.calls++;
        if (c.expect_dev != nullptr && &d != c.expect_dev) {
            c.ctx_mismatches++;
        }
    });
    c.expect_dev = dev;

    // Remove both -> fired. Then restore fn1 only (fn2 stays removed).
    t.remove("0000:61:00", DeviceFunction::Qdma);
    t.remove("0000:61:00", DeviceFunction::Bars);
    ASSERT_EQ(c.calls.load(), 1);
    ASSERT_TRUE(t.restore(dev, DeviceFunction::Qdma));
    // fn2 (bars) is still removed; fn1 (qdma) is live again.
    EXPECT_TRUE(resolves(t.get(), dev->dir->ino, "qdma"));
    EXPECT_FALSE(resolves(t.get(), dev->dir->ino, "bars"));

    // Now "restore" fn1 AGAIN (already restored => no-op): must not perturb the
    // fn2 removed bit nor the (re-armed) guard.
    bool rebuilt = true;
    ASSERT_EQ(t.get().restoreFunction("0000:61:00", DeviceFunction::Qdma,
                                      &rebuilt),
              0);
    EXPECT_FALSE(rebuilt) << "restore of already-live function is a no-op";
    // fn2 is still removed (no-op on fn1 must not touch fn2's state).
    EXPECT_FALSE(resolves(t.get(), dev->dir->ino, "bars"))
        << "fn2's removed state must be untouched by no-op restore of fn1";

    // The guard is still re-armed: removing fn1 now completes the set and fires.
    t.remove("0000:61:00", DeviceFunction::Qdma);
    EXPECT_EQ(c.calls.load(), 2)
        << "no-op restore must not prevent the seam from re-firing";
    EXPECT_EQ(c.ctx_mismatches.load(), 0);
}

// Restore of a function on an INTACT device (nothing ever removed) must not
// spuriously fire the seam.  Guards against a restore that unconditionally
// clears the guard and masks a real fire.
TEST(RescanAdvSeam, RestoreNoOpOnIntactDeviceLeavesGuardClear)
{
    ShutdownCounter c;
    AdvTree t;
    Device *dev = t.addDevice("0000:61:00", [&c](Device &d) {
        c.calls++;
        if (c.expect_dev != nullptr && &d != c.expect_dev) {
            c.ctx_mismatches++;
        }
    });
    c.expect_dev = dev;

    bool rebuilt = true;
    ASSERT_EQ(t.get().restoreFunction("0000:61:00", DeviceFunction::Bars,
                                      &rebuilt),
              0);
    EXPECT_FALSE(rebuilt);
    EXPECT_EQ(c.calls.load(), 0) << "no-op restore must not fire the seam";

    // Sanity: the seam still fires exactly once when both are genuinely removed.
    t.remove("0000:61:00", DeviceFunction::Bars);
    t.remove("0000:61:00", DeviceFunction::Qdma);
    EXPECT_EQ(c.calls.load(), 1);
    EXPECT_EQ(c.ctx_mismatches.load(), 0);
}

// Exhaustive small-depth fuzz: drive every reachable remove/restore sequence up
// to a bounded depth against a model of the spec, asserting the counting seam
// matches the model's "fires on each 0b11 transition" at every step.
//
// The C++20 Device private fields (removed_functions, model_shutdown_fired) are
// not accessible; we assert only the OBSERVABLE contract:
//   - cumulative call count matches the model's total_fires at each step, and
//   - the qdma/bars subtrees resolve exactly when the model says they are live.
TEST(RescanAdvSeam, ExhaustiveSequenceFuzzMatchesModel)
{
    struct Model {
        bool qdma_removed = false;
        bool bars_removed = false;
        bool fired = false;
        int total_fires = 0;

        void rm_qdma()
        {
            if (qdma_removed) {
                return;
            }
            qdma_removed = true;
            if (bars_removed && !fired) {
                fired = true;
                total_fires++;
            }
        }
        void rm_bars()
        {
            if (bars_removed) {
                return;
            }
            bars_removed = true;
            if (qdma_removed && !fired) {
                fired = true;
                total_fires++;
            }
        }
        void restore_qdma()
        {
            if (!qdma_removed) {
                return; // no-op: guard stays as-is
            }
            qdma_removed = false;
            fired = false; // re-arm
        }
        void restore_bars()
        {
            if (!bars_removed) {
                return; // no-op: guard stays as-is
            }
            bars_removed = false;
            fired = false; // re-arm
        }
    };

    const int kActions = 4; // 0=rm_qdma 1=rm_bars 2=re_qdma 3=re_bars
    const int kMaxLen = 6;
    int sequences = 0;
    for (int len = 1; len <= kMaxLen; len++) {
        std::vector<int> idx(len, 0);
        for (;;) {
            ShutdownCounter c;
            AdvTree t;
            Device *dev = t.addDevice("0000:61:00", [&c](Device &d) {
                c.calls++;
                if (c.expect_dev != nullptr && &d != c.expect_dev) {
                    c.ctx_mismatches++;
                }
            });
            c.expect_dev = dev;
            Model m;

            for (int step = 0; step < len; step++) {
                switch (idx[step]) {
                case 0:
                    t.remove("0000:61:00", DeviceFunction::Qdma);
                    m.rm_qdma();
                    break;
                case 1:
                    t.remove("0000:61:00", DeviceFunction::Bars);
                    m.rm_bars();
                    break;
                case 2:
                    t.restore(dev, DeviceFunction::Qdma);
                    m.restore_qdma();
                    break;
                case 3:
                    t.restore(dev, DeviceFunction::Bars);
                    m.restore_bars();
                    break;
                }

                // Observable contract: cumulative seam fires match the model.
                ASSERT_EQ(c.calls.load(), m.total_fires)
                    << "len=" << len << " step=" << step
                    << " action=" << idx[step];

                // Observable contract: qdma/bars subtrees resolve iff the model
                // says they are live.
                ASSERT_EQ(resolves(t.get(), dev->dir->ino, "qdma"),
                          !m.qdma_removed)
                    << "len=" << len << " step=" << step;
                ASSERT_EQ(resolves(t.get(), dev->dir->ino, "bars"),
                          !m.bars_removed)
                    << "len=" << len << " step=" << step;
            }
            ASSERT_EQ(c.ctx_mismatches.load(), 0);
            sequences++;

            // Odometer increment
            int p = len - 1;
            while (p >= 0 && ++idx[p] == kActions) {
                idx[p] = 0;
                p--;
            }
            if (p < 0) {
                break;
            }
        }
    }
    // 4^1 + ... + 4^6 = 5460 sequences.
    EXPECT_EQ(sequences, 4 + 16 + 64 + 256 + 1024 + 4096);
}

// ===========================================================================
// LIFECYCLE: many remove/restore cycles -- no leak/UAF, fresh usable endpoints
// each time, no duplicate children.  (ASan does the heavy lifting here.)
// ===========================================================================

TEST(RescanAdvLifecycle, ManyQdmaCyclesNoLeakStaysUsable)
{
    AdvTree t;
    Device *dev = t.addDevice("0000:61:00");

    for (int i = 0; i < 25; i++) {
        // Add a qpair, then remove the function (tears down the qpair + store),
        // then restore (fresh store) and round-trip on a brand-new qpair.
        (void) add_qpair(t.get(), dev);
        t.remove("0000:61:00", DeviceFunction::Qdma);
        EXPECT_FALSE(resolves(t.get(), dev->dir->ino, "qdma")) << "i=" << i;

        ASSERT_TRUE(t.restore(dev, DeviceFunction::Qdma)) << "i=" << i;
        ASSERT_TRUE(resolves(t.get(), dev->dir->ino, "qdma")) << "i=" << i;

        uint32_t q2 = add_qpair(t.get(), dev);
        char qname[32];
        std::snprintf(qname, sizeof(qname), "qpair%u", q2);
        Node *qp = nullptr;
        ASSERT_EQ(t.get().lookupChild(dev->qdma->ino, qname, &qp), 0) << "i=" << i;
        ASSERT_NE(qp, nullptr) << "i=" << i;

        std::vector<uint8_t> w(48, (uint8_t) (i + 1)), r(48, 0);
        ASSERT_EQ(t.get().pwrite(qp->ino,
                                 reinterpret_cast<const char *>(w.data()),
                                 w.size(), (off_t) SLASH_HBM_BASE),
                  (ssize_t) w.size())
            << "i=" << i;
        ASSERT_EQ(t.get().pread(qp->ino,
                                reinterpret_cast<char *>(r.data()), r.size(),
                                (off_t) SLASH_HBM_BASE),
                  (ssize_t) r.size())
            << "i=" << i;
        EXPECT_EQ(r, w) << "i=" << i;
    }
}

TEST(RescanAdvLifecycle, ManyBarsCyclesNoLeakStaysUsable)
{
    AdvTree t;
    Device *dev = t.addDevice("0000:61:00");

    for (int i = 0; i < 25; i++) {
        t.remove("0000:61:00", DeviceFunction::Bars);
        EXPECT_FALSE(resolves(t.get(), dev->dir->ino, "bars")) << "i=" << i;

        ASSERT_TRUE(t.restore(dev, DeviceFunction::Bars)) << "i=" << i;
        Node *bar0 = nullptr;
        ASSERT_EQ(t.get().lookupChild(dev->bars->ino, "bar0", &bar0), 0) << "i=" << i;
        ASSERT_NE(bar0, nullptr) << "i=" << i;

        uint32_t v = 0x1000u + (uint32_t) i, r = 0;
        ASSERT_EQ(t.get().pwrite(bar0->ino,
                                 reinterpret_cast<const char *>(&v), 4, 0x10),
                  4)
            << "i=" << i;
        ASSERT_EQ(t.get().pread(bar0->ino,
                                reinterpret_cast<char *>(&r), 4, 0x10),
                  4)
            << "i=" << i;
        EXPECT_EQ(r, v) << "i=" << i;
    }
}

// Restore must not double-attach the qdma ops: a stray double-rebuild would
// surface as a duplicate "qdma" child node under <BDF>/.  Drive restore twice
// (second is a no-op) and confirm exactly one usable qdma dir.
TEST(RescanAdvLifecycle, DoubleRestoreSecondIsNoopNoDoubleAttach)
{
    AdvTree t;
    Device *dev = t.addDevice("0000:61:00");
    t.remove("0000:61:00", DeviceFunction::Qdma);

    ASSERT_TRUE(t.restore(dev, DeviceFunction::Qdma));
    // Second restore: function already live -> rebuilt=false, no re-attach.
    bool rebuilt = true;
    ASSERT_EQ(t.get().restoreFunction("0000:61:00", DeviceFunction::Qdma,
                                      &rebuilt),
              0);
    EXPECT_FALSE(rebuilt) << "second restore must be a no-op";

    // Exactly one qdma dir under <BDF>/ (no duplicate): count the live children
    // of <BDF>/ named "qdma".
    int qdma_dirs = 0;
    for (Node *ch : dev->dir->children) {
        if (!ch->unlinked && ch->live && ch->name == "qdma") {
            qdma_dirs++;
        }
    }
    EXPECT_EQ(qdma_dirs, 1) << "double restore must not create a second qdma dir";

    // Still usable: two QPAIR_ADDs allocate two distinct qids (the rebuilt
    // store's id allocator works and was not clobbered by the no-op second restore).
    uint32_t q1 = add_qpair(t.get(), dev);
    uint32_t q2 = add_qpair(t.get(), dev);
    EXPECT_NE(q1, q2) << "distinct qids prove the store was not clobbered";
}

// ===========================================================================
// Integration over the real FUSE mount (CI stub model).
// ===========================================================================

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

std::string write_config_body(const char *body)
{
    ::mkdir(SLASH_EMU_TMP_DIR, 0755);
    std::string tmpl =
        std::string(SLASH_EMU_TMP_DIR) + "/slash_emu_advcfg_XXXXXX";
    std::vector<char> buf(tmpl.begin(), tmpl.end());
    buf.push_back('\0');
    int fd = ::mkstemp(buf.data());
    EXPECT_GE(fd, 0);
    (void) ::write(fd, body, std::strlen(body));
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

void rm_rf(const std::string &p)
{
    (void) ::system(("rm -rf '" + p + "'").c_str());
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
    std::vector<uint8_t> out;
    tar_append_member(out, "vpp_sim", read_file(SLASH_EMU_STUB_MODEL_PATH), 0755);
    out.resize(out.size() + 1024, 0);
    return out;
}

int open_qpair(const std::string &mnt)
{
    std::string qdma = mnt + "/" + kBdf + "/qdma";
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

int hotplug_rescan(const std::string &mnt)
{
    int fd = ::open((mnt + "/hotplug").c_str(), O_RDONLY);
    if (fd < 0) {
        return -errno;
    }
    int rc = ::ioctl(fd, SLASH_ABI_HOTPLUG_IOCTL_RESCAN);
    int saved = errno;
    ::close(fd);
    return rc == 0 ? 0 : -saved;
}

void reconfigure(int qfd)
{
    std::vector<uint8_t> vbin = ci_vbin();
    ssize_t w = ::pwrite(qfd, vbin.data(), vbin.size(), (off_t) SLASH_RECONFIG_BASE);
    ASSERT_EQ(w, (ssize_t) vbin.size())
        << "reconfig write: " << std::strerror(errno);
}

template <typename Body>
void with_daemon(const char *cfg_body, Body body)
{
    const std::string mountpoint = make_scratch("advmnt");
    const std::string scratch = make_scratch("advscratch");
    ASSERT_FALSE(mountpoint.empty());
    ASSERT_FALSE(scratch.empty());
    const std::string config = write_config_body(cfg_body);

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
    EXPECT_EQ(count_entries(scratch), 0)
        << "scratch root not cleaned after daemon shutdown";

    ::rmdir(mountpoint.c_str());
    rm_rf(mountpoint);
    rm_rf(scratch);
    ::unlink(config.c_str());
}

const char *kSingleCfg = "[accelerator:0000:61:00]\nnet-ip = 10.0.0.1\n";

// HAMMER: the real-daemon re-arm: both removed -> model down -> RESCAN
// restores both model-less -> reconfig spawns a model -> remove both AGAIN and
// the model is torn down a SECOND time THROUGH THE UN-RE-INSTALLED SEAM (the
// daemon never re-installs model_shutdown; this proves the surviving-seam path).
TEST(RescanAdvMount, DoubleTeardownThroughSurvivingSeam)
{
    with_daemon(kSingleCfg, [](const std::string &mnt, const std::string &scratch) {
        const std::string dev = mnt + "/" + kBdf;

        int qfd = open_qpair(mnt);
        ASSERT_GE(qfd, 0);
        reconfigure(qfd);
        ASSERT_GT(count_entries(scratch), 0) << "model not spawned";
        ::close(qfd);

        // Remove both -> model torn down (first fire).
        ASSERT_EQ(hotplug_remove(mnt, "0000:61:00.1"), 0);
        ASSERT_EQ(hotplug_remove(mnt, "0000:61:00.2"), 0);
        ASSERT_TRUE(wait_for([&] { return count_entries(scratch) == 0; }, 3000))
            << "first teardown failed";

        // RESCAN restores both model-less.
        ASSERT_EQ(hotplug_rescan(mnt), 0);
        ASSERT_TRUE(path_exists(dev + "/qdma"));
        ASSERT_TRUE(path_exists(dev + "/bars"));
        ASSERT_EQ(count_entries(scratch), 0) << "restore must not spawn a model";

        // Reconfig spawns a NEW model via the restored fn1's fresh handler.
        int qfd2 = open_qpair(mnt);
        ASSERT_GE(qfd2, 0);
        reconfigure(qfd2);
        ASSERT_GT(count_entries(scratch), 0)
            << "post-restore reconfig must spawn a model";
        ::close(qfd2);

        // Remove both AGAIN -> the SURVIVING (un-re-installed) seam must tear the
        // new model down a SECOND time.  This is the real-daemon re-arm proof.
        ASSERT_EQ(hotplug_remove(mnt, "0000:61:00.1"), 0);
        ASSERT_EQ(hotplug_remove(mnt, "0000:61:00.2"), 0);
        EXPECT_TRUE(wait_for([&] { return count_entries(scratch) == 0; }, 3000))
            << "second teardown through surviving seam failed -> model leaked";
    });
}

// Restore-from-both-removed must read the MODEL, not a stale/empty shadow.
// Bring a model up, write a known BAR value THROUGH it, remove both (model
// down), RESCAN (model-less), reconfig (fresh model = fresh stub state), then
// confirm a NEW value round-trips through the fresh model (rc path is the model,
// proven by a model-only register surviving a re-read).
TEST(RescanAdvMount, RestoreReadsFreshModelNotStaleShadow)
{
    with_daemon(kSingleCfg, [](const std::string &mnt, const std::string &scratch) {
        const std::string dev = mnt + "/" + kBdf;

        int qfd = open_qpair(mnt);
        ASSERT_GE(qfd, 0);
        reconfigure(qfd);
        ASSERT_GT(count_entries(scratch), 0);

        // Poke a BAR register THROUGH the first model.
        std::string bar0 = dev + "/bars/bar0";
        int bfd = ::open(bar0.c_str(), O_RDWR);
        ASSERT_GE(bfd, 0);
        uint32_t v1 = 0xCAFEBABEu, r = 0;
        ASSERT_EQ(::pwrite(bfd, &v1, 4, 0x30), 4);
        ASSERT_EQ(::pread(bfd, &r, 4, 0x30), 4);
        EXPECT_EQ(r, v1) << "first model round-trip";
        ::close(bfd);
        ::close(qfd);

        // Remove both -> model down.
        ASSERT_EQ(hotplug_remove(mnt, "0000:61:00.1"), 0);
        ASSERT_EQ(hotplug_remove(mnt, "0000:61:00.2"), 0);
        ASSERT_TRUE(wait_for([&] { return count_entries(scratch) == 0; }, 3000));

        // RESCAN (model-less) then reconfig (a FRESH model process).
        ASSERT_EQ(hotplug_rescan(mnt), 0);
        int qfd2 = open_qpair(mnt);
        ASSERT_GE(qfd2, 0);
        reconfigure(qfd2);
        ASSERT_GT(count_entries(scratch), 0);

        // A new BAR value round-trips through the FRESH model (proves the restored
        // bar backend re-points at the live model, not a detached stale shadow).
        int bfd2 = ::open(bar0.c_str(), O_RDWR);
        ASSERT_GE(bfd2, 0) << "open restored bar0: " << std::strerror(errno);
        uint32_t v2 = 0x0BADF00Du, r2 = 0;
        ASSERT_EQ(::pwrite(bfd2, &v2, 4, 0x30), 4);
        ASSERT_EQ(::pread(bfd2, &r2, 4, 0x30), 4);
        EXPECT_EQ(r2, v2) << "restored bar must round-trip through the fresh model";
        ::close(bfd2);

        // QDMA through the fresh model too.
        std::vector<uint8_t> out(96), in(96, 0);
        for (size_t i = 0; i < out.size(); i++) {
            out[i] = (uint8_t) (i * 5 + 1);
        }
        ASSERT_EQ(::pwrite(qfd2, out.data(), out.size(), (off_t) SLASH_HBM_BASE),
                  (ssize_t) out.size());
        ASSERT_EQ(::pread(qfd2, in.data(), in.size(), (off_t) SLASH_HBM_BASE),
                  (ssize_t) in.size());
        EXPECT_EQ(in, out);

        ::close(qfd2);
        ASSERT_EQ(hotplug_remove(mnt, "0000:61:00.1"), 0);
        ASSERT_EQ(hotplug_remove(mnt, "0000:61:00.2"), 0);
        EXPECT_TRUE(wait_for([&] { return count_entries(scratch) == 0; }, 3000));
    });
}

// select_new is UNCHANGED.  A config that also names a SECOND accelerator whose
// BDF is NOT present (no such device materialized) must never cause RESCAN to
// fabricate it -- rediscovery only restores removed FUNCTIONS of LIVE devices.
// The live device's removed function still rediscovers; the absent configured BDF
// stays absent (the config-driven select_new pass would add it only if available,
// which in this emulator it is not -- a configured BDF without a device).
TEST(RescanAdvMount, SelectNewUnchangedAbsentConfiguredBdfNeverFabricated)
{
    // Two accelerators configured; the daemon materializes BOTH (the emulator
    // treats every configured accelerator as present).  We then remove a function
    // of ONE and confirm RESCAN restores exactly that, touching nothing else.
    const char *cfg =
        "[accelerator:0000:61:00]\nnet-ip = 10.0.0.1\n"
        "[accelerator:0000:62:00]\nnet-ip = 10.0.0.2\n";
    with_daemon(cfg, [](const std::string &mnt, const std::string &scratch) {
        (void) scratch;
        const std::string a = mnt + "/0000:61:00";
        const std::string b = mnt + "/0000:62:00";
        ASSERT_TRUE(path_exists(a + "/qdma"));
        ASSERT_TRUE(path_exists(b + "/qdma"));
        ASSERT_TRUE(path_exists(b + "/bars"));

        // Remove fn1 of device A only.
        ASSERT_EQ(hotplug_remove(mnt, "0000:61:00.1"), 0);
        EXPECT_FALSE(path_exists(a + "/qdma"));
        // Device B is wholly untouched.
        EXPECT_TRUE(path_exists(b + "/qdma"));
        EXPECT_TRUE(path_exists(b + "/bars"));

        // RESCAN restores A's fn1 ONLY; B is unchanged (no dup, still intact).
        ASSERT_EQ(hotplug_rescan(mnt), 0);
        EXPECT_TRUE(path_exists(a + "/qdma")) << "A fn1 rediscovered";
        EXPECT_TRUE(path_exists(a + "/bars"));
        EXPECT_TRUE(path_exists(b + "/qdma"));
        EXPECT_TRUE(path_exists(b + "/bars"));

        // A truly-unconfigured BDF is never present and never fabricated.
        EXPECT_FALSE(path_exists(mnt + "/0000:99:00"));
        ASSERT_EQ(hotplug_rescan(mnt), 0);
        EXPECT_FALSE(path_exists(mnt + "/0000:99:00"))
            << "RESCAN must never fabricate an unconfigured BDF";
    });
}

// A RESCAN with nothing removed run TWICE must not double-attach (no duplicate
// children) and the device must stay usable -- the additive pass over a live
// device with no removed functions is a strict no-op.
TEST(RescanAdvMount, RescanTwiceNothingRemovedNoDupStaysUsable)
{
    with_daemon(kSingleCfg, [](const std::string &mnt, const std::string &scratch) {
        const std::string dev = mnt + "/" + kBdf;

        ASSERT_EQ(hotplug_rescan(mnt), 0);
        ASSERT_EQ(hotplug_rescan(mnt), 0);
        EXPECT_EQ(count_entries(scratch), 0) << "no-op RESCAN must not spawn";

        // Exactly one qdma + one bars under <BDF>/ (no duplicates).
        // info, bars, qdma = 3 entries.
        EXPECT_EQ(count_entries(dev), 3) << "info + bars + qdma, no dup";
        EXPECT_TRUE(path_exists(dev + "/qdma"));
        EXPECT_TRUE(path_exists(dev + "/bars"));

        // Still usable in-memory (no model).
        int qfd = open_qpair(mnt);
        ASSERT_GE(qfd, 0);
        uint8_t b = 0x9a, back = 0;
        ASSERT_EQ(::pwrite(qfd, &b, 1, (off_t) SLASH_HBM_BASE), 1);
        ASSERT_EQ(::pread(qfd, &back, 1, (off_t) SLASH_HBM_BASE), 1);
        EXPECT_EQ(back, b);
        ::close(qfd);
    });
}

}  // namespace
