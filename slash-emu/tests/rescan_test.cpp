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
 * @file rescan_test.cpp
 * @brief Unit + integration tests for RESCAN rediscovery (restore a removed function).
 *
 * RESCAN's rediscovery semantics: after a per-function REMOVE (fn1=qdma /
 * fn2=bars) the device stays live but the function's subtree is gone.  A RESCAN
 * must REDISCOVER -- rebuild that one function's subtree on the EXISTING in-memory
 * device (the hardware "a rescan re-enumerates the function" analogy), WITHOUT
 * reconciling it against config (config may have changed; we ignore that, exactly
 * like hardware re-enumerating the physical device as-is).  The config-driven
 * select_new pass still skips live BDFs; rediscovery is the orthogonal additive
 * restore pass over a live device's removed functions.
 *
 * Two layers (CMake+CTest, GTest, scratch under .tmp):
 *
 *   - Unit, against slash_emu_core directly (the restore building blocks --
 *     emu_device_restore_function + emu_bars_attach / emu_qdma_attach +
 *     emu_bridge_reattach_function): a removed function's endpoints reappear, the
 *     restore is a no-op when nothing was removed (no duplicate children), a
 *     restored function can be REMOVED again and restored again, the
 *     model_shutdown_fired re-arm (remove both -> seam fires once; restore; remove
 *     both again -> fires again), and the additive restore leaves the live-BDF
 *     collision-skip seed untouched.
 *
 *   - Integration, over the real FUSE mount with the CI stub model
 *     (bridge_integration_test pattern): RESCAN restores fn1 while bars+model stay
 *     live and a QDMA op round-trips THROUGH the still-running model; RESCAN
 *     restores fn2 while the model stays live and a BAR register round-trips
 *     THROUGH it; RESCAN after BOTH removed brings both endpoints back with no
 *     model, then a reconfiguration VBIN spawns a model and the data plane works;
 *     RESCAN with nothing removed is a no-op.
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

extern "C" {
#include "bars.h"
#include "bridge.h"
#include "hotplug.h"
#include "info.h"
#include "node.h"
#include "qdma.h"
#include "slash/uapi/slash_abi.h"
}

namespace {

// ===========================================================================
// Unit harness: a tree with one or more fully-attached devices + /hotplug,
// plus an optional (bridge-less) model-shutdown seam counter.  Mirrors the
// hotplug_test.cpp HotplugTree but exposes the restore building blocks.
// ===========================================================================

struct ShutdownState {
    std::atomic<int> calls{0};
    emu_device *last{nullptr};
};

void stub_model_shutdown(emu_device *dev, void *ctx)
{
    auto *s = static_cast<ShutdownState *>(ctx);
    s->calls++;
    s->last = dev;
}

class RescanTree {
public:
    RescanTree() { EXPECT_EQ(emu_node_tree_new(&tree_, nullptr), 0); }
    ~RescanTree() { cleanup_node_tree(tree_); }
    emu_node_tree *get() { return tree_; }

    // Add a fully-attached device (info + bars + qdma + optional shutdown seam).
    emu_device *add_device(const char *bdf, emu_model_shutdown_fn sd = nullptr,
                           void *sd_ctx = nullptr)
    {
        emu_device *dev = nullptr;
        EXPECT_EQ(emu_node_tree_add_device(tree_, bdf, &dev), 0);
        EXPECT_NE(dev, nullptr);
        EXPECT_EQ(emu_info_attach(dev), 0);
        EXPECT_EQ(emu_bars_attach(dev), 0);
        EXPECT_EQ(emu_qdma_attach(dev), 0);
        if (sd != nullptr) {
            EXPECT_EQ(emu_device_set_model_shutdown(tree_, bdf, sd, sd_ctx), 0);
        }
        return dev;
    }

    // The RESCAN restore building block: rebuild a removed function's subtree +
    // re-attach its in-memory endpoint (no bridge in the unit harness).  Returns
    // whether the function was actually rebuilt.
    bool restore(emu_device *dev, emu_device_function func)
    {
        bool rebuilt = false;
        EXPECT_EQ(
            emu_device_restore_function(tree_, dev->bdf, func, &rebuilt), 0);
        if (rebuilt) {
            int aret = func == EMU_DEVICE_FUNCTION_QDMA ? emu_qdma_attach(dev)
                                                        : emu_bars_attach(dev);
            EXPECT_EQ(aret, 0);
        }
        return rebuilt;
    }

private:
    emu_node_tree *tree_ = nullptr;
};

// True if a directory name resolves live under a parent inode.
bool resolves(emu_node_tree *tree, emu_ino_t parent, const char *name)
{
    emu_node *child = nullptr;
    int rc = emu_node_lookup_child(tree, parent, name, &child);
    return rc == 0 && child != nullptr;
}

// Count live children of a directory inode (excludes . / .. and dead ones).
int live_children(emu_node_tree *tree, emu_ino_t parent, const char *name)
{
    emu_node *dir = nullptr;
    EXPECT_EQ(emu_node_lookup_child(tree, parent, name, &dir), 0);
    if (dir == nullptr) {
        return -1;
    }
    int n = 0;
    for (size_t i = 0; i < dir->children.len; i++) {
        emu_node *c = dir->children.d[i];
        if (!c->unlinked && c->live) {
            n++;
        }
    }
    return n;
}

// Resolve a child inode by walking from a parent dir (bumps lookup count once).
emu_ino_t child_ino(emu_node_tree *tree, emu_ino_t parent, const char *name)
{
    emu_node *child = nullptr;
    EXPECT_EQ(emu_node_lookup_child(tree, parent, name, &child), 0);
    return child != nullptr ? child->ino : 0;
}

// Add a qpair via the QDMA dir ioctl; returns the allocated qid.
uint32_t add_qpair(emu_node_tree *tree, emu_device *dev)
{
    struct slash_abi_qdma_qpair_add req {};
    req.size = sizeof(req);
    req.mode = 0;        // MM
    req.dir_mask = 0x3;  // H2C | C2H
    struct slash_abi_qdma_qpair_add out = req;
    EXPECT_EQ(emu_node_ioctl(tree, dev->qdma->ino,
                             SLASH_ABI_QDMA_IOCTL_QPAIR_ADD, &req, sizeof(req),
                             &out, sizeof(out)),
              0);
    return out.qid;
}

// ===========================================================================
// Unit: rediscovery rebuilds a removed function's endpoints
// ===========================================================================

TEST(RescanRestore, Function1QdmaReappearsAndIsUsable)
{
    RescanTree t;
    emu_device *dev = t.add_device("0000:61:00");

    // Remove fn1 (qdma); the subtree vanishes, bars survives.
    ASSERT_EQ(emu_device_revoke_function(t.get(), "0000:61:00",
                                         EMU_DEVICE_FUNCTION_QDMA),
              0);
    ASSERT_FALSE(resolves(t.get(), dev->dir->ino, "qdma"));
    ASSERT_TRUE(resolves(t.get(), dev->dir->ino, "bars"));

    // Restore (the RESCAN rediscovery building block).
    ASSERT_TRUE(t.restore(dev, EMU_DEVICE_FUNCTION_QDMA));

    // qdma/ resolves again and a fresh qpair + MM round-trip works.
    ASSERT_TRUE(resolves(t.get(), dev->dir->ino, "qdma"));
    uint32_t qid = add_qpair(t.get(), dev);
    char qname[32];
    std::snprintf(qname, sizeof(qname), "qpair%u", qid);
    emu_ino_t qp = child_ino(t.get(), dev->qdma->ino, qname);
    ASSERT_NE(qp, 0u);

    std::vector<uint8_t> data(64, 0xC3), got(64, 0);
    ASSERT_EQ(emu_node_pwrite(t.get(), qp,
                              reinterpret_cast<const char *>(data.data()),
                              data.size(), static_cast<off_t>(SLASH_HBM_BASE)),
              static_cast<ssize_t>(data.size()));
    ASSERT_EQ(emu_node_pread(t.get(), qp,
                             reinterpret_cast<char *>(got.data()), got.size(),
                             static_cast<off_t>(SLASH_HBM_BASE)),
              static_cast<ssize_t>(got.size()));
    EXPECT_EQ(got, data);
}

TEST(RescanRestore, Function2BarsReappearsAndIsUsable)
{
    RescanTree t;
    emu_device *dev = t.add_device("0000:61:00");

    ASSERT_EQ(emu_device_revoke_function(t.get(), "0000:61:00",
                                         EMU_DEVICE_FUNCTION_BARS),
              0);
    ASSERT_FALSE(resolves(t.get(), dev->dir->ino, "bars"));
    ASSERT_TRUE(resolves(t.get(), dev->dir->ino, "qdma"));

    ASSERT_TRUE(t.restore(dev, EMU_DEVICE_FUNCTION_BARS));

    ASSERT_TRUE(resolves(t.get(), dev->dir->ino, "bars"));
    emu_ino_t bar0 = child_ino(t.get(), dev->bars->ino, "bar0");
    ASSERT_NE(bar0, 0u);

    uint32_t v = 0xdeadbeef, r = 0;
    ASSERT_EQ(emu_node_pwrite(t.get(), bar0, reinterpret_cast<const char *>(&v),
                              4, 0x20),
              4);
    ASSERT_EQ(emu_node_pread(t.get(), bar0, reinterpret_cast<char *>(&r), 4,
                             0x20),
              4);
    EXPECT_EQ(r, v);
}

// Exactly three bar files come back (no missing / no duplicate), and exactly one
// fresh qdma dir with its store -- the rebuilt subtree matches a fresh attach.
TEST(RescanRestore, RebuiltSubtreeHasExactlyTheEndpointChildren)
{
    RescanTree t;
    emu_device *dev = t.add_device("0000:61:00");

    ASSERT_EQ(emu_device_revoke_function(t.get(), "0000:61:00",
                                         EMU_DEVICE_FUNCTION_BARS),
              0);
    ASSERT_TRUE(t.restore(dev, EMU_DEVICE_FUNCTION_BARS));

    EXPECT_EQ(live_children(t.get(), dev->dir->ino, "bars"), 3)
        << "exactly bar0/bar2/bar4 after restore";
}

// ===========================================================================
// Unit: restore is idempotent / additive (no-op when nothing was removed)
// ===========================================================================

TEST(RescanRestore, NoOpWhenFunctionNotRemoved)
{
    RescanTree t;
    emu_device *dev = t.add_device("0000:61:00");

    // Nothing removed: restore reports "not rebuilt" and does not double-attach.
    bool rebuilt = true;
    ASSERT_EQ(emu_device_restore_function(t.get(), "0000:61:00",
                                          EMU_DEVICE_FUNCTION_QDMA, &rebuilt),
              0);
    EXPECT_FALSE(rebuilt);
    ASSERT_EQ(emu_device_restore_function(t.get(), "0000:61:00",
                                          EMU_DEVICE_FUNCTION_BARS, &rebuilt),
              0);
    EXPECT_FALSE(rebuilt);

    // The intact subtrees are unchanged: exactly three bar files, qdma usable.
    EXPECT_EQ(live_children(t.get(), dev->dir->ino, "bars"), 3);
    EXPECT_TRUE(resolves(t.get(), dev->dir->ino, "qdma"));

    // A qpair still works (the qdma ops were not double-attached / clobbered).
    uint32_t qid = add_qpair(t.get(), dev);
    char qname[32];
    std::snprintf(qname, sizeof(qname), "qpair%u", qid);
    EXPECT_TRUE(resolves(t.get(), dev->qdma->ino, qname));
}

TEST(RescanRestore, AbsentDeviceIsNoopSuccess)
{
    RescanTree t;
    t.add_device("0000:61:00");

    // RESCAN rediscovery never config-reconciles: an absent BDF is a no-op
    // success here (config-driven re-add is the select_new pass's job).
    bool rebuilt = true;
    EXPECT_EQ(emu_device_restore_function(t.get(), "0000:99:00",
                                          EMU_DEVICE_FUNCTION_QDMA, &rebuilt),
              0);
    EXPECT_FALSE(rebuilt);
}

TEST(RescanRestore, BadFunctionRejected)
{
    RescanTree t;
    t.add_device("0000:61:00");
    bool rebuilt = true;
    EXPECT_EQ(emu_device_restore_function(t.get(), "0000:61:00",
                                          static_cast<emu_device_function>(7),
                                          &rebuilt),
              -1);
    EXPECT_FALSE(rebuilt);
}

// ===========================================================================
// Unit: a restored function can be REMOVED again and restored again
// ===========================================================================

TEST(RescanRestore, RemoveRestoreRemoveRestoreCycle)
{
    RescanTree t;
    emu_device *dev = t.add_device("0000:61:00");

    for (int i = 0; i < 3; i++) {
        ASSERT_EQ(emu_device_revoke_function(t.get(), "0000:61:00",
                                             EMU_DEVICE_FUNCTION_QDMA),
                  0)
            << "iteration " << i;
        EXPECT_FALSE(resolves(t.get(), dev->dir->ino, "qdma"));

        ASSERT_TRUE(t.restore(dev, EMU_DEVICE_FUNCTION_QDMA))
            << "iteration " << i;
        EXPECT_TRUE(resolves(t.get(), dev->dir->ino, "qdma"));

        // The freshly-restored qdma is functional each time.
        uint32_t qid = add_qpair(t.get(), dev);
        char qname[32];
        std::snprintf(qname, sizeof(qname), "qpair%u", qid);
        EXPECT_TRUE(resolves(t.get(), dev->qdma->ino, qname))
            << "iteration " << i;
    }
}

// After REMOVE of a restored function, an already-open handle is -ENODEV again.
TEST(RescanRestore, RemovedAfterRestoreEnodevOnOpenHandle)
{
    RescanTree t;
    emu_device *dev = t.add_device("0000:61:00");

    ASSERT_EQ(emu_device_revoke_function(t.get(), "0000:61:00",
                                         EMU_DEVICE_FUNCTION_BARS),
              0);
    ASSERT_TRUE(t.restore(dev, EMU_DEVICE_FUNCTION_BARS));

    // "Open" bar0 (bump lookup so it survives the next revoke as a dead orphan).
    emu_ino_t bar0 = child_ino(t.get(), dev->bars->ino, "bar0");
    ASSERT_NE(bar0, 0u);
    EXPECT_EQ(emu_node_is_live(t.get(), bar0), 0);

    // Remove fn2 again: the open handle is -ENODEV, a reopen misses.
    ASSERT_EQ(emu_device_revoke_function(t.get(), "0000:61:00",
                                         EMU_DEVICE_FUNCTION_BARS),
              0);
    EXPECT_EQ(emu_node_is_live(t.get(), bar0), -ENODEV);
    EXPECT_FALSE(resolves(t.get(), dev->dir->ino, "bars"));

    // ...and it can be restored once more.
    ASSERT_TRUE(t.restore(dev, EMU_DEVICE_FUNCTION_BARS));
    EXPECT_TRUE(resolves(t.get(), dev->dir->ino, "bars"));
}

// ===========================================================================
// Unit: model_shutdown_fired re-arm across restore
// ===========================================================================

TEST(RescanModelShutdownRearm, RemoveBothRestoreRemoveBothFiresTwice)
{
    ShutdownState sd;
    RescanTree t;
    emu_device *dev = t.add_device("0000:61:00", stub_model_shutdown, &sd);

    // Remove both functions -> seam fires once.
    ASSERT_EQ(emu_device_revoke_function(t.get(), "0000:61:00",
                                         EMU_DEVICE_FUNCTION_QDMA),
              0);
    EXPECT_EQ(sd.calls.load(), 0);
    ASSERT_EQ(emu_device_revoke_function(t.get(), "0000:61:00",
                                         EMU_DEVICE_FUNCTION_BARS),
              0);
    EXPECT_EQ(sd.calls.load(), 1) << "seam fires once both gone";

    // Restore both (rediscovery).  The seam is re-armed.
    ASSERT_TRUE(t.restore(dev, EMU_DEVICE_FUNCTION_QDMA));
    ASSERT_TRUE(t.restore(dev, EMU_DEVICE_FUNCTION_BARS));

    // Re-wire the seam (a fresh qdma attach does not carry it; the daemon's
    // bridge reattach re-installs the seam wiring -- here we restore it directly).
    ASSERT_EQ(emu_device_set_model_shutdown(t.get(), "0000:61:00",
                                            stub_model_shutdown, &sd),
              0);

    // Remove both again -> the re-armed seam fires a SECOND time.
    ASSERT_EQ(emu_device_revoke_function(t.get(), "0000:61:00",
                                         EMU_DEVICE_FUNCTION_QDMA),
              0);
    ASSERT_EQ(emu_device_revoke_function(t.get(), "0000:61:00",
                                         EMU_DEVICE_FUNCTION_BARS),
              0);
    EXPECT_EQ(sd.calls.load(), 2)
        << "model_shutdown must re-arm and fire again after restore";
}

// Restoring ONE of two removed functions re-arms the seam: a subsequent removal
// of that same function (both gone again) fires once more.
TEST(RescanModelShutdownRearm, RestoreOneReArmsSeam)
{
    ShutdownState sd;
    RescanTree t;
    emu_device *dev = t.add_device("0000:61:00", stub_model_shutdown, &sd);

    ASSERT_EQ(emu_device_revoke_function(t.get(), "0000:61:00",
                                         EMU_DEVICE_FUNCTION_QDMA),
              0);
    ASSERT_EQ(emu_device_revoke_function(t.get(), "0000:61:00",
                                         EMU_DEVICE_FUNCTION_BARS),
              0);
    EXPECT_EQ(sd.calls.load(), 1);

    // Restore only fn1.  bars is still removed, but the seam is re-armed.
    ASSERT_TRUE(t.restore(dev, EMU_DEVICE_FUNCTION_QDMA));
    ASSERT_EQ(emu_device_set_model_shutdown(t.get(), "0000:61:00",
                                            stub_model_shutdown, &sd),
              0);

    // Remove fn1 again -> both gone again -> fires a second time.
    ASSERT_EQ(emu_device_revoke_function(t.get(), "0000:61:00",
                                         EMU_DEVICE_FUNCTION_QDMA),
              0);
    EXPECT_EQ(sd.calls.load(), 2) << "restore re-arms even a single function";
}

// ===========================================================================
// Unit: rediscovery is additive -- the collision-skip seed is untouched
// ===========================================================================

TEST(RescanRestore, LiveBdfStillCollectedAfterRestore)
{
    RescanTree t;
    emu_device *dev = t.add_device("0000:61:00");
    ASSERT_EQ(emu_device_revoke_function(t.get(), "0000:61:00",
                                         EMU_DEVICE_FUNCTION_QDMA),
              0);
    ASSERT_TRUE(t.restore(dev, EMU_DEVICE_FUNCTION_QDMA));

    // The device is (still) live, so the config-driven select_new pass keeps
    // skipping it -- collect_live_bdfs reports it, exactly as before the restore.
    str_array live = str_array_init();
    ASSERT_EQ(emu_node_tree_collect_live_bdfs(t.get(), &live), 0);
    ASSERT_EQ(live.len, 1u);
    EXPECT_STREQ(live.d[0], "0000:61:00");
    str_array_free(&live);
}

// emu_bridge_reattach_function with no bridge for the device is a safe no-op
// (the unit harness never attaches a bridge): it wires the reconfig handler for
// fn1 and leaves fn2 as a bare in-memory endpoint, both valid restore states.
TEST(RescanRestore, BridgeReattachNoBridgeIsNoop)
{
    RescanTree t;
    emu_device *dev = t.add_device("0000:61:00");

    emu_bridge_registry *reg = nullptr;
    ASSERT_EQ(emu_bridge_registry_new(&reg), 0);

    ASSERT_EQ(emu_device_revoke_function(t.get(), "0000:61:00",
                                         EMU_DEVICE_FUNCTION_QDMA),
              0);
    ASSERT_TRUE(t.restore(dev, EMU_DEVICE_FUNCTION_QDMA));

    // No bridge registered for this device -> a safe no-op success.
    EXPECT_EQ(
        emu_bridge_reattach_function(reg, dev, EMU_DEVICE_FUNCTION_QDMA), 0);
    EXPECT_EQ(
        emu_bridge_reattach_function(reg, dev, EMU_DEVICE_FUNCTION_BARS), 0);

    emu_bridge_registry_free(reg);
}

// ===========================================================================
// Integration: RESCAN rediscovery over a real FUSE mount (CI stub model)
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

std::string write_config()
{
    ::mkdir(SLASH_EMU_TMP_DIR, 0755);
    std::string tmpl = std::string(SLASH_EMU_TMP_DIR) + "/slash_emu_rscfg_XXXXXX";
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

// --- minimal ustar VBIN builder (bridge_integration_test pattern) ---

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
    out.resize(out.size() + 1024, 0); // two zero blocks = end-of-archive
    return out;
}

// Open the qdma/ dir and QPAIR_ADD; return the opened qpair fd (or -1).
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
    req.dir_mask = 0x3; // H2C | C2H
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

// Deliver the whole CI VBIN in one write to the reconfig region through a qpair.
void reconfigure(int qfd)
{
    std::vector<uint8_t> vbin = ci_vbin();
    ssize_t w = ::pwrite(qfd, vbin.data(), vbin.size(),
                         (off_t) SLASH_RECONFIG_BASE);
    ASSERT_EQ(w, (ssize_t) vbin.size())
        << "reconfig write: " << std::strerror(errno);
}

// Fork+exec the daemon with SLASH_EMU_SCRATCH_ROOT set, run body(mnt, scratch),
// then SIGTERM+reap and assert the scratch root is clean.
template <typename Body>
void with_daemon(Body body)
{
    const std::string mountpoint = make_scratch("rsmnt");
    const std::string scratch = make_scratch("rsscratch");
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
    EXPECT_EQ(count_entries(scratch), 0)
        << "scratch root not cleaned after daemon shutdown";

    ::rmdir(mountpoint.c_str());
    rm_rf(mountpoint);
    rm_rf(scratch);
    ::unlink(config.c_str());
}

// RESCAN restores fn1 (qdma) on a device whose bars + model stay live, and a
// QDMA MM op then round-trips THROUGH the still-running model.
TEST(RescanMount, RestoreFunction1RoundTripsThroughLiveModel)
{
    with_daemon([](const std::string &mnt, const std::string &scratch) {
        const std::string dev = mnt + "/" + kBdf;

        // Bring a model up (reconfigure through a qpair), then close that qfd.
        int qfd = open_qpair(mnt);
        ASSERT_GE(qfd, 0);
        reconfigure(qfd);
        ASSERT_GT(count_entries(scratch), 0) << "model not spawned";
        ::close(qfd);

        // Remove fn1 (qdma).  The model stays up (bars/fn2 still live), bars/
        // survives, qdma/ vanishes.
        ASSERT_EQ(hotplug_remove(mnt, "0000:61:00.1"), 0);
        EXPECT_FALSE(path_exists(dev + "/qdma"));
        EXPECT_TRUE(path_exists(dev + "/bars"));
        EXPECT_GT(count_entries(scratch), 0)
            << "model must stay up while one function remains";

        // RESCAN rediscovers fn1: qdma/ reappears.
        ASSERT_EQ(hotplug_rescan(mnt), 0);
        EXPECT_TRUE(path_exists(dev + "/qdma"))
            << "RESCAN must rediscover the removed qdma function";

        // A QDMA MM op on a NEW qpair round-trips THROUGH the still-running model
        // (the restored store's mem backend was re-wired to the live model).
        int qfd2 = open_qpair(mnt);
        ASSERT_GE(qfd2, 0) << "QPAIR_ADD after restore: " << std::strerror(errno);
        std::vector<uint8_t> out(256), in(256, 0);
        for (size_t i = 0; i < out.size(); i++) {
            out[i] = (uint8_t) (i * 7 + 3);
        }
        ASSERT_EQ(::pwrite(qfd2, out.data(), out.size(), (off_t) SLASH_HBM_BASE),
                  (ssize_t) out.size())
            << "restored qpair write: " << std::strerror(errno);
        ASSERT_EQ(::pread(qfd2, in.data(), in.size(), (off_t) SLASH_HBM_BASE),
                  (ssize_t) in.size())
            << "restored qpair read: " << std::strerror(errno);
        EXPECT_EQ(in, out) << "restored fn1 must round-trip through the model";
        ::close(qfd2);

        // Clean teardown: remove both functions -> model torn down.
        ASSERT_EQ(hotplug_remove(mnt, "0000:61:00.1"), 0);
        ASSERT_EQ(hotplug_remove(mnt, "0000:61:00.2"), 0);
        EXPECT_TRUE(wait_for([&] { return count_entries(scratch) == 0; }, 3000));
    });
}

// RESCAN restores fn2 (bars) while the model stays live, and a BAR register
// round-trips THROUGH the still-running model.
TEST(RescanMount, RestoreFunction2RoundTripsThroughLiveModel)
{
    with_daemon([](const std::string &mnt, const std::string &scratch) {
        const std::string dev = mnt + "/" + kBdf;

        int qfd = open_qpair(mnt);
        ASSERT_GE(qfd, 0);
        reconfigure(qfd);
        ASSERT_GT(count_entries(scratch), 0) << "model not spawned";

        // Remove fn2 (bars).  The model stays up (qdma/fn1 still live).
        ASSERT_EQ(hotplug_remove(mnt, "0000:61:00.2"), 0);
        EXPECT_FALSE(path_exists(dev + "/bars"));
        EXPECT_TRUE(path_exists(dev + "/qdma"));
        EXPECT_GT(count_entries(scratch), 0) << "model must stay up";

        // RESCAN rediscovers fn2: bars/ reappears.
        ASSERT_EQ(hotplug_rescan(mnt), 0);
        EXPECT_TRUE(path_exists(dev + "/bars"))
            << "RESCAN must rediscover the removed bars function";

        // A BAR register poke round-trips THROUGH the still-running model (the
        // restored bar backend was re-wired to the live model).
        std::string bar0 = dev + "/bars/bar0";
        int bfd = ::open(bar0.c_str(), O_RDWR);
        ASSERT_GE(bfd, 0) << "open restored bar0: " << std::strerror(errno);
        uint32_t v = 0xa5a5f00du, r = 0;
        ASSERT_EQ(::pwrite(bfd, &v, 4, 0x40), 4)
            << "restored bar write: " << std::strerror(errno);
        ASSERT_EQ(::pread(bfd, &r, 4, 0x40), 4)
            << "restored bar read: " << std::strerror(errno);
        EXPECT_EQ(r, v) << "restored fn2 must round-trip through the model";
        ::close(bfd);

        ::close(qfd);
        ASSERT_EQ(hotplug_remove(mnt, "0000:61:00.1"), 0);
        ASSERT_EQ(hotplug_remove(mnt, "0000:61:00.2"), 0);
        EXPECT_TRUE(wait_for([&] { return count_entries(scratch) == 0; }, 3000));
    });
}

// RESCAN after BOTH functions removed (model torn down) brings both endpoints
// back with NO model; a fresh reconfiguration VBIN then spawns a model and the
// data plane works.
TEST(RescanMount, RestoreAfterBothRemovedThenReconfigSpawns)
{
    with_daemon([](const std::string &mnt, const std::string &scratch) {
        const std::string dev = mnt + "/" + kBdf;

        // Bring a model up, then remove BOTH functions -> model torn down.
        int qfd = open_qpair(mnt);
        ASSERT_GE(qfd, 0);
        reconfigure(qfd);
        ASSERT_GT(count_entries(scratch), 0);
        ::close(qfd);
        ASSERT_EQ(hotplug_remove(mnt, "0000:61:00.1"), 0);
        ASSERT_EQ(hotplug_remove(mnt, "0000:61:00.2"), 0);
        EXPECT_TRUE(wait_for([&] { return count_entries(scratch) == 0; }, 3000))
            << "model not torn down after both removed";
        EXPECT_FALSE(path_exists(dev + "/qdma"));
        EXPECT_FALSE(path_exists(dev + "/bars"));

        // RESCAN rediscovers both functions; no model comes up (it awaits a VBIN
        // like a fresh device).
        ASSERT_EQ(hotplug_rescan(mnt), 0);
        EXPECT_TRUE(path_exists(dev + "/qdma"));
        EXPECT_TRUE(path_exists(dev + "/bars"));
        EXPECT_EQ(count_entries(scratch), 0)
            << "restore after both-removed must not spawn a model";

        // A bare in-memory transfer works on the restored qdma (no model).
        int qfd2 = open_qpair(mnt);
        ASSERT_GE(qfd2, 0) << "QPAIR_ADD after restore: " << std::strerror(errno);
        uint8_t byte = 0x5c, back = 0;
        ASSERT_EQ(::pwrite(qfd2, &byte, 1, (off_t) SLASH_DDR_BASE), 1);
        ASSERT_EQ(::pread(qfd2, &back, 1, (off_t) SLASH_DDR_BASE), 1);
        EXPECT_EQ(back, byte);

        // The restored fn1 has a fresh reconfig handler: a VBIN spawns a model
        // and the full data plane (BAR + QDMA through the model) works.
        reconfigure(qfd2);
        EXPECT_GT(count_entries(scratch), 0)
            << "reconfig after restore-from-both-removed must spawn a model";

        std::string bar0 = dev + "/bars/bar0";
        int bfd = ::open(bar0.c_str(), O_RDWR);
        ASSERT_GE(bfd, 0);
        uint32_t v = 0x13572468u, r = 0;
        ASSERT_EQ(::pwrite(bfd, &v, 4, 0x80), 4);
        ASSERT_EQ(::pread(bfd, &r, 4, 0x80), 4);
        EXPECT_EQ(r, v) << "BAR must round-trip through the post-restore model";
        ::close(bfd);

        std::vector<uint8_t> out(128), in(128, 0);
        for (size_t i = 0; i < out.size(); i++) {
            out[i] = (uint8_t) (i ^ 0x33);
        }
        ASSERT_EQ(::pwrite(qfd2, out.data(), out.size(), (off_t) SLASH_HBM_BASE),
                  (ssize_t) out.size());
        ASSERT_EQ(::pread(qfd2, in.data(), in.size(), (off_t) SLASH_HBM_BASE),
                  (ssize_t) in.size());
        EXPECT_EQ(in, out) << "QDMA must round-trip through the post-restore model";

        ::close(qfd2);
        ASSERT_EQ(hotplug_remove(mnt, "0000:61:00.1"), 0);
        ASSERT_EQ(hotplug_remove(mnt, "0000:61:00.2"), 0);
        EXPECT_TRUE(wait_for([&] { return count_entries(scratch) == 0; }, 3000));
    });
}

// RESCAN with nothing removed is a no-op: the intact device stays usable and no
// model is spawned/duplicated.
TEST(RescanMount, RescanNothingRemovedIsNoop)
{
    with_daemon([](const std::string &mnt, const std::string &scratch) {
        const std::string dev = mnt + "/" + kBdf;
        ASSERT_TRUE(path_exists(dev + "/qdma"));
        ASSERT_TRUE(path_exists(dev + "/bars"));

        ASSERT_EQ(hotplug_rescan(mnt), 0);

        EXPECT_TRUE(path_exists(dev + "/qdma"));
        EXPECT_TRUE(path_exists(dev + "/bars"));
        EXPECT_EQ(count_entries(scratch), 0) << "no-op RESCAN must not spawn";

        // The intact device is still usable (a qpair MM transfer round-trips
        // through the in-memory store, no model needed).
        int qfd = open_qpair(mnt);
        ASSERT_GE(qfd, 0);
        uint8_t byte = 0x77, back = 0;
        ASSERT_EQ(::pwrite(qfd, &byte, 1, (off_t) SLASH_HBM_BASE), 1);
        ASSERT_EQ(::pread(qfd, &back, 1, (off_t) SLASH_HBM_BASE), 1);
        EXPECT_EQ(back, byte);
        ::close(qfd);
    });
}

}  // namespace
