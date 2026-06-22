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
 * @file hotplug_test.cpp
 * @brief Unit + integration tests for the global /hotplug endpoint (T9).
 *
 * Two layers, matching the build rules (CMake+CTest, GTest, scratch under .tmp):
 *
 *   - Unit, against slash_emu_core directly (no FUSE mount): BDF-with-function
 *     parsing (valid / invalid / unknown function), the per-function REMOVE
 *     (only the targeted subtree disappears; the other stays live), the
 *     revocation matrix for a removed function (-ENOENT on reopen, -ENODEV on an
 *     already-open handle, close OK), the both-functions-removed model-shutdown
 *     seam (fires exactly once, via two REMOVEs or a whole-device revoke), the
 *     RESCAN/SBR/HOTPLUG reload sequencing through a stub reload callback, the
 *     ioctl buffer/versioning matrix, collision-skip + no-double-attach on the
 *     materialize re-invocation primitives, and the injectable SBR sleep.
 *
 *   - Integration, over the real FUSE mount (fork+exec the freshly-built daemon,
 *     the qdma_test.cpp pattern): the /hotplug file exists at root, REMOVE of a
 *     function makes that subtree vanish while the sibling survives, an already-
 *     open fd of a removed endpoint returns -ENODEV, and RESCAN/HOTPLUG re-init
 *     a removed device.
 */

#include <gtest/gtest.h>

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
#include "config.hpp"
#include "hotplug.hpp"
#include "info.hpp"
#include "node.hpp"
#include "qdma.hpp"

extern "C" {
#include "slash/uapi/slash_abi.h"
}

using slash::emu::Config;
using slash::emu::Device;
using slash::emu::DeviceFunction;
using slash::emu::Ino;
using slash::emu::Node;
using slash::emu::NodeTree;
using slash::emu::NodeType;
using slash::emu::ModelShutdownFn;
using slash::emu::ReloadFn;
using slash::emu::barsAttach;
using slash::emu::infoAttach;
using slash::emu::qdmaAttach;
using slash::emu::kRootIno;
using slash::emu::hotplugAttach;
using slash::emu::hotplugParseBdf;
using slash::emu::hotplugSetSbrSleepUs;

namespace {

// ===========================================================================
// Unit: BDF-with-function parsing
// ===========================================================================

TEST(HotplugParseBdf, ValidFunctionsMap)
{
    std::string bdf;
    DeviceFunction func;

    ASSERT_EQ(hotplugParseBdf("0000:61:00.1", bdf, func), 0);
    EXPECT_EQ(bdf, "0000:61:00");
    EXPECT_EQ(func, DeviceFunction::Qdma);

    ASSERT_EQ(hotplugParseBdf("0000:61:00.2", bdf, func), 0);
    EXPECT_EQ(bdf, "0000:61:00");
    EXPECT_EQ(func, DeviceFunction::Bars);
}

TEST(HotplugParseBdf, ShortFormExpandsDomain)
{
    std::string bdf;
    DeviceFunction func;

    // "BB:DD.F" -> implicit domain 0000.
    ASSERT_EQ(hotplugParseBdf("61:00.1", bdf, func), 0);
    EXPECT_EQ(bdf, "0000:61:00");
    EXPECT_EQ(func, DeviceFunction::Qdma);
}

TEST(HotplugParseBdf, UppercaseLowercased)
{
    std::string bdf;
    DeviceFunction func;
    ASSERT_EQ(hotplugParseBdf("0000:AB:0F.2", bdf, func), 0);
    EXPECT_EQ(bdf, "0000:ab:0f");
}

TEST(HotplugParseBdf, MissingFunctionRejected)
{
    std::string bdf;
    DeviceFunction func;
    // No function suffix at all.
    EXPECT_EQ(hotplugParseBdf("0000:61:00", bdf, func), -EINVAL);
    // Trailing dot with no digit.
    EXPECT_EQ(hotplugParseBdf("0000:61:00.", bdf, func), -EINVAL);
}

TEST(HotplugParseBdf, MalformedFunctionRejected)
{
    std::string bdf;
    DeviceFunction func;
    // Multi-char / non-digit function fields.
    EXPECT_EQ(hotplugParseBdf("0000:61:00.12", bdf, func), -EINVAL);
    EXPECT_EQ(hotplugParseBdf("0000:61:00.x", bdf, func), -EINVAL);
}

TEST(HotplugParseBdf, UnknownFunctionUnsupported)
{
    std::string bdf;
    DeviceFunction func;
    // Syntactically valid but not a removable function (only 1 and 2 are).
    EXPECT_EQ(hotplugParseBdf("0000:61:00.0", bdf, func), -EOPNOTSUPP);
    EXPECT_EQ(hotplugParseBdf("0000:61:00.3", bdf, func), -EOPNOTSUPP);
}

TEST(HotplugParseBdf, MalformedBoardRejected)
{
    std::string bdf;
    DeviceFunction func;
    EXPECT_EQ(hotplugParseBdf("not-a-bdf.1", bdf, func), -EINVAL);
    EXPECT_EQ(hotplugParseBdf(".1", bdf, func), -EINVAL);
}

// ===========================================================================
// Unit harness: a tree with one or more fully-attached devices + /hotplug.
// ===========================================================================

// Count of reload-callback invocations, and an injectable return code.
struct ReloadState {
    std::atomic<int> calls{0};
    int rc{0};
};

// Model-shutdown seam counter, keyed per device via the dev pointer.
struct ShutdownState {
    std::atomic<int> calls{0};
    Device *last{nullptr};
};

// A tree owning helper; builds the root + the /hotplug file.
class HotplugTree {
public:
    explicit HotplugTree(ReloadFn reload = {})
        : tree_()
    {
        EXPECT_EQ(hotplugAttach(tree_, std::move(reload)), 0);
    }
    NodeTree &get() { return tree_; }

    // Add a fully-attached device (info + bars + qdma + model-shutdown seam).
    Device *addDevice(const std::string &bdf,
                      ModelShutdownFn sd = {})
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

    // Resolve the /hotplug file inode at the root.
    Ino hotplugIno()
    {
        Node *child = nullptr;
        EXPECT_EQ(tree_.lookupChild(kRootIno, "hotplug", &child), 0);
        return child != nullptr ? child->ino : 0;
    }

private:
    NodeTree tree_;
};

// Issue a device-request ioctl (REMOVE/SBR/HOTPLUG) on the hotplug file.
int hotplugDevIoctl(NodeTree &tree, Ino ino, unsigned int cmd,
                    const char *bdf_with_func)
{
    struct slash_abi_hotplug_device_request req {};
    req.size = sizeof(req);
    std::snprintf(req.bdf, sizeof(req.bdf), "%s", bdf_with_func);
    // No out payload for any hotplug command.
    return tree.ioctl(ino, cmd, &req, sizeof(req), &req, sizeof(req));
}

// True if a directory name resolves live under a parent inode.
bool resolves(NodeTree &tree, Ino parent, const char *name)
{
    Node *child = nullptr;
    int rc = tree.lookupChild(parent, name, &child);
    return rc == 0 && child != nullptr;
}

// ===========================================================================
// Unit: per-function REMOVE
// ===========================================================================

TEST(HotplugRemove, Function1RemovesOnlyQdma)
{
    HotplugTree t;
    Device *dev = t.addDevice("0000:61:00");
    Ino hp = t.hotplugIno();
    ASSERT_NE(hp, 0u);

    // Both subtrees resolve before removal.
    ASSERT_TRUE(resolves(t.get(), dev->dir->ino, "qdma"));
    ASSERT_TRUE(resolves(t.get(), dev->dir->ino, "bars"));

    ASSERT_EQ(hotplugDevIoctl(t.get(), hp, SLASH_ABI_HOTPLUG_IOCTL_REMOVE,
                              "0000:61:00.1"),
              0);

    // qdma/ is gone (new lookup -> not found); bars/ survives.
    EXPECT_FALSE(resolves(t.get(), dev->dir->ino, "qdma"))
        << "qdma/ should be removed by function-1 REMOVE";
    EXPECT_TRUE(resolves(t.get(), dev->dir->ino, "bars"))
        << "bars/ must stay live after a function-1 REMOVE";
}

TEST(HotplugRemove, Function2RemovesOnlyBars)
{
    HotplugTree t;
    Device *dev = t.addDevice("0000:61:00");
    Ino hp = t.hotplugIno();

    ASSERT_EQ(hotplugDevIoctl(t.get(), hp, SLASH_ABI_HOTPLUG_IOCTL_REMOVE,
                              "0000:61:00.2"),
              0);

    EXPECT_FALSE(resolves(t.get(), dev->dir->ino, "bars"))
        << "bars/ should be removed by function-2 REMOVE";
    EXPECT_TRUE(resolves(t.get(), dev->dir->ino, "qdma"))
        << "qdma/ must stay live after a function-2 REMOVE";
}

TEST(HotplugRemove, RemovedFunctionIsIdempotent)
{
    HotplugTree t;
    t.addDevice("0000:61:00");
    Ino hp = t.hotplugIno();

    EXPECT_EQ(hotplugDevIoctl(t.get(), hp, SLASH_ABI_HOTPLUG_IOCTL_REMOVE,
                              "0000:61:00.1"),
              0);
    // Second REMOVE of the same function: no-op success.
    EXPECT_EQ(hotplugDevIoctl(t.get(), hp, SLASH_ABI_HOTPLUG_IOCTL_REMOVE,
                              "0000:61:00.1"),
              0);
}

TEST(HotplugRemove, UnknownDeviceIsNoopSuccess)
{
    HotplugTree t;
    t.addDevice("0000:61:00");
    Ino hp = t.hotplugIno();

    // REMOVE of a device that does not exist: the postcondition (endpoint gone)
    // already holds, so this is a no-op success, not an error.
    EXPECT_EQ(hotplugDevIoctl(t.get(), hp, SLASH_ABI_HOTPLUG_IOCTL_REMOVE,
                              "0000:99:00.1"),
              0);
}

// ===========================================================================
// Unit: revocation matrix for a removed function
// ===========================================================================

TEST(HotplugRevocation, EnodevOnOpenHandleEnoentOnReopen)
{
    HotplugTree t;
    Device *dev = t.addDevice("0000:61:00");
    Ino hp = t.hotplugIno();

    // "Open" bar0 (bumps lookup_count so the node survives as a dead orphan).
    Node *bar0 = nullptr;
    ASSERT_EQ(t.get().lookupChild(dev->bars->ino, "bar0", &bar0), 0);
    ASSERT_NE(bar0, nullptr);
    Ino bar0_ino = bar0->ino;

    // The open handle works before removal.
    EXPECT_EQ(t.get().isLive(bar0_ino), 0);

    // Remove function 2 (the bars subtree).
    ASSERT_EQ(hotplugDevIoctl(t.get(), hp, SLASH_ABI_HOTPLUG_IOCTL_REMOVE,
                              "0000:61:00.2"),
              0);

    // Any op on the already-open handle now returns -ENODEV.
    EXPECT_EQ(t.get().isLive(bar0_ino), -ENODEV);
    char buf[8] = {};
    EXPECT_EQ(t.get().pread(bar0_ino, buf, sizeof(buf), 0), -ENODEV);

    // A fresh lookup of the removed endpoint misses (-ENOENT at the FS layer).
    Node *gone = nullptr;
    EXPECT_EQ(t.get().lookupChild(dev->dir->ino, "bars", &gone), -ENOENT);
}

TEST(HotplugRevocation, QpairHandleEnodevAfterFunction1Remove)
{
    HotplugTree t;
    Device *dev = t.addDevice("0000:61:00");
    Ino hp = t.hotplugIno();

    // Create a qpair and "open" it.
    struct slash_abi_qdma_qpair_add qreq {};
    qreq.size = sizeof(qreq);
    qreq.mode = 0;       // MM
    qreq.dir_mask = 0x1; // H2C
    struct slash_abi_qdma_qpair_add qout = qreq;
    ASSERT_EQ(t.get().ioctl(dev->qdma->ino, SLASH_ABI_QDMA_IOCTL_QPAIR_ADD,
                            &qreq, sizeof(qreq), &qout, sizeof(qout)),
              0);
    Node *qp = nullptr;
    char qname[32];
    std::snprintf(qname, sizeof(qname), "qpair%u", qout.qid);
    ASSERT_EQ(t.get().lookupChild(dev->qdma->ino, qname, &qp), 0);
    Ino qp_ino = qp->ino;

    // The qpair works before removal.
    std::vector<uint8_t> data(16, 0xab);
    ASSERT_EQ(t.get().pwrite(qp_ino,
                             reinterpret_cast<const char *>(data.data()),
                             data.size(), static_cast<off_t>(SLASH_HBM_BASE)),
              static_cast<ssize_t>(data.size()));

    // Remove function 1 (qdma): the qpair resource is eagerly revoked.
    ASSERT_EQ(hotplugDevIoctl(t.get(), hp, SLASH_ABI_HOTPLUG_IOCTL_REMOVE,
                              "0000:61:00.1"),
              0);

    // The open qpair handle now returns -ENODEV; bars survive.
    EXPECT_EQ(t.get().pread(qp_ino,
                            reinterpret_cast<char *>(data.data()), data.size(),
                            static_cast<off_t>(SLASH_HBM_BASE)),
              -ENODEV);
    EXPECT_TRUE(resolves(t.get(), dev->dir->ino, "bars"));
}

// ===========================================================================
// Unit: both-functions-removed model-shutdown seam
// ===========================================================================

TEST(HotplugModelShutdown, FiresOnceWhenBothFunctionsRemovedViaTwoRemoves)
{
    ShutdownState sd;
    HotplugTree t;
    Device *dev = t.addDevice("0000:61:00", [&sd](Device &d) {
        sd.calls++;
        sd.last = &d;
    });
    Ino hp = t.hotplugIno();

    // Remove function 1: model still running (only one function gone).
    ASSERT_EQ(hotplugDevIoctl(t.get(), hp, SLASH_ABI_HOTPLUG_IOCTL_REMOVE,
                              "0000:61:00.1"),
              0);
    EXPECT_EQ(sd.calls.load(), 0) << "model must keep running after one function";

    // Remove function 2: now both gone -> seam fires exactly once.
    ASSERT_EQ(hotplugDevIoctl(t.get(), hp, SLASH_ABI_HOTPLUG_IOCTL_REMOVE,
                              "0000:61:00.2"),
              0);
    EXPECT_EQ(sd.calls.load(), 1) << "model shutdown must fire once both gone";
    EXPECT_EQ(sd.last, dev);

    // A redundant REMOVE does not re-fire it.
    ASSERT_EQ(hotplugDevIoctl(t.get(), hp, SLASH_ABI_HOTPLUG_IOCTL_REMOVE,
                              "0000:61:00.1"),
              0);
    EXPECT_EQ(sd.calls.load(), 1) << "model shutdown must fire exactly once";
}

TEST(HotplugModelShutdown, OrderIndependent)
{
    ShutdownState sd;
    HotplugTree t;
    t.addDevice("0000:61:00", [&sd](Device &d) {
        sd.calls++;
        sd.last = &d;
    });
    Ino hp = t.hotplugIno();

    // Remove function 2 first, then function 1.
    ASSERT_EQ(hotplugDevIoctl(t.get(), hp, SLASH_ABI_HOTPLUG_IOCTL_REMOVE,
                              "0000:61:00.2"),
              0);
    EXPECT_EQ(sd.calls.load(), 0);
    ASSERT_EQ(hotplugDevIoctl(t.get(), hp, SLASH_ABI_HOTPLUG_IOCTL_REMOVE,
                              "0000:61:00.1"),
              0);
    EXPECT_EQ(sd.calls.load(), 1);
}

TEST(HotplugModelShutdown, WholeDeviceRevokeFiresSeamOnce)
{
    ShutdownState sd;
    HotplugTree t;
    t.addDevice("0000:61:00", [&sd](Device &d) {
        sd.calls++;
        sd.last = &d;
    });

    // A whole-device revoke (the TOGGLE_SBR/HOTPLUG path) removes both functions
    // at once: the seam fires exactly once even with no prior per-function REMOVE.
    ASSERT_EQ(t.get().revokeDevice("0000:61:00"), 0);
    EXPECT_EQ(sd.calls.load(), 1);
}

// ===========================================================================
// Unit: RESCAN / TOGGLE_SBR / HOTPLUG reload sequencing (stub reload)
// ===========================================================================

TEST(HotplugRescan, InvokesReloadOnce)
{
    ReloadState rs;
    HotplugTree t([&rs]() -> int {
        rs.calls++;
        return rs.rc;
    });
    Ino hp = t.hotplugIno();

    // RESCAN takes no argument (_IO).
    ASSERT_EQ(t.get().ioctl(hp, SLASH_ABI_HOTPLUG_IOCTL_RESCAN, nullptr, 0,
                            nullptr, 0),
              0);
    EXPECT_EQ(rs.calls.load(), 1);
}

TEST(HotplugRescan, ReloadFailureSurfacesEio)
{
    ReloadState rs;
    rs.rc = -1; // make the reload callback fail
    HotplugTree t([&rs]() -> int {
        rs.calls++;
        return rs.rc;
    });
    Ino hp = t.hotplugIno();

    EXPECT_EQ(t.get().ioctl(hp, SLASH_ABI_HOTPLUG_IOCTL_RESCAN, nullptr, 0,
                            nullptr, 0),
              -EIO);
}

TEST(HotplugSbr, RemovesDeviceAndReloads)
{
    ReloadState rs;
    HotplugTree t([&rs]() -> int {
        rs.calls++;
        return rs.rc;
    });
    Device *dev = t.addDevice("0000:61:00");
    Ino hp = t.hotplugIno();
    Ino devdir_ino = dev->dir->ino;

    // Short-circuit the emulated sleep so the single-threaded daemon is not
    // blocked for a real second.
    ASSERT_EQ(hotplugSetSbrSleepUs(t.get(), 0), 0);

    ASSERT_EQ(hotplugDevIoctl(t.get(), hp, SLASH_ABI_HOTPLUG_IOCTL_TOGGLE_SBR,
                              "0000:61:00.1"),
              0);

    // The whole device is removed (its <BDF>/ dir no longer resolves) and reload
    // ran once.
    EXPECT_FALSE(resolves(t.get(), kRootIno, "0000:61:00"));
    EXPECT_EQ(rs.calls.load(), 1);
    EXPECT_EQ(t.get().lookupChild(devdir_ino, "bars", nullptr), -ESTALE);
}

TEST(HotplugHotplugCmd, RemovesDeviceAndReloadsNoSleep)
{
    ReloadState rs;
    HotplugTree t([&rs]() -> int {
        rs.calls++;
        return rs.rc;
    });
    t.addDevice("0000:61:00");
    Ino hp = t.hotplugIno();

    ASSERT_EQ(hotplugDevIoctl(t.get(), hp, SLASH_ABI_HOTPLUG_IOCTL_HOTPLUG,
                              "0000:61:00.2"),
              0);
    EXPECT_FALSE(resolves(t.get(), kRootIno, "0000:61:00"));
    EXPECT_EQ(rs.calls.load(), 1);
}

// ===========================================================================
// Unit: ioctl buffer / versioning matrix
// ===========================================================================

TEST(HotplugIoctl, UnknownCommandIsEnotty)
{
    HotplugTree t;
    Ino hp = t.hotplugIno();
    char buf[64] = {};
    EXPECT_EQ(t.get().ioctl(hp, 0xdeadbeef, buf, sizeof(buf), buf, sizeof(buf)),
              -ENOTTY);
}

TEST(HotplugIoctl, ShortDeviceRequestRejected)
{
    HotplugTree t;
    t.addDevice("0000:61:00");
    Ino hp = t.hotplugIno();

    struct slash_abi_hotplug_device_request req {};
    req.size = sizeof(req);
    std::snprintf(req.bdf, sizeof(req.bdf), "0000:61:00.1");
    // in_size too small to hold the struct.
    EXPECT_EQ(t.get().ioctl(hp, SLASH_ABI_HOTPLUG_IOCTL_REMOVE, &req, 4, &req,
                            sizeof(req)),
              -EINVAL);
}

TEST(HotplugIoctl, NonTerminatedBdfRejected)
{
    HotplugTree t;
    t.addDevice("0000:61:00");
    Ino hp = t.hotplugIno();

    struct slash_abi_hotplug_device_request req {};
    req.size = sizeof(req);
    // Fill the whole bdf array with non-NUL bytes (no terminator).
    std::memset(req.bdf, 'a', sizeof(req.bdf));
    EXPECT_EQ(t.get().ioctl(hp, SLASH_ABI_HOTPLUG_IOCTL_REMOVE, &req,
                            sizeof(req), &req, sizeof(req)),
              -EINVAL);
}

TEST(HotplugIoctl, UnknownFunctionInRequestUnsupported)
{
    HotplugTree t;
    t.addDevice("0000:61:00");
    Ino hp = t.hotplugIno();
    // Function 0 is syntactically valid but not removable.
    EXPECT_EQ(hotplugDevIoctl(t.get(), hp, SLASH_ABI_HOTPLUG_IOCTL_REMOVE,
                              "0000:61:00.0"),
              -EOPNOTSUPP);
}

TEST(HotplugIoctl, SetSbrSleepWithoutAttachFails)
{
    // A bare tree with no hotplug file: the injection seam reports failure.
    NodeTree tree;
    EXPECT_EQ(hotplugSetSbrSleepUs(tree, 0), -1);
}

// ===========================================================================
// Unit: materialize re-invocation primitives (collision-skip / no double-attach)
// ===========================================================================

TEST(HotplugMaterializeSeed, CollectLiveBdfsSeedsRunningSet)
{
    HotplugTree t;
    t.addDevice("0000:61:00");
    t.addDevice("0000:62:00");

    std::vector<std::string> live = t.get().collectLiveBdfs();
    ASSERT_EQ(live.size(), 2u);

    // Both BDFs are present in the live set (exact values, any order).
    bool has61 = false, has62 = false;
    for (const auto &bdf : live) {
        if (bdf == "0000:61:00") { has61 = true; }
        if (bdf == "0000:62:00") { has62 = true; }
    }
    EXPECT_TRUE(has61) << "0000:61:00 must be in the live set";
    EXPECT_TRUE(has62) << "0000:62:00 must be in the live set";

    // Seed a Config::selectNew call: a config that re-lists both must select
    // NEITHER (collision-skip), so a re-invocation never re-attaches.
    std::vector<const slash::emu::Accelerator *> sel =
        Config{}.selectNew(live);
    EXPECT_TRUE(sel.empty()) << "all live BDFs must be skipped by selectNew";
}

TEST(HotplugMaterializeSeed, RemovedFunctionDeviceStillCollectedUntilBothGone)
{
    // A device with only one function removed is still LIVE (the spine does not
    // mark it dead), so RESCAN must still skip it -- collectLiveBdfs reports it.
    HotplugTree t;
    t.addDevice("0000:61:00");
    Ino hp = t.hotplugIno();
    ASSERT_EQ(hotplugDevIoctl(t.get(), hp, SLASH_ABI_HOTPLUG_IOCTL_REMOVE,
                              "0000:61:00.1"),
              0);

    std::vector<std::string> live = t.get().collectLiveBdfs();
    ASSERT_EQ(live.size(), 1u);
    EXPECT_EQ(live[0], "0000:61:00");
}

// ===========================================================================
// Integration: /hotplug over a real FUSE mount
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
        std::string(SLASH_EMU_TMP_DIR) + "/slash_emu_hpcfg_XXXXXX";
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

// Fork+exec the daemon with a config carrying one device, run body() against the
// mount, then SIGTERM + reap.
template <typename Body>
void with_mounted_daemon(Body body)
{
    const std::string mountpoint = make_scratch("hotplug");
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

    ASSERT_EQ(::kill(pid, SIGTERM), 0) << "kill failed: " << std::strerror(errno);
    int status = 0;
    bool exited = wait_for(
        [&] { return ::waitpid(pid, &status, WNOHANG) == pid; },
        kShutdownTimeoutMs);
    if (!exited) {
        ::kill(pid, SIGKILL);
        ::waitpid(pid, &status, 0);
    }
    ::rmdir(mountpoint.c_str());
    ::unlink(config.c_str());
}

// Issue a hotplug device-request ioctl over the real mount.
int mount_hotplug_ioctl(const std::string &mountpoint, unsigned int cmd,
                        const char *bdf_with_func)
{
    int fd = ::open((mountpoint + "/hotplug").c_str(), O_RDONLY);
    if (fd < 0) {
        return -errno;
    }
    struct slash_abi_hotplug_device_request req {};
    req.size = sizeof(req);
    std::snprintf(req.bdf, sizeof(req.bdf), "%s", bdf_with_func);
    int rc = ::ioctl(fd, cmd, &req);
    int saved = errno;
    ::close(fd);
    return rc == 0 ? 0 : -saved;
}

bool path_exists(const std::string &p)
{
    struct stat st {};
    return ::stat(p.c_str(), &st) == 0;
}

TEST(HotplugMount, HotplugFileExistsAtRoot)
{
    with_mounted_daemon([](const std::string &mnt) {
        struct stat st {};
        ASSERT_EQ(::stat((mnt + "/hotplug").c_str(), &st), 0)
            << "hotplug file missing at root: " << std::strerror(errno);
        EXPECT_TRUE(S_ISREG(st.st_mode));
    });
}

TEST(HotplugMount, RemoveFunctionMakesSubtreeVanishSiblingSurvives)
{
    with_mounted_daemon([](const std::string &mnt) {
        const std::string dev = mnt + "/0000:61:00";
        ASSERT_TRUE(path_exists(dev + "/qdma"));
        ASSERT_TRUE(path_exists(dev + "/bars"));

        // REMOVE function 1 (qdma).
        ASSERT_EQ(mount_hotplug_ioctl(mnt, SLASH_ABI_HOTPLUG_IOCTL_REMOVE,
                                      "0000:61:00.1"),
                  0);

        EXPECT_FALSE(path_exists(dev + "/qdma"))
            << "qdma/ should vanish after function-1 REMOVE";
        EXPECT_TRUE(path_exists(dev + "/bars"))
            << "bars/ must survive a function-1 REMOVE";
    });
}

TEST(HotplugMount, OpenHandleOfRemovedEndpointReturnsEnodev)
{
    with_mounted_daemon([](const std::string &mnt) {
        const std::string bar0 = mnt + "/0000:61:00/bars/bar0";

        // Open a bar fd, then REMOVE function 2 (bars).
        int fd = ::open(bar0.c_str(), O_RDWR);
        ASSERT_GE(fd, 0) << "open bar0: " << std::strerror(errno);

        ASSERT_EQ(mount_hotplug_ioctl(mnt, SLASH_ABI_HOTPLUG_IOCTL_REMOVE,
                                      "0000:61:00.2"),
                  0);

        // An op on the already-open fd returns -ENODEV.
        char buf[4] = {};
        ssize_t n = ::pread(fd, buf, sizeof(buf), 0);
        int saved = errno;
        EXPECT_EQ(n, -1);
        EXPECT_EQ(saved, ENODEV) << "op on removed endpoint must be ENODEV";

        // close always succeeds.
        EXPECT_EQ(::close(fd), 0);

        // A fresh open of the removed endpoint misses with ENOENT.
        int fd2 = ::open(bar0.c_str(), O_RDWR);
        EXPECT_EQ(fd2, -1);
        EXPECT_EQ(errno, ENOENT) << "reopen of removed endpoint must be ENOENT";
        if (fd2 >= 0) {
            ::close(fd2);
        }
    });
}

TEST(HotplugMount, HotplugCmdReinitsRemovedDevice)
{
    with_mounted_daemon([](const std::string &mnt) {
        const std::string dev = mnt + "/0000:61:00";
        ASSERT_TRUE(path_exists(dev));

        // HOTPLUG = remove + reload + re-init: the device is re-materialized from
        // the (unchanged) config, so its subtree reappears.
        ASSERT_EQ(mount_hotplug_ioctl(mnt, SLASH_ABI_HOTPLUG_IOCTL_HOTPLUG,
                                      "0000:61:00.1"),
                  0);

        EXPECT_TRUE(path_exists(dev + "/qdma"))
            << "HOTPLUG must re-init the available device";
        EXPECT_TRUE(path_exists(dev + "/bars"));
    });
}

TEST(HotplugMount, RescanIsNoopWhenAllConfiguredAlreadyRunning)
{
    with_mounted_daemon([](const std::string &mnt) {
        // RESCAN with the one configured device already running: collision-skip
        // means nothing is double-attached and the device stays intact.
        int fd = ::open((mnt + "/hotplug").c_str(), O_RDONLY);
        ASSERT_GE(fd, 0);
        EXPECT_EQ(::ioctl(fd, SLASH_ABI_HOTPLUG_IOCTL_RESCAN), 0)
            << "RESCAN: " << std::strerror(errno);
        ::close(fd);

        const std::string dev = mnt + "/0000:61:00";
        EXPECT_TRUE(path_exists(dev + "/qdma"));
        EXPECT_TRUE(path_exists(dev + "/bars"));
    });
}

}  // namespace
