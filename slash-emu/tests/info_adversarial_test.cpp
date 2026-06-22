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
 * @file info_adversarial_test.cpp
 * @brief Adversarial T6 conformance suite (the "T6 tester" hammer list).
 *
 * This file is the codified counterpart of the adversarial review of the T6
 * info endpoint and the SPINE touches it shipped with.  It is intentionally
 * separate from the implementer's info_test.cpp so the destructive cases have
 * a single home and clear attribution.  Everything is a pure unit test against
 * slash_emu_core (node.cpp is FUSE-free); the integration path is exercised by
 * info_test.cpp's InfoMount.
 *
 * Hammer items (mapped to the brief):
 *   A.  NodeTree destructor destroy pass: EXACTLY-ONCE per node, no double
 *       destroy across the cooperative / revoke-orphan / normal-destroy paths;
 *       orthogonality + ordering vs the T5 resource-free passes.
 *   B.  tree.pread dispatch: -ENOENT / -ENODEV / -EINVAL / -EIO, hook value
 *       propagation, and check+read atomicity under one lock.
 *   C.  info endpoint semantics: pread matrix, struct contents, getattr size,
 *       read-after-revoke -> -ENODEV, per-device bdf isolation.
 *   E.  infoAttach ownership: success transfers backing (freed once at
 *       shutdown), failure paths leak nothing (ASan).
 */

#include <gtest/gtest.h>

#include <cerrno>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include "info.hpp"
#include "node.hpp"
#include "slash/uapi/slash_abi.h"

using namespace slash::emu;

namespace {

// ---------------------------------------------------------------------------
// RAII tree wrapper.  NodeTree destructor is the system-under-test for every
// item-A shutdown case.
// ---------------------------------------------------------------------------
class Tree {
public:
    Tree() = default;
    NodeTree &get() { return tree_; }

private:
    NodeTree tree_;
};

// A counting destroy NodeOps: when the object is destructed (node destroyed),
// it bumps *observer_.  In the C++ port the "destroy hook" is simply the
// NodeOps destructor.  A double-destroy is caught by ASan (double-free) AND
// by observer > 1.
class CountingOps : public NodeOps {
public:
    explicit CountingOps(int *observer) : observer_(observer) {}
    ~CountingOps() override { (*observer_)++; }

private:
    int *observer_;
};

// Helper: attach a counting-destroy file node under `parent`.
// Returns the node (non-owning).
Node *attach_counting(NodeTree &tree, Node *parent, const char *name,
                      int *observer)
{
    return tree.createChild(parent, name, NodeType::File, 0444,
                            std::make_unique<CountingOps>(observer));
}

// Resolve a named child's inode under a parent WITHOUT bumping lookup_count.
// We use lookupChild where a lookup ref is intentional, and this otherwise.
Ino child_ino_no_ref(NodeTree &tree, Node *parent, const char *name)
{
    // Walk children directly to avoid bumping lookup_count.
    for (Node *child : parent->children) {
        if (child->name == name) {
            return child->ino;
        }
    }
    return 0;
}

// ===========================================================================
// A. NodeTree destructor destroy pass: EXACTLY ONCE, no double-destroy.
// ===========================================================================

// A1. The plain-shutdown case: a live device with an info-like backing.
// The NodeOps destructor must fire exactly once at NodeTree destruction.
TEST(SpineDestroyHook, PlainShutdownFiresDestroyExactlyOnce)
{
    int fired = 0;
    {
        Tree t;
        Device *dev = t.get().addDevice("0000:61:00");
        ASSERT_NE(dev, nullptr);
        attach_counting(t.get(), dev->dir, "info", &fired);
        EXPECT_EQ(fired, 0); // nothing destroyed yet
    } // NodeTree destructor here
    EXPECT_EQ(fired, 1) << "plain shutdown must run destroy exactly once";
}

// A2. A node cooperatively destroyed BEFORE shutdown (unlink with no
// outstanding lookups) must NOT be destroyed again by the shutdown pass.
TEST(SpineDestroyHook, CooperativeDestroyNotRepeatedAtShutdown)
{
    int fired = 0;
    {
        Tree t;
        Device *dev = t.get().addDevice("0000:61:00");
        ASSERT_NE(dev, nullptr);
        Node *node = attach_counting(t.get(), dev->dir, "ep", &fired);

        // lookup_count == 0, so unlink reaps immediately.
        t.get().unlink(node);
        EXPECT_EQ(fired, 1) << "cooperative reap should destroy once";
    } // shutdown pass must not re-destroy
    EXPECT_EQ(fired, 1) << "no second destroy at shutdown for an already-reaped node";
}

// A3. The revoke-orphan case (HIGHEST RISK): revoke a device whose endpoint
// node is held open (lookup_count > 0).  Revoke leaves it as a dead orphan
// still in the tree's node list with ops intact and never runs destroy on it.
// The shutdown pass must then run destroy EXACTLY once.
TEST(SpineDestroyHook, RevokeOrphanDestroyedExactlyOnceAtShutdown)
{
    int fired = 0;
    {
        Tree t;
        Device *dev = t.get().addDevice("0000:61:00");
        ASSERT_NE(dev, nullptr);
        Node *node = attach_counting(t.get(), dev->dir, "ep", &fired);

        // Pin a kernel lookup so revoke keeps the node as a dead orphan.
        Node *looked = nullptr;
        ASSERT_EQ(t.get().lookupChild(dev->dir->ino, "ep", &looked), 0);
        ASSERT_EQ(looked, node);

        ASSERT_EQ(t.get().revokeDevice("0000:61:00"), 0);
        EXPECT_EQ(fired, 0) << "revoke must NOT destroy a still-looked-up orphan";
    } // shutdown destroy pass
    EXPECT_EQ(fired, 1) << "dead orphan's ops must be destroyed once at shutdown";
}

// A4. The full mixed shutdown: ONE tree containing
//   (a) a live device with an endpoint (plain-shutdown path),
//   (b) a revoked device whose endpoint is a dead orphan (revoke-orphan path),
//   (c) a normally-destroyed endpoint (cooperative path, already reaped).
// Each NodeOps destructor must fire exactly once; ASan proves no double-free.
TEST(SpineDestroyHook, MixedShutdownEachNodeDestroyedExactlyOnce)
{
    int live_fired = 0, orphan_fired = 0, coop_fired = 0;
    {
        Tree t;

        // (a) live device + endpoint.
        Device *live = t.get().addDevice("0000:61:00");
        ASSERT_NE(live, nullptr);
        attach_counting(t.get(), live->dir, "info", &live_fired);

        // (b) revoked device, endpoint held open -> dead orphan.
        Device *rev = t.get().addDevice("0000:62:00");
        ASSERT_NE(rev, nullptr);
        Node *orphan = attach_counting(t.get(), rev->dir, "info", &orphan_fired);
        Node *looked = nullptr;
        ASSERT_EQ(t.get().lookupChild(rev->dir->ino, "info", &looked), 0);
        ASSERT_EQ(looked, orphan);
        ASSERT_EQ(t.get().revokeDevice("0000:62:00"), 0);

        // (c) cooperative destroy before shutdown.
        Device *coop = t.get().addDevice("0000:63:00");
        ASSERT_NE(coop, nullptr);
        Node *coop_node = attach_counting(t.get(), coop->dir, "info", &coop_fired);
        t.get().unlink(coop_node);

        EXPECT_EQ(live_fired, 0);
        EXPECT_EQ(orphan_fired, 0);
        EXPECT_EQ(coop_fired, 1); // already reaped
    } // shutdown

    EXPECT_EQ(live_fired, 1) << "live endpoint destroyed once at shutdown";
    EXPECT_EQ(orphan_fired, 1) << "dead orphan destroyed once at shutdown";
    EXPECT_EQ(coop_fired, 1) << "cooperatively-reaped endpoint not re-destroyed";
}

// A5. Orthogonality + ordering vs the T5 resource passes: a node with BOTH a
// NodeOps (destructor frees state) AND an attached resource (spine drops the
// shared_ptr).  At shutdown neither must read the other's freed memory; each
// frees exactly once.  ASan is the real judge here.
struct ResProbe {
    int teardown = 0;
    int freed = 0;
};

TEST(SpineDestroyHook, NodeWithOpsAndResourceBothFreedOnceNoCrossRead)
{
    int destroy_fired = 0;
    ResProbe probe;
    {
        Tree t;
        Device *dev = t.get().addDevice("0000:61:00");
        ASSERT_NE(dev, nullptr);

        // Endpoint node carrying a counting destroy NodeOps.
        Node *node = attach_counting(t.get(), dev->qdma, "qpair0", &destroy_fired);

        // Register a resource on the device and attach it to the SAME node.
        Resource *res = dev->registerResource(
            0, [&probe](Resource &) { probe.teardown++; });
        ASSERT_NE(res, nullptr);
        res->backing = std::shared_ptr<void>(
            &probe, [](void *p) {
                static_cast<ResProbe *>(p)->freed++;
            });
        ASSERT_EQ(t.get().attachResource(node, res->shared_from_this()), 0);
    } // shutdown: resource freed; ops destructor runs

    EXPECT_EQ(destroy_fired, 1) << "node ops destroyed exactly once";
    EXPECT_EQ(probe.freed, 1) << "resource backing freed exactly once";
    // Plain shutdown does not run resource teardown callbacks.
    EXPECT_EQ(probe.teardown, 0) << "plain shutdown does not run resource teardown";
}

// A6. Defensive: a node with ops set but no extra state -- like a size-only
// ops -- must not crash at shutdown (the base NodeOps destructor is a no-op).
class SizeOnlyOps : public NodeOps {
public:
    off_t size(const Node &) const override { return 0; }
};

TEST(SpineDestroyHook, NullBodyOpsSkippedAtShutdown)
{
    Tree t;
    Device *dev = t.get().addDevice("0000:61:00");
    ASSERT_NE(dev, nullptr);
    t.get().createChild(dev->dir, "ro", NodeType::File, 0444,
                        std::make_unique<SizeOnlyOps>());
    // No crash at shutdown is the assertion.
    SUCCEED();
}

// ===========================================================================
// B. tree.pread dispatch: error matrix, value propagation, atomicity.
// ===========================================================================

// A read hook that returns a sentinel positive value to prove propagation,
// and records the (size, off) it saw to prove arguments pass through unchanged.
struct ReadProbe {
    size_t last_size = 0;
    off_t last_off = 0;
    ssize_t to_return = 7;
};

class ReadProbeOps : public NodeOps {
public:
    explicit ReadProbeOps(ReadProbe *probe) : probe_(probe) {}

    ssize_t read(const Node & /*node*/, char *buf, size_t sz,
                 off_t off) override
    {
        probe_->last_size = sz;
        probe_->last_off = off;
        if (probe_->to_return > 0 && sz > 0) {
            buf[0] = 0x5a;
        }
        return probe_->to_return;
    }

private:
    ReadProbe *probe_;
};

TEST(SpinePread, BadInoIsEnoent)
{
    Tree t;
    char buf[8];
    EXPECT_EQ(t.get().pread(999999u, buf, sizeof(buf), 0), -ENOENT);
}

TEST(SpinePread, NegativeOffsetIsEinval)
{
    Tree t;
    Device *dev = t.get().addDevice("0000:61:00");
    ASSERT_NE(dev, nullptr);
    ASSERT_EQ(infoAttach(*dev), 0);
    Ino ino = child_ino_no_ref(t.get(), dev->dir, "info");
    char buf[8];
    EXPECT_EQ(t.get().pread(ino, buf, sizeof(buf), -1), -EINVAL);
}

TEST(SpinePread, NullBufIsEinval)
{
    Tree t;
    EXPECT_EQ(t.get().pread(kRootIno, nullptr, 8, 0), -EINVAL);
}

// A directory (no read hook) yields -EIO.  The root is a dir with nullptr ops.
TEST(SpinePread, NodeWithoutReadHookIsEio)
{
    Tree t;
    char buf[8];
    EXPECT_EQ(t.get().pread(kRootIno, buf, sizeof(buf), 0), -EIO);
}

// Hook return value and arguments propagate verbatim.
TEST(SpinePread, HookValueAndArgsPropagate)
{
    Tree t;
    Device *dev = t.get().addDevice("0000:61:00");
    ASSERT_NE(dev, nullptr);
    ReadProbe probe;
    probe.to_return = 7;
    Node *node = t.get().createChild(dev->dir, "probe", NodeType::File, 0444,
                                     std::make_unique<ReadProbeOps>(&probe));
    ASSERT_NE(node, nullptr);

    char buf[16] = {};
    ssize_t n = t.get().pread(node->ino, buf, 13, 4);
    EXPECT_EQ(n, 7);
    EXPECT_EQ(probe.last_size, 13u);
    EXPECT_EQ(probe.last_off, 4);
    EXPECT_EQ(static_cast<unsigned char>(buf[0]), 0x5au);

    // A hook may return a negative errno; it propagates unchanged.
    probe.to_return = -EACCES;
    EXPECT_EQ(t.get().pread(node->ino, buf, 1, 0), -EACCES);
}

// Atomicity: the liveness gate and the read run under ONE lock acquisition.
// After a revoke (which marks live=false under the same lock), the read
// returns -ENODEV and the hook is NEVER entered.
TEST(SpinePread, RevokeIsAtomicWithRespectToReadDispatch)
{
    Tree t;
    Device *dev = t.get().addDevice("0000:61:00");
    ASSERT_NE(dev, nullptr);
    ReadProbe probe;
    Node *node = t.get().createChild(dev->dir, "probe", NodeType::File, 0444,
                                     std::make_unique<ReadProbeOps>(&probe));
    ASSERT_NE(node, nullptr);
    Ino ino = node->ino;

    // Pin a lookup so the node survives revoke as a dead orphan.
    Node *looked = nullptr;
    ASSERT_EQ(t.get().lookupChild(dev->dir->ino, "probe", &looked), 0);

    ASSERT_EQ(t.get().revokeDevice("0000:61:00"), 0);

    char buf[8] = {};
    EXPECT_EQ(t.get().pread(ino, buf, sizeof(buf), 0), -ENODEV);
    // The hook must NOT have run: the gate short-circuited before dispatch.
    EXPECT_EQ(probe.last_size, 0u) << "read hook ran despite revoke (gate not atomic)";
}

// ===========================================================================
// C. info endpoint semantics (matrix + struct contents + isolation).
// ===========================================================================

Ino info_ino(NodeTree &tree, Node *dir)
{
    return child_ino_no_ref(tree, dir, "info");
}

TEST(InfoSemantics, GetattrSizeIsSizeofSlashInfo)
{
    Tree t;
    Device *dev = t.get().addDevice("0000:61:00");
    ASSERT_NE(dev, nullptr);
    ASSERT_EQ(infoAttach(*dev), 0);
    struct stat st{};
    ASSERT_EQ(t.get().stat(info_ino(t.get(), dev->dir), &st), 0);
    EXPECT_TRUE(S_ISREG(st.st_mode));
    EXPECT_EQ(st.st_size, static_cast<off_t>(sizeof(struct slash_info)));
    EXPECT_EQ(st.st_mode & 0777u, 0444u) << "info is read-only 0444";
}

TEST(InfoSemantics, StructContentsBdfAccTypeSize)
{
    Tree t;
    Device *dev = t.get().addDevice("0000:61:00");
    ASSERT_NE(dev, nullptr);
    ASSERT_EQ(infoAttach(*dev), 0);
    struct slash_info got{};
    ssize_t n = t.get().pread(info_ino(t.get(), dev->dir),
                               reinterpret_cast<char *>(&got), sizeof(got), 0);
    ASSERT_EQ(n, static_cast<ssize_t>(sizeof(struct slash_info)));
    EXPECT_EQ(got.size, sizeof(struct slash_info));
    EXPECT_EQ(got.acc_type, SLASH_ACC_TYPE_SYSTEM_EMULATED);
    EXPECT_STREQ(got.bdf, "0000:61:00");
}

// Full pread matrix straight through the spine to the info read hook.
TEST(InfoSemantics, PreadMatrix)
{
    Tree t;
    Device *dev = t.get().addDevice("0000:61:00");
    ASSERT_NE(dev, nullptr);
    ASSERT_EQ(infoAttach(*dev), 0);
    Ino ino = info_ino(t.get(), dev->dir);
    const off_t sz = static_cast<off_t>(sizeof(struct slash_info));

    char buf[256] = {};

    // Full read.
    EXPECT_EQ(t.get().pread(ino, buf, sizeof(struct slash_info), 0), sz);
    // Over-read clamps to size (short read).
    EXPECT_EQ(t.get().pread(ino, buf, sizeof(buf), 0), sz);
    // Offset read returns the tail.
    EXPECT_EQ(t.get().pread(ino, buf, sizeof(buf), 4), sz - 4);
    // Straddle EOF: from 4 bytes before end, ask for a lot -> 4 bytes.
    EXPECT_EQ(t.get().pread(ino, buf, 999, sz - 4), 4);
    // Exactly at EOF -> 0.
    EXPECT_EQ(t.get().pread(ino, buf, sizeof(buf), sz), 0);
    // Past EOF -> 0.
    EXPECT_EQ(t.get().pread(ino, buf, sizeof(buf), sz + 100), 0);
    // Zero-size read -> 0 (and must not write buf / crash).
    EXPECT_EQ(t.get().pread(ino, buf, 0, 0), 0);
    // Negative offset -> -EINVAL.
    EXPECT_EQ(t.get().pread(ino, buf, sizeof(buf), -1), -EINVAL);
}

// Read-after-revoke on an open handle -> -ENODEV (open-fd revocation contract).
TEST(InfoSemantics, ReadAfterRevokeIsEnodev)
{
    Tree t;
    Device *dev = t.get().addDevice("0000:61:00");
    ASSERT_NE(dev, nullptr);
    ASSERT_EQ(infoAttach(*dev), 0);
    Node *looked = nullptr;
    ASSERT_EQ(t.get().lookupChild(dev->dir->ino, "info", &looked), 0);
    Ino ino = looked->ino;
    ASSERT_EQ(t.get().revokeDevice("0000:61:00"), 0);
    struct slash_info got{};
    EXPECT_EQ(t.get().pread(ino, reinterpret_cast<char *>(&got),
                             sizeof(got), 0),
              -ENODEV);
}

// Multi-device: each /<BDF>/info reports its OWN bdf, no cross-device bleed.
TEST(InfoSemantics, PerDeviceBdfIsolation)
{
    Tree t;
    const char *bdfs[] = {"0000:61:00", "0000:62:00", "0000:c1:00"};
    Device *devs[3] = {};
    for (int i = 0; i < 3; i++) {
        devs[i] = t.get().addDevice(bdfs[i]);
        ASSERT_NE(devs[i], nullptr);
        ASSERT_EQ(infoAttach(*devs[i]), 0);
    }
    for (int i = 0; i < 3; i++) {
        struct slash_info got{};
        ssize_t n = t.get().pread(info_ino(t.get(), devs[i]->dir),
                                   reinterpret_cast<char *>(&got), sizeof(got), 0);
        ASSERT_EQ(n, static_cast<ssize_t>(sizeof(got)));
        EXPECT_STREQ(got.bdf, bdfs[i]) << "device " << i << " bdf bled";
        EXPECT_EQ(got.acc_type, SLASH_ACC_TYPE_SYSTEM_EMULATED);
    }
}

// D. Versioning convention: a short read yields a valid struct prefix whose
// leading size word reports the daemon's full sizeof.
TEST(InfoSemantics, ShortReadYieldsValidPrefixWithFullSizeWord)
{
    Tree t;
    Device *dev = t.get().addDevice("0000:61:00");
    ASSERT_NE(dev, nullptr);
    ASSERT_EQ(infoAttach(*dev), 0);
    Ino ino = info_ino(t.get(), dev->dir);

    uint32_t sz = 0;
    ASSERT_EQ(t.get().pread(ino, reinterpret_cast<char *>(&sz),
                             sizeof(sz), 0),
              static_cast<ssize_t>(sizeof(sz)));
    EXPECT_EQ(sz, sizeof(struct slash_info))
        << "size word must report the daemon's full sizeof, not the read length";
}

// ===========================================================================
// E. infoAttach ownership / failure paths.
// ===========================================================================

// Success path: the backing is owned by the node and freed exactly once at
// shutdown (ASan proves no leak and no double free).
TEST(InfoAttach, SuccessOwnershipNoLeakManyDevices)
{
    Tree t;
    for (int i = 0; i < 16; i++) {
        char bdf[16];
        std::snprintf(bdf, sizeof(bdf), "0000:%02x:00", 0x10 + i);
        Device *dev = t.get().addDevice(bdf);
        ASSERT_NE(dev, nullptr);
        ASSERT_EQ(infoAttach(*dev), 0);
    }
    // Shutdown frees every backing exactly once (ASan).
    SUCCEED();
}

// Invalid arguments are rejected without allocating/leaking.
TEST(InfoAttach, NullDirRejected)
{
    // A device whose dir is nullptr must be rejected.
    NodeTree tree;
    Device *dev = tree.addDevice("0000:61:00");
    ASSERT_NE(dev, nullptr);
    // Temporarily null out dir and restore to simulate the guard.
    Node *saved = dev->dir;
    dev->dir = nullptr;
    EXPECT_EQ(infoAttach(*dev), -1);
    dev->dir = saved;
}

// Double attach: attaching info twice creates two "info" children.  This is
// not a supported call pattern, but it must not corrupt or leak -- both
// NodeOps objects are owned and destroyed at shutdown.
TEST(InfoAttach, DoubleAttachDoesNotLeak)
{
    Tree t;
    Device *dev = t.get().addDevice("0000:61:00");
    ASSERT_NE(dev, nullptr);
    ASSERT_EQ(infoAttach(*dev), 0);
    ASSERT_EQ(infoAttach(*dev), 0);
    // Two info nodes now exist; both NodeOps freed at shutdown (ASan).
    SUCCEED();
}

}  // namespace
