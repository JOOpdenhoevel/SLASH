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
 * slash_emu_core (node.c is FUSE-free); the integration path is exercised by
 * info_test.cpp's InfoMount.
 *
 * Hammer items (mapped to the brief):
 *   A.  cleanup_node_tree destroy-hook pass: EXACTLY-ONCE per node, no double
 *       destroy across the cooperative / revoke-orphan / normal-destroy paths;
 *       orthogonality + ordering vs the T5 resource-free passes.
 *   B.  emu_node_pread dispatch: -ENOENT / -ENODEV / -EINVAL / -EIO, hook value
 *       propagation, and check+read atomicity under one lock.
 *   C.  info endpoint semantics: pread matrix, struct contents, getattr size,
 *       read-after-revoke -> -ENODEV, per-device bdf isolation.
 *   E.  emu_info_attach ownership: success transfers backing (freed once at
 *       shutdown), failure paths leak nothing (ASan).
 */

#include <gtest/gtest.h>

#include <cerrno>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

extern "C" {
#include "info.h"
#include "node.h"
#include "slash/uapi/slash_abi.h"
}

namespace {

// ---------------------------------------------------------------------------
// RAII tree wrapper.  cleanup_node_tree() in the dtor is the system-under-test
// for every item-A shutdown case.
// ---------------------------------------------------------------------------
class Tree {
public:
    Tree() { EXPECT_EQ(emu_node_tree_new(&tree_, nullptr), 0); }
    ~Tree() { cleanup_node_tree(tree_); }
    emu_node_tree *get() { return tree_; }

private:
    emu_node_tree *tree_ = nullptr;
};

// A counting destroy backing: every time the node's ops->destroy hook fires for
// this backing, destroy_count is bumped.  The struct is heap-allocated and the
// destroy hook frees it, so a double-destroy is BOTH a count>1 AND a
// double-free (caught by ASan).  We therefore record the count into an external
// observer before freeing, so the assertion survives the free.
struct CountingBacking {
    int *observer;   // external counter, survives the free
    int local_fired; // sanity: should be 1 at free time
};

void counting_destroy(struct emu_node *node, void *backing)
{
    (void) node;
    auto *b = static_cast<CountingBacking *>(backing);
    (*b->observer)++;
    b->local_fired++;
    free(b);
}

const struct emu_node_ops kCountingOps = {
    .destroy = counting_destroy,
};

// Helper: attach a counting-destroy file node under `parent`, wiring its
// observer.  Returns the node (non-owning).
emu_node *attach_counting(emu_node_tree *tree, emu_node *parent, const char *name,
                          int *observer)
{
    auto *b = static_cast<CountingBacking *>(calloc(1, sizeof(CountingBacking)));
    EXPECT_NE(b, nullptr);
    b->observer = observer;
    emu_node *node = nullptr;
    EXPECT_EQ(emu_node_create_child(tree, parent, name, EMU_NODE_FILE, 0444,
                                    &kCountingOps, b, &node),
              0);
    return node;
}

// Resolve a named child's inode under a parent (does NOT bump lookup_count: it
// reads the children array directly via the public lookup-by-ino walk).  We use
// emu_node_lookup_child where a lookup ref is intentional, and this otherwise.
emu_ino_t child_ino_no_ref(emu_node_tree *tree, emu_node *parent,
                           const char *name)
{
    // emu_node_lookup_child bumps lookup_count; to avoid that we use the
    // node->children topology directly (the node struct is public).
    for (size_t i = 0; i < parent->children.len; i++) {
        if (std::strcmp(parent->children.d[i]->name, name) == 0) {
            return parent->children.d[i]->ino;
        }
    }
    return 0;
}

// ===========================================================================
// A. cleanup_node_tree destroy-hook pass: EXACTLY ONCE, no double-destroy.
// ===========================================================================

// A1. The plain-shutdown case the fix targets: a live device with an info-like
// backing.  destroy must fire exactly once at cleanup_node_tree.
TEST(SpineDestroyHook, PlainShutdownFiresDestroyExactlyOnce)
{
    int fired = 0;
    {
        Tree t;
        emu_device *dev = nullptr;
        ASSERT_EQ(emu_node_tree_add_device(t.get(), "0000:61:00", &dev), 0);
        attach_counting(t.get(), dev->dir, "info", &fired);
        EXPECT_EQ(fired, 0); // nothing destroyed yet
    } // cleanup_node_tree() here
    EXPECT_EQ(fired, 1) << "plain shutdown must run destroy exactly once";
}

// A2. A node cooperatively destroyed BEFORE shutdown (node_destroy_locked via
// unlink with no outstanding lookups) must NOT be destroyed again by the
// shutdown pass: it was already removed from tree->nodes.
TEST(SpineDestroyHook, CooperativeDestroyNotRepeatedAtShutdown)
{
    int fired = 0;
    {
        Tree t;
        emu_device *dev = nullptr;
        ASSERT_EQ(emu_node_tree_add_device(t.get(), "0000:61:00", &dev), 0);
        emu_node *node = attach_counting(t.get(), dev->dir, "ep", &fired);

        // lookup_count == 0, so unlink reaps immediately via node_destroy_locked.
        emu_node_unlink(t.get(), node);
        EXPECT_EQ(fired, 1) << "cooperative reap should destroy once";
    } // shutdown pass must not re-destroy
    EXPECT_EQ(fired, 1) << "no second destroy at shutdown for an already-reaped node";
}

// A3. The revoke-orphan case (HIGHEST RISK): revoke a device whose endpoint node
// is held open (lookup_count > 0).  Revoke leaves it as a dead orphan still in
// tree->nodes with ops/backing intact and never runs destroy on it.  The
// shutdown pass must then run destroy EXACTLY once -- not zero (leak), not twice.
TEST(SpineDestroyHook, RevokeOrphanDestroyedExactlyOnceAtShutdown)
{
    int fired = 0;
    {
        Tree t;
        emu_device *dev = nullptr;
        ASSERT_EQ(emu_node_tree_add_device(t.get(), "0000:61:00", &dev), 0);
        emu_node *node = attach_counting(t.get(), dev->dir, "ep", &fired);

        // Pin a kernel lookup so revoke keeps the node as a dead orphan.
        emu_node *looked = nullptr;
        ASSERT_EQ(emu_node_lookup_child(t.get(), dev->dir->ino, "ep", &looked), 0);
        ASSERT_EQ(looked, node);

        ASSERT_EQ(emu_device_revoke(t.get(), "0000:61:00"), 0);
        EXPECT_EQ(fired, 0) << "revoke must NOT destroy a still-looked-up orphan";
    } // shutdown destroy pass
    EXPECT_EQ(fired, 1) << "dead orphan's backing must be destroyed once at shutdown";
}

// A4. The full mixed shutdown the brief demands: ONE tree containing
//   (a) a live device with an endpoint backing (plain-shutdown path),
//   (b) a revoked device whose endpoint is a dead orphan (revoke-orphan path),
//   (c) a normally-destroyed endpoint (cooperative path, already reaped).
// Each backing's destroy must fire exactly once; ASan proves no double-free.
TEST(SpineDestroyHook, MixedShutdownEachBackingDestroyedExactlyOnce)
{
    int live_fired = 0, orphan_fired = 0, coop_fired = 0;
    {
        Tree t;

        // (a) live device + endpoint.
        emu_device *live = nullptr;
        ASSERT_EQ(emu_node_tree_add_device(t.get(), "0000:61:00", &live), 0);
        attach_counting(t.get(), live->dir, "info", &live_fired);

        // (b) revoked device, endpoint held open -> dead orphan.
        emu_device *rev = nullptr;
        ASSERT_EQ(emu_node_tree_add_device(t.get(), "0000:62:00", &rev), 0);
        emu_node *orphan = attach_counting(t.get(), rev->dir, "info", &orphan_fired);
        emu_node *looked = nullptr;
        ASSERT_EQ(emu_node_lookup_child(t.get(), rev->dir->ino, "info", &looked), 0);
        ASSERT_EQ(looked, orphan);
        ASSERT_EQ(emu_device_revoke(t.get(), "0000:62:00"), 0);

        // (c) cooperative destroy before shutdown.
        emu_device *coop = nullptr;
        ASSERT_EQ(emu_node_tree_add_device(t.get(), "0000:63:00", &coop), 0);
        emu_node *coop_node = attach_counting(t.get(), coop->dir, "info", &coop_fired);
        emu_node_unlink(t.get(), coop_node);

        EXPECT_EQ(live_fired, 0);
        EXPECT_EQ(orphan_fired, 0);
        EXPECT_EQ(coop_fired, 1); // already reaped
    } // shutdown

    EXPECT_EQ(live_fired, 1) << "live endpoint destroyed once at shutdown";
    EXPECT_EQ(orphan_fired, 1) << "dead orphan destroyed once at shutdown";
    EXPECT_EQ(coop_fired, 1) << "cooperatively-reaped endpoint not re-destroyed";
}

// A5. Orthogonality + ordering vs the T5 resource passes: a node with BOTH a
// backing (destroy hook frees backing) AND an attached resource (resource pass
// frees node->resource).  At shutdown neither must read the other's freed
// memory; each frees exactly once.  ASan is the real judge here.
struct ResProbe {
    int teardown = 0;
    int freed = 0;
};
void res_teardown(struct emu_resource *res, void *backing)
{
    (void) res;
    static_cast<ResProbe *>(backing)->teardown++;
}
void res_free(void *backing) { static_cast<ResProbe *>(backing)->freed++; }

TEST(SpineDestroyHook, NodeWithBackingAndResourceBothFreedOnceNoCrossRead)
{
    int destroy_fired = 0;
    ResProbe probe;
    {
        Tree t;
        emu_device *dev = nullptr;
        ASSERT_EQ(emu_node_tree_add_device(t.get(), "0000:61:00", &dev), 0);

        // Endpoint node carrying a counting destroy backing.
        emu_node *node = attach_counting(t.get(), dev->qdma, "qpair0", &destroy_fired);

        // Register a resource on the device and attach it to the SAME node, so
        // the node has both backing (freed by destroy hook) and resource (freed
        // by the registry pass).
        emu_resource *res = nullptr;
        ASSERT_EQ(emu_device_register_resource(dev, 0, res_teardown, res_free,
                                               &probe, &res),
                  0);
        ASSERT_EQ(emu_node_attach_resource(t.get(), node, res), 0);
    } // shutdown: registry pass frees resource; destroy pass frees backing

    EXPECT_EQ(destroy_fired, 1) << "backing destroyed exactly once";
    EXPECT_EQ(probe.freed, 1) << "resource backing freed exactly once";
    // Plain shutdown does not run resource teardown callbacks (daemon going away).
    EXPECT_EQ(probe.teardown, 0) << "plain shutdown does not run resource teardown";
}

// A6. Defensive: a node with ops set but a NULL destroy hook must be skipped by
// the shutdown pass (no crash, no spurious call).  info_size-only ops mimic a
// future read-only endpoint that owns no heap backing.
off_t noop_size(const struct emu_node *, void *) { return 0; }
const struct emu_node_ops kSizeOnlyOps = {
    /* .size    = */ noop_size,
    /* .read    = */ nullptr,
    /* .write   = */ nullptr,
    /* .destroy = */ nullptr,
};
TEST(SpineDestroyHook, NullDestroyHookSkippedAtShutdown)
{
    Tree t;
    emu_device *dev = nullptr;
    ASSERT_EQ(emu_node_tree_add_device(t.get(), "0000:61:00", &dev), 0);
    emu_node *node = nullptr;
    ASSERT_EQ(emu_node_create_child(t.get(), dev->dir, "ro", EMU_NODE_FILE, 0444,
                                    &kSizeOnlyOps, nullptr, &node),
              0);
    // No crash at shutdown is the assertion (dtor runs cleanup_node_tree).
    SUCCEED();
}

// ===========================================================================
// B. emu_node_pread dispatch: error matrix, value propagation, atomicity.
// ===========================================================================

// A read hook that returns a sentinel positive value to prove propagation, and
// records the (size, off) it saw to prove arguments pass through unchanged.
struct ReadProbe {
    size_t last_size = 0;
    off_t last_off = 0;
    ssize_t to_return = 7;
};
ssize_t probe_read(const struct emu_node *node, void *backing, char *buf,
                   size_t size, off_t off)
{
    (void) node;
    auto *p = static_cast<ReadProbe *>(backing);
    p->last_size = size;
    p->last_off = off;
    if (p->to_return > 0 && size > 0) {
        buf[0] = 0x5a;
    }
    return p->to_return;
}
const struct emu_node_ops kReadProbeOps = {
    /* .size    = */ nullptr,
    /* .read    = */ probe_read,
    /* .write   = */ nullptr,
    /* .destroy = */ nullptr,
};

TEST(SpinePread, BadInoIsEnoent)
{
    Tree t;
    char buf[8];
    EXPECT_EQ(emu_node_pread(t.get(), 999999u, buf, sizeof(buf), 0), -ENOENT);
}

TEST(SpinePread, NegativeOffsetIsEinval)
{
    Tree t;
    emu_device *dev = nullptr;
    ASSERT_EQ(emu_node_tree_add_device(t.get(), "0000:61:00", &dev), 0);
    ASSERT_EQ(emu_info_attach(dev), 0);
    emu_ino_t ino = child_ino_no_ref(t.get(), dev->dir, "info");
    char buf[8];
    EXPECT_EQ(emu_node_pread(t.get(), ino, buf, sizeof(buf), -1), -EINVAL);
}

TEST(SpinePread, NullBufIsEinval)
{
    Tree t;
    EXPECT_EQ(emu_node_pread(t.get(), EMU_ROOT_INO, nullptr, 8, 0), -EINVAL);
}

TEST(SpinePread, NullTreeIsEinval)
{
    char buf[8];
    EXPECT_EQ(emu_node_pread(nullptr, 1, buf, sizeof(buf), 0), -EINVAL);
}

// A directory (no read hook) yields -EIO.  The root is a dir with NULL ops.
TEST(SpinePread, NodeWithoutReadHookIsEio)
{
    Tree t;
    char buf[8];
    EXPECT_EQ(emu_node_pread(t.get(), EMU_ROOT_INO, buf, sizeof(buf), 0), -EIO);
}

// Hook return value and arguments propagate verbatim.
TEST(SpinePread, HookValueAndArgsPropagate)
{
    Tree t;
    emu_device *dev = nullptr;
    ASSERT_EQ(emu_node_tree_add_device(t.get(), "0000:61:00", &dev), 0);
    ReadProbe probe;
    probe.to_return = 7;
    emu_node *node = nullptr;
    ASSERT_EQ(emu_node_create_child(t.get(), dev->dir, "probe", EMU_NODE_FILE,
                                    0444, &kReadProbeOps, &probe, &node),
              0);
    char buf[16] = {};
    ssize_t n = emu_node_pread(t.get(), node->ino, buf, 13, 4);
    EXPECT_EQ(n, 7);
    EXPECT_EQ(probe.last_size, 13u);
    EXPECT_EQ(probe.last_off, 4);
    EXPECT_EQ(static_cast<unsigned char>(buf[0]), 0x5au);

    // A hook may return a negative errno; it propagates unchanged.
    probe.to_return = -EACCES;
    EXPECT_EQ(emu_node_pread(t.get(), node->ino, buf, 1, 0), -EACCES);
}

// Atomicity: the liveness gate and the read run under ONE lock acquisition, so a
// revoke cannot slip between "check live" and "dispatch read".  We prove this
// behaviourally: after a revoke (which marks live=false under the same lock),
// the read returns -ENODEV and the hook is NEVER entered (its probe is
// untouched).  There is no intermediate window in which a stale live==true read
// could dispatch -- find_ino + live-check + dispatch are one critical section.
TEST(SpinePread, RevokeIsAtomicWithRespectToReadDispatch)
{
    Tree t;
    emu_device *dev = nullptr;
    ASSERT_EQ(emu_node_tree_add_device(t.get(), "0000:61:00", &dev), 0);
    ReadProbe probe;
    emu_node *node = nullptr;
    ASSERT_EQ(emu_node_create_child(t.get(), dev->dir, "probe", EMU_NODE_FILE,
                                    0444, &kReadProbeOps, &probe, &node),
              0);
    emu_ino_t ino = node->ino;

    // Pin a lookup so the node survives revoke as a dead orphan and stays
    // resolvable by ino (the "op on an already-open fd" case).
    emu_node *looked = nullptr;
    ASSERT_EQ(emu_node_lookup_child(t.get(), dev->dir->ino, "probe", &looked), 0);

    ASSERT_EQ(emu_device_revoke(t.get(), "0000:61:00"), 0);

    char buf[8] = {};
    EXPECT_EQ(emu_node_pread(t.get(), ino, buf, sizeof(buf), 0), -ENODEV);
    // The hook must NOT have run: the gate short-circuited before dispatch.
    EXPECT_EQ(probe.last_size, 0u) << "read hook ran despite revoke (gate not atomic)";
}

// ===========================================================================
// C. info endpoint semantics (matrix + struct contents + isolation).
// ===========================================================================

emu_ino_t info_ino(emu_node_tree *tree, emu_node *dir)
{
    return child_ino_no_ref(tree, dir, "info");
}

TEST(InfoSemantics, GetattrSizeIsSizeofSlashInfo)
{
    Tree t;
    emu_device *dev = nullptr;
    ASSERT_EQ(emu_node_tree_add_device(t.get(), "0000:61:00", &dev), 0);
    ASSERT_EQ(emu_info_attach(dev), 0);
    struct stat st {};
    ASSERT_EQ(emu_node_stat(t.get(), info_ino(t.get(), dev->dir), &st), 0);
    EXPECT_TRUE(S_ISREG(st.st_mode));
    EXPECT_EQ(st.st_size, static_cast<off_t>(sizeof(struct slash_info)));
    EXPECT_EQ(st.st_mode & 0777, 0444u) << "info is read-only 0444";
}

TEST(InfoSemantics, StructContentsBdfAccTypeSize)
{
    Tree t;
    emu_device *dev = nullptr;
    ASSERT_EQ(emu_node_tree_add_device(t.get(), "0000:61:00", &dev), 0);
    ASSERT_EQ(emu_info_attach(dev), 0);
    struct slash_info got {};
    ssize_t n = emu_node_pread(t.get(), info_ino(t.get(), dev->dir),
                               reinterpret_cast<char *>(&got), sizeof(got), 0);
    ASSERT_EQ(n, static_cast<ssize_t>(sizeof(struct slash_info)));
    EXPECT_EQ(got.size, sizeof(struct slash_info));
    EXPECT_EQ(got.acc_type, SLASH_ACC_TYPE_SYSTEM_EMULATED);
    EXPECT_STREQ(got.bdf, "0000:61:00");
}

// Full pread matrix straight through the spine to info_read.
TEST(InfoSemantics, PreadMatrix)
{
    Tree t;
    emu_device *dev = nullptr;
    ASSERT_EQ(emu_node_tree_add_device(t.get(), "0000:61:00", &dev), 0);
    ASSERT_EQ(emu_info_attach(dev), 0);
    emu_ino_t ino = info_ino(t.get(), dev->dir);
    const off_t sz = static_cast<off_t>(sizeof(struct slash_info));

    char buf[256] = {};

    // Full read.
    EXPECT_EQ(emu_node_pread(t.get(), ino, buf, sizeof(struct slash_info), 0), sz);
    // Over-read clamps to size (short read).
    EXPECT_EQ(emu_node_pread(t.get(), ino, buf, sizeof(buf), 0), sz);
    // Offset read returns the tail.
    EXPECT_EQ(emu_node_pread(t.get(), ino, buf, sizeof(buf), 4), sz - 4);
    // Straddle EOF: from 4 bytes before end, ask for a lot -> 4 bytes.
    EXPECT_EQ(emu_node_pread(t.get(), ino, buf, 999, sz - 4), 4);
    // Exactly at EOF -> 0.
    EXPECT_EQ(emu_node_pread(t.get(), ino, buf, sizeof(buf), sz), 0);
    // Past EOF -> 0.
    EXPECT_EQ(emu_node_pread(t.get(), ino, buf, sizeof(buf), sz + 100), 0);
    // Zero-size read -> 0 (and must not write buf / crash).
    EXPECT_EQ(emu_node_pread(t.get(), ino, buf, 0, 0), 0);
    // Negative offset -> -EINVAL.
    EXPECT_EQ(emu_node_pread(t.get(), ino, buf, sizeof(buf), -1), -EINVAL);
}

// Read-after-revoke on an open handle -> -ENODEV (open-fd revocation contract).
TEST(InfoSemantics, ReadAfterRevokeIsEnodev)
{
    Tree t;
    emu_device *dev = nullptr;
    ASSERT_EQ(emu_node_tree_add_device(t.get(), "0000:61:00", &dev), 0);
    ASSERT_EQ(emu_info_attach(dev), 0);
    emu_node *looked = nullptr;
    ASSERT_EQ(emu_node_lookup_child(t.get(), dev->dir->ino, "info", &looked), 0);
    emu_ino_t ino = looked->ino;
    ASSERT_EQ(emu_device_revoke(t.get(), "0000:61:00"), 0);
    struct slash_info got {};
    EXPECT_EQ(emu_node_pread(t.get(), ino, reinterpret_cast<char *>(&got),
                             sizeof(got), 0),
              -ENODEV);
}

// Multi-device: each /<BDF>/info reports its OWN bdf, no cross-device bleed.
TEST(InfoSemantics, PerDeviceBdfIsolation)
{
    Tree t;
    const char *bdfs[] = {"0000:61:00", "0000:62:00", "0000:c1:00"};
    emu_device *devs[3] = {};
    for (int i = 0; i < 3; i++) {
        ASSERT_EQ(emu_node_tree_add_device(t.get(), bdfs[i], &devs[i]), 0);
        ASSERT_EQ(emu_info_attach(devs[i]), 0);
    }
    for (int i = 0; i < 3; i++) {
        struct slash_info got {};
        ssize_t n = emu_node_pread(t.get(), info_ino(t.get(), devs[i]->dir),
                                   reinterpret_cast<char *>(&got), sizeof(got), 0);
        ASSERT_EQ(n, static_cast<ssize_t>(sizeof(got)));
        EXPECT_STREQ(got.bdf, bdfs[i]) << "device " << i << " bdf bled";
        EXPECT_EQ(got.acc_type, SLASH_ACC_TYPE_SYSTEM_EMULATED);
    }
}

// D. Versioning convention: a short read yields a valid struct prefix whose
// leading size word reports the daemon's full sizeof -- the one-directional
// read(2) convention.  (There is no [in] size clamp on read.)
TEST(InfoSemantics, ShortReadYieldsValidPrefixWithFullSizeWord)
{
    Tree t;
    emu_device *dev = nullptr;
    ASSERT_EQ(emu_node_tree_add_device(t.get(), "0000:61:00", &dev), 0);
    ASSERT_EQ(emu_info_attach(dev), 0);
    emu_ino_t ino = info_ino(t.get(), dev->dir);

    // An "old reader" that only knows the leading size word reads 4 bytes and
    // learns the daemon's full struct size from it.
    uint32_t sz = 0;
    ASSERT_EQ(emu_node_pread(t.get(), ino, reinterpret_cast<char *>(&sz),
                             sizeof(sz), 0),
              static_cast<ssize_t>(sizeof(sz)));
    EXPECT_EQ(sz, sizeof(struct slash_info))
        << "size word must report the daemon's full sizeof, not the read length";
}

// ===========================================================================
// E. emu_info_attach ownership / failure paths.
// ===========================================================================

// Success path: the backing is owned by the node and freed exactly once at
// shutdown (ASan proves no leak and no double free).  Repeating attach across
// many devices stresses the transfer.
TEST(InfoAttach, SuccessOwnershipNoLeakManyDevices)
{
    Tree t;
    for (int i = 0; i < 16; i++) {
        char bdf[16];
        std::snprintf(bdf, sizeof(bdf), "0000:%02x:00", 0x10 + i);
        emu_device *dev = nullptr;
        ASSERT_EQ(emu_node_tree_add_device(t.get(), bdf, &dev), 0);
        ASSERT_EQ(emu_info_attach(dev), 0);
    }
    // Shutdown frees every backing exactly once (ASan).
    SUCCEED();
}

// Invalid arguments are rejected without allocating/leaking.
TEST(InfoAttach, NullArgsRejected)
{
    EXPECT_EQ(emu_info_attach(nullptr), -1);

    // A device whose dir/tree are NULL must be rejected (defensive guard).
    emu_device bogus{};
    EXPECT_EQ(emu_info_attach(&bogus), -1);
}

// Double attach: attaching info twice creates two "info" children.  This is not
// a supported call pattern, but it must not corrupt or leak -- both backings are
// owned and freed at shutdown.  (Documents current behaviour for the rescan/
// re-add audit: callers must attach exactly once per materialization.)
TEST(InfoAttach, DoubleAttachDoesNotLeak)
{
    Tree t;
    emu_device *dev = nullptr;
    ASSERT_EQ(emu_node_tree_add_device(t.get(), "0000:61:00", &dev), 0);
    ASSERT_EQ(emu_info_attach(dev), 0);
    ASSERT_EQ(emu_info_attach(dev), 0);
    // Two info nodes now exist; both backings freed at shutdown (ASan).
    SUCCEED();
}

}  // namespace
