# ##################################################################################################
#  The MIT License (MIT)
#  Copyright (c) 2026 Advanced Micro Devices, Inc. All rights reserved.
#
#  Permission is hereby granted, free of charge, to any person obtaining a copy of this software
#  and associated documentation files (the "Software"), to deal in the Software without restriction,
#  including without limitation the rights to use, copy, modify, merge, publish, distribute,
#  sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
#  furnished to do so, subject to the following conditions:
#
#  The above copyright notice and this permission notice shall be included in all copies or
#  substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
# NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
# NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
# DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
# ##################################################################################################

include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/refs/tags/v1.17.0.zip
)
FetchContent_MakeAvailable(googletest)

include(GoogleTest)

# Repo-level scratch dir: all temp mountpoints/logs live under .tmp, never /tmp.
set(SLASH_EMU_TMP_DIR "${CMAKE_SOURCE_DIR}/../.tmp")

# Every test exec is handed the freshly-built daemon path and the scratch dir as
# compile definitions so it never hand-runs or guesses a binary (CTest drives
# everything).
function(slash_emu_add_test target)
  add_executable(${target} ${ARGN})
  target_link_libraries(${target} PRIVATE GTest::gtest_main)
  set_target_properties(${target} PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED YES
  )
  target_compile_definitions(${target} PRIVATE
    SLASH_EMUD_PATH="$<TARGET_FILE:slash-emud>"
    SLASH_EMU_TMP_DIR="${SLASH_EMU_TMP_DIR}"
  )
  gtest_discover_tests(${target})
endfunction()

# End-to-end mount/browse/unmount smoke test.
slash_emu_add_test(smoke_test smoke_test.cpp)

# Adversarial lifecycle / robustness suite (codified T3 audit): CLI error
# paths, mount-failure cleanup, no-leak idempotency, mount/unmount de-race.
slash_emu_add_test(daemon_lifecycle_test daemon_lifecycle_test.cpp)

# Config parser + accelerator model unit tests.  Links the daemon core directly
# (it calls the C config API), and is handed the path to the shipped sample
# config so it can assert that what we ship actually parses.
slash_emu_add_test(config_test config_test.cpp)
target_link_libraries(config_test PRIVATE slash_emu_core)
target_compile_definitions(config_test PRIVATE
  SLASH_EMU_SAMPLE_CONF="${CMAKE_SOURCE_DIR}/conf/slash-emu.conf"
)

# Spine unit tests: node tree, per-device registry, refcounted resources, and
# the revocation state machine.  Exercises the pure C data model in node.c
# directly (no FUSE session needed).
slash_emu_add_test(node_test node_test.cpp)
target_link_libraries(node_test PRIVATE slash_emu_core)

# Adversarial conformance suite for the spine (T5): exhaustive refcount drop
# orders, idempotent teardown triggers, the dead-orphan revocation state machine,
# nameless-qpair lifetime, notifier re-entry, and OOM/error-path cleanup (ASan).
slash_emu_add_test(node_revocation_test node_revocation_test.cpp)
target_link_libraries(node_revocation_test PRIVATE slash_emu_core)

# End-to-end: the spine materializes the configured device tree over the real
# FUSE mount (lookup / getattr / readdir wired to the node model).
slash_emu_add_test(fs_tree_test fs_tree_test.cpp)

# info endpoint (T6): unit (pread/offset math + struct population and the
# read-dispatch liveness gate, against slash_emu_core) and integration (read
# /<BDF>/info over the real FUSE mount).
slash_emu_add_test(info_test info_test.cpp)
target_link_libraries(info_test PRIVATE slash_emu_core)

# Adversarial T6 conformance suite: the destructive counterpart of info_test.
# Hammers the SPINE touches that shipped with the info endpoint -- the
# cleanup_node_tree destroy-hook pass (exactly-once / no double-destroy across
# the cooperative / revoke-orphan / normal paths and orthogonality vs the T5
# resource passes), the emu_node_pread error matrix + check/read atomicity, the
# info pread/struct semantics + per-device isolation, and emu_info_attach
# ownership / failure paths.
slash_emu_add_test(info_adversarial_test info_adversarial_test.cpp)
target_link_libraries(info_adversarial_test PRIVATE slash_emu_core)

# bars endpoint (T7): unit (the emu_bar_check_access validation matrix + register
# round-trip / geometry / revocation through the spine, against slash_emu_core)
# and integration (pread/pwrite bar0/2/4 over the real FUSE mount, alignment /
# width / beyond-BAR rejection, and the assertion that mmap is not offered).
slash_emu_add_test(bars_test bars_test.cpp)
target_link_libraries(bars_test PRIVATE slash_emu_core)

# Adversarial T7 conformance suite: the destructive counterpart of bars_test.
# Exhaustive validation matrix (pure kernel + spine dispatch), integer-overflow
# offset rejection, width==size byte exactness / neighbour integrity, the spine
# write-path revoke gate (-ENODEV both directions, no TOCTOU) + no-write-hook
# (-EIO) + read-only-node confirmation, lazy-shadow no-eager-128MiB (RSS witness)
# + destroy-once (ASan), exactly-three-BARs + isolation, and the no-mmap /
# unbuffered-direct_io guarantees over the real mount.
slash_emu_add_test(bars_adversarial_test bars_adversarial_test.cpp)
target_link_libraries(bars_adversarial_test PRIVATE slash_emu_core)

# OOM-injection coverage for the BAR shadow allocation (T7 defect-1): a
# link-time --wrap=calloc interposer makes bar_write's first-write shadow calloc
# fail on demand so the -ENOMEM path is exercised deterministically.  The wrap
# applies to THIS executable only (slash_emu_core's bars.c is linked in), so no
# other test's allocations are affected.
slash_emu_add_test(bars_oom_test bars_oom_test.cpp)
target_link_libraries(bars_oom_test PRIVATE slash_emu_core)
target_link_options(bars_oom_test PRIVATE -Wl,--wrap=calloc)

# qdma endpoint (T8): unit (the emu_qdma_check_range + emu_qdma_check_qpair_add
# validation matrices, plus the full qpair lifecycle through the spine -- QID
# allocation, MM round-trip into HBM/DDR, out-of-range rejection, nameless-qpair
# lifecycle, forced/cooperative teardown idempotency, per-device isolation,
# against slash_emu_core) and integration (QPAIR_ADD ioctl + qpair pread/pwrite,
# unlink-while-open VRTD pattern, readdir visibility over the real FUSE mount).
slash_emu_add_test(qdma_test qdma_test.cpp)
target_link_libraries(qdma_test PRIVATE slash_emu_core)

# Adversarial T8 conformance suite: the destructive counterpart of qdma_test.
# Hammers the _locked spine refactor (QPAIR_ADD's whole create/register/attach/
# set_direct_io chain under the held non-recursive lock -- self-deadlock watchdog
# + per-tree concurrency stress + public/locked equivalence), the ioctl buffer
# matrix (short/oversized/zero in&out, unknown cmd, unknown-inode -ENOENT, post-
# revoke -ENODEV, malformed dir_mask/ring), the full qpair lifecycle matrix
# (cooperative-exactly-once, forced -ENODEV, double-teardown, unlink-not-open,
# unlink-dir -EISDIR / non-dir-parent -ENOTDIR, QID reuse-after-free), the sparse
# store (multi-page straddle, exhaustive -ERANGE incl. reconfig+overflow, lazy
# alloc RSS witness, destroy-frees-once ASan), the T10 mem-backend rc-contract
# via a fake backend (rc==0 model / rc>0 store fallback / rc<0 -ENODEV; populate
# keeps store current), and the qpair direct_io flag.  Per-test ctest TIMEOUTs so
# a deadlock fails the suite rather than blocking it (the Watchdog is the inner
# guard, the ctest TIMEOUT the outer backstop).
add_executable(qdma_adversarial_test qdma_adversarial_test.cpp)
target_link_libraries(qdma_adversarial_test PRIVATE GTest::gtest_main slash_emu_core)
set_target_properties(qdma_adversarial_test PROPERTIES
  CXX_STANDARD 20
  CXX_STANDARD_REQUIRED YES
)
target_compile_definitions(qdma_adversarial_test PRIVATE
  SLASH_EMUD_PATH="$<TARGET_FILE:slash-emud>"
  SLASH_EMU_TMP_DIR="${SLASH_EMU_TMP_DIR}"
)
# Per-test ctest TIMEOUT: a self-deadlock in the _locked chain must FAIL the
# suite, never block it.  60s is far above the healthy runtime (<1s) yet below
# any reasonable CI wall-clock; the in-test Watchdog (setitimer) is the faster
# inner guard.
gtest_discover_tests(qdma_adversarial_test PROPERTIES TIMEOUT 60)

# hotplug endpoint (T9): unit (BDF-with-function parsing, per-function REMOVE,
# the revocation matrix for a removed function, the both-functions-removed
# model-shutdown seam, RESCAN/SBR/HOTPLUG reload sequencing via a stub reload,
# the ioctl buffer/versioning matrix, the materialize re-invocation seed
# primitives, and the injectable SBR sleep, against slash_emu_core) and
# integration (the /hotplug file at root, REMOVE makes a function's subtree
# vanish while the sibling survives, -ENODEV on an open handle of a removed
# endpoint + -ENOENT on reopen, HOTPLUG re-inits a removed device, and RESCAN
# collision-skip over the real FUSE mount).  Per-test ctest TIMEOUT so a mount
# hang or a misconfigured SBR sleep FAILS rather than blocks the suite.
add_executable(hotplug_test hotplug_test.cpp)
target_link_libraries(hotplug_test PRIVATE GTest::gtest_main slash_emu_core)
set_target_properties(hotplug_test PROPERTIES
  CXX_STANDARD 20
  CXX_STANDARD_REQUIRED YES
)
target_compile_definitions(hotplug_test PRIVATE
  SLASH_EMUD_PATH="$<TARGET_FILE:slash-emud>"
  SLASH_EMU_TMP_DIR="${SLASH_EMU_TMP_DIR}"
)
gtest_discover_tests(hotplug_test PROPERTIES TIMEOUT 60)

# Adversarial T9 conformance suite: the destructive counterpart of hotplug_test.
# Hammers the highest-risk spine/locking pieces -- ioctl_unlocked SAFETY (only
# hotplug is unlocked, the unlocked hook re-enters the self-locking spine without
# deadlock, and the "never unlinked" invariant the dropped-lock pointer relies on
# is PROBED for the FUSE-unlink path that can destroy/free the node), per-function
# REVOKE in both orders with the surviving function asserted fully usable + device
# not marked dead, model-shutdown EXACTLY-ONCE incl. the REMOVE-both-then-SBR
# double-fire guard, NO double-attach across repeated materialize seeds, and the
# BDF-with-function + ioctl buffer adversarial edges.  Per-test ctest TIMEOUT so a
# reentrancy deadlock FAILS rather than blocks the suite.
add_executable(hotplug_adversarial_test hotplug_adversarial_test.cpp)
target_link_libraries(hotplug_adversarial_test PRIVATE GTest::gtest_main slash_emu_core)
set_target_properties(hotplug_adversarial_test PROPERTIES
  CXX_STANDARD 20
  CXX_STANDARD_REQUIRED YES
)
target_compile_definitions(hotplug_adversarial_test PRIVATE
  SLASH_EMUD_PATH="$<TARGET_FILE:slash-emud>"
  SLASH_EMU_TMP_DIR="${SLASH_EMU_TMP_DIR}"
)
gtest_discover_tests(hotplug_adversarial_test PROPERTIES TIMEOUT 60)
