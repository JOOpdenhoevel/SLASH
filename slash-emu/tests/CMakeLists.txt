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
#
# An optional RESOURCE_LOCK <name> keyword serialises a suite against any other
# suite holding the same lock (CTest never schedules two tests sharing a lock
# concurrently).  Used to keep the CPU-heavy mount/fork/watchdog suites from
# piling onto the same cores under ASan + -j16 -- a belt to the per-test watchdog
# margins (the watchdogs are the real no-hang guard; the lock just keeps the slow
# path from being hit).
function(slash_emu_add_test target)
  cmake_parse_arguments(SEAT "" "RESOURCE_LOCK" "" ${ARGN})
  add_executable(${target} ${SEAT_UNPARSED_ARGUMENTS})
  target_link_libraries(${target} PRIVATE GTest::gtest_main)
  set_target_properties(${target} PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED YES
  )
  target_compile_definitions(${target} PRIVATE
    SLASH_EMUD_PATH="$<TARGET_FILE:slash-emud>"
    SLASH_EMU_TMP_DIR="${SLASH_EMU_TMP_DIR}"
  )
  if(SEAT_RESOURCE_LOCK)
    gtest_discover_tests(${target} PROPERTIES RESOURCE_LOCK ${SEAT_RESOURCE_LOCK})
  else()
    gtest_discover_tests(${target})
  endif()
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
# RESOURCE_LOCK bars_mount: bars_test and bars_adversarial_test both fork a
# child that mmaps a BAR and faults under a watchdog over the real FUSE mount.
# Sharing one lock keeps those two CPU-heavy mount suites from running
# concurrently (with each other or themselves), so the watchdog'd deref is never
# starved of cores by its sibling under ASan + -j16.
slash_emu_add_test(bars_test bars_test.cpp RESOURCE_LOCK bars_mount)
target_link_libraries(bars_test PRIVATE slash_emu_core)

# Adversarial T7 conformance suite: the destructive counterpart of bars_test.
# Exhaustive validation matrix (pure kernel + spine dispatch), integer-overflow
# offset rejection, width==size byte exactness / neighbour integrity, the spine
# write-path revoke gate (-ENODEV both directions, no TOCTOU) + no-write-hook
# (-EIO) + read-only-node confirmation, lazy-shadow no-eager-128MiB (RSS witness)
# + destroy-once (ASan), exactly-three-BARs + isolation, and the no-mmap /
# unbuffered-direct_io guarantees over the real mount.
slash_emu_add_test(bars_adversarial_test bars_adversarial_test.cpp
  RESOURCE_LOCK bars_mount)
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

# ----------------------------------------------------------------------------
# T10: SIM bridge (reconfiguration + vpp_sim model bridge)
# ----------------------------------------------------------------------------

# CI stub model: the standalone address-keyed SIM-dialect server the daemon's
# bridge is tested against (a real vpp_sim cannot run in CI).  Built as its own
# executable and linked against libzmq's C API, exactly as the daemon client is.
add_executable(stub_model stub_model.c)
target_link_libraries(stub_model PRIVATE PkgConfig::ZMQ)
target_compile_options(stub_model PRIVATE -Wall -Wextra -Werror)

# bridge unit suite (T10): the model client protocol vs the stub, the reconfig-
# region write predicate, the bar/qdma backend rc-contract end-to-end through a
# spawned stub, VBIN unpack/locate, and the no-hang failure paths (stub that
# never binds / never answers must not wedge the client).  Links slash_emu_core
# directly (it drives the C bridge/model_client/vbin/qdma APIs) and is handed the
# stub path + a tar helper so it can build a CI VBIN.  Per-test TIMEOUT so a hang
# FAILS rather than blocks.
add_executable(bridge_test bridge_test.cpp)
target_link_libraries(bridge_test PRIVATE GTest::gtest_main slash_emu_core PkgConfig::ZMQ)
set_target_properties(bridge_test PROPERTIES
  CXX_STANDARD 20
  CXX_STANDARD_REQUIRED YES
)
target_compile_definitions(bridge_test PRIVATE
  SLASH_EMU_TMP_DIR="${SLASH_EMU_TMP_DIR}"
  SLASH_EMU_STUB_MODEL_PATH="$<TARGET_FILE:stub_model>"
)
add_dependencies(bridge_test stub_model)
gtest_discover_tests(bridge_test PROPERTIES TIMEOUT 60)

# bridge integration suite (T10): the full end-to-end data plane over the real
# FUSE mount -- write a CI VBIN (containing the stub as vpp_sim) into the reconfig
# region through a qpair, spawn the model, round-trip BAR registers and qpair MM
# transfers THROUGH the model, then remove both functions and assert the model is
# torn down with no leaked process / socket / scratch.  Drives the daemon binary
# and points its scratch root under .tmp via SLASH_EMU_SCRATCH_ROOT.  Per-test
# TIMEOUT so a mount/handshake hang FAILS rather than blocks.
add_executable(bridge_integration_test bridge_integration_test.cpp)
target_link_libraries(bridge_integration_test PRIVATE GTest::gtest_main slash_emu_core)
set_target_properties(bridge_integration_test PROPERTIES
  CXX_STANDARD 20
  CXX_STANDARD_REQUIRED YES
)
target_compile_definitions(bridge_integration_test PRIVATE
  SLASH_EMUD_PATH="$<TARGET_FILE:slash-emud>"
  SLASH_EMU_TMP_DIR="${SLASH_EMU_TMP_DIR}"
  SLASH_EMU_STUB_MODEL_PATH="$<TARGET_FILE:stub_model>"
)
add_dependencies(bridge_integration_test stub_model)
gtest_discover_tests(bridge_integration_test PROPERTIES TIMEOUT 90)

# RESCAN rediscovery suite (task 14): restore an individually-removed function of
# a still-live device.  Unit (against slash_emu_core: the restore building blocks
# -- emu_device_restore_function + bars/qdma re-attach + bridge reattach, the
# no-double-attach idempotency, the remove/restore cycle, and the
# model_shutdown_fired re-arm) and integration over the real FUSE mount with the
# CI stub model (RESCAN restores fn1/fn2 and the data plane round-trips THROUGH
# the still-running model; restore after both-removed comes back model-less and a
# fresh VBIN then spawns a model; RESCAN with nothing removed is a no-op).  Drives
# the daemon binary + points its scratch root under .tmp; per-test TIMEOUT so a
# mount/handshake hang FAILS rather than blocks.
add_executable(rescan_test rescan_test.cpp)
target_link_libraries(rescan_test PRIVATE GTest::gtest_main slash_emu_core)
set_target_properties(rescan_test PROPERTIES
  CXX_STANDARD 20
  CXX_STANDARD_REQUIRED YES
)
target_compile_definitions(rescan_test PRIVATE
  SLASH_EMUD_PATH="$<TARGET_FILE:slash-emud>"
  SLASH_EMU_TMP_DIR="${SLASH_EMU_TMP_DIR}"
  SLASH_EMU_STUB_MODEL_PATH="$<TARGET_FILE:stub_model>"
)
add_dependencies(rescan_test stub_model)
gtest_discover_tests(rescan_test PROPERTIES TIMEOUT 90)

# Adversarial RESCAN rediscovery suite (task 14): the destructive counterpart of
# rescan_test.  Hammers the model_shutdown re-arm bookkeeping WITHOUT re-installing
# the seam (the real daemon never does -- it relies on the fn/ctx surviving a
# per-function revoke), an exhaustive small-depth remove/restore sequence fuzz
# against a reference model, the removed_functions bit integrity, leak/UAF over
# many remove/restore cycles (ASan), and integration over the real FUSE mount:
# the real-daemon double-teardown through the surviving seam, restore-from-both-
# removed reading the FRESH model (not a stale shadow), and select_new staying
# unchanged (no fabricated/unconfigured BDFs).  Per-test TIMEOUT so a hang FAILS.
add_executable(rescan_adversarial_test rescan_adversarial_test.cpp)
target_link_libraries(rescan_adversarial_test
  PRIVATE GTest::gtest_main slash_emu_core)
set_target_properties(rescan_adversarial_test PROPERTIES
  CXX_STANDARD 20
  CXX_STANDARD_REQUIRED YES
)
target_compile_definitions(rescan_adversarial_test PRIVATE
  SLASH_EMUD_PATH="$<TARGET_FILE:slash-emud>"
  SLASH_EMU_TMP_DIR="${SLASH_EMU_TMP_DIR}"
  SLASH_EMU_STUB_MODEL_PATH="$<TARGET_FILE:stub_model>"
)
add_dependencies(rescan_adversarial_test stub_model)
gtest_discover_tests(rescan_adversarial_test PROPERTIES TIMEOUT 90)

# Adversarial T10 conformance suite: the destructive counterpart of the bridge
# tests.  Hammers process/fd/socket/scratch lifecycle (no leak on normal/shutdown/
# re-VBIN/exit/crash/no-bind/no-answer paths, no zombie), no-hang under a dead/hung
# model (watchdog + ctest TIMEOUT so a real hang FAILS), ustar/VBIN parser safety
# (path traversal incl. longname, symlink/hardlink/device entries, truncated/
# oversized/zero/duplicate members, name-length + exec-buffer edges), reconfig
# interception (re-VBIN tears down prior model, reconfig-region read stays -ERANGE
# after model up, large/chunked-VBIN contract), ZMQ desync (die-between-req-and-
# reply latches dead; protocol garbage -> clean -EPROTO not UB), the backend
# rc-contract end-to-end (BAR2/4 + width-8 served from shadow while a model runs,
# never -EIO), and two-device no-cross-talk.  Per-test ctest TIMEOUT so a wedge
# FAILS rather than blocks; the in-test Watchdog (alarm) is the faster inner guard.
add_executable(bridge_adversarial_test bridge_adversarial_test.cpp)
target_link_libraries(bridge_adversarial_test PRIVATE GTest::gtest_main slash_emu_core PkgConfig::ZMQ)
set_target_properties(bridge_adversarial_test PROPERTIES
  CXX_STANDARD 20
  CXX_STANDARD_REQUIRED YES
)
target_compile_definitions(bridge_adversarial_test PRIVATE
  SLASH_EMUD_PATH="$<TARGET_FILE:slash-emud>"
  SLASH_EMU_TMP_DIR="${SLASH_EMU_TMP_DIR}"
  SLASH_EMU_STUB_MODEL_PATH="$<TARGET_FILE:stub_model>"
)
add_dependencies(bridge_adversarial_test stub_model)
gtest_discover_tests(bridge_adversarial_test PROPERTIES TIMEOUT 90)
