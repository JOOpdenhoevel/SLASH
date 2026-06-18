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
#
# The shared ABI conformance suite -- THE single source of truth for the SLASH
# filesystem ABI.  Unlike every other slash-emu test (GTest), this one uses the
# kernel KSELFTEST harness so the SAME binary can later run UNCHANGED against the
# in-kernel filesystem (architecture step 6), not just the FUSE daemon.  It is a
# pure black-box over the mount path: only the UAPI header + raw syscalls.

# Locate the kselftest harness and the UAPI header in the driver tree (the
# conformance suite shares them with the kernel-module kselftests verbatim).
set(SLASH_DRIVER_TESTS "${CMAKE_SOURCE_DIR}/../driver/tests")
set(SLASH_UAPI_INCLUDE "${CMAKE_SOURCE_DIR}/../driver/libslash/include")

add_executable(slash_abi_conformance slash_abi_conformance.c)

# C11 (the harness + UAPI are C); warning-clean with the project's strict set.
set_target_properties(slash_abi_conformance PROPERTIES
  C_STANDARD 11
  C_STANDARD_REQUIRED YES
)
target_include_directories(slash_abi_conformance PRIVATE
  ${SLASH_DRIVER_TESTS}
  ${SLASH_UAPI_INCLUDE}
)
# -Wall -Wextra -Werror across the suite.  The shared kernel kselftest harness
# carries a kernel-mandated signal-handler signature (siginfo_t handler with an
# unused ucontext arg) that trips -Wunused-parameter; the driver's own kselftest
# Makefile builds the harness with -Wall -Werror (no -Wextra) for the same
# reason.  We keep -Wextra for OUR code and silence only that one harness-inherent
# warning, so the suite stays substantively -Wextra-clean.
target_compile_options(slash_abi_conformance PRIVATE
  -Wall -Wextra -Werror -Wno-unused-parameter)

# The CTest runner brings up the FUSE daemon backend via the standard
# emud-scratch.sh, points the suite's mount-root env var at the live mount, runs
# this binary, then tears the daemon down.  It is handed the three paths it needs
# (the binary, the daemon, the stub model packed as the CI VBIN) so it never
# guesses.  Registered SERIAL (it stands up a real mount) with a generous TIMEOUT
# so a wedge FAILS rather than blocks; the kselftest harness bounds each test
# with its own alarm() as the faster inner guard.
add_test(
  NAME conformance
  COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/run_conformance.sh
          $<TARGET_FILE:slash_abi_conformance>
          $<TARGET_FILE:slash-emud>
          $<TARGET_FILE:stub_model>
)
set_tests_properties(conformance PROPERTIES
  RUN_SERIAL YES
  TIMEOUT 180
)
