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
