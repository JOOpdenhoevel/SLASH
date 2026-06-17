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

# Greenfield code: build the daemon and its core library under a strict warning
# set with warnings-as-errors.  Scoped to our own targets only (via
# target_compile_options) -- the FetchContent'd GoogleTest is built with its own
# flags and must not be subject to these.
set(SLASH_EMU_WARNING_FLAGS -Wall -Wextra -Werror)

# Static library with all daemon logic except main(), so tests can link against
# it directly (mirrors vrtd's vrtd_core split).
add_library(slash_emu_core STATIC
  ${CMAKE_CURRENT_SOURCE_DIR}/config.c
  ${CMAKE_CURRENT_SOURCE_DIR}/fs.c
)

target_compile_options(slash_emu_core PRIVATE ${SLASH_EMU_WARNING_FLAGS})

target_include_directories(slash_emu_core
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(slash_emu_core
  PUBLIC
    PkgConfig::FUSE3
    PkgConfig::SYSTEMD
  PRIVATE
    PkgConfig::INIH
)

# Daemon executable.
add_executable(slash-emud ${CMAKE_CURRENT_SOURCE_DIR}/main.c)

target_compile_options(slash-emud PRIVATE ${SLASH_EMU_WARNING_FLAGS})

target_link_libraries(slash-emud PRIVATE slash_emu_core)
