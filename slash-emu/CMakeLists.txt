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

cmake_minimum_required(VERSION 3.22.1)

project(slash-emu
  LANGUAGES C CXX
)

# C11 with GNU extensions throughout, per vrt/vrtd/STYLE.md.
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

option(SLASH_EMU_BUILD_TESTS "Build slash-emu tests" OFF)

option(ENABLE_SANITIZERS "Build with AddressSanitizer and UBSan" OFF)
if(ENABLE_SANITIZERS)
  add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer)
  add_link_options(-fsanitize=address,undefined)
endif()

find_package(PkgConfig REQUIRED)

# pthreads: the node tree / registry / refcounts are mutex-guarded so the data
# model is safe for a future multi-threaded FUSE session.
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

# libfuse3 low-level API (we need fuse_lowlevel_notify_* in later tasks).
pkg_check_modules(FUSE3 REQUIRED IMPORTED_TARGET fuse3)

# libsystemd: sd-event loop, sd-journal logging, sd_notify readiness.
pkg_check_modules(SYSTEMD REQUIRED IMPORTED_TARGET libsystemd)

# libinih: INI config parser, taken from the distro package exactly as vrtd's
# config layer does (pkg_check_modules + PkgConfig::INIH).  Stock libinih ships
# INI_CALL_HANDLER_ON_NEW_SECTION OFF, so a section fires no callback until it
# carries at least one key; the accelerator schema accounts for this -- each
# [accelerator:<bdf>] section must carry >=1 key, and a keyless section is
# silently ignored (see config.h and conf/slash-emu.conf).
pkg_check_modules(INIH REQUIRED IMPORTED_TARGET inih)

# libzmq (C API, zmq.h): the SIM bridge's daemon-side REQ client to the running
# vpp_sim model (T10).  The daemon is C11, so we use the C API, not cppzmq.
pkg_check_modules(ZMQ REQUIRED IMPORTED_TARGET libzmq)

# enable_testing() at the top level so ctest discovers tests from the
# top-level build dir (slash-emu/build).
enable_testing()

add_subdirectory(src)

if(SLASH_EMU_BUILD_TESTS)
  add_subdirectory(tests)
  # The shared ABI conformance kselftest suite depends on the stub_model target
  # defined in tests/, so it is added after it.
  add_subdirectory(conformance)
endif()
