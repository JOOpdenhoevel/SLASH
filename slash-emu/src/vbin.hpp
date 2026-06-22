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
 * @file vbin.hpp
 * @brief VBIN unpacking for the SIM bridge (C++20).
 *
 * A VBIN is a POSIX @b ustar tar archive (the same format @c vrt/src/vrtbin.cpp
 * reads).  For the SIM path the daemon needs only one member: the @c vpp_sim
 * executable, located by filename anywhere in the tree.  @c system_map.xml is
 * unpacked if present but not required for SIM.
 *
 * The unpacker is deliberately minimal: uncompressed ustar, regular files +
 * directories + the GNU long-name (@c 'L') extension, with strict path-traversal
 * rejection (no absolute paths, no @c "..").  Executable bits are preserved so the
 * located @c vpp_sim passes @c access(X_OK).
 */

#ifndef SLASH_EMU_VBIN_HPP
#define SLASH_EMU_VBIN_HPP

#include <cstddef>
#include <span>
#include <string>

namespace slash::emu {

/**
 * @brief Classification of a (possibly partial) ustar byte stream.
 *
 * Used by the reconfiguration reassembly path: the kernel splits a multi-MB VBIN
 * write into chunks; the bridge accumulates them and asks "complete yet?" after
 * each.  Completion is driven off the tar structure (the two trailing 512-byte
 * zero blocks), not a byte count.
 */
enum class VbinStatus {
    Incomplete = 0, /**< Well-formed so far but no terminator yet. */
    Complete = 1,   /**< The end-of-archive terminator was reached. */
    Invalid = 2,    /**< Structurally broken (bad header / unparsable). */
};

/**
 * @brief Classify a (possibly partial) ustar archive without unpacking it.
 *
 * Walks the tar headers in @p data and reports @ref VbinStatus::Complete (the
 * zero-block terminator was reached at a 512-aligned boundary),
 * @ref VbinStatus::Incomplete (well-formed up to the bytes available; accept
 * more), or @ref VbinStatus::Invalid (a header is structurally broken).  A
 * non-block-aligned trailing remainder and an empty stream are @c Incomplete.
 */
VbinStatus vbinClassify(std::span<const std::byte> data);

/**
 * @brief Unpack a VBIN archive into @p dest_dir and locate the @c vpp_sim member.
 *
 * Extracts every member of the ustar archive @p vbin under @p dest_dir (preserving
 * mode bits) and finds the file named @c "vpp_sim" anywhere in the unpacked tree.
 * @p dest_dir must already exist.
 *
 * @param      vbin     The whole archive bytes.
 * @param      dest_dir Existing directory to unpack into.
 * @param[out] exec_out Receives the absolute path of the located @c vpp_sim.
 * @return 0 on success (a runnable @c vpp_sim was found); a negative errno on a
 *         malformed archive, a path-traversal attempt, an I/O error, or a missing
 *         / non-executable @c vpp_sim member.
 */
int vbinUnpackFindSim(std::span<const std::byte> vbin, const std::string &dest_dir,
                      std::string &exec_out);

} // namespace slash::emu

#endif // SLASH_EMU_VBIN_HPP
