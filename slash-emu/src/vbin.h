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
 * @file vbin.h
 * @brief VBIN unpacking for the SIM bridge (T10).
 *
 * A VBIN is a POSIX @b ustar tar archive (the same format @c vrt/src/vrtbin.cpp
 * reads).  For the step-1 SIM path the daemon needs only one member: the
 * @c vpp_sim executable, located by filename anywhere in the tree (matching
 * @c vrtbin.cpp's @c findExtractedFile("vpp_sim")).  @c system_map.xml is not
 * required for SIM (the daemon forwards addresses 1:1 and needs no interval map;
 * see @c docs/bridge-design.md §2.2/§4), so it is unpacked if present but not
 * required.
 *
 * This unpacker is deliberately minimal: uncompressed ustar, regular files +
 * directories + the GNU long-name (@c 'L') extension, with strict path-traversal
 * rejection (no absolute paths, no @c "..").  Executable bits from the tar mode
 * are preserved so the located @c vpp_sim passes @c access(X_OK) for exec.  The
 * CI VBIN is produced by @c tar(1) (see @c docs/bridge-protocol.md §3.1).
 */

#ifndef SLASH_EMU_VBIN_H
#define SLASH_EMU_VBIN_H

#include <stddef.h>

/**
 * @brief Classification of a (possibly partial) ustar byte stream.
 *
 * Used by the reconfiguration reassembly path (the kernel splits a multi-MB VBIN
 * write into several @c ops->write chunks; the bridge accumulates them and asks
 * "is the archive complete yet?" after each).  Completion is driven off the tar
 * structure itself -- the two trailing 512-byte zero blocks -- rather than a byte
 * count, so the bridge needs no out-of-band length.
 */
enum emu_vbin_status {
    EMU_VBIN_INCOMPLETE = 0, /**< Well-formed so far but no terminator yet. */
    EMU_VBIN_COMPLETE = 1,   /**< The end-of-archive terminator was reached. */
    EMU_VBIN_INVALID = 2,    /**< Structurally broken (bad header / unparsable). */
};

/**
 * @brief Classify a (possibly partial) ustar archive without unpacking it.
 *
 * Walks the tar headers in @p data (@p len bytes) and reports whether the stream
 * is a @em complete archive (the zero-block terminator was reached at a
 * 512-aligned boundary), @em incomplete (well-formed up to the bytes available
 * but not yet terminated -- the caller should accept more), or @em invalid (a
 * header is structurally broken, so no amount of further bytes will help).
 *
 * A non-block-aligned trailing remainder is treated as @c INCOMPLETE (a chunk
 * boundary fell mid-block), not invalid -- the caller keeps accumulating.  An
 * empty stream is @c INCOMPLETE.
 *
 * @param data The accumulated bytes so far.
 * @param len  Number of bytes available.
 * @return one of @ref emu_vbin_status.
 */
enum emu_vbin_status emu_vbin_classify(const void *data, size_t len);

/**
 * @brief Unpack a VBIN archive into @p dest_dir and locate the @c vpp_sim member.
 *
 * Extracts every member of the ustar archive @p vbin under @p dest_dir
 * (preserving the mode bits, so an executable member stays executable), then
 * finds the file named @c "vpp_sim" anywhere in the unpacked tree and writes its
 * absolute path into @p exec_out.  @p dest_dir must already exist.
 *
 * @param      vbin     The VBIN bytes (whole archive).
 * @param      len      Archive length in bytes.
 * @param      dest_dir Existing directory to unpack into.
 * @param[out] exec_out Buffer for the located @c vpp_sim path.
 * @param      exec_len Size of @p exec_out.
 * @return 0 on success (a runnable @c vpp_sim was found); a negative errno on a
 *         malformed archive, a path-traversal attempt, an I/O error, or a missing
 *         / non-executable @c vpp_sim member.
 */
int emu_vbin_unpack_find_sim(const void *vbin, size_t len, const char *dest_dir,
                             char *exec_out, size_t exec_len);

#endif // SLASH_EMU_VBIN_H
