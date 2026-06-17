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
 * @file info.h
 * @brief The @c /<BDF>/info endpoint: a read-only binary @c struct @c slash_info.
 *
 * This is the first and simplest of the four endpoints (info / bars / qdma /
 * hotplug) and the reference for the attach pattern the others mirror: an
 * @c emu_<endpoint>_attach() that creates a file node under the device subtree
 * with an @ref emu_node_ops vtable (a @c size hook for @c getattr, a @c read
 * hook for @c pread, and a @c destroy hook to free the node's backing), then
 * lets the spine and the FUSE layer do the rest.
 *
 * @section versioning Read(2) size-versioning convention
 *
 * The ABI versions every struct by a leading @c size word so fields can be
 * appended without breaking old callers.  For an @em ioctl that word is
 * bidirectional: the caller passes its @c sizeof in, the kernel clamps to
 * @c min(in, its own sizeof) and writes back what it filled.  A plain
 * @c read(2) has no such [in] channel -- there is nowhere for the reader to
 * deliver its @c sizeof before the bytes come back.  So for @c info the
 * convention is one-directional and offset-driven:
 *
 *   - The file content @em is a @c struct @c slash_info whose @c size field is
 *     set to the daemon's own @c sizeof(struct slash_info) (an [out] value).
 *   - The file's reported size (via @c getattr) equals that same
 *     @c sizeof(struct slash_info).
 *   - A reader does @c read(buf, n) and gets @c min(n, filesize - offset)
 *     bytes.  An older reader that knows a shorter struct asks for fewer bytes
 *     and receives a valid prefix; a newer reader asks for its larger struct
 *     and receives only what exists (a short read), learning the daemon's
 *     actual struct size from the returned @c size field and the byte count.
 *
 * In short: there is no [in] size on a read; the reader self-clamps by how many
 * bytes it requests, and @c size is purely informational [out].
 */

#ifndef SLASH_EMU_INFO_H
#define SLASH_EMU_INFO_H

#include <stddef.h>
#include <sys/types.h>

#include "node.h"

/**
 * @brief Attach the read-only @c info file to a device's @c <BDF>/ directory.
 *
 * Allocates and populates a @c struct @c slash_info backing (bdf, acc_type =
 * @c SLASH_ACC_TYPE_SYSTEM_EMULATED, size = the daemon's
 * @c sizeof(struct slash_info)) and creates an @c info FILE node under
 * @c dev->dir whose ops report that size for @c getattr and serve the struct's
 * bytes for @c pread.  The backing is freed by the node's @c destroy hook.
 *
 * @param dev The device to attach the endpoint to (its @c dir must exist).
 * @return 0 on success, -1 on error (allocation / node creation failure).
 */
int emu_info_attach(struct emu_device *dev);

/**
 * @brief Copy a positioned slice out of an in-memory object (pread(2) math).
 *
 * The reusable kernel of every fixed-size read-only endpoint: given a source
 * buffer of @p src_len bytes, copy up to @p size bytes starting at @p off into
 * @p dst and return the count.  Embodies the read(2) contract this endpoint
 * relies on: an @p off at or past @p src_len yields 0 (EOF); an @p off + @p size
 * that straddles the end yields the available prefix (a short read).
 *
 * Exposed (rather than file-static) so the offset/short-read logic can be
 * unit-tested without standing up a FUSE mount.
 *
 * @param dst     Destination buffer (at least @p size bytes).
 * @param size    Maximum bytes to copy.
 * @param off     Source offset (>= 0).
 * @param src     Source object.
 * @param src_len Source length in bytes.
 * @return Bytes copied (0 at/after EOF), or @c -EINVAL if @p off < 0.
 */
ssize_t emu_info_pread_buf(char *dst, size_t size, off_t off, const void *src,
                           size_t src_len);

#endif // SLASH_EMU_INFO_H
