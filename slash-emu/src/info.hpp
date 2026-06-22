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
 * @file info.hpp
 * @brief The @c /<BDF>/info endpoint: a read-only binary @c struct @c slash_info.
 *
 * The first and simplest of the four endpoints and the reference for the attach
 * pattern the others mirror: an @c <endpoint>Attach() that creates a file node
 * under the device subtree backed by a @ref NodeOps subclass (a @c size hook for
 * @c getattr and a @c read hook for @c pread), then lets the spine and FUSE layer
 * do the rest.
 *
 * @section versioning Read(2) size-versioning convention
 *
 * The ABI versions every struct by a leading @c size word so fields can be
 * appended.  A plain @c read(2) has no [in] channel, so for @c info the
 * convention is one-directional: the file content is a @c struct @c slash_info
 * whose @c size field is the daemon's own @c sizeof, the file size equals that
 * @c sizeof, and a reader self-clamps by how many bytes it requests (an older
 * reader gets a valid prefix; a newer reader gets a short read).
 */

#ifndef SLASH_EMU_INFO_HPP
#define SLASH_EMU_INFO_HPP

#include <cstddef>
#include <sys/types.h>

#include "node.hpp"

namespace slash::emu {

/**
 * @brief Attach the read-only @c info file to a device's @c <BDF>/ directory.
 *
 * Populates a @c struct @c slash_info backing (bdf, acc_type =
 * @c SLASH_ACC_TYPE_SYSTEM_EMULATED, size = the daemon's @c sizeof) and creates
 * an @c info FILE node under @c dev.dir whose ops report that size for @c getattr
 * and serve the struct's bytes for @c pread.
 *
 * @param dev The device to attach to (its @c dir must exist).
 * @return 0 on success, -1 on error (node creation failure).
 */
int infoAttach(Device &dev);

/**
 * @brief Copy a positioned slice out of an in-memory object (pread(2) math).
 *
 * The reusable kernel of every fixed-size read-only endpoint: copy up to @p size
 * bytes starting at @p off from @p src (@p src_len bytes) into @p dst and return
 * the count.  An @p off at/past @p src_len yields 0 (EOF); a straddling read
 * yields the available prefix (short read).  Exposed for unit testing without a
 * FUSE mount.
 *
 * @return Bytes copied (0 at/after EOF), or @c -EINVAL if @p off < 0.
 */
ssize_t infoPreadBuf(char *dst, size_t size, off_t off, const void *src,
                     size_t src_len);

} // namespace slash::emu

#endif // SLASH_EMU_INFO_HPP
