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
 * @file info.cpp
 * @brief Implementation of the @c /<BDF>/info endpoint (see info.hpp).
 *
 * The info file's backing is a single fully-populated @c slash_info, built once
 * at attach time and held inside @ref InfoOps as an immutable member.  It never
 * changes for the device's lifetime, so reads need no locking of their own
 * beyond the spine's (the tree lock is held across NodeOps::read, and the struct
 * is immutable after construction anyway).
 */

#include "info.hpp"

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <syslog.h>

#include "slash/uapi/slash_abi.h"

namespace slash::emu {

// ---------------------------------------------------------------------------
// infoPreadBuf -- the reusable pread(2) math kernel
// ---------------------------------------------------------------------------

/**
 * @brief Copy a positioned slice out of an in-memory object (pread(2) math).
 *
 * The reusable kernel of every fixed-size read-only endpoint: copy up to
 * @p size bytes starting at @p off from @p src (@p src_len bytes) into @p dst
 * and return the count.  An @p off at/past @p src_len yields 0 (EOF); a
 * straddling read yields the available prefix (short read).
 *
 * @return Bytes copied (0 at/after EOF), or @c -EINVAL if @p off < 0.
 */
ssize_t infoPreadBuf(char *dst, size_t size, off_t off, const void *src,
                     size_t src_len)
{
    if (off < 0) {
        return -EINVAL;
    }

    /* At or past EOF: nothing to copy (a valid zero-length read). */
    if (static_cast<size_t>(off) >= src_len) {
        return 0;
    }

    size_t avail = src_len - static_cast<size_t>(off);
    size_t n = size < avail ? size : avail; /* short read when size > avail */

    std::memcpy(dst, static_cast<const char *>(src) + off, n);

    return static_cast<ssize_t>(n);
}

// ---------------------------------------------------------------------------
// InfoOps -- the NodeOps subclass that backs the info file
// ---------------------------------------------------------------------------

/**
 * @brief NodeOps implementation for the read-only @c info file.
 *
 * Holds an immutable @c struct @c slash_info built at attach time.  The size
 * hook reports @c sizeof(struct slash_info) for @c getattr; the read hook
 * delegates to @ref infoPreadBuf.
 */
class InfoOps : public NodeOps {
public:
    explicit InfoOps(struct slash_info info) : info_(info) {}

    /**
     * @brief Report the file size (exactly @c sizeof(struct slash_info)).
     */
    off_t size(const Node & /*node*/) const override
    {
        return static_cast<off_t>(sizeof(struct slash_info));
    }

    /**
     * @brief Serve a positioned read out of the immutable backing struct.
     */
    ssize_t read(const Node & /*node*/, char *buf, size_t sz,
                 off_t off) override
    {
        return infoPreadBuf(buf, sz, off, &info_, sizeof(info_));
    }

private:
    struct slash_info info_;
};

// ---------------------------------------------------------------------------
// infoAttach
// ---------------------------------------------------------------------------

/**
 * @brief Attach the read-only @c info file to a device's @c <BDF>/ directory.
 *
 * Populates a @c struct @c slash_info backing (bdf, acc_type =
 * @c SLASH_ACC_TYPE_SYSTEM_EMULATED, size = the daemon's @c sizeof) and creates
 * an @c info FILE node under @c dev.dir whose ops report that size for
 * @c getattr and serve the struct's bytes for @c pread.
 *
 * @param dev The device to attach to (its @c dir must exist).
 * @return 0 on success, -1 on error (node creation failure).
 */
int infoAttach(Device &dev)
{
    if (dev.dir == nullptr) {
        return -1;
    }

    struct slash_info info{};
    info.size = static_cast<uint32_t>(sizeof(struct slash_info));
    info.acc_type = SLASH_ACC_TYPE_SYSTEM_EMULATED;

    /*
     * dev.bdf() is the normalized board-level BDF "DDDD:BB:DD" (no function),
     * which is exactly what slash_info.bdf advertises.  Copy it bounded; the
     * destination is SLASH_PCI_BDF_LEN and the device's bdf string is smaller,
     * so this never truncates, but stay defensive.
     */
    int n = std::snprintf(info.bdf, sizeof(info.bdf), "%s",
                          dev.bdf().c_str());
    if (n < 0 || static_cast<size_t>(n) >= sizeof(info.bdf)) {
        syslog(LOG_ERR, "BDF '%s' too long for info struct",
               dev.bdf().c_str());
        return -1;
    }

    /*
     * Hand the backing to the node via unique_ptr<InfoOps>; the node owns it
     * and frees it via the NodeOps destructor.  0444: read-only, the ABI's
     * info file is never written.
     */
    Node *node = dev.tree().createChild(dev.dir, "info", NodeType::File,
                                        0444,
                                        std::make_unique<InfoOps>(info));
    if (node == nullptr) {
        syslog(LOG_ERR, "Failed to create info node for '%s'",
               dev.bdf().c_str());
        return -1;
    }

    return 0;
}

} // namespace slash::emu
