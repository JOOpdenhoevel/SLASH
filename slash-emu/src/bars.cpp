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
 * @file bars.cpp
 * @brief Implementation of the @c /<BDF>/bars/ endpoint (see bars.hpp).
 *
 * Per-BAR state lives inside @ref BarOps.  Geometry is fixed at attach time;
 * the register state is an in-memory shadow allocated lazily on first write
 * (so the 128 MiB user/service BARs cost nothing until actually poked).  A
 * read of an unallocated shadow returns zero bytes -- the defined "never
 * written" value.  The SIM bridge seam is the optional @ref BarBackend
 * pointer (nullptr today, meaning "use the shadow").
 *
 * The shadow is allocated via @c calloc(1, size) so the @c --wrap=calloc
 * interposer in bars_oom_test can intercept first-write failures
 * deterministically.  The destructor frees it with @c free().
 *
 * @c BarOps is defined in this translation unit so @ref barsSetBackend can
 * use @c dynamic_cast to find it on child nodes.
 */

#include "bars.hpp"

#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <syslog.h>

#include "slash/uapi/slash_abi.h"

namespace slash::emu {

// ---------------------------------------------------------------------------
// barCheckAccess -- the access-policy kernel (free function)
// ---------------------------------------------------------------------------

/**
 * @brief Validate a single BAR register transfer (the access-policy kernel).
 *
 * Accepts iff @p width is one of @c {1,2,4,8}, @p off is a multiple of
 * @p width, and @c [off, off + width) lies wholly within @p bar_size.
 * Applies to reads and writes alike.
 *
 * @return 0 if valid; @c -EINVAL otherwise.
 */
int barCheckAccess(off_t off, size_t width, uint64_t bar_size)
{
    /* Width must be a single register transfer of 1, 2, 4, or 8 bytes. */
    if (width != 1 && width != 2 && width != 4 && width != 8) {
        return -EINVAL;
    }

    /* Offset must be non-negative and naturally aligned to the width. */
    if (off < 0 || static_cast<uint64_t>(off) % width != 0) {
        return -EINVAL;
    }

    /*
     * The whole transfer must fit inside the BAR.  Compare in uint64_t and
     * guard the add against overflow (off is already >= 0 here).  A transfer
     * that straddles the end is invalid, not clamped: width == transfer size.
     */
    uint64_t end = static_cast<uint64_t>(off) + width;
    if (end < static_cast<uint64_t>(off) || end > bar_size) {
        return -EINVAL;
    }

    return 0;
}

// ---------------------------------------------------------------------------
// BarOps -- the NodeOps subclass that backs each bar<M> file
// ---------------------------------------------------------------------------

/**
 * @brief NodeOps implementation for a single BAR register-access file.
 *
 * Holds the BAR's geometry (index, size), the lazily-allocated register
 * shadow, and an optional pointer to a @ref BarBackend (the SIM bridge seam).
 * The shadow is a raw @c calloc buffer so the @c --wrap=calloc interposer in
 * bars_oom_test can inject failures deterministically; the destructor frees it
 * with @c free().
 */
class BarOps : public NodeOps {
public:
    BarOps(uint32_t index, uint64_t size)
        : index_(index), size_(size)
    {
    }

    ~BarOps() override
    {
        /* free(nullptr) is a no-op, so no guard needed. */
        free(shadow_);
    }

    BarOps(const BarOps &) = delete;
    BarOps &operator=(const BarOps &) = delete;

    /** @brief Report the file size (the BAR size in bytes). */
    off_t size(const Node & /*node*/) const override
    {
        return static_cast<off_t>(size_);
    }

    /**
     * @brief Serve a validated, fixed-width register read.
     *
     * Validates the transfer with @ref barCheckAccess; if a backend is
     * attached, asks the model first.  Falls back to the shadow (zero for an
     * unwritten location, or the last-written bytes); never returns @c -EIO
     * for an in-range access (G7).
     */
    ssize_t read(const Node & /*node*/, char *buf, size_t sz,
                 off_t off) override
    {
        int rc = barCheckAccess(off, sz, size_);
        if (rc != 0) {
            return rc;
        }

        /*
         * SIM bridge seam: if a backend is attached, ask the model for the
         * register value first.  rc == 0 => model answered (use `value`);
         * rc > 0 => "fall back to the shadow" (an in-range read the model
         * cannot answer must still return defined bytes, never -EIO);
         * rc < 0 => transport failure.
         */
        if (backend_ != nullptr) {
            uint64_t value = 0;
            rc = backend_->read(index_, off, sz, value);
            if (rc < 0) {
                return -ENODEV;
            }
            if (rc == 0) {
                /* Little-endian register word, low `sz` bytes. */
                for (size_t i = 0; i < sz; i++) {
                    buf[i] = static_cast<char>((value >> (8 * i)) & 0xFF);
                }
                return static_cast<ssize_t>(sz);
            }
            /* rc > 0: fall through to the shadow. */
        }

        if (shadow_ == nullptr) {
            /* Never written: defined zero value. */
            std::memset(buf, 0, sz);
        } else {
            std::memcpy(buf, shadow_ + off, sz);
        }

        return static_cast<ssize_t>(sz);
    }

    /**
     * @brief Store a validated, fixed-width register write.
     *
     * Validates the transfer; allocates the shadow on first write (calloc so
     * the whole shadow is zero-initialised).  Returns @c -ENOMEM if the
     * shadow allocation fails.  Forwards the validated poke to the model if a
     * backend is attached.
     */
    ssize_t write(const Node & /*node*/, const char *buf, size_t sz,
                  off_t off) override
    {
        int rc = barCheckAccess(off, sz, size_);
        if (rc != 0) {
            return rc;
        }

        /*
         * Update the in-memory shadow, allocating it on first write.  The
         * shadow is always kept current -- even when a SIM backend is attached
         * -- so an in-range read the model cannot answer can fall back to it
         * (G7: never surface -EIO for an in-range access).
         */
        if (shadow_ == nullptr) {
            shadow_ = static_cast<uint8_t *>(calloc(1, size_));
            if (shadow_ == nullptr) {
                /*
                 * This is a NodeOps::write hook: the return is fed straight to
                 * fuse_reply_err(req, -n), so it MUST be a negative errno.  Do
                 * NOT return -1 -- that would surface to the client as EPERM
                 * rather than ENOMEM.
                 */
                syslog(LOG_ERR, "Failed to allocate BAR%u shadow", index_);
                return -ENOMEM;
            }
        }
        std::memcpy(shadow_ + off, buf, sz);

        /* SIM bridge seam: forward the validated poke to the model. */
        if (backend_ != nullptr) {
            uint64_t value = 0;
            for (size_t i = 0; i < sz; i++) {
                value |= static_cast<uint64_t>(
                             static_cast<uint8_t>(buf[i])) << (8 * i);
            }
            if (backend_->write(index_, off, sz, value) < 0) {
                return -ENODEV;
            }
        }

        return static_cast<ssize_t>(sz);
    }

    /** @brief The BAR index (0, 2, or 4). */
    uint32_t index() const { return index_; }

    /** @brief The SIM bridge backend (may be nullptr). */
    BarBackend *backend_ = nullptr;

private:
    uint32_t index_;
    uint64_t size_;
    uint8_t *shadow_ = nullptr;
};

// ---------------------------------------------------------------------------
// The three BARs PF2 exposes.  Order matches creation order under bars/.
// ---------------------------------------------------------------------------

struct BarTableEntry {
    uint32_t index;
    uint64_t size;
    const char *name;
};

static constexpr BarTableEntry kBarTable[] = {
    { SLASH_BAR_USER_IDX, SLASH_BAR_USER_SIZE, "bar0" },
    { SLASH_BAR_SL_IDX,   SLASH_BAR_SL_SIZE,   "bar2" },
    { SLASH_BAR_CLK_IDX,  SLASH_BAR_CLK_SIZE,  "bar4" },
};

// ---------------------------------------------------------------------------
// barsAttach
// ---------------------------------------------------------------------------

/**
 * @brief Attach the @c bars/bar0, @c bars/bar2, @c bars/bar4 files for a device.
 *
 * Creates one FILE per existing PF2 BAR under @c dev.bars, each with an
 * in-memory register-shadow backing sized to the BAR and ops implementing
 * @c getattr size, validated register @c pread / @c pwrite, liveness gating,
 * and cleanup.  No SIM backend is attached (the bridge does that).
 *
 * @param dev The device to attach to (its @c bars dir must exist).
 * @return 0 on success, -1 on error (node creation failure).
 */
int barsAttach(Device &dev)
{
    if (dev.bars == nullptr) {
        return -1;
    }

    for (const auto &entry : kBarTable) {
        /*
         * 0600: a BAR file is read/write register access, owner-only.  Hand
         * the backing to the node via unique_ptr<BarOps>; the node owns it
         * and frees it (shadow included) via the BarOps destructor.
         */
        Node *node = dev.tree().createChild(
            dev.bars, entry.name, NodeType::File, 0600,
            std::make_unique<BarOps>(entry.index, entry.size));
        if (node == nullptr) {
            syslog(LOG_ERR, "Failed to create %s node for '%s'",
                   entry.name, dev.bdf().c_str());
            return -1;
        }

        /*
         * Register access is unbuffered: open the BAR file with direct_io so
         * the kernel forwards each pread/pwrite verbatim (exact size + offset)
         * and does not synthesize page-sized, page-cached transfers the width
         * validation would reject.
         */
        dev.tree().setDirectIo(node);
    }

    return 0;
}

// ---------------------------------------------------------------------------
// barsSetBackend
// ---------------------------------------------------------------------------

/**
 * @brief Attach (or detach) the SIM register-bridge backend for a device's BARs.
 *
 * Sets @p backend on every BAR file of the device; a subsequent register
 * @c pread/pwrite forwards to the model (with the shadow as fallback).
 * @p backend is borrowed (must outlive the device) and may be nullptr to
 * detach.  The @c bars endpoint must already be attached.
 *
 * Uses @c dynamic_cast to identify @c BarOps children (so only the children
 * this endpoint created are updated).
 *
 * @return 0 on success, -1 if the device has no attached @c bars endpoint.
 */
int barsSetBackend(Device &dev, BarBackend *backend)
{
    if (dev.bars == nullptr) {
        return -1;
    }

    bool found = false;
    for (Node *child : dev.bars->children) {
        auto *b = dynamic_cast<BarOps *>(child->ops.get());
        if (b != nullptr) {
            b->backend_ = backend;
            found = true;
        }
    }

    return found ? 0 : -1;
}

} // namespace slash::emu
