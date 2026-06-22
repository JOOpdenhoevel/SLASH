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
 * @file bars.hpp
 * @brief The @c /<BDF>/bars/ endpoint: register-access files for PF2 BARs.
 *
 * Mirrors the @ref info.hpp attach pattern but is read/write.  PF2 exposes exactly
 * three BARs (see @c slash_abi.h): BAR 0 (user, 128 MiB), BAR 2 (service layer,
 * 128 MiB), BAR 4 (clock wizard, 512 KiB).  @ref barsAttach creates @c bars/bar0,
 * @c bars/bar2, @c bars/bar4 and only those.  The reported file size equals the
 * BAR size; the old start-address attribute is dropped.
 *
 * @section access Access policy (register access, not buffered I/O)
 *
 * A BAR file is for register access only: @c pread / @c pwrite, no @c mmap, no
 * buffering.  Each access is one fixed-width transfer validated by
 * @ref barCheckAccess: width in @c {1,2,4,8}, naturally aligned, wholly within
 * the BAR; anything else is @c -EINVAL (an out-of-range register transfer is
 * rejected, not truncated).  @c mmap is not offered (the low-level FUSE op set
 * has no @c .mmap handler).
 *
 * @section bridge SIM register-bridge seam
 *
 * The data plane is an in-memory, lazily-allocated zero-initialised register
 * shadow today (an in-range read of an unwritten location returns 0, never
 * @c -EIO).  The SIM bridge attaches a @ref BarBackend per device to forward
 * validated transfers to @c vpp_sim, with the shadow as the defined-bytes
 * fallback.  Wiring the backend is the only bridge-side change here.
 */

#ifndef SLASH_EMU_BARS_HPP
#define SLASH_EMU_BARS_HPP

#include <cstddef>
#include <cstdint>
#include <sys/types.h>

#include "node.hpp"

namespace slash::emu {

/**
 * @brief SIM register-bridge backend interface (the bridge extension point).
 *
 * Null on every BAR today (the in-memory shadow is used).  The bridge attaches
 * one per device to forward validated register transfers to @c vpp_sim.  Both
 * methods are invoked with the tree lock @em held, after @ref barCheckAccess has
 * accepted the transfer (so @c off + @c width is in-range and @c width is one of
 * @c {1,2,4,8}).
 */
class BarBackend {
public:
    virtual ~BarBackend() = default;

    /**
     * @brief Forward a validated register read to the model (@c fetch scalar).
     * @param[out] value Receives the read value (little-endian register word).
     * @return 0 if the model answered (then @p value is used); a positive value
     *         to fall back to the shadow (an in-range read the model cannot answer
     *         must still return defined bytes, never -EIO); a negative errno for a
     *         transport failure (mapped to -ENODEV).
     */
    virtual int read(uint32_t bar_index, off_t off, size_t width,
                     uint64_t &value) = 0;

    /**
     * @brief Forward a validated register write to the model (@c reg{addr,val}).
     * @return 0 on success (the shadow is updated regardless); a negative errno
     *         for a transport failure (mapped to -ENODEV).
     */
    virtual int write(uint32_t bar_index, off_t off, size_t width,
                      uint64_t value) = 0;
};

/**
 * @brief Attach the @c bars/bar0, @c bars/bar2, @c bars/bar4 files for a device.
 *
 * Creates one FILE per existing PF2 BAR under @c dev.bars, each with an in-memory
 * register-shadow backing sized to the BAR and ops implementing @c getattr size,
 * validated register @c pread / @c pwrite, liveness gating, and cleanup.  No SIM
 * backend is attached (the bridge does that).
 *
 * @param dev The device to attach to (its @c bars dir must exist).
 * @return 0 on success, -1 on error (node creation failure).
 */
int barsAttach(Device &dev);

/**
 * @brief Attach (or detach) the SIM register-bridge backend for a device's BARs.
 *
 * Sets @p backend on every BAR file of the device; a subsequent register
 * @c pread/pwrite forwards to the model (with the shadow as fallback).
 * @p backend is borrowed (must outlive the device) and may be nullptr to detach.
 * The @c bars endpoint must already be attached.  Not internally locked: call at
 * attach time or with the tree lock held.
 *
 * @return 0 on success, -1 if the device has no attached @c bars endpoint.
 */
int barsSetBackend(Device &dev, BarBackend *backend);

/**
 * @brief Validate a single BAR register transfer (the access-policy kernel).
 *
 * Accepts iff @p width is one of @c {1,2,4,8}, @p off is a multiple of @p width,
 * and @c [off, off + width) lies wholly within @p bar_size.  Applies to reads and
 * writes alike.  Exposed for unit testing without a FUSE mount.
 *
 * @return 0 if valid; @c -EINVAL otherwise.
 */
int barCheckAccess(off_t off, size_t width, uint64_t bar_size);

} // namespace slash::emu

#endif // SLASH_EMU_BARS_HPP
