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
 * @file bars.h
 * @brief The @c /<BDF>/bars/ endpoint: register-access files for PF2 BARs.
 *
 * The second of the four endpoints (info / bars / qdma / hotplug); it mirrors
 * the @ref info.h attach pattern (an @c emu_bars_attach() that creates file
 * nodes under the device subtree with an @ref emu_node_ops vtable) but is a
 * read/write endpoint, so it also exercises the spine's @c write hook
 * (@ref emu_node_pwrite) added alongside it.
 *
 * @section geometry Geometry
 *
 * PF2 exposes exactly three BARs (see @c slash_abi.h): BAR 0 (user region,
 * 128 MiB), BAR 2 (service layer, 128 MiB), BAR 4 (clock wizard, 512 KiB).
 * BARs 1, 3, 5 do not exist.  @ref emu_bars_attach creates @c bars/bar0,
 * @c bars/bar2, @c bars/bar4 -- and only those -- under @c dev->bars.  The file
 * size reported by @c getattr equals the BAR size; the start-address attribute
 * the old ABI carried is intentionally dropped (not reported).
 *
 * @section access Access policy (register access, not buffered I/O)
 *
 * A BAR file is for @em register access only: @c pread / @c pwrite, no @c mmap,
 * no buffering.  Each access is a single fixed-width register transfer and is
 * validated by @ref emu_bar_check_access:
 *
 *   - the transfer width (@c size) must be one of @c {1, 2, 4, 8};
 *   - the offset must be naturally aligned (a multiple of the width);
 *   - the whole access @c [off, off + size) must lie within the BAR.
 *
 * Anything else is rejected with @c -EINVAL.  Note the deliberate choice for an
 * out-of-range access: rather than the short-read / clamped-write a byte-stream
 * file would do, a register transfer that does not fit wholly inside the BAR is
 * an @em invalid transfer (it would violate @c width @c == @c size), so it is
 * rejected, not truncated.  This is the "rejected ... per pread/pwrite
 * semantics" arm of the spec, chosen because a partial register poke is
 * meaningless.  @c mmap is simply not offered: the FUSE op set has no @c mmap
 * handler, so the kernel returns @c -ENODEV to an @c mmap attempt.
 *
 * @section bridge SIM register-bridge seam (T10)
 *
 * The data plane is in-memory today: each BAR is backed by a lazily-allocated
 * zero-initialised byte buffer (a register shadow).  A read of a never-written
 * location returns 0; a write stores the bytes; a subsequent read returns them.
 * An in-range read NEVER fails with @c -EIO -- only revocation/transport errors
 * use @c -ENODEV (handled by the spine's liveness gate).
 *
 * T10 will route register access to @c vpp_sim's address-keyed dialect
 * (@c reg{addr,val} for a write, @c fetch @c scalar{addr} for a read; see
 * @c docs/bridge-design.md).  The seam for that is @ref emu_bar_backend: an
 * optional vtable on each BAR's backing.  When @c backend is NULL (today) the
 * in-memory shadow is used.  When T10 attaches a backend, the BAR's pread/pwrite
 * hooks call @c backend->read / @c backend->write with the already-validated
 * @c (bar_index, byte_offset, width) tuple -- exactly the
 * @c (bar_index, byte_offset, width, value) the bridge consumes -- and fall
 * back to the shadow for anything the model cannot answer, so an in-range read
 * still yields defined bytes and never @c -EIO.  Wiring the backend is the only
 * change T10 needs here; the validation, geometry, and liveness gating are
 * already in place.
 */

#ifndef SLASH_EMU_BARS_H
#define SLASH_EMU_BARS_H

#include <stddef.h>
#include <stdint.h>
#include <sys/types.h>

#include "node.h"

/**
 * @brief SIM register-bridge backend vtable (the T10 extension point).
 *
 * NULL on every BAR today (the in-memory shadow is used).  T10 attaches one per
 * device to forward validated register transfers to @c vpp_sim.  Both hooks are
 * invoked with the tree lock @em held, after @ref emu_bar_check_access has
 * accepted the transfer, so @c off + @c width is guaranteed in-range and
 * @c width is one of @c {1,2,4,8}.
 */
struct emu_bar_backend {
    /**
     * @brief Forward a validated register read to the model (@c fetch scalar).
     * @param ctx        Backend context (the bridge/session handle).
     * @param bar_index  PF2 BAR index (0, 2, or 4).
     * @param off        Validated, aligned, in-range byte offset.
     * @param width      Transfer width (1/2/4/8).
     * @param[out] value Receives the read value (little-endian register word).
     * @return 0 if the model answered (then @p value is used); a positive value
     *         to signal "fall back to the shadow" (an in-range read the model
     *         cannot answer must still return defined bytes, never -EIO); or a
     *         negative errno for a transport failure (mapped to -ENODEV).
     */
    int (*read)(void *ctx, uint32_t bar_index, off_t off, size_t width,
                uint64_t *value);

    /**
     * @brief Forward a validated register write to the model (@c reg{addr,val}).
     * @param ctx        Backend context (the bridge/session handle).
     * @param bar_index  PF2 BAR index (0, 2, or 4).
     * @param off        Validated, aligned, in-range byte offset.
     * @param width      Transfer width (1/2/4/8).
     * @param value      The value to write (little-endian register word).
     * @return 0 on success (the shadow is updated regardless); a negative errno
     *         for a transport failure (mapped to -ENODEV).
     */
    int (*write)(void *ctx, uint32_t bar_index, off_t off, size_t width,
                 uint64_t value);

    /** @brief Opaque context passed back to both hooks. */
    void *ctx;
};

/**
 * @brief Attach the @c bars/bar0, @c bars/bar2, @c bars/bar4 files for a device.
 *
 * Creates one @c EMU_NODE_FILE per existing PF2 BAR (0, 2, 4) under
 * @c dev->bars, each with an in-memory register-shadow backing sized to the BAR
 * and an ops vtable implementing @c getattr size, register @c pread / @c pwrite
 * (validated per @ref emu_bar_check_access), liveness gating (via the spine),
 * and @c destroy cleanup.  No SIM backend is attached (that is T10).
 *
 * @param dev The device to attach the endpoint to (its @c bars dir must exist).
 * @return 0 on success, -1 on error (allocation / node creation failure).
 */
int emu_bars_attach(struct emu_device *dev);

/**
 * @brief Attach (or detach) the SIM register-bridge backend for a device's BARs.
 *
 * The T10 wiring point and the seam the conformance suite uses to pin the bar
 * backend rc-contract.  Sets @p backend on every BAR file (@c bar0/bar2/bar4) of
 * the device, so a subsequent register @c pread/pwrite forwards to the model
 * (with the shadow as the defined-bytes fallback).  @p backend is borrowed
 * (non-owning, must outlive the device) and may be NULL to detach.  The device's
 * @c bars endpoint must already be attached (@ref emu_bars_attach).  Not
 * internally locked: call at attach time or with the tree lock held; the bridge
 * attaches it right after a successful reconfiguration.
 *
 * @param dev     The device whose BAR files get the backend.
 * @param backend The backend vtable (borrowed), or NULL to clear it.
 * @return 0 on success, -1 if the device has no attached @c bars endpoint.
 */
int emu_bars_set_backend(struct emu_device *dev,
                         const struct emu_bar_backend *backend);

/**
 * @brief Validate a single BAR register transfer (the access policy kernel).
 *
 * Exposed (rather than file-static) so the validation matrix can be unit-tested
 * without standing up a FUSE mount.  Accepts a transfer iff @p width is one of
 * @c {1,2,4,8}, @p off is a multiple of @p width, and @c [off, off + width)
 * lies wholly within @p bar_size.  Applies to both reads and writes -- a
 * register transfer that does not fit is invalid, not clamped.
 *
 * @param off      Byte offset of the transfer (may be negative -> rejected).
 * @param width    Transfer width in bytes.
 * @param bar_size Size of the BAR in bytes.
 * @return 0 if the transfer is valid; @c -EINVAL otherwise.
 */
int emu_bar_check_access(off_t off, size_t width, uint64_t bar_size);

#endif // SLASH_EMU_BARS_H
