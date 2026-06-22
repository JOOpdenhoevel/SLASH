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
 * @file qdma.hpp
 * @brief The @c /<BDF>/qdma/ endpoint: the QPAIR_ADD ioctl and the nameless,
 *        reference-counted @c qpair<Q> memory-transfer files.
 *
 * The endpoint that exercises the spine's resource / refcount / revocation
 * machinery the most.  Follows the @ref bars.hpp / @ref info.hpp attach pattern
 * but adds a directory-level @em ioctl (QPAIR_ADD) and a refcounted @ref Resource
 * per qpair whose lifetime is decoupled from its inode.
 *
 * @section qpair_add QPAIR_ADD (on the qdma/ directory)
 *
 * @ref qdmaAttach wires an ioctl handler onto the @c qdma/ directory node.  A
 * @c SLASH_ABI_QDMA_IOCTL_QPAIR_ADD validates mode (MM only) / dir_mask /
 * ring-size indices, allocates a unique per-device QID, creates the
 * @c qdma/qpair<Q> file with a refcounted resource backing, marks it direct-I/O,
 * and writes the QID back.  The caller then opens it and (per the VRTD pattern)
 * unlinks it while open, so the qpair is nameless for its whole life and only
 * reachable via the registry.
 *
 * @section qpair The qpair<Q> data files (pread/pwrite only)
 *
 * A @c qpair<Q> moves bytes to/from device memory by device address (the file
 * offset @em is the device address).  Range-validated (one HBM bank-range or the
 * DDR range; the reconfig region is the bridge's concern): out-of-range /
 * boundary-straddling / overflowing accesses are @c -ERANGE; an in-range read of
 * never-written memory returns defined zero bytes, never @c -EIO.  Streaming (ST)
 * is deferred: MM only.
 *
 * @section lifetime Qpair lifetime
 *
 * Each qpair is a @ref Resource (registry ref + inode ref).  Two idempotent
 * teardown triggers stop the queue and free the QID: cooperative (last close of
 * an unlinked qpair) and forced (device removal).  The first runs teardown; the
 * second is a no-op.  After a forced teardown every op on a still-open fd returns
 * @c -ENODEV.
 *
 * @section store Per-device sparse memory store + the bridge seam
 *
 * Device memory (32 GiB HBM + 32 GiB DDR) is shared per device, modelled as a
 * sparse paged map (lazily allocated, zero on read-before-write).  The SIM bridge
 * attaches a @ref QdmaMemBackend to forward validated MM transfers to @c vpp_sim,
 * with the store as the defined-bytes fallback.
 */

#ifndef SLASH_EMU_QDMA_HPP
#define SLASH_EMU_QDMA_HPP

#include <cstddef>
#include <cstdint>
#include <functional>
#include <sys/types.h>

#include "node.hpp"

namespace slash::emu {

/**
 * @brief SIM memory-bridge backend interface (the bridge extension point).
 *
 * Null on every device today (the in-memory sparse store is used).  Both methods
 * are invoked with the tree lock @em held, after the qpair range validation has
 * accepted the transfer (so @c [addr, addr+len) lies within one valid window).
 */
class QdmaMemBackend {
public:
    virtual ~QdmaMemBackend() = default;

    /**
     * @brief Forward a validated device-memory read (@c fetch buffer{addr,size}).
     * @return 0 if the model answered (then @p buf is filled); a positive value to
     *         fall back to the sparse store (defined bytes, never -EIO); a negative
     *         errno for a transport failure (mapped to -ENODEV).
     */
    virtual int fetch(uint64_t addr, void *buf, size_t len) = 0;

    /**
     * @brief Forward a validated device-memory write (@c populate{addr,size}+data).
     * @return 0 on success; a negative errno for a transport failure (-> -ENODEV).
     */
    virtual int populate(uint64_t addr, const void *buf, size_t len) = 0;
};

/**
 * @brief Reconfiguration handler: a (chunk of a) VBIN write landed in the region.
 *
 * Invoked from the qpair write path with the tree lock @em held when a write's
 * @c [addr, addr+len) lies wholly within the reconfiguration region.  The kernel
 * splits a large write into several chunks; the handler reassembles them (a write
 * at @c SLASH_RECONFIG_BASE starts/replaces; at @c BASE+accumulated appends).
 *
 * @return 0 on success (chunk accepted, whether it completed the VBIN or awaits
 *         more); a negative errno on a malformed VBIN / non-contiguous chunk /
 *         over-cap stream / spawn failure.
 */
using QdmaReconfigFn =
    std::function<int(uint64_t addr, const void *vbin, size_t len)>;

/**
 * @brief Attach the @c qdma/ QPAIR_ADD ioctl handler for a device.
 *
 * Wires an ioctl handler onto @c dev.qdma so a @c SLASH_ABI_QDMA_IOCTL_QPAIR_ADD
 * allocates a QID, creates the @c qdma/qpair<Q> file with a refcounted resource
 * backing, and returns the QID.  No SIM backend is attached (the bridge does that).
 *
 * @param dev The device to attach to (its @c qdma dir must exist).
 * @return 0 on success, -1 on error (attach failure).
 */
int qdmaAttach(Device &dev);

/**
 * @brief Attach (or detach) the SIM memory-bridge backend for a device's store.
 *
 * @p backend is borrowed (must outlive the device) and may be nullptr to detach.
 * The @c qdma/ endpoint must already be attached.  Not internally locked: call at
 * attach time or with the tree lock held.
 *
 * @return 0 on success, -1 if the device has no attached @c qdma endpoint.
 */
int qdmaSetMemBackend(Device &dev, QdmaMemBackend *backend);

/**
 * @brief Attach (or detach) the reconfiguration handler for a device's store.
 *
 * The @c qdma/ endpoint must already be attached.  Pass an empty @p handler to
 * detach.  Not internally locked: call at attach time or with the tree lock held.
 *
 * @return 0 on success, -1 if the device has no attached @c qdma endpoint.
 */
int qdmaSetReconfigHandler(Device &dev, QdmaReconfigFn handler);

/* ---- pure helpers exposed for unit testing (no FUSE mount) ---- */

/**
 * @brief Validate a single MM transfer's device-address range.
 *
 * Accepts iff @c [addr, addr+len) lies wholly within one valid memory window (an
 * HBM bank-range or the DDR range) and does not overflow.  A zero-length transfer
 * is accepted.  The reconfiguration region is intentionally not accepted here.
 *
 * @return 0 if in range; @c -ERANGE otherwise.
 */
int qdmaCheckRange(uint64_t addr, size_t len);

/**
 * @brief Validate the QPAIR_ADD parameters (mode / dir_mask / ring sizes).
 *
 * Accepts iff @p mode is MM (0), @p dir_mask selects only H2C/C2H bits (and at
 * least one), and each ring size is a CSR index in 0..15.
 *
 * @return 0 if accepted; @c -EOPNOTSUPP for ST mode / CMPT direction; @c -EINVAL
 *         for a bad dir_mask or out-of-range ring index.
 */
int qdmaCheckQpairAdd(uint32_t mode, uint32_t dir_mask, uint32_t h2c_ring,
                      uint32_t c2h_ring, uint32_t cmpt_ring);

/**
 * @brief Test whether a write is a reconfiguration (whole-VBIN) delivery.
 *
 * Returns true iff @c [addr, addr+len) lies wholly within the reconfiguration
 * region and @p len is non-zero and does not overflow.  Checked before the
 * ordinary range check (which keeps rejecting the region with @c -ERANGE).
 */
bool qdmaIsReconfigWrite(uint64_t addr, size_t len);

} // namespace slash::emu

#endif // SLASH_EMU_QDMA_HPP
