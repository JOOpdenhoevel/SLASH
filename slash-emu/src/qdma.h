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
 * @file qdma.h
 * @brief The @c /<BDF>/qdma/ endpoint: the QPAIR_ADD ioctl and the nameless,
 *        reference-counted @c qpair<Q> memory-transfer files.
 *
 * The third of the four endpoints (info / bars / qdma / hotplug) and the one
 * that exercises the spine's resource / refcount / revocation machinery the
 * most.  It follows the @ref bars.h / @ref info.h attach pattern -- an
 * @ref emu_qdma_attach() that wires behaviour onto the device subtree via an
 * @ref emu_node_ops vtable + an opaque backing whose ownership transfers to the
 * node -- but adds two things bars does not: a directory-level @em ioctl
 * (QPAIR_ADD) and a @em refcounted resource per qpair whose lifetime is
 * decoupled from its inode.
 *
 * @section qpair_add The QPAIR_ADD ioctl (on the qdma/ directory)
 *
 * @c emu_qdma_attach attaches an ioctl vtable to the @c qdma/ @em directory
 * node (created earlier by @ref emu_node_tree_add_device).  A
 * @c SLASH_ABI_QDMA_IOCTL_QPAIR_ADD on the directory fd:
 *
 *   - validates @c mode (only MM == 0 is supported; ST == 1 -> @c -EOPNOTSUPP),
 *     @c dir_mask (H2C/C2H bits; the CMPT bit -> @c -EOPNOTSUPP), and the four
 *     ring-size CSR indices (0..15);
 *   - allocates a unique per-device QID (a monotonic counter -- the architecture
 *     requires only uniqueness, not a particular id; ids are not reused below
 *     the counter except after a full u32 wrap, which never happens in practice);
 *   - "starts the queue" (a no-op in the in-memory model -- the queue is a
 *     lifecycle token, the memory store is shared per-device);
 *   - creates the @c qdma/qpair<Q> file node, registers a refcounted resource
 *     for it in the per-device registry, attaches that resource to the node,
 *     and marks the file for direct I/O (memory transfers are unbuffered);
 *   - writes the QID back into the caller's struct.
 *
 * The caller then opens @c qdma/qpair<Q> and (per the VRTD pattern) unlinks it
 * while the fd is open, so the qpair is nameless for its whole life and is only
 * reachable through the registry -- which is exactly how forced removal still
 * tears it down.
 *
 * @section qpair The qpair<Q> data files (pread/pwrite only)
 *
 * A @c qpair<Q> file moves bytes to/from the accelerator's device memory by
 * @em device address: the @c pread/pwrite file offset @em is the device address
 * (HBM/DDR), mirroring how the kernel QDMA path lseeks to the buffer's phys
 * addr.  There is no register-width rule here (unlike a BAR): a transfer is a
 * byte range @c [addr, addr+len).  Validation is range-based:
 *
 *   - the whole @c [addr, addr+len) must lie within a single valid memory
 *     window (one HBM bank-range or the DDR range; the reconfiguration region
 *     is T10's concern and is not accepted here);
 *   - an access wholly outside the valid windows, straddling a window boundary,
 *     or overflowing is rejected with @c -ERANGE (the errno the UAPI names for
 *     out-of-range memory accesses);
 *   - an in-range @c pread of never-written memory returns defined zero bytes,
 *     never @c -EIO (mirroring the bars G7 invariant).
 *
 * Streaming (ST mode, @c stream_in / @c stream_out) is deferred per the step-1
 * decision: MM transfers only.
 *
 * @section lifetime Qpair lifetime (the refcounted resource)
 *
 * Each qpair is modelled as an @ref emu_resource (see node.h): two references,
 * one held by the registry and one by the inode, freed only when @em both drop.
 * Two idempotent teardown triggers stop the queue and free the QID:
 *
 *   - @b cooperative: last close of an undisturbed/unlinked qpair (inode
 *     eviction) -> @ref emu_node_forget -> node destroy -> drops the inode ref;
 *   - @b forced: device removal (@ref emu_device_revoke) -> tears down every
 *     registered resource eagerly and marks it dead.
 *
 * The first trigger runs the teardown; the second is a no-op.  After a forced
 * teardown every @c pread / @c pwrite on a still-open fd returns @c -ENODEV
 * (enforced by the spine's liveness gate in @ref emu_node_pread /
 * @ref emu_node_pwrite, made robust by the qpair backing also consulting
 * @ref emu_resource_check).
 *
 * @section pruning Pruning leftovers (named-but-not-unlinked qpairs)
 *
 * A qpair the daemon created but the holder has not yet unlinked stays visible
 * in @c readdir(qdma/) and is removable by @c unlink -- this is how VRTD prunes
 * qpairs left over from a previous crash on its next startup.  The daemon itself
 * holds qpair state in-memory only, so a daemon @em restart has no leftover
 * qpairs at all (the FUSE analogue of the spec's "prune on startup"); the
 * prune-by-unlink path exists for the VRTD-crash-between-ADD-and-unlink case.
 *
 * @section store Per-device sparse memory store + the T10 bridge seam
 *
 * Device memory (32 GiB HBM + 32 GiB DDR) is shared by all qpairs of a device,
 * so the store is @em per device, not per qpair.  It is far too large to
 * allocate, so it is a @em sparse, paged map: a flat dynamic array of
 * fixed-size pages keyed by aligned device address, allocated lazily on first
 * write, zero on read-before-write.  This is the in-memory data plane today.
 *
 * The T10 SIM bridge attaches via @ref emu_qdma_mem_backend -- the qpair
 * read/write hooks call @c backend->fetch / @c backend->populate with the
 * validated @c (device_addr, len) tuple (the exact @c populate{addr,size} /
 * @c fetch buffer{addr,size} the vpp_sim dialect consumes; see
 * @c docs/bridge-design.md) and fall back to the sparse store for anything the
 * model cannot answer, so an in-range read still yields defined bytes and never
 * @c -EIO.  Wiring the backend is the only change T10 needs here.
 */

#ifndef SLASH_EMU_QDMA_H
#define SLASH_EMU_QDMA_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <sys/types.h>

#include "node.h"

/**
 * @brief SIM memory-bridge backend vtable (the T10 extension point).
 *
 * NULL on every device today (the in-memory sparse store is used).  T10
 * attaches one per device to forward validated MM transfers to @c vpp_sim.
 * Both hooks are invoked with the tree lock @em held, after the qpair
 * address-range validation has accepted the transfer, so @c [addr, addr+len)
 * is guaranteed to lie within a single valid memory window.
 */
struct emu_qdma_mem_backend {
    /**
     * @brief Forward a validated device-memory read to the model
     *        (@c fetch buffer{addr,size}).
     * @param ctx   Backend context (the bridge/session handle).
     * @param addr  Validated, in-range device address.
     * @param buf   Destination for @p len bytes.
     * @param len   Number of bytes to fetch.
     * @return 0 if the model answered (then @p buf is filled); a positive value
     *         to signal "fall back to the sparse store" (an in-range read the
     *         model cannot answer must still return defined bytes, never -EIO);
     *         or a negative errno for a transport failure (mapped to -ENODEV).
     */
    int (*fetch)(void *ctx, uint64_t addr, void *buf, size_t len);

    /**
     * @brief Forward a validated device-memory write to the model
     *        (@c populate{addr,size}+payload).
     * @param ctx   Backend context (the bridge/session handle).
     * @param addr  Validated, in-range device address.
     * @param buf   Source of @p len bytes.
     * @param len   Number of bytes to populate.
     * @return 0 on success; a negative errno for a transport failure (mapped to
     *         -ENODEV).
     */
    int (*populate)(void *ctx, uint64_t addr, const void *buf, size_t len);

    /** @brief Opaque context passed back to both hooks. */
    void *ctx;
};

/**
 * @brief Attach the @c qdma/ QPAIR_ADD ioctl handler for a device.
 *
 * Wires an ioctl vtable onto @c dev->qdma (the @c qdma/ directory node) so a
 * @c SLASH_ABI_QDMA_IOCTL_QPAIR_ADD on the directory fd allocates a QID, creates
 * the @c qdma/qpair<Q> file with a refcounted resource backing, and returns the
 * QID.  No SIM backend is attached (that is T10).
 *
 * @param dev The device to attach the endpoint to (its @c qdma dir must exist).
 * @return 0 on success, -1 on error (allocation / attach failure).
 */
int emu_qdma_attach(struct emu_device *dev);

/**
 * @brief Attach (or detach) the SIM memory-bridge backend for a device's store.
 *
 * The T10 wiring point and the seam the conformance suite uses to pin the
 * backend rc-contract (fetch rc==0 model-answered / rc>0 fall back to the store
 * / rc<0 transport failure -> -ENODEV; populate keeps the store current).  The
 * device's @c qdma/ endpoint must already be attached (@ref emu_qdma_attach).
 * @p backend is borrowed (non-owning, must outlive the device) and may be NULL
 * to detach.  Not locked: call it at attach time (single-threaded setup) or with
 * the tree lock held; the daemon attaches it right after @ref emu_qdma_attach.
 *
 * @param dev     The device whose @c qdma store gets the backend.
 * @param backend The backend vtable (borrowed), or NULL to clear it.
 * @return 0 on success, -1 if the device has no attached @c qdma endpoint.
 */
int emu_qdma_set_mem_backend(struct emu_device *dev,
                             const struct emu_qdma_mem_backend *backend);

/**
 * @brief Reconfiguration handler: a VBIN-delivery write landed in the reconfig region.
 *
 * The T10 reconfiguration seam.  A @c pwrite through a @c qpair<Q> file whose
 * @c [addr, addr+len) lies wholly within the reconfiguration region
 * (@c SLASH_RECONFIG_BASE .. @c SLASH_RECONFIG_END) is @em not an ordinary memory
 * transfer -- it is (part of) the delivery of a VBIN (architecture
 * §Reconfiguration).  The qpair write hook detects that case @em before the
 * normal HBM/DDR range check (which still rejects the region with @c -ERANGE) and
 * routes the bytes here.  Invoked with the tree lock @em held.
 *
 * The kernel splits a write larger than its @c max_write into several
 * @c ops->write calls, so a multi-MB VBIN arrives as several in-region chunks.
 * The handler therefore receives the chunk's device address @p addr and
 * reassembles contiguous chunks itself: a write at @c SLASH_RECONFIG_BASE starts
 * (or replaces) a transfer; a write at the running @c BASE+accumulated offset
 * appends; completion is detected from the archive structure (see the bridge).
 *
 * @param ctx  The handler context (the device's @ref emu_bridge).
 * @param addr The chunk's device address (the qpair write offset).
 * @param vbin The chunk bytes.
 * @param len  Chunk length in bytes.
 * @return 0 on success (chunk accepted, whether it completed the VBIN or awaits
 *         more); a negative errno on a malformed VBIN, a non-contiguous chunk, an
 *         over-cap stream, or a spawn failure.
 */
typedef int (*emu_qdma_reconfig_fn)(void *ctx, uint64_t addr, const void *vbin,
                                    size_t len);

/**
 * @brief Attach (or detach) the reconfiguration handler for a device's store.
 *
 * The T10 wiring point for the reconfig-region write path.  The device's
 * @c qdma/ endpoint must already be attached (@ref emu_qdma_attach).  @p ctx is
 * borrowed (must outlive the device or be cleared first).  Pass @p handler NULL
 * to detach.  Not internally locked: call at attach time or with the tree lock
 * held.
 *
 * @param dev     The device whose @c qdma store gets the handler.
 * @param handler The reconfig callback, or NULL to clear it.
 * @param ctx     Opaque context passed back to @p handler (borrowed).
 * @return 0 on success, -1 if the device has no attached @c qdma endpoint.
 */
int emu_qdma_set_reconfig_handler(struct emu_device *dev,
                                  emu_qdma_reconfig_fn handler, void *ctx);

/* ------------------------------------------------------------------ */
/* Pure helpers exposed for unit testing (no FUSE mount required)     */
/* ------------------------------------------------------------------ */

/**
 * @brief Validate a single MM transfer's device-address range.
 *
 * Exposed (rather than file-static) so the range-policy matrix can be
 * unit-tested without standing up a FUSE mount.  Accepts a transfer iff
 * @c [addr, addr+len) lies wholly within a single valid memory window -- one
 * HBM bank-range (@c SLASH_HBM_BASE..END) or the DDR range
 * (@c SLASH_DDR_BASE..END) -- and does not overflow.  A zero-length transfer is
 * accepted (it copies nothing).  The reconfiguration region is intentionally
 * @em not accepted here (it is T10's VBIN path).
 *
 * @param addr Device address of the transfer.
 * @param len  Transfer length in bytes.
 * @return 0 if the transfer is in range; @c -ERANGE otherwise.
 */
int emu_qdma_check_range(uint64_t addr, size_t len);

/**
 * @brief Validate the QPAIR_ADD parameters (mode / dir_mask / ring sizes).
 *
 * Exposed for unit testing the acceptance matrix.  Accepts iff @p mode is MM
 * (0), @p dir_mask selects only H2C/C2H bits (and at least one), and each ring
 * size is a CSR table index in 0..15.
 *
 * @param mode      Queue operating mode (0 = MM, 1 = ST).
 * @param dir_mask  Direction bitmask (bit0 H2C, bit1 C2H, bit2 CMPT).
 * @param h2c_ring  H2C ring-size CSR index.
 * @param c2h_ring  C2H ring-size CSR index.
 * @param cmpt_ring CMPT ring-size CSR index.
 * @return 0 if accepted; @c -EOPNOTSUPP for ST mode / CMPT direction;
 *         @c -EINVAL for a bad dir_mask or out-of-range ring index.
 */
int emu_qdma_check_qpair_add(uint32_t mode, uint32_t dir_mask,
                             uint32_t h2c_ring, uint32_t c2h_ring,
                             uint32_t cmpt_ring);

/**
 * @brief Test whether a write is a reconfiguration (whole-VBIN) delivery.
 *
 * Exposed for unit testing the region-detection.  Returns 1 iff
 * @c [addr, addr+len) lies wholly within the reconfiguration region
 * (@c SLASH_RECONFIG_BASE .. @c SLASH_RECONFIG_END) and @p len is non-zero and
 * does not overflow.  A zero-length write carries no VBIN and is not a
 * reconfiguration.  This is deliberately a separate predicate from
 * @ref emu_qdma_check_range (which keeps rejecting the region with @c -ERANGE for
 * ordinary transfers): the reconfig path is write-only and is checked first.
 *
 * @param addr Device address of the write.
 * @param len  Write length in bytes.
 * @return 1 if the write is a reconfig-region VBIN delivery; 0 otherwise.
 */
int emu_qdma_is_reconfig_write(uint64_t addr, size_t len);

#endif // SLASH_EMU_QDMA_H
