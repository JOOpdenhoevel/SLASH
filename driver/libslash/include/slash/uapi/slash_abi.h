/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
/**
 * Copyright (C) 2026 Advanced Micro Devices, Inc. All rights reserved.
 *
 * This file is dual-licensed: you may select either the GNU General Public
 * License version 2 (GPL-2.0-only) or the MIT License.  See the LICENSE
 * files in the repository root for the full text of each license.
 */

/**
 * @file slash_abi.h
 *
 * User-kernel ABI for the new SLASH filesystem interface.
 *
 * The SLASH driver exposes a custom filesystem mounted at @c /dev/slash.
 * Each accelerator receives a directory named by its PCI BDF (excluding the
 * function number), e.g. @c /dev/slash/0000:61:00/.  Within each device
 * directory the following endpoints are defined:
 *
 *   - @c info               — read-only binary info struct (struct slash_info)
 *   - @c qdma/              — directory; accepts QPAIR_ADD ioctl
 *   - @c qdma/qpair\<Q\>    — nameless qpair file; pread/pwrite only
 *   - @c bars/              — directory
 *   - @c bars/bar\<M\>      — BAR M of PF2; pread/pwrite only
 *   - @c hotplug            — global file; accepts hotplug ioctls
 *
 * The system-emulation daemon exposes the identical structure under
 * @c /run/slash_emu/, allowing the same UAPI to target real hardware or a
 * software-emulated accelerator without change.
 *
 * All ioctl structs carry a leading @c size field for forward-compatible ABI
 * versioning: the caller sets @c size = sizeof(struct ...) before the call.
 * The kernel (or daemon) uses this to detect older/newer callers.  New fields
 * are always appended; old readers ignore fields beyond their @c size.
 *
 * This header is shared between kernel and userspace (UAPI) and must remain
 * compatible with both build environments.  In particular:
 *
 *   - Only @c <linux/types.h> types (__u32, __u64, char) are used.
 *   - The ioctl include is wrapped in an @c __KERNEL__ guard.
 *   - No @c _GNU_SOURCE or other userspace-only definitions are required.
 */

#ifndef SLASH_UAPI_ABI_H
#define SLASH_UAPI_ABI_H

#include <linux/types.h>

#ifdef __KERNEL__
#include <linux/ioctl.h>
#else
#include <sys/ioctl.h>
#endif /* __KERNEL__ */

/* ─────────────────────────────────────────────────────────────────────────────
 * Common length constants
 * ─────────────────────────────────────────────────────────────────────────── */

/**
 * Maximum length (including NUL) of a PCI BDF string without the function
 * component, e.g. "0000:61:00".  Fits the longest possible DDDD:BB:DD form.
 *
 * Guarded to avoid a redefinition clash when this header is included in the
 * same translation unit as slash_interface.h, which defines the same constant
 * with the same value.
 */
#ifndef SLASH_PCI_BDF_LEN
#define SLASH_PCI_BDF_LEN 32
#endif

/**
 * Maximum length (including NUL) of a PCI BDF string *including* the function
 * component, e.g. "0000:03:00.0".  Used in hotplug requests where the full
 * BDF.F is required.
 *
 * Guarded for the same reason as SLASH_PCI_BDF_LEN above; slash_hotplug.h
 * defines this constant with the same value.
 */
#ifndef SLASH_HOTPLUG_BDF_LEN
#define SLASH_HOTPLUG_BDF_LEN 32
#endif


/* ─────────────────────────────────────────────────────────────────────────────
 * Information file  —  /dev/slash/<BDF>/info
 * ─────────────────────────────────────────────────────────────────────────── */

/**
 * Accelerator type flag: the accelerator is a system-emulated device, i.e. it
 * is implemented by the SLASH emulation daemon rather than real hardware.
 *
 * If this flag is clear the accelerator is a physical FPGA.
 */
#define SLASH_ACC_TYPE_SYSTEM_EMULATED 0x1u

/**
 * @brief Binary information returned by reading @c /dev/slash/<BDF>/info.
 *
 * The file is read-only.  The caller performs a single @c read(2) of at least
 * @c sizeof(struct slash_info) bytes.  New fields will be appended in future
 * ABI revisions; set and check @c size for compatibility.
 */
struct slash_info {
    /**
     * [in/out] ABI version / struct size.
     *
     * Caller must set this to @c sizeof(struct slash_info) before the read.
     * On return the kernel writes back the size of the struct *it* populated,
     * which may be smaller (older kernel) or equal (matched version).  The
     * caller must not access fields beyond the returned @c size.
     */
    __u32 size;

    /**
     * [out] Bitfield describing the nature of the accelerator.
     *
     * Currently defined bits:
     *   - @ref SLASH_ACC_TYPE_SYSTEM_EMULATED (0x1)
     *
     * All other bits are reserved and must be treated as zero.
     */
    __u32 acc_type;

    /**
     * [out] PCI Bus/Device string *without* the function component,
     * NUL-terminated, e.g. "0000:61:00".  Matches the directory name under
     * @c /dev/slash/.
     */
    char bdf[SLASH_PCI_BDF_LEN];
};


/* ─────────────────────────────────────────────────────────────────────────────
 * QDMA endpoint  —  /dev/slash/<BDF>/qdma/
 *
 * ioctl number allocation
 * ───────────────────────
 * The existing SLASH control-device ioctls occupy magic 'v', numbers 0x30–0x32.
 * The existing SLASH QDMA ioctls (slash_interface.h) occupy magic 'v', numbers
 * 0x50–0x53.
 * The hotplug ioctls (slash_hotplug.h / this header) occupy magic 'w', numbers
 * 0x30–0x33.
 *
 * The new filesystem QDMA directory ioctl uses magic 'x', starting at 0x40.
 * Magic 'x' is recorded as free in the kernel ioctl registry
 * (Documentation/userspace-api/ioctl/ioctl-number.rst) in the range we use,
 * and does not clash with any of the existing SLASH ioctls listed above.
 * ─────────────────────────────────────────────────────────────────────────── */

/** ioctl magic byte for filesystem QDMA directory commands. */
#define SLASH_ABI_QDMA_IOCTL_MAGIC 'x'

/**
 * @brief Input/output struct for the QPAIR_ADD ioctl.
 *
 * The ioctl is issued on the @c qdma/ directory file descriptor.  On success
 * the kernel allocates a queue pair, starts it immediately, and writes the
 * assigned ID into @c qid.  The caller may then open the newly created
 * @c qdma/qpair\<qid\> file.
 *
 * @c mode and @c dir_mask semantics are identical to the legacy
 * @c slash_qdma_qpair_add struct in @c slash_interface.h:
 *
 *   - @c mode: 0 = AXI Memory Mapped (QDMA_Q_MODE_MM); 1 = AXI Streaming
 *     (not yet supported, returns -EOPNOTSUPP).
 *   - @c dir_mask bits: 0x1 = H2C, 0x2 = C2H, 0x4 = CMPT (CMPT not yet
 *     supported).
 *   - Ring-size fields are CSR table indices (0–15), not byte counts.
 */
struct slash_abi_qdma_qpair_add {
    /**
     * [in] Struct size for ABI versioning.
     * Caller must set to @c sizeof(struct slash_abi_qdma_qpair_add).
     */
    __u32 size;

    /** [in] Queue operating mode (0 = MM, 1 = ST). */
    __u32 mode;

    /** [in] Direction bitmask: bit 0 = H2C, bit 1 = C2H, bit 2 = CMPT. */
    __u32 dir_mask;

    /** [in] Host-to-card descriptor ring size (CSR table index 0–15). */
    __u32 h2c_ring_sz;

    /** [in] Card-to-host descriptor ring size (CSR table index 0–15). */
    __u32 c2h_ring_sz;

    /** [in] Completion ring size (CSR table index 0–15). */
    __u32 cmpt_ring_sz;

    /**
     * [out] Kernel-assigned queue pair ID.
     *
     * Used to derive the path of the qpair file:
     * @c /dev/slash/<BDF>/qdma/qpair<qid>.
     * The queue is started before this ioctl returns; no separate START
     * operation is required.
     */
    __u32 qid;
};

/**
 * Add (allocate and start) a new QDMA queue pair.
 *
 * Issued on the @c qdma/ directory fd.  Fills in @c qid on success.
 * The kernel registers the new qpair in a per-device registry so that
 * forced teardown (device removal) can reach it even after the inode is
 * unlinked.
 */
#define SLASH_ABI_QDMA_IOCTL_QPAIR_ADD \
    _IOWR(SLASH_ABI_QDMA_IOCTL_MAGIC, 0x40, struct slash_abi_qdma_qpair_add)


/* ─────────────────────────────────────────────────────────────────────────────
 * Hotplug endpoint  —  /dev/slash/hotplug
 * ─────────────────────────────────────────────────────────────────────────── */

/** ioctl magic byte for hotplug commands (same as slash_hotplug.h). */
#define SLASH_ABI_HOTPLUG_IOCTL_MAGIC 'w'

/**
 * @brief Identify a device for a hotplug operation.
 *
 * Passed as the argument to REMOVE, TOGGLE_SBR, and HOTPLUG ioctls.
 */
struct slash_abi_hotplug_device_request {
    /**
     * [in] Struct size for ABI versioning.
     * Caller must set to @c sizeof(struct slash_abi_hotplug_device_request).
     */
    __u32 size;

    /**
     * [in] PCI Bus/Device/Function string *including* the function component,
     * NUL-terminated, e.g. "0000:03:00.0".
     *
     * For TOGGLE_SBR only the domain and bus number are used to locate the
     * upstream bridge; the device and function fields are accepted but
     * ignored beyond that.
     */
    char bdf[SLASH_HOTPLUG_BDF_LEN];
};

/**
 * Rescan all PCI root buses to discover new or reconfigured devices.
 *
 * No per-device argument; takes no struct.  Typically called after REMOVE or
 * TOGGLE_SBR to rediscover a device.
 *
 * On system-emulation: reloads daemon configuration and sets up all configured
 * system-emulated accelerators, skipping any whose BDF collides with an
 * already-running accelerator.
 */
#define SLASH_ABI_HOTPLUG_IOCTL_RESCAN \
    _IO(SLASH_ABI_HOTPLUG_IOCTL_MAGIC, 0x30)

/**
 * Remove a PCI device from the PCI hierarchy.
 *
 * On hardware: disables bus mastering (pci_clear_master()), then removes the
 * device (pci_stop_and_remove_bus_device()), invoking the driver's .remove
 * callback.  Associated @c bars/ and @c qdma/ endpoints disappear.
 *
 * On system-emulation: revokes communication resources (QDMA queue pairs and
 * BAR file descriptors) per the revocation contract below, and removes the
 * endpoint directories.  The underlying vpp_emu/vpp_sim model continues
 * running until both PF1 and PF2 have been removed.
 *
 * Revocation contract (both backends):
 *   - Resources are eagerly revoked on removal.
 *   - New open()/lookup of a removed endpoint returns -ENOENT.
 *   - Any operation on an already-open fd of a removed endpoint returns -ENODEV.
 *   - close() always succeeds; release is idempotent.
 */
#define SLASH_ABI_HOTPLUG_IOCTL_REMOVE \
    _IOW(SLASH_ABI_HOTPLUG_IOCTL_MAGIC, 0x31, struct slash_abi_hotplug_device_request)

/**
 * Assert a Secondary Bus Reset (SBR) on the upstream PCIe bridge.
 *
 * On hardware: saves bridge config space, asserts PCI_BRIDGE_CTL_BUS_RESET
 * for at least 2 ms, deasserts, restores config space, then sleeps ~1000 ms
 * for PCIe link retraining before returning.  The FPGA may still be
 * initializing after return; the caller should wait an additional 5–10 s
 * before rescanning.
 *
 * Only the domain and bus number from @c bdf are used to locate the upstream
 * bridge; the device and function fields are ignored.  The bridge resolves via
 * the bus number, which persists even after the endpoint has been removed.
 *
 * On system-emulation: fully removes the referenced accelerator, reloads
 * configuration, re-initializes all accelerators whose BDF is currently
 * available, and emulates the 1 s sleep.
 */
#define SLASH_ABI_HOTPLUG_IOCTL_TOGGLE_SBR \
    _IOW(SLASH_ABI_HOTPLUG_IOCTL_MAGIC, 0x32, struct slash_abi_hotplug_device_request)

/**
 * Atomically remove and rescan a single PCI device under the PCI lock.
 *
 * Equivalent to REMOVE followed immediately by RESCAN on the same parent bus,
 * without releasing pci_lock_rescan_remove() between the two operations.
 * Does not include an SBR; use TOGGLE_SBR separately if a hardware reset is
 * needed.
 *
 * On hardware postconditions:
 *   - Device is removed (pci_clear_master() + pci_stop_and_remove_bus_device()).
 *   - Parent bus is rescanned (pci_rescan_bus()); the device reappears if
 *     hardware is present.
 *
 * On system-emulation: fully removes the accelerator, reloads configuration,
 * and re-initializes all accelerators whose BDF is currently available.
 */
#define SLASH_ABI_HOTPLUG_IOCTL_HOTPLUG \
    _IOW(SLASH_ABI_HOTPLUG_IOCTL_MAGIC, 0x33, struct slash_abi_hotplug_device_request)


/* ─────────────────────────────────────────────────────────────────────────────
 * Memory-range constants  —  valid address ranges for QDMA transfers
 *
 * Accesses that target addresses outside these regions produce undefined
 * behavior; the driver or daemon may reject them with -ERANGE.
 * ─────────────────────────────────────────────────────────────────────────── */

/**
 * @name HBM address range
 *
 * 64 banks of 512 MiB each, for a total of 32 GiB.
 * @{
 */
/** First byte of the HBM address window. */
#define SLASH_HBM_BASE  0x0000004000000000ULL
/** First byte *past* the end of the HBM address window. */
#define SLASH_HBM_END   0x0000004800000000ULL
/** Number of HBM banks. */
#define SLASH_HBM_BANKS 64u
/** Size of one HBM bank in bytes (512 MiB). */
#define SLASH_HBM_BANK_SIZE (512ULL * 1024ULL * 1024ULL)
/** @} */

/**
 * @name DDR address range
 *
 * 4 banks of 8 GiB each, for a total of 32 GiB.
 * @{
 */
/** First byte of the DDR address window. */
#define SLASH_DDR_BASE  0x0000060000000000ULL
/** First byte *past* the end of the DDR address window. */
#define SLASH_DDR_END   0x0000060800000000ULL
/** Number of DDR banks. */
#define SLASH_DDR_BANKS 4u
/** Size of one DDR bank in bytes (8 GiB). */
#define SLASH_DDR_BANK_SIZE (8ULL * 1024ULL * 1024ULL * 1024ULL)
/** @} */

/**
 * @name Reconfiguration region
 *
 * The target address range for writing a new bitstream (PDI or VBIN).
 * The entire payload must be written in a single @c write(2) call to allow
 * the driver/daemon to identify the start and end of the reconfiguration
 * transfer.
 * @{
 */
/** First byte of the reconfiguration region. */
#define SLASH_RECONFIG_BASE 0x0000000102100000ULL
/** First byte *past* the end of the reconfiguration region. */
#define SLASH_RECONFIG_END  0x0000000142100000ULL
/** @} */


/* ─────────────────────────────────────────────────────────────────────────────
 * BAR geometry constants  —  PF2 BARs exposed under /dev/slash/<BDF>/bars/
 *
 * Only BARs 0, 2, and 4 exist on PF2.  BAR 1, 3, and 5 are absent.
 * The size of each BAR is also encoded as the size of the corresponding
 * bars/bar<M> file; these constants are provided for callers that need the
 * value before opening the file.
 * ─────────────────────────────────────────────────────────────────────────── */

/**
 * @name BAR 0 — user region
 * @{
 */
/** BAR index of the user region on PF2. */
#define SLASH_BAR_USER_IDX  0u
/** Size of BAR 0 (user region) in bytes: 128 MiB. */
#define SLASH_BAR_USER_SIZE (128ULL * 1024ULL * 1024ULL)
/** @} */

/**
 * @name BAR 2 — service layer
 * @{
 */
/** BAR index of the service layer on PF2. */
#define SLASH_BAR_SL_IDX    2u
/** Size of BAR 2 (service layer) in bytes: 128 MiB. */
#define SLASH_BAR_SL_SIZE   (128ULL * 1024ULL * 1024ULL)
/** @} */

/**
 * @name BAR 4 — clock wizard
 * @{
 */
/** BAR index of the clock wizard on PF2. */
#define SLASH_BAR_CLK_IDX   4u
/** Size of BAR 4 (clock wizard) in bytes: 512 KiB. */
#define SLASH_BAR_CLK_SIZE  (512ULL * 1024ULL)
/** @} */

#endif /* SLASH_UAPI_ABI_H */
