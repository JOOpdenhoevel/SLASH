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
 * @file hotplug.h
 * @brief The global @c /hotplug endpoint: the device-lifecycle control surface.
 *
 * Unlike the per-device endpoints (info / bars / qdma), there is exactly @em one
 * @c hotplug file, attached at the mount root as a sibling of the per-device
 * @c <BDF>/ directories.  It is ioctl-only and drives the four device-lifecycle
 * commands defined by the ABI (@c slash_abi.h):
 *
 *   - @b RESCAN (@c _IO, no arg): reload the daemon configuration and set up all
 *     configured accelerators, skipping any whose BDF collides with one already
 *     running.  Re-invokes the (idempotent) materialize path.
 *   - @b REMOVE (@c slash_abi_hotplug_device_request, BDF @em with function):
 *     remove a single PCI function of a device.  Function 1 is the QDMA endpoint
 *     (@c qdma/), function 2 the control-register BARs (@c bars/).  The endpoint
 *     subtree disappears and its resources are eagerly revoked (the revocation
 *     contract: new lookup -> @c -ENOENT, op on an open fd -> @c -ENODEV, close
 *     always succeeds).  The underlying vpp_emu/vpp_sim model keeps running until
 *     @em both functions have been removed, at which point the per-device
 *     model-shutdown seam fires (see @ref emu_device_set_model_shutdown).
 *   - @b TOGGLE_SBR (request): fully remove the referenced accelerator, reload
 *     config, re-initialize all configured accelerators whose BDF is available,
 *     and emulate the ~1 s PCIe-link-retraining sleep.
 *   - @b HOTPLUG (request): like TOGGLE_SBR but without the SBR and without the
 *     sleep (remove + reload + re-init).
 *
 * @section reload Reload seam
 *
 * RESCAN / TOGGLE_SBR / HOTPLUG all need to reload the on-disk configuration and
 * re-run the materialize path -- both of which live in the FUSE layer (it owns
 * the config path and the per-device attach sequence).  This module therefore
 * does not reach into @c fs.c; instead the FUSE layer supplies a reload callback
 * (@ref emu_hotplug_reload_fn) at attach time, and this module invokes it.  Tests
 * supply a stub reload to assert the command sequencing without a real config or
 * mount.
 *
 * @section sleep SBR sleep
 *
 * The FUSE session is single-threaded, so a literal 1 s sleep in the TOGGLE_SBR
 * handler blocks the whole daemon for that second (no other request is serviced
 * meanwhile).  This is an accepted tradeoff for an operation that is rare and, on
 * real hardware, genuinely blocking; we do @em not spin up a thread for step 1.
 * The sleep duration is injectable (@ref emu_hotplug_set_sbr_sleep_us) so tests
 * use a tiny value and never pay the wall-clock second.
 */

#ifndef SLASH_EMU_HOTPLUG_H
#define SLASH_EMU_HOTPLUG_H

#include "node.h"

/** @brief Default emulated SBR sleep, in microseconds (~1 s; see @ref sleep). */
#define EMU_HOTPLUG_SBR_SLEEP_US_DEFAULT (1000u * 1000u)

/**
 * @brief Reload callback the FUSE layer supplies for RESCAN/SBR/HOTPLUG.
 *
 * Reloads the daemon configuration from its source and re-runs the materialize
 * path (which is idempotent: it seeds the running-set from the live devices, so
 * an already-running accelerator is not reselected or double-attached).  Invoked
 * with the tree lock @em not held -- the materialize path takes the lock itself,
 * and the hotplug ioctl hook drops the spine lock before calling here (it runs
 * the revoke + reload as two separately-locked phases, not one atomic critical
 * section).
 *
 * @param ctx The opaque context registered alongside the callback.
 * @return 0 on success, -1 on error (logged by the callback).
 */
typedef int (*emu_hotplug_reload_fn)(void *ctx);

/**
 * @brief Attach the global @c /hotplug file at the mount root.
 *
 * Creates a single @c hotplug @em file node under the tree root with the ioctl
 * vtable dispatching the four device-lifecycle commands.  Must be called exactly
 * once (it is not per-device); the FUSE layer calls it from @c emu_fs_create
 * alongside the initial materialize.
 *
 * @param tree    The node tree (the root is the parent of the @c hotplug file).
 * @param reload  Reload callback for RESCAN/SBR/HOTPLUG (may be NULL in tests
 *                that do not exercise reload; those commands then succeed without
 *                re-initializing anything).
 * @param ctx     Opaque context passed back to @p reload (borrowed).
 * @return 0 on success, -1 on error (logged via sd_journal).
 */
int emu_hotplug_attach(struct emu_node_tree *tree, emu_hotplug_reload_fn reload,
                       void *ctx);

/**
 * @brief Override the emulated SBR sleep duration (test injection seam).
 *
 * Sets the microsecond duration the TOGGLE_SBR handler sleeps to emulate PCIe
 * link retraining.  Defaults to @ref EMU_HOTPLUG_SBR_SLEEP_US_DEFAULT.  Tests
 * set a tiny value so the single-threaded daemon is not blocked for a real
 * second.  No-op if the @c hotplug file has not been attached.
 *
 * @param tree       The tree (for locking / to find the hotplug node).
 * @param sleep_us   The new sleep duration in microseconds (0 disables it).
 * @return 0 on success; -1 if the hotplug endpoint is not attached.
 */
int emu_hotplug_set_sbr_sleep_us(struct emu_node_tree *tree,
                                 unsigned int sleep_us);

/**
 * @brief Parse and validate a hotplug request BDF ("DDDD:BB:DD.F").
 *
 * Splits a function-qualified BDF into the normalized board-level BDF (the
 * per-device folder name, no function) and the PCI function.  The function maps
 * to a removable endpoint per the ABI: @c .1 -> @ref EMU_DEVICE_FUNCTION_QDMA,
 * @c .2 -> @ref EMU_DEVICE_FUNCTION_BARS.  Any other function value is rejected.
 *
 * Exposed (rather than file-static) so it can be unit-tested directly.
 *
 * @param      input    The raw request BDF including the function, e.g.
 *                      "0000:61:00.1" (short "61:00.1" form also accepted).
 * @param[out] bdf_out  Buffer for the normalized board-level BDF (>= EMU_BDF_LEN).
 * @param      bdf_len  Size of @p bdf_out.
 * @param[out] func_out Receives the parsed function.
 * @return 0 on success; -EINVAL on a malformed BDF or a missing/garbled function
 *         suffix; -EOPNOTSUPP on a syntactically valid but unsupported function
 *         (not 1 or 2).
 */
int emu_hotplug_parse_bdf(const char *input, char *bdf_out, size_t bdf_len,
                          enum emu_device_function *func_out);

#endif // SLASH_EMU_HOTPLUG_H
