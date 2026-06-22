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
 * @file hotplug.hpp
 * @brief The global @c /hotplug endpoint: the device-lifecycle control surface.
 *
 * Exactly one @c hotplug file, attached at the mount root as a sibling of the
 * per-device @c <BDF>/ directories.  It is ioctl-only and drives the four
 * device-lifecycle commands (@c slash_abi.h):
 *
 *   - @b RESCAN: reload the config and set up all configured accelerators, skipping
 *     any whose BDF collides with one already running (re-invokes the idempotent
 *     materialize path).
 *   - @b REMOVE (BDF with function): remove one PCI function (function 1 = @c qdma/,
 *     function 2 = @c bars/); the subtree disappears and resources are eagerly
 *     revoked.  The model keeps running until @em both functions are removed, then
 *     the per-device model-shutdown seam fires.
 *   - @b TOGGLE_SBR: fully remove the accelerator, reload, re-init available
 *     accelerators, and emulate the ~1 s PCIe-retraining sleep.
 *   - @b HOTPLUG: like TOGGLE_SBR but without the SBR and without the sleep.
 *
 * @section reload Reload seam
 *
 * RESCAN / TOGGLE_SBR / HOTPLUG reload the on-disk config and re-run the
 * materialize path, both of which live in the FUSE layer.  The FUSE layer supplies
 * a @ref ReloadFn at attach time; this module invokes it (with the tree lock @em
 * not held -- the hotplug ioctl hook runs revoke + reload as two separately-locked
 * phases).  Tests supply a stub reload.
 *
 * @section sleep SBR sleep
 *
 * The single-threaded session means a literal 1 s sleep blocks the daemon for that
 * second (accepted for a rare, genuinely-blocking op; no thread for step 1).  The
 * duration is injectable (@ref hotplugSetSbrSleepUs) so tests use a tiny value.
 */

#ifndef SLASH_EMU_HOTPLUG_HPP
#define SLASH_EMU_HOTPLUG_HPP

#include <functional>
#include <string>

#include "node.hpp"

namespace slash::emu {

/** @brief Default emulated SBR sleep, in microseconds (~1 s). */
inline constexpr unsigned kHotplugSbrSleepUsDefault = 1000u * 1000u;

/**
 * @brief Reload callback the FUSE layer supplies for RESCAN/SBR/HOTPLUG.
 *
 * Reloads the daemon config and re-runs the (idempotent) materialize path.
 * Invoked with the tree lock @em not held.
 *
 * @return 0 on success, -1 on error (logged by the callback).
 */
using ReloadFn = std::function<int()>;

/**
 * @brief Attach the global @c /hotplug file at the mount root.
 *
 * Creates a single @c hotplug FILE node under the tree root with the ioctl handler
 * dispatching the four commands.  Call exactly once.
 *
 * @param tree   The node tree (the root is the parent of the @c hotplug file).
 * @param reload Reload callback (empty in tests that do not exercise reload; those
 *               commands then succeed without re-initializing anything).
 * @return 0 on success, -1 on error (logged via sd_journal).
 */
int hotplugAttach(NodeTree &tree, ReloadFn reload);

/**
 * @brief Override the emulated SBR sleep duration (test injection seam).
 * @param sleep_us New sleep duration in microseconds (0 disables it).
 * @return 0 on success; -1 if the hotplug endpoint is not attached.
 */
int hotplugSetSbrSleepUs(NodeTree &tree, unsigned int sleep_us);

/**
 * @brief Parse and validate a hotplug request BDF ("DDDD:BB:DD.F").
 *
 * Splits a function-qualified BDF into the normalized board-level BDF and the PCI
 * function (@c .1 -> @ref DeviceFunction::Qdma, @c .2 -> @ref DeviceFunction::Bars).
 * Exposed for unit testing.
 *
 * @param      input    The raw request BDF including the function (short
 *                      @c "61:00.1" form also accepted).
 * @param[out] bdf_out  Receives the normalized board-level BDF.
 * @param[out] func_out Receives the parsed function.
 * @return 0 on success; @c -EINVAL on a malformed BDF or missing/garbled function;
 *         @c -EOPNOTSUPP on a valid-but-unsupported function (not 1 or 2).
 */
int hotplugParseBdf(const std::string &input, std::string &bdf_out,
                    DeviceFunction &func_out);

} // namespace slash::emu

#endif // SLASH_EMU_HOTPLUG_HPP
