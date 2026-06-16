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
 * @file config.h
 * @brief Configuration data model and persistent accelerator model for slash-emu.
 *
 * slash-emu reads an INI-style configuration file (parsed via libinih, mirroring
 * vrtd's config layer) that describes the set of system-emulated SLASH
 * accelerators the daemon should expose.  Per the architecture, the configurable
 * surface is deliberately small:
 *
 *   - The accelerator @em BDF (the per-device folder name, board-level
 *     @c DDDD:BB:DD form @em without a function suffix).
 *   - Reserved schema space for future per-accelerator @em network configuration
 *     (parsed into a forward-compatible struct, but otherwise unused today).
 *
 * Everything else is hard-wired.  The on-disk format is one section per
 * accelerator:
 *
 * @code{.ini}
 *   [accelerator:0000:61:00]
 *   ; reserved network keys (forward-compatible, currently inert):
 *   net-mac  = 02:00:00:00:00:01
 *   net-ip   = 10.0.0.1
 *   net-port = 4791
 * @endcode
 *
 * @section model Persistent accelerator model
 *
 * Accelerators are persistent: they outlive any individual user process.  The
 * daemon keeps two notions of state:
 *
 *   - The @em configured set: every @c struct @c emu_accelerator parsed from the
 *     config file, owned by @c struct @c emu_config.
 *   - The @em running set: the BDFs of accelerators currently instantiated in
 *     the live FUSE tree.  This module models the running set as an opaque
 *     @c struct @c emu_running_set so that the RESCAN / hotplug reload logic
 *     (T9) can ask "which configured accelerators should I bring up now?".
 *
 * @section reload Reload / merge semantics (RESCAN)
 *
 * The hotplug RESCAN operation reloads the config and instantiates every
 * configured accelerator @em whose BDF does not collide with an already-running
 * accelerator.  @c emu_config_select_new() implements exactly this set
 * computation as a pure data operation; materialising the selected accelerators
 * into FUSE nodes is the job of the filesystem layer (T5).
 *
 * @note The interface seam used by @c main.c and @c fs.c -- the opaque
 *       @c struct @c emu_config, @c emu_config_load(), @c cleanup_config(), and
 *       @c cleanup_configp() -- is intentionally preserved from the scaffold.
 *       The data model below is layered behind it.
 */

#ifndef SLASH_EMU_CONFIG_H
#define SLASH_EMU_CONFIG_H

#include "array.h"

/**
 * @brief Length of a normalized board-level BDF string, including the NUL.
 *
 * Board-level BDFs are @c "DDDD:BB:DD" (domain:bus:device, no function): four
 * hex domain digits, two hex bus digits, two hex device digits, two colons,
 * and a terminating NUL -> 11 bytes.  Sized to a round 16 for headroom.
 */
#define EMU_BDF_LEN 16

/**
 * @brief Reserved per-accelerator network configuration.
 *
 * The architecture reserves schema space for future network configuration of an
 * emulated accelerator; it is parsed here for forward compatibility but is not
 * acted upon yet.  Storing the raw strings (rather than parsed binary forms)
 * keeps the parser permissive and avoids committing to a validation policy
 * before the feature is designed.
 *
 * All fields are optional; an absent key leaves the corresponding pointer NULL.
 */
struct emu_net_config {
    /** @brief Reserved: MAC address string, or NULL if unset (heap, owning). */
    char *mac; /* owning */
    /** @brief Reserved: IP address string, or NULL if unset (heap, owning). */
    char *ip; /* owning */
    /** @brief Reserved: UDP/TCP port string, or NULL if unset (heap, owning). */
    char *port; /* owning */
};

/**
 * @brief A single configured system-emulated accelerator.
 *
 * Identified by its board-level BDF (the per-device folder name).  Carries the
 * reserved network configuration and is sized to grow: as the model gains
 * backing descriptors (SIM model, vpp_emu bridge, ...), they are added here.
 */
struct emu_accelerator {
    /** @brief Normalized board-level BDF "DDDD:BB:DD" (NUL-terminated). */
    char bdf[EMU_BDF_LEN];

    /** @brief Reserved network configuration (forward-compatible, inert). */
    struct emu_net_config net;
};

/**
 * @brief Release all resources owned by an accelerator.
 * @param acc Pointer to the accelerator to clean up (may be NULL).
 */
void cleanup_accelerator(struct emu_accelerator *acc);

/** @brief Owning array of accelerator pointers (frees accelerators on cleanup). */
DECLARE_OWNING_PTR_ARRAY(emu_accelerator_ptr_array, struct emu_accelerator *, cleanup_accelerator)

/**
 * @brief Top-level slash-emu configuration container.
 *
 * Owns the configured accelerator set and remembers the path it was loaded from
 * (for diagnostics and reloads).
 */
struct emu_config {
    /** @brief Path the config was loaded from, or NULL for the built-in default
     *         (heap-allocated, owning). */
    char *source_path; /* owning */

    /** @brief All configured accelerators (owning array). */
    struct emu_accelerator_ptr_array accelerators;
};

/**
 * @brief Load and parse the slash-emu configuration from @p path.
 *
 * Parses the INI file at @p path, validating every accelerator BDF and rejecting
 * malformed, empty, or duplicate-BDF configurations.  A NULL @p path yields an
 * empty (zero-accelerator) configuration without touching the filesystem, which
 * is the built-in default used when @c --config is not supplied.
 *
 * @param path        Path to the configuration file, or NULL for the built-in
 *                    empty default.
 * @param[out] config On success, receives a heap-allocated config.  The caller
 *                    owns it and must release it with @c cleanup_config.
 * @return 0 on success, -1 on error (logged via sd_journal).
 */
int emu_config_load(const char *path, struct emu_config **config);

/**
 * @brief Release all resources owned by a config.
 * @param config Pointer to the config to clean up (may be NULL).
 */
void cleanup_config(struct emu_config *config);

/**
 * @brief Cleanup helper for use with @c __attribute__((cleanup)).
 * @param configp Address of a @c struct @c emu_config pointer.
 */
static inline
void cleanup_configp(struct emu_config **configp)
{
    if (configp == NULL) {
        return;
    }

    cleanup_config(*configp);

    *configp = NULL;
}

/**
 * @brief Look up a configured accelerator by normalized BDF.
 *
 * @param config The configuration to search.
 * @param bdf    Normalized board-level BDF ("DDDD:BB:DD").
 * @return Borrowed pointer to the matching accelerator, or NULL if none.
 */
const struct emu_accelerator *emu_config_find(const struct emu_config *config,
                                              const char *bdf);

/**
 * @brief Normalize and validate a board-level BDF string.
 *
 * Accepts a board-level BDF in canonical @c "DDDD:BB:DD" form, or the short
 * @c "BB:DD" form (expanded to domain @c 0000).  A trailing @c ".F" function
 * suffix is rejected: accelerator folders are named at board level and must not
 * carry a function.  All hex fields are validated and lower-cased.
 *
 * @param input       The raw BDF string.
 * @param[out] out    Output buffer for the normalized BDF.
 * @param out_len     Size of @p out (must be at least @c EMU_BDF_LEN).
 * @return 0 on success, -1 on invalid format or insufficient buffer.
 */
int emu_bdf_normalize(const char *input, char *out, size_t out_len);

/* ========================================================================
 * Running set + reload/merge (RESCAN) -- consumed by the FUSE layer (T5/T9).
 * ======================================================================== */

/**
 * @brief The set of BDFs currently instantiated in the live FUSE tree.
 *
 * Models the "running" accelerators independently of the configuration so the
 * reload logic can decide which configured accelerators to bring up without
 * colliding with one that is already live.  Opaque; manipulate via the helpers
 * below.
 */
struct emu_running_set;

/**
 * @brief Allocate an empty running set.
 * @param[out] setp On success, receives a heap-allocated running set.  The
 *                  caller owns it and must release it with @c cleanup_running_set.
 * @return 0 on success, -1 on allocation failure.
 */
int emu_running_set_new(struct emu_running_set **setp);

/**
 * @brief Release a running set and all BDF strings it holds.
 * @param set The running set to clean up (may be NULL).
 */
void cleanup_running_set(struct emu_running_set *set);

/**
 * @brief Cleanup helper for use with @c __attribute__((cleanup)).
 * @param setp Address of a @c struct @c emu_running_set pointer.
 */
static inline
void cleanup_running_setp(struct emu_running_set **setp)
{
    if (setp == NULL) {
        return;
    }

    cleanup_running_set(*setp);

    *setp = NULL;
}

/**
 * @brief Test whether a normalized BDF is in the running set.
 * @param set The running set.
 * @param bdf Normalized board-level BDF.
 * @return true if present, false otherwise.
 */
bool emu_running_set_contains(const struct emu_running_set *set, const char *bdf);

/**
 * @brief Mark a normalized BDF as running (idempotent).
 * @param set The running set to modify.
 * @param bdf Normalized board-level BDF to add.
 * @return 0 on success, -1 on allocation failure.
 */
int emu_running_set_add(struct emu_running_set *set, const char *bdf);

/**
 * @brief Mark a normalized BDF as no longer running (no-op if absent).
 * @param set The running set to modify.
 * @param bdf Normalized board-level BDF to remove.
 */
void emu_running_set_remove(struct emu_running_set *set, const char *bdf);

/**
 * @brief Compute the accelerators to instantiate on a RESCAN-style reload.
 *
 * Implements the RESCAN data semantics: every accelerator in @p config whose
 * BDF is @em not already in @p running is selected for instantiation; BDFs that
 * collide with an already-running accelerator are skipped.  The returned array
 * holds @em borrowed pointers into @p config (it does not own the accelerators);
 * free it with @c emu_accelerator_ptr_array_free, which does not touch the
 * referenced accelerators because the array is non-owning here -- see the note.
 *
 * @param      config The freshly loaded configuration.
 * @param      running The set of already-running BDFs (NULL is treated as empty).
 * @param[out] out    On success, receives borrowed pointers to the selected
 *                    accelerators (caller frees the array storage via
 *                    @c emu_accelerator_ref_array_free; elements are not owned).
 * @return 0 on success, -1 on allocation failure.
 */
int emu_config_select_new(const struct emu_config *config,
                          const struct emu_running_set *running,
                          struct emu_accelerator_ref_array *out);

#endif // SLASH_EMU_CONFIG_H
