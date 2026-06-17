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
 * @file config.c
 * @brief INI configuration loader and persistent accelerator model for slash-emu.
 *
 * This file implements the configuration subsystem behind @c config.h.  It uses
 * an INI-style configuration file (parsed via libinih, mirroring vrtd's config
 * layer) with one section per emulated accelerator:
 *
 * @code{.ini}
 *   [accelerator:0000:61:00]
 *   net-mac  = 02:00:00:00:00:01   ; reserved, currently inert
 *   net-ip   = 10.0.0.1            ; reserved, currently inert
 *   net-port = 4791                ; reserved, currently inert
 * @endcode
 *
 * The section object name is the accelerator's board-level BDF (no function
 * suffix), normalized to canonical @c "DDDD:BB:DD" form.  Duplicate BDFs (after
 * normalization) and malformed BDFs are rejected.  Per the vrtd return-value
 * convention every fallible function returns 0 on success and -1 on error; the
 * inih callback is the lone exception, following inih's own 1/0 convention.
 *
 * @section keyless Keyless sections (stock libinih)
 *
 * slash-emu uses the distro libinih, exactly as vrtd does.  Stock libinih ships
 * @c INI_CALL_HANDLER_ON_NEW_SECTION OFF, so the parser callback fires only for
 * key/value pairs -- a section header with no keys produces no callback at all
 * and is therefore @em invisible to us.  Consequently each
 * @c [accelerator:<bdf>] section MUST carry at least one key; a truly keyless
 * section is silently ignored (it cannot be detected without the new-section
 * callback).  Each key callback find-or-creates the accelerator for its
 * (normalized) section BDF, so the accelerator is materialized on its first key.
 * The shipped sample config and every documented example carry a reserved
 * @c net-* key for exactly this reason.
 *
 * Beyond parsing, this file owns the persistent accelerator model: the
 * configured set (@c struct @c emu_config) and the "running" set
 * (@c struct @c emu_running_set), plus the pure set computation
 * (@c emu_config_select_new) backing the hotplug RESCAN reload.
 */

#define _GNU_SOURCE

#include "config.h"

#include <assert.h>
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <syslog.h>

#include <ini.h>

#include "array.h"
#include "utils.h"

/* On Ubuntu's libinih the callback takes (user, section, name, value). */
static_assert(INI_HANDLER_LINENO == 0, "slash-emu does not support INI_HANDLER_LINENO = 1");

/** @brief INI section prefix introducing an accelerator: "[accelerator:<BDF>]". */
static const char ACCELERATOR_PREFIX[] = "accelerator";

/**
 * @brief Transient state carried through a single emu_config_load() invocation.
 *
 * Stock libinih reports no new-section callback, so the parser does a
 * find-or-create by normalized BDF on every key.  To still reject a duplicate
 * BDF that is spelled differently across two sections, @c section_bdfs records,
 * in lock-step with @c config->accelerators, the @em raw section string that
 * first created each accelerator.  A later key whose raw section string differs
 * but whose normalized BDF already exists is a duplicate and is rejected; a key
 * whose raw section string matches is simply the same section continuing.
 */
struct config_parse_state {
    /** @brief The config being populated (non-owning; owned by the caller). */
    struct emu_config *config;
    /** @brief Raw section strings, parallel to @c config->accelerators: entry
     *         @c i is the @c [accelerator:<raw>] header that created accelerator
     *         @c i.  Owning. */
    struct str_array section_bdfs;
    /** @brief Set true by the callback on any hard error, so the loader can
     *         distinguish a real failure from inih's own error codes. */
    bool failed;
};

/* Forward declarations for internal helpers. */
static int parse_config_callback(void *user, const char *section,
                                 const char *name, const char *value);
static int accelerator_find_or_create(struct config_parse_state *state,
                                      const char *section, const char *bdf_raw,
                                      struct emu_accelerator **out);
static int accelerator_add_value(struct emu_accelerator *acc,
                                 const char *name, const char *value);
static int dup_into(char **dst, const char *value);

/* ========================================================================
 * Cleanup helpers
 * ======================================================================== */

void cleanup_accelerator(struct emu_accelerator *acc)
{
    if (acc == NULL) {
        return;
    }

    free(acc->net.mac);
    free(acc->net.ip);
    free(acc->net.port);

    free(acc);
}

void cleanup_config(struct emu_config *config)
{
    if (config == NULL) {
        return;
    }

    free(config->source_path);

    emu_accelerator_ptr_array_free(&config->accelerators);

    free(config);
}

/* ========================================================================
 * BDF normalization / validation
 * ======================================================================== */

/**
 * @brief Validate that @p s is exactly @p n lower-cased hex digits.
 *
 * @param s Pointer to the first character.
 * @param n Required number of hex digits.
 * @return 0 if @p s[0..n) are all hex digits, -1 otherwise.
 */
static int check_hex_run(const char *s, size_t n)
{
    for (size_t i = 0; i < n; i++) {
        if (!isxdigit((unsigned char) s[i])) {
            return -1;
        }
    }

    return 0;
}

int emu_bdf_normalize(const char *input, char *out, size_t out_len)
{
    if (input == NULL || out == NULL || out_len < EMU_BDF_LEN) {
        return -1;
    }

    /* Reject a function suffix outright: accelerator folders are board-level. */
    if (strchr(input, '.') != NULL) {
        LOG(LOG_ERR, "BDF '%s' must not include a function suffix", input);
        return -1;
    }

    /* Count colons to distinguish "BB:DD" from "DDDD:BB:DD". */
    size_t colons = 0;
    for (const char *p = input; *p != '\0'; p++) {
        if (*p == ':') {
            colons++;
        }
    }

    const char *domain;
    size_t domain_len;
    const char *bus_dev;

    if (colons == 1) {
        /* Short form "BB:DD" -> implicit domain 0000. */
        domain = "0000";
        domain_len = 4;
        bus_dev = input;
    } else if (colons == 2) {
        /* Full form "DDDD:BB:DD". */
        const char *first = strchr(input, ':');
        domain = input;
        domain_len = (size_t) (first - input);
        bus_dev = first + 1;
    } else {
        LOG(LOG_ERR, "Invalid BDF format: '%s'", input);
        return -1;
    }

    /* Validate domain (only present explicitly in the full form). */
    if (colons == 2) {
        if (domain_len != 4 || check_hex_run(domain, 4) == -1) {
            LOG(LOG_ERR, "Invalid BDF domain in '%s'", input);
            return -1;
        }
    }

    /* bus_dev must be exactly "BB:DD": 2 hex, colon, 2 hex. */
    if (strlen(bus_dev) != 5 || bus_dev[2] != ':' ||
        check_hex_run(bus_dev, 2) == -1 || check_hex_run(bus_dev + 3, 2) == -1) {
        LOG(LOG_ERR, "Invalid BDF bus/device in '%s'", input);
        return -1;
    }

    int ret = snprintf(out, out_len, "%.*s:%c%c:%c%c",
                       (int) domain_len, domain,
                       bus_dev[0], bus_dev[1], bus_dev[3], bus_dev[4]);
    if (ret < 0 || (size_t) ret >= out_len) {
        LOG(LOG_ERR, "BDF buffer too small for '%s'", input);
        return -1;
    }

    /* Canonicalize to lower-case hex. */
    for (char *p = out; *p != '\0'; p++) {
        *p = (char) tolower((unsigned char) *p);
    }

    return 0;
}

/* ========================================================================
 * Accelerator parsing
 * ======================================================================== */

const struct emu_accelerator *emu_config_find(const struct emu_config *config,
                                              const char *bdf)
{
    if (config == NULL || bdf == NULL) {
        return NULL;
    }

    for (size_t i = 0; i < config->accelerators.len; i++) {
        if (strcmp(config->accelerators.d[i]->bdf, bdf) == 0) {
            return config->accelerators.d[i];
        }
    }

    return NULL;
}

/**
 * @brief Resolve the accelerator a key belongs to, creating it on first sight.
 *
 * Normalizes @p bdf_raw (the part after @c "accelerator:" in the section header)
 * and looks it up in the config:
 *
 *   - If no accelerator with that normalized BDF exists, a fresh one is created,
 *     appended to @c config->accelerators, and its raw section string recorded in
 *     @c state->section_bdfs (kept parallel to the accelerator array).
 *   - If one exists and was created from the @em same raw section string, this is
 *     the same section continuing -- the existing accelerator is returned.
 *   - If one exists but was created from a @em different raw section string, the
 *     same BDF has been declared twice (spelled differently) -- a duplicate, and
 *     a hard error.
 *
 * @param      state    Parse state (provides the config and the section map).
 * @param      section  The full raw section string ("accelerator:<raw>").
 * @param      bdf_raw  The raw BDF (the part after the colon).
 * @param[out] out      Receives the resolved accelerator on success.
 * @return 0 on success, -1 on invalid/duplicate BDF or allocation failure.
 */
static int accelerator_find_or_create(struct config_parse_state *state,
                                      const char *section, const char *bdf_raw,
                                      struct emu_accelerator **out)
{
    struct emu_config *config = state->config;

    char bdf[EMU_BDF_LEN];
    int ret = emu_bdf_normalize(bdf_raw, bdf, sizeof(bdf));
    PROPAGATE_ERROR(ret);

    /* Already created?  Same raw section => continue it; different => duplicate. */
    for (size_t i = 0; i < config->accelerators.len; i++) {
        if (strcmp(config->accelerators.d[i]->bdf, bdf) != 0) {
            continue;
        }

        if (strcmp(state->section_bdfs.d[i], section) == 0) {
            *out = config->accelerators.d[i];
            return 0;
        }

        LOG(LOG_ERR, "Duplicate accelerator BDF '%s'", bdf);
        return -1;
    }

    /* First sight of this BDF: create and record its raw section string. */
    _cleanup_(cleanup_acceleratorp)
    struct emu_accelerator *acc = calloc(1, sizeof(*acc));
    PROPAGATE_ERROR_NULL_LOG(acc, LOG_ERR, "Failed to allocate accelerator");

    memcpy(acc->bdf, bdf, sizeof(bdf));

    _cleanup_(cleanup_free)
    char *section_copy = strdup(section);
    PROPAGATE_ERROR_NULL_LOG(section_copy, LOG_ERR, "Failed to track section");

    ret = emu_accelerator_ptr_array_push(&config->accelerators, acc);
    PROPAGATE_ERROR_LOG(ret, LOG_ERR, "Failed to store accelerator '%s'", bdf);

    /* Keep section_bdfs in lock-step.  If this push fails, drop the accelerator
     * we just appended so the two arrays do not desync. */
    ret = str_array_push_move(&state->section_bdfs, &section_copy);
    if (ret == -1) {
        emu_accelerator_ptr_array_pop_safe(&config->accelerators, NULL);
        cleanup_accelerator(acc);
        acc = NULL;
        LOG(LOG_ERR, "Failed to track section for accelerator '%s'", bdf);
        return -1;
    }

    *out = acc;
    acc = NULL;

    return 0;
}

/**
 * @brief Duplicate @p value into @p *dst, replacing any prior value.
 *
 * @param dst   Address of an owning string pointer (freed and overwritten).
 * @param value The string to duplicate.
 * @return 0 on success, -1 on allocation failure.
 */
static int dup_into(char **dst, const char *value)
{
    char *copy = strdup(value);
    PROPAGATE_ERROR_NULL_LOG(copy, LOG_ERR, "Failed to duplicate config value");

    free(*dst);
    *dst = copy;

    return 0;
}

/**
 * @brief Apply a single key/value pair to an accelerator.
 *
 * Recognizes the reserved (currently inert) network keys.  Unknown keys are a
 * hard error so typos in the config are caught rather than silently ignored.
 *
 * @param acc   The accelerator to modify.
 * @param name  Key name.
 * @param value Value string.
 * @return 0 on success, -1 on unknown key or allocation failure.
 */
static int accelerator_add_value(struct emu_accelerator *acc,
                                 const char *name, const char *value)
{
    if (strcmp(name, "net-mac") == 0) {
        return dup_into(&acc->net.mac, value);
    } else if (strcmp(name, "net-ip") == 0) {
        return dup_into(&acc->net.ip, value);
    } else if (strcmp(name, "net-port") == 0) {
        return dup_into(&acc->net.port, value);
    }

    LOG(LOG_ERR, "Unknown accelerator key: '%s'", name);
    return -1;
}

/**
 * @brief inih callback: dispatch each INI key/value to the right handler.
 *
 * Stock libinih invokes this once per key/value pair only -- never on a bare
 * section header (see the keyless-section note in the file header), so a section
 * is observed exactly when its first key arrives.  Every call therefore:
 *   1. validates that @p section is @c "accelerator:<bdf>";
 *   2. find-or-creates the accelerator for that (normalized) BDF -- which also
 *      enforces duplicate-BDF rejection across differently-spelled sections; and
 *   3. applies the key to that accelerator.
 * A non-accelerator section, or a key in the top-level (empty) section, is a hard
 * error.  Follows the inih convention: returns 1 on success, 0 on error (which
 * aborts the parse).
 *
 * @param user    Opaque pointer to struct config_parse_state.
 * @param section INI section name (e.g. "accelerator:0000:61:00").
 * @param name    Key name within the section.
 * @param value   Value string associated with the key.
 * @return 1 on success, 0 on error (per inih convention).
 */
static int parse_config_callback(void *user, const char *section,
                                 const char *name, const char *value)
{
    struct config_parse_state *state = user;

    /* The section must be "accelerator:<bdf>". */
    const char *colon = strchr(section, ':');
    size_t prefix_len = strlen(ACCELERATOR_PREFIX);
    bool is_accelerator =
        colon != NULL &&
        (size_t) (colon - section) == prefix_len &&
        memcmp(section, ACCELERATOR_PREFIX, prefix_len) == 0 &&
        colon[1] != '\0';

    if (!is_accelerator) {
        LOG(LOG_ERR, "Unknown section/key: [%s] %s", section, name);
        state->failed = true;
        return 0;
    }

    /* Resolve (creating on first sight) the accelerator this key belongs to. */
    struct emu_accelerator *acc = NULL;
    int ret = accelerator_find_or_create(state, section, colon + 1, &acc);
    if (ret == -1) {
        state->failed = true;
        return 0;
    }

    ret = accelerator_add_value(acc, name, value);
    if (ret == -1) {
        LOG(LOG_ERR, "Invalid key/value for accelerator [%s]: '%s' = '%s'",
            section, name, value);
        state->failed = true;
        return 0;
    }

    return 1;
}

/* ========================================================================
 * Public loader
 * ======================================================================== */

int emu_config_load(const char *path, struct emu_config **config)
{
    _cleanup_(cleanup_configp)
    struct emu_config *cfg = calloc(1, sizeof(*cfg));
    PROPAGATE_ERROR_NULL_LOG(cfg, LOG_ERR, "Failed to allocate config");

    cfg->accelerators = emu_accelerator_ptr_array_init();

    if (path != NULL) {
        cfg->source_path = strdup(path);
        PROPAGATE_ERROR_NULL_LOG(cfg->source_path, LOG_ERR,
                                 "Failed to duplicate config path");

        struct config_parse_state state = {
            .config = cfg,
            .section_bdfs = str_array_init(),
            .failed = false,
        };

        int ret = ini_parse(path, parse_config_callback, &state);

        /* section_bdfs is scratch tracking state, owned by this scope. */
        str_array_free(&state.section_bdfs);

        if (ret > 0) {
            LOG(LOG_ERR, "Parse error in %s at line %d", path, ret);
            return -1;
        } else if (ret == -1) {
            LOG(LOG_ERR, "Could not open config file %s", path);
            return -1;
        } else if (ret == -2) {
            LOG(LOG_ERR, "Out of memory reading config file %s", path);
            return -1;
        }

        /* ret == 0: inih saw no error, but our callback may have flagged one. */
        if (state.failed) {
            LOG(LOG_ERR, "Invalid configuration in %s", path);
            return -1;
        }
    }

    LOG(LOG_INFO, "Loaded configuration from %s (%zu accelerator(s))",
        path != NULL ? path : "<default>", cfg->accelerators.len);

    *config = cfg;
    cfg = NULL;

    return 0;
}

/* ========================================================================
 * Running set
 * ======================================================================== */

/**
 * @brief The set of BDFs currently instantiated in the live FUSE tree.
 *
 * Backed by an owning string array; membership is a linear scan, which is ample
 * for the handful of accelerators a host carries.
 */
struct emu_running_set {
    /** @brief Normalized BDF strings of running accelerators (owning). */
    struct str_array bdfs;
};

int emu_running_set_new(struct emu_running_set **setp)
{
    struct emu_running_set *set = calloc(1, sizeof(*set));
    PROPAGATE_ERROR_NULL_LOG(set, LOG_ERR, "Failed to allocate running set");

    set->bdfs = str_array_init();

    *setp = set;

    return 0;
}

void cleanup_running_set(struct emu_running_set *set)
{
    if (set == NULL) {
        return;
    }

    str_array_free(&set->bdfs);

    free(set);
}

bool emu_running_set_contains(const struct emu_running_set *set, const char *bdf)
{
    if (set == NULL || bdf == NULL) {
        return false;
    }

    for (size_t i = 0; i < set->bdfs.len; i++) {
        if (strcmp(set->bdfs.d[i], bdf) == 0) {
            return true;
        }
    }

    return false;
}

int emu_running_set_add(struct emu_running_set *set, const char *bdf)
{
    if (emu_running_set_contains(set, bdf)) {
        return 0;
    }

    _cleanup_(cleanup_free)
    char *copy = strdup(bdf);
    PROPAGATE_ERROR_NULL_LOG(copy, LOG_ERR, "Failed to duplicate running BDF");

    int ret = str_array_push_move(&set->bdfs, &copy);
    PROPAGATE_ERROR_LOG(ret, LOG_ERR, "Failed to record running BDF '%s'", bdf);

    return 0;
}

void emu_running_set_remove(struct emu_running_set *set, const char *bdf)
{
    if (set == NULL || bdf == NULL) {
        return;
    }

    for (size_t i = 0; i < set->bdfs.len; i++) {
        if (strcmp(set->bdfs.d[i], bdf) == 0) {
            /* Free the matched string, fill the hole with the last element. */
            free(set->bdfs.d[i]);
            set->bdfs.d[i] = set->bdfs.d[set->bdfs.len - 1];
            set->bdfs.len--;
            return;
        }
    }
}

/* ========================================================================
 * Reload / merge (RESCAN data semantics)
 * ======================================================================== */

int emu_config_select_new(const struct emu_config *config,
                          const struct emu_running_set *running,
                          struct emu_accelerator_ref_array *out)
{
    if (config == NULL || out == NULL) {
        return -1;
    }

    for (size_t i = 0; i < config->accelerators.len; i++) {
        struct emu_accelerator *acc = config->accelerators.d[i];

        if (emu_running_set_contains(running, acc->bdf)) {
            LOG(LOG_INFO, "Skipping accelerator '%s': BDF already running",
                acc->bdf);
            continue;
        }

        int ret = emu_accelerator_ref_array_push(out, acc);
        PROPAGATE_ERROR_LOG(ret, LOG_ERR, "Failed to select accelerator '%s'",
                            acc->bdf);
    }

    return 0;
}
