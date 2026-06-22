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
 * @file config.cpp
 * @brief INI configuration loader and persistent accelerator model for slash-emu (C++20).
 *
 * This file implements the configuration subsystem behind @c config.hpp.  It uses
 * an INI-style configuration file (parsed via libinih's C API, mirroring vrtd's
 * config layer) with one section per emulated accelerator:
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
 * normalization) and malformed BDFs are rejected.
 *
 * Config load is control-plane: @ref slash::emu::Config::load throws
 * @ref slash::emu::SystemError on any parse/validation error.
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
 */

#include "config.hpp"
#include "utils.hpp"

#include <cassert>
#include <cctype>
#include <cstring>
#include <string>
#include <vector>

#include <ini.h>

/* On Ubuntu's libinih the callback takes (user, section, name, value). */
static_assert(INI_HANDLER_LINENO == 0, "slash-emu does not support INI_HANDLER_LINENO = 1");

namespace slash::emu {

namespace {

/** @brief INI section prefix introducing an accelerator: "[accelerator:<BDF>]". */
static constexpr const char kAcceleratorPrefix[] = "accelerator";
static constexpr size_t kAcceleratorPrefixLen = sizeof(kAcceleratorPrefix) - 1;

/* ========================================================================
 * BDF normalization helpers
 * ======================================================================== */

/**
 * @brief Validate that @p s[0..n) are all hex digits.
 *
 * @param s Pointer to the first character.
 * @param n Required number of hex digits.
 * @return true if all n characters are hex digits, false otherwise.
 */
bool checkHexRun(const char *s, size_t n)
{
    for (size_t i = 0; i < n; i++) {
        if (!std::isxdigit(static_cast<unsigned char>(s[i]))) {
            return false;
        }
    }
    return true;
}

/* ========================================================================
 * Parse state
 * ======================================================================== */

/**
 * @brief Transient state carried through a single Config::load() invocation.
 *
 * Stock libinih reports no new-section callback, so the parser does a
 * find-or-create by normalized BDF on every key.  To still reject a duplicate
 * BDF that is spelled differently across two sections, @c sectionBdfs records,
 * in lock-step with @c accelerators, the @em raw section string that first
 * created each accelerator.  A later key whose raw section string differs but
 * whose normalized BDF already exists is a duplicate and is rejected; a key
 * whose raw section string matches is simply the same section continuing.
 */
struct ParseState {
    /** @brief The accelerators being populated (owned by this state during parse). */
    std::vector<Accelerator> *accelerators;
    /** @brief Raw section strings, parallel to @c accelerators: entry @c i is the
     *         @c [accelerator:<raw>] header that created accelerator @c i. */
    std::vector<std::string> sectionBdfs;
    /** @brief True if the callback encountered a hard error. */
    bool failed{false};
    /** @brief Human-readable error message on failure. */
    std::string errorMsg;
};

/**
 * @brief Apply a single key/value pair to an accelerator.
 *
 * Recognizes the reserved (currently inert) network keys.  Unknown keys are a
 * hard error so typos in the config are caught rather than silently ignored.
 *
 * @param acc   The accelerator to modify.
 * @param name  Key name.
 * @param value Value string.
 * @return true on success, false on unknown key.
 */
bool acceleratorAddValue(Accelerator &acc, const char *name, const char *value)
{
    if (std::strcmp(name, "net-mac") == 0) {
        acc.net.mac = value;
        return true;
    } else if (std::strcmp(name, "net-ip") == 0) {
        acc.net.ip = value;
        return true;
    } else if (std::strcmp(name, "net-port") == 0) {
        acc.net.port = value;
        return true;
    }

    LOG(LOG_ERR, "Unknown accelerator key: '%s'", name);
    return false;
}

/**
 * @brief Resolve the accelerator a key belongs to, creating it on first sight.
 *
 * Normalizes @p bdfRaw (the part after @c "accelerator:" in the section header)
 * and looks it up in the accelerators vector:
 *
 *   - If no accelerator with that normalized BDF exists, a fresh one is created
 *     and appended, with its raw section string recorded in @c state.sectionBdfs.
 *   - If one exists and was created from the @em same raw section string, this is
 *     the same section continuing -- the existing accelerator is returned.
 *   - If one exists but was created from a @em different raw section string, the
 *     same BDF has been declared twice (spelled differently) -- a duplicate error.
 *
 * @param      state    Parse state.
 * @param      section  The full raw section string ("accelerator:<raw>").
 * @param      bdfRaw   The raw BDF (the part after the colon).
 * @param[out] out      Receives a pointer to the resolved accelerator on success.
 * @return true on success, false on invalid/duplicate BDF.
 */
bool acceleratorFindOrCreate(ParseState &state, const std::string &section,
                             const char *bdfRaw, Accelerator **out)
{
    std::optional<std::string> bdfOpt = normalizeBdf(bdfRaw);
    if (!bdfOpt) {
        return false;
    }
    const std::string &bdf = *bdfOpt;

    /* Already created?  Same raw section => continue it; different => duplicate. */
    for (size_t i = 0; i < state.accelerators->size(); i++) {
        if ((*state.accelerators)[i].bdf != bdf) {
            continue;
        }

        if (state.sectionBdfs[i] == section) {
            *out = &(*state.accelerators)[i];
            return true;
        }

        LOG(LOG_ERR, "Duplicate accelerator BDF '%s'", bdf.c_str());
        return false;
    }

    /* First sight of this BDF: create and record its raw section string. */
    state.accelerators->push_back(Accelerator{bdf, {}});
    state.sectionBdfs.push_back(section);
    *out = &state.accelerators->back();
    return true;
}

/**
 * @brief inih callback: dispatch each INI key/value to the right handler.
 *
 * Stock libinih invokes this once per key/value pair only -- never on a bare
 * section header, so a section is observed exactly when its first key arrives.
 * Every call therefore:
 *   1. validates that @p section is @c "accelerator:<bdf>";
 *   2. find-or-creates the accelerator for that (normalized) BDF; and
 *   3. applies the key to that accelerator.
 * A non-accelerator section, or a key in the top-level (empty) section, is a
 * hard error.  Follows the inih convention: returns 1 on success, 0 on error.
 *
 * @param user    Opaque pointer to ParseState.
 * @param section INI section name (e.g. "accelerator:0000:61:00").
 * @param name    Key name within the section.
 * @param value   Value string associated with the key.
 * @return 1 on success, 0 on error (per inih convention).
 */
int parseConfigCallback(void *user, const char *section, const char *name, const char *value)
{
    auto *state = static_cast<ParseState *>(user);

    /* The section must be exactly "accelerator:<bdf>" where bdf is non-empty. */
    const char *colon = std::strchr(section, ':');
    bool isAccelerator =
        colon != nullptr &&
        static_cast<size_t>(colon - section) == kAcceleratorPrefixLen &&
        std::memcmp(section, kAcceleratorPrefix, kAcceleratorPrefixLen) == 0 &&
        colon[1] != '\0';

    if (!isAccelerator) {
        LOG(LOG_ERR, "Unknown section/key: [%s] %s", section, name);
        state->failed = true;
        state->errorMsg = std::string("Unknown section: [") + section + "]";
        return 0;
    }

    /* Resolve (creating on first sight) the accelerator this key belongs to. */
    Accelerator *acc = nullptr;
    if (!acceleratorFindOrCreate(*state, section, colon + 1, &acc)) {
        state->failed = true;
        state->errorMsg = std::string("Invalid or duplicate BDF in section [") + section + "]";
        return 0;
    }

    if (!acceleratorAddValue(*acc, name, value)) {
        LOG(LOG_ERR, "Invalid key/value for accelerator [%s]: '%s' = '%s'",
            section, name, value);
        state->failed = true;
        state->errorMsg = std::string("Unknown key '") + name + "' in section [" + section + "]";
        return 0;
    }

    return 1;
}

} // namespace

/* ========================================================================
 * Public BDF normalization
 * ======================================================================== */

/**
 * @brief Normalize and validate a board-level BDF string.
 *
 * Accepts canonical @c "DDDD:BB:DD" or the short @c "BB:DD" form (expanded to
 * domain @c 0000).  A trailing @c ".F" function suffix is rejected: accelerator
 * folders are named at board level.  All hex fields are validated and
 * lower-cased.
 *
 * @param input The raw BDF string.
 * @return The normalized BDF, or @c std::nullopt on invalid format.
 */
std::optional<std::string> normalizeBdf(const std::string &input)
{
    /* Reject a function suffix outright: accelerator folders are board-level. */
    if (input.find('.') != std::string::npos) {
        LOG(LOG_ERR, "BDF '%s' must not include a function suffix", input.c_str());
        return std::nullopt;
    }

    /* Count colons to distinguish "BB:DD" from "DDDD:BB:DD". */
    size_t colons = 0;
    for (char c : input) {
        if (c == ':') {
            colons++;
        }
    }

    const char *domain;
    size_t domainLen;
    const char *busDev;
    const char *inputCstr = input.c_str();

    if (colons == 1) {
        /* Short form "BB:DD" -> implicit domain 0000. */
        domain = "0000";
        domainLen = 4;
        busDev = inputCstr;
    } else if (colons == 2) {
        /* Full form "DDDD:BB:DD". */
        const char *first = std::strchr(inputCstr, ':');
        domain = inputCstr;
        domainLen = static_cast<size_t>(first - inputCstr);
        busDev = first + 1;
    } else {
        LOG(LOG_ERR, "Invalid BDF format: '%s'", inputCstr);
        return std::nullopt;
    }

    /* Validate domain (only present explicitly in the full form). */
    if (colons == 2) {
        if (domainLen != 4 || !checkHexRun(domain, 4)) {
            LOG(LOG_ERR, "Invalid BDF domain in '%s'", inputCstr);
            return std::nullopt;
        }
    }

    /* busDev must be exactly "BB:DD": 2 hex, colon, 2 hex -> length 5. */
    if (std::strlen(busDev) != 5 || busDev[2] != ':' ||
        !checkHexRun(busDev, 2) || !checkHexRun(busDev + 3, 2)) {
        LOG(LOG_ERR, "Invalid BDF bus/device in '%s'", inputCstr);
        return std::nullopt;
    }

    /* Build the canonical "DDDD:BB:DD" form. */
    char buf[kBdfLen];
    int ret = std::snprintf(buf, sizeof(buf), "%.*s:%c%c:%c%c",
                            static_cast<int>(domainLen), domain,
                            busDev[0], busDev[1], busDev[3], busDev[4]);
    if (ret < 0 || static_cast<size_t>(ret) >= sizeof(buf)) {
        LOG(LOG_ERR, "BDF buffer too small for '%s'", inputCstr);
        return std::nullopt;
    }

    /* Canonicalize to lower-case hex. */
    for (char *p = buf; *p != '\0'; p++) {
        *p = static_cast<char>(std::tolower(static_cast<unsigned char>(*p)));
    }

    return std::string(buf);
}

/* ========================================================================
 * Config public API
 * ======================================================================== */

/**
 * @brief Load and parse the configuration from @p path.
 *
 * A @c std::nullopt @p path yields an empty (zero-accelerator) configuration
 * without touching the filesystem.  Validates every accelerator BDF and rejects
 * malformed-BDF, duplicate-BDF, and unknown-section/key configurations.
 *
 * @param path Path to the configuration file, or @c std::nullopt for the
 *             built-in empty default.
 * @return The loaded configuration.
 * @throws SystemError on a parse/validation error (logged via sd_journal).
 */
Config Config::load(const std::optional<std::string> &path)
{
    Config cfg;

    if (!path) {
        LOG(LOG_INFO, "Loaded configuration from <default> (0 accelerator(s))");
        return cfg;
    }

    cfg.source_path_ = path;

    ParseState state;
    state.accelerators = &cfg.accelerators_;

    int ret = ini_parse(path->c_str(), parseConfigCallback, &state);

    if (ret > 0) {
        LOG(LOG_ERR, "Parse error in %s at line %d", path->c_str(), ret);
        throw SystemError(EINVAL, "Parse error in config file: " + *path);
    } else if (ret == -1) {
        LOG(LOG_ERR, "Could not open config file %s", path->c_str());
        throw SystemError(ENOENT, "Could not open config file: " + *path);
    } else if (ret == -2) {
        LOG(LOG_ERR, "Out of memory reading config file %s", path->c_str());
        throw SystemError(ENOMEM, "Out of memory reading config file: " + *path);
    }

    /* ret == 0: inih saw no error, but our callback may have flagged one. */
    if (state.failed) {
        LOG(LOG_ERR, "Invalid configuration in %s", path->c_str());
        throw SystemError(EINVAL, "Invalid configuration in " + *path + ": " + state.errorMsg);
    }

    LOG(LOG_INFO, "Loaded configuration from %s (%zu accelerator(s))",
        path->c_str(), cfg.accelerators_.size());

    return cfg;
}

/**
 * @brief Look up a configured accelerator by normalized BDF.
 * @return Borrowed pointer to the match, or nullptr if none.
 */
const Accelerator *Config::find(const std::string &bdf) const
{
    for (const Accelerator &acc : accelerators_) {
        if (acc.bdf == bdf) {
            return &acc;
        }
    }
    return nullptr;
}

/**
 * @brief Compute the accelerators to instantiate on a RESCAN-style reload.
 *
 * Returns every accelerator whose BDF is @em not in @p running; colliding
 * BDFs are skipped.  The result holds borrowed pointers into this config.
 *
 * @param running The set of already-running normalized BDFs.
 * @return Borrowed pointers to the selected accelerators.
 */
std::vector<const Accelerator *>
Config::selectNew(const std::vector<std::string> &running) const
{
    std::vector<const Accelerator *> result;

    for (const Accelerator &acc : accelerators_) {
        bool isRunning = false;
        for (const std::string &bdf : running) {
            if (acc.bdf == bdf) {
                isRunning = true;
                break;
            }
        }

        if (isRunning) {
            LOG(LOG_INFO, "Skipping accelerator '%s': BDF already running", acc.bdf.c_str());
            continue;
        }

        result.push_back(&acc);
    }

    return result;
}

} // namespace slash::emu
