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
 * @file config.hpp
 * @brief Configuration data model and persistent accelerator model (C++20).
 *
 * slash-emu reads an INI-style configuration file (parsed via libinih's C API,
 * mirroring vrtd's config layer) describing the set of system-emulated SLASH
 * accelerators the daemon should expose.  Per the architecture the configurable
 * surface is deliberately small:
 *
 *   - the accelerator @em BDF (the per-device folder name, board-level
 *     @c DDDD:BB:DD form without a function suffix);
 *   - reserved schema space for future per-accelerator @em network configuration
 *     (parsed for forward compatibility, otherwise unused).
 *
 * On-disk format is one section per accelerator:
 *
 * @code{.ini}
 *   [accelerator:0000:61:00]
 *   net-mac  = 02:00:00:00:00:01
 *   net-ip   = 10.0.0.1
 *   net-port = 4791
 * @endcode
 *
 * @warning Each @c [accelerator:<bdf>] section MUST carry at least one key.  The
 *          distro libinih invokes its handler only per key/value pair, so a
 *          keyless section produces no callback and is silently ignored.  Add a
 *          reserved @c net-* key to every accelerator; the shipped sample does so.
 *
 * @section reload Reload / merge semantics (RESCAN)
 *
 * RESCAN reloads the config and instantiates every configured accelerator whose
 * BDF does not collide with an already-running one.  @ref Config::selectNew
 * implements exactly that set computation; materialising the selection into FUSE
 * nodes is the filesystem layer's job.
 *
 * @section errors Error model
 *
 * Config load is control-plane: @ref Config::load throws @ref SystemError on a
 * malformed file (bad/duplicate BDF, unknown section/key); @c main catches it.
 */

#ifndef SLASH_EMU_CONFIG_HPP
#define SLASH_EMU_CONFIG_HPP

#include <optional>
#include <string>
#include <vector>

namespace slash::emu {

/**
 * @brief Length of a normalized board-level BDF string, including the NUL.
 *
 * Board-level BDFs are @c "DDDD:BB:DD" -> 11 bytes; sized to a round 16.
 */
inline constexpr size_t kBdfLen = 16;

/**
 * @brief Reserved per-accelerator network configuration (forward-compatible).
 *
 * Parsed for forward compatibility but not acted upon yet.  All fields optional;
 * an absent key leaves the corresponding optional empty.
 */
struct NetConfig {
    std::optional<std::string> mac;  /**< Reserved: MAC address string. */
    std::optional<std::string> ip;   /**< Reserved: IP address string. */
    std::optional<std::string> port; /**< Reserved: UDP/TCP port string. */
};

/**
 * @brief A single configured system-emulated accelerator.
 *
 * Identified by its normalized board-level BDF (the per-device folder name).
 */
struct Accelerator {
    std::string bdf;  /**< Normalized board-level BDF "DDDD:BB:DD". */
    NetConfig net;    /**< Reserved network configuration (inert today). */
};

/**
 * @brief Normalize and validate a board-level BDF string.
 *
 * Accepts canonical @c "DDDD:BB:DD" or the short @c "BB:DD" form (expanded to
 * domain @c 0000).  A trailing @c ".F" function suffix is rejected: accelerator
 * folders are named at board level.  All hex fields are validated and lower-cased.
 *
 * @param input The raw BDF string.
 * @return The normalized BDF, or @c std::nullopt on invalid format.
 */
std::optional<std::string> normalizeBdf(const std::string &input);

/**
 * @brief Top-level slash-emu configuration container.
 *
 * Owns the configured accelerator set and remembers the path it was loaded from.
 */
class Config {
public:
    /**
     * @brief Load and parse the configuration from @p path.
     *
     * A @c std::nullopt @p path yields an empty (zero-accelerator) configuration
     * without touching the filesystem (the built-in default when @c --config is
     * not supplied); an empty file likewise yields an empty configuration.
     * Validates every accelerator BDF and rejects malformed-BDF, duplicate-BDF,
     * and unknown-section/key configurations.
     *
     * @param path Path to the configuration file, or @c std::nullopt for the
     *             built-in empty default.
     * @return The loaded configuration.
     * @throws SystemError on a parse/validation error (logged via sd_journal).
     */
    static Config load(const std::optional<std::string> &path);

    /** @brief The path the config was loaded from, or empty for the default. */
    const std::optional<std::string> &sourcePath() const { return source_path_; }

    /** @brief All configured accelerators. */
    const std::vector<Accelerator> &accelerators() const { return accelerators_; }

    /**
     * @brief Look up a configured accelerator by normalized BDF.
     * @return Borrowed pointer to the match, or nullptr if none.
     */
    const Accelerator *find(const std::string &bdf) const;

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
    selectNew(const std::vector<std::string> &running) const;

private:
    std::optional<std::string> source_path_;
    std::vector<Accelerator> accelerators_;
};

} // namespace slash::emu

#endif // SLASH_EMU_CONFIG_HPP
