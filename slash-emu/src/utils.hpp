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
 * @file utils.hpp
 * @brief Small shared utilities for the slash-emu daemon (C++20).
 *
 * The C daemon carried a large @c utils.h of macros (the @c PROPAGATE_ERROR
 * family, @c _cleanup_, the @c array.h generator).  The C++20 port deletes those
 * in favour of language facilities: RAII (destructors, @c std::unique_ptr /
 * @c std::shared_ptr / @c std::vector / @c std::string), @c std::scoped_lock,
 * and exceptions in the control/startup plane.  What remains here is genuinely
 * shared and small:
 *
 *   - @ref LOG — the journald logging shim (unchanged spelling, so call sites
 *     read the same).
 *   - @ref slash::emu::SystemError — a control-plane exception that carries an
 *     errno, plus @ref slash::emu::throwErrno to raise it from a failed libc /
 *     libsystemd call.
 *
 * @section errno_boundary Error model
 *
 * Two conventions coexist by design:
 *
 *   - The @b data-plane boundary (FUSE op dispatch in @ref node.hpp / @ref fs.hpp
 *     and the model-protocol seams in @ref model_client.hpp) keeps returning a
 *     @b negative @b errno (or a byte count), because the FUSE layer and the
 *     bar/qdma backends branch on the exact rc to choose shadow/store fallback.
 *     These paths do @em not throw.
 *   - The @b control/startup plane (CLI parsing, config load, mount bring-up,
 *     bridge spawn) uses @b exceptions (@ref SystemError); @c main catches them
 *     and exits non-zero.
 */

#ifndef SLASH_EMU_UTILS_HPP
#define SLASH_EMU_UTILS_HPP

#include <cstring>
#include <stdexcept>
#include <string>
#include <system_error>

#include <systemd/sd-journal.h>

/**
 * @brief Shorthand for @c sd_journal_print, cast to void to discard its result.
 *
 * Spelled exactly as the C daemon's macro so log call sites are unchanged, e.g.
 * @code LOG(LOG_ERR, "Failed to mount %s", path); @endcode
 */
#define LOG (void) sd_journal_print

namespace slash::emu {

/**
 * @brief A control-plane exception carrying a POSIX @c errno.
 *
 * Thrown by the startup/config/bring-up code when a libc or libsystemd call
 * fails.  Derives from @c std::system_error so @c what() includes the standard
 * error string and @c code().value() recovers the errno for the exit path.
 */
class SystemError : public std::system_error {
public:
    /**
     * @brief Construct from an errno and a context message.
     * @param err     POSIX errno value (e.g. from @c errno, or @c -rc for an
     *                sd_* call that returns a negative errno).
     * @param context Human-readable context prefix ("Failed to ...").
     */
    SystemError(int err, const std::string &context)
        : std::system_error(err, std::generic_category(), context) {}
};

/**
 * @brief Raise a @ref SystemError for the current @c errno.
 * @param context Context message describing the failed operation.
 * @throws SystemError always.
 */
[[noreturn]] inline void throwErrno(const std::string &context)
{
    throw SystemError(errno, context);
}

/**
 * @brief Raise a @ref SystemError for an explicit errno value.
 * @param err     POSIX errno value (pass @c -rc for negative-errno sd_* returns).
 * @param context Context message describing the failed operation.
 * @throws SystemError always.
 */
[[noreturn]] inline void throwErrno(int err, const std::string &context)
{
    throw SystemError(err, context);
}

} // namespace slash::emu

#endif // SLASH_EMU_UTILS_HPP
