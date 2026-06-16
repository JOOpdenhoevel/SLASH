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
 * @file utils.h
 * @brief Utility macros and helpers used throughout the slash-emu daemon.
 *
 * This header is a focused port of the vrtd @c utils.h, carrying over the
 * pieces the system-emulation daemon actually needs.  See vrt/vrtd/STYLE.md
 * for the authoritative rationale behind these conventions.
 *
 * @section error_prop Error-propagation macros
 *
 * The PROPAGATE_ERROR family of macros provides concise, consistent error
 * handling for functions that return -1 (POSIX), NULL (allocation), or
 * negative errno (systemd).  Each macro evaluates an expression, checks
 * the result against a type-specific failure condition, and if the check
 * fails, optionally logs a message via sd_journal and returns -1 from the
 * calling function.
 *
 * Variants:
 *   - @c PROPAGATE_ERROR(expr)           -- fails on -1, no log.
 *   - @c PROPAGATE_ERROR_NULL(expr)      -- fails on NULL, no log.
 *   - @c PROPAGATE_ERROR_SD(expr)        -- fails on < 0 (systemd convention), no log.
 *   - @c PROPAGATE_ERROR_LOG(...)        -- fails on -1, logs a formatted message.
 *   - @c PROPAGATE_ERROR_NULL_LOG(...)   -- fails on NULL, logs a formatted message.
 *   - @c PROPAGATE_ERROR_STDC_LOG(...)   -- fails on -1, logs with appended strerror (%m).
 *   - @c PROPAGATE_ERROR_NULL_STDC_LOG(...)  -- fails on NULL, logs with appended strerror.
 *   - @c PROPAGATE_ERROR_SD_LOG(...)     -- fails on < 0, logs with systemd error string.
 *
 * @section cleanup Cleanup attribute helpers
 *
 * The @c _cleanup_(fn) macro wraps GCC/Clang's @c __attribute__((cleanup))
 * to automatically call @p fn when the annotated variable goes out of scope.
 * Combined with @c cleanup_free this enables RAII-style resource management
 * in C.
 *
 * @section misc Miscellaneous
 *
 * - @c LOG -- shorthand for sd_journal_print (cast to void to suppress
 *   unused-result warnings).
 * - @c NODISCARD -- portable warn_unused_result attribute.
 * - @c bit_ceil -- type-generic smallest power-of-two >= n.
 * - @c max / @c min -- type-safe comparisons using GCC statement expressions.
 * - @c SIZEOF_ARRAY -- element count for statically-sized arrays.
 * - @c unlikely / @c likely -- branch-prediction hints.
 */

#ifndef SLASH_EMU_UTILS_H
#define SLASH_EMU_UTILS_H

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

#include <systemd/sd-journal.h>

/** @brief Shorthand for sd_journal_print, cast to void to suppress unused-result warnings. */
#define LOG (void) sd_journal_print

#if defined(__has_include)
#  if __has_include(<stdbit.h>)
#    include <stdbit.h>
#    define HAVE_STDBIT 1
#  endif
#endif

/** @brief Portable attribute to warn if a function's return value is discarded. */
#if defined(__STDC_VERSION__) && __STDC_VERSION__ >= 202311L
    #define NODISCARD [[nodiscard]]
#elif defined(__GNUC__) || defined(__clang__)
    #define NODISCARD __attribute__((warn_unused_result))
#else
    #define NODISCARD
#endif

/* ---- type-specific helpers ---- */
#if HAVE_STDBIT
/**
 * @brief Return the smallest power of two >= @p n (32-bit).
 * @param n Input value.
 * @return Smallest power of two >= n, or 0 if not representable.
 */
static inline uint32_t bit_ceil_u32(uint32_t n) {
    return stdc_bit_ceil((unsigned int)(n));
}
/**
 * @brief Return the smallest power of two >= @p n (64-bit).
 * @param n Input value.
 * @return Smallest power of two >= n, or 0 if not representable.
 */
static inline uint64_t bit_ceil_u64(uint64_t n) {
   return stdc_bit_ceil((unsigned long long)(n));
}
#else
/**
 * @brief Return the smallest power of two >= @p n (32-bit, GCC/Clang fallback).
 * @param n Input value.
 * @return Smallest power of two >= n, or 0 if not representable.
 */
static inline uint32_t bit_ceil_u32(uint32_t n) {
    if (n == 0) return 1u;
    if (n > 0x80000000u) return 0u;                 // not representable
    return 1u << (32 - __builtin_clz(n - 1));       // GCC/Clang
}
/**
 * @brief Return the smallest power of two >= @p n (64-bit, GCC/Clang fallback).
 * @param n Input value.
 * @return Smallest power of two >= n, or 0 if not representable.
 */
static inline uint64_t bit_ceil_u64(uint64_t n) {
    if (n == 0) return 1ull;
    if (n > 0x8000000000000000ull) return 0ull;     // not representable
    return 1ull << (64 - __builtin_clzll(n - 1));
}
#endif

/**
 * @brief Type-generic smallest power-of-two >= n.
 *
 * Dispatches to bit_ceil_u32 or bit_ceil_u64 based on the argument type.
 */
/* ---- generic front-end ---- */
#ifdef __cplusplus
#define bit_ceil(n) \
    (sizeof(n) <= sizeof(uint32_t) ? bit_ceil_u32(static_cast<uint32_t>(n)) \
                                   : bit_ceil_u64(static_cast<uint64_t>(n)))
#else
#define bit_ceil(n) _Generic((n), \
    uint32_t:               bit_ceil_u32, \
    uint64_t:               bit_ceil_u64  \
)(n)
#endif

/**
 * @brief Type-safe maximum of two values (GCC statement expression).
 * @param a First value.
 * @param b Second value.
 * @return The larger of @p a and @p b.
 */
#define max(a,b) \
   ({ __auto_type _a = (a); \
      __auto_type _b = (b); \
     _a > _b ? _a : _b; })

/**
 * @brief Type-safe minimum of two values (GCC statement expression).
 * @param a First value.
 * @param b Second value.
 * @return The smaller of @p a and @p b.
 */
#define min(a,b) \
   ({ __auto_type _a = (a); \
      __auto_type _b = (b); \
     _a < _b ? _a : _b; })

/** @brief Number of elements in a statically-sized array. */
#define SIZEOF_ARRAY(X) (sizeof(X) / sizeof(X[0]))

/** @brief No-operation statement (void expression). */
#define NOP() ((void) 0)

/** @brief Branch-prediction hint: expression is expected to be false. */
#ifndef unlikely
#define unlikely(x) __builtin_expect(!!(x), 0)
#endif

/** @brief Branch-prediction hint: expression is expected to be true. */
#ifndef likely
#define likely(x) __builtin_expect(!!(x), 1)
#endif

/**
 * @brief Shorthand for __attribute__((cleanup(FOO))).
 *
 * Automatically calls @p FOO on the annotated variable when it goes out of
 * scope, enabling RAII-style resource management in C.
 *
 * @param FOO Cleanup function taking a pointer to the variable's type.
 */
#ifndef _cleanup_
#define _cleanup_(FOO) __attribute__((cleanup(FOO)))
#endif

/*
 * Internal error-propagation building blocks.
 * Do not use directly -- use the PROPAGATE_ERROR_* macros below.
 */
#define _PROPAGATE_ERROR_INTERNAL_NOLOG(RET, CMP, JUMP)      \
    ({                                                       \
        __auto_type _ret = RET;                              \
        if (CMP) {                                           \
            JUMP;                                            \
        }                                                    \
    })

#define _PROPAGATE_ERROR_INTERNAL_LOG(RET, CMP, JUMP, LOGLEVEL, FMT, ...) \
    ({                                                                    \
        __auto_type _ret = RET;                                          \
        if (CMP) {                                                        \
            sd_journal_print(LOGLEVEL, FMT, ##__VA_ARGS__);               \
            JUMP;                                                         \
        }                                                                \
    })

/** @brief Propagate error: return -1 if @p RET == -1 (POSIX convention). No logging. */
#define PROPAGATE_ERROR(RET) \
    _PROPAGATE_ERROR_INTERNAL_NOLOG(RET, (_ret == -1), return -1)
/** @brief Propagate error: return -1 if @p RET == NULL (allocation failure). No logging. */
#define PROPAGATE_ERROR_NULL(RET) \
    _PROPAGATE_ERROR_INTERNAL_NOLOG(RET, (_ret == NULL), return -1)
/** @brief Propagate error: return -1 if @p RET < 0 (systemd/negative-errno convention). No logging. */
#define PROPAGATE_ERROR_SD(RET) \
    _PROPAGATE_ERROR_INTERNAL_NOLOG(RET, (_ret < 0), return -1)

/** @brief Propagate error with logging: return -1 if @p RET == -1. */
#define PROPAGATE_ERROR_LOG(RET, LOGLEVEL, FMT, ...) \
    _PROPAGATE_ERROR_INTERNAL_LOG(RET, (_ret == -1), return -1, LOGLEVEL, FMT, ##__VA_ARGS__)
/** @brief Propagate error with logging: return -1 if @p RET == NULL. */
#define PROPAGATE_ERROR_NULL_LOG(RET, LOGLEVEL, FMT, ...) \
    _PROPAGATE_ERROR_INTERNAL_LOG(RET, (_ret == NULL), return -1, LOGLEVEL, FMT, ##__VA_ARGS__)
/** @brief Propagate error with strerror logging: return -1 if @p RET == -1. Appends ": <errno message>". */
#define PROPAGATE_ERROR_STDC_LOG(RET, LOGLEVEL, FMT, ...) \
    _PROPAGATE_ERROR_INTERNAL_LOG(RET, (_ret == -1), return -1, LOGLEVEL, FMT ": %m", ##__VA_ARGS__)
/** @brief Propagate error with strerror logging: return -1 if @p RET == NULL. Appends ": <errno message>". */
#define PROPAGATE_ERROR_NULL_STDC_LOG(RET, LOGLEVEL, FMT, ...) \
    _PROPAGATE_ERROR_INTERNAL_LOG(RET, (_ret == NULL), return -1, LOGLEVEL, FMT ": %m", ##__VA_ARGS__)
/** @brief Propagate error with systemd error logging: return -1 if @p RET < 0. Appends ": <sd error>". */
#define PROPAGATE_ERROR_SD_LOG(RET, LOGLEVEL, FMT, ...) \
    _PROPAGATE_ERROR_INTERNAL_LOG(RET, (_ret < 0), return -1, LOGLEVEL, FMT ": %s", ##__VA_ARGS__, strerrordesc_np(-_ret))

/**
 * @brief Generic cleanup function for heap-allocated pointers.
 *
 * Suitable for use with @c _cleanup_. Frees the pointer and sets it to NULL.
 *
 * @param p Address of a @c void* variable to free.
 */
static inline
void cleanup_free(void *p) {
    void **ptr = (void**)p;
    free(*ptr);

    *ptr = NULL;
}

#endif // SLASH_EMU_UTILS_H
