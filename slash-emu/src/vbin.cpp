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
 * @file vbin.cpp
 * @brief Implementation of the minimal ustar VBIN unpacker (see vbin.hpp).
 *
 * Port of vbin.c to C++20: std::filesystem replaces the POSIX mkdir/readdir
 * walk; std::span replaces raw pointer+length; path-traversal hardening is
 * preserved byte-exact with the original.
 */

#include "vbin.hpp"

#include <cerrno>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>

#include <sys/stat.h>
#include <unistd.h>

namespace slash::emu {

namespace {

static constexpr size_t kTarBlock = 512u;
static constexpr char kTarLongname = 'L'; /* GNU long-name extension typeflag */

/* The standard ustar header occupies the first 512 bytes of a block. */
struct TarHeader {
    char name[100];
    char mode[8];
    char uid[8];
    char gid[8];
    char size[12];
    char mtime[12];
    char chksum[8];
    char typeflag;
    char linkname[100];
    char magic[6];
    char version[2];
    char uname[32];
    char gname[32];
    char devmajor[8];
    char devminor[8];
    char prefix[155];
    char pad[12];
};

static_assert(sizeof(TarHeader) == kTarBlock, "TarHeader must be exactly 512 bytes");

/**
 * Parse an octal field (tar stores sizes/modes as NUL/space-terminated octal).
 * Returns 0 on success, -EINVAL on a bad field.
 */
static int parseOctal(const char *field, size_t len, uint64_t &out)
{
    uint64_t v = 0;
    size_t i = 0;
    while (i < len && (field[i] == ' ' || field[i] == '\0')) {
        i++;
    }
    bool any = false;
    for (; i < len; i++) {
        char c = field[i];
        if (c == '\0' || c == ' ') {
            break;
        }
        if (c < '0' || c > '7') {
            return -EINVAL;
        }
        v = (v << 3) + static_cast<uint64_t>(c - '0');
        any = true;
    }
    if (!any) {
        out = 0;
        return 0;
    }
    out = v;
    return 0;
}

/**
 * Reject absolute paths and any ".." component (path-traversal hardening).
 * Returns 0 if safe, -EINVAL if not.
 */
static int safeMemberName(const char *name)
{
    if (name[0] == '/' || name[0] == '\0') {
        return -EINVAL;
    }
    const char *p = name;
    while (*p != '\0') {
        const char *seg = p;
        while (*p != '\0' && *p != '/') {
            p++;
        }
        size_t seglen = static_cast<size_t>(p - seg);
        if (seglen == 2 && seg[0] == '.' && seg[1] == '.') {
            return -EINVAL;
        }
        if (*p == '/') {
            p++;
        }
    }
    return 0;
}

/** Is a 512-byte block entirely zero? */
static bool blockIsZero(const uint8_t *b)
{
    for (size_t i = 0; i < kTarBlock; i++) {
        if (b[i] != 0) {
            return false;
        }
    }
    return true;
}

/**
 * Recursively search @p dir for a regular file named @p want, returning its
 * path in @p found.  Returns 0 if found, -ENOENT if not, or a negative errno
 * on an I/O error.
 */
static int findFile(const std::filesystem::path &dir, const std::string &want,
                    std::string &found)
{
    std::error_code ec;
    for (auto const &entry : std::filesystem::recursive_directory_iterator(dir, ec)) {
        if (ec) {
            /* Treat iteration errors as I/O failures. */
            return -EIO;
        }
        if (entry.is_regular_file(ec) && !ec && entry.path().filename() == want) {
            found = entry.path().string();
            return 0;
        }
    }
    if (ec) {
        return -EIO;
    }
    return -ENOENT;
}

} // namespace

VbinStatus vbinClassify(std::span<const std::byte> data)
{
    if (data.empty()) {
        return VbinStatus::Incomplete;
    }

    const auto *base = reinterpret_cast<const uint8_t *>(data.data());
    size_t len = data.size();
    size_t off = 0;

    while (true) {
        /* Need a whole header block to make any decision. */
        if (off + kTarBlock > len) {
            return VbinStatus::Incomplete;
        }

        /* The end-of-archive terminator is (at least) one zero block.  GNU tar
         * writes two; a single zero block at a 512-aligned boundary is enough to
         * recognise completion for our purposes (the unpacker stops there too).
         *
         * Agree with vbinUnpackFindSim, which requires a whole-block
         * (len % 512 == 0) archive: if a non-512-aligned tail trails the
         * terminator block, the stream is not yet a clean block-aligned archive,
         * so report Incomplete (the caller keeps accumulating) rather than a
         * Complete that unpack would then reject with -EINVAL.  This keeps
         * classify and unpack consistent. */
        if (blockIsZero(base + off)) {
            if (len % kTarBlock != 0) {
                return VbinStatus::Incomplete;
            }
            return VbinStatus::Complete;
        }

        /* A real header must carry the ustar magic; anything else is garbage and
         * no amount of further bytes will fix it. */
        const auto *h = reinterpret_cast<const TarHeader *>(base + off);
        if (std::memcmp(h->magic, "ustar", 5) != 0) {
            return VbinStatus::Invalid;
        }

        uint64_t fsize = 0;
        if (parseOctal(h->size, sizeof(h->size), fsize) != 0) {
            return VbinStatus::Invalid;
        }

        size_t data_blocks = static_cast<size_t>((fsize + kTarBlock - 1) / kTarBlock);
        /* Guard against multiplication overflow on a hostile size field. */
        if (data_blocks > (SIZE_MAX - kTarBlock) / kTarBlock) {
            return VbinStatus::Invalid;
        }
        size_t advance = kTarBlock + data_blocks * kTarBlock;
        if (off + advance < off) {
            return VbinStatus::Invalid; /* overflow */
        }
        if (off + advance > len) {
            return VbinStatus::Incomplete; /* member body not fully arrived yet */
        }
        off += advance;
    }
}

int vbinUnpackFindSim(std::span<const std::byte> vbin, const std::string &dest_dir,
                      std::string &exec_out)
{
    if (dest_dir.empty()) {
        return -EINVAL;
    }
    size_t len = vbin.size();
    if (len < kTarBlock || (len % kTarBlock) != 0) {
        return -EINVAL; /* not a block-aligned tar */
    }

    const auto *base = reinterpret_cast<const uint8_t *>(vbin.data());
    size_t off = 0;

    /* GNU long-name carry-over: when a 'L' record precedes an entry, its payload
     * is the true name of the following entry. */
    std::string longname;
    bool have_longname = false;

    while (off + kTarBlock <= len) {
        /* Two consecutive zero blocks mark end-of-archive; a single all-zero
         * header also terminates parsing for our purposes. */
        if (blockIsZero(base + off)) {
            break;
        }

        const auto *h = reinterpret_cast<const TarHeader *>(base + off);

        uint64_t fsize = 0;
        if (parseOctal(h->size, sizeof(h->size), fsize) != 0) {
            return -EINVAL;
        }
        uint64_t mode = 0;
        (void) parseOctal(h->mode, sizeof(h->mode), mode);

        size_t data_off = off + kTarBlock;
        size_t data_blocks = static_cast<size_t>((fsize + kTarBlock - 1) / kTarBlock);
        if (data_off + data_blocks * kTarBlock > len) {
            return -EINVAL; /* truncated archive */
        }

        if (h->typeflag == kTarLongname) {
            if (fsize == 0 || fsize > 4095u) {
                return -EINVAL;
            }
            longname.assign(reinterpret_cast<const char *>(base + data_off),
                            static_cast<size_t>(fsize));
            /* Strip a trailing NUL that GNU tar includes in the payload length. */
            if (!longname.empty() && longname.back() == '\0') {
                longname.pop_back();
            }
            have_longname = true;
            off = data_off + data_blocks * kTarBlock;
            continue;
        }

        /* Determine the member name (long-name overrides the header name). */
        std::string name;
        if (have_longname) {
            name = std::move(longname);
            have_longname = false;
        } else {
            /* name field is at most 100 bytes, not necessarily NUL-terminated. */
            size_t nl = ::strnlen(h->name, sizeof(h->name));
            name.assign(h->name, nl);
        }

        if (name.empty()) {
            off = data_off + data_blocks * kTarBlock;
            continue;
        }
        if (safeMemberName(name.c_str()) != 0) {
            return -EINVAL;
        }

        std::filesystem::path dest = std::filesystem::path(dest_dir) / name;

        bool is_dir = (h->typeflag == '5') ||
                      (!name.empty() && name.back() == '/');

        std::error_code ec;
        if (is_dir) {
            std::filesystem::create_directories(dest, ec);
            if (ec) {
                return -EIO;
            }
        } else if (h->typeflag == '0' || h->typeflag == '\0') {
            /* Regular file: create parent directories first. */
            std::filesystem::create_directories(dest.parent_path(), ec);
            if (ec) {
                return -EIO;
            }

            /* Preserve at least the user-exec bit so vpp_sim is runnable; mask to
             * a sane 0700/0600 range (we run unsandboxed under our own uid). */
            mode_t fmode = (mode & 0100u) ? 0700 : 0600;

            {
                std::ofstream ofs(dest, std::ios::binary | std::ios::trunc);
                if (!ofs) {
                    return -errno;
                }
                if (fsize > 0) {
                    ofs.write(reinterpret_cast<const char *>(base + data_off),
                              static_cast<std::streamsize>(fsize));
                    if (!ofs) {
                        return -EIO;
                    }
                }
            } /* ofs closes here */

            if (::chmod(dest.c_str(), fmode) != 0) {
                return -errno;
            }
        }
        /* Other typeflags (symlink, etc.) are ignored for the SIM VBIN. */

        off = data_off + data_blocks * kTarBlock;
    }

    /* Locate the vpp_sim executable anywhere in the unpacked tree. */
    int rc = findFile(std::filesystem::path(dest_dir), "vpp_sim", exec_out);
    if (rc != 0) {
        return rc;
    }
    if (::access(exec_out.c_str(), X_OK) != 0) {
        return -EACCES;
    }
    return 0;
}

} // namespace slash::emu
