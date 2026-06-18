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
 * @file vbin.c
 * @brief Implementation of the minimal ustar VBIN unpacker (see vbin.h).
 */

#define _GNU_SOURCE

#include "vbin.h"

#include <dirent.h>
#include <errno.h>
#include <limits.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include "utils.h"

#define TAR_BLOCK 512u
#define TAR_LONGNAME 'L' /* GNU long-name extension typeflag */

/* The standard ustar header occupies the first 512 bytes of a block. */
struct tar_header {
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

/* Parse an octal field (tar stores sizes/modes as NUL/space-terminated octal). */
static int parse_octal(const char *field, size_t len, uint64_t *out)
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
        v = (v << 3) + (uint64_t) (c - '0');
        any = true;
    }
    if (!any) {
        *out = 0;
        return 0;
    }
    *out = v;
    return 0;
}

/* Reject absolute paths and any ".." component (path-traversal hardening). */
static int safe_member_name(const char *name)
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
        size_t seglen = (size_t) (p - seg);
        if (seglen == 2 && seg[0] == '.' && seg[1] == '.') {
            return -EINVAL;
        }
        if (*p == '/') {
            p++;
        }
    }
    return 0;
}

/* mkdir -p of every parent directory component of dest (a file path). */
static int mkdirs_for(const char *dest)
{
    char tmp[PATH_MAX];
    size_t n = strlen(dest);
    if (n >= sizeof(tmp)) {
        return -ENAMETOOLONG;
    }
    memcpy(tmp, dest, n + 1);
    for (char *q = tmp + 1; *q != '\0'; q++) {
        if (*q == '/') {
            *q = '\0';
            if (mkdir(tmp, 0700) != 0 && errno != EEXIST) {
                return -errno;
            }
            *q = '/';
        }
    }
    return 0;
}

/*
 * Recursively search `dir` for a regular file named `want`, writing its full
 * path into `out`.  Returns 0 if found, -ENOENT if not, or a negative errno on
 * an I/O error.
 */
static int find_file(const char *dir, const char *want, char *out,
                     size_t out_len)
{
    DIR *d = opendir(dir);
    if (d == NULL) {
        return -errno;
    }
    struct dirent *ent;
    int rc = -ENOENT;
    while ((ent = readdir(d)) != NULL) {
        if (strcmp(ent->d_name, ".") == 0 || strcmp(ent->d_name, "..") == 0) {
            continue;
        }
        char path[PATH_MAX];
        int n = snprintf(path, sizeof(path), "%s/%s", dir, ent->d_name);
        if (n < 0 || (size_t) n >= sizeof(path)) {
            continue;
        }
        struct stat st;
        if (lstat(path, &st) != 0) {
            continue;
        }
        if (S_ISDIR(st.st_mode)) {
            rc = find_file(path, want, out, out_len);
            if (rc == 0) {
                break;
            }
        } else if (S_ISREG(st.st_mode) && strcmp(ent->d_name, want) == 0) {
            if ((size_t) n >= out_len) {
                rc = -ENAMETOOLONG;
                break;
            }
            memcpy(out, path, (size_t) n + 1);
            rc = 0;
            break;
        }
    }
    closedir(d);
    return rc;
}

/* Is a 512-byte block entirely zero? */
static bool block_is_zero(const uint8_t *b)
{
    for (size_t i = 0; i < TAR_BLOCK; i++) {
        if (b[i] != 0) {
            return false;
        }
    }
    return true;
}

enum emu_vbin_status emu_vbin_classify(const void *data, size_t len)
{
    if (data == NULL || len == 0) {
        return EMU_VBIN_INCOMPLETE;
    }

    const uint8_t *base = (const uint8_t *) data;
    size_t off = 0;

    while (true) {
        /* Need a whole header block to make any decision. */
        if (off + TAR_BLOCK > len) {
            return EMU_VBIN_INCOMPLETE;
        }

        const struct tar_header *h = (const struct tar_header *) (base + off);

        /* The end-of-archive terminator is (at least) one zero block.  GNU tar
         * writes two; a single zero block at a 512-aligned boundary is enough to
         * recognise completion for our purposes (the unpacker stops there too). */
        if (block_is_zero(base + off)) {
            return EMU_VBIN_COMPLETE;
        }

        /* A real header must carry the ustar magic; anything else is garbage and
         * no amount of further bytes will fix it. */
        if (memcmp(h->magic, "ustar", 5) != 0) {
            return EMU_VBIN_INVALID;
        }

        uint64_t fsize = 0;
        if (parse_octal(h->size, sizeof(h->size), &fsize) != 0) {
            return EMU_VBIN_INVALID;
        }

        size_t data_blocks = (size_t) ((fsize + TAR_BLOCK - 1) / TAR_BLOCK);
        /* Guard against multiplication overflow on a hostile size field. */
        if (data_blocks > (SIZE_MAX - TAR_BLOCK) / TAR_BLOCK) {
            return EMU_VBIN_INVALID;
        }
        size_t advance = TAR_BLOCK + data_blocks * TAR_BLOCK;
        if (off + advance < off) {
            return EMU_VBIN_INVALID; /* overflow */
        }
        if (off + advance > len) {
            return EMU_VBIN_INCOMPLETE; /* member body not fully arrived yet */
        }
        off += advance;
    }
}

int emu_vbin_unpack_find_sim(const void *vbin, size_t len, const char *dest_dir,
                             char *exec_out, size_t exec_len)
{
    if (vbin == NULL || dest_dir == NULL || exec_out == NULL || exec_len == 0) {
        return -EINVAL;
    }
    if (len < TAR_BLOCK || (len % TAR_BLOCK) != 0) {
        return -EINVAL; /* not a block-aligned tar */
    }

    const uint8_t *base = vbin;
    size_t off = 0;

    /* GNU long-name carry-over: when a 'L' record precedes an entry, its payload
     * is the true name of the following entry. */
    char longname[PATH_MAX];
    bool have_longname = false;

    while (off + TAR_BLOCK <= len) {
        const struct tar_header *h = (const struct tar_header *) (base + off);

        /* Two consecutive zero blocks mark end-of-archive; a single all-zero
         * header also terminates parsing for our purposes. */
        bool all_zero = true;
        for (size_t i = 0; i < TAR_BLOCK; i++) {
            if (base[off + i] != 0) {
                all_zero = false;
                break;
            }
        }
        if (all_zero) {
            break;
        }

        uint64_t fsize = 0;
        if (parse_octal(h->size, sizeof(h->size), &fsize) != 0) {
            return -EINVAL;
        }
        uint64_t mode = 0;
        (void) parse_octal(h->mode, sizeof(h->mode), &mode);

        size_t data_off = off + TAR_BLOCK;
        size_t data_blocks = (size_t) ((fsize + TAR_BLOCK - 1) / TAR_BLOCK);
        if (data_off + data_blocks * TAR_BLOCK > len) {
            return -EINVAL; /* truncated archive */
        }

        if (h->typeflag == TAR_LONGNAME) {
            if (fsize == 0 || fsize >= sizeof(longname)) {
                return -EINVAL;
            }
            memcpy(longname, base + data_off, (size_t) fsize);
            longname[fsize] = '\0';
            have_longname = true;
            off = data_off + data_blocks * TAR_BLOCK;
            continue;
        }

        /* Determine the member name (long-name overrides the header name). */
        char name[PATH_MAX];
        if (have_longname) {
            size_t nl = strlen(longname);
            if (nl >= sizeof(name)) {
                return -EINVAL;
            }
            memcpy(name, longname, nl + 1);
            have_longname = false;
        } else {
            /* name field is at most 100 bytes, not necessarily NUL-terminated. */
            size_t nl = strnlen(h->name, sizeof(h->name));
            if (nl >= sizeof(name)) {
                return -EINVAL;
            }
            memcpy(name, h->name, nl);
            name[nl] = '\0';
        }

        if (name[0] == '\0') {
            off = data_off + data_blocks * TAR_BLOCK;
            continue;
        }
        if (safe_member_name(name) != 0) {
            return -EINVAL;
        }

        char dest[PATH_MAX];
        int dn = snprintf(dest, sizeof(dest), "%s/%s", dest_dir, name);
        if (dn < 0 || (size_t) dn >= sizeof(dest)) {
            return -ENAMETOOLONG;
        }

        bool is_dir = (h->typeflag == '5') ||
                      (name[strlen(name) - 1] == '/');

        if (is_dir) {
            if (mkdirs_for(dest) != 0) {
                return -EIO;
            }
            if (mkdir(dest, 0700) != 0 && errno != EEXIST) {
                return -errno;
            }
        } else if (h->typeflag == '0' || h->typeflag == '\0') {
            /* Regular file. */
            if (mkdirs_for(dest) != 0) {
                return -EIO;
            }
            /* Preserve at least the user-exec bit so vpp_sim is runnable; mask to
             * a sane 0700/0600 range (we run unsandboxed under our own uid). */
            mode_t fmode = (mode & 0100) ? 0700 : 0600;
            FILE *f = fopen(dest, "wb");
            if (f == NULL) {
                return -errno;
            }
            if (fsize > 0) {
                size_t w = fwrite(base + data_off, 1, (size_t) fsize, f);
                if (w != (size_t) fsize) {
                    fclose(f);
                    return -EIO;
                }
            }
            fclose(f);
            if (chmod(dest, fmode) != 0) {
                return -errno;
            }
        }
        /* Other typeflags (symlink, etc.) are ignored for the SIM VBIN. */

        off = data_off + data_blocks * TAR_BLOCK;
    }

    /* Locate the vpp_sim executable anywhere in the unpacked tree. */
    int rc = find_file(dest_dir, "vpp_sim", exec_out, exec_len);
    if (rc != 0) {
        return rc == -ENOENT ? -ENOENT : rc;
    }
    if (access(exec_out, X_OK) != 0) {
        return -EACCES;
    }
    return 0;
}
