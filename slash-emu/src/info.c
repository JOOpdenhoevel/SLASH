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
 * @file info.c
 * @brief Implementation of the @c /<BDF>/info endpoint (see info.h).
 */

#define _GNU_SOURCE

#include "info.h"

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <syslog.h>

#include "slash/uapi/slash_abi.h"
#include "utils.h"

/*
 * The info file's backing: a single fully-populated slash_info, built once at
 * attach time.  It never changes for the device's lifetime, so reads need no
 * locking of their own beyond the spine's (emu_node_pread holds the tree lock
 * across the read hook, and this struct is immutable after attach anyway).
 */
struct emu_info_backing {
    struct slash_info info;
};

ssize_t emu_info_pread_buf(char *dst, size_t size, off_t off, const void *src,
                           size_t src_len)
{
    if (off < 0) {
        return -EINVAL;
    }

    /* At or past EOF: nothing to copy (a valid zero-length read). */
    if ((size_t) off >= src_len) {
        return 0;
    }

    size_t avail = src_len - (size_t) off;
    size_t n = size < avail ? size : avail; /* short read when size > avail */

    memcpy(dst, (const char *) src + off, n);

    return (ssize_t) n;
}

/* getattr size hook: the file size is exactly sizeof(struct slash_info). */
static off_t info_size(const struct emu_node *node, void *backing)
{
    (void) node;
    (void) backing;

    return (off_t) sizeof(struct slash_info);
}

/* pread hook: serve the bytes of the immutable backing struct. */
static ssize_t info_read(const struct emu_node *node, void *backing, char *buf,
                         size_t size, off_t off)
{
    (void) node;

    const struct emu_info_backing *b = backing;

    return emu_info_pread_buf(buf, size, off, &b->info, sizeof(b->info));
}

/* destroy hook: free the backing allocated in emu_info_attach. */
static void info_destroy(struct emu_node *node, void *backing)
{
    (void) node;

    free(backing);
}

static const struct emu_node_ops emu_info_ops = {
    .size = info_size,
    .read = info_read,
    .destroy = info_destroy,
};

int emu_info_attach(struct emu_device *dev)
{
    if (dev == NULL || dev->dir == NULL || dev->tree == NULL) {
        return -1;
    }

    _cleanup_(cleanup_free)
    struct emu_info_backing *backing = calloc(1, sizeof(*backing));
    PROPAGATE_ERROR_NULL_LOG(backing, LOG_ERR,
                             "Failed to allocate info backing for '%s'",
                             dev->bdf);

    backing->info = (struct slash_info) {
        .size = (uint32_t) sizeof(struct slash_info),
        .acc_type = SLASH_ACC_TYPE_SYSTEM_EMULATED,
    };
    /*
     * dev->bdf is the normalized board-level BDF "DDDD:BB:DD" (no function),
     * which is exactly what slash_info.bdf advertises.  Copy it bounded; the
     * destination is SLASH_PCI_BDF_LEN and the device's bdf buffer is smaller,
     * so this never truncates, but stay defensive.
     */
    int n = snprintf(backing->info.bdf, sizeof(backing->info.bdf), "%s",
                     dev->bdf);
    if (n < 0 || (size_t) n >= sizeof(backing->info.bdf)) {
        LOG(LOG_ERR, "BDF '%s' too long for info struct", dev->bdf);
        return -1;
    }

    /*
     * Hand the backing to the node; from here the node owns it and frees it via
     * info_destroy.  0444: read-only, the ABI's info file is never written.
     */
    if (emu_node_create_child(dev->tree, dev->dir, "info", EMU_NODE_FILE, 0444,
                              &emu_info_ops, backing, NULL) == -1) {
        LOG(LOG_ERR, "Failed to create info node for '%s'", dev->bdf);
        return -1;
    }
    backing = NULL; /* ownership transferred to the node */

    return 0;
}
