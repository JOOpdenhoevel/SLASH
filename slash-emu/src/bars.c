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
 * @file bars.c
 * @brief Implementation of the @c /<BDF>/bars/ endpoint (see bars.h).
 */

#define _GNU_SOURCE

#include "bars.h"

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <syslog.h>

#include "slash/uapi/slash_abi.h"
#include "utils.h"

/*
 * Per-BAR backing.  Geometry is fixed at attach time; the register state is an
 * in-memory shadow allocated lazily on first write (so the 128 MiB user/service
 * BARs cost nothing until actually poked).  A read of an unallocated shadow
 * returns zero bytes -- the defined "never written" value.  The SIM bridge seam
 * (T10) is the optional `backend` vtable; NULL today, meaning "use the shadow".
 *
 * The struct is mutated only from the pread/pwrite hooks, which the spine
 * invokes with the tree lock held, so it needs no locking of its own.
 */
struct emu_bar_backing {
    uint32_t index;   /* PF2 BAR index: 0, 2, or 4. */
    uint64_t size;    /* BAR size in bytes (== reported file size). */
    uint8_t *shadow;  /* Lazily-allocated register shadow, or NULL. */

    /* SIM register-bridge seam (T10).  NULL => in-memory shadow only. */
    const struct emu_bar_backend *backend; /* non-owning, static */
};

/* The three BARs PF2 exposes.  Order matches creation order under bars/. */
static const struct {
    uint32_t index;
    uint64_t size;
    const char *name;
} EMU_BAR_TABLE[] = {
    { SLASH_BAR_USER_IDX, SLASH_BAR_USER_SIZE, "bar0" },
    { SLASH_BAR_SL_IDX,   SLASH_BAR_SL_SIZE,   "bar2" },
    { SLASH_BAR_CLK_IDX,  SLASH_BAR_CLK_SIZE,  "bar4" },
};

int emu_bar_check_access(off_t off, size_t width, uint64_t bar_size)
{
    /* Width must be a single register transfer of 1, 2, 4, or 8 bytes. */
    if (width != 1 && width != 2 && width != 4 && width != 8) {
        return -EINVAL;
    }

    /* Offset must be non-negative and naturally aligned to the width. */
    if (off < 0 || (uint64_t) off % width != 0) {
        return -EINVAL;
    }

    /*
     * The whole transfer must fit inside the BAR.  Compare in uint64_t and
     * guard the add against overflow (off is already >= 0 here).  A transfer
     * that straddles the end is invalid, not clamped: width == transfer size.
     */
    uint64_t end = (uint64_t) off + width;
    if (end < (uint64_t) off || end > bar_size) {
        return -EINVAL;
    }

    return 0;
}

/* getattr size hook: the file size is exactly the BAR size. */
static off_t bar_size_hook(const struct emu_node *node, void *backing)
{
    (void) node;

    const struct emu_bar_backing *b = backing;

    return (off_t) b->size;
}

/* pread hook: serve a validated, fixed-width register read. */
static ssize_t bar_read(const struct emu_node *node, void *backing, char *buf,
                        size_t size, off_t off)
{
    (void) node;

    struct emu_bar_backing *b = backing;

    int rc = emu_bar_check_access(off, size, b->size);
    if (rc != 0) {
        return rc;
    }

    /*
     * SIM bridge seam (T10): if a backend is attached, ask the model for the
     * register value first.  rc == 0 => model answered (use `value`); rc > 0 =>
     * "fall back to the shadow" (an in-range read the model cannot answer must
     * still return defined bytes, never -EIO); rc < 0 => transport failure.
     */
    if (b->backend != NULL && b->backend->read != NULL) {
        uint64_t value = 0;
        rc = b->backend->read(b->backend->ctx, b->index, off, size, &value);
        if (rc < 0) {
            return -ENODEV;
        }
        if (rc == 0) {
            /* Little-endian register word, low `size` bytes. */
            for (size_t i = 0; i < size; i++) {
                buf[i] = (char) ((value >> (8 * i)) & 0xFF);
            }
            return (ssize_t) size;
        }
        /* rc > 0: fall through to the shadow. */
    }

    if (b->shadow == NULL) {
        /* Never written: defined zero value. */
        memset(buf, 0, size);
    } else {
        memcpy(buf, b->shadow + off, size);
    }

    return (ssize_t) size;
}

/* pwrite hook: store a validated, fixed-width register write. */
static ssize_t bar_write(const struct emu_node *node, void *backing,
                         const char *buf, size_t size, off_t off)
{
    (void) node;

    struct emu_bar_backing *b = backing;

    int rc = emu_bar_check_access(off, size, b->size);
    if (rc != 0) {
        return rc;
    }

    /*
     * Update the in-memory shadow, allocating it on first write.  The shadow is
     * always kept current -- even when a SIM backend is attached -- so an
     * in-range read the model cannot answer can fall back to it (G7: never
     * surface -EIO for an in-range access).
     */
    if (b->shadow == NULL) {
        b->shadow = calloc(1, b->size);
        if (b->shadow == NULL) {
            /*
             * This is an ops->write hook: the return is fed straight to
             * fuse_reply_err(req, -n), so it MUST be a negative errno.  Do NOT
             * use PROPAGATE_ERROR_NULL_* here -- those return -1, which would
             * surface to the client as EPERM rather than ENOMEM.
             */
            LOG(LOG_ERR, "Failed to allocate BAR%u shadow", b->index);
            return -ENOMEM;
        }
    }
    memcpy(b->shadow + off, buf, size);

    /* SIM bridge seam (T10): forward the validated poke to the model. */
    if (b->backend != NULL && b->backend->write != NULL) {
        uint64_t value = 0;
        for (size_t i = 0; i < size; i++) {
            value |= (uint64_t) (uint8_t) buf[i] << (8 * i);
        }
        if (b->backend->write(b->backend->ctx, b->index, off, size, value) < 0) {
            return -ENODEV;
        }
    }

    return (ssize_t) size;
}

/* destroy hook: free the shadow and the backing allocated in emu_bars_attach. */
static void bar_destroy(struct emu_node *node, void *backing)
{
    (void) node;

    struct emu_bar_backing *b = backing;
    if (b == NULL) {
        return;
    }

    free(b->shadow);
    free(b);
}

static const struct emu_node_ops emu_bar_ops = {
    .size = bar_size_hook,
    .read = bar_read,
    .write = bar_write,
    .destroy = bar_destroy,
};

int emu_bars_attach(struct emu_device *dev)
{
    if (dev == NULL || dev->bars == NULL || dev->tree == NULL) {
        return -1;
    }

    for (size_t i = 0; i < SIZEOF_ARRAY(EMU_BAR_TABLE); i++) {
        _cleanup_(cleanup_free)
        struct emu_bar_backing *backing = calloc(1, sizeof(*backing));
        PROPAGATE_ERROR_NULL_LOG(backing, LOG_ERR,
                                 "Failed to allocate BAR backing for '%s'",
                                 dev->bdf);

        *backing = (struct emu_bar_backing) {
            .index = EMU_BAR_TABLE[i].index,
            .size = EMU_BAR_TABLE[i].size,
            .shadow = NULL,
            .backend = NULL, /* T10 attaches the SIM bridge here. */
        };

        /*
         * 0600: a BAR file is read/write register access, owner-only.  Hand the
         * backing to the node; from here the node owns it and frees it (shadow
         * included) via bar_destroy.
         */
        struct emu_node *node = NULL;
        if (emu_node_create_child(dev->tree, dev->bars, EMU_BAR_TABLE[i].name,
                                  EMU_NODE_FILE, 0600, &emu_bar_ops, backing,
                                  &node) == -1) {
            LOG(LOG_ERR, "Failed to create %s node for '%s'",
                EMU_BAR_TABLE[i].name, dev->bdf);
            return -1;
        }
        backing = NULL; /* ownership transferred to the node */

        /*
         * Register access is unbuffered: open the BAR file with direct_io so the
         * kernel forwards each pread/pwrite verbatim (exact size + offset) and
         * does not synthesize page-sized, page-cached transfers the width
         * validation would reject.
         */
        emu_node_set_direct_io(dev->tree, node);
    }

    return 0;
}

int emu_bars_set_backend(struct emu_device *dev,
                         const struct emu_bar_backend *backend)
{
    if (dev == NULL || dev->bars == NULL) {
        return -1;
    }

    /* The bars/ dir node's children are exactly the bar<M> files this endpoint
     * created; set the backend on each one whose ops are ours.  No bar file
     * created => the endpoint was not attached. */
    bool found = false;
    for (size_t i = 0; i < dev->bars->children.len; i++) {
        struct emu_node *child = dev->bars->children.d[i];
        if (child->ops != &emu_bar_ops) {
            continue;
        }
        struct emu_bar_backing *b = child->backing;
        if (b != NULL) {
            b->backend = backend;
            found = true;
        }
    }

    return found ? 0 : -1;
}
