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
 * @file hotplug.c
 * @brief Implementation of the global @c /hotplug endpoint (see hotplug.h).
 */

#define _GNU_SOURCE

#include "hotplug.h"

#include <ctype.h>
#include <errno.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <syslog.h>
#include <time.h>

#include "config.h"
#include "slash/uapi/slash_abi.h"
#include "utils.h"

/** @brief Entry name of the global hotplug file under the mount root. */
#define EMU_HOTPLUG_NAME "hotplug"

/*
 * Backing for the hotplug file node.  Owns nothing heavy: the tree is borrowed
 * (the node belongs to it), and the reload callback + ctx are borrowed from the
 * FUSE layer.  The SBR sleep duration is injectable so tests do not block the
 * single-threaded daemon for a real second.  Read by the ioctl hook, which runs
 * with the tree lock DROPPED (the node is ioctl_unlocked); the backing is set
 * once at attach and freed only at tree destruction, so no locking is needed for
 * the borrowed pointers.  The mutable sbr_sleep_us is only written by the test
 * injection seam (single-threaded), so it likewise needs no lock here.
 */
struct emu_hotplug_backing {
    struct emu_node_tree *tree;    /* borrowed */
    emu_hotplug_reload_fn reload;  /* borrowed, may be NULL */
    void *reload_ctx;              /* borrowed */
    unsigned int sbr_sleep_us;
};

/* ================================================================== */
/* BDF-with-function parsing                                          */
/* ================================================================== */

int emu_hotplug_parse_bdf(const char *input, char *bdf_out, size_t bdf_len,
                          enum emu_device_function *func_out)
{
    if (input == NULL || bdf_out == NULL || func_out == NULL) {
        return -EINVAL;
    }

    /*
     * The request BDF includes the function ("DDDD:BB:DD.F"); the per-device
     * folder is the BDF WITHOUT the function.  Split on the last '.', normalize
     * the board-level prefix via the existing validator (which rejects a function
     * suffix), then map the function digit to the removable endpoint.
     */
    const char *dot = strrchr(input, '.');
    if (dot == NULL || dot == input || dot[1] == '\0') {
        LOG(LOG_ERR, "Hotplug BDF '%s' missing a function suffix", input);
        return -EINVAL;
    }

    /* The function field must be exactly one decimal digit. */
    if (!isdigit((unsigned char) dot[1]) || dot[2] != '\0') {
        LOG(LOG_ERR, "Hotplug BDF '%s' has a malformed function suffix", input);
        return -EINVAL;
    }

    /* Copy the board-level prefix (everything before the '.') and normalize it.
     * Use a bounded scratch buffer so a pathologically long input cannot overrun;
     * emu_bdf_normalize rejects anything that does not fit the BDF grammar. */
    char board[64];
    size_t prefix_len = (size_t) (dot - input);
    if (prefix_len >= sizeof(board)) {
        LOG(LOG_ERR, "Hotplug BDF '%s' board prefix too long", input);
        return -EINVAL;
    }
    memcpy(board, input, prefix_len);
    board[prefix_len] = '\0';

    if (emu_bdf_normalize(board, bdf_out, bdf_len) == -1) {
        /* emu_bdf_normalize already logged the specific reason. */
        return -EINVAL;
    }

    switch (dot[1]) {
    case '1':
        *func_out = EMU_DEVICE_FUNCTION_QDMA;
        return 0;
    case '2':
        *func_out = EMU_DEVICE_FUNCTION_BARS;
        return 0;
    default:
        LOG(LOG_ERR, "Hotplug BDF '%s' function %c is not removable (expect 1 or 2)",
            input, dot[1]);
        return -EOPNOTSUPP;
    }
}

/*
 * Validate the device-request struct and extract a NUL-terminated BDF string.
 * The struct carries a leading size word for ABI versioning; require at least
 * the prefix through bdf, and insist the bdf field is NUL-terminated within its
 * fixed array (a non-terminated bdf is a malformed request, not a truncation we
 * silently accept).
 */
static int read_device_request(const void *in, size_t in_size,
                               const char **bdf_out)
{
    if (in_size < sizeof(struct slash_abi_hotplug_device_request)) {
        return -EINVAL;
    }

    const struct slash_abi_hotplug_device_request *req = in;

    if (memchr(req->bdf, '\0', sizeof(req->bdf)) == NULL) {
        LOG(LOG_ERR, "Hotplug request BDF is not NUL-terminated");
        return -EINVAL;
    }

    *bdf_out = req->bdf;
    return 0;
}

/* ================================================================== */
/* Command handlers                                                   */
/* ================================================================== */

/*
 * RESCAN: reload configuration and (re-)materialize all configured accelerators,
 * skipping BDFs that collide with an already-running one.  The skip + idempotent
 * re-init are entirely in the reload callback's materialize path (which seeds the
 * running-set from the live devices), so RESCAN is just "reload".
 */
static int hotplug_rescan(struct emu_hotplug_backing *b)
{
    if (b->reload == NULL) {
        return 0; /* no reload seam wired (e.g. a unit test): nothing to do. */
    }
    return b->reload(b->reload_ctx) == 0 ? 0 : -EIO;
}

/*
 * REMOVE: parse the function-qualified BDF, then revoke just that one function's
 * subtree (qdma/ for fn1, bars/ for fn2).  The other function stays live; the
 * model-shutdown seam fires from the spine once both functions are gone.
 *
 * emu_device_revoke_function is idempotent and treats an absent device as a
 * no-op success; per the revocation contract a REMOVE of an unknown/already-gone
 * endpoint is not an error (the postcondition "the endpoint is gone" already
 * holds).
 */
static int hotplug_remove(struct emu_hotplug_backing *b, const void *in,
                          size_t in_size)
{
    const char *raw = NULL;
    int rc = read_device_request(in, in_size, &raw);
    if (rc != 0) {
        return rc;
    }

    char bdf[EMU_BDF_LEN];
    enum emu_device_function func;
    rc = emu_hotplug_parse_bdf(raw, bdf, sizeof(bdf), &func);
    if (rc != 0) {
        return rc;
    }

    if (emu_device_revoke_function(b->tree, bdf, func) == -1) {
        return -EINVAL;
    }

    return 0;
}

/* Emulate the SBR PCIe-link-retraining sleep.  The FUSE session is
 * single-threaded, so this blocks the whole daemon for the duration; the
 * duration is injectable (tiny in tests) precisely so that tradeoff is bounded.
 * A 0 duration disables it. */
static void hotplug_sbr_sleep(unsigned int sleep_us)
{
    if (sleep_us == 0) {
        return;
    }

    struct timespec ts = {
        .tv_sec = (time_t) (sleep_us / 1000000u),
        .tv_nsec = (long) (sleep_us % 1000000u) * 1000L,
    };
    (void) nanosleep(&ts, NULL);
}

/*
 * TOGGLE_SBR / HOTPLUG: fully remove the referenced accelerator (whole device,
 * both functions), reload configuration, and re-initialize every configured
 * accelerator whose BDF is currently available.  TOGGLE_SBR additionally
 * emulates the ~1 s sleep; HOTPLUG does not.
 *
 * Per the ABI, TOGGLE_SBR uses only the domain+bus of the BDF to locate the
 * bridge and ignores the device/function fields.  Here the system-emulated
 * "bridge" is the single accelerator at that board BDF, so we resolve the
 * board-level BDF and remove that whole device; the function suffix is parsed
 * for validation but is otherwise ignored (both functions go).
 */
static int hotplug_remove_and_reload(struct emu_hotplug_backing *b,
                                     const void *in, size_t in_size, bool sbr)
{
    const char *raw = NULL;
    int rc = read_device_request(in, in_size, &raw);
    if (rc != 0) {
        return rc;
    }

    char bdf[EMU_BDF_LEN];
    enum emu_device_function func; /* parsed for validation; ignored below. */
    rc = emu_hotplug_parse_bdf(raw, bdf, sizeof(bdf), &func);
    if (rc != 0) {
        return rc;
    }

    /* Fully remove the referenced accelerator (both functions). */
    if (emu_device_revoke(b->tree, bdf) == -1) {
        return -EINVAL;
    }

    /* Reload config + re-init available accelerators (idempotent materialize). */
    if (b->reload != NULL && b->reload(b->reload_ctx) != 0) {
        return -EIO;
    }

    if (sbr) {
        hotplug_sbr_sleep(b->sbr_sleep_us);
    }

    return 0;
}

/* ================================================================== */
/* ioctl dispatch                                                     */
/* ================================================================== */

/*
 * The hotplug file's ioctl hook.  Runs with the tree lock DROPPED (the node is
 * marked ioctl_unlocked) so the self-locking revoke/reload spine API it calls
 * does not deadlock the non-recursive mutex.  None of the commands produce an
 * out-bound payload, so `out`/`out_size` are unused.
 */
static int hotplug_ioctl(struct emu_node *node, void *backing, unsigned int cmd,
                         const void *in, size_t in_size, void *out,
                         size_t out_size)
{
    (void) node;
    (void) out;
    (void) out_size;

    struct emu_hotplug_backing *b = backing;

    switch (cmd) {
    case SLASH_ABI_HOTPLUG_IOCTL_RESCAN:
        return hotplug_rescan(b);
    case SLASH_ABI_HOTPLUG_IOCTL_REMOVE:
        return hotplug_remove(b, in, in_size);
    case SLASH_ABI_HOTPLUG_IOCTL_TOGGLE_SBR:
        return hotplug_remove_and_reload(b, in, in_size, /*sbr=*/true);
    case SLASH_ABI_HOTPLUG_IOCTL_HOTPLUG:
        return hotplug_remove_and_reload(b, in, in_size, /*sbr=*/false);
    default:
        return -ENOTTY;
    }
}

static void hotplug_destroy(struct emu_node *node, void *backing)
{
    (void) node;
    free(backing);
}

static const struct emu_node_ops emu_hotplug_ops = {
    .ioctl = hotplug_ioctl,
    .destroy = hotplug_destroy,
};

/* ================================================================== */
/* Attach + injection seams                                          */
/* ================================================================== */

/* Resolve the hotplug file node under the tree root (a plain child finder; does
 * not bump any lookup count).  Returns NULL if not attached. */
static struct emu_node *find_hotplug_node(struct emu_node_tree *tree)
{
    struct emu_node *root = emu_node_lookup_ino(tree, EMU_ROOT_INO);
    if (root == NULL) {
        return NULL;
    }

    for (size_t i = 0; i < root->children.len; i++) {
        struct emu_node *child = root->children.d[i];
        if (child->type == EMU_NODE_FILE && !child->unlinked &&
            strcmp(child->name, EMU_HOTPLUG_NAME) == 0) {
            return child;
        }
    }
    return NULL;
}

int emu_hotplug_attach(struct emu_node_tree *tree, emu_hotplug_reload_fn reload,
                       void *ctx)
{
    if (tree == NULL) {
        return -1;
    }

    struct emu_node *root = emu_node_lookup_ino(tree, EMU_ROOT_INO);
    PROPAGATE_ERROR_NULL_LOG(root, LOG_ERR, "Hotplug attach: no tree root");

    _cleanup_(cleanup_free)
    struct emu_hotplug_backing *backing = calloc(1, sizeof(*backing));
    PROPAGATE_ERROR_NULL_LOG(backing, LOG_ERR,
                             "Failed to allocate hotplug backing");

    *backing = (struct emu_hotplug_backing) {
        .tree = tree,
        .reload = reload,
        .reload_ctx = ctx,
        .sbr_sleep_us = EMU_HOTPLUG_SBR_SLEEP_US_DEFAULT,
    };

    /* The global hotplug file: read/write are meaningless (ioctl-only), so the
     * mode advertises no read/write content; 0600 matches the other endpoints. */
    struct emu_node *node = NULL;
    if (emu_node_create_child(tree, root, EMU_HOTPLUG_NAME, EMU_NODE_FILE, 0600,
                              &emu_hotplug_ops, backing, &node) == -1) {
        LOG(LOG_ERR, "Failed to create hotplug node");
        return -1;
    }
    backing = NULL; /* ownership transferred to the node (freed via destroy). */

    /*
     * The command IS the self-locking revoke/reload machinery -> run the hook
     * with the lock dropped (see node.h ioctl_unlocked).  This is safe because
     * the node outlives every unlocked call: it is created NON-unlinkable (the
     * default -- the FUSE unlink op now rejects it with -EPERM), and it has no
     * device, so neither emu_device_revoke nor emu_device_revoke_function ever
     * touches it.  The "hotplug file is never removed" precondition is thus
     * enforced, not merely assumed.
     */
    emu_node_set_ioctl_unlocked(tree, node);

    return 0;
}

int emu_hotplug_set_sbr_sleep_us(struct emu_node_tree *tree,
                                 unsigned int sleep_us)
{
    if (tree == NULL) {
        return -1;
    }

    struct emu_node *node = find_hotplug_node(tree);
    if (node == NULL || node->backing == NULL) {
        return -1;
    }

    struct emu_hotplug_backing *b = node->backing;
    b->sbr_sleep_us = sleep_us;
    return 0;
}
