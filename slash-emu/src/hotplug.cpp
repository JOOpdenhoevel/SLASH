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
 * @file hotplug.cpp
 * @brief Implementation of the global @c /hotplug endpoint (see hotplug.hpp).
 *
 * Port of hotplug.c to C++20: std::function replaces the raw fn/ctx reload
 * pointer pair; @ref HotplugOps subclasses @ref NodeOps to carry the backing
 * state; RAII members replace the malloc/free pattern.
 */

#include "hotplug.hpp"

#include <cctype>
#include <cerrno>
#include <cstring>
#include <memory>
#include <string>
#include <time.h>

#include "config.hpp"
#include "node.hpp"
#include "utils.hpp"

#include "slash/uapi/slash_abi.h"

namespace slash::emu {

namespace {

/** @brief Entry name of the global hotplug file under the mount root. */
static constexpr const char kHotplugName[] = "hotplug";

/* ================================================================== */
/* BDF-with-function parsing (internal helper)                        */
/* ================================================================== */

/**
 * @brief Validate and extract a NUL-terminated BDF from a device-request struct.
 *
 * Requires that @p in_size covers the full struct and that the @c bdf array is
 * NUL-terminated within its fixed bounds.  Returns a pointer into the struct's
 * bdf array on success.
 *
 * @return 0 and sets @p bdf_out on success; negative errno on failure.
 */
static int readDeviceRequest(const void *in, size_t in_size,
                             const char **bdf_out)
{
    if (in == nullptr ||
        in_size < sizeof(struct slash_abi_hotplug_device_request)) {
        return -EINVAL;
    }
    const auto *req =
        static_cast<const struct slash_abi_hotplug_device_request *>(in);
    if (std::memchr(req->bdf, '\0', sizeof(req->bdf)) == nullptr) {
        LOG(LOG_ERR, "Hotplug request BDF is not NUL-terminated");
        return -EINVAL;
    }
    *bdf_out = req->bdf;
    return 0;
}

/* ================================================================== */
/* Command handlers                                                   */
/* ================================================================== */

/**
 * @brief RESCAN: invoke the reload seam.
 *
 * The reload callback performs config re-read + idempotent materialize;
 * the hotplug module simply calls it.  An empty @p reload is a no-op
 * (unit tests that do not exercise the reload path).
 */
static int doRescan(NodeTree &tree, const ReloadFn &reload)
{
    (void) tree;
    if (!reload) {
        return 0;
    }
    return reload() == 0 ? 0 : -EIO;
}

/**
 * @brief REMOVE: revoke one function's subtree (fn1 = qdma, fn2 = bars).
 */
static int doRemove(NodeTree &tree, const void *in, size_t in_size)
{
    const char *raw = nullptr;
    int rc = readDeviceRequest(in, in_size, &raw);
    if (rc != 0) {
        return rc;
    }

    std::string bdf;
    DeviceFunction func;
    rc = hotplugParseBdf(raw, bdf, func);
    if (rc != 0) {
        return rc;
    }

    /*
     * PF0 (board management, owned by the ami driver) is not emulated: a REMOVE of
     * a syntactically-valid <BDF>.0 is tolerated as a no-op success and must change
     * nothing in the tree.  Short-circuit before the spine's per-function revoke,
     * which accepts only the QDMA / BARS functions.
     */
    if (func == DeviceFunction::Pf0) {
        return 0;
    }

    if (tree.revokeFunction(bdf, func) == -1) {
        return -EINVAL;
    }
    return 0;
}

/**
 * @brief Emulate the SBR PCIe-link-retraining sleep.
 *
 * A 0 duration disables it.  The FUSE session is single-threaded, so this
 * blocks the whole daemon for the duration; the duration is injectable
 * (tiny in tests).
 */
static void sbrSleep(unsigned int sleep_us)
{
    if (sleep_us == 0) {
        return;
    }
    struct timespec ts {
        static_cast<time_t>(sleep_us / 1000000u),
        static_cast<long>(sleep_us % 1000000u) * 1000L
    };
    (void) ::nanosleep(&ts, nullptr);
}

/**
 * @brief TOGGLE_SBR / HOTPLUG: fully remove the device, reload, and optionally
 *        sleep to emulate PCIe link retraining.
 */
static int doRemoveAndReload(NodeTree &tree, const ReloadFn &reload,
                             unsigned int sbr_sleep_us, const void *in,
                             size_t in_size, bool sbr)
{
    const char *raw = nullptr;
    int rc = readDeviceRequest(in, in_size, &raw);
    if (rc != 0) {
        return rc;
    }

    std::string bdf;
    DeviceFunction func; /* parsed for validation; ignored below (whole device) */
    rc = hotplugParseBdf(raw, bdf, func);
    if (rc != 0) {
        return rc;
    }

    /* Fully remove the referenced accelerator (both functions). */
    if (tree.revokeDevice(bdf) == -1) {
        return -EINVAL;
    }

    /* Reload config + re-init available accelerators (idempotent materialize). */
    if (reload && reload() != 0) {
        return -EIO;
    }

    if (sbr) {
        sbrSleep(sbr_sleep_us);
    }

    return 0;
}

/* ================================================================== */
/* NodeOps subclass                                                   */
/* ================================================================== */

/**
 * @brief Backing state and ioctl dispatch for the global @c /hotplug file.
 *
 * Owns the reload callback (captured by value from the FUSE layer).  The SBR
 * sleep duration is mutable so the test injection seam can change it without
 * re-attaching.  The ioctl hook runs with the tree lock @em dropped (the node
 * is marked @c ioctl_unlocked), so the self-locking spine API (revokeFunction /
 * revokeDevice / reload) can take the lock safely.
 */
class HotplugOps final : public NodeOps {
public:
    HotplugOps(NodeTree &tree, ReloadFn reload)
        : tree_(tree)
        , reload_(std::move(reload))
        , sbr_sleep_us_(kHotplugSbrSleepUsDefault)
    {
    }

    /**
     * @brief Dispatch the four hotplug commands.
     *
     * None of the commands produce an out-bound payload so @p out / @p out_size
     * are unused.  Invoked with the tree lock @em dropped (@c ioctl_unlocked).
     */
    int ioctl(Node &node, unsigned int cmd, const void *in, size_t in_size,
              void *out, size_t out_size) override
    {
        (void) node;
        (void) out;
        (void) out_size;

        switch (cmd) {
        case SLASH_ABI_HOTPLUG_IOCTL_RESCAN:
            return doRescan(tree_, reload_);
        case SLASH_ABI_HOTPLUG_IOCTL_REMOVE:
            return doRemove(tree_, in, in_size);
        case SLASH_ABI_HOTPLUG_IOCTL_TOGGLE_SBR:
            return doRemoveAndReload(tree_, reload_, sbr_sleep_us_, in, in_size,
                                     /*sbr=*/true);
        case SLASH_ABI_HOTPLUG_IOCTL_HOTPLUG:
            return doRemoveAndReload(tree_, reload_, sbr_sleep_us_, in, in_size,
                                     /*sbr=*/false);
        default:
            return -ENOTTY;
        }
    }

private:
    NodeTree &tree_;
    ReloadFn reload_;

public:
    /** @brief SBR sleep duration (microseconds); injectable by tests. */
    unsigned int sbr_sleep_us_;
};

/* ================================================================== */
/* Helpers: find the hotplug node                                     */
/* ================================================================== */

/**
 * @brief Locate the @c hotplug child of the root (does not bump lookup count).
 * @return The node, or nullptr if not attached.
 */
static Node *findHotplugNode(NodeTree &tree)
{
    Node *root = tree.root();
    if (root == nullptr) {
        return nullptr;
    }
    for (Node *child : root->children) {
        if (child->type == NodeType::File && !child->unlinked &&
            child->name == kHotplugName) {
            return child;
        }
    }
    return nullptr;
}

} // namespace

/* ================================================================== */
/* Public API                                                         */
/* ================================================================== */

int hotplugAttach(NodeTree &tree, ReloadFn reload)
{
    Node *root = tree.root();
    if (root == nullptr) {
        LOG(LOG_ERR, "Hotplug attach: no tree root");
        return -1;
    }

    /* The global hotplug file: ioctl-only (no read/write content), 0600. */
    Node *node = tree.createChild(root, kHotplugName, NodeType::File, 0600,
                                  std::make_unique<HotplugOps>(tree, std::move(reload)));
    if (node == nullptr) {
        LOG(LOG_ERR, "Failed to create hotplug node");
        return -1;
    }

    /*
     * The command IS the self-locking revoke/reload machinery -> run the hook
     * with the lock dropped so it does not deadlock the non-recursive mutex.
     * Safe because the node outlives every unlocked call: it is created
     * NON-unlinkable (the default), so the FUSE unlink op rejects it with
     * -EPERM, enforcing the "hotplug file is never removed" precondition.
     */
    tree.setIoctlUnlocked(node);

    return 0;
}

int hotplugSetSbrSleepUs(NodeTree &tree, unsigned int sleep_us)
{
    Node *node = findHotplugNode(tree);
    if (node == nullptr || node->ops == nullptr) {
        return -1;
    }
    auto *ops = dynamic_cast<HotplugOps *>(node->ops.get());
    if (ops == nullptr) {
        return -1;
    }
    ops->sbr_sleep_us_ = sleep_us;
    return 0;
}

int hotplugParseBdf(const std::string &input, std::string &bdf_out,
                    DeviceFunction &func_out)
{
    /*
     * The request BDF includes the function ("DDDD:BB:DD.F"); the per-device
     * folder is the BDF WITHOUT the function.  Split on the last '.', normalize
     * the board-level prefix via normalizeBdf (which rejects a function suffix),
     * then map the function digit to the removable endpoint.
     */
    auto dot_pos = input.rfind('.');
    if (dot_pos == std::string::npos || dot_pos == 0 ||
        dot_pos + 1 >= input.size()) {
        LOG(LOG_ERR, "Hotplug BDF '%s' missing a function suffix",
            input.c_str());
        return -EINVAL;
    }

    /* The function field must be exactly one decimal digit. */
    char func_char = input[dot_pos + 1];
    if (!std::isdigit(static_cast<unsigned char>(func_char)) ||
        dot_pos + 2 != input.size()) {
        LOG(LOG_ERR, "Hotplug BDF '%s' has a malformed function suffix",
            input.c_str());
        return -EINVAL;
    }

    /* Extract the board-level prefix (everything before the '.'). */
    std::string board = input.substr(0, dot_pos);

    /* Validate length: a pathologically long prefix must be rejected. */
    if (board.size() >= 64) {
        LOG(LOG_ERR, "Hotplug BDF '%s' board prefix too long", input.c_str());
        return -EINVAL;
    }

    auto normalized = normalizeBdf(board);
    if (!normalized.has_value()) {
        /* normalizeBdf already logs the specific reason via LOG. */
        return -EINVAL;
    }
    bdf_out = std::move(*normalized);

    switch (func_char) {
    case '0':
        /* PF0 is board management owned by the ami driver; the daemon does not
         * emulate it.  A syntactically-valid <BDF>.0 parses to the Pf0 sentinel
         * so REMOVE can treat it as a tolerated no-op (see doRemove). */
        func_out = DeviceFunction::Pf0;
        return 0;
    case '1':
        func_out = DeviceFunction::Qdma;
        return 0;
    case '2':
        func_out = DeviceFunction::Bars;
        return 0;
    default:
        LOG(LOG_ERR,
            "Hotplug BDF '%s' function %c is not removable (expect 0, 1 or 2)",
            input.c_str(), func_char);
        return -EOPNOTSUPP;
    }
}

} // namespace slash::emu
