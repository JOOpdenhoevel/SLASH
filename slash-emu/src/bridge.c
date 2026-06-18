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
 * @file bridge.c
 * @brief Implementation of the SIM data-plane bridge (see bridge.h).
 */

#define _GNU_SOURCE

#include "bridge.h"

#include <errno.h>
#include <fcntl.h>
#include <ftw.h>
#include <limits.h>
#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

#include "array.h"
#include "bars.h"
#include "model_client.h"
#include "qdma.h"
#include "slash/uapi/slash_abi.h"
#include "utils.h"
#include "vbin.h"

/** @brief Default scratch root if none is configured. */
#define EMU_BRIDGE_DEFAULT_SCRATCH "/run/slash_emu/.scratch"

/** @brief Bounded wait for the child to die after SIGTERM, before SIGKILL (ms). */
#define EMU_BRIDGE_TERM_WAIT_MS 500

/**
 * @brief Cap on the reassembled VBIN size (the accumulation buffer ceiling).
 *
 * A reconfiguration delivers the whole VBIN; the kernel splits it into chunks the
 * bridge reassembles.  This caps how large that in-memory accumulation may grow
 * so an abandoned / hostile never-terminating reconfig stream cannot exhaust
 * memory.  Sized generously above a real synthesized VBIN (vpp_sim + sibling
 * .so's + system_map.xml is a few-to-tens of MB) but bounded.  It must not exceed
 * the reconfiguration window size (the address space the chunks land in).
 */
#define EMU_BRIDGE_MAX_VBIN (256ull * 1024ull * 1024ull)

/* ================================================================== */
/* Per-device bridge                                                  */
/* ================================================================== */

struct emu_bridge {
    struct emu_device *dev;  /* borrowed (the owning device) */

    char *scratch_root;      /* owning: root under which the run dir is made */
    char *run_dir;           /* owning: this model's unpack/scratch dir, or NULL */
    char *endpoint;          /* owning: ipc:// endpoint string, or NULL */

    pid_t pid;               /* spawned model pid, or -1 if none running */
    struct emu_model_client *client; /* owning, or NULL if no model running */

    /* Serialises all model I/O for this device (the REQ/REP one-in-flight
     * discipline) so a future multi-threaded FUSE session stays correct. */
    pthread_mutex_t io_lock;

    /* Backend vtables we attach onto the endpoints while a model is running.
     * Their ctx is this bridge; they are detached (set NULL) on teardown. */
    struct emu_bar_backend bar_backend;
    struct emu_qdma_mem_backend mem_backend;

    /*
     * Reconfiguration reassembly (the kernel splits a >max_write VBIN into
     * several in-region chunks).  `acc` accumulates CONTIGUOUS reconfig-region
     * writes: a write at SLASH_RECONFIG_BASE (re)starts the buffer; a write at
     * BASE+acc_len appends.  Completion is detected from the ustar structure.
     * Reset (acc_len=0) between transfers; the buffer itself is kept for reuse.
     */
    uint8_t *acc;      /* owning accumulation buffer, or NULL */
    size_t acc_len;    /* bytes accumulated so far */
    size_t acc_cap;    /* allocated capacity of `acc` */
};

/* Element-free for the owning array below (defined later); forward-declared so
 * the array macro's static-inline _free body can name it. */
static void emu_bridge_free_one(struct emu_bridge *b);

DECLARE_OWNING_PTR_ARRAY(emu_bridge_array, struct emu_bridge *,
                         emu_bridge_free_one)

struct emu_bridge_registry {
    struct emu_bridge_array bridges;
};

/* ================================================================== */
/* Backend vtables: route bar/qdma ops to the running model            */
/* ================================================================== */

/*
 * BAR read backend (emu_bar_backend.read rc-contract: 0 model / >0 shadow /
 * <0 transport).  The SIM model is purely address-keyed 32-bit AXI-Lite over the
 * User region (BAR 0).  We forward only BAR 0, width <= 4 reads; everything else
 * (BAR 2/4, or a 64-bit slot the SIM dialect has no verb for) falls back to the
 * shadow (rc>0), which is always kept current, so an in-range read never yields
 * -EIO (G7).
 */
static int bridge_bar_read(void *ctx, uint32_t bar_index, off_t off,
                           size_t width, uint64_t *value)
{
    struct emu_bridge *b = ctx;

    if (bar_index != SLASH_BAR_USER_IDX || width > 4 || b->client == NULL) {
        return 1; /* fall back to the shadow */
    }

    pthread_mutex_lock(&b->io_lock);
    uint32_t v = 0;
    int rc = emu_model_scalar_read(b->client, (uint64_t) off, &v);
    pthread_mutex_unlock(&b->io_lock);

    if (rc < 0) {
        return rc; /* transport failure -> -ENODEV at the seam */
    }
    *value = v;
    return 0;
}

/*
 * BAR write backend (emu_bar_backend.write rc-contract: 0 ok / <0 transport;
 * the shadow is updated by the caller regardless).  Forward only BAR 0, width
 * <= 4 as reg{off,val}; a wider/other-BAR write is shadow-only (return 0 without
 * forwarding -- there is no SIM verb for it, and the shadow already holds it).
 */
static int bridge_bar_write(void *ctx, uint32_t bar_index, off_t off,
                            size_t width, uint64_t value)
{
    struct emu_bridge *b = ctx;

    if (bar_index != SLASH_BAR_USER_IDX || width > 4 || b->client == NULL) {
        return 0; /* shadow-only; nothing forwarded */
    }

    pthread_mutex_lock(&b->io_lock);
    int rc = emu_model_reg_write(b->client, (uint64_t) off, (uint32_t) value);
    pthread_mutex_unlock(&b->io_lock);

    return rc < 0 ? rc : 0;
}

/*
 * QDMA fetch backend (emu_qdma_mem_backend.fetch rc-contract: 0 model /
 * >0 store / <0 transport).  Always forward to the model; on transport failure
 * return the negative errno (-> -ENODEV).
 */
static int bridge_mem_fetch(void *ctx, uint64_t addr, void *buf, size_t len)
{
    struct emu_bridge *b = ctx;

    if (b->client == NULL) {
        return 1; /* no model: fall back to the store */
    }

    pthread_mutex_lock(&b->io_lock);
    int rc = emu_model_fetch(b->client, addr, buf, len);
    pthread_mutex_unlock(&b->io_lock);

    return rc < 0 ? rc : 0;
}

/*
 * QDMA populate backend (emu_qdma_mem_backend.populate rc-contract: 0 ok /
 * <0 transport; the store is kept current by the caller regardless).
 */
static int bridge_mem_populate(void *ctx, uint64_t addr, const void *buf,
                               size_t len)
{
    struct emu_bridge *b = ctx;

    if (b->client == NULL) {
        return 0; /* no model: store-only (caller already wrote the store) */
    }

    pthread_mutex_lock(&b->io_lock);
    int rc = emu_model_populate(b->client, addr, buf, len);
    pthread_mutex_unlock(&b->io_lock);

    return rc < 0 ? rc : 0;
}

/* ================================================================== */
/* Scratch dir + spawn                                                */
/* ================================================================== */

/* recursive rmdir via nftw, used to clean the run dir on teardown. */
static int rm_cb(const char *path, const struct stat *sb, int typeflag,
                 struct FTW *ftwbuf)
{
    (void) sb;
    (void) typeflag;
    (void) ftwbuf;
    return remove(path);
}

static void rmrf(const char *path)
{
    if (path == NULL) {
        return;
    }
    (void) nftw(path, rm_cb, 16, FTW_DEPTH | FTW_PHYS);
}

/* mkdir -p for a single path (parents assumed to exist or be creatable). */
static int mkdir_p(const char *path)
{
    char tmp[PATH_MAX];
    size_t n = strlen(path);
    if (n >= sizeof(tmp)) {
        return -ENAMETOOLONG;
    }
    memcpy(tmp, path, n + 1);

    for (char *p = tmp + 1; *p != '\0'; p++) {
        if (*p == '/') {
            *p = '\0';
            if (mkdir(tmp, 0700) != 0 && errno != EEXIST) {
                return -errno;
            }
            *p = '/';
        }
    }
    if (mkdir(tmp, 0700) != 0 && errno != EEXIST) {
        return -errno;
    }
    return 0;
}

/*
 * Build a unique per-run scratch dir under the bridge's scratch root and the
 * ipc:// endpoint inside it.  Uses the BDF + pid + a monotonic nonce so repeated
 * reconfigurations of the same device do not collide.
 */
static int bridge_make_run_dir(struct emu_bridge *b)
{
    static unsigned long nonce = 0;

    char dir[PATH_MAX];
    int n = snprintf(dir, sizeof(dir), "%s/%s.%d.%lu", b->scratch_root,
                     b->dev->bdf, (int) getpid(), nonce++);
    if (n < 0 || (size_t) n >= sizeof(dir)) {
        return -ENAMETOOLONG;
    }

    int rc = mkdir_p(dir);
    if (rc != 0) {
        LOG(LOG_ERR, "bridge: failed to create run dir %s: %s", dir,
            strerror(-rc));
        return rc;
    }

    char ep[PATH_MAX];
    n = snprintf(ep, sizeof(ep), "ipc://%s/model.sock", dir);
    if (n < 0 || (size_t) n >= sizeof(ep)) {
        rmrf(dir);
        return -ENAMETOOLONG;
    }

    b->run_dir = strdup(dir);
    b->endpoint = strdup(ep);
    if (b->run_dir == NULL || b->endpoint == NULL) {
        free(b->run_dir);
        b->run_dir = NULL;
        free(b->endpoint);
        b->endpoint = NULL;
        rmrf(dir);
        return -ENOMEM;
    }
    return 0;
}

/*
 * fork/exec the model executable UNSANDBOXED with SLASH_EMU_ENDPOINT set and the
 * cwd at the executable's directory (the real vpp_sim loads sibling .so's by
 * relative path; device.cpp does the same cd).  The endpoint (an ipc:// path
 * under the daemon's scratch, never under the FUSE mount) is the only channel.
 */
static int bridge_spawn(struct emu_bridge *b, const char *exec_path)
{
    /* Split exec_path into dir + file for the cwd-relative exec. */
    char path_copy[PATH_MAX];
    size_t n = strlen(exec_path);
    if (n >= sizeof(path_copy)) {
        return -ENAMETOOLONG;
    }
    memcpy(path_copy, exec_path, n + 1);

    char *slash = strrchr(path_copy, '/');
    const char *dir = ".";
    const char *file = path_copy;
    char rel[PATH_MAX];
    if (slash != NULL) {
        *slash = '\0';
        dir = path_copy;
        file = slash + 1;
    }
    int rn = snprintf(rel, sizeof(rel), "./%s", file);
    if (rn < 0 || (size_t) rn >= sizeof(rel)) {
        return -ENAMETOOLONG;
    }

    pid_t pid = fork();
    if (pid < 0) {
        return -errno;
    }
    if (pid == 0) {
        /* Child: become a session leader so we can signal the whole group, set
         * the endpoint env, cd into the exec dir, and exec it.  Any failure here
         * exits the child with a distinctive code; the parent detects via the
         * handshake timeout / waitpid. */
        if (chdir(dir) != 0) {
            _exit(127);
        }
        if (setenv("SLASH_EMU_ENDPOINT", b->endpoint, 1) != 0) {
            _exit(127);
        }
        char *const argv[] = { (char *) rel, NULL };
        execv(rel, argv);
        _exit(127); /* exec failed */
    }

    b->pid = pid;
    return 0;
}

/* Reap the child: SIGTERM, bounded wait, then SIGKILL.  Idempotent. */
static void bridge_reap(struct emu_bridge *b)
{
    if (b->pid <= 0) {
        return;
    }

    (void) kill(b->pid, SIGTERM);

    /* Bounded poll for the child to exit. */
    struct timespec slice = { .tv_sec = 0, .tv_nsec = 5 * 1000 * 1000 }; /* 5ms */
    int waited_ms = 0;
    bool reaped = false;
    while (waited_ms < EMU_BRIDGE_TERM_WAIT_MS) {
        int status = 0;
        pid_t r = waitpid(b->pid, &status, WNOHANG);
        if (r == b->pid || (r < 0 && errno == ECHILD)) {
            reaped = true;
            break;
        }
        nanosleep(&slice, NULL);
        waited_ms += 5;
    }

    if (!reaped) {
        (void) kill(b->pid, SIGKILL);
        (void) waitpid(b->pid, NULL, 0); /* blocking: SIGKILL is prompt */
    }

    b->pid = -1;
}

/* ================================================================== */
/* Teardown                                                           */
/* ================================================================== */

/*
 * Tear down a running model: detach backends, send exit (best-effort), reap the
 * child, close the client, remove the run dir.  Idempotent: a no-model bridge is
 * left untouched.  Runs with the tree lock held (seam contract) -- it only
 * touches bridge-private state and the borrowed device, never the spine's
 * re-locking API.
 */
static void bridge_teardown_model(struct emu_bridge *b)
{
    if (b == NULL) {
        return;
    }

    /* Detach the backends first so no new op routes to a model we are killing.
     * (The hooks also check b->client == NULL and fall back to the shadow/store,
     * so the NULLing of the client below is a second line of defence.) */
    if (b->dev != NULL) {
        (void) emu_qdma_set_mem_backend(b->dev, NULL);
        (void) emu_bars_set_backend(b->dev, NULL);
    }

    if (b->client != NULL) {
        pthread_mutex_lock(&b->io_lock);
        (void) emu_model_client_exit(b->client); /* best-effort, bounded */
        struct emu_model_client *c = b->client;
        b->client = NULL; /* disarm the bar backend hooks */
        pthread_mutex_unlock(&b->io_lock);
        emu_model_client_close(c);
    }

    bridge_reap(b);

    if (b->run_dir != NULL) {
        rmrf(b->run_dir);
        free(b->run_dir);
        b->run_dir = NULL;
    }
    free(b->endpoint);
    b->endpoint = NULL;
}

void emu_bridge_shutdown(struct emu_device *dev, void *ctx)
{
    (void) dev;
    struct emu_bridge *b = ctx;
    bridge_teardown_model(b);
}

/* ================================================================== */
/* Reconfiguration                                                    */
/* ================================================================== */

/* Discard any partial reassembly in progress (keeps the buffer for reuse). */
static void bridge_acc_reset(struct emu_bridge *b)
{
    b->acc_len = 0;
}

/*
 * Apply a COMPLETE VBIN: tear down any running model, unpack to a fresh run dir,
 * spawn the model unsandboxed, connect + handshake, and attach the backends.
 * This is the original single-shot reconfiguration flow; the chunk reassembler
 * (emu_bridge_reconfigure) calls it once the accumulated archive is complete.
 */
static int bridge_apply_vbin(struct emu_bridge *b, const void *vbin, size_t len)
{
    /* A new VBIN replaces any running model (idempotent teardown first). */
    bridge_teardown_model(b);

    int rc = bridge_make_run_dir(b);
    if (rc != 0) {
        return rc;
    }

    /* Unpack the VBIN and locate the vpp_sim executable. */
    char exec_path[PATH_MAX];
    rc = emu_vbin_unpack_find_sim(vbin, len, b->run_dir, exec_path,
                                  sizeof(exec_path));
    if (rc != 0) {
        LOG(LOG_ERR, "bridge: VBIN unpack/locate failed for '%s': %s",
            b->dev->bdf, strerror(-rc));
        rmrf(b->run_dir);
        free(b->run_dir);
        b->run_dir = NULL;
        free(b->endpoint);
        b->endpoint = NULL;
        return rc;
    }

    /* Spawn the model unsandboxed. */
    rc = bridge_spawn(b, exec_path);
    if (rc != 0) {
        LOG(LOG_ERR, "bridge: spawn failed for '%s': %s", b->dev->bdf,
            strerror(-rc));
        bridge_teardown_model(b);
        return rc;
    }

    /* Connect the client and run the bounded readiness handshake. */
    rc = emu_model_client_connect(b->endpoint, EMU_MODEL_DEFAULT_TIMEOUT_MS,
                                  &b->client);
    if (rc != 0) {
        LOG(LOG_ERR, "bridge: client connect failed for '%s': %s", b->dev->bdf,
            strerror(-rc));
        bridge_teardown_model(b);
        return rc;
    }

    rc = emu_model_client_start(b->client);
    if (rc != 0) {
        LOG(LOG_ERR, "bridge: model handshake failed for '%s': %s", b->dev->bdf,
            strerror(-rc));
        bridge_teardown_model(b);
        return rc;
    }

    /* Attach the data-plane backends so subsequent ops route to the model.  The
     * bar backend is attached by storing the vtable on the bridge and pointing
     * the qdma store at it; bars consults b->client at call time. */
    if (emu_qdma_set_mem_backend(b->dev, &b->mem_backend) != 0) {
        LOG(LOG_ERR, "bridge: failed to attach qdma backend for '%s'",
            b->dev->bdf);
        bridge_teardown_model(b);
        return -EIO;
    }
    if (emu_bars_set_backend(b->dev, &b->bar_backend) != 0) {
        LOG(LOG_ERR, "bridge: failed to attach bar backend for '%s'",
            b->dev->bdf);
        bridge_teardown_model(b);
        return -EIO;
    }

    LOG(LOG_INFO, "bridge: model up for '%s' (pid %d, %s)", b->dev->bdf,
        (int) b->pid, b->endpoint);
    return 0;
}

int emu_bridge_reconfigure(struct emu_bridge *b, uint64_t addr,
                           const void *vbin, size_t len)
{
    if (b == NULL || vbin == NULL || len == 0) {
        return -EINVAL;
    }

    /*
     * Reassemble contiguous reconfig-region chunks (the kernel splits a large
     * VBIN write).  A write at the window base (re)starts the buffer; a write at
     * the running BASE+acc_len offset appends.  Anything else is a non-contiguous
     * / seeking write into the reconfig region -- not how a VBIN is delivered --
     * so we reset any partial transfer and reject it (-EINVAL) rather than
     * silently corrupting the stream.
     */
    if (addr == SLASH_RECONFIG_BASE) {
        bridge_acc_reset(b); /* a fresh transfer (replaces any partial one) */
    } else if (addr != SLASH_RECONFIG_BASE + b->acc_len) {
        bridge_acc_reset(b);
        LOG(LOG_ERR,
            "bridge: non-contiguous reconfig write for '%s' (addr %#llx, "
            "expected %#llx)",
            b->dev->bdf, (unsigned long long) addr,
            (unsigned long long) (SLASH_RECONFIG_BASE + b->acc_len));
        return -EINVAL;
    }

    /* Cap the accumulation so an abandoned / never-terminating stream cannot
     * grow memory without bound. */
    if (b->acc_len + len > EMU_BRIDGE_MAX_VBIN) {
        bridge_acc_reset(b);
        LOG(LOG_ERR, "bridge: VBIN exceeds cap (%llu B) for '%s'",
            (unsigned long long) EMU_BRIDGE_MAX_VBIN, b->dev->bdf);
        return -EFBIG;
    }

    /* Grow the buffer if needed and append the chunk. */
    if (b->acc_len + len > b->acc_cap) {
        size_t want = b->acc_len + len;
        size_t newcap = b->acc_cap != 0 ? b->acc_cap : (1u << 20);
        while (newcap < want) {
            newcap *= 2;
        }
        if (newcap > EMU_BRIDGE_MAX_VBIN) {
            newcap = EMU_BRIDGE_MAX_VBIN;
        }
        uint8_t *grown = realloc(b->acc, newcap);
        if (grown == NULL) {
            bridge_acc_reset(b);
            return -ENOMEM;
        }
        b->acc = grown;
        b->acc_cap = newcap;
    }
    memcpy(b->acc + b->acc_len, vbin, len);
    b->acc_len += len;

    /*
     * Classify the accumulated archive: keep accepting chunks until the ustar
     * terminator is seen; reject a structurally-invalid stream up front (so a
     * bogus first chunk fails immediately, not after exhausting the cap).
     */
    enum emu_vbin_status st = emu_vbin_classify(b->acc, b->acc_len);
    if (st == EMU_VBIN_INVALID) {
        bridge_acc_reset(b);
        LOG(LOG_ERR, "bridge: malformed VBIN for '%s'", b->dev->bdf);
        return -EINVAL;
    }
    if (st == EMU_VBIN_INCOMPLETE) {
        return 0; /* accept the chunk; await the rest */
    }

    /* Complete: apply it, then reset the buffer regardless of outcome. */
    int rc = bridge_apply_vbin(b, b->acc, b->acc_len);
    bridge_acc_reset(b);
    return rc;
}

/* Reconfig hook trampoline registered on the qdma store. */
static int bridge_reconfig_hook(void *ctx, uint64_t addr, const void *vbin,
                                size_t len)
{
    return emu_bridge_reconfigure(ctx, addr, vbin, len);
}

/* ================================================================== */
/* Registry + attach                                                  */
/* ================================================================== */

static void emu_bridge_free_one(struct emu_bridge *b)
{
    if (b == NULL) {
        return;
    }
    bridge_teardown_model(b);
    pthread_mutex_destroy(&b->io_lock);
    free(b->scratch_root);
    free(b->acc); /* reassembly buffer */
    /* run_dir/endpoint freed by bridge_teardown_model */
    free(b);
}

int emu_bridge_registry_new(struct emu_bridge_registry **out)
{
    if (out == NULL) {
        return -1;
    }
    struct emu_bridge_registry *reg = calloc(1, sizeof(*reg));
    if (reg == NULL) {
        return -1;
    }
    reg->bridges = emu_bridge_array_init();
    *out = reg;
    return 0;
}

void emu_bridge_registry_free(struct emu_bridge_registry *reg)
{
    if (reg == NULL) {
        return;
    }
    emu_bridge_array_free(&reg->bridges); /* frees each bridge via free_one */
    free(reg);
}

int emu_bridge_attach(struct emu_bridge_registry *reg, struct emu_device *dev,
                      const char *scratch_root)
{
    if (reg == NULL || dev == NULL || dev->qdma == NULL) {
        return -1;
    }

    _cleanup_(cleanup_free) struct emu_bridge *b = calloc(1, sizeof(*b));
    PROPAGATE_ERROR_NULL_LOG(b, LOG_ERR, "Failed to allocate bridge for '%s'",
                             dev->bdf);

    b->dev = dev;
    b->pid = -1;
    b->client = NULL;
    if (pthread_mutex_init(&b->io_lock, NULL) != 0) {
        return -1;
    }

    const char *root = scratch_root != NULL ? scratch_root
                                            : EMU_BRIDGE_DEFAULT_SCRATCH;
    b->scratch_root = strdup(root);
    if (b->scratch_root == NULL) {
        pthread_mutex_destroy(&b->io_lock);
        return -1;
    }

    b->bar_backend = (struct emu_bar_backend) {
        .read = bridge_bar_read,
        .write = bridge_bar_write,
        .ctx = b,
    };
    b->mem_backend = (struct emu_qdma_mem_backend) {
        .fetch = bridge_mem_fetch,
        .populate = bridge_mem_populate,
        .ctx = b,
    };

    /* Wire the reconfig handler so a reconfig-region write routes here. */
    if (emu_qdma_set_reconfig_handler(dev, bridge_reconfig_hook, b) != 0) {
        LOG(LOG_ERR, "Failed to wire reconfig handler for '%s'", dev->bdf);
        pthread_mutex_destroy(&b->io_lock);
        free(b->scratch_root);
        return -1;
    }

    /* Replace the T9 model-shutdown seam default with the real teardown. */
    if (emu_device_set_model_shutdown(dev->tree, dev->bdf, emu_bridge_shutdown,
                                      b) != 0) {
        LOG(LOG_ERR, "Failed to wire model-shutdown seam for '%s'", dev->bdf);
        (void) emu_qdma_set_reconfig_handler(dev, NULL, NULL);
        pthread_mutex_destroy(&b->io_lock);
        free(b->scratch_root);
        return -1;
    }

    struct emu_bridge *kept = b;
    if (emu_bridge_array_push(&reg->bridges, b) == -1) {
        (void) emu_qdma_set_reconfig_handler(dev, NULL, NULL);
        (void) emu_device_set_model_shutdown(dev->tree, dev->bdf, NULL, NULL);
        pthread_mutex_destroy(&b->io_lock);
        free(b->scratch_root);
        return -1;
    }
    b = NULL; /* ownership transferred to the registry */
    (void) kept;

    return 0;
}
