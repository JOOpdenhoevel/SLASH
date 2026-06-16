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
 * @file main.c
 * @brief Entry point and event-loop wiring for the slash-emu daemon.
 *
 * slash-emud is the SLASH system-emulation daemon.  It mounts a FUSE
 * filesystem exposing an emulated SLASH device tree and serves requests off an
 * sd-event loop.  This is the step-1 MVP scaffold: it stands up the daemon
 * lifecycle (CLI parsing, config load, FUSE mount, signal handling, clean
 * teardown) and serves an empty root directory.  Endpoints, the SIM model
 * bridge, and streaming are added by later tasks.
 */

#define _GNU_SOURCE

#include <getopt.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <syslog.h>

#include <systemd/sd-daemon.h>
#include <systemd/sd-event.h>
#include <systemd/sd-journal.h>

#include "config.h"
#include "fs.h"
#include "utils.h"

/** @brief Default mountpoint when --mount is not supplied. */
#define EMU_DEFAULT_MOUNTPOINT "/run/slash_emu"

/** @brief Parsed command-line options. */
struct emu_options {
    /** @brief Path to the config file (non-owning; points into argv). */
    const char *config_path;
    /** @brief Mountpoint for the FUSE filesystem (non-owning; points into argv). */
    const char *mountpoint;
};

static void usage(const char *argv0, FILE *out)
{
    (void) fprintf(out,
        "Usage: %s [--config PATH] [--mount PATH]\n"
        "\n"
        "  --config PATH   Path to the slash-emu configuration file.\n"
        "  --mount PATH    Directory to mount the emulated device tree on\n"
        "                  (default: %s).\n"
        "  --help          Show this help and exit.\n",
        argv0, EMU_DEFAULT_MOUNTPOINT);
}

static int parse_args(int argc, char **argv, struct emu_options *opts)
{
    *opts = (struct emu_options) {
        .config_path = NULL,
        .mountpoint = EMU_DEFAULT_MOUNTPOINT,
    };

    enum {
        OPT_CONFIG = 'c',
        OPT_MOUNT = 'm',
        OPT_HELP = 'h',
    };

    static const struct option long_opts[] = {
        { "config", required_argument, NULL, OPT_CONFIG },
        { "mount",  required_argument, NULL, OPT_MOUNT },
        { "help",   no_argument,       NULL, OPT_HELP },
        { 0 },
    };

    int c;
    while ((c = getopt_long(argc, argv, "c:m:h", long_opts, NULL)) != -1) {
        switch (c) {
        case OPT_CONFIG:
            opts->config_path = optarg;
            break;
        case OPT_MOUNT:
            opts->mountpoint = optarg;
            break;
        case OPT_HELP:
            usage(argv[0], stdout);
            exit(EXIT_SUCCESS);
        default:
            usage(argv[0], stderr);
            return -1;
        }
    }

    if (optind < argc) {
        (void) fprintf(stderr, "%s: unexpected argument '%s'\n",
                       argv[0], argv[optind]);
        usage(argv[0], stderr);
        return -1;
    }

    return 0;
}

/*
 * Signal handler dispatched via sd_event's signalfd integration.  SIGINT and
 * SIGTERM request a clean exit from the event loop so that destructors run and
 * the FUSE filesystem is unmounted; STOPPING=1 is sent afterwards in main().
 */
static int on_signal(sd_event_source *s, const struct signalfd_siginfo *si,
                     void *userdata)
{
    (void) userdata;

    LOG(LOG_INFO, "Received signal %s (%u), shutting down",
        sigabbrev_np(si->ssi_signo), si->ssi_signo);

    sd_event_exit(sd_event_source_get_event(s), 0);

    return 0;
}

static int configure_signals(sd_event *ev)
{
    struct sigaction sa_ignore = { .sa_handler = SIG_IGN };
    int ret = sigemptyset(&sa_ignore.sa_mask);
    PROPAGATE_ERROR_STDC_LOG(ret, LOG_ERR, "Error manipulating signal set");

    ret = sigaction(SIGPIPE, &sa_ignore, NULL);
    PROPAGATE_ERROR_STDC_LOG(ret, LOG_ERR, "Failed to ignore SIGPIPE");

    int signals[] = { SIGINT, SIGTERM, SIGQUIT };

    sigset_t set;
    sigemptyset(&set);
    for (size_t i = 0; i < SIZEOF_ARRAY(signals); i++) {
        sigaddset(&set, signals[i]);
    }
    ret = sigprocmask(SIG_BLOCK, &set, NULL);
    PROPAGATE_ERROR_STDC_LOG(ret, LOG_CRIT, "Failed to mask signals");

    for (size_t i = 0; i < SIZEOF_ARRAY(signals); i++) {
        ret = sd_event_add_signal(ev, NULL, signals[i], on_signal, NULL);
        PROPAGATE_ERROR_SD_LOG(ret, LOG_ERR, "Failed to add signal source: %s",
                               sigabbrev_np(signals[i]));
    }

    return 0;
}

int main(int argc, char **argv)
{
    struct emu_options opts;
    if (parse_args(argc, argv, &opts) == -1) {
        return EXIT_FAILURE;
    }

    LOG(LOG_INFO, "Starting slash-emud (mount=%s)", opts.mountpoint);

    _cleanup_(cleanup_configp)
    struct emu_config *config = NULL;
    if (emu_config_load(opts.config_path, &config) == -1) {
        LOG(LOG_CRIT, "Failed to load configuration");
        return EXIT_FAILURE;
    }

    _cleanup_(sd_event_unrefp)
    sd_event *ev = NULL;
    int ret = sd_event_default(&ev);
    if (ret < 0) {
        LOG(LOG_CRIT, "Failed to allocate event loop: %s",
            strerrordesc_np(-ret));
        return EXIT_FAILURE;
    }

    if (configure_signals(ev) == -1) {
        LOG(LOG_CRIT, "Failed to configure signals");
        return EXIT_FAILURE;
    }

    _cleanup_(cleanup_fsp)
    struct emu_fs *fs = NULL;
    if (emu_fs_create(&fs, opts.mountpoint, config) == -1) {
        LOG(LOG_CRIT, "Failed to create FUSE session");
        return EXIT_FAILURE;
    }

    if (emu_fs_attach(fs, ev) == -1) {
        LOG(LOG_CRIT, "Failed to attach FUSE session to event loop");
        return EXIT_FAILURE;
    }

    /*
     * Readiness is signalled only AFTER emu_fs_create() (and thus
     * fuse_session_mount()) has succeeded above -- both calls are on the
     * straight-line path and any failure returns before we get here.  So when a
     * Type=notify consumer (systemd, or VRTD device discovery) observes
     * READY=1, the FUSE mount is already established and the emulated device
     * tree is browsable; they can act on it without racing the mount.
     *
     * Best-effort: when run outside systemd (e.g. the smoke test) there is no
     * notification socket and sd_notify returns 0, which is fine.
     *
     * TODO(later): once the daemon grows a work/streaming loop and endpoints,
     * enable the systemd watchdog here (sd_event_set_watchdog + WatchdogSec= in
     * the unit), as vrtd does, so a wedged daemon gets restarted.  It is
     * deliberately omitted in this scaffold: there is no work loop yet that
     * could hang, so a watchdog would only add a liveness ping with nothing to
     * guard.
     */
    (void) sd_notify(0, "READY=1");

    ret = sd_event_loop(ev);
    if (ret < 0) {
        LOG(LOG_CRIT, "Event loop failed: %s", strerrordesc_np(-ret));
        (void) sd_notify(0, "STOPPING=1");
        return EXIT_FAILURE;
    }

    (void) sd_notify(0, "STOPPING=1");

    LOG(LOG_INFO, "slash-emud stopped");

    return EXIT_SUCCESS;
}
