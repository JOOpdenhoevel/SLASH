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
 * @file main.cpp
 * @brief Entry point and event-loop wiring for the slash-emu daemon (C++20).
 *
 * slash-emud is the SLASH system-emulation daemon.  It mounts a FUSE filesystem
 * exposing an emulated SLASH device tree and serves requests off an sd-event loop.
 * This file stands up the daemon lifecycle (CLI parsing, config load, FUSE mount,
 * signal handling, clean teardown) and signals readiness only after the mount is
 * established.  Endpoints and the SIM bridge are wired by @ref slash::emu::Fs.
 *
 * The startup path is control-plane: failures surface as @ref slash::emu::SystemError
 * (or other exceptions) and are caught here, logged, and turned into a non-zero
 * exit.  RAII (the @ref slash::emu::Fs / @ref slash::emu::BridgeRegistry objects, the
 * sd_event handle) guarantees the mount is torn down on every exit path.
 */

#include <cstdio>
#include <cstdlib>
#include <exception>
#include <memory>
#include <optional>
#include <string>

#include <getopt.h>
#include <signal.h>

#include <systemd/sd-daemon.h>
#include <systemd/sd-event.h>
#include <systemd/sd-journal.h>

#include "bridge.hpp"
#include "config.hpp"
#include "fs.hpp"
#include "utils.hpp"

namespace {

/** @brief Default mountpoint when --mount is not supplied. */
constexpr const char *kDefaultMountpoint = "/run/slash_emu";

/** @brief Parsed command-line options. */
struct Options {
    std::optional<std::string> config_path;
    std::string mountpoint = kDefaultMountpoint;
};

void usage(const char *argv0, FILE *out)
{
    (void) std::fprintf(out,
        "Usage: %s [--config PATH] [--mount PATH]\n"
        "\n"
        "  --config PATH   Path to the slash-emu configuration file.\n"
        "  --mount PATH    Directory to mount the emulated device tree on\n"
        "                  (default: %s).\n"
        "  --help          Show this help and exit.\n",
        argv0, kDefaultMountpoint);
}

/** @return true on success, false to exit failure. */
bool parseArgs(int argc, char **argv, Options &opts)
{
    enum {
        OptConfig = 'c',
        OptMount = 'm',
        OptHelp = 'h',
    };

    static const struct option long_opts[] = {
        { "config", required_argument, nullptr, OptConfig },
        { "mount",  required_argument, nullptr, OptMount },
        { "help",   no_argument,       nullptr, OptHelp },
        { nullptr,  0,                 nullptr, 0 },
    };

    int c;
    while ((c = getopt_long(argc, argv, "c:m:h", long_opts, nullptr)) != -1) {
        switch (c) {
        case OptConfig:
            opts.config_path = optarg;
            break;
        case OptMount:
            opts.mountpoint = optarg;
            break;
        case OptHelp:
            usage(argv[0], stdout);
            std::exit(EXIT_SUCCESS);
        default:
            usage(argv[0], stderr);
            return false;
        }
    }

    if (optind < argc) {
        (void) std::fprintf(stderr, "%s: unexpected argument '%s'\n", argv[0],
                            argv[optind]);
        usage(argv[0], stderr);
        return false;
    }

    return true;
}

/*
 * Signal handler dispatched via sd_event's signalfd integration.  SIGINT/SIGTERM/
 * SIGQUIT request a clean exit from the event loop so RAII destructors run and the
 * FUSE filesystem is unmounted; STOPPING=1 is sent afterwards in run().
 */
int onSignal(sd_event_source *s, const struct signalfd_siginfo *si, void *userdata)
{
    (void) userdata;
    LOG(LOG_INFO, "Received signal %s (%u), shutting down",
        sigabbrev_np(si->ssi_signo), si->ssi_signo);
    sd_event_exit(sd_event_source_get_event(s), 0);
    return 0;
}

void configureSignals(sd_event *ev)
{
    struct sigaction sa_ignore = {};
    sa_ignore.sa_handler = SIG_IGN;
    if (sigemptyset(&sa_ignore.sa_mask) == -1) {
        slash::emu::throwErrno("Error manipulating signal set");
    }
    if (sigaction(SIGPIPE, &sa_ignore, nullptr) == -1) {
        slash::emu::throwErrno("Failed to ignore SIGPIPE");
    }

    int signals[] = { SIGINT, SIGTERM, SIGQUIT };

    sigset_t set;
    sigemptyset(&set);
    for (int sig : signals) {
        sigaddset(&set, sig);
    }
    if (sigprocmask(SIG_BLOCK, &set, nullptr) == -1) {
        slash::emu::throwErrno("Failed to mask signals");
    }

    for (int sig : signals) {
        int ret = sd_event_add_signal(ev, nullptr, sig, onSignal, nullptr);
        if (ret < 0) {
            slash::emu::throwErrno(-ret, "Failed to add signal source");
        }
    }
}

/** @brief RAII deleter for sd_event. */
struct SdEventDeleter {
    void operator()(sd_event *ev) const { sd_event_unref(ev); }
};
using SdEventPtr = std::unique_ptr<sd_event, SdEventDeleter>;

int run(const Options &opts)
{
    LOG(LOG_INFO, "Starting slash-emud (mount=%s)", opts.mountpoint.c_str());

    slash::emu::Config config = slash::emu::Config::load(opts.config_path);

    sd_event *ev_raw = nullptr;
    int ret = sd_event_default(&ev_raw);
    if (ret < 0) {
        slash::emu::throwErrno(-ret, "Failed to allocate event loop");
    }
    SdEventPtr ev(ev_raw);

    configureSignals(ev.get());

    slash::emu::BridgeRegistry bridges;
    slash::emu::Fs fs(opts.mountpoint, config, bridges);
    fs.attach(ev.get());

    /*
     * Readiness is signalled only AFTER the FUSE mount (in the Fs constructor) has
     * succeeded -- both are on the straight-line path and any failure throws before
     * we get here.  So when a Type=notify consumer observes READY=1, the emulated
     * tree is browsable.  Best-effort: outside systemd sd_notify returns 0.
     *
     * TODO(later): once the daemon grows a work/streaming loop, enable the systemd
     * watchdog here (sd_event_set_watchdog + WatchdogSec=), as vrtd does.
     */
    (void) sd_notify(0, "READY=1");

    ret = sd_event_loop(ev.get());
    if (ret < 0) {
        (void) sd_notify(0, "STOPPING=1");
        slash::emu::throwErrno(-ret, "Event loop failed");
    }

    (void) sd_notify(0, "STOPPING=1");
    LOG(LOG_INFO, "slash-emud stopped");
    return EXIT_SUCCESS;
}

} // namespace

int main(int argc, char **argv)
{
    Options opts;
    if (!parseArgs(argc, argv, opts)) {
        return EXIT_FAILURE;
    }

    try {
        return run(opts);
    } catch (const std::exception &e) {
        LOG(LOG_CRIT, "Fatal: %s", e.what());
        return EXIT_FAILURE;
    }
}
