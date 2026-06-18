#!/usr/bin/env bash
# ##################################################################################################
#  The MIT License (MIT)
#  Copyright (c) 2026 Advanced Micro Devices, Inc. All rights reserved.
#
#  Permission is hereby granted, free of charge, to any person obtaining a copy of this software
#  and associated documentation files (the "Software"), to deal in the Software without restriction,
#  including without limitation the rights to use, copy, modify, merge, publish, distribute,
#  sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
#  furnished to do so, subject to the following conditions:
#
#  The above copyright notice and this permission notice shall be included in all copies or
#  substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
# NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
# NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
# DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
# ##################################################################################################
#
# emud-scratch.sh -- spin up a throwaway slash-emud instance for manual probing,
# optionally run a command against the live mount, then tear everything down
# cleanly (SIGTERM the daemon, unmount, reap, remove scratch files).
#
# This is a developer convenience for poking at the real FUSE mount by hand
# (mmap behavior, pread/pwrite edge cases, readdir, ...).  Automated coverage
# still lives in the GTest/CTest suites; this is for exploration, not CI.
#
# USAGE
#   scripts/emud-scratch.sh [options]                 # mount, print path, wait for ^C
#   scripts/emud-scratch.sh [options] -- CMD [ARG...] # mount, run CMD, then tear down
#
# The mountpoint is exported to CMD as $MNT, so e.g.:
#   scripts/emud-scratch.sh -- ls -l "$MNT"/0000:61:00/bars
#   scripts/emud-scratch.sh -- bash -c 'xxd -l16 "$MNT"/0000:61:00/info'
#   scripts/emud-scratch.sh -- timeout 5 ./my-probe "$MNT"/0000:61:00/bars/bar0
# The exit status of the wrapper is CMD's exit status (handy in scripts).
#
# OPTIONS
#   -b, --bdf BDF       Accelerator BDF to configure (default: 0000:61:00).
#                       Repeatable to configure several accelerators.
#   -B, --build DIR     Build dir to find slash-emud in (default: autodetect
#                       under <repo>/build, then <repo>/.tmp/*build*).
#   -c, --config FILE   Use an existing config file instead of generating one.
#   -m, --mount DIR     Use an existing (empty) mountpoint instead of mktemp.
#   -t, --timeout SEC   Max seconds to wait for the mount to appear (default 5).
#   -k, --keep          Do NOT tear down on exit (leave mount + config; prints
#                       how to clean up).  Useful to attach more tools by hand.
#   -h, --help          Show this help.
#
# ENVIRONMENT
#   SLASH_EMUD          Explicit path to the slash-emud binary (overrides search).
#
# All generated scratch lives under <repo>/slash-emu/.tmp (never /tmp), matching
# the project test convention.  This includes the SIM bridge's per-device model
# runtime dirs (the VBIN unpack dir + the ipc:// socket for a spawned vpp_sim):
# the script points SLASH_EMU_SCRATCH_ROOT at a dir under .tmp so a reconfigured
# accelerator's model leaves nothing outside .tmp, and the teardown removes it.
set -u

# --------------------------------------------------------------------------- #
# Locate the repo (this script lives in <repo>/slash-emu/scripts/).
# --------------------------------------------------------------------------- #
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
EMU_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"   # the slash-emu/ dir
TMP="$EMU_ROOT/.tmp"

# --------------------------------------------------------------------------- #
# Defaults + arg parsing.
# --------------------------------------------------------------------------- #
declare -a BDFS=()
BUILD_DIR=""
CONFIG=""
MOUNT=""
WAIT_TIMEOUT=5
KEEP=0

usage() { sed -n '24,59p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; }

CMD=()
while [[ $# -gt 0 ]]; do
    case "$1" in
        -b|--bdf)     BDFS+=("$2"); shift 2 ;;
        -B|--build)   BUILD_DIR="$2"; shift 2 ;;
        -c|--config)  CONFIG="$2"; shift 2 ;;
        -m|--mount)   MOUNT="$2"; shift 2 ;;
        -t|--timeout) WAIT_TIMEOUT="$2"; shift 2 ;;
        -k|--keep)    KEEP=1; shift ;;
        -h|--help)    usage; exit 0 ;;
        --)           shift; CMD=("$@"); break ;;
        *) echo "unknown option: $1 (try --help)" >&2; exit 2 ;;
    esac
done

[[ ${#BDFS[@]} -eq 0 ]] && BDFS=("0000:61:00")

# --------------------------------------------------------------------------- #
# Locate the daemon binary.
# --------------------------------------------------------------------------- #
EMUD="${SLASH_EMUD:-}"
if [[ -z "$EMUD" ]]; then
    declare -a SEARCH=()
    [[ -n "$BUILD_DIR" ]] && SEARCH+=("$BUILD_DIR")
    SEARCH+=("$EMU_ROOT/build" "$TMP")
    for d in "${SEARCH[@]}"; do
        [[ -d "$d" ]] || continue
        EMUD="$(find "$d" -name slash-emud -type f 2>/dev/null | head -n1)"
        [[ -n "$EMUD" ]] && break
    done
fi
if [[ -z "${EMUD:-}" || ! -x "$EMUD" ]]; then
    echo "slash-emud not found (looked under build/ and .tmp/); build first or set SLASH_EMUD" >&2
    exit 1
fi

# --------------------------------------------------------------------------- #
# Build the config + mountpoint (unless caller supplied them).
# --------------------------------------------------------------------------- #
mkdir -p "$TMP"

GEN_CONFIG=0
if [[ -z "$CONFIG" ]]; then
    CONFIG="$(mktemp "$TMP/scratch_cfg_XXXXXX")"
    GEN_CONFIG=1
    n=1
    for bdf in "${BDFS[@]}"; do
        printf '[accelerator:%s]\nnet-ip = 10.0.0.%d\n' "$bdf" "$n" >> "$CONFIG"
        n=$((n + 1))
    done
fi

GEN_MOUNT=0
if [[ -z "$MOUNT" ]]; then
    MOUNT="$(mktemp -d "$TMP/scratch_mnt_XXXXXX")"
    GEN_MOUNT=1
fi

# SIM bridge (T10) model runtime scratch: the daemon unpacks a reconfigured
# accelerator's VBIN here and binds the spawned vpp_sim's ipc:// socket here.
# Keep it under .tmp (never the bridge default /run/...) so a manual reconfigure
# leaves nothing outside the repo, and remove it on teardown.
MODEL_SCRATCH="$(mktemp -d "$TMP/scratch_model_XXXXXX")"

DPID=""
cleanup() {
    if [[ "$KEEP" -eq 1 ]]; then
        echo "--keep: leaving mount '$MOUNT' (daemon pid ${DPID:-?})." >&2
        echo "  model scratch: '$MODEL_SCRATCH'" >&2
        echo "  clean up with: fusermount3 -u '$MOUNT'; kill ${DPID:-?}; rm -rf '$MOUNT' '$MODEL_SCRATCH' ${GEN_CONFIG:+'$CONFIG'}" >&2
        return
    fi
    [[ -n "$DPID" ]] && kill -TERM "$DPID" 2>/dev/null
    sleep 0.3
    fusermount3 -u "$MOUNT" 2>/dev/null
    [[ -n "$DPID" ]] && kill -9 "$DPID" 2>/dev/null
    [[ -n "$DPID" ]] && wait "$DPID" 2>/dev/null
    [[ "$GEN_MOUNT" -eq 1 ]] && rm -rf "$MOUNT"
    rm -rf "$MODEL_SCRATCH"
    [[ "$GEN_CONFIG" -eq 1 ]] && rm -f "$CONFIG"
}
trap cleanup EXIT INT TERM

# --------------------------------------------------------------------------- #
# Launch + wait for the mount to come up.
# --------------------------------------------------------------------------- #
SLASH_EMU_SCRATCH_ROOT="$MODEL_SCRATCH" "$EMUD" --config "$CONFIG" --mount "$MOUNT" &
DPID=$!

deadline=$(( WAIT_TIMEOUT * 20 ))   # 50ms ticks
ready=0
for _ in $(seq 1 "$deadline"); do
    if ! kill -0 "$DPID" 2>/dev/null; then
        echo "daemon exited before mounting (check config '$CONFIG')" >&2
        exit 1
    fi
    if mountpoint -q "$MOUNT" 2>/dev/null; then ready=1; break; fi
    sleep 0.05
done
if [[ "$ready" -ne 1 ]]; then
    echo "mount did not come up within ${WAIT_TIMEOUT}s at '$MOUNT'" >&2
    exit 1
fi

export MNT="$MOUNT"
echo "mounted at: $MOUNT  (accelerators: ${BDFS[*]}, daemon pid: $DPID)"

# --------------------------------------------------------------------------- #
# Run the command (if any), else wait for the daemon (^C tears down).
# --------------------------------------------------------------------------- #
if [[ ${#CMD[@]} -gt 0 ]]; then
    "${CMD[@]}"
    rc=$?
    exit "$rc"
fi

echo "press Ctrl-C to unmount and exit"
wait "$DPID"
