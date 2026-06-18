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
# run_conformance.sh -- CTest runner for the shared SLASH ABI conformance suite.
#
# Brings up the FUSE emulation daemon backend (via the standard emud-scratch.sh),
# points the suite's mount-root env var at the live mount, runs the kselftest
# conformance binary against it, then tears the daemon down cleanly.  The suite
# itself is backend-neutral: to run it against a DIFFERENT backend (e.g. the
# future kernel module, architecture step 6) you do NOT use this script -- you
# mount that backend yourself and run the binary directly:
#
#     SLASH_CONFORMANCE_MOUNT=/dev/slash ./slash_abi_conformance
#
# i.e. this script only knows how to stand up the FUSE daemon; the binary knows
# nothing about FUSE.
#
# Arguments (positional, supplied by CMake):
#   $1  path to the slash_abi_conformance kselftest binary
#   $2  path to slash-emud (the daemon binary)
#   $3  path to the CI stub model (packed as the VBIN "vpp_sim" by the suite)
#
# Environment:
#   SLASH_CONFORMANCE_BDF   accelerator BDF to configure (default 0000:61:00).
#
# All scratch lives under <slash-emu>/.tmp (never /tmp), per project convention;
# emud-scratch.sh handles mount/config/model-scratch teardown on exit.

set -u

CONF_BIN="${1:?conformance binary path required}"
EMUD="${2:?slash-emud path required}"
STUB="${3:?stub model path required}"

BDF="${SLASH_CONFORMANCE_BDF:-0000:61:00}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
EMU_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
SCRATCH="$EMU_ROOT/scripts/emud-scratch.sh"

if [[ ! -x "$CONF_BIN" ]]; then
    echo "conformance binary not executable: $CONF_BIN" >&2
    exit 1
fi
if [[ ! -x "$SCRATCH" ]]; then
    echo "emud-scratch.sh not found/executable: $SCRATCH" >&2
    exit 1
fi

# Hand the daemon binary to emud-scratch.sh explicitly so it does not have to
# guess which build dir to search.  The scratch script exports $MNT (the live
# mount root) into the inner command's environment; we run a tiny bash -c that
# turns it into the suite's parameterisation env vars and execs the kselftest
# binary.  The binary's exit status (nonzero on any failed test) propagates out
# through emud-scratch.sh, becoming this script's -- and thus ctest's -- status.
export SLASH_EMUD="$EMUD"
export SLASH_CONFORMANCE_STUB_MODEL="$STUB"
exec "$SCRATCH" -b "$BDF" -- \
    bash -c 'export SLASH_CONFORMANCE_MOUNT="$MNT"; exec "$1"' _ "$CONF_BIN"
