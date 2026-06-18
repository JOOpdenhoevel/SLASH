# SLASH ABI conformance suite

`slash_abi_conformance.c` is **the single source of truth for the SLASH
filesystem ABI** (see `abi_rebuild_architecture.md`, "Testing and Code quality").
It is the one deliberate exception to the project's GTest rule: it is written
against the kernel **kselftest** harness (`driver/tests/kselftest_harness.h`) so
the *same, unchanged* suite can run against **both** backends and prove they do
not drift:

* the FUSE **system-emulation daemon** (`slash-emud`) — what CI exercises today;
* the in-kernel SLASH filesystem — architecture **step 6**, future.

## What makes it backend-neutral

The suite is a pure **black box over the mount path**. It uses only:

* the shared UAPI header `slash/uapi/slash_abi.h`, and
* raw syscalls (`open`/`read`/`pread`/`pwrite`/`ioctl`/`unlink`/`mmap`/`readdir`/
  `lseek`/`stat`).

It includes **no** daemon-internal headers, **no** libfuse, and assumes nothing
about the backend being FUSE. The device under test is **discovered** by
`readdir` of the mount root (the single non-`hotplug` directory entry), so no BDF
is hard-coded.

## Parameterisation (env vars)

| Variable | Default | Meaning |
|---|---|---|
| `SLASH_CONFORMANCE_MOUNT` | `/run/slash_emu` | Mount root to test. |
| `SLASH_CONFORMANCE_STUB_MODEL` | *(unset)* | Path to a model executable packed as the VBIN member `vpp_sim` for the reconfiguration round-trip. If unset, the reconfiguration tests **SKIP**. |
| `SLASH_CONFORMANCE_BDF` | `0000:61:00` | (Runner only) the accelerator BDF the FUSE daemon is configured with. |

## Running it via CTest (FUSE daemon backend)

Built and wired into the top-level slash-emu CMake. From `slash-emu/build`:

```sh
ctest -R conformance --output-on-failure
```

CTest invokes `run_conformance.sh`, which stands up `slash-emud` through the
standard `scripts/emud-scratch.sh` (mount + config + per-model scratch all under
`slash-emu/.tmp`), points `SLASH_CONFORMANCE_MOUNT` at the live mount, runs the
kselftest binary, and tears the daemon down. The test is registered **serial**
(`RUN_SERIAL`) because it stands up a real mount, and carries a CTest `TIMEOUT`
so a wedge fails rather than blocks. The kselftest harness additionally bounds
each individual test with its own `alarm()`.

## Running it against a different backend (e.g. the kernel module)

`run_conformance.sh` only knows how to stand up the FUSE daemon. For any other
backend you mount it yourself and run the binary directly — that is the whole
point of the parameterisation:

```sh
# Kernel module example (step 6): mount the SLASH filesystem, then:
SLASH_CONFORMANCE_MOUNT=/dev/slash ./slash_abi_conformance
```

The reconfiguration tests SKIP unless `SLASH_CONFORMANCE_STUB_MODEL` is also
set, since the VBIN-stub mechanism is a daemon-side CI affordance.

## Contracts covered

* **info** — read-only binary `slash_info`; `size` is `[out]`; full read asserts
  `acc_type` has `SYSTEM_EMULATED` set and a board-level `bdf`; file size equals
  `sizeof(slash_info)`; short read = valid prefix; offset read; read at/after EOF
  returns 0.
* **bars** — bar0/2/4 present with file size == BAR size; bar1/3/5 absent
  (`ENOENT`); pread/pwrite round-trip at widths {1,2,4,8}; bad width / misaligned
  → `EINVAL`; out-of-range rejected (not clamped); in-range virgin read = 0
  (never `EIO`); mmap `MAP_SHARED` → `ENODEV` at `mmap()`; `MAP_PRIVATE` deref
  faults promptly (SIGBUS) under a watchdog and never hangs.
* **qdma** — `QPAIR_ADD` yields a QID and the `qpair<Q>` file; MM round-trip into
  HBM and DDR; out-of-range and reconfig-region read → `ERANGE`; ST mode and CMPT
  → `EOPNOTSUPP`; parameter validation (no/undefined direction bit, out-of-range
  ring index, unknown mode) → `EINVAL`, with a minimal-valid positive case so the
  `EINVAL` gate is not vacuous. MM only (streaming deferred).
* **hotplug** — all four ioctls with sysemu semantics; per-function REMOVE
  (`.1`=qdma, `.2`=bars, the sibling stays live); REMOVE idempotent; HOTPLUG /
  TOGGLE_SBR re-init; RESCAN restores; malformed BDF (no/short/non-digit
  function suffix) → `EINVAL`; a well-formed but non-removable function (`.0`,
  `.3`) → `EOPNOTSUPP` (distinct from the malformed-syntax `EINVAL`).
* **reconfiguration** — a single write of a whole stub VBIN triggers reconfig and
  the data plane then round-trips through the model; a single write of a
  **multi-MiB** VBIN forces the kernel to fragment it and proves the daemon
  **reassembles the contiguous chunks** (the T10 reassembly contract) by
  round-tripping through the reassembled model; a non-contiguous (seeking)
  reconfig-region write → `EINVAL` and the daemon then still accepts a fresh VBIN;
  reconfig-region read → `ERANGE`; a malformed VBIN fails the write (`EINVAL`)
  without wedging the daemon (a later transfer still works, and recovery reconfig
  still spawns).
* **revocation** (the headline requirement) — with an fd open on bar / qpair /
  info, REMOVE the owning function(s); ops on the open fd → `ENODEV`; reopen →
  `ENOENT`; `close()` always succeeds. Covered for all three endpoint kinds.
* **unlinkability** — only `qpair<Q>` is unlinkable (and the fd survives the
  unlink); unlink of info / bar / the global hotplug file → `EPERM` and the file
  survives.
