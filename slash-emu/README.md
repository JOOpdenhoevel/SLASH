# slash-emu

`slash-emu` is the SLASH **system-emulation daemon**. It presents an emulated
SLASH device tree to the rest of the system as a FUSE filesystem, so software
can be developed and tested against the SLASH ABI without real AMD V80 FPGA
hardware attached. The tree it exposes under its mount path is structurally
identical to the kernel driver's `/dev/slash` tree (see
`driver/libslash/include/slash/uapi/slash_abi.h`), so the same UAPI targets real
hardware or the emulator unchanged.

This directory holds the **step-1 MVP**. It stands up the full daemon lifecycle
(CLI parsing, config load, FUSE mount, sd-event loop, signal handling, clean
teardown) in idiomatic C++20 (`namespace slash::emu`, RAII throughout),
materializes the per-device node tree over the
FUSE mount, and implements the full step-1 endpoint set plus a **SIM-only**
data-plane bridge that spawns and talks to a `vpp_sim` model.

## Endpoints

Per configured accelerator (a directory named by its PCI BDF, e.g.
`0000:61:00/`):

- **`info`** — read-only binary info struct (`struct slash_info`); `pread` only.
- **`bars/bar<M>`** — BARs `bar0` (User, 128 MiB), `bar2` (SL, 128 MiB), and
  `bar4` (clk, 512 KiB); `pread`/`pwrite` of register widths {1,2,4,8} only,
  aligned and in range. No `mmap` is offered (the low-level FUSE op set has no
  `.mmap` handler), and I/O is `direct_io` (unbuffered, exact-width). BAR
  shadows are lazily allocated on first write.
- **`qdma/`** — directory accepting the `QPAIR_ADD` ioctl.
- **`qdma/qpair<Q>`** — a qpair file (`pread`/`pwrite` MM transfers into the
  device store). Qpairs can be unlinked while open (nameless lifetime) and are
  refcounted across their registry ref and inode ref.

And one global file at the mount root:

- **`hotplug`** — accepts the hotplug ioctls (`RESCAN` / `REMOVE` / `TOGGLE_SBR`
  / `HOTPLUG`): per-function removal, device re-init, SBR/secondary-bus-reset
  reload sequencing, and rescan rediscovery of an individually-removed function.

## SIM data-plane bridge

A reconfiguration (a VBIN written into the reconfig region through a qpair) is
reassembled from chunks, unpacked (minimal ustar, with path-traversal
hardening), and the `vpp_sim` executable it contains is spawned **unsandboxed**.
The bridge then forwards BAR-0 register reads/writes and qdma MM transfers to the
running model over a per-device `ipc://` ZeroMQ channel, falling back to the
in-memory shadow/store for anything the SIM dialect has no verb for (BAR 2/4,
width-8 reads). Removing both functions tears the model down (exit + reap + close
+ scratch cleanup); `RESCAN` rediscovery re-wires a restored function to a still-
running model. See `docs/bridge-design.md` and `docs/bridge-protocol.md`.

## Layout

```
slash-emu/
├── CMakeLists.txt            # top-level: finds fuse3 + libsystemd + libzmq + jsoncpp, enable_testing()
├── README.md                 # this file
├── conf/
│   └── slash-emu.conf        # sample config (one [accelerator:<BDF>] section per device)
├── docs/
│   ├── bridge-design.md      # SIM bridge architecture (model, addressing, reconfig)
│   └── bridge-protocol.md    # the address-keyed SIM-dialect wire protocol + VBIN format
├── scripts/
│   └── emud-scratch.sh       # spin up a throwaway daemon over a real mount for hand-probing
├── src/                      # C++20, namespace slash::emu (one class/area per .hpp/.cpp)
│   ├── CMakeLists.txt        # slash_emu_core static lib + slash-emud executable
│   ├── utils.hpp             # LOG (journald) shim + SystemError (control-plane errno bridge)
│   ├── config.hpp / .cpp     # Config/Accelerator/RunningSet: parser + persistent model
│   ├── node.hpp / .cpp       # spine: NodeTree/Node/Device, shared_ptr-refcounted Resource,
│   │                         #   polymorphic NodeOps, revocation, model-shutdown seam (mutex-guarded)
│   ├── fs.hpp / .cpp         # Fs: libfuse3 LOW-LEVEL session (RAII) adapting ops to the node tree
│   ├── info.hpp / .cpp       # info endpoint (read-only struct slash_info)
│   ├── bars.hpp / .cpp       # bars/bar<M> endpoint + BAR access validation + shadow + BarBackend seam
│   ├── qdma.hpp / .cpp       # qdma/ + qpair<Q> endpoint, QPAIR_ADD, sparse store + QdmaMemBackend seam
│   ├── hotplug.hpp / .cpp    # global hotplug endpoint (RESCAN/REMOVE/TOGGLE_SBR/HOTPLUG)
│   ├── vbin.hpp / .cpp       # minimal ustar VBIN unpacker + chunk-stream classifier
│   ├── model_client.hpp/.cpp # ModelClient: cppzmq REQ client + JsonCpp for the SIM-dialect protocol
│   ├── bridge.hpp / .cpp     # Bridge/BridgeRegistry: spawn/teardown, reconfig, backend routing
│   └── main.cpp              # CLI args, sd-event loop, signals, mount/teardown
├── systemd/
│   └── slash-emu.service     # privileged unit (mounts /run/slash_emu)
├── sysusers/
│   └── slash-emu.conf        # client-access group (daemon itself runs as root)
└── tests/
    ├── CMakeLists.txt        # GTest via FetchContent, gtest_discover_tests; CI stub_model
    ├── smoke_test.cpp        # mount/browse/unmount end-to-end smoke test
    ├── daemon_lifecycle_test.cpp  # adversarial CLI / mount-failure / no-leak suite
    ├── config_test.cpp       # config parser + accelerator model unit tests
    ├── node_test.cpp / node_revocation_test.cpp   # spine: tree, registry, refcount, revocation
    ├── fs_tree_test.cpp      # end-to-end: configured device tree over the mount
    ├── info_test.cpp / info_adversarial_test.cpp  # info endpoint
    ├── bars_test.cpp / bars_adversarial_test.cpp / bars_oom_test.cpp   # bars endpoint
    ├── qdma_test.cpp / qdma_adversarial_test.cpp  # qdma endpoint
    ├── hotplug_test.cpp / hotplug_adversarial_test.cpp                 # hotplug endpoint
    ├── stub_model.cpp        # CI stand-in for vpp_sim (cppzmq + JsonCpp; bridge is tested against it)
    ├── bridge_test.cpp / bridge_integration_test.cpp / bridge_adversarial_test.cpp  # SIM bridge
    └── rescan_test.cpp / rescan_adversarial_test.cpp                   # RESCAN rediscovery
```

## Build and test

The daemon is **C++20**. It depends on libfuse3, libsystemd, libinih, libzmq
(via the header-only **cppzmq** API), and **JsonCpp** (the SIM-dialect wire
protocol is built/parsed with JsonCpp). All building and testing goes through
**CMake + CTest**.

```sh
cmake -S slash-emu -B slash-emu/build -DSLASH_EMU_BUILD_TESTS=ON
cmake --build slash-emu/build -j16
ctest --test-dir slash-emu/build --output-on-failure
```

Component tests are GTest (run via CTest); the daemon binary, the CI stub model,
and the scratch dir are passed to each test executable as compile definitions so
no test ever hand-runs or guesses a binary. The mount-based suites fork the
freshly-built `slash-emud` and drive it over a real FUSE mount under `.tmp/`. The
suite is `-Wall -Wextra -Werror` and ASan/UBSan clean
(`-DENABLE_SANITIZERS=ON`).

> FUSE mounting requires kernel FUSE support and permission to mount. In
> environments where unprivileged FUSE mounts are disallowed, run the test (or
> the daemon) with sufficient privileges.

## Running the daemon manually

```sh
slash-emud --config /etc/slash-emu/slash-emu.conf --mount /run/slash_emu
```

- `--config PATH` — configuration file (one `[accelerator:<BDF>]` section per
  device; see `conf/slash-emu.conf`).
- `--mount PATH` — mountpoint for the emulated tree (default `/run/slash_emu`).
- `SLASH_EMU_SCRATCH_ROOT` (env) — root under which the SIM bridge unpacks a
  reconfigured accelerator's VBIN and binds the spawned model's `ipc://` socket
  (defaults to `/run/slash_emu/.scratch`).

For interactive exploration over a throwaway mount (mmap behaviour, pread/pwrite
edge cases, a hand-poked reconfigure), use `scripts/emud-scratch.sh`, which
points the bridge scratch at `.tmp/` and tears everything down on exit:

```sh
scripts/emud-scratch.sh -- ls -l "$MNT"/0000:61:00/bars
```

## Design notes

- **Low-level FUSE API.** We use libfuse3's *low-level* interface
  (`fuse_session_new` + `fuse_lowlevel_ops`), not the high-level path-based one,
  because hotplug needs `fuse_lowlevel_notify_delete` /
  `fuse_lowlevel_notify_inval_entry`. The session's channel fd is integrated into
  the sd-event loop as an I/O source (rather than `fuse_session_loop`) so FUSE
  and daemon events share one loop.
- **Node tree (the spine).** The filesystem is a generic tree of inodes
  (the `Node` class in `node.hpp`), owned by a `NodeTree`. The FUSE ops in
  `fs.cpp` are thin adapters over the node model (`NodeTree::lookupChild` /
  `forget` / `stat` / `readdir` / `pread` / `pwrite` / `ioctl` dispatch); no
  op hard-codes an inode. Endpoints (info/bars/qdma/hotplug) attach their files
  and behaviour as polymorphic `NodeOps` subclasses via the node API without
  touching the session plumbing. The tree, per-device registry, and resource
  refcounts are `std::mutex`-guarded so the data model is safe for a
  multi-threaded session. See `node.hpp` for the full design and locking model.
- **Revocation & nameless qpairs.** Each device tracks live communication
  resources — including QDMA qpairs unlinked-while-open (nameless, unreachable by
  walking `qdma/`). A qpair is a `std::shared_ptr`-refcounted `Resource` whose
  lifetime is decoupled from its inode (a registry ref + an inode ref, freed when
  both drop), with idempotent teardown on cooperative inode eviction or forced
  removal. Forced
  removal eagerly revokes: new lookups get `-ENOENT`, ops on already-open fds get
  `-ENODEV`, names are invalidated via `fuse_lowlevel_notify_delete`, and `close`
  always succeeds. A per-device **model-shutdown seam** fires when both functions
  are removed, which the bridge wires to its model teardown.
- **SIM bridge, out-of-process model, unsandboxed.** The data-plane model
  (`vpp_sim`) runs as a spawned child over an `ipc://` channel; backend seams on
  the bars/qdma endpoints route ops to it with shadow/store fallback. This
  milestone spawns the model **unsandboxed** under the daemon's uid; the
  `vpp_emu` bridge and live streaming/work loop are **deferred to later tasks**.
- **Privileged service.** Unlike `vrtd`, the systemd unit is intentionally *not*
  hardened: it runs as root (FUSE needs `CAP_SYS_ADMIN`) and avoids mount-
  namespacing options that would hide the FUSE mount. Hardening is revisited once
  the model is fully sandboxed.
- **No watchdog yet (known later item).** slash-emud deliberately does not enable
  the systemd watchdog yet; it is to be wired (`sd_event_set_watchdog` +
  `WatchdogSec=`) when a streaming/work loop that could hang lands. See the
  `TODO` near the event loop in `main.cpp`.
- **Readiness ordering.** `READY=1` is signalled only after the FUSE mount is
  established, so `Type=notify` consumers and VRTD discovery can trust the
  emulated tree is browsable the moment the service reports started.
```