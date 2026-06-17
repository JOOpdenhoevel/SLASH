# slash-emu

`slash-emu` is the SLASH **system-emulation daemon**. It presents an emulated
SLASH device tree to the rest of the system as a FUSE filesystem, so software
can be developed and tested against the SLASH ABI without real AMD V80 FPGA
hardware attached.

This directory holds the **step-1 MVP** under construction. It stands up the
full daemon lifecycle (CLI parsing, config load, FUSE mount, sd-event loop,
signal handling, clean teardown) in the vrtd house style, parses the accelerator
configuration, and materializes the per-device node tree (`<BDF>/` with `bars/`
and `qdma/` subdirs) over the FUSE mount, with a thread-safe per-device registry
and revocation machinery as its spine. The endpoint **files** (`info` /
`bar<M>` / `qpair<Q>` / `hotplug`), the SIM model bridge, the `vpp_emu` bridge,
and streaming are **deferred to later tasks**.

## Layout

```
slash-emu/
├── CMakeLists.txt            # top-level: finds fuse3 + libsystemd, enable_testing()
├── README.md                 # this file
├── src/
│   ├── CMakeLists.txt        # slash_emu_core static lib + slash-emud executable
│   ├── utils.h               # PROPAGATE_ERROR family, _cleanup_, LOG (ported from vrtd)
│   ├── array.h               # type-safe dynamic arrays (ported from vrtd)
│   ├── config.h / config.c   # config parser + persistent accelerator model
│   ├── node.h / node.c       # spine: node tree, per-device registry, refcounted
│   │                         #   resources, revocation machinery (thread-safe)
│   ├── fs.h / fs.c           # libfuse3 LOW-LEVEL session: ops adapt to the node tree
│   └── main.c                # CLI args, sd-event loop, signals, mount/teardown
├── systemd/
│   └── slash-emu.service     # privileged unit (mounts /run/slash_emu)
├── sysusers/
│   └── slash-emu.conf        # client-access group (daemon itself runs as root)
└── tests/
    ├── CMakeLists.txt        # GTest via FetchContent, gtest_discover_tests
    ├── smoke_test.cpp        # mount/browse/unmount end-to-end smoke test
    ├── daemon_lifecycle_test.cpp  # adversarial CLI / mount-failure / no-leak suite
    ├── config_test.cpp      # config parser + accelerator model unit tests
    ├── node_test.cpp        # spine: node tree, registry, refcount, revocation
    └── fs_tree_test.cpp     # end-to-end: configured device tree over the mount
```

## Build and test

All building and testing goes through **CMake + CTest**. The build directory is
`slash-emu/build/`. Scratch/temp output lives in the repo's `.tmp/` dir.

```sh
cmake -S slash-emu -B slash-emu/build -DSLASH_EMU_BUILD_TESTS=ON
cmake --build slash-emu/build
ctest --test-dir slash-emu/build --output-on-failure
```

The daemon binary is built at `slash-emu/build/src/slash-emud`. The smoke test
(`SlashEmuSmoke.MountBrowseAndUnmount`) launches it as a child process, mounts
an empty filesystem under a temp dir in `.tmp/`, verifies the root is browsable
(`stat` + `readdir`, and that a missing name returns `ENOENT`), then sends
`SIGTERM` and asserts a clean exit and a cleanly-unmounted mountpoint.

> FUSE mounting requires kernel FUSE support and permission to mount. In
> environments where unprivileged FUSE mounts are disallowed, run the test (or
> the daemon) with sufficient privileges.

## Running the daemon manually

```sh
slash-emud --config /etc/slash-emu/slash-emu.conf --mount /run/slash_emu
```

- `--config PATH` — configuration file (currently parsed as a stub).
- `--mount PATH` — mountpoint for the emulated tree (default `/run/slash_emu`).

## Design notes

- **Low-level FUSE API.** We use libfuse3's *low-level* interface
  (`fuse_session_new` + `fuse_lowlevel_ops`), not the high-level path-based one,
  because hotplug support in a later task needs
  `fuse_lowlevel_notify_delete` / `fuse_lowlevel_notify_inval_entry`, which are
  only available there. The session's channel fd is integrated into the
  sd-event loop as an I/O source (rather than `fuse_session_loop`) so FUSE and
  daemon events share one loop.
- **Node tree (the spine).** The filesystem is a generic tree of inodes
  (`struct emu_node` in `node.h`), owned by an `emu_node_tree`. The FUSE ops in
  `fs.c` are thin adapters over the node model (`emu_node_lookup_child` /
  `_forget` / `_stat` / `_readdir`); no op hard-codes an inode. At startup the
  daemon materializes one `<BDF>/` dir per configured accelerator with `bars/`
  and `qdma/` subdirs. Endpoints (info/bars/qdma) attach their files via the
  node API without touching the session plumbing. The tree, per-device registry,
  and resource refcounts are mutex-guarded so the data model is safe for a future
  multi-threaded session. See `node.h` for the full design and locking model.
- **Revocation & nameless qpairs.** Each device has a registry that tracks live
  communication resources — including QDMA qpairs that are unlinked-while-open
  (nameless, so unreachable by walking `qdma/`). A qpair is a refcounted resource
  whose lifetime is decoupled from its inode (registry ref + inode ref, freed
  when both drop), with idempotent teardown on either cooperative inode eviction
  or forced device removal. Forced removal eagerly revokes: new lookups get
  `-ENOENT`, ops on already-open fds get `-ENODEV`, names are invalidated via
  `fuse_lowlevel_notify_delete`, and `close` always succeeds. See `node.h`.
- **Privileged service.** Unlike `vrtd`, the systemd unit is intentionally
  *not* hardened: it runs as root (FUSE needs `CAP_SYS_ADMIN`), and avoids
  mount-namespacing options that would hide the FUSE mount. The emulation model
  runs in-process and unsandboxed in this milestone; hardening is revisited once
  the model moves out-of-process.
- **No watchdog yet (known later item).** `vrtd` enables the systemd watchdog;
  slash-emud deliberately does not, because this scaffold has no work/streaming
  loop that could hang and therefore nothing for a watchdog to guard. It is to
  be wired (`sd_event_set_watchdog` + `WatchdogSec=` in the unit) when the
  endpoints / work loop land. See the `TODO` near the event loop in `main.c`.
- **Readiness ordering.** `READY=1` is signalled only after the FUSE mount is
  established, so `Type=notify` consumers and VRTD discovery can trust the
  emulated tree is browsable the moment the service reports started.
