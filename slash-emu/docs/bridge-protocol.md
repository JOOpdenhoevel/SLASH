# T10 SIM bridge: wire protocol, reconfiguration, lifecycle (implementation)

This is the **implementation** companion to `bridge-design.md` (the T2 design
spike).  `bridge-design.md` decided *what* to build; this document records *what
was built* in T10 and is the contract the CI stub model, the daemon-side model
client, and the integration tests all agree on.

Scope (step-1 MVP, per `abi_rebuild_architecture.md`): **SIM-only** (no
`vpp_emu`), **MM-only** (no streaming), model spawned **unsandboxed** directly
under the privileged daemon (the hardened systemd transient unit is step 5).

---

## 1. Transport

* **Library:** ZeroMQ 4.3.5.  The daemon is C++20 and uses the header-only
  **cppzmq** API (`zmq.hpp`) on the client side; JSON is built/parsed with
  **JsonCpp**.  The model side may be C or C++; the CI stub (`stub_model.cpp`)
  is C++ using cppzmq + JsonCpp, and the wire bytes are identical either way.
* **Pattern:** `ZMQ_REQ` (daemon, client) ↔ `ZMQ_REP` (model, server).  Strict
  lock-step: one request, one reply, one in flight.  This matches the real
  `vpp_sim` (`sim.cpp` binds `ZMQ_REP`) and VRT's `ZmqServer` (`ZMQ_REQ`).
* **Endpoint:** a per-device `ipc://` unix socket under the daemon's runtime
  scratch dir, **never** under the FUSE mount (architecture: "the model never
  sees the FUSE mount").  The path is passed to the model in the environment
  variable **`SLASH_EMU_ENDPOINT`**.  Per `bridge-design.md` §3 / G5:
    * **The model binds; the daemon connects.**  (`vpp_sim` owns the `REP`
      socket today.)
    * Real pre-built `vpp_sim` hard-codes `tcp://*:5555` and ignores the env
      var; making it honour `SLASH_EMU_ENDPOINT` is a one-line linker template
      change tracked as G5 and is **out of T10 scope**.  The **CI stub model
      honours `SLASH_EMU_ENDPOINT`**, which is what T10's tests drive.
* **Who waits for whom:** the daemon spawns the model, then **connects** its
  `REQ` socket.  `zmq_connect` to an `ipc://` endpoint succeeds even before the
  peer has `bind`ed (ZMQ queues the connection), so the daemon does a bounded
  **readiness handshake**: it sends `start` and waits up to a timeout for the
  `"OK"` reply.  A model that never binds/answers therefore fails reconfiguration
  with a timeout instead of wedging the daemon (see §4).
* **Timeouts (no-hang guarantee):** the daemon sets `ZMQ_RCVTIMEO`,
  `ZMQ_SNDTIMEO`, and `ZMQ_LINGER` (0) on its `REQ` socket.  Every typed call has
  a bounded wait; a timeout is surfaced as a transport failure (negative errno),
  which the bar/qdma seams map to `-ENODEV`.  `ZMQ_LINGER=0` means teardown never
  blocks on undelivered frames.  Because `REQ`/`REP` is strict-alternation, a
  receive timeout desynchronises the socket; the client treats any transport
  error as fatal for the session and marks the model dead so subsequent ops fail
  fast with `-ENODEV` rather than re-using a broken socket.
* **Concurrency:** the FUSE session is single-threaded today; all model I/O for a
  device is additionally serialised behind a per-model mutex so a future
  multi-threaded session keeps the strict one-in-flight `REQ`/`REP` discipline.

---

## 2. Command catalogue (byte-exact, the `vpp_sim` address-keyed dialect)

Frame 0 is a JSON object (compact, jsoncpp `Json::writeString` style — the stub
accepts any whitespace; the daemon emits compact).  Frame 1, when present, is a
raw binary payload sent with `ZMQ_SNDMORE` on frame 0.  Replies are a single
frame.

| Command   | Frame 0 fields                                   | Frame 1 | Reply                              |
|-----------|--------------------------------------------------|---------|------------------------------------|
| `start`   | `{"command":"start"}`                            | –       | literal `OK` (2 bytes, no NUL/NL)  |
| `exit`    | `{"command":"exit"}`                              | –       | literal `OK`                       |
| `reg`     | `{"command":"reg","addr":<u64>,"val":<u32>}`     | –       | literal `OK`                       |
| `populate`| `{"command":"populate","addr":<u64>,"size":<u64>}`| bytes  | literal `OK`                       |
| `fetch` scalar | `{"command":"fetch","type":"scalar","addr":<u64>}` | – | bare JSON uint (e.g. `42`)         |
| `fetch` buffer | `{"command":"fetch","type":"buffer","addr":<u64>,"size":<u64>}` | – | JSON array of byte ints (`[0,1,..]`) |

Notes matching the real models exactly:

* `addr` is an unsigned 64-bit integer; `reg`'s `val` is a 32-bit register word
  (`sim.cpp` reads it with `asUInt()`).  The daemon only ever forwards
  width-≤4 BAR writes as `reg` (see §2.1); the low `width` bytes carry the value.
* The `"OK"` reply is the literal two-byte ASCII `OK` — **not** JSON, **no**
  trailing newline.  The client accepts exactly `OK`.
* A scalar-fetch reply is a bare JSON number; a buffer-fetch reply is a JSON
  array whose elements are byte values 0..255.  The daemon parses both leniently
  (it scans the integers out of the frame) so a trailing newline from
  `StreamWriterBuilder` is tolerated.

### 2.1 BAR ↔ `reg` / `fetch scalar` mapping (G4: widths)

The SIM dialect is 32-bit AXI-Lite (`reg.val` is `u32`, scalar fetch returns
`u32`).  The raw ABI accepts widths {1,2,4,8} (`emu_bar_check_access`).  T10
maps **the SIM-addressable forwardable case (width ≤ 4)** to a single
`reg`/`fetch scalar` at the BAR-relative byte address and lets the daemon's
shadow cover the rest:

* **width 1/2/4 write** → `reg{addr, val}` where `addr` is the validated
  BAR-relative byte offset and `val` is the little-endian low-`width` bytes.
* **width 1/2/4 read** → `fetch scalar{addr}`; the low `width` bytes of the
  returned `u32` are delivered.
* **width 8** is **not forwarded** to the SIM model (it has no 64-bit AXI-Lite
  verb).  The seam's backend returns the "fall back to the shadow" code (`rc>0`)
  for an 8-byte read and treats an 8-byte write as shadow-only (returns success
  without forwarding).  The shadow is always kept current, so 64-bit register
  slots remain coherent and an in-range 8-byte read never yields `-EIO` (G7).

This keeps the bridge a thin forwarder for the common 32-bit AXI-Lite case while
honouring the ABI's wider widths through the shadow.  The BAR *index* is not sent
to the SIM model (it is purely address-keyed); T10 forwards only BAR 0 (the User
region) writes/reads to the model and serves BAR 2/4 (service-layer/clock) from
the shadow, because the SIM AXI space corresponds to the User region.

### 2.2 QDMA ↔ `populate` / `fetch buffer` mapping

1:1, no allocator state (`bridge-design.md` §2.3):

* `qpair pwrite@addr` (validated in-range HBM/DDR) → `populate{addr,size}` +
  binary frame 1.
* `qpair pread@addr` → `fetch buffer{addr,size}`; the returned byte array fills
  the read buffer.

---

## 3. Reconfiguration (the whole-VBIN write, reassembled from chunks)

Per the architecture, **a single write of the entire VBIN to the reconfiguration
region triggers reconfiguration.**  The region is `SLASH_RECONFIG_BASE ..
SLASH_RECONFIG_END` and is written **through a device's `qdma/qpair<Q>` file**
(the file offset is the device address, exactly as for an HBM/DDR transfer).

* `emu_qdma_check_range()` still **rejects** the reconfig region with `-ERANGE`
  (unchanged — the committed T8 tests pin this).  The interception is a
  **separate, write-only check in the qpair write hook**, evaluated *before* the
  range check: a `pwrite` whose `[addr,addr+len)` lies wholly within the reconfig
  region is a VBIN delivery, routed to the reconfiguration handler; a `pread` of
  the reconfig region stays `-ERANGE` (you cannot read back a VBIN).

### 3.0 Chunk reassembly (the load-bearing detail)

The user issues the VBIN as **one logical write**, but **the kernel splits any
write larger than its FUSE `max_write` (~1 MiB) into multiple `ops->write`
calls.**  A real synthesized VBIN (`vpp_sim` + sibling `.so`s + `system_map.xml`)
is many MB, so reconfiguration *always* arrives as several in-region chunks.  The
bridge therefore **reassembles** contiguous reconfig-region writes into a
per-device accumulation buffer:

* a write at **`SLASH_RECONFIG_BASE`** (re)starts the buffer — a fresh VBIN; any
  prior partial transfer is discarded;
* a write at **`BASE + accumulated_len`** appends;
* any other (non-contiguous / seeking) reconfig-region write **resets** the
  partial transfer and is rejected with **`-EINVAL`** (it is not how a VBIN is
  delivered; resetting prevents silent corruption).

**Completion is detected from the archive structure, not a byte count** — the
bridge has no out-of-band length.  After each chunk the accumulated bytes are
classified by `emu_vbin_classify` (in `vbin.c`):

* **`INCOMPLETE`** — well-formed so far but the ustar zero-block terminator has
  not been reached (or a chunk boundary fell mid-block): accept the chunk
  (`write` returns its byte count) and await more;
* **`INVALID`** — a header is structurally broken (a full 512-byte block lacking
  the `ustar` magic, a bad octal size field, …): no further bytes can fix it, so
  the write fails **`-EINVAL`** and the buffer resets (a bogus first chunk fails
  fast, not after exhausting the cap);
* **`COMPLETE`** — the terminator was reached: apply the VBIN (steps below), then
  reset the buffer.

The accumulation is **capped at `EMU_BRIDGE_MAX_VBIN` (256 MiB)**; a stream that
exceeds it (an abandoned or hostile never-terminating reconfig) is rejected with
**`-EFBIG`** and reset, so it cannot exhaust daemon memory.  An abandoned partial
transfer holds only the accumulation buffer (≤ cap) and is freed at bridge
teardown; it never wedges the daemon (each chunk returns promptly).

A small single-write VBIN (e.g. the ~21 KB CI stub) still works unchanged: it
arrives in one chunk that classifies `COMPLETE` immediately.

> A FUSE `.init` hook raising `max_write` was evaluated as a chunk-count
> optimisation but **omitted**: with the low-level API + the custom sd-event
> receive-buffer, raising `max_write` past the session buffer makes the kernel
> reject reads (`fuse: reading device: Invalid argument`).  Reassembly is the
> correct and sufficient fix regardless of `max_write`.

### 3.1 Applying a complete VBIN

* **Handler steps** (`bridge_apply_vbin`, invoked once the buffer is `COMPLETE`):
  1. Tear down any model already running (a new VBIN replaces it; idempotent).
  2. Unpack the VBIN to a fresh per-device runtime scratch dir.
  3. Locate the `vpp_sim` executable inside the unpacked tree.
  4. `fork`/`exec` it **unsandboxed**, with `SLASH_EMU_ENDPOINT=ipc://…` in the
     environment and the cwd set to the executable's directory (the real model
     loads sibling `.so`s by relative path — `device.cpp` does the same `cd`).
  5. Connect the `REQ` client and run the readiness handshake (`start`).
  6. Attach the **bar** and **qdma** backends so subsequent register/memory ops
     route to the model.
* Reconfiguring a device that already has a running model first tears the old one
  down (idempotent teardown, §4), then brings up the new one.

### 3.1 The CI "VBIN"

A real synthesized `vpp_sim` cannot run in CI, so T10 defines a **minimal VBIN**:
a POSIX **ustar tar** archive (the same format `vrtbin.cpp` reads, optionally
gzip is *not* required for CI) containing at least a `vpp_sim` executable
(`access(X_OK)`).  The daemon locates the member by the filename `vpp_sim`
anywhere in the tree (matching `vrtbin.cpp findExtractedFile("vpp_sim")`).  The
test packs the **CI stub model** binary as `vpp_sim`.  `system_map.xml` is
optional for the SIM path (the daemon does not need the interval map for SIM —
`bridge-design.md` §2.2/§4) and is not required by the CI VBIN.

---

## 4. Model lifecycle + the T9 model-shutdown seam

* The bridge (`struct emu_bridge`) is **per device**, created lazily at first
  reconfiguration and owned by the device's bridge slot.
* The **T9 model-shutdown seam** (`emu_device_set_model_shutdown`) default
  (which only logged) is replaced by the real teardown, fired by the spine
  exactly once when **both** PCI functions of the device are removed:
  1. send `exit` (best-effort, bounded), 2. reap the child (`SIGTERM` then
  `SIGKILL` fallback with a bounded wait, `waitpid`), 3. close the client and
  the socket, 4. remove the scratch dir, 5. detach the bar/qdma backends.
  Idempotent: a second invocation (e.g. whole-device revoke after both
  per-function REMOVEs) is a no-op.
* The seam runs **with the tree lock held** (spine contract).  Teardown must not
  re-enter the spine's public re-locking API; it only touches bridge-private
  state and the borrowed device pointer.
* On daemon shutdown (`cleanup_fs`) any device whose model was never both-removed
  still has its bridge torn down so no child process / socket / scratch leaks.

---

## 5. Carry-forward resolutions

* **bars write: shadow-vs-forward ordering on backend failure.**  The shadow is
  updated **before** the model forward (`bars.c` `bar_write`).  Rationale: the
  shadow is the daemon's authoritative "defined bytes" fallback (G7); keeping it
  current even when the forward later fails means an in-range read after a
  transient transport error still returns the last-written value rather than
  `-EIO`/stale.  On a forward failure the write returns `-ENODEV` (the transport
  is dead) but the shadow already reflects the attempted value — there is **no
  rollback**, by design: the register state the user wrote is what a subsequent
  read should observe, and the `-ENODEV` tells the user the device is gone.
* **qdma store: store-written-before-forward divergence.**  Identical ordering:
  `store_write` runs **before** `populate` (`qdma.c` `qpair_write`).  Same
  rationale and same no-rollback decision; pinned by the committed adversarial
  test "store must stay current despite populate failure".  Divergence between
  the store and the model after a forward failure is acceptable because the model
  is being torn down (the failure *is* revocation/transport death); the store
  remains the defined-bytes source for any read that races the teardown.
