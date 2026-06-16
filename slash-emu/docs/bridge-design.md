# Bridge Design: raw SLASH data plane ↔ vpp_emu/vpp_sim ZMQ protocol

Task T2 design spike. Status: **design/research only**. This document
specifies how the system-emulation daemon translates the new **raw,
address-keyed** kernel ABI (`bars/bar<M>`, `qdma/qpair<Q>`) into the
**existing named/address ZMQ-JSON protocol** spoken by `vpp_emu` and
`vpp_sim`, and — most importantly — flags where that translation is
infeasible, under-specified, or forces changes elsewhere.

The headline finding is in §2 and §7: **`vpp_emu` cannot be driven by a
raw register/address data plane without changes to either the model or
the bridge contract.** `vpp_sim` can. This has direct consequences for
the step-1 MVP scope and for the T1 UAPI header.

---

## 0. Sources reviewed

Wire protocol (host side, the bytes the daemon must emulate the peer of):
- `vrt/src/utils/zmq_server.cpp`, `vrt/include/vrt/utils/zmq_server.hpp`

Model side (the peer the daemon replaces / must imitate for the stub):
- `linker/slashkit/resources/templates/sw_emu_tb.cpp` — the real generated `vpp_emu`.
- `linker/slashkit/resources/sim/sim.cpp` — the real `vpp_sim`.
- `vrt/tests/fixtures/stub_vbin/vrt_stub_server.py` — existing test stub.

How VRT maps registers/buffers onto the protocol today:
- `vrt/src/device.cpp` (launch, manifest application, `parseEmuArgIndex`, `applyEmuManifestToKernels`)
- `vrt/include/vrt/kernel.hpp`, `vrt/src/kernel.cpp` (`read`/`write`, `call`/`start`, arg encoding)
- `vrt/include/vrt/buffer.hpp`, `vrt/include/vrt/streaming_buffer.hpp` (sync → populate/fetch/stream)
- `vrt/src/parser/xml_parser.cpp` (system_map.xml schema)
- `vrt/src/vrtbin.cpp` (VBIN layout, which metadata is actually extracted)
- `vrt/tests/fixtures/stub_vbin/emu_manifest.json`, `linker/.../templates/system_map.xml`

New ABI consumer side (what produces the raw ops the daemon receives):
- `vrt/vrtd/libvrtd/include/vrtd/vrtd.h`, `vrt/vrtd/libvrtdpp/src/{bar_file,qdma_qpair}.cpp`
- `abi_rebuild_architecture.md`

---

## 1. The existing model wire protocol (precise)

### 1.1 Transport & framing

- **Library:** ZeroMQ. **Pattern:** `REQ` (client = VRT, `ZmqServer`) ↔
  `REP` (server = model). Strict lock-step: every request gets exactly
  one reply, one in flight at a time. (`zmq_server.cpp:28`,
  `sw_emu_tb.cpp:112`, `sim.cpp:529`.)
- **Endpoint today:** hard-coded `tcp://localhost:5555`; the model binds
  `tcp://*:5555` (`zmq_server.hpp:45`, both models). One endpoint per
  process ⇒ today only one model per host. The daemon must replace this
  with a per-accelerator unix socket (see §3).
- **Framing:** a request is **1 or 2 ZMQ frames**:
  - Frame 0: a JSON object (the command). Encoded two ways in the code —
    `Json::writeString` (compact) in most calls, but `command.toStyledString()`
    (pretty, multi-line) in `sendBuffer`/`sendStream` (`zmq_server.cpp:36,190`).
    A parser must accept **both**; the models use a tolerant `Json::Reader`.
  - Frame 1 (only for `populate`/`stream_in`): the **raw binary payload**,
    sent with `ZMQ_SNDMORE` on frame 0.
- **Replies are a single frame**, and are one of three shapes depending on
  command (no envelope, caller knows which to expect):
  - Literal ASCII `"OK"` (control commands, populate, stream_in).
  - Literal ASCII `"ERR"` from the real models on failure (`sw_emu_tb.cpp`);
    `ZmqServer::sendCommand` treats **anything other than `"OK"` as an error**
    and throws (`zmq_server.cpp:65`).
  - A **JSON value** (scalar number, or array of byte-ints, or `{"error":...}`).
  - **Raw binary** (only `stream_out`, which returns the bytes directly,
    `zmq_server.cpp:224`).

### 1.2 Command catalogue

Every verb, its request schema, payload, and reply. `F0`=frame 0 JSON,
`F1`=frame 1 binary.

| Command | F0 fields | F1 | Reply | Used by |
|---|---|---|---|---|
| `start` (sim global) | `{command:"start"}` | – | `"OK"` | sim init (`device.cpp:297`) |
| `populate` (named) | `{command:"populate", name, size}` | bytes | `"OK"` | EMU buffer H2D (`zmq_server.cpp:30`) |
| `populate` (addr) | `{command:"populate", addr, size}` | bytes | `"OK"` | SIM buffer H2D (`zmq_server.cpp:288`) |
| `fetch` buffer (named) | `{command:"fetch", type:"buffer", name}` | – | JSON array of byte ints | EMU buffer D2H (`zmq_server.cpp:153`) |
| `fetch` buffer (addr) | `{command:"fetch", type:"buffer", addr, size}` | – | JSON array of byte ints | SIM buffer D2H (`zmq_server.cpp:231`) |
| `fetch` scalar (named) | `{command:"fetch", type:"scalar", function, arg [, offset]}` | – | JSON uint, or `{error}` | EMU scalar fetch (`zmq_server.cpp:74`) |
| `fetch` scalar (addr) | `{command:"fetch", type:"scalar", addr}` | – | JSON uint | SIM scalar fetch (`zmq_server.cpp:261`) |
| `read_register` | `{command:"read_register", function, offset}` | – | JSON uint, or `{error,function,offset}` | EMU register read (`zmq_server.cpp:112`) |
| `reg` | `{command:"reg", addr, val}` | – | `"OK"` | SIM register write (`zmq_server.cpp:307`) |
| `stream_in` | `{command:"stream_in", name}` | bytes | `"OK"` | EMU stream H2D (`zmq_server.cpp:185`) |
| `stream_out` | `{command:"stream_out", name, size}` | – | **raw bytes** | EMU stream D2H (`zmq_server.cpp:206`) |
| `call` | `{command:"call", function, args:{argN:{type,value\|name}}}` | – | `"OK"`/`"ERR"` | EMU sync launch (`kernel.hpp:390`) |
| `start` (kernel) | `{command:"start", function, args:{...}}` | – | `"OK"`/`"ERR"` | EMU async launch (`kernel.hpp:454`) |
| `wait` | `{command:"wait", function}` | – | `"OK"`/`"ERR"` | EMU join (`kernel.cpp:425`) |
| `exit` | `{command:"exit"}` | – | `"OK"` | teardown (`device.cpp:355`) |

Arg encoding inside `call`/`start` (`kernel.cpp:292`):
`args["arg<idx>"] = {"type":"buffer","name":"<decimal phys addr>"}` for
buffer args, or `{"type":"scalar","value":<u64>}` for scalars. The `idx`
is the functional-arg index from system_map.xml, **not** a register offset.

**Buffer naming convention (load-bearing):** in EMU, a buffer's "name" is
`std::to_string(getPhysAddr())` — the **decimal** string of the fake phys
addr assigned by `detail::reserveFakePhysAddr` (`buffer.hpp:42,272,360`).
Streams are named `streamingBuffer_<qid>` / `outputStreamingBuffer_<qid>`
(`streaming_buffer.hpp:126`).

### 1.3 The two models are NOT the same protocol

This is the crux of the whole task. The same `ZmqServer` speaks two
**disjoint dialects** depending on platform:

- **`vpp_sim` (`sim.cpp`) is purely address-keyed.** It implements only
  `populate{addr}`, `fetch buffer{addr,size}`, `fetch scalar{addr}`,
  `reg{addr,val}`, plus global `start`/`exit`. It drives real AXI-Lite /
  AXI-MM FSMs by address. There is **no kernel name anywhere**.
- **`vpp_emu` (`sw_emu_tb.cpp`) is named + command-oriented.** It
  implements `populate{name}`, `fetch buffer{name}`, `read_register{function,
  offset}`, `fetch scalar{function,arg}`, `stream_in/out{name}`, and
  `call`/`start{function,args}`. It has **no `reg{addr,val}` handler at
  all**, and **no address-keyed write path**. Register reads come from a
  per-kernel **shadow map** seeded from the manifest; the only way to make
  a kernel execute is a fully-formed `call`/`start` with a validated args
  object.

The new kernel ABI delivers **only** the address-keyed dialect (raw BAR
offset pokes, raw device-address transfers). So:

- Bridging raw ABS → **`vpp_sim`** is a near-1:1 address remap. Feasible.
- Bridging raw ABS → **`vpp_emu`** requires the daemon to *reconstruct
  named, command-level intent from a byte stream of register pokes* —
  which the emu model is explicitly built to reject unless it arrives as a
  complete `call`. This is the hard part and the main risk (§2, §7).

---

## 2. Reconstructing intent from VBIN metadata

The daemon receives, on the data plane, only: `(bar_index, byte_offset,
width, value)` for BAR ops and `(device_addr, bytes)` for QDMA ops. It
must recover enough structure to talk to the model.

### 2.1 BAR offset → (kernel instance, AXI-Lite register offset)

The PF2 **User region is BAR 0** in the new ABI (architecture §"BAR
Access"). On hardware today, VRT computes a kernel's BAR offset as
`absoluteAddr % barLen` where `absoluteAddr = kernel.baseAddr + regOffset`
(`kernel.cpp:53-66`, `resolveBarOffset`). So the inverse map the daemon
needs is:

```
given bar0_offset:
  find kernel K such that K.baseAddr <= (window_base + bar0_offset) < K.baseAddr + K.range
  reg_offset = (window_base + bar0_offset) - K.baseAddr
```

**Metadata source:** `system_map.xml`, which is **present in every VBIN**
and already parsed (`xml_parser.cpp:91`). Each `<Kernel>` gives `<Name>`
(instance), `<BaseAddress>`, `<Range>`, the `<register offset=...>` list,
and `<functional_args>` with per-arg `idx`, `offset`, `range`, `r`/`w`,
`port`. This is exactly the data VRT's `Kernel` already holds. The daemon
can build an interval map `[baseAddr, baseAddr+range) → kernel` at
reconfiguration time and an inner `regOffset → FunctionalArg` map per
kernel.

The unknown is the **window base / BAR0→AXI mapping**: `resolveBarOffset`
assumes the kernel base addresses are absolute within a contiguous AXI
window whose base is `absoluteAddr - absoluteAddr % barLen`. For a 128 MB
BAR0 and base addresses like `0x10000` (template/stub) this reduces to
"bar0_offset == kernel.baseAddr + regOffset" — i.e. the AXI window base is
0. **This assumption must be pinned down and validated against real
VBINs** (GAP G1).

### 2.2 BAR write → model command (the infeasible step for EMU)

For **`vpp_sim`** this is trivial: a 4-byte BAR0 write becomes
`reg{addr: bar0_offset (or absolute AXI addr), val}` and a 4-byte read
becomes `fetch scalar{addr}`. The daemon does **not** need kernel identity
at all for sim — it just forwards addresses. (Width handling: sim only
ever does 32-bit AXI-Lite; see G4.)

For **`vpp_emu`** there is no `reg` path. The daemon would have to:
1. Resolve `bar0_offset → (kernel K, regOffset)`.
2. **Shadow** the write locally (the emu model keeps its own shadow but
   only accepts updates via `call`, not via pokes).
3. Detect the **`ap_start` poke** — `regOffset == 0x00 && (val & 0x1)` —
   and at that moment synthesize a `call`/`start{function:K, args:{...}}`
   by reading back the shadowed register file and mapping each
   `FunctionalArg` (`idx`,`offset`,`type`,`port`) to an `argN` entry:
   - scalar arg → `{type:"scalar", value: <shadow words at arg.offset, range bits>}`
   - buffer arg → `{type:"buffer", name: <decimal of the 64-bit addr at arg.offset>}`
4. For reads of a kernel register, translate `fetch`/`read_register`:
   `read(0x00)` (ap_done poll in `Kernel::wait`, `kernel.cpp:434`) →
   `read_register{function:K, offset:0}`; a functional-arg readback →
   `fetch scalar{function:K, arg:<name>, offset:regOffset}`.

This is the python stub's `reconstruct_64bit` + `run_vadd` trick
(`vrt_stub_server.py:120`) generalised — but generalising it is exactly
what makes the **full** bridge large and fragile (see §7 G2). The emu
model's `call` validation (`sw_emu_tb.cpp:497-564`) rejects wrong
arg-count or wrong kind, so the synthesized command must be **byte-exactly**
what `Kernel::call` would have produced — meaning the daemon must
re-implement `Kernel`'s arg-encoding, `_r` suffix handling, 64-bit
word-split (`argWordCount`/`argWordValue`), and writable-arg filtering.

### 2.3 Device address → buffer name/arg for QDMA

QDMA `qpair<Q>` `pwrite(buf, off)`/`pread` carries a **device address**
(the `pwrite` file offset is the device address, mirroring how
`QdmaIntf::write_from_buffer` lseeks to `base` = the buffer phys addr,
`qdma_intf.cpp:76`). Mapping:

- **`vpp_sim`:** 1:1. `pwrite@addr` → `populate{addr, size}`+payload;
  `pread@addr` → `fetch buffer{addr, size}`. The HBM/DDR/reconfig windows
  in the architecture (`0x40_0000_0000…`, `0x60_0000_0000…`,
  `0x01_0210_0000…`) are passed straight through as `addr`.
- **`vpp_emu`:** buffers are keyed by the **decimal phys-addr string**
  (`buffer.hpp:272`). So `pwrite@addr` → `populate{name: to_string(addr),
  size}`. This actually works **without** any name table, because VRT
  itself uses the address-as-name. The daemon just needs to format the
  device address as a decimal string. **Streams** are the exception: they
  are named `streamingBuffer_<qid>`, and the raw ABI has no stream concept
  at all — streams ride the *same* `qpair<Q>` file (see G3).

**No allocator state is needed for QDMA addressing.** VRT assigns the
fake phys addrs (`reserveFakePhysAddr`) and uses them as both the
device address (sim) and the buffer name (emu). The daemon is downstream
of that: it receives whatever address VRT chose and forwards it. The
daemon does **not** own the address-space map (see §4).

### 2.4 What metadata is actually in the VBIN — confirmed

- `system_map.xml`: **yes**, always (`vrtbin.cpp:259`). Has kernels, base
  addrs, ranges, registers, functional_args (idx/offset/range/r/w/port),
  port→memory connections.
- `emu_manifest.json`: **yes for EMU VBINs** (`vrtbin.cpp:282`). Has
  `kernels[].call_args[]` (arg→kind), `kernels[].registers[]`,
  `fetch.scalar[]` routes (function/arg→register_offset and var/const
  source), `manifest_schema`, autostart/callable/shutdown_policy
  (`sw_emu_tb.cpp:185-296`). The fixture
  (`tests/fixtures/stub_vbin/emu_manifest.json`) is a **reduced** form
  (no `manifest_schema`/`registers`), so the daemon and the stub must
  agree on the reduced schema for CI (G6).
- `vpp_emu` / `vpp_sim` executables: yes, per platform (`vrtbin.cpp:280,289`).
- **Not present:** any explicit BAR0→AXI window base, any QID→stream-name
  table beyond the system_map `<Qdma>` block, any allocator address map
  (that lives only in the running VRT process).

---

## 3. Daemon ↔ model transport (replacing tcp:5555)

- **Socket type stays ZMQ REQ/REP** — reusing the models unmodified means
  reusing their `ZMQ_REP` bind. Switch endpoint from
  `tcp://*:5555` to `ipc:///run/slash_emu/<bdf>/model.sock` (ZMQ `ipc://`
  = AF_UNIX). Per-accelerator path ⇒ multiple models coexist (fixes the
  single-5555 limitation).
- **Who binds:** the **model binds** (it owns the REP socket today). The
  **daemon connects** as REQ. To make this work the model source must take
  the endpoint from an env var / argv instead of the hard-coded literal —
  this is a **one-line change** in both `sw_emu_tb.cpp:113` and
  `sim.cpp:530` (template-time, so it lands in newly linked VBINs). GAP
  G5: existing pre-linked VBINs hard-code 5555; for those the daemon must
  fall back to a private network namespace or a `tcp` loopback port it
  allocates per model. Recommend env var `SLASH_EMU_ENDPOINT`.
- **Endpoint location:** under the daemon's scratch dir, **not** under the
  FUSE mount (architecture: "model never sees the FUSE mount"). E.g.
  `/run/slash_emu/.scratch/<bdf>/model.sock`, dir mode 0700 owned by the
  daemon.
- **Lifecycle:** the daemon spawns the model after unpacking the VBIN
  (reconfiguration), waits for the socket to accept a connection (poll
  with timeout), then services FUSE ops by REQ/REP. On REMOVE of *both*
  function 1 and 2, or on TOGGLE_SBR/HOTPLUG, the daemon sends `exit` and
  reaps the process (architecture §Reconfiguration, §Hotplug). The MVP
  runs the model directly under the privileged daemon (unsandboxed); the
  transient hardened unit is step 5.
- **Concurrency:** REQ/REP is strictly one-in-flight. The FUSE daemon is
  multi-threaded; **all model I/O for one accelerator must be serialized**
  behind a per-model mutex (or a single per-model I/O thread with a
  request queue). This matches the lock-step protocol and the models'
  single worker loop.

---

## 4. Daemon state: in-memory vs forwarded

| State | Owner | Rationale |
|---|---|---|
| BAR0 interval map (kernel base/range), per-kernel reg/arg maps | **daemon, in-mem**, built from system_map.xml at reconfig | needed to resolve offsets → kernel for EMU; cheap |
| emu_manifest (call_args kinds, fetch routes) | **daemon, in-mem** | needed to synthesize/validate `call` args for EMU |
| **Shadow of BAR register bytes** | **daemon, in-mem (EMU only)** | EMU has no `reg` write path; daemon must accumulate arg writes until `ap_start`, then emit `call`. SIM needs no shadow (forwards each `reg`). |
| Device-memory contents (HBM/DDR) | **model** | `populate`/`fetch` go straight through; daemon does not buffer |
| Address-space allocation map | **neither the daemon nor the kernel** | VRT assigns addresses and uses them as names; daemon forwards. (Raw ABI is explicitly address-keyed and "accesses beyond regions are undefined".) |
| QID registry, qpair lifetime | **daemon** | required by architecture (registry survives unlink; prune on startup) — but this is T7/T10 infra, not protocol mapping |
| Reads the model can't answer | depends | SIM: AXI returns whatever the DUT drives. EMU: `read_register`/`fetch scalar` may return `{error:...}` (`sw_emu_tb.cpp:699`); the daemon must map that to a **defined BAR read result** — propose returning the daemon's shadow value, or zero, and never surfacing `-EIO` for an in-range read (G7). |

**Key point:** for the SIM path the daemon is almost stateless (pure
forwarder). For the EMU path the daemon must hold a write-shadow and the
manifest, and must contain the entire `Kernel::call` arg-encoding logic.

---

## 5. GAPS / UNDER-SPECIFICATIONS / RISKS

Ordered by severity.

- **G1 — BAR0→AXI window base is implicit.** `resolveBarOffset`
  (`kernel.cpp:53`) bakes in "window base = addr − addr%barLen". The
  architecture says BAR0 = User region, 128 MB, but never states the AXI
  address of offset 0. If kernel base addresses in real VBINs are not
  simply offsets-from-0 within BAR0, the inverse map breaks. **Action:**
  confirm against a real synthesized VBIN; if non-zero, the window base
  must be recorded in metadata (new system_map field or manifest field →
  feedback to linker, not just T1).

- **G2 — EMU "full data-plane bridge" is the dominant risk.** The emu
  model is command-oriented and rejects anything but a complete, correctly
  typed `call`. Driving it from raw register pokes requires the daemon to
  (a) shadow all writes, (b) detect `ap_start`, (c) re-implement
  `Kernel`'s exact arg encoding (idx→argN, `_r` suffix, 64-bit split,
  writable filtering, buffer-name = decimal addr), and (d) re-implement
  `read_register`/`fetch scalar` routing including the manifest's
  `synthetic ctrl_valid` and `var_u32_hi` semantics
  (`sw_emu_tb.cpp:318-396`). This is **substantially more than a thin
  bridge** — it is a partial re-host of VRT's kernel layer inside the
  daemon. **Recommendation:** for step-1 MVP, make the **SIM path the
  primary supported model** (it bridges cleanly), and treat EMU either as
  (i) deferred, or (ii) supported only via a small **model-side change**
  that adds a `reg{addr,val}`/`fetch scalar{addr}` address dialect to
  `sw_emu_tb.cpp` so EMU looks like SIM to the daemon. Option (ii) is far
  smaller than re-hosting `Kernel` and keeps the bridge uniform. This is
  the single most important decision for T7/T8.

- **G3 — Streams have no home in the raw ABI.** `stream_in`/`stream_out`
  are named per-QID streaming buffers. The raw ABI exposes only
  `qpair<Q>` byte transfers with a device address; there is no stream
  endpoint and no name. Either (a) the daemon maps a *reserved address
  range per QID* to `stream_in/out{name:streamingBuffer_<qid>}`, or (b)
  streaming is declared out of scope for emulation step 1. Needs a
  decision; if (a), it needs a new convention (feedback to T1/architecture).

- **G4 — BAR access widths {1,2,8} vs model assumptions.** The ABI accepts
  widths 1/2/4/8 (architecture §BAR). Both models assume **32-bit**
  AXI-Lite (`sim.cpp` reads/writes `<32>`; emu shadow is `uint32_t`). The
  daemon must define how 8-byte writes (64-bit arg low/high) and sub-word
  writes map onto 32-bit `reg`/shadow slots, and reject or split as
  appropriate. Underspecified by the architecture.

- **G5 — Endpoint is hard-coded in already-built VBINs.** `tcp://*:5555`
  is compiled into the model. New VBINs can read an env var (1-line
  template change), but the daemon cannot assume that. Need a fallback
  (per-model loopback port or netns) and a way to detect which dialect a
  VBIN's model speaks. Feedback to linker (template) + daemon config.

- **G6 — emu_manifest schema drift.** The real emu model requires
  `manifest_schema.version>=1` + `required_sections` + per-kernel
  `registers` (`sw_emu_tb.cpp:185-287`), but the test fixture manifest
  omits all of these. The daemon's reconstruction logic and the CI **stub
  model** must agree on a single schema. Pin the schema the daemon relies
  on; extend the fixture or document the reduced CI schema.

- **G7 — Defined behavior for unanswerable reads.** EMU returns
  `{error:...}` for unknown function/register/unresolved scalar. The raw
  BAR `pread` ABI must return *bytes*, not a protocol error. Define: a BAR
  read of an in-range, model-unanswerable offset returns the daemon
  shadow (or 0), never `-ENODEV`/`-EIO` (those are reserved for
  revocation/transport failure). Not specified anywhere yet.

- **G8 — No T1 UAPI fields are strictly required by the bridge**, *if*
  G2 is resolved via the SIM-first / model-side-`reg` route. The bridge
  works off VBIN metadata, not UAPI. The one thing worth adding to the
  UAPI/`info` struct for the daemon's benefit: **expose the BAR0 AXI
  window base** (or guarantee it is 0) so G1 is closeable without
  per-VBIN heuristics. Recommend the daemon, not the kernel, own this —
  so likely a *linker/system_map* change rather than a UAPI change.

- **G9 — `wait`/async semantics across the file ABI.** `Kernel::wait`
  polls `read(0x00)&0x2` on hardware/sim but sends an explicit
  `wait{function}` in EMU (`kernel.cpp:425`). Over the raw ABI the client
  will *poll the CTRL register via `pread`*. For SIM that's just repeated
  `fetch scalar{ctrl_addr}` and works. For EMU the daemon must translate
  the ap_done poll into the model's shadow (which `setKernelCtrlCompleted`
  updates after the async call) — i.e. the daemon must have issued the
  `start` and then answer polls from `read_register{offset:0}`. Another
  reason EMU needs the daemon to track per-kernel control state.

---

## 6. Implementation recommendation (T7/T8/T10)

Ordered, with the CI stub strategy.

1. **T10 first — model lifecycle + transport.** Implement VBIN unpack →
   spawn model with `SLASH_EMU_ENDPOINT=ipc://.../model.sock` → connect
   REQ → readiness handshake → `exit`/reap. Per-model serialized I/O
   thread. Land the **1-line template change** in `sw_emu_tb.cpp` and
   `sim.cpp` to honor the env var (coordinate with linker owners).

2. **T8 — QDMA bridge (do before BAR; it's the clean one).** Map
   `qpair<Q>` `pwrite/pread@addr` to `populate{addr}` / `fetch
   buffer{addr,size}` for SIM and `populate{name:to_string(addr)}` /
   `fetch buffer{name}` for EMU. No allocator state. Carve out reconfig
   region writes (the whole-VBIN transfer) as a special target handled by
   T10, per architecture (`pwrite` to the reconfig window = "send VBIN").

3. **T7 — BAR bridge.**
   - **SIM dialect:** pure forwarder. `pwrite@off` → `reg{addr}`,
     `pread@off` → `fetch scalar{addr}`. Build the system_map interval map
     only for validation/logging.
   - **EMU dialect:** only after G2 is decided. If model-side `reg`
     dialect is added (recommended), EMU collapses into the SIM path. If
     not, implement the shadow+`call`-synthesis path and accept the larger
     surface; reuse VRT's `FunctionalArg` encoding rules verbatim.

4. **CI stub model.** A real synthesized `vpp_emu`/`vpp_sim` cannot run in
   CI. Ship a **stub model** modeled on `vrt_stub_server.py` but:
   - binds `SLASH_EMU_ENDPOINT` (unix socket), not tcp:5555;
   - implements **both dialects** the daemon emits, so the same stub
     validates SIM-style (`reg`/`fetch scalar{addr}`/`populate{addr}`) and
     EMU-style (`call`/`read_register`/`populate{name}`) bridging;
   - keeps an addr→bytes map and a per-kernel register shadow, and runs a
     trivial kernel (the `vadd` reconstruct trick) so end-to-end data flow
     is checkable;
   - is the **single source of truth** the daemon is tested against, and
     is exercised by the shared ABI conformance kselftest (architecture
     §Testing) against the FUSE mount.

   This stub is also what lets the ABI conformance suite run identically
   against daemon and (later) kernel module.

---

## 7. Executive summary

- The existing model protocol is **ZMQ REQ/REP, JSON frame 0 (+ optional
  binary frame 1), `"OK"`/`"ERR"`/JSON/raw replies**, fully catalogued in
  §1.2.
- `vpp_sim` is **address-keyed** and bridges to the raw ABI almost 1:1
  (BAR↔`reg`/`fetch scalar`, QDMA↔`populate`/`fetch buffer`). The daemon
  is nearly stateless for SIM.
- `vpp_emu` is **named/command-oriented** and has **no raw register write
  path**; bridging it from raw pokes means the daemon must shadow writes,
  detect `ap_start`, and re-synthesize a fully-validated `call` using VRT's
  exact arg-encoding — effectively re-hosting VRT's kernel layer. **This
  is the principal risk and likely makes the full EMU bridge much larger
  than the architecture implies.**
- All metadata needed (system_map.xml interval map, emu_manifest routes)
  **is present in the VBIN**; no allocator state is needed because VRT
  uses device addresses as buffer names.
- **Recommendation:** make SIM the first-class step-1 path; close the EMU
  gap with a small **model-side `reg`/address dialect** in `sw_emu_tb.cpp`
  rather than re-hosting `Kernel` in the daemon. Re-examine G1 (BAR window
  base), G3 (streams), G4 (widths), G5 (endpoint) before committing T7.
- **T1 feedback:** the bridge needs **no new UAPI fields** if G2 is taken
  via the SIM-first route. The one useful guarantee is that **BAR0 offset
  0 maps to AXI address 0** (or the window base is published in
  system_map) — that is a linker/system_map concern more than a UAPI one.
