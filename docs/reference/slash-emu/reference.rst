..
   comment:: SPDX-License-Identifier: MIT
   comment:: Copyright (C) 2025 Advanced Micro Devices, Inc

#########################
slash-emu Reference
#########################

This page documents the on-disk surface of the ``slash-emud`` daemon: the FUSE
mount layout, the binary ABI exposed by each endpoint, the configuration file
format, the command-line arguments, and the systemd integration. It describes
the **step-1 MVP** as committed; behaviour that is intentionally out of scope
for step 1 is called out in :ref:`slash-emu-step1-scope`.

The endpoint structure and the binary ABI are defined by
:doc:`/reference/kernel-abi/index` and the shared UAPI header
``slash/uapi/slash_abi.h``; the daemon reproduces them verbatim. Constants below
(``SLASH_*``) are those from that header.

Mount Layout
============

The daemon serves one FUSE filesystem rooted at its mountpoint (``/run/slash_emu``
by default). The tree mirrors ``/dev/slash``:

.. code-block:: text

   /run/slash_emu/
   ├── hotplug                       # global ioctl file (rescan/remove/SBR/hotplug)
   └── <BDF>/                        # one directory per configured accelerator
       ├── info                      # read-only struct slash_info
       ├── bars/
       │   ├── bar0                  # PF2 BAR 0 — user region   (128 MiB)
       │   ├── bar2                  # PF2 BAR 2 — service layer  (128 MiB)
       │   └── bar4                  # PF2 BAR 4 — clock wizard   (512 KiB)
       └── qdma/                     # QPAIR_ADD ioctl target
           └── qpair<Q>              # created on demand; MM transfer window

- ``<BDF>`` is the board-level PCI BDF *without* a function suffix, normalized to
  canonical lower-case ``DDDD:BB:DD`` form (e.g. ``0000:61:00``), matching
  ``/dev/slash/<BDF>`` on hardware.
- ``bars/`` always exposes exactly ``bar0``, ``bar2``, and ``bar4``. BARs 1, 3,
  and 5 do not exist on PF2 and are absent.
- ``qdma/`` is empty until a ``QPAIR_ADD`` ioctl creates a ``qpair<Q>``; ``Q`` is
  the daemon-assigned queue-pair ID.
- ``hotplug`` is a single file at the mount root, shared by all accelerators.

Endpoint ABI
============

``info``
--------

A read-only file whose contents are exactly one ``struct slash_info``. The
caller performs a single ``read(2)``; the file size equals
``sizeof(struct slash_info)``.

.. code-block:: c

    struct slash_info {
        __u32 size;                   /* [out] ABI version: sizeof(struct) as populated */
        __u32 acc_type;               /* [out] type bitfield */
        char  bdf[SLASH_PCI_BDF_LEN]; /* [out] board BDF, NUL-terminated, e.g. "0000:61:00" */
    };

- ``size`` is set to the size of the struct the daemon populated, for
  forward-compatible versioning (new fields are only ever appended).
- ``acc_type`` always has ``SLASH_ACC_TYPE_SYSTEM_EMULATED`` (``0x1``) set, since
  every accelerator the daemon serves is system-emulated.
- ``bdf`` is the directory's board-level BDF.

**Permissions:** ``0444`` (read-only).

``bars/bar<M>``
---------------

Each BAR file is a fixed-size register window. The file size equals the BAR
size:

.. list-table::
   :header-rows: 1
   :widths: 12 18 18 22

   * - File
     - BAR index
     - Region
     - Size
   * - ``bar0``
     - ``SLASH_BAR_USER_IDX`` (0)
     - User region
     - ``SLASH_BAR_USER_SIZE`` — 128 MiB
   * - ``bar2``
     - ``SLASH_BAR_SL_IDX`` (2)
     - Service layer
     - ``SLASH_BAR_SL_SIZE`` — 128 MiB
   * - ``bar4``
     - ``SLASH_BAR_CLK_IDX`` (4)
     - Clock wizard
     - ``SLASH_BAR_CLK_SIZE`` — 512 KiB

Access is by ``pread``/``pwrite`` only, as fixed-width register transfers:

- Each transfer width must be 1, 2, 4, or 8 bytes.
- The offset must be naturally aligned to the width.
- The whole transfer must fit within the BAR; a transfer that straddles the end
  is rejected (it is not clamped).

A read of a register never written returns zero. Violations of the rules above
return ``-EINVAL``. BAR files are opened with FUSE direct I/O so the kernel
forwards each transfer verbatim rather than synthesizing page-cached accesses
(see :ref:`slash-emu-no-mmap`).

**Permissions:** ``0600`` (owner read/write).

``qdma/`` — ``QPAIR_ADD`` ioctl
-------------------------------

The ``qdma/`` directory file descriptor accepts a single ioctl that allocates a
queue pair, starts it, and returns its ID. The caller may then open
``qdma/qpair<qid>``.

.. code-block:: c

    #define SLASH_ABI_QDMA_IOCTL_QPAIR_ADD \
        _IOWR('x', 0x40, struct slash_abi_qdma_qpair_add)

    struct slash_abi_qdma_qpair_add {
        __u32 size;        /* [in]  sizeof(struct) for ABI versioning */
        __u32 mode;        /* [in]  0 = MM (AXI Memory Mapped), 1 = ST (streaming) */
        __u32 dir_mask;    /* [in]  bit 0 = H2C, bit 1 = C2H, bit 2 = CMPT */
        __u32 h2c_ring_sz; /* [in]  H2C ring size: CSR table index 0–15 */
        __u32 c2h_ring_sz; /* [in]  C2H ring size: CSR table index 0–15 */
        __u32 cmpt_ring_sz;/* [in]  completion ring size: CSR table index 0–15 */
        __u32 qid;         /* [out] daemon-assigned queue-pair ID */
    };

**Direction:** ``_IOWR`` — the caller fills the input fields; the daemon writes
back ``qid``.

**Return values:**

- ``0`` — success; ``qid`` is set and ``qdma/qpair<qid>`` exists.
- ``-EINVAL`` — malformed request (bad ``mode``, ``dir_mask`` with unknown bits
  or no direction, ring index > 15, or undersized struct).
- ``-EOPNOTSUPP`` — streaming mode (``mode == 1``) or the ``CMPT`` direction bit
  (both deferred in step 1; see :ref:`slash-emu-step1-scope`).
- ``-ENOSPC`` — queue-pair ID space exhausted (not reachable in practice).
- ``-ENOMEM`` — allocation failure.

``qdma/qpair<Q>``
-----------------

A qpair file is a memory-mapped transfer window into device memory. The file
*offset is the device address*: a ``pread``/``pwrite`` at offset ``A`` reads or
writes device memory at address ``A``. There is no meaningful file size (it is an
address window, not a byte file); the file is opened with direct I/O.

The transfer's whole range must lie within a single valid device-memory window:

.. list-table::
   :header-rows: 1
   :widths: 18 28 18

   * - Window
     - Address range
     - Geometry
   * - HBM
     - ``[SLASH_HBM_BASE, SLASH_HBM_END)`` = ``0x40_0000_0000``–``0x48_0000_0000``
     - 64 banks × 512 MiB = 32 GiB
   * - DDR
     - ``[SLASH_DDR_BASE, SLASH_DDR_END)`` = ``0x600_0000_0000``–``0x608_0000_0000``
     - 4 banks × 8 GiB = 32 GiB

A transfer that falls outside both windows, or straddles a window boundary,
returns ``-ERANGE``. A read of never-written memory returns zero.

**Reconfiguration.** A single ``write(2)`` whose whole range lies within the
reconfiguration region — ``[SLASH_RECONFIG_BASE, SLASH_RECONFIG_END)`` =
``0x1_0210_0000``–``0x1_4210_0000`` — is treated as a bitstream (VBIN) delivery,
not an ordinary memory transfer. The entire payload must be written in one
``write(2)`` so the daemon can identify the bounds. In the step-1 MVP no
reconfiguration handler is attached, so a write into this region returns
``-ERANGE`` (the VBIN has nowhere to go); the path is the dormant ``vpp_emu`` /
``vpp_sim`` seam.

qpair files are user-unlinkable: a client may ``unlink`` a ``qpair<Q>`` while
holding it open (the SLASH delete-on-last-close pattern). ``info``, ``bar<M>``,
and ``hotplug`` are not unlinkable.

**Permissions:** ``0600`` (owner read/write).

``hotplug`` — device lifecycle ioctls
-------------------------------------

The single global ``hotplug`` file at the mount root accepts four ioctls. Three
take a device-request struct; ``RESCAN`` takes none.

.. code-block:: c

    #define SLASH_ABI_HOTPLUG_IOCTL_RESCAN     _IO ('w', 0x30)
    #define SLASH_ABI_HOTPLUG_IOCTL_REMOVE     _IOW('w', 0x31, struct slash_abi_hotplug_device_request)
    #define SLASH_ABI_HOTPLUG_IOCTL_TOGGLE_SBR _IOW('w', 0x32, struct slash_abi_hotplug_device_request)
    #define SLASH_ABI_HOTPLUG_IOCTL_HOTPLUG    _IOW('w', 0x33, struct slash_abi_hotplug_device_request)

    struct slash_abi_hotplug_device_request {
        __u32 size;                       /* [in] sizeof(struct) for ABI versioning */
        char  bdf[SLASH_HOTPLUG_BDF_LEN]; /* [in] BDF *with* function, e.g. "0000:61:00.1" */
    };

Note the request BDF includes the function suffix (``.F``), unlike the per-device
folder name. The function selects which endpoint subtree a ``REMOVE`` targets:
function ``1`` maps to ``qdma/``, function ``2`` maps to ``bars/``.

.. list-table::
   :header-rows: 1
   :widths: 18 40

   * - ioctl
     - System-emulation behaviour
   * - ``RESCAN``
     - Two additive passes. First, reload the configuration and add every
       configured accelerator whose BDF is not already live (live BDFs are
       skipped; no reconciliation of a still-running device). Second, rediscover
       any individually-removed function (fn 1 / ``qdma/`` or fn 2 / ``bars/``)
       of a still-live device, rebuilding that function's subtree on the existing
       in-memory device — re-wiring it to the live model if one is running, or
       leaving it model-less awaiting a VBIN if both functions had been removed.
   * - ``REMOVE``
     - Revoke just the one referenced function's subtree (``qdma/`` for fn 1,
       ``bars/`` for fn 2). The other function stays live.
   * - ``TOGGLE_SBR``
     - Fully remove the referenced accelerator (both functions), reload the
       configuration, re-initialize every available accelerator, and emulate the
       ~1 s PCIe-link-retraining sleep. Only the function suffix is ignored
       (parsed for validation); the full board-level BDF — domain, bus, and
       device — locates the accelerator to remove.
   * - ``HOTPLUG``
     - As ``TOGGLE_SBR`` but without the SBR sleep — an atomic
       remove-and-rescan. As with ``TOGGLE_SBR``, only the function suffix is
       ignored; the full board-level BDF (domain, bus, and device) locates the
       accelerator to remove.

**Return values:** ``0`` on success; ``-EINVAL`` for a malformed request or BDF;
``-EOPNOTSUPP`` for a function other than 1 or 2 on ``REMOVE``; ``-EIO`` if a
configuration reload fails; ``-ENOTTY`` for an unrecognized ioctl. Per the
revocation contract, removing an unknown or already-gone endpoint is *not* an
error — the postcondition "the endpoint is gone" already holds.

**Permissions:** ``0600`` (owner read/write).

.. _slash-emu-revocation:

Revocation Semantics
====================

Removal (via the ``hotplug`` ioctls) eagerly revokes the affected endpoints.
The contract matches the hardware backend:

- New ``open()`` / lookup of a removed endpoint returns ``-ENOENT``.
- Any operation on an already-open file descriptor of a removed endpoint returns
  ``-ENODEV``.
- ``close()`` always succeeds; release is idempotent.

Names are invalidated in the kernel's dentry cache as part of removal, so a
removed endpoint disappears from ``readdir`` promptly.

Configuration File
==================

The daemon reads an INI-style configuration file (the same parser layer as
``vrtd``). Its configurable surface is intentionally small: per the architecture,
only the accelerator BDF and reserved network settings are configurable;
everything else is hard-wired.

One section per accelerator, keyed by its board-level BDF:

.. code-block:: ini

   ; A single emulated accelerator at domain 0000, bus 61, device 00.
   [accelerator:0000:61:00]
   net-ip = 10.0.0.1

   ; A second accelerator with the full reserved network configuration.
   [accelerator:0000:62:00]
   net-mac  = 02:00:00:00:00:02
   net-ip   = 10.0.0.2
   net-port = 4791

**Section name.** The ``<BDF>`` after ``accelerator:`` is the per-device folder
name. It is a board-level address — domain, bus, device — *without* a function
suffix. Both the canonical ``DDDD:BB:DD`` form and the short ``BB:DD`` form
(domain defaults to ``0000``) are accepted and normalized to lower-case canonical
form. A ``.F`` function suffix is rejected. Duplicate BDFs (after normalization)
and malformed BDFs are configuration errors and cause the daemon to refuse to
start.

.. warning::

   Each ``[accelerator:<bdf>]`` section **must carry at least one key**.
   slash-emu uses the system libinih, whose parser only reports a section once it
   has a key — a *keyless* section is silently ignored, and the accelerator is
   never created. Give every accelerator at least one reserved ``net-*`` key so
   it is never dropped.

**Reserved keys.** The schema reserves space for future per-accelerator network
configuration. These keys are parsed for forward compatibility but are **not
acted upon yet**:

.. list-table::
   :header-rows: 1
   :widths: 18 40

   * - Key
     - Meaning (reserved; currently inert)
   * - ``net-mac``
     - MAC address string.
   * - ``net-ip``
     - IP address string.
   * - ``net-port``
     - Port string.

Any key other than the reserved network keys, and any section other than an
``[accelerator:<bdf>]`` section, is a configuration error.

Command-Line Arguments
======================

.. code-block:: text

   slash-emud [--config PATH] [--mount PATH] [--help]

.. option:: -c, --config PATH

   Path to the configuration file. If omitted, the daemon starts with no
   accelerators (an empty tree, still mountable).

.. option:: -m, --mount PATH

   Directory to mount the emulated device tree on. Defaults to
   ``/run/slash_emu``.

.. option:: -h, --help

   Show usage and exit.

systemd Integration
===================

The daemon ships a systemd service unit and a sysusers fragment.

**Service unit** (``slash-emu.service``):

- ``Type=notify`` — the daemon signals ``READY=1`` only *after* the FUSE mount is
  established, so a consumer that observes readiness can trust the tree is
  browsable.
- ``RuntimeDirectory=slash_emu`` creates ``/run/slash_emu`` (mode ``0755``) on
  start and removes it on stop. ``ExecStart`` mounts the tree there.
- ``Restart=on-failure`` with ``RestartSec=2s``; ``TimeoutStopSec=30s`` gives the
  daemon time to flush in-flight FUSE requests and unmount.

.. note::

   The unit is deliberately **not hardened** like ``vrtd``. FUSE mounting needs
   ``CAP_SYS_ADMIN``, so the daemon runs as ``root``; and the mount must
   propagate system-wide, so ``PrivateMounts`` / ``PrivateTmp`` /
   ``ProtectSystem`` are *not* set (they would hide or block the mount).
   ``NoNewPrivileges``, ``MemoryDenyWriteExecute``, and syscall filters are
   omitted because the model is unsandboxed in this milestone. Hardening is
   revisited once the model moves out-of-process (see
   :ref:`slash-emu-step1-scope`).

**sysusers fragment** (``slash-emu.conf``):

The daemon itself runs as ``root``, so no dedicated service user is created. The
fragment creates a ``slash-emu`` group so clients can be granted access to the
emulated device tree without being root:

.. code-block:: text

   g   slash-emu   -

.. _slash-emu-step1-scope:

Step-1 Scope
============

The committed daemon is the **step-1 MVP**. The following are intentionally out
of scope and are documented here so the boundaries are not over-read from the
endpoint descriptions above:

- **SIM-only data plane.** Reads and writes are backed by a built-in in-memory
  model. The ``vpp_emu`` / ``vpp_sim`` compute bridge — and reconfiguration via
  a VBIN write into the reconfiguration region — is a wired-but-dormant seam:
  with no handler attached, a reconfiguration write returns ``-ERANGE``.
- **Streaming deferred.** Only memory-mapped (MM) QDMA transfers are supported.
  ``QPAIR_ADD`` with streaming mode (``mode == 1``) or the ``CMPT`` direction
  returns ``-EOPNOTSUPP``.
- **Unsandboxed model.** The emulation model runs in-process, directly under the
  privileged daemon. Moving it out-of-process into a hardened, transient systemd
  unit is a later step.

.. _slash-emu-no-mmap:

**No ``mmap`` contract.** The ABI is read/write/ioctl only; there is no
``mmap`` support, by design (it keeps kernel-side checks and emulation simple).
Register and transfer files are opened with FUSE direct I/O, which delivers the
guarantee in two parts:

- ``MAP_SHARED`` is rejected at ``mmap()`` time by the kernel with ``-ENODEV``
  (it cannot provide shared-mapping coherency for a direct-I/O file). There is
  no coherent register window to keep in sync or zap on revocation.
- ``MAP_PRIVATE`` cannot be vetoed at ``mmap()`` time from a low-level FUSE
  daemon, but the daemon guarantees it never *hangs*: the daemon answers every
  read, so the first page fault resolves to a prompt ``SIGBUS`` rather than
  blocking. There is no usable mapping either way.
