..
   comment:: SPDX-License-Identifier: MIT
   comment:: Copyright (C) 2025 Advanced Micro Devices, Inc

#################
System Emulation
#################

*System emulation* is a distinct concept from the :doc:`platform modes
<platform-modes>` (Hardware, Emulation, Simulation). It is easy to conflate the
words "emulation" and "simulation" across the two, so this page draws the line
precisely.

- **FPGA emulation** and **FPGA simulation** are *platform modes*: ways to
  predict the behaviour of a group of kernels that would otherwise run on the
  FPGA fabric. FPGA emulation runs a C-model of the kernels; FPGA simulation
  runs their RTL in a Verilog simulator. Both answer the question *"what would
  these kernels compute?"*

- **System emulation** is the emulation of an *entire accelerator as the host
  system sees it* — i.e. how the device is presented to the user application,
  VRT, and VRTD through the SLASH kernel ABI. It answers a different question:
  *"what does the software stack see when it talks to a SLASH device?"*

The two are orthogonal and compose: a system-emulated accelerator can run FPGA
emulation or FPGA simulation as its underlying compute model. *How* the fabric's
behaviour is predicted does not matter to system emulation, which is concerned
only with reproducing the device-facing ABI.

The SLASH kernel ABI
====================

The premise that makes system emulation tractable is the shape of the
:doc:`kernel ABI </reference/kernel-abi/index>`. The SLASH driver exposes a
custom filesystem mounted at ``/dev/slash``. Every accelerator is a directory
named by its PCI BDF (without the function), and every operation on the
endpoints inside it is a ``read``, a ``write`` (optionally at an offset), or an
``ioctl``. All payloads are plain-old-data: no syscall returns a file descriptor
or any other reference that is only valid in the calling process.

Because the ABI is *just* a directory of files served by reads, writes, and
ioctls over plain data, it can be reproduced faithfully by a userspace FUSE
filesystem. There are no kernel-passed file descriptors and no ``mmap`` to
emulate.

The system-emulation daemon
===========================

``slash-emud`` is the SLASH system-emulation daemon. It serves a FUSE
filesystem at ``/run/slash_emu`` whose layout is identical to the ``/dev/slash``
tree a real driver would expose. The same userspace stack — libslash, libvrtd,
VRT — can therefore target a real board or a software-emulated accelerator
without changing a line: only the mount path differs.

Each accelerator the daemon is configured with becomes a ``<BDF>/`` directory
holding the same endpoints the hardware ABI defines:

- ``info`` — the read-only ``struct slash_info`` binary struct.
- ``bars/bar<M>`` — register windows for PF2 BARs 0, 2, and 4.
- ``qdma/`` — accepts the ``QPAIR_ADD`` ioctl; created qpairs appear as
  ``qdma/qpair<Q>`` files for memory-mapped transfers.

A single global ``hotplug`` file at the mount root accepts the rescan / remove /
SBR / hotplug ioctls, mirroring the hardware hotplug endpoint.

See :doc:`/reference/slash-emu/index` for the full mount layout, the binary ABI
surface, the configuration format, and the command-line and systemd interfaces;
and :doc:`/howto/run-slash-emu` for running an instance.

Where it fits in the roadmap
============================

System emulation is built in stages. The committed daemon is the **step-1
MVP**, which deliberately scopes itself narrowly:

- The data plane is **SIM-only**: a built-in in-memory model backs reads and
  writes. The ``vpp_emu`` / ``vpp_sim`` compute bridge is a wired-but-dormant
  seam, deferred to a later step.
- **Streaming is deferred**: only memory-mapped (MM) QDMA transfers are
  accepted; streaming-mode qpairs are rejected.
- The model runs **in-process and unsandboxed** under the privileged daemon.
  Moving it out-of-process and hardening it (a transient, sandboxed systemd
  unit) is a later step.

These scope boundaries are called out wherever they affect observable behaviour
in :doc:`/reference/slash-emu/index`.
