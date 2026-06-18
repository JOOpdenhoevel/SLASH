..
   comment:: SPDX-License-Identifier: MIT
   comment:: Copyright (C) 2025 Advanced Micro Devices, Inc

##########################
Run the slash-emu Daemon
##########################

This guide walks through building, configuring, and running the SLASH
:doc:`system-emulation </explanation/system-emulation>` daemon (``slash-emud``),
so the userspace stack can talk to a software-emulated accelerator with no FPGA
hardware attached. For the full ABI and configuration surface, see
:doc:`/reference/slash-emu/index`.

.. note::

   FUSE mounting needs kernel FUSE support and permission to mount. Where
   unprivileged FUSE mounts are disallowed, run the daemon (and its tests) with
   sufficient privileges.

Build
=====

The daemon builds with CMake. It needs ``fuse3`` and ``libsystemd``.

.. code-block:: bash

   cmake -S slash-emu -B slash-emu/build -DSLASH_EMU_BUILD_TESTS=ON
   cmake --build slash-emu/build
   ctest --test-dir slash-emu/build --output-on-failure

The daemon binary is produced at ``slash-emu/build/src/slash-emud``.

Write a configuration
=====================

Create an INI file with one section per accelerator. Remember each
``[accelerator:<bdf>]`` section must carry at least one key, or libinih silently
drops it — give every accelerator a reserved ``net-*`` key:

.. code-block:: ini

   [accelerator:0000:61:00]
   net-ip = 10.0.0.1

The BDF is the board-level address without a function suffix. See the
:doc:`/reference/slash-emu/index` configuration section for the full rules.

Run it
======

.. code-block:: bash

   slash-emud --config /etc/slash-emu/slash-emu.conf --mount /run/slash_emu

The emulated tree appears under the mountpoint. Browse it like any filesystem:

.. code-block:: bash

   ls -l /run/slash_emu/0000:61:00
   ls -l /run/slash_emu/0000:61:00/bars
   xxd -l 16 /run/slash_emu/0000:61:00/info

Under systemd, the shipped unit mounts the tree for you:

.. code-block:: bash

   systemctl start slash-emu.service

Because the unit is ``Type=notify`` and signals readiness only after the mount
is up, the tree is browsable the moment ``systemctl`` reports the service
started.

Reconfigure an accelerator
==========================

Reconfiguration (loading a new bitstream / VBIN) is delivered as a single
``write(2)`` whose whole range lies inside the reconfiguration region —
``[SLASH_RECONFIG_BASE, SLASH_RECONFIG_END)`` — issued on a ``qdma/qpair<Q>``
file. The entire payload must go in one ``write`` so the daemon can identify its
bounds.

.. note::

   In the step-1 MVP no reconfiguration handler is attached: a write into the
   reconfiguration region returns ``-ERANGE``. This is the dormant
   ``vpp_emu`` / ``vpp_sim`` model seam, enabled in a later step. See
   :ref:`slash-emu-step1-scope`.

A throwaway instance for poking
===============================

For manual exploration there is a developer convenience script,
``scripts/emud-scratch.sh``, which spins up a throwaway ``slash-emud``, prints
the mount path, optionally runs a command against the live mount, and tears
everything down cleanly on exit. All scratch lives under ``slash-emu/.tmp``.

.. code-block:: bash

   # Mount and wait (Ctrl-C tears down):
   scripts/emud-scratch.sh

   # Mount, run a command against $MNT, then tear down:
   scripts/emud-scratch.sh -- ls -l "$MNT"/0000:61:00/bars
   scripts/emud-scratch.sh -b 0000:62:00 -- xxd -l 16 "$MNT"/0000:62:00/info

This is for exploration, not CI — automated coverage lives in the GTest/CTest
suites.
