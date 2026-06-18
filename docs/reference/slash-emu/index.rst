..
   comment:: SPDX-License-Identifier: MIT
   comment:: Copyright (C) 2025 Advanced Micro Devices, Inc

###########################
slash-emu Daemon
###########################

``slash-emud`` is the SLASH :doc:`system-emulation </explanation/system-emulation>`
daemon. It mounts a FUSE filesystem at ``/run/slash_emu`` that reproduces the
``/dev/slash`` :doc:`kernel ABI </reference/kernel-abi/index>`, so the userspace
stack can target a software-emulated accelerator without change.

.. toctree::
   :maxdepth: 1

   reference
