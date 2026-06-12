# New kernel ABI 

Idea: Kernel driver exposes a custom "SLASH" filesystem that is mounted in `/dev/slash`. Each accelerator receives its own directory named by device ID, e.g. `/dev/slash/0000:61:00/`. Each accelerator directory contains sub-folders and files to control the accelerator. All file operations are either `read`, `write` (both potentially with an offset), or `ioctl`. All written and returned data is plain-old-data. In particular, syscalls must not return file descriptors or other references that are only valid in the context of the calling process.

With these restrictions on the operations available for the ABI, it is possible and relatively straight-forward to emulate the kernel ABI with a FUSE filesystem, using a dedicated system emulation daemon. Such an daemon would expose a file system in `/run/slash_emu/` with the same structure as `/dev/slash`, but instead exposing ways to manipulate an emulated accelerator.

Note: This introduces a new concept "system emulation", independent of the existing "FPGA emulation" and "FPGA simulation" concepts. "System emulation" is the emulation of the entire accelerator in the host system, i.e. how it is handled by VRT and VRTD. "FPGA emulation" and "FPGA simulation" describe ways to predict the behavior of a group of kernels on the FPGA. How the actual FPGA's behavior is predicted doesn't matter much for the system emulation that we want to introduce.

## File endpoints

### QDMA

* `/dev/slash/<BDF>/qdma/`
  * Directory
  * GET_INFO IOCTL like before (excluding info expressed by file presence)
  * "QPAIR_ADD" IOCTL
    * Inputs: Mode, directions, H2C/C2H/CMPT ring sizes
    * Output: Path to the newly created `/dev/slash/<BDF>/qdma/qpair<Q>` file
    * Also starts the queue
* `/dev/slash/<BDF>/qdma/qpair<Q>`
  * llseek, read, write
    * Memory transfers, as done previously
  * removal stops the underlying qpair and removes it
  * Effectively, a qpair file acts as a file representing the accelerator memory

### BAR Access - traditional with one file for the entire BAR

* `/dev/slash/<BDF>/bars/`
  * Directory
  * GET_INFO IOCTL like before (excluding info expressed by file presence)
* `/dev/slash/<BDF>/bars/bar<M>`
  * "INFO" IOCTL
    * Only returns "start_address"
    * Usability encoded in file presence
    * Length encoded in file length
  * read and write with offset
    * Reads and writes the BAR
    * Verifies read/write widths
    * Explicitly no Mmap'ing to enable kernel-side checks and easier emulation

### Hotpluging/Resets

* `/dev/slash/hotplug`, with the same behavior as before.

## Authorization of user processes

* Only VRTD can open files, checked using normal UNIX permissions
* VRTD checks permissions of users (according to configuration)
  * Then creates/opens files for them
  * Passes the FD via SCM_RIGHTS
* We thus get permission handling done by the kernel for free, even for system emulation
* Also, all data plane operations (accessing BAR, starting DMA transfers) are going directly from the user to the kernel/system emulation daemon.

## Implementation of the file system

* System emulation daemon exposes a FUSE filesystem exactly matching the ABI described above
  * No passing of FDs by the kernel ABI, no mmap's, no trouble!
* Kernel module:
  * Implementing a new filesystem is something that many drivers do
    * Should be doable
  * Example: drivers/android/binderfs.c
  * Framework: fs/libfs.c
    * Register a new filesystem type, which is then mounted by userspace
    * Implement all operations there
  * Alternatives:
    * kernfs (the engine behind sysfs/cgroupfs)
      * handles dynamic node trees beautifully, but its file ops are attribute-oriented
      * reads go through a bounded kernel buffer and writes through a single kernel buffer. 
        * No zero-copy DMA
    * configfs: Strictly for config attributes, no bulk read/write data
    * Dynamic char devices:
      * Flat /dev, no subtrees
      * Nodes appear asynchronously, an open() right after a creation ioctl may fail

## Reconfiguration

* On hardware: QDMA of the DCP to AVED, just as before
* In system emulation:
  * Especially FPGA simulation needs many other files from the VBIN
  * Therefore: User has to transfer entire VBIN to emulation daemon via "QDMA"
  * System emulation daemon unpacks VBIN somewhere
  * MVP (step 1): runs the model directly under the privileged daemon, unsandboxed
  * Step 5: runs either vpp_emu or vpp_sim in a systemd transient unit (daemon must be privileged, model is not)
    * Identity: `DynamicUser=yes` (per-session throwaway uid), `RemoveIPC=yes`, `PrivateTmp=yes` → cross-tenant isolation for free
    * Scratch (unpacked VBIN) via `RuntimeDirectory=`/`BindPaths=`; rest read-only/hidden (`ProtectSystem=strict`, `ProtectHome=yes`, `ReadWritePaths=<scratch>`)
    * Harden: `NoNewPrivileges`, `PrivateDevices`, `RestrictSUIDSGID`, `ProtectKernel*`, `RestrictNamespaces`, `LockPersonality`
    * DoS bounds: `MemoryMax`, `TasksMax`, `CPUQuota`, `RuntimeMaxSec`
    * Data plane: model talks to daemon over unix socket in scratch (replace tcp://localhost:5555), never sees the FUSE mount
    * emu: self-contained → `PrivateNetwork=yes`, tight `SystemCallFilter=@system-service`
    * sim (xsim): needs license egress + Vivado tree → can't fully sandbox
      * `IPAddressDeny=any` + `IPAddressAllow=localhost <license-server>`, `BindReadOnlyPaths=<vivado>`, looser syscall/Tasks limits
      * Residual: license-egress widens surface; kernel sandbox-escape is the ultimate threat (require current kernel)

## Features of the emulation daemon

* Manages emulated accelerators
  * Each of them persistent, defined in a configuration file
* Exposed via a FUSE filesystem, using the kernel ABI

## Testing

* Principle: unit AND integration tests on everything — every component, every layer
* Unit tests: component-internal logic (GTest for C++, equivalent for C); ship with the component, not later
* Integration tests: each component against its real interface/neighbors
* ABI conformance = one kselftest suite (modeled on `driver/tests/`), the single source of truth for the ABI
  * Same suite runs against BOTH the FUSE daemon AND the kernel module → guarantees they don't drift
* Every implementation step below ships its tests as part of that step

## Implementation steps

1. Build a minimum viable version of the emulation daemon
  * Includes "reconfiguration" (swapping the vpp_emu/vpp_sim model)
  * But unsandboxed: model runs directly under the privileged daemon; sandboxing is deferred to step 5
2. Update/rewrite libslash for the new kernel ABI
  * Using the emulation daemon for testing
3. Update VRTD to use the new libslash
  * Again, using the emulation daemon for testing
4. Update libvrtd and VRT
  * Including reconfiguration, although it doesn't change anything yet
5. Add sandboxing to the system-emulated "reconfiguration" (built unsandboxed in step 1)
  * Wrap vpp_emu/vpp_sim in the hardened systemd transient unit (see "Reconfiguration" above)
  * Can now be tested through the entire stack
6. Implement the kernel module, relying on the now built stack.

## Open TODOs (resolve before/while implementing)

### Blocks step 1 (MVP daemon)

* [ ] 1. Source of an emulated accelerator's shape (BAR count/sizes/start_address, qdma info, memory targets)
  * From dummy VBIN's `system_map.xml`, separate daemon config, or hardcoded? Define config-vs-system_map.xml split
* [X] 2. Define the step-1 test client (libslash not ready, VRT still on ZeroMQ) — throwaway ioctl/`dd` tool? Becomes ABI conformance harness
* [ ] 3. Mount-root discovery: how libslash/VRTD pick `/dev/slash` vs `/run/slash_emu` (env var / config / presence detection)

### Settle the ABI surface

* [ ] 4. One shared ABI header (ioctl numbers + POD structs) as single source of truth for kernel module AND daemon — prevents drift
* [ ] 5. Concrete GET_INFO/INFO struct contents ("excluding file-presence info" is too vague); define fake BAR start_address (stable? consistent with VRT's fake phys addrs)
* [ ] 6. ioctl-over-FUSE rule: all ioctls fixed-size `_IOR/_IOW/_IOWR` POD (no unrestricted retries); QPAIR_ADD path return needs fixed max-length buffer
* [ ] 7. qpair lifecycle: who allocates `Q` (stable?); make user→VRTD→QPAIR_ADD→VRTD-opens-path→SCM_RIGHTS flow explicit; "removal" = unlink / last-close / ioctl?
* [X] 8. BDF directory naming: per-device (aggregate PF1 qdma + PF2 ctl under one dir) vs per-PF-BDF; fix the `0000:61:00` vs `…:00.0` example
* [ ] 9. BAR access-width contract: `pread`/`pwrite` only, no buffering, width == transfer size, reject misaligned/odd widths (also a libslash constraint)

### Defer but reserve now

* [ ] 10. Reconfiguration endpoint undefined in "File endpoints"; reserve dedicated control file (e.g. `<BDF>/reconfig`) instead of overloading qpair write; how does daemon tell VBIN upload from memory transfer?
* [ ] 11. Hotplug semantics under system emulation (reset = restart model process?); global privileged file

### Minor

* [ ] 12. Reword "permissions for free": UNIX perms only gate who may open (VRTD); real policy is VRTD/`vrtd.conf`. Confirm FUSE node ownership + whether `default_permissions` is used (data-plane caller ≠ opener)
