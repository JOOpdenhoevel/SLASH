# New kernel ABI 

Idea: Kernel driver exposes a custom "SLASH" filesystem that is mounted in `/dev/slash`. Each accelerator receives its own directory named by device ID, e.g. `/dev/slash/0000:61:00/`. Each accelerator directory contains sub-folders and files to control the accelerator. All file operations are either `read`, `write` (both potentially with an offset), or `ioctl`. All written and returned data is plain-old-data. In particular, syscalls must not return file descriptors or other references that are only valid in the context of the calling process.

With these restrictions on the operations available for the ABI, it is possible and relatively straight-forward to emulate the kernel ABI with a FUSE filesystem, using a dedicated system emulation daemon. Such an daemon would expose a file system in `/run/slash_emu/` with the same structure as `/dev/slash`, but instead exposing ways to manipulate an emulated accelerator.

Note: This introduces a new concept "system emulation", independent of the existing "FPGA emulation" and "FPGA simulation" concepts. "System emulation" is the emulation of the entire accelerator in the host system, i.e. how it is handled by VRT and VRTD. "FPGA emulation" and "FPGA simulation" describe ways to predict the behavior of a group of kernels on the FPGA. How the actual FPGA's behavior is predicted doesn't matter much for the system emulation that we want to introduce.

## File endpoints

### Information file

* Path `/dev/slash/<BDF>/info`
* Read-only, returns binary information struct
* Encodes all information that can't be expressed as file metadata
* Uses the previous `size`-based versioning:
  * New fields are only appended to the field
  * Old readers only read the fields they understand
* Current state of the information struct:

```C
#define SLASH_PCI_BDF_LEN 32

struct slash_info {
    __u32 size;                   /* [in/out] ABI version */
    __u32 acc_type;               /* [out] Bitflags describing the accelerator type. Currently: 0x1: System-Emulated */
    char  bdf[SLASH_PCI_BDF_LEN]; /* [out] PCI BDF string without function, NUL-terminated, e.g. "0000:61:00" */
    __u16 vendor_id;              /* [out] PCI vendor ID (0x10EE for AMD/Xilinx) */
    __u16 device_id;              /* [out] PCI device ID (0x50B6 for PF2) */
    __u16 subsystem_vendor_id;    /* [out] PCI subsystem vendor ID */
    __u16 subsystem_device_id;    /* [out] PCI subsystem device ID */
    __u32 qdma_qsets_max;         /* [out] Max queue sets (currently always 0) */
    __u32 qdma_msix_qvecs;        /* [out] MSI-X vectors for queues (currently always 0) */
    __u32 qdma_vf_max;            /* [out] Max VFs (currently always 0) */
    __u32 qdma_caps;              /* [out] Capability bitmask (currently always 0) */
};
```

### QDMA

* `/dev/slash/<BDF>/qdma/`
  * Directory
  * "QPAIR_ADD" IOCTL
    * Inputs: Mode, directions, H2C/C2H/CMPT ring sizes
    * Output: QID, allocated by kernel/daemon
      * Used to construct the queue pair path `/dev/slash/<BDF>/qdma/qpair<Q>`
      * Then opened separately
    * Also starts the queue
* `/dev/slash/<BDF>/qdma/qpair<Q>`
  * llseek, read, write
    * Memory transfers, as done previously
  * TODO: Resolve lifetime
* Memory ranges (HBM banks, DDR, reconfiguration target)
  * Part of UAPI header in libslash
  * Both offsets and lengths

### BAR Access - traditional with one file for the entire BAR

* `/dev/slash/<BDF>/bars/`
  * Directory
  * GET_INFO IOCTL like before (excluding info expressed by file presence)
* `/dev/slash/<BDF>/bars/bar<M>`
  * File, only pread and pwrite
    * Reads and writes the BAR
    * Verifies read/write widths
    * Only meant for register access
  * `pread`/`pwrite` only, no buffering, width == transfer size, reject misaligned/odd widths
  * Explicitly no Mmap'ing to enable kernel-side checks and easier emulation
  * Size of the BAR encoded as the size of the file
* Available BARs:
  * BAR 0: User region
    * Size: 128 MB
  * BAR 2: Service Layer
    * Size: 128 MB
  * BAR 4: Clock wizard
    * Size: 512 KB
  * Codified in the UAPI header
* Side note: The start address attribute has been dropped and is not reported anymore

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
  * Ship .mount systemd unit to automatically mount /dev/slash/

## Reconfiguration

* In both cases, the user writes their payload to the "reconfiguration region"
* On hardware: QDMA of the DCP to AVED, just as before
* In system emulation:
  * Especially FPGA simulation needs many other files from the VBIN
  * Therefore: User has to transfer entire VBIN to emulation daemon via "QDMA"
  * System emulation daemon unpacks VBIN somewhere
  * Runs either vpp_emu or vpp_sim in a hardened systemd transient unit (daemon must be privileged, model is not)
    * To note: Emu is self-contained, but sim needs Vivado and potentially network egress to check licenses
    * Data plane: model talks to daemon over unix socket in scratch (replace tcp://localhost:5555), never sees the FUSE mount
  * MVP (step 1): runs the model directly under the privileged daemon, unsandboxed

## Emulation daemon configuration & deployment

* Accelerators are persistent (persist beyond the lifetime of a user process)
* Accelerator configuration covers:
  * Accelerator BDF
  * Space in the schema for network configuration (to be done in the future)
* Everything else currently hard-wired
* Command line arguments:
  * Path to config, Path to mount path
* Ship with systemd units to tie the daemon in

## Testing

* Principle: unit AND integration tests on everything — every component, every layer
* Unit tests: component-internal logic (GTest for C++, equivalent for C); ship with the component, not later
* Integration tests: each component against its real interface/neighbors
* ABI conformance = one kselftest suite (modeled on `driver/tests/`), the single source of truth for the ABI
  * Same suite runs against BOTH the FUSE daemon AND the kernel module → guarantees they don't drift
* Every implementation step below ships its tests as part of that step

## Filesystem discovery by VRTD

* VRTD config contains an ordered list of mount paths where to look for a SLASH endpoint
  * Default configuration shipped with VRTD lists `/dev/slash` and `/run/slash_emu`
  * But, in theory an arbitrary number of endpoints allowed, even excluding the hardware endpoint
* No endpoint listed is an error
* If multiple accelerators with same BDF from different endpoints exist, earlier endpoints take precedence

## Implementation steps

1. Build a minimum viable version of the emulation daemon
  * Includes "reconfiguration" (swapping the vpp_emu/vpp_sim model)
    * But unsandboxed: model runs directly under the privileged daemon; sandboxing is deferred to step 5
  * Kernel ABI description written to UAPI header in libslash
2. Update/rewrite libslash for the new kernel ABI
  * Using the emulation daemon for testing
3. Update VRTD to use the new libslash
  * Again, using the emulation daemon for testing
4. Update libvrtd and VRT
  * Add the "system emulated" field, so that VRT uses libvrtd to talk to system-emulated accelerators
  * But: Keep branching on hardware or system-emulation minimal
5. Add sandboxing to the system-emulated "reconfiguration" (built unsandboxed in step 1)
  * Wrap vpp_emu/vpp_sim in the hardened systemd transient unit (see "Reconfiguration" above)
  * Can now be tested through the entire stack
6. Implement the kernel module, relying on the now built stack.

## Open TODOs (resolve before/while implementing)

### Settle the ABI surface

* [ ] qpair lifecycle: who allocates `Q` (stable?); make user→VRTD→QPAIR_ADD→VRTD-opens-path→SCM_RIGHTS flow explicit; "removal" = unlink / last-close / ioctl?

### Defer but reserve now

* [ ] Reconfiguration endpoint undefined in "File endpoints"; reserve dedicated control file (e.g. `<BDF>/reconfig`) instead of overloading qpair write; how does daemon tell VBIN upload from memory transfer?
* [ ] Hotplug semantics under system emulation (reset = restart model process?); global privileged file

### Minor

* [ ] Reword "permissions for free": UNIX perms only gate who may open (VRTD); real policy is VRTD/`vrtd.conf`. Confirm FUSE node ownership + whether `default_permissions` is used (data-plane caller ≠ opener)
