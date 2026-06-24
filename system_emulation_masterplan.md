# New kernel ABI 

Idea: Kernel driver exposes a custom "SLASH" filesystem that is mounted in `/dev/slash`. Each accelerator receives its own directory named by device ID, e.g. `/dev/slash/0000:61:00/`. Each accelerator directory contains sub-folders and files to control the accelerator. All file operations are either `read`, `write` (both potentially with an offset), or `ioctl`. All written and returned data is plain-old-data. In particular, syscalls must not return file descriptors or other references that are only valid in the context of the calling process.

With these restrictions on the operations available for the ABI, it is possible and relatively straight-forward to emulate the kernel ABI with a FUSE filesystem, using a dedicated system emulation daemon. Such an daemon would expose a file system in `/run/slash_emu/` with the same structure as `/dev/slash`, but instead exposing ways to manipulate a system-emulated accelerator.

## General notes

This introduces a new concept "system emulation", independent of the existing "FPGA emulation" and "FPGA simulation" concepts. "System emulation" is the emulation of the entire accelerator in the host system, i.e. how it is handled by the user application, VRT, and VRTD. Contrarily, "FPGA emulation" and "FPGA simulation" describe ways to predict the behavior of a group of kernels on the FPGA in software. How the actual FPGA's behavior is predicted doesn't matter much for the system emulation that we want to introduce, and system emulation can be combined with FPGA emulation or FPGA simulation.

This refactor does not cover the board management via the primary function 0. This is handled by the ami driver.

What can be implemented thread-safe should be implemented thread-safe.

## Emulation daemon ABI

### Per-device folders

* One folder in `/dev/slash` for each accelerator
* named after the board BDF identifier, i.e. everything from the BDF except for the function
* Example: Accelerator with physical functions `0000:61:00.1` and `0000:61:00.2`
  * Represented with the folder `/dev/slash/0000:61:00`
* Driver uses all physical functions to implement one folder of endpoints

### QDMA / Memory transfers

#### Managed resource hierarchy

* Root QDMA subsystem
  * Queue groups (exported as queue group files)
    * Qpairs (only internally accessible, also only within the kernel module)
    * Host buffers (exported as host buffer files)

#### Resource management pattern

* The QDMA subsystem requires the management of different resources and permissions
  * Queue groups
    * Contain one or more queue pairs as internal components
    * May only be created by a privileged user (i.e. VRTD)
  * Host buffers
    * Backed by a specific queue group
    * Can be created by any process that has received permission to use a queue group
    * Can be used to transfer data between card and host
* These resources are exported as files
  * with operations implemented with read, write, lseek, stat, ioctl, ...
* One privileged process can thus create/open a resource file for an unprivileged process and pass them the FD via SCM_RIGHTS
* Resources are allocated with ADD ioctls
  * Thus require an open FD to the managing file
  * On success, these ioctls return a path to the newly created resource file
  * Created with UID/GID of the caller, mode 600
  * Caller can immediately open the file, then unlinks them
* Reason why the ADD ioctl does not open the file for the caller:
  * Primiarily: A FUSE server can not return a FD that is meaningful to the client
    * No problem for the kernel, but not possible with FUSE
  * Also: Do one thing and do it well
    * For example, ADD ioctls do not need file opening flags
  * Con: Resources may leak if the user does not unlink resource files
    * Accepted cost of the solution
* Resources are referenced both by an inode and a device-wide registry
  * Resources are released when the inode is released (i.e. file is unlinked and closed by all processes)
  * But: Resources are released and the inode is invalidated if the hotplugging subsystem tears down the device
    * Open FDs remain valid, but all operations on them fail.

#### File endpoints

* `/dev/slash/<BDF>/qdma/`
  * Directory
  * "INFO" IOCTL (`#define SLASH_QDMA_IOCTL_INFO _IOWR('v', 0xXX, struct slash_qdma_info)`)
    ``` C
    struct slash_qdma_info {
      __u32 size;          /**< Struct size for ABI versioning. */

      /* Kernel to userspace */
      __u32 qsets_max;     /**< [out] Maximum number of queue sets the hardware supports. */
      __u32 msix_qvecs;    /**< [out] Number of MSI-X vectors available for queues. */
      __u32 vf_max;        /**< [out] Maximum number of virtual functions. */
      __u32 caps;          /**< [out] Capability bitmask. */
      __u32 acc_type;      /**< [out] (NEW) Bitflags describing the accelerator type. Currently: 0x1: System-Emulated. */
    };
    ```
    * Extension of the existing INFO ioctl, but now also including the accelerator type
      * Used by libslash to choose the right code paths
  * "QGROUP_ADD" IOCTL (`#define SLASH_QDMA_IOCTL_QGROUP_ADD _IOWR('v', 0xXX, struct slash_qdma_qgroup_add)`)
    ``` C
    /** Maximum length (including NUL) of a QDMA file path (e.g. "/dev/slash/0000:61:00/qdma/qgroup42"). */
    #define SLASH_QDMA_MAX_PATH_LEN 128

    struct slash_qdma_qgroup_add {
      __u32 size;          /**< Struct size for ABI versioning. */

      /* Userspace to kernel */
      __u32 mode;          /**< [in]  Queue operating mode. */
      __u32 dir_mask;      /**< [in]  Direction bitmask — which directions to enable. */
      __u32 mm_channel;    /**< [in]  AXI-MM/NoC channel selection (enum slash_qdma_mm_channel). */

      __u32 h2c_ring_sz;   /**< [in]  Host-to-card descriptor ring size. */
      __u32 c2h_ring_sz;   /**< [in]  Card-to-host descriptor ring size. */
      __u32 cmpt_ring_sz;  /**< [in]  Completion ring size. */
      __u32 n_qpairs;      /**< [in]  No. of qpairs to allocate for the group. */

      /* Kernel to userspace */
      char queue_path[SLASH_QDMA_MAX_PATH_LEN]; /**< [out] Path to the newly created queue group file. */
    };
    ```
    * Extension of the existing `SLASH_QDMA_IOCTL_QPAIR_ADD`
      * Now allocates a desired number of queue pairs and associates them with a queue group file
    * Output: Path to the newly created queue group file `/dev/slash/<BDF>/qdma/qgroup<QID>`
      * Owner and group is the user and group of the ioctl-calling process
      * In practise, this will be the VRTD daemon anyways
      * Mode 700
    * An open FD to a qgroup can be handed from VRTD to VRT to
      * Allow users to manage their host buffers
      * Transfer data
* `/dev/slash/<BDF>/qdma/qgroup<Q>`
  * Does not support any data path operations itself
    * Just a handle for resources (qpairs, buffers) and the permission to create buffers and transfer data
  * IOCTL `#define SLASH_QDMA_IOCTL_BUF_CREATE _IOWR('v', 0xXX, struct slash_qdma_buf_create)`
    ``` C
    struct slash_qdma_buf_create {
      __u32 size;                             /**< Struct size for ABI versioning. */

      /* Userspace to kernel */
      __u32 pad;                              /**< [in]  Padding. */
      __u64 length;                           /**< [in]  Buffer length in bytes (page multiple). */

      /* Kernel to userspace */
      __u64 granule;                          /**< [out] Bytes per SGL descriptor (host page size). */
      char buf_path[SLASH_QDMA_MAX_PATH_LEN]; /**< [out] Path to the newly created buffer file. */
    };
    ```
    * Allocates a host buffer, to be used for memory transfers
    * Accessible as `/dev/slash/<BDF>/qdma/buffer<bid>`
      * Owner and group is the user and group of the ioctl-calling process
      * Mode 700
      * Allows user processes that received a qgroup FD from VRTD to create new buffers on their own.
    * Kernel behavior:
      * allocates @length bytes of host memory as a set of 4 KiB base pages (not physically contiguous)
      * builds the transfer scatter-gather list
      * DMA-maps every page once.
      * Installs new inode plus filesystem entry for the buffer file
      * Returns path to buffer file
    * System emulation daemon behavior:
      * Create a new memory-backed FUSE file
      * Installs new inode plus filesystem entry for the buffer file
      * Returns path to buffer file
* `/dev/slash/<BDF>/qdma/buffer<B>`
  * File backed by host memory
  * Associated with a qgroup
  * Supports read, write, lseek, pread, pwrite, mmap
    * Which only modify the underlying host buffer
  * IOCTL `#define SLASH_QDMA_BUFFER_IOCTL_TRANSFER _IOWR('v', 0xXX, struct slash_qdma_transfer)`
    ``` C
    #define SLASH_QDMA_FD_MAX_QPAIRS 2u

    struct slash_qdma_subxfer {
      __u32 qpair_index; /**< [in] Index to the fd's bound qpair. */
      __u32 direction;   /**< [in] enum slash_qdma_transfer_dir (H2C or C2H). */
      __u32 pad0;        /**< Padding for natural alignment. */
      __u64 buf_offset;  /**< [in] Byte offset within the buffer. */
      __u64 dev_addr;    /**< [in] Device-side (endpoint) address. */
      __u64 length;      /**< [in] Number of bytes to transfer. */
    };
    struct slash_qdma_transfer {
      __u32 size;        /**< Struct size for ABI versioning. */
      __u32 count;       /**< [in] Number of sub-transfers (1..SLASH_QDMA_FD_MAX_QPAIRS). */
      struct slash_qdma_subxfer xfers[SLASH_QDMA_FD_MAX_QPAIRS]; /**< [in] Sub-transfers. */
    };
    ```
    * Initiates one or more transfers, either from the host buffer to the accelerator or back
    * Requires a flush, so that data has arrived at the host buffer in the FUSE server
    * Kernel behavior:
      * Each sub-transfer uses one of the indicated queue pairs out of those owned by the queue group
      * If different queue pairs are used, transfers happen in parallel
    * System emulation daemon behavior:
      * The referenced data from the host buffer is sent to the model server via ZeroMQ
* Expected usage pattern for VRTD:
  * User (VRT) requests a queue group
    * With a given number of queue pairs
  * VRTD issues "QGROUP_ADD" ioctl
  * Opens the newly created file qgroup file, unlinks it
  * Passes the qgroup FD to the user (VRT)
  * Effect:
    * Maintains "delete-on-last-close" AND automatic resource freeing
    * Because the qgroup is unlinked while open, it is nameless for its most of its life
    * Each user's request for a QDMA FD creates a new queue and file
      * The users/VRT may therefore assume that they are the only process with an FD to that file
  * VRT uses the qgroup FD to create host buffers
    * Opens host buffer file, immediately unlinks it
  * User application stages their data in the host buffer
  * User/VRT initiates a transfer, potentially using multiple queues at once
  * User application executes compute kernels via BARs (see below)
  * User/VRT initiates transfers back, user fetches data from host buffers
* VRTD prunes QDMA pairs that it finds during startup
  * Thus frees artifacts from a previous crash
* Memory ranges (HBM banks, DDR, reconfiguration target)
  * Part of UAPI header in libslash
  * HBM: 0x0000004000000000ULL to 0x0000004800000000ULL
    * Split in 64 banks with 512MiB each
  * DDR: 0x0000060000000000ULL to 0x0000060800000000ULL
    * Split into four banks with 8GiB each
  * reconfiguration region: 0x0000000102100000ULL to 0x0000000142100000ULL
  * Accesses beyond these regions are undefined

### BAR Access - traditional with one file for the entire BAR

* Exposes the BARs of the accelerator's PF 2:
  * Codified in the UAPI header, static during the lifetime of the accelerator
  * BAR 0: User region
    * Size: 128 MB
  * BAR 2: Service Layer
    * Size: 128 MB
  * BAR 4: Clock wizard
    * Size: 512 KB
* `/dev/slash/<BDF>/bars/`
  * Directory
  * IOCTL `#define SLASH_CTLDEV_IOCTL_GET_DEVICE_INFO _IOWR('v', 0x32, struct slash_ioctl_device_info)`
    ``` C
      /** Maximum length (including NUL) of a PCI BDF string ("DDDD:BB:DD.F"). */
      #define SLASH_PCI_BDF_LEN 32

      struct slash_ioctl_device_info {
        __u32 size;                       /**< Struct size for ABI versioning. */

        /* Kernel to userspace */
        char bdf[SLASH_PCI_BDF_LEN];      /**< [out] PCI Bus/Device/Function string, NUL-terminated. */
        __u16 vendor_id;                  /**< [out] PCI vendor ID. */
        __u16 device_id;                  /**< [out] PCI device ID. */
        __u16 subsystem_vendor_id;        /**< [out] PCI subsystem vendor ID. */
        __u16 subsystem_device_id;        /**< [out] PCI subsystem device ID. */
        __u32 acc_type;                   /**< [out] (NEW) Bitflags describing the accelerator type. Currently: 0x1: System-Emulated. */
      };
    ```
    * Extension of the existing INFO ioctl, but now also including the accelerator type
      * Used by libslash to choose the right code paths
    * vendor_id=subsystem_vendor_id=0x10EE for AMD/Xilinx, device_id=0x50B6 for PF2, subsystem_device_id=0x000e
* `/dev/slash/<BDF>/bars/bar<M>`
  * File
    * Reads and writes the BAR M of the physical function 2
    * Only meant for register access
    * Supports read, write, llseek, pread, pwrite
    * no buffering, no page caching
      * Each syscall has to reach the kernel driver/system emulation daemon directly
      * Should therefore always be opened with `O_SYNC | O_DIRECT`
    * width == transfer size
      * reject operations with size not in {1, 2, 4, 8} or incorrect alignment
  * Explicitly no Mmap'ing to enable kernel-side checks and easier emulation
    * Also keeps revocation cheap: every access is a file op, so removal needs only a liveness check, no PTE zapping
    * Note: The higher latency of one system call per BAR access is a cost we're willing to take
  * Example usage pattern:
    * Scenario: Kernel at offset 0x10000. 
      * All registers 32 bits wide
      * Control register at 0x10000
      * Input parameter registers starting at 0x10004
    * Pattern:
      * `llseek` to 0x10004
      * one 4-byte `write` per register
        * Advancing the file position with each write
      * finally, one `pwrite` to 0x10000 to start the kernel
    * Necessitates proper synchronization within the user, of course
  * Like qgroups, BAR fds survive device removal as orphans returning `-ENODEV` until closed (see Hotplugging/Resets)
  * Size of the BAR encoded as the size of the file
* Side note: The start address attribute has been dropped and is not reported anymore

### Hotpluging/Resets

* `/dev/slash/hotplug`
  * File, only allowing IOCTLs
    * Generally the same behavior for hardware as before
    * Behavior for system emulation adapted accordingly
  * `#define SLASH_HOTPLUG_IOCTL_RESCAN _IO('w', 0x30)`
    * On hardware: Rescans all PCI root buses to discover new or reconfigured devices. Typically called after REMOVE or TOGGLE_SBR to rediscover a device.
    * On system-emulation: Reloads configuration and sets up all system-emulated accelerators
      * Skips accelerators who's BDF collides with another running accelerator
  * `#define SLASH_HOTPLUG_IOCTL_REMOVE _IOW('w', 0x31, struct slash_hotplug_device_request)`
    * The corresponding `/dev/slash/<BDF>/bars` or `/dev/slash/<BDF>/qdma` directory disappears/is removed
      * Function 1 corresponds to QDMA, Function 2 corresponds to the control register BARs
    * Revocation semantics (both backends, enforced by the conformance suite):
      * Resources are eagerly revoked
      * New `open`/lookup of a removed endpoint returns `-ENOENT`
      * Any op on an already-open fd of a removed endpoint returns `-ENODEV`
      * `close` always succeeds and remains the holder's responsibility; `release` is idempotent
    * On hardware: Removes a PCI device identified by BDF from the PCI hierarchy, triggering the driver’s .remove callback.
      * Postconditions:
        * Bus mastering is disabled on the device (pci_clear_master()).
        * The device is removed from the PCI hierarchy (pci_stop_and_remove_bus_device()).
        * The driver’s .remove callback is invoked; associated device nodes disappear.
    * On system-emulation: Removes the endpoint folders and revokes the communication resources (e.g. QDMA queue pairs) per the semantics above
      * Marks open handles dead (subsequent ops return `-ENODEV`)
      * Invalidates names via `fuse_lowlevel_notify_delete`/`notify_inval_entry`
      * Tracks open handles in its own per-device structures, since FUSE nodes may be unlinked while open
      * Effectively removes the means to communicate while leaving the vpp_emu/vpp_sim in the background running
      * Shut down vpp_emu/vpp_sim only once both Function 1 and 2 are removed
  * `#define SLASH_HOTPLUG_IOCTL_TOGGLE_SBR _IOW('w', 0x32, struct slash_hotplug_device_request)`
    * Asserts a secondary bus reset (SBR) on the upstream PCIe bridge for the bus specified by BDF, performing a full hardware reset of all endpoints on that bus. The ioctl blocks for approximately 1000 ms internally for PCIe link retraining; userspace should wait an additional 5–10 seconds after the call returns before rescanning.
    * bdf must be a valid DDDD:BB:DD.F string; only the domain and bus number are used to locate the upstream bridge
    * On hardware:
      * Preconditions:
        * The endpoint device may have been removed before calling; the kernel resolves the bridge via the bus number, which persists after endpoint removal
      * Postconditions:
        * Bridge config space is saved, PCI_BRIDGE_CTL_BUS_RESET is asserted for at least 2 ms, deasserted, and config space is restored.
        * The ioctl sleeps 1000 ms for PCIe link retraining before returning.
        * The PCIe link is retrained; the FPGA may still be initializing after return.
    * On system-emulation:
      * Fully remove the referenced system-emulated accelerator
      * Reload the configuration
      * Re-initialize all configured accelerators who's BDF is currently available
      * Emulate the 1s sleep
  * `#define SLASH_HOTPLUG_IOCTL_HOTPLUG _IOW('w', 0x33, struct slash_hotplug_device_request)`
    * Atomically removes and rescans a single PCI device under the PCI lock.
    * This is equivalent to REMOVE followed immediately by RESCAN on the same parent bus, without releasing the lock between operations.
    * Does not include an SBR; use TOGGLE_SBR separately if a hardware reset is needed.
    * On hardware:
      * Preconditions:
        * The device and its parent bus must exist in the PCI subsystem
      * Postconditions:
        * The device is removed (pci_clear_master() + pci_stop_and_remove_bus_device()).
        * The parent bus is rescanned (pci_rescan_bus()); the device reappears if hardware is present.
        * Both operations complete atomically under pci_lock_rescan_remove().
    * On system-emulation:
      * Fully remove a system-emulated accelerator
      * Reload the configuration
      * Re-initialize all configured accelerators who's BDF is currently available


#### Type definitions:

``` C
#define SLASH_HOTPLUG_BDF_LEN 32

struct slash_hotplug_device_request {
    __u32 size;                        /* ABI version: set to sizeof(struct) */
    char  bdf[SLASH_HOTPLUG_BDF_LEN]; /* NUL-terminated PCI BDF, *including function*, e.g. "0000:03:00.0" */
};
```

## Authorization of user processes

* Only VRTD can open files, checked using normal UNIX file permissions
* VRTD checks permissions of users (according to configuration)
  * Then creates/opens files for them
  * Passes the FD via SCM_RIGHTS
* Thus, permission checking for data plane operations is done by the kernel
  * Either a process *is* VRTD, or has received an FD from VRTD to run a data plane operation
  * No need to handroll this performance and security critical component

## Implementation of the file system

* System emulation daemon exposes a FUSE filesystem exactly matching the ABI described above
  * No passing of FDs by the kernel ABI, no mmap's, no trouble!
* Kernel module:
  * Implementing a new filesystem is something that many drivers do
    * Should be doable
  * Example: drivers/android/binderfs.c
  * Framework: fs/libfs.c
    * Register a new filesystem type, which is then mounted by userspace
    * Implement all operations based on this structure
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
  * New configuration must be written in one "write" operation
  * Otherwise, start and end of reconfiguration writing impossible to identify
* On hardware: QDMA of the DCP to AVED, just as before
* In system emulation:
  * Especially FPGA simulation needs many other files from the VBIN
  * Therefore: User has to transfer entire VBIN to emulation daemon via "QDMA"
  * System emulation daemon unpacks VBIN somewhere
  * Runs either vpp_emu or vpp_sim in a hardened systemd transient unit (daemon must be privileged, model is not)
    * To note: Emu is self-contained, but sim needs Vivado and potentially network egress to check licenses
    * Data plane: model talks to daemon over unix socket in scratch (replace tcp://localhost:5555), never sees the FUSE mount
  * MVP (step 1): runs the model directly under the privileged daemon, unsandboxed

## Local FPGA emulation/simulation in VRT, necessary branches on system-emulation in VRT/libVRTD

* Local emulation/simulation in VRT must remain
  * Needed for setups where the user isn't authorized to use the centrally system-emulated accelerators
  * Also for setups where the daemons aren't even running
    * For example CI
  * Apart from that, it's just very valuable in many instances
* New pattern how to decide how to execute:
  * If given BDF is "local" (case-insensitive)
    * If VBIN platform is emulation or simulation, run in local emulation/simulation
    * If VBIN platform is hardware, throw an appropriate exception
  * Otherwise, query VRTD with the given BDF
    * Of course, throw an appropriate exception if the accelerator doesn't exist
    * Accept if:
      * VBIN platform is emulation or simulation, and accelerator is system-emulated
      * VBIN platform is hardware and accelerator is hardware
* Identifying whether an accelerator is usable with the given VBIN is the only instance where VRT should branch on system-emulation or hardware
  * If this is not possible, there is a design error in this architecture
* The branch between writing a PDI or the entire VBIN is done in libvrtd
  * Again, only instance where this branch is necessary
  * Everything else is a design error

## Emulation daemon configuration & deployment

* Accelerators are persistent (persist beyond the lifetime of a user process)
* Accelerator configuration covers:
  * Accelerator BDF
  * Space in the schema for network configuration (to be done in the future)
* Everything else currently hard-wired
* Command line arguments:
  * Path to config, Path to mount path
* Ship with systemd units to tie the daemon in

## Testing and Code quality

* Principle: unit AND integration tests on everything — every component, every layer
* Unit tests: component-internal logic (GTest for C++, equivalent for C); ship with the component, not later
* Integration tests: each component against its real interface/neighbors
* ABI conformance = one kselftest suite (modeled on `driver/tests/`), the single source of truth for the ABI
  * Same suite runs against BOTH the FUSE daemon AND the kernel module → guarantees they don't drift
  * Must cover revocation: removal with fds still open, then `-ENODEV` on ops, `-ENOENT` on reopen, and clean `close`
* Every implementation step below ships its tests as part of that step
* Also, the system emulation daemon must follow the code style of VRTD as described in `vrt/vrtd/STYLE.md`.

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

## Open issues to resolve before refactoring (post perf-merge review)

The QDMA section above was written against the pre-merge per-qpair model. The
performance refactor (merge a3310595) made per-qpair channel pinning and the
fd-returned-directly buffer load-bearing. The following must be resolved before
commencing the refactor. Severity tags: **[BLOCK]** blocks current usage,
**[PERF]** erases the perf win, **[SPEC]** under-specified gap.

### Resolve the QDMA ABI mismatches

* **[BLOCK] A single `mm_channel` per qgroup cannot express the dual-channel buffer.**
  * The perf path backs one buffer with two qpairs on *different* channels:
    `vrt/vrtd/src/buffer.c:78-86` pins `qpair[0]→MM_CHANNEL_0`, `qpair[1]→MM_CHANNEL_1`
    and binds them into one transfer fd. A group whose `n_qpairs` qpairs all share
    one `mm_channel` cannot reproduce this split.
  * Fix: either make the channel per-qpair (`__u32 mm_channel[...]` in `qgroup_add`),
    or define that qpair index `i` is deterministically pinned to channel `i % n_channels`
    (and document `n_channels`).
* **[BLOCK] The `qpair_index → NoC channel` mapping is load-bearing but unspecified.**
  * The client placement policy (`vrt/vrtd/libvrtd/src/v80_policy.h`) picks `qpair_index`
    purely from the device physical address, relying on index 0 = channel 0, index 1 = channel 1
    (the contract documented at `buffer.c:67`).
  * Specify that `qpair_index` indexes into the owning qgroup's qpairs, in a fixed,
    client-discoverable channel order. Prerequisite for the fix above.
* **[BLOCK] `SLASH_QDMA_FD_MAX_QPAIRS = 2` vs. arbitrary `n_qpairs`; stale name.**
  * The TRANSFER struct caps `xfers[]` at 2, so a group with `n_qpairs > 2` is unusable
    in one transfer. Decide: cap `n_qpairs` at the same constant, or raise the per-transfer cap.
  * The name references the deleted "transfer fd binds qpairs" concept (transfers now ride
    the *buffer* fd). Rename, e.g. `SLASH_QDMA_QGROUP_MAX_QPAIRS`.
* **[BLOCK] `transfer_hint` was dropped from `buf_create`, but the shared client branches on it.**
  * The same libvrtd code runs against kernel and FUSE; it reads `transfer_hint`
    (`libvrtd/src/buffer.c:473`) to decide whether to run the V80 split.
  * Either keep `transfer_hint` in the buffer-create output, or define a single source
    (e.g. derive from `acc_type` + qgroup config). Do not just remove the field.
* **[BLOCK] Buffers can only be created via a qgroup — give the design writer a home.**
  * The design writer creates its DMA buffer on the device/control handle today
    (`slash_qdma_buffer_create(writer->qdma, …)`, `design_writer.c:283`) with its own
    single H2C qpair (ring idx 9) and aperture-wrapping loop. Under the new model it must
    QGROUP_ADD its own group, then BUF_CREATE. State this migration explicitly.
  * Also note as a deliberate constraint: buffers bound to a qgroup forecloses cross-group
    buffer reuse (acceptable — each `vrtd_buffer` already owns its qpairs).
* **[SPEC] Fix the stale `slash_qdma_subxfer.qpair_index` doc** ("Index to the fd's bound
  qpair") — the ioctl now rides the buffer fd; redefine as "index into the owning qgroup's qpairs."

### Address the path-based resource cost

* **[PERF] The bounce-buffer hot path becomes ~6 syscalls + inode churn per partial sync.**
  * Every sub-granule/unaligned sync creates and destroys a transient buffer
    (`libvrtd/src/buffer.c:447`, `:535`). "ioctl returns a path, open+unlink" turns one
    fd-returning ioctl into BUF_CREATE→open→unlink→mmap→TRANSFER→munmap→close + a real
    inode create/teardown (multi-message round trip on FUSE), on a path that fires on
    every unaligned transfer.
  * Decide: per-qgroup bounce-buffer pool, a pointer-based small-transfer ioctl, or
    explicitly document the accepted cost.
* **[PERF] FUSE buffer mmap is hand-waved by "requires a flush".**
  * MAP_SHARED writeback so the daemon sees client writes (H2C) and page invalidation
    after the daemon fills them (C2H) is nontrivial in FUSE.
  * It forces an `msync`/invalidate into the *shared* client path the hardware path
    doesn't need — a third hw/sysemu divergence beyond the two allowed. Make it uniform
    (always `msync`, cheap on kernel) rather than branching.
  * Validate FUSE shared-mmap writeback as a spike *before* step 2; fallback is
    `pwrite`/`pread` on the buffer file. Pin the contract now: client always mmaps +
    `msync` (H2C before transfer) + `msync(MS_INVALIDATE)`/re-read (C2H after transfer).

### Close the permission and behavior gaps

* **[SPEC] Specify who opens buffer files and the directory traversal permissions.**
  * Clients creating buffers via the qgroup fd must open the returned buffer path, which
    needs `+x` on `/dev/slash/<BDF>/` and `.../qdma/`. Resource-file modes (700) are
    specified; directory modes are not. Specify dirs `0755`, resource files `0700` — or
    decide VRTD brokers every open and passes fds (contradicts "create buffers on their own").
  * State that the ioctl returns the backend-correct absolute path
    (`/dev/slash/...` vs `/run/slash_emu/...`).
* **[SPEC] Define mmap behavior under revocation/hotplug for buffers.**
  * BARs forbid mmap to keep revocation cheap; buffers allow it. Sane rule for
    host-RAM-backed buffers: mapping stays valid, only TRANSFER returns `-ENODEV`.
    Add conformance coverage for "removal with a buffer still mmapped".
* **[SPEC] State that concurrent TRANSFER on distinct buffer fds sharing a qgroup is supported**
  (serialized per qpair, parallel across qpairs). VRT carves many buffers from one
  superblock → one qgroup, and syncs them concurrently.

### Forward-looking (deferred, but acknowledge now)

* **[SPEC] Streaming (ST mode) does not fit qgroup/buffer/transfer.**
  * `vrt/src/qdma/qdma_intf.cpp` uses a single ST-mode qpair with direct transfers; the
    new model exports no `read`/`write`/`poll`-able qpair fd. Streaming is deferred, but
    the "qpairs are internal-only, never exported" decision will need revisiting.
* **[SPEC] The libvrtdpp `QdmaQpair` escape hatch must be reframed.**
  * `VRTD_REQ_QDMA_QPAIR_ADD/OP/GET_FD` (libvrtdpp `qdma_qpair.cpp`) lets clients manage
    raw qpairs by qid. With qpairs internal-only this becomes qgroup management — call it
    out in steps 3/4.
