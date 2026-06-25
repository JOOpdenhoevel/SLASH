# New kernel ABI 

Idea: Kernel driver exposes a custom "SLASH" filesystem that is mounted in `/dev/slash`. Each accelerator receives its own directory named by device ID, e.g. `/dev/slash/0000:61:00/`. Each accelerator directory contains sub-folders and files to control the accelerator. All file operations are either `read`, `write` (both potentially with an offset), or `ioctl`. All written and returned data is plain-old-data. In particular, syscalls must not return file descriptors or other references that are only valid in the context of the calling process.

With these restrictions on the operations available for the ABI, it is possible and relatively straight-forward to emulate the kernel ABI with a FUSE filesystem, using a dedicated system emulation daemon. Such an daemon would expose a file system in `/run/slash-emu/` with the same structure as `/dev/slash`, but instead exposing ways to manipulate a system-emulated accelerator.

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
* Also, all folders have mode `0755` to discoverable by anyone, but resource files have mode `0600`.

* All endpoints are relative to the file system root
* Default roots:
  * Kernel driver: `/dev/slash`
  * System emulation daemon: `/run/slash-emu/`
* However, could be arbitrarily chosen for either backend
  * Backends thus have to know their mount points
* Returned file paths are always absolute
  * So that libvrtd/VRT doesn't need to construct correct paths based on the backend

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
* Maximum path length is 128 bytes (including null character)
  * Requests that would yield a longer path fail with -ENAMETOOLONG.
  * Admins have to choose a short root for the backend
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

* `<BDF>/qdma/`
  * Directory
  * "INFO" IOCTL (`#define SLASH_QDMA_IOCTL_INFO _IOWR('v', 0xXX, struct slash_qdma_info)`)
    ``` C
    enum slash_acc_features {
      /**
       * @brief The accelerator is system-emulated
       * 
       * If set, the accelerator must be reconfigured with a full FPGA emulation or simulation
       * VBIN, i.e. one that contains a `vpp_emu` or `vpp_sim` executable.
       * 
       * If not set, the accelerator must be reconfigured with a user region DCP.
       */
      SLASH_ACC_FEATURES_EMULATED = 0b1u;

      /**
       * @brief The accelerator supports dual-channel transfers, as prototypical for the Alveo V80.
       *
       * If set, users should apply the V80 transfer policy using both channels for full performance.
       */
      SLASH_ACC_FEATURES_V80 = 0b10u;
    };

    struct slash_qdma_info {
      __u32 size;          /**< Struct size for ABI versioning. */

      /* Kernel to userspace */
      __u32 qsets_max;     /**< [out] Maximum number of queue sets the hardware supports. */
      __u32 msix_qvecs;    /**< [out] Number of MSI-X vectors available for queues. */
      __u32 vf_max;        /**< [out] Maximum number of virtual functions. */
      __u32 caps;          /**< [out] Capability bitmask. */
      __u32 acc_features;  /**< [out] (NEW) Bitflags describing the accelerator features, values from enum slash_acc_features */
      __u32 n_mm_channels; /**< [out] (NEW) The number of AXI-MM/NoC channels supported. */
    };
    ```
    * Extension of the existing INFO ioctl, but now also including the accelerator features
      * Used by libslash to choose the right code paths
      * system emulator sets EMULATED and V80, driver only sets V80
    * The V80 feature flag only provides a suggestion for the transfer policy
      * But it guarantess that `n_mm_channels == 2`
    * `n_mm_channels` is a hard feature description
      * Trying to access more channels then that lead to an error.
  * "QGROUP_ADD" IOCTL (`#define SLASH_QDMA_IOCTL_QGROUP_ADD _IOWR('v', 0xXX, struct slash_qdma_qgroup_add)`)
    ``` C
    /** Maximum length (including NUL) of a QDMA file path (e.g. "/dev/slash/0000:61:00/qdma/qgroup42"). */
    #define SLASH_QDMA_MAX_PATH_LEN 128
    #define SLASH_QDMA_MAX_QPAIRS 2u

    struct slash_qdma_qgroup_add {
      __u32 size;                                 /**< Struct size for ABI versioning. */

      /* Userspace to kernel */
      __u32 mode;                                 /**< [in]  Queue operating mode. */
      __u32 dir_mask;                             /**< [in]  Direction bitmask — which directions to enable. */

      __u32 h2c_ring_sz;                          /**< [in]  Host-to-card descriptor ring size. */
      __u32 c2h_ring_sz;                          /**< [in]  Card-to-host descriptor ring size. */
      __u32 cmpt_ring_sz;                         /**< [in]  Completion ring size. */

      __u32 n_qpairs;                             /**< [in]  No. of qpairs to allocate for the group. */
      __u32 mm_channel[SLASH_QDMA_MAX_QPAIRS];    /**< [in]  AXI-MM/NoC channel selection (one per qpair). */

      /* Kernel to userspace */
      char queue_path[SLASH_QDMA_MAX_PATH_LEN];   /**< [out] Path to the newly created queue group file. */
    };
    ```
    * Extension of the existing `SLASH_QDMA_IOCTL_QPAIR_ADD`
      * Now allocates a desired number of queue pairs and associates them with a queue group file
      * Number of qpairs must be <= SLASH_QDMA_MAX_QPAIRS
      * Channel selection:
        * Channel indices must be less than `n_mm_channels`, as returned by the info ioctl
        * Multiple qpairs targeting the same channel are valid/correct, but may lead to lower throughput
        * Superfluous `mm_channel` entries are ignored
    * Output: Path to the newly created queue group file `<BDF>/qdma/qgroup<QID>`
    * An open FD to a qgroup can be handed from VRTD to VRT to
      * Allow users to manage their host buffers and transfer data
    * QDMA queue starts and stops are internal now
      * Done automatically at resource allocation and release
      * Thus, exposed operations are removed
* `<BDF>/qdma/qgroup<Q>`
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
    * Accessible as `<BDF>/qdma/buffer<bid>`
    * The `transfer_hint` has been removed.
      * Instead, users should derive the channel policy from the device-wide accelerator features 
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
* `<BDF>/qdma/buffer<B>`
  * File backed by host/kernel memory
  * Associated with a qgroup
  * Supports read, write, lseek, pread, pwrite, mmap
    * Which only modify the underlying host buffer
  * IOCTL `#define SLASH_QDMA_BUFFER_IOCTL_TRANSFER _IOWR('v', 0xXX, struct slash_qdma_transfer)`
    ``` C
    struct slash_qdma_subxfer {
      __u32 qpair_index; /**< [in] Index to the qgroup's bound qpair. */
      __u32 direction;   /**< [in] enum slash_qdma_transfer_dir (H2C or C2H). */
      __u32 pad0;        /**< Padding for natural alignment. */
      __u64 buf_offset;  /**< [in] Byte offset within the buffer. */
      __u64 dev_addr;    /**< [in] Device-side (endpoint) address. */
      __u64 length;      /**< [in] Number of bytes to transfer. */
    };
    struct slash_qdma_transfer {
      __u32 size;        /**< Struct size for ABI versioning. */
      __u32 count;       /**< [in] Number of sub-transfers. */
      struct slash_qdma_subxfer xfers[SLASH_QDMA_MAX_QPAIRS]; /**< [in] Sub-transfers. */
    };
    ```
    * Initiates one or more transfers, either from the host buffer to the accelerator or back
    * Each sub-transfer may use a different qpair from the queue group
      * If possible, those transfers will happen in parallel
    * Ideally, each subxfer maps to one unique qpair, which maps to one unique MM channel
    * Kernel behavior:
      * Each sub-transfer uses one of the indicated queue pairs out of those owned by the queue group
    * System emulation daemon behavior:
      * The referenced data from the host buffer is sent to the model server via ZeroMQ, or back
      * The system emulation daemon supports two channels to the outside
      * But only executes subxfers sequentially
    * Users must always `msync` before calling this ioctl
      * Otherwise, unwritten data may be sent to the (system-emulated) accelerator
      * Similarly, the system emulation daemon must also invalidate the user mapping before returning

#### Expected usage pattern for VRT/libvrtd/VRTD:

* Permission and usage flow for user data:
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
    * Bounce-buffer overheads from unaligned memory transfers are an accepted cost
  * User application executes compute kernels via BARs (see below)
  * User/VRT initiates transfers back, user fetches data from host buffers

* Reconfiguration:
  * VRTD continuously owns a qgroup for reconfiguration
    * One qpair, H2C only
  * User/VRT sends an FD to the VBIN to VRTD
  * VRTD allocates a host buffer, writes DCP/VBIN to it
  * Initiates transfer
  * Closes host buffer

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
* `<BDF>/bars/`
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
        __u32 acc_features;               /**< [out] (NEW) Bitflags describing the accelerator type. Values from enum slash_acc_features */
      };
    ```
    * Extension of the existing INFO ioctl, but now also including the accelerator features
      * Used by libslash to choose the right code paths
      * These features must match those returned by the QDMA info ioctl
        * Features are returned by both since both can be independently queried
    * vendor_id=subsystem_vendor_id=0x10EE for AMD/Xilinx, device_id=0x50B6 for PF2, subsystem_device_id=0x000e
* `<BDF>/bars/bar<M>`
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

* `hotplug` file, only allowing IOCTLs
  * Generally the same behavior for hardware as before
  * Behavior for system emulation adapted accordingly
* `#define SLASH_HOTPLUG_IOCTL_RESCAN _IO('w', 0x30)`
  * On hardware: Rescans all PCI root buses to discover new or reconfigured devices. Typically called after REMOVE or TOGGLE_SBR to rediscover a device.
  * On system-emulation: Reloads configuration and sets up all system-emulated accelerators
    * Skips accelerators who's BDF collides with another running accelerator
* `#define SLASH_HOTPLUG_IOCTL_REMOVE _IOW('w', 0x31, struct slash_hotplug_device_request)`
  * The corresponding `<BDF>/bars` or `<BDF>/qdma` directory disappears/is removed
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

* Only VRTD can open control files
  * Ensured by normal UNIX file permissions
* VRTD checks permissions of users (according to configuration)
  * Then creates/opens files for them
  * Passes the FD via SCM_RIGHTS
* Thus, permission checking for data plane operations is done by the kernel
  * Either a process *is* VRTD, or has received an FD from VRTD to run a data plane operation
  * No need to handroll this performance and security critical component
* Exception: Newly created buffer files are immediately accessible to the user that created them
  * The permission to do is handed to a user process via a qgroup FD.

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
  * Default configuration shipped with VRTD lists `/dev/slash` and `/run/slash-emu`
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

5. Buffer mmap behavior under revocation/hotplug. The deleted item proposed: host-RAM-backed mapping stays valid, only TRANSFER returns -ENODEV, plus conformance coverage for "removal with a
buffer still mmapped." The hotplug section (304-363) covers BAR fds and qgroups but says nothing about mmap'd buffer pages. BARs forbid mmap specifically to keep revocation cheap (line 286-288);
buffers allow it, so the revocation story for a live mapping is a genuine gap — and your conformance suite is supposed to be the source of truth for revocation (line 463). This needs to be in
the buffer or hotplug section, not deleted.

9. Design-writer aperture wrapping isn't expressible in the new reconfig flow. The reconfiguration flow (lines 225-231) is "write VBIN to buffer → initiate transfer → close." But
design_writer.c:126,297-308 feeds the PDI through a 64 KiB boot-stream aperture, wrapping dev_addr with off % APERTURE_BYTES so a multi-MB PDI doesn't advance linearly. A single TRANSFER with
one dev_addr+length can't reproduce that — the writer must issue one TRANSFER per aperture chunk to a fixed window. The plan should say the design-writer migration preserves the aperture loop (N
transfers), or the reconfiguration will silently linearly-address a PDI that must wrap.

E. "Return absolute paths" is clean for FUSE but architecturally awkward for the kernel backend. A kernel filesystem has no single canonical absolute path — the same superblock can be mounted at
multiple points and in other mount namespaces, so "backends know their mount points" (line 31) is well-defined for the FUSE daemon (it's told its mountpoint) but not for the kernel module. It
is doable — the kernel must derive the path from the caller's own managing fd (d_path() on file->f_path of the qgroup/qdma dir, then append the new name), not from any global notion of mount
point. Two options: (1) keep absolute paths but spec that the kernel derives them from the caller's fd path; or (2) return root-relative paths and let the client prepend the discovery root it
already knows (lines 486-490) — which sidesteps the whole problem for both backends at the cost of one join in libvrtd. Worth a sentence either way; right now line 31 hand-waves the hard part.
