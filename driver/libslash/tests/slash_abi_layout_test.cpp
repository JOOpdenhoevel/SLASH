/**
 * The MIT License (MIT)
 * Copyright (c) 2026 Advanced Micro Devices, Inc. All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software
 * and associated documentation files (the "Software"), to deal in the Software without restriction,
 * including without limitation the rights to use, copy, modify, merge, publish, distribute,
 * sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
 * NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

/**
 * Layout / ABI-stability test for the new filesystem-ABI UAPI header
 * slash_abi.h. Locks down struct sizes and offsets, ioctl numeric encodings,
 * and the memory-range / BAR geometry constants against
 * abi_rebuild_architecture.md.
 *
 * The hard invariants are expressed as static_assert (evaluated when this
 * translation unit is compiled as C++); the GTest cases re-check the same
 * values at runtime so failures surface as ordinary test reports. C-language
 * compilation of the UAPI header is exercised by the kselftest ABI
 * conformance suite, which is C.
 *
 * The legacy UAPI headers are included alongside the new one to prove the two
 * coexist in a single translation unit without redefinition conflicts.
 */

#include <gtest/gtest.h>

#include <cstddef>  // offsetof

extern "C" {
#include <slash/uapi/slash_abi.h>
#include <slash/uapi/slash_interface.h>
#include <slash/uapi/slash_hotplug.h>
}

// ─── Compile-time invariants ─────────────────────────────────────────────────

static_assert(SLASH_PCI_BDF_LEN == 32, "SLASH_PCI_BDF_LEN must be 32");
static_assert(SLASH_HOTPLUG_BDF_LEN == 32, "SLASH_HOTPLUG_BDF_LEN must be 32");

static_assert(sizeof(struct slash_info) == 40, "sizeof(struct slash_info) must be 40");
static_assert(offsetof(struct slash_info, size) == 0, "slash_info::size must be first");
static_assert(sizeof(((struct slash_info *)0)->bdf) == SLASH_PCI_BDF_LEN,
              "slash_info::bdf must be SLASH_PCI_BDF_LEN");
static_assert(SLASH_ACC_TYPE_SYSTEM_EMULATED == 0x1u, "SYSTEM_EMULATED flag must be 0x1");

static_assert(sizeof(struct slash_abi_qdma_qpair_add) == 28,
              "sizeof(struct slash_abi_qdma_qpair_add) must be 28");
static_assert(offsetof(struct slash_abi_qdma_qpair_add, size) == 0,
              "slash_abi_qdma_qpair_add::size must be first");

static_assert(sizeof(struct slash_abi_hotplug_device_request) == 36,
              "sizeof(struct slash_abi_hotplug_device_request) must be 36");
static_assert(offsetof(struct slash_abi_hotplug_device_request, size) == 0,
              "slash_abi_hotplug_device_request::size must be first");
static_assert(sizeof(((struct slash_abi_hotplug_device_request *)0)->bdf) == SLASH_HOTPLUG_BDF_LEN,
              "slash_abi_hotplug_device_request::bdf must be SLASH_HOTPLUG_BDF_LEN");

static_assert(SLASH_ABI_HOTPLUG_IOCTL_RESCAN == _IO('w', 0x30), "RESCAN must be _IO('w',0x30)");
static_assert(SLASH_ABI_HOTPLUG_IOCTL_REMOVE ==
                  _IOW('w', 0x31, struct slash_abi_hotplug_device_request),
              "REMOVE must be _IOW('w',0x31,...)");
static_assert(SLASH_ABI_HOTPLUG_IOCTL_TOGGLE_SBR ==
                  _IOW('w', 0x32, struct slash_abi_hotplug_device_request),
              "TOGGLE_SBR must be _IOW('w',0x32,...)");
static_assert(SLASH_ABI_HOTPLUG_IOCTL_HOTPLUG ==
                  _IOW('w', 0x33, struct slash_abi_hotplug_device_request),
              "HOTPLUG must be _IOW('w',0x33,...)");
static_assert(SLASH_ABI_QDMA_IOCTL_QPAIR_ADD ==
                  _IOWR('x', 0x40, struct slash_abi_qdma_qpair_add),
              "QPAIR_ADD must be _IOWR('x',0x40,...)");

static_assert(SLASH_HBM_BANKS * SLASH_HBM_BANK_SIZE == SLASH_HBM_END - SLASH_HBM_BASE,
              "HBM bank geometry must cover the full window");
static_assert(SLASH_DDR_BANKS * SLASH_DDR_BANK_SIZE == SLASH_DDR_END - SLASH_DDR_BASE,
              "DDR bank geometry must cover the full window");

// ─── Runtime checks (so GTest reports them) ──────────────────────────────────

TEST(SlashAbiLayout, StructSizesAndOffsets) {
    EXPECT_EQ(sizeof(struct slash_info), 40u);
    EXPECT_EQ(offsetof(struct slash_info, size), 0u);
    EXPECT_EQ(sizeof(struct slash_abi_qdma_qpair_add), 28u);
    EXPECT_EQ(sizeof(struct slash_abi_hotplug_device_request), 36u);
    EXPECT_EQ(SLASH_ACC_TYPE_SYSTEM_EMULATED, 0x1u);
}

TEST(SlashAbiLayout, IoctlEncodings) {
    EXPECT_EQ(SLASH_ABI_HOTPLUG_IOCTL_RESCAN, _IO('w', 0x30));
    EXPECT_EQ(SLASH_ABI_HOTPLUG_IOCTL_REMOVE,
              _IOW('w', 0x31, struct slash_abi_hotplug_device_request));
    EXPECT_EQ(SLASH_ABI_HOTPLUG_IOCTL_TOGGLE_SBR,
              _IOW('w', 0x32, struct slash_abi_hotplug_device_request));
    EXPECT_EQ(SLASH_ABI_HOTPLUG_IOCTL_HOTPLUG,
              _IOW('w', 0x33, struct slash_abi_hotplug_device_request));
    EXPECT_EQ(SLASH_ABI_QDMA_IOCTL_QPAIR_ADD,
              _IOWR('x', 0x40, struct slash_abi_qdma_qpair_add));
}

TEST(SlashAbiLayout, MemoryAndBarConstants) {
    EXPECT_EQ(SLASH_HBM_BASE, 0x0000004000000000ULL);
    EXPECT_EQ(SLASH_HBM_END, 0x0000004800000000ULL);
    EXPECT_EQ(SLASH_HBM_BANKS, 64u);
    EXPECT_EQ(SLASH_HBM_BANK_SIZE, 512ULL * 1024 * 1024);
    EXPECT_EQ(SLASH_DDR_BASE, 0x0000060000000000ULL);
    EXPECT_EQ(SLASH_DDR_END, 0x0000060800000000ULL);
    EXPECT_EQ(SLASH_DDR_BANKS, 4u);
    EXPECT_EQ(SLASH_DDR_BANK_SIZE, 8ULL * 1024 * 1024 * 1024);
    EXPECT_EQ(SLASH_RECONFIG_BASE, 0x0000000102100000ULL);
    EXPECT_EQ(SLASH_RECONFIG_END, 0x0000000142100000ULL);
    EXPECT_EQ(SLASH_BAR_USER_SIZE, 128ULL * 1024 * 1024);
    EXPECT_EQ(SLASH_BAR_SL_SIZE, 128ULL * 1024 * 1024);
    EXPECT_EQ(SLASH_BAR_CLK_SIZE, 512ULL * 1024);
}
