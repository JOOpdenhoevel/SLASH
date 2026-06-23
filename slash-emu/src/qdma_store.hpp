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
 * @file qdma_store.hpp
 * @brief The per-device sparse memory store (@ref QdmaStore) and the SIM bridge
 *        seams it carries.
 *
 * Split out of @ref qdma.cpp so the store can be owned at @em device scope
 * (@ref Device::qdmaStore) and survive a single-function QDMA REMOVE+RESCAN: a
 * per-function revoke destroys the @c qdma/ directory node and its
 * @ref QdmaDirOps, but the device keeps the store, so the rediscovered endpoint
 * reuses the same HBM/DDR contents (see @ref qdmaAttach).  The store is freed only
 * on a whole-device teardown (@ref NodeTree::revokeDevice), which drops the
 * device's reference.
 *
 * Defining the full type in a header (rather than @c qdma.cpp's anonymous
 * namespace) gives @ref node.cpp a complete type for the device-scoped
 * @c shared_ptr member and its out-of-line destructor, while keeping the store's
 * mechanics next to the QDMA endpoint that drives them.
 */

#ifndef SLASH_EMU_QDMA_STORE_HPP
#define SLASH_EMU_QDMA_STORE_HPP

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <unordered_map>

#include "qdma.hpp"

namespace slash::emu {

/*
 * Sparse memory store page size.  Device memory (32 GiB HBM + 32 GiB DDR) is far
 * too large to allocate, so the store is a paged, lazily-populated map: each
 * touched 64 KiB page is allocated on first write and zero-filled.  64 KiB keeps
 * the page table small while not wasting much on a single-byte poke.
 */
inline constexpr size_t kQdmaPageSize = 64u * 1024u;

/**
 * @brief The per-device sparse store: the page table plus the optional SIM bridge
 *        seam.
 *
 * Held by a @c std::shared_ptr owned at device scope (@ref Device::qdmaStore) and
 * co-owned by the @c qdma/ directory ops and every qpair of the device (one store
 * per device, shared by every qpair).  Mutated only from the ioctl/read/write
 * hooks, which the spine invokes with the tree lock held, so it needs no locking
 * of its own.
 *
 * A page is a lazily-allocated, zero-initialised 64 KiB block keyed by its
 * page-aligned device address; @c std::unordered_map both owns the pages (RAII, no
 * explicit free) and gives O(1) lookup, replacing the C linear-scan page array.
 */
struct QdmaStore {
    std::unordered_map<uint64_t, std::array<uint8_t, kQdmaPageSize>> pages;

    /* SIM memory-bridge seam.  Null => in-memory sparse store only. */
    QdmaMemBackend *backend = nullptr; /* borrowed, static */

    /* Reconfiguration seam: a write into the reconfig region is a VBIN. */
    QdmaReconfigFn reconfig; /* empty => no reconfig handler attached */

    /* Copy `len` bytes out of the store starting at device address `addr` into
     * `dst`, treating never-written pages as zero.  Range is pre-validated. */
    void read(uint64_t addr, void *dst, size_t len) const
    {
        auto *out = static_cast<uint8_t *>(dst);
        size_t done = 0;

        while (done < len) {
            uint64_t cur = addr + done;
            uint64_t base = cur - (cur % kQdmaPageSize);
            size_t in_page = static_cast<size_t>(cur - base);
            size_t chunk = kQdmaPageSize - in_page;
            if (chunk > len - done) {
                chunk = len - done;
            }

            auto it = pages.find(base);
            if (it == pages.end()) {
                std::memset(out + done, 0, chunk); /* never written: zero */
            } else {
                std::memcpy(out + done, it->second.data() + in_page, chunk);
            }

            done += chunk;
        }
    }

    /* Copy `len` bytes from `src` into the store starting at device address
     * `addr`, allocating zero-filled pages on demand.  Range is pre-validated. */
    void write(uint64_t addr, const void *src, size_t len)
    {
        const auto *in = static_cast<const uint8_t *>(src);
        size_t done = 0;

        while (done < len) {
            uint64_t cur = addr + done;
            uint64_t base = cur - (cur % kQdmaPageSize);
            size_t in_page = static_cast<size_t>(cur - base);
            size_t chunk = kQdmaPageSize - in_page;
            if (chunk > len - done) {
                chunk = len - done;
            }

            /* operator[] value-initialises (zero-fills) a fresh page. */
            auto &page = pages[base];
            std::memcpy(page.data() + in_page, in + done, chunk);

            done += chunk;
        }
    }
};

} // namespace slash::emu

#endif // SLASH_EMU_QDMA_STORE_HPP
