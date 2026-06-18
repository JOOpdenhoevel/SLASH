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
 * @file bridge_test.cpp
 * @brief Unit tests for the T10 SIM bridge, against slash_emu_core (no FUSE mount).
 *
 *   - emu_qdma_is_reconfig_write predicate matrix.
 *   - The model client protocol vs the CI stub model (spawned over an ipc://
 *     socket): start/exit handshake, reg/scalar round-trip, populate/fetch
 *     round-trip, byte-exactness.
 *   - The no-hang failure paths: a stub that never binds (handshake times out
 *     promptly) and one that never answers (each call times out, none wedges).
 *   - VBIN unpack + locate (a CI tar carrying the stub as vpp_sim), incl. the
 *     malformed-archive and missing-vpp_sim rejections.
 *
 * All bounded by the ctest TIMEOUT; the client's own per-call timeout is the
 * inner guard that makes the no-hang tests fast.
 */

#include <gtest/gtest.h>

#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include <fcntl.h>
#include <signal.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

extern "C" {
#include "model_client.h"
#include "qdma.h"
#include "slash/uapi/slash_abi.h"
#include "vbin.h"
}

namespace {

// ---------------------------------------------------------------------------
// Scratch helpers
// ---------------------------------------------------------------------------

std::string make_scratch_dir(const char *suffix)
{
    ::mkdir(SLASH_EMU_TMP_DIR, 0755);
    std::string tmpl =
        std::string(SLASH_EMU_TMP_DIR) + "/bridge_" + suffix + "_XXXXXX";
    std::vector<char> buf(tmpl.begin(), tmpl.end());
    buf.push_back('\0');
    char *r = ::mkdtemp(buf.data());
    EXPECT_NE(r, nullptr) << "mkdtemp: " << std::strerror(errno);
    return r ? std::string(r) : std::string();
}

void rm_rf(const std::string &path)
{
    std::string cmd = "rm -rf '" + path + "'";
    (void) ::system(cmd.c_str());
}

// Spawn the CI stub model bound to `endpoint`; returns its pid (or -1).  Extra
// env (key=value) entries are injected (e.g. to force the no-bind / hang modes).
pid_t spawn_stub(const std::string &endpoint,
                 const std::vector<std::string> &extra_env = {})
{
    pid_t pid = ::fork();
    if (pid < 0) {
        return -1;
    }
    if (pid == 0) {
        ::setenv("SLASH_EMU_ENDPOINT", endpoint.c_str(), 1);
        for (const auto &kv : extra_env) {
            auto eq = kv.find('=');
            ::setenv(kv.substr(0, eq).c_str(), kv.substr(eq + 1).c_str(), 1);
        }
        ::execl(SLASH_EMU_STUB_MODEL_PATH, "stub_model", (char *) nullptr);
        ::_exit(127);
    }
    return pid;
}

void reap(pid_t pid)
{
    if (pid <= 0) {
        return;
    }
    ::kill(pid, SIGKILL);
    int status = 0;
    ::waitpid(pid, &status, 0);
}

// Build a minimal ustar archive in memory containing `member_name` with the
// given file content and mode.  Used to produce a CI "VBIN".
std::vector<uint8_t> make_tar(const std::string &member_name,
                              const std::vector<uint8_t> &content,
                              unsigned mode = 0755)
{
    std::vector<uint8_t> out;
    auto block = [&]() { out.resize(out.size() + 512, 0); };

    size_t hdr = out.size();
    block();
    auto *h = out.data() + hdr;
    std::snprintf((char *) h, 100, "%s", member_name.c_str());      // name
    std::snprintf((char *) (h + 100), 8, "%07o", mode & 07777);     // mode
    std::snprintf((char *) (h + 108), 8, "%07o", 0);                // uid
    std::snprintf((char *) (h + 116), 8, "%07o", 0);                // gid
    std::snprintf((char *) (h + 124), 12, "%011o",
                  (unsigned) content.size());                       // size
    std::snprintf((char *) (h + 136), 12, "%011o", 0);             // mtime
    h[156] = '0';                                                   // typeflag REG
    std::memcpy(h + 257, "ustar", 5);                              // magic
    h[263] = '0';
    h[264] = '0';                                                   // version "00"

    // checksum: sum of header bytes with chksum field treated as spaces.
    std::memset(h + 148, ' ', 8);
    unsigned sum = 0;
    for (int i = 0; i < 512; i++) {
        sum += h[i];
    }
    std::snprintf((char *) (h + 148), 8, "%06o", sum);
    h[154] = '\0';
    h[155] = ' ';

    // file data, padded to 512.
    size_t data = out.size();
    out.resize(data + ((content.size() + 511) / 512) * 512, 0);
    std::memcpy(out.data() + data, content.data(), content.size());

    // two zero blocks (end of archive).
    block();
    block();
    return out;
}

// Read the stub model binary so we can pack it as the VBIN's vpp_sim member.
std::vector<uint8_t> read_stub_binary()
{
    FILE *f = ::fopen(SLASH_EMU_STUB_MODEL_PATH, "rb");
    EXPECT_NE(f, nullptr);
    std::vector<uint8_t> data;
    if (f != nullptr) {
        std::fseek(f, 0, SEEK_END);
        long sz = std::ftell(f);
        std::fseek(f, 0, SEEK_SET);
        data.resize((size_t) sz);
        size_t r = std::fread(data.data(), 1, (size_t) sz, f);
        EXPECT_EQ(r, (size_t) sz);
        std::fclose(f);
    }
    return data;
}

// ===========================================================================
// Unit: reconfig-region write predicate
// ===========================================================================

TEST(BridgeReconfigPredicate, RegionMatchedWriteOnly)
{
    EXPECT_EQ(emu_qdma_is_reconfig_write(SLASH_RECONFIG_BASE, 8), 1);
    EXPECT_EQ(emu_qdma_is_reconfig_write(SLASH_RECONFIG_END - 8, 8), 1);
    EXPECT_EQ(emu_qdma_is_reconfig_write(SLASH_RECONFIG_BASE,
                                         SLASH_RECONFIG_END - SLASH_RECONFIG_BASE),
              1);
}

TEST(BridgeReconfigPredicate, OutsideRegionNotMatched)
{
    EXPECT_EQ(emu_qdma_is_reconfig_write(SLASH_HBM_BASE, 8), 0);
    EXPECT_EQ(emu_qdma_is_reconfig_write(SLASH_DDR_BASE, 8), 0);
    EXPECT_EQ(emu_qdma_is_reconfig_write(SLASH_RECONFIG_BASE - 1, 1), 0);
    EXPECT_EQ(emu_qdma_is_reconfig_write(SLASH_RECONFIG_END, 1), 0);
}

TEST(BridgeReconfigPredicate, ZeroLengthAndOverflowNotMatched)
{
    EXPECT_EQ(emu_qdma_is_reconfig_write(SLASH_RECONFIG_BASE, 0), 0);
    EXPECT_EQ(emu_qdma_is_reconfig_write(SLASH_RECONFIG_END - 4, 8), 0); // straddle
    EXPECT_EQ(emu_qdma_is_reconfig_write(UINT64_MAX, 8), 0);            // overflow
}

// ===========================================================================
// Unit: model client protocol round-trip vs the stub
// ===========================================================================

class StubModel : public ::testing::Test {
protected:
    std::string dir;
    std::string endpoint;
    pid_t pid = -1;
    emu_model_client *client = nullptr;

    void SetUp() override
    {
        dir = make_scratch_dir("client");
        endpoint = "ipc://" + dir + "/model.sock";
    }
    void start_stub(const std::vector<std::string> &env = {})
    {
        pid = spawn_stub(endpoint, env);
        ASSERT_GT(pid, 0);
    }
    void connect()
    {
        ASSERT_EQ(emu_model_client_connect(endpoint.c_str(),
                                           EMU_MODEL_DEFAULT_TIMEOUT_MS, &client),
                  0);
    }
    void TearDown() override
    {
        if (client != nullptr) {
            emu_model_client_close(client);
        }
        reap(pid);
        rm_rf(dir);
    }
};

TEST_F(StubModel, StartRegScalarRoundTrip)
{
    start_stub();
    connect();
    ASSERT_EQ(emu_model_client_start(client), 0);

    ASSERT_EQ(emu_model_reg_write(client, 0x40, 0xdeadbeef), 0);
    uint32_t v = 0;
    ASSERT_EQ(emu_model_scalar_read(client, 0x40, &v), 0);
    EXPECT_EQ(v, 0xdeadbeefu);

    // Unwritten register reads zero.
    ASSERT_EQ(emu_model_scalar_read(client, 0x80, &v), 0);
    EXPECT_EQ(v, 0u);

    EXPECT_EQ(emu_model_client_exit(client), 0);
}

TEST_F(StubModel, PopulateFetchRoundTrip)
{
    start_stub();
    connect();
    ASSERT_EQ(emu_model_client_start(client), 0);

    std::vector<uint8_t> payload(256);
    for (size_t i = 0; i < payload.size(); i++) {
        payload[i] = (uint8_t) (i * 7 + 1);
    }
    ASSERT_EQ(emu_model_populate(client, SLASH_HBM_BASE, payload.data(),
                                 payload.size()),
              0);

    std::vector<uint8_t> got(256, 0xFF);
    ASSERT_EQ(emu_model_fetch(client, SLASH_HBM_BASE, got.data(), got.size()), 0);
    EXPECT_EQ(got, payload);

    // Unwritten memory fetches zero.
    std::vector<uint8_t> zeros(16, 0xAA);
    ASSERT_EQ(emu_model_fetch(client, SLASH_DDR_BASE, zeros.data(), zeros.size()),
              0);
    for (auto b : zeros) {
        EXPECT_EQ(b, 0u);
    }

    EXPECT_EQ(emu_model_client_exit(client), 0);
}

// ===========================================================================
// Unit: no-hang failure paths
// ===========================================================================

TEST_F(StubModel, HandshakeTimesOutWhenModelNeverBinds)
{
    // Use a short timeout so the test is fast; the stub binds nothing.
    start_stub({"SLASH_EMU_STUB_NO_BIND=1"});
    ASSERT_EQ(emu_model_client_connect(endpoint.c_str(), 300, &client), 0);

    // start must fail promptly (not hang) with a timeout-class errno.
    int rc = emu_model_client_start(client);
    EXPECT_LT(rc, 0);
    EXPECT_EQ(rc, -ETIMEDOUT);

    // After a transport failure the client is latched dead: subsequent calls
    // fail fast with -ENODEV, they do not block.
    uint32_t v = 0;
    EXPECT_EQ(emu_model_scalar_read(client, 0, &v), -ENODEV);
}

TEST_F(StubModel, CallTimesOutWhenModelNeverAnswers)
{
    start_stub({"SLASH_EMU_STUB_HANG=1"});
    ASSERT_EQ(emu_model_client_connect(endpoint.c_str(), 300, &client), 0);

    // The stub binds and accepts but never replies; the call must time out.
    int rc = emu_model_client_start(client);
    EXPECT_EQ(rc, -ETIMEDOUT);
}

// ===========================================================================
// Unit: VBIN unpack + locate
// ===========================================================================

TEST(BridgeVbin, UnpacksAndLocatesExecutableSim)
{
    std::string dir = make_scratch_dir("vbin");
    std::vector<uint8_t> stub = read_stub_binary();
    ASSERT_FALSE(stub.empty());

    std::vector<uint8_t> tar = make_tar("vpp_sim", stub, 0755);

    char exec[4096];
    ASSERT_EQ(emu_vbin_unpack_find_sim(tar.data(), tar.size(), dir.c_str(), exec,
                                       sizeof(exec)),
              0);
    EXPECT_EQ(::access(exec, X_OK), 0);
    rm_rf(dir);
}

TEST(BridgeVbin, NestedSimLocated)
{
    std::string dir = make_scratch_dir("vbin");
    std::vector<uint8_t> stub = read_stub_binary();
    std::vector<uint8_t> tar = make_tar("sub/dir/vpp_sim", stub, 0755);

    char exec[4096];
    ASSERT_EQ(emu_vbin_unpack_find_sim(tar.data(), tar.size(), dir.c_str(), exec,
                                       sizeof(exec)),
              0);
    EXPECT_EQ(::access(exec, X_OK), 0);
    rm_rf(dir);
}

TEST(BridgeVbin, MissingSimRejected)
{
    std::string dir = make_scratch_dir("vbin");
    std::vector<uint8_t> tar =
        make_tar("system_map.xml", {'<', 'x', '/', '>'}, 0644);

    char exec[4096];
    EXPECT_EQ(emu_vbin_unpack_find_sim(tar.data(), tar.size(), dir.c_str(), exec,
                                       sizeof(exec)),
              -ENOENT);
    rm_rf(dir);
}

TEST(BridgeVbin, MalformedArchiveRejected)
{
    std::string dir = make_scratch_dir("vbin");
    // Not block-aligned / too small.
    std::vector<uint8_t> junk(100, 0xAB);
    char exec[4096];
    EXPECT_EQ(emu_vbin_unpack_find_sim(junk.data(), junk.size(), dir.c_str(),
                                       exec, sizeof(exec)),
              -EINVAL);
    rm_rf(dir);
}

TEST(BridgeVbin, PathTraversalRejected)
{
    std::string dir = make_scratch_dir("vbin");
    std::vector<uint8_t> tar = make_tar("../escape", {'x'}, 0644);
    char exec[4096];
    EXPECT_EQ(emu_vbin_unpack_find_sim(tar.data(), tar.size(), dir.c_str(), exec,
                                       sizeof(exec)),
              -EINVAL);
    rm_rf(dir);
}

// ===========================================================================
// Unit: ustar completeness classifier (drives the chunk reassembly path)
// ===========================================================================

TEST(BridgeVbinClassify, EmptyIsIncomplete)
{
    EXPECT_EQ(emu_vbin_classify(nullptr, 0), EMU_VBIN_INCOMPLETE);
    uint8_t z = 0;
    EXPECT_EQ(emu_vbin_classify(&z, 0), EMU_VBIN_INCOMPLETE);
}

TEST(BridgeVbinClassify, CompleteArchiveRecognized)
{
    std::vector<uint8_t> tar = make_tar("vpp_sim", std::vector<uint8_t>(3000, 7));
    EXPECT_EQ(emu_vbin_classify(tar.data(), tar.size()), EMU_VBIN_COMPLETE);
}

TEST(BridgeVbinClassify, TruncatedPrefixesAreIncomplete)
{
    // Every strict prefix of a real archive short of the terminator must read as
    // INCOMPLETE -- never INVALID, never a premature COMPLETE -- which is exactly
    // what makes chunked reassembly safe.
    std::vector<uint8_t> tar =
        make_tar("vpp_sim", std::vector<uint8_t>(5000, 0xCD));
    size_t body_end = tar.size() - 1024; // start of the two trailing zero blocks
    for (size_t cut = 1; cut < body_end; cut += 137) {
        EXPECT_EQ(emu_vbin_classify(tar.data(), cut), EMU_VBIN_INCOMPLETE)
            << "prefix len " << cut << " should be INCOMPLETE";
    }
    EXPECT_EQ(emu_vbin_classify(tar.data(), 512 + 16), EMU_VBIN_INCOMPLETE);
}

TEST(BridgeVbinClassify, GarbageHeaderIsInvalid)
{
    // 512 non-zero bytes without the ustar magic: structurally invalid, no amount
    // of further bytes fixes it (a bogus first chunk fails fast).
    std::vector<uint8_t> junk(512, 0xAB);
    EXPECT_EQ(emu_vbin_classify(junk.data(), junk.size()), EMU_VBIN_INVALID);
}

TEST(BridgeVbinClassify, ChunkBoundaryMidBlockIsIncomplete)
{
    std::vector<uint8_t> tar = make_tar("vpp_sim", std::vector<uint8_t>(1000, 1));
    // A non-block-aligned prefix (a chunk boundary fell mid-block) is incomplete,
    // not invalid.
    EXPECT_EQ(emu_vbin_classify(tar.data(), 512 + 300), EMU_VBIN_INCOMPLETE);
}

} // namespace
