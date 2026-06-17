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
 * @file config_test.cpp
 * @brief Unit tests for the slash-emu config parser and accelerator model.
 *
 * These tests exercise the C config API (config.h) directly against the
 * slash_emu_core static library:
 *   - Parsing valid configs (1 and N accelerators), including BDF normalization.
 *   - Reserved network-key round-trip.
 *   - Rejection of malformed, empty-file, missing-file, duplicate-BDF, unknown
 *     section/key, and function-suffix configs.
 *   - Standalone BDF normalization edge cases.
 *   - The running set and the RESCAN reload/merge selection (collision skip).
 *
 * Configs are written to throwaway files under the repo .tmp scratch area
 * (SLASH_EMU_TMP_DIR, injected by CMake) so nothing touches /tmp and CTest
 * drives everything.
 */

#include <gtest/gtest.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

extern "C" {
#include "config.h"
}

namespace {

// RAII helper: writes `contents` to a unique file under the repo .tmp scratch
// dir and removes it on destruction.  Empty contents => an empty (0-byte) file.
class TempConfig {
public:
    explicit TempConfig(const std::string &contents)
    {
        ::mkdir(SLASH_EMU_TMP_DIR, 0755);

        std::string tmpl = std::string(SLASH_EMU_TMP_DIR) + "/slash_emu_cfg_XXXXXX";
        std::vector<char> buf(tmpl.begin(), tmpl.end());
        buf.push_back('\0');

        int fd = ::mkstemp(buf.data());
        EXPECT_GE(fd, 0) << "mkstemp failed: " << std::strerror(errno);
        path_ = buf.data();

        if (!contents.empty()) {
            ssize_t n = ::write(fd, contents.data(), contents.size());
            EXPECT_EQ(static_cast<size_t>(n), contents.size());
        }
        ::close(fd);
    }

    ~TempConfig()
    {
        if (!path_.empty()) {
            ::unlink(path_.c_str());
        }
    }

    const char *path() const { return path_.c_str(); }

private:
    std::string path_;
};

// Convenience: find an accelerator by BDF, returning nullptr if absent.
const struct emu_accelerator *find(const struct emu_config *cfg, const char *bdf)
{
    return emu_config_find(cfg, bdf);
}

}  // namespace

/* ---- BDF normalization (standalone) ---------------------------------- */

TEST(BdfNormalize, FullFormPassesThrough)
{
    char out[EMU_BDF_LEN];
    ASSERT_EQ(emu_bdf_normalize("0000:61:00", out, sizeof(out)), 0);
    EXPECT_STREQ(out, "0000:61:00");
}

TEST(BdfNormalize, ShortFormExpandsDomain)
{
    char out[EMU_BDF_LEN];
    ASSERT_EQ(emu_bdf_normalize("61:00", out, sizeof(out)), 0);
    EXPECT_STREQ(out, "0000:61:00");
}

TEST(BdfNormalize, UpperCaseHexLowered)
{
    char out[EMU_BDF_LEN];
    ASSERT_EQ(emu_bdf_normalize("00AB:CD:0F", out, sizeof(out)), 0);
    EXPECT_STREQ(out, "00ab:cd:0f");
}

TEST(BdfNormalize, FunctionSuffixRejected)
{
    char out[EMU_BDF_LEN];
    EXPECT_EQ(emu_bdf_normalize("0000:61:00.0", out, sizeof(out)), -1);
    EXPECT_EQ(emu_bdf_normalize("0000:61:00.1", out, sizeof(out)), -1);
}

TEST(BdfNormalize, MalformedRejected)
{
    char out[EMU_BDF_LEN];
    EXPECT_EQ(emu_bdf_normalize("", out, sizeof(out)), -1);
    EXPECT_EQ(emu_bdf_normalize("garbage", out, sizeof(out)), -1);
    EXPECT_EQ(emu_bdf_normalize("0000:61", out, sizeof(out)), -1);       // missing dev
    EXPECT_EQ(emu_bdf_normalize("0000:61:00:00", out, sizeof(out)), -1); // too many fields
    EXPECT_EQ(emu_bdf_normalize("zz:00", out, sizeof(out)), -1);         // non-hex bus
    EXPECT_EQ(emu_bdf_normalize("000:61:00", out, sizeof(out)), -1);     // short domain
    EXPECT_EQ(emu_bdf_normalize("0000:611:00", out, sizeof(out)), -1);   // long bus
}

TEST(BdfNormalize, RejectsUndersizedBuffer)
{
    char out[4];
    EXPECT_EQ(emu_bdf_normalize("0000:61:00", out, sizeof(out)), -1);
}

/* ---- Loading: defaults and valid configs ----------------------------- */

TEST(ConfigLoad, NullPathYieldsEmptyConfig)
{
    struct emu_config *cfg = nullptr;
    ASSERT_EQ(emu_config_load(nullptr, &cfg), 0);
    ASSERT_NE(cfg, nullptr);
    EXPECT_EQ(cfg->accelerators.len, 0u);
    EXPECT_EQ(cfg->source_path, nullptr);
    cleanup_config(cfg);
}

TEST(ConfigLoad, EmptyFileYieldsEmptyConfig)
{
    TempConfig tc("");
    struct emu_config *cfg = nullptr;
    ASSERT_EQ(emu_config_load(tc.path(), &cfg), 0);
    ASSERT_NE(cfg, nullptr);
    EXPECT_EQ(cfg->accelerators.len, 0u);
    EXPECT_STREQ(cfg->source_path, tc.path());
    cleanup_config(cfg);
}

TEST(ConfigLoad, MissingFileFails)
{
    std::string missing = std::string(SLASH_EMU_TMP_DIR) + "/does_not_exist_zzz.conf";
    ::unlink(missing.c_str());
    struct emu_config *cfg = nullptr;
    EXPECT_EQ(emu_config_load(missing.c_str(), &cfg), -1);
}

TEST(ConfigLoad, SingleAccelerator)
{
    // Each accelerator section must carry >=1 key (stock libinih ignores keyless
    // sections); a reserved net-* key satisfies that.
    TempConfig tc(
        "[accelerator:0000:61:00]\n"
        "net-ip = 10.0.0.1\n");
    struct emu_config *cfg = nullptr;
    ASSERT_EQ(emu_config_load(tc.path(), &cfg), 0);
    ASSERT_NE(cfg, nullptr);
    ASSERT_EQ(cfg->accelerators.len, 1u);
    EXPECT_STREQ(cfg->accelerators.d[0]->bdf, "0000:61:00");
    EXPECT_NE(find(cfg, "0000:61:00"), nullptr);
    EXPECT_EQ(find(cfg, "0000:62:00"), nullptr);
    cleanup_config(cfg);
}

TEST(ConfigLoad, MultipleAccelerators)
{
    TempConfig tc(
        "[accelerator:0000:61:00]\n"
        "net-ip = 10.0.0.1\n"
        "[accelerator:0000:62:00]\n"
        "net-ip = 10.0.0.2\n"
        "[accelerator:01ab:03:00]\n"
        "net-ip = 10.0.0.3\n");
    struct emu_config *cfg = nullptr;
    ASSERT_EQ(emu_config_load(tc.path(), &cfg), 0);
    ASSERT_NE(cfg, nullptr);
    ASSERT_EQ(cfg->accelerators.len, 3u);
    EXPECT_NE(find(cfg, "0000:61:00"), nullptr);
    EXPECT_NE(find(cfg, "0000:62:00"), nullptr);
    EXPECT_NE(find(cfg, "01ab:03:00"), nullptr);
    cleanup_config(cfg);
}

TEST(ConfigLoad, ShortFormBdfNormalizedInSection)
{
    TempConfig tc(
        "[accelerator:61:00]\n"
        "net-ip = 10.0.0.1\n");
    struct emu_config *cfg = nullptr;
    ASSERT_EQ(emu_config_load(tc.path(), &cfg), 0);
    ASSERT_EQ(cfg->accelerators.len, 1u);
    EXPECT_STREQ(cfg->accelerators.d[0]->bdf, "0000:61:00");
    cleanup_config(cfg);
}

TEST(ConfigLoad, ReservedNetworkKeysRoundTrip)
{
    TempConfig tc(
        "[accelerator:0000:61:00]\n"
        "net-mac  = 02:00:00:00:00:01\n"
        "net-ip   = 10.0.0.1\n"
        "net-port = 4791\n");
    struct emu_config *cfg = nullptr;
    ASSERT_EQ(emu_config_load(tc.path(), &cfg), 0);
    ASSERT_EQ(cfg->accelerators.len, 1u);
    const struct emu_accelerator *acc = cfg->accelerators.d[0];
    ASSERT_NE(acc->net.mac, nullptr);
    ASSERT_NE(acc->net.ip, nullptr);
    ASSERT_NE(acc->net.port, nullptr);
    EXPECT_STREQ(acc->net.mac, "02:00:00:00:00:01");
    EXPECT_STREQ(acc->net.ip, "10.0.0.1");
    EXPECT_STREQ(acc->net.port, "4791");
    cleanup_config(cfg);
}

TEST(ConfigLoad, MissingNetworkKeysStayNull)
{
    // Only net-mac is set (the required key); the other two reserved keys must
    // remain NULL.
    TempConfig tc(
        "[accelerator:0000:61:00]\n"
        "net-mac = 02:00:00:00:00:01\n");
    struct emu_config *cfg = nullptr;
    ASSERT_EQ(emu_config_load(tc.path(), &cfg), 0);
    ASSERT_EQ(cfg->accelerators.len, 1u);
    const struct emu_accelerator *acc = cfg->accelerators.d[0];
    EXPECT_STREQ(acc->net.mac, "02:00:00:00:00:01");
    EXPECT_EQ(acc->net.ip, nullptr);
    EXPECT_EQ(acc->net.port, nullptr);
    cleanup_config(cfg);
}

/* ---- Loading: rejection paths ---------------------------------------- */

// Two byte-identical section headers are MERGED by stock libinih before our
// callback sees them (it cannot tell "section reopened" from "more keys in the
// same section"), so this is NOT a detectable duplicate -- the two blocks fold
// into a single accelerator with the union of their keys (last value wins).  The
// dangerous case (the same BDF spelled two different ways) yields distinct
// section strings and IS rejected -- see DuplicateBdfAfterNormalizationRejected,
// DuplicateBdfFullThenShortRejected, DuplicateBdfCaseInsensitiveRejected.
TEST(ConfigLoad, IdenticalSectionHeadersMerge)
{
    TempConfig tc(
        "[accelerator:0000:61:00]\n"
        "net-ip = 10.0.0.1\n"
        "[accelerator:0000:61:00]\n"
        "net-mac = 02:00:00:00:00:01\n");
    struct emu_config *cfg = nullptr;
    ASSERT_EQ(emu_config_load(tc.path(), &cfg), 0);
    ASSERT_EQ(cfg->accelerators.len, 1u);
    const struct emu_accelerator *a = cfg->accelerators.d[0];
    EXPECT_STREQ(a->net.ip, "10.0.0.1");
    EXPECT_STREQ(a->net.mac, "02:00:00:00:00:01");
    cleanup_config(cfg);
}

TEST(ConfigLoad, DuplicateBdfAfterNormalizationRejected)
{
    // Short form and full form denote the same board-level BDF.
    TempConfig tc(
        "[accelerator:61:00]\n"
        "net-ip = 10.0.0.1\n"
        "[accelerator:0000:61:00]\n"
        "net-ip = 10.0.0.2\n");
    struct emu_config *cfg = nullptr;
    EXPECT_EQ(emu_config_load(tc.path(), &cfg), -1);
}

TEST(ConfigLoad, MalformedBdfInSectionRejected)
{
    // The section needs a key to be reported by stock libinih at all; the
    // malformed BDF must then be rejected.
    TempConfig tc(
        "[accelerator:not-a-bdf]\n"
        "net-ip = 10.0.0.1\n");
    struct emu_config *cfg = nullptr;
    EXPECT_EQ(emu_config_load(tc.path(), &cfg), -1);
}

TEST(ConfigLoad, FunctionSuffixInSectionRejected)
{
    TempConfig tc(
        "[accelerator:0000:61:00.0]\n"
        "net-ip = 10.0.0.1\n");
    struct emu_config *cfg = nullptr;
    EXPECT_EQ(emu_config_load(tc.path(), &cfg), -1);
}

TEST(ConfigLoad, UnknownKeyRejected)
{
    TempConfig tc(
        "[accelerator:0000:61:00]\n"
        "bogus-key = 1\n");
    struct emu_config *cfg = nullptr;
    EXPECT_EQ(emu_config_load(tc.path(), &cfg), -1);
}

TEST(ConfigLoad, UnknownSectionRejected)
{
    TempConfig tc(
        "[role:admin]\n"
        "query-devices = yes\n");
    struct emu_config *cfg = nullptr;
    EXPECT_EQ(emu_config_load(tc.path(), &cfg), -1);
}

TEST(ConfigLoad, TopLevelKeyRejected)
{
    TempConfig tc("stray = value\n");
    struct emu_config *cfg = nullptr;
    EXPECT_EQ(emu_config_load(tc.path(), &cfg), -1);
}

TEST(ConfigLoad, ShippedSampleConfigParses)
{
    // The sample config shipped with the daemon must itself be valid.
    struct emu_config *cfg = nullptr;
    ASSERT_EQ(emu_config_load(SLASH_EMU_SAMPLE_CONF, &cfg), 0);
    ASSERT_NE(cfg, nullptr);
    EXPECT_GE(cfg->accelerators.len, 1u);
    cleanup_config(cfg);
}

/* ---- Running set ----------------------------------------------------- */

TEST(RunningSet, AddContainsRemove)
{
    struct emu_running_set *set = nullptr;
    ASSERT_EQ(emu_running_set_new(&set), 0);

    EXPECT_FALSE(emu_running_set_contains(set, "0000:61:00"));
    ASSERT_EQ(emu_running_set_add(set, "0000:61:00"), 0);
    EXPECT_TRUE(emu_running_set_contains(set, "0000:61:00"));

    // Idempotent add does not duplicate.
    ASSERT_EQ(emu_running_set_add(set, "0000:61:00"), 0);
    EXPECT_TRUE(emu_running_set_contains(set, "0000:61:00"));

    emu_running_set_remove(set, "0000:61:00");
    EXPECT_FALSE(emu_running_set_contains(set, "0000:61:00"));

    // Removing an absent BDF is a no-op.
    emu_running_set_remove(set, "0000:99:00");

    cleanup_running_set(set);
}

/* ---- RESCAN reload / merge selection --------------------------------- */

TEST(SelectNew, AllSelectedWhenNothingRunning)
{
    TempConfig tc(
        "[accelerator:0000:61:00]\n"
        "net-ip = 10.0.0.1\n"
        "[accelerator:0000:62:00]\n"
        "net-ip = 10.0.0.2\n");
    struct emu_config *cfg = nullptr;
    ASSERT_EQ(emu_config_load(tc.path(), &cfg), 0);

    struct emu_running_set *running = nullptr;
    ASSERT_EQ(emu_running_set_new(&running), 0);

    struct emu_accelerator_ref_array sel = emu_accelerator_ref_array_init();
    ASSERT_EQ(emu_config_select_new(cfg, running, &sel), 0);
    EXPECT_EQ(sel.len, 2u);

    emu_accelerator_ref_array_free(&sel);
    cleanup_running_set(running);
    cleanup_config(cfg);
}

TEST(SelectNew, RunningBdfsAreSkipped)
{
    TempConfig tc(
        "[accelerator:0000:61:00]\n"
        "net-ip = 10.0.0.1\n"
        "[accelerator:0000:62:00]\n"
        "net-ip = 10.0.0.2\n"
        "[accelerator:0000:63:00]\n"
        "net-ip = 10.0.0.3\n");
    struct emu_config *cfg = nullptr;
    ASSERT_EQ(emu_config_load(tc.path(), &cfg), 0);

    struct emu_running_set *running = nullptr;
    ASSERT_EQ(emu_running_set_new(&running), 0);
    ASSERT_EQ(emu_running_set_add(running, "0000:62:00"), 0);

    struct emu_accelerator_ref_array sel = emu_accelerator_ref_array_init();
    ASSERT_EQ(emu_config_select_new(cfg, running, &sel), 0);

    // 0000:62:00 is already running -> only the other two are selected.
    ASSERT_EQ(sel.len, 2u);
    for (size_t i = 0; i < sel.len; i++) {
        EXPECT_STRNE(sel.d[i]->bdf, "0000:62:00");
    }

    emu_accelerator_ref_array_free(&sel);
    cleanup_running_set(running);
    cleanup_config(cfg);
}

TEST(SelectNew, NullRunningTreatedAsEmpty)
{
    TempConfig tc(
        "[accelerator:0000:61:00]\n"
        "net-ip = 10.0.0.1\n");
    struct emu_config *cfg = nullptr;
    ASSERT_EQ(emu_config_load(tc.path(), &cfg), 0);

    struct emu_accelerator_ref_array sel = emu_accelerator_ref_array_init();
    ASSERT_EQ(emu_config_select_new(cfg, nullptr, &sel), 0);
    EXPECT_EQ(sel.len, 1u);

    emu_accelerator_ref_array_free(&sel);
    cleanup_config(cfg);
}

TEST(SelectNew, NoneSelectedWhenAllRunning)
{
    TempConfig tc(
        "[accelerator:0000:61:00]\n"
        "net-ip = 10.0.0.1\n");
    struct emu_config *cfg = nullptr;
    ASSERT_EQ(emu_config_load(tc.path(), &cfg), 0);

    struct emu_running_set *running = nullptr;
    ASSERT_EQ(emu_running_set_new(&running), 0);
    ASSERT_EQ(emu_running_set_add(running, "0000:61:00"), 0);

    struct emu_accelerator_ref_array sel = emu_accelerator_ref_array_init();
    ASSERT_EQ(emu_config_select_new(cfg, running, &sel), 0);
    EXPECT_EQ(sel.len, 0u);

    emu_accelerator_ref_array_free(&sel);
    cleanup_running_set(running);
    cleanup_config(cfg);
}

/* ====================================================================== *
 * Adversarial / codified-audit additions (T4 review).
 *
 * Everything below was added by the adversarial reviewer to pin down the
 * documented contract.  Cases marked FAIL-DOCUMENTS-BUG in a comment are
 * expected to fail against the current implementation and record the bar.
 * ====================================================================== */

/* ---- BDF normalization: hardening edge cases ------------------------- */

// Surrounding whitespace on the raw BDF.  emu_bdf_normalize() is the standalone
// validator; it does not (and should not) silently accept embedded spaces in a
// hex field.  " 0000:61:00" has a non-hex leading char in the domain field.
TEST(BdfNormalize, LeadingWhitespaceRejected)
{
    char out[EMU_BDF_LEN];
    EXPECT_EQ(emu_bdf_normalize(" 0000:61:00", out, sizeof(out)), -1);
}

TEST(BdfNormalize, TrailingWhitespaceRejected)
{
    char out[EMU_BDF_LEN];
    // Trailing space makes the device field "00 " -> length 5 bus_dev region
    // is "00:00 " etc.; in every framing this is not 2 clean hex digits.
    EXPECT_EQ(emu_bdf_normalize("0000:61:00 ", out, sizeof(out)), -1);
    EXPECT_EQ(emu_bdf_normalize("61:00 ", out, sizeof(out)), -1);
}

// Internal whitespace inside a field must be rejected (not a hex digit).
TEST(BdfNormalize, InternalWhitespaceRejected)
{
    char out[EMU_BDF_LEN];
    EXPECT_EQ(emu_bdf_normalize("0000:6 :00", out, sizeof(out)), -1);
}

// A '+' / '-' sign is not a hex digit; isxdigit must reject it.
TEST(BdfNormalize, SignCharactersRejected)
{
    char out[EMU_BDF_LEN];
    EXPECT_EQ(emu_bdf_normalize("0000:+1:00", out, sizeof(out)), -1);
    EXPECT_EQ(emu_bdf_normalize("0000:-1:00", out, sizeof(out)), -1);
}

// "0x" prefixes are not valid hex-digit runs.
TEST(BdfNormalize, HexPrefixRejected)
{
    char out[EMU_BDF_LEN];
    EXPECT_EQ(emu_bdf_normalize("0x00:61:00", out, sizeof(out)), -1);
}

// Empty fields around colons.
TEST(BdfNormalize, EmptyFieldsRejected)
{
    char out[EMU_BDF_LEN];
    EXPECT_EQ(emu_bdf_normalize(":", out, sizeof(out)), -1);
    EXPECT_EQ(emu_bdf_normalize("::", out, sizeof(out)), -1);
    EXPECT_EQ(emu_bdf_normalize("0000::00", out, sizeof(out)), -1);   // empty bus
    EXPECT_EQ(emu_bdf_normalize("0000:61:", out, sizeof(out)), -1);   // empty dev
    EXPECT_EQ(emu_bdf_normalize(":61:00", out, sizeof(out)), -1);     // empty domain
}

// Trailing colon yields 3 colons -> rejected by the colon-count switch.
TEST(BdfNormalize, TrailingColonRejected)
{
    char out[EMU_BDF_LEN];
    EXPECT_EQ(emu_bdf_normalize("0000:61:00:", out, sizeof(out)), -1);
}

// Function suffix in short form too.
TEST(BdfNormalize, ShortFormFunctionSuffixRejected)
{
    char out[EMU_BDF_LEN];
    EXPECT_EQ(emu_bdf_normalize("61:00.0", out, sizeof(out)), -1);
}

// A bare "." or a "." not part of ".F" still trips the function-suffix guard.
TEST(BdfNormalize, AnyDotRejected)
{
    char out[EMU_BDF_LEN];
    EXPECT_EQ(emu_bdf_normalize("0000.61.00", out, sizeof(out)), -1);
}

// NULL inputs must be handled, not crash.
TEST(BdfNormalize, NullInputsRejected)
{
    char out[EMU_BDF_LEN];
    EXPECT_EQ(emu_bdf_normalize(nullptr, out, sizeof(out)), -1);
    EXPECT_EQ(emu_bdf_normalize("0000:61:00", nullptr, sizeof(out)), -1);
}

// Exactly EMU_BDF_LEN must succeed; one less than required must fail cleanly.
TEST(BdfNormalize, MinimumBufferBoundary)
{
    char out[EMU_BDF_LEN];
    EXPECT_EQ(emu_bdf_normalize("0000:61:00", out, EMU_BDF_LEN), 0);
    EXPECT_STREQ(out, "0000:61:00");
    EXPECT_EQ(emu_bdf_normalize("0000:61:00", out, EMU_BDF_LEN - 1), -1);
}

// Full-form, all-uppercase including the alpha hex digits, fully lowercased.
TEST(BdfNormalize, AllUpperLowered)
{
    char out[EMU_BDF_LEN];
    ASSERT_EQ(emu_bdf_normalize("FFFF:AB:CD", out, sizeof(out)), 0);
    EXPECT_STREQ(out, "ffff:ab:cd");
}

/* ---- Section header edge cases ---------------------------------------- */

// "[accelerator]" with no colon: not an accelerator section -> rejected once it
// carries a key (a keyless one would be silently ignored; see KeylessSection*).
TEST(ConfigLoad, AcceleratorSectionWithoutColonRejected)
{
    TempConfig tc(
        "[accelerator]\n"
        "net-ip = 10.0.0.1\n");
    struct emu_config *cfg = nullptr;
    EXPECT_EQ(emu_config_load(tc.path(), &cfg), -1);
}

// "[accelerator:]" with empty BDF -> rejected (colon[1] == '\0').
TEST(ConfigLoad, AcceleratorSectionEmptyBdfRejected)
{
    TempConfig tc(
        "[accelerator:]\n"
        "net-ip = 10.0.0.1\n");
    struct emu_config *cfg = nullptr;
    EXPECT_EQ(emu_config_load(tc.path(), &cfg), -1);
}

// A section whose name merely starts with "accelerator" but is a different
// token ("acceleratorX:...") must NOT be treated as an accelerator section.
TEST(ConfigLoad, AcceleratorPrefixSubstringRejected)
{
    TempConfig tc(
        "[acceleratorX:0000:61:00]\n"
        "net-ip = 10.0.0.1\n");
    struct emu_config *cfg = nullptr;
    EXPECT_EQ(emu_config_load(tc.path(), &cfg), -1);
}

// Uppercased keyword: section keyword is matched case-sensitively (memcmp);
// "[ACCELERATOR:...]" is an unknown section and must be rejected.
TEST(ConfigLoad, AcceleratorKeywordCaseSensitive)
{
    TempConfig tc(
        "[ACCELERATOR:0000:61:00]\n"
        "net-ip = 10.0.0.1\n");
    struct emu_config *cfg = nullptr;
    EXPECT_EQ(emu_config_load(tc.path(), &cfg), -1);
}

// A non-accelerator section name "[junk]" with a key -> rejected.
TEST(ConfigLoad, EmptySectionNameRejected)
{
    TempConfig tc(
        "[junk]\n"
        "net-ip = 10.0.0.1\n");
    struct emu_config *cfg = nullptr;
    EXPECT_EQ(emu_config_load(tc.path(), &cfg), -1);
}

/* ---- The keyless-section behavior (stock libinih) -------------------- */

// With stock libinih (INI_CALL_HANDLER_ON_NEW_SECTION OFF), a section fires no
// callback until it has a key, so a KEYLESS accelerator section is silently
// ignored -- the accelerator is NOT created and the load still succeeds.  This
// pins the documented schema rule: every [accelerator:<bdf>] must carry >=1 key.
TEST(ConfigLoad, KeylessSectionSilentlyIgnored)
{
    TempConfig tc("[accelerator:0000:61:00]\n");
    struct emu_config *cfg = nullptr;
    ASSERT_EQ(emu_config_load(tc.path(), &cfg), 0);
    ASSERT_NE(cfg, nullptr);
    EXPECT_EQ(cfg->accelerators.len, 0u);
    EXPECT_EQ(find(cfg, "0000:61:00"), nullptr);
    cleanup_config(cfg);
}

// Multiple keyless sections back-to-back: all are silently ignored.  A keyed
// section interleaved among them is the only one that materializes.
TEST(ConfigLoad, KeylessSectionsIgnoredKeyedSurvives)
{
    TempConfig tc(
        "[accelerator:0000:61:00]\n"          // keyless -> ignored
        "[accelerator:0000:62:00]\n"          // keyless -> ignored
        "net-ip = 10.0.0.2\n"                 // ...except this one has a key
        "[accelerator:0000:63:00]\n");        // keyless -> ignored
    struct emu_config *cfg = nullptr;
    ASSERT_EQ(emu_config_load(tc.path(), &cfg), 0);
    ASSERT_EQ(cfg->accelerators.len, 1u);
    EXPECT_NE(find(cfg, "0000:62:00"), nullptr);
    EXPECT_EQ(find(cfg, "0000:61:00"), nullptr);
    EXPECT_EQ(find(cfg, "0000:63:00"), nullptr);
    cleanup_config(cfg);
}

/* ---- Parser robustness ----------------------------------------------- */

// Comments and blank lines around a section are ignored.
TEST(ConfigLoad, CommentsAndBlankLinesIgnored)
{
    TempConfig tc(
        "; leading comment\n"
        "\n"
        "[accelerator:0000:61:00]\n"
        "  ; indented comment\n"
        "\n"
        "net-ip = 10.0.0.1\n"
        "\n");
    struct emu_config *cfg = nullptr;
    ASSERT_EQ(emu_config_load(tc.path(), &cfg), 0);
    ASSERT_EQ(cfg->accelerators.len, 1u);
    EXPECT_STREQ(cfg->accelerators.d[0]->net.ip, "10.0.0.1");
    cleanup_config(cfg);
}

// CRLF line endings must parse the same as LF.
TEST(ConfigLoad, CrlfLineEndings)
{
    TempConfig tc(
        "[accelerator:0000:61:00]\r\n"
        "net-ip = 10.0.0.1\r\n");
    struct emu_config *cfg = nullptr;
    ASSERT_EQ(emu_config_load(tc.path(), &cfg), 0);
    ASSERT_EQ(cfg->accelerators.len, 1u);
    // inih strips the trailing CR; the value must not carry a stray '\r'.
    ASSERT_NE(cfg->accelerators.d[0]->net.ip, nullptr);
    EXPECT_STREQ(cfg->accelerators.d[0]->net.ip, "10.0.0.1");
    cleanup_config(cfg);
}

// A repeated key within one section: last value wins (dup_into frees the prior).
// Primarily a leak guard under ASan.
TEST(ConfigLoad, RepeatedKeyLastWins)
{
    TempConfig tc(
        "[accelerator:0000:61:00]\n"
        "net-mac = aa\n"
        "net-mac = bb\n");
    struct emu_config *cfg = nullptr;
    ASSERT_EQ(emu_config_load(tc.path(), &cfg), 0);
    ASSERT_EQ(cfg->accelerators.len, 1u);
    EXPECT_STREQ(cfg->accelerators.d[0]->net.mac, "bb");
    cleanup_config(cfg);
}

// Keys must attach to their own section, not bleed across sections.  This is the
// load-bearing guard for the find-or-create-by-BDF parser: a regression that
// attached keys to the wrong (e.g. last-created) accelerator would surface here.
TEST(ConfigLoad, KeysAttachToOwningSection)
{
    TempConfig tc(
        "[accelerator:0000:61:00]\n"
        "net-ip = 10.0.0.1\n"
        "[accelerator:0000:62:00]\n"
        "net-ip = 10.0.0.2\n");
    struct emu_config *cfg = nullptr;
    ASSERT_EQ(emu_config_load(tc.path(), &cfg), 0);
    ASSERT_EQ(cfg->accelerators.len, 2u);
    const struct emu_accelerator *a = find(cfg, "0000:61:00");
    const struct emu_accelerator *b = find(cfg, "0000:62:00");
    ASSERT_NE(a, nullptr);
    ASSERT_NE(b, nullptr);
    EXPECT_STREQ(a->net.ip, "10.0.0.1");
    EXPECT_STREQ(b->net.ip, "10.0.0.2");
    cleanup_config(cfg);
}

// Stronger key-attribution guard: three sections, EACH with all three reserved
// keys set to per-accelerator-distinct values.  Every key must land on exactly
// its owning accelerator -- no bleed across the find-or-create boundary.
TEST(ConfigLoad, MultiSectionKeysDoNotBleed)
{
    TempConfig tc(
        "[accelerator:0000:61:00]\n"
        "net-mac  = 00:00:00:00:00:61\n"
        "net-ip   = 10.0.0.61\n"
        "net-port = 6100\n"
        "[accelerator:0000:62:00]\n"
        "net-mac  = 00:00:00:00:00:62\n"
        "net-ip   = 10.0.0.62\n"
        "net-port = 6200\n"
        "[accelerator:0000:63:00]\n"
        "net-mac  = 00:00:00:00:00:63\n"
        "net-ip   = 10.0.0.63\n"
        "net-port = 6300\n");
    struct emu_config *cfg = nullptr;
    ASSERT_EQ(emu_config_load(tc.path(), &cfg), 0);
    ASSERT_EQ(cfg->accelerators.len, 3u);

    const struct emu_accelerator *a = find(cfg, "0000:61:00");
    const struct emu_accelerator *b = find(cfg, "0000:62:00");
    const struct emu_accelerator *c = find(cfg, "0000:63:00");
    ASSERT_NE(a, nullptr);
    ASSERT_NE(b, nullptr);
    ASSERT_NE(c, nullptr);

    EXPECT_STREQ(a->net.mac, "00:00:00:00:00:61");
    EXPECT_STREQ(a->net.ip, "10.0.0.61");
    EXPECT_STREQ(a->net.port, "6100");
    EXPECT_STREQ(b->net.mac, "00:00:00:00:00:62");
    EXPECT_STREQ(b->net.ip, "10.0.0.62");
    EXPECT_STREQ(b->net.port, "6200");
    EXPECT_STREQ(c->net.mac, "00:00:00:00:00:63");
    EXPECT_STREQ(c->net.ip, "10.0.0.63");
    EXPECT_STREQ(c->net.port, "6300");
    cleanup_config(cfg);
}

// Non-consecutive keys for the same accelerator: a section that "reappears"
// (byte-identical header) after another section is in between.  With
// find-or-create the keys still converge on the right accelerators; this also
// proves the parser does not rely on a section's keys being delivered
// contiguously.  (libinih actually keeps them contiguous, but the model must
// not depend on it.)
TEST(ConfigLoad, ReappearingSectionAccumulatesKeys)
{
    TempConfig tc(
        "[accelerator:0000:61:00]\n"
        "net-ip = 10.0.0.61\n"
        "[accelerator:0000:62:00]\n"
        "net-ip = 10.0.0.62\n"
        "[accelerator:0000:61:00]\n"
        "net-mac = 00:00:00:00:00:61\n");
    struct emu_config *cfg = nullptr;
    ASSERT_EQ(emu_config_load(tc.path(), &cfg), 0);
    ASSERT_EQ(cfg->accelerators.len, 2u);
    const struct emu_accelerator *a = find(cfg, "0000:61:00");
    const struct emu_accelerator *b = find(cfg, "0000:62:00");
    ASSERT_NE(a, nullptr);
    ASSERT_NE(b, nullptr);
    EXPECT_STREQ(a->net.ip, "10.0.0.61");
    EXPECT_STREQ(a->net.mac, "00:00:00:00:00:61");
    EXPECT_STREQ(b->net.ip, "10.0.0.62");
    EXPECT_EQ(b->net.mac, nullptr);
    cleanup_config(cfg);
}

// All three reserved net-* keys round-trip and survive a reload (lifetime).
TEST(ConfigLoad, AllReservedKeysRoundTrip)
{
    TempConfig tc(
        "[accelerator:0000:61:00]\n"
        "net-mac = 02:00:00:00:00:01\n"
        "net-ip = 10.0.0.1\n"
        "net-port = 4791\n");
    struct emu_config *cfg = nullptr;
    ASSERT_EQ(emu_config_load(tc.path(), &cfg), 0);
    const struct emu_accelerator *a = cfg->accelerators.d[0];
    EXPECT_STREQ(a->net.mac, "02:00:00:00:00:01");
    EXPECT_STREQ(a->net.ip, "10.0.0.1");
    EXPECT_STREQ(a->net.port, "4791");
    cleanup_config(cfg);
}

// A malformed BDF in the *second* section must fail the whole load AND free the
// first (successfully parsed) accelerator -- partial-parse cleanup.  Run under
// ASan this is the error-path leak guard.
TEST(ConfigLoad, PartialParseFailureFreesEverything)
{
    TempConfig tc(
        "[accelerator:0000:61:00]\n"
        "net-mac = 02:00:00:00:00:01\n"
        "[accelerator:not-a-bdf]\n"
        "net-ip = 10.0.0.2\n");
    struct emu_config *cfg = nullptr;
    EXPECT_EQ(emu_config_load(tc.path(), &cfg), -1);
    // cfg must not be handed back on failure.
    EXPECT_EQ(cfg, nullptr);
}

// Unknown key after a valid one in the same section must fail and free.
TEST(ConfigLoad, UnknownKeyAfterValidKeyFails)
{
    TempConfig tc(
        "[accelerator:0000:61:00]\n"
        "net-mac = aa\n"
        "bogus = 1\n");
    struct emu_config *cfg = nullptr;
    EXPECT_EQ(emu_config_load(tc.path(), &cfg), -1);
    EXPECT_EQ(cfg, nullptr);
}

// On any failure, the out-param must be left untouched (nullptr).
TEST(ConfigLoad, OutParamUntouchedOnFailure)
{
    TempConfig tc(
        "[accelerator:not-a-bdf]\n"
        "net-ip = 10.0.0.1\n");
    struct emu_config *cfg = nullptr;
    EXPECT_EQ(emu_config_load(tc.path(), &cfg), -1);
    EXPECT_EQ(cfg, nullptr);
}

// A key with no '=' / ':' value separator is malformed; inih rejects it (stock
// libinih ships INI_ALLOW_NO_VALUE OFF).
TEST(ConfigLoad, KeyWithoutSeparatorRejected)
{
    TempConfig tc(
        "[accelerator:0000:61:00]\n"
        "net-mac\n");
    struct emu_config *cfg = nullptr;
    EXPECT_EQ(emu_config_load(tc.path(), &cfg), -1);
}

// A section short-form BDF colliding with a full-form one in the OTHER order
// (full first, then short) must also be rejected as duplicate.
TEST(ConfigLoad, DuplicateBdfFullThenShortRejected)
{
    TempConfig tc(
        "[accelerator:0000:61:00]\n"
        "net-ip = 10.0.0.1\n"
        "[accelerator:61:00]\n"
        "net-ip = 10.0.0.2\n");
    struct emu_config *cfg = nullptr;
    EXPECT_EQ(emu_config_load(tc.path(), &cfg), -1);
}

// Uppercase-vs-lowercase hex duplicate (same BDF after lower-casing).
TEST(ConfigLoad, DuplicateBdfCaseInsensitiveRejected)
{
    TempConfig tc(
        "[accelerator:00AB:CD:0F]\n"
        "net-ip = 10.0.0.1\n"
        "[accelerator:00ab:cd:0f]\n"
        "net-ip = 10.0.0.2\n");
    struct emu_config *cfg = nullptr;
    EXPECT_EQ(emu_config_load(tc.path(), &cfg), -1);
}

/* ---- Running set: additional coverage -------------------------------- */

// Remove from the middle (hole-fill via last element) keeps the rest intact.
TEST(RunningSet, RemoveFromMiddlePreservesOthers)
{
    struct emu_running_set *set = nullptr;
    ASSERT_EQ(emu_running_set_new(&set), 0);
    ASSERT_EQ(emu_running_set_add(set, "0000:61:00"), 0);
    ASSERT_EQ(emu_running_set_add(set, "0000:62:00"), 0);
    ASSERT_EQ(emu_running_set_add(set, "0000:63:00"), 0);

    emu_running_set_remove(set, "0000:62:00");
    EXPECT_FALSE(emu_running_set_contains(set, "0000:62:00"));
    EXPECT_TRUE(emu_running_set_contains(set, "0000:61:00"));
    EXPECT_TRUE(emu_running_set_contains(set, "0000:63:00"));

    cleanup_running_set(set);
}

// NULL set / NULL bdf must be handled by the query/remove helpers.
TEST(RunningSet, NullSafeQueries)
{
    EXPECT_FALSE(emu_running_set_contains(nullptr, "0000:61:00"));
    struct emu_running_set *set = nullptr;
    ASSERT_EQ(emu_running_set_new(&set), 0);
    EXPECT_FALSE(emu_running_set_contains(set, nullptr));
    emu_running_set_remove(set, nullptr);   // no crash
    emu_running_set_remove(nullptr, "x");    // no crash
    cleanup_running_set(set);
}

/* ---- RESCAN select_new: additional coverage -------------------------- */

// Empty config -> nothing selected, regardless of running set.
TEST(SelectNew, EmptyConfigSelectsNothing)
{
    struct emu_config *cfg = nullptr;
    ASSERT_EQ(emu_config_load(nullptr, &cfg), 0);

    struct emu_running_set *running = nullptr;
    ASSERT_EQ(emu_running_set_new(&running), 0);
    ASSERT_EQ(emu_running_set_add(running, "0000:61:00"), 0);

    struct emu_accelerator_ref_array sel = emu_accelerator_ref_array_init();
    ASSERT_EQ(emu_config_select_new(cfg, running, &sel), 0);
    EXPECT_EQ(sel.len, 0u);

    emu_accelerator_ref_array_free(&sel);
    cleanup_running_set(running);
    cleanup_config(cfg);
}

// NULL out-param must be rejected, not dereferenced.
TEST(SelectNew, NullOutRejected)
{
    struct emu_config *cfg = nullptr;
    ASSERT_EQ(emu_config_load(nullptr, &cfg), 0);
    EXPECT_EQ(emu_config_select_new(cfg, nullptr, nullptr), -1);
    cleanup_config(cfg);
}

// NULL config must be rejected.
TEST(SelectNew, NullConfigRejected)
{
    struct emu_accelerator_ref_array sel = emu_accelerator_ref_array_init();
    EXPECT_EQ(emu_config_select_new(nullptr, nullptr, &sel), -1);
    emu_accelerator_ref_array_free(&sel);
}

// Returned borrowed refs point into the config and stay valid for its lifetime;
// freeing the ref-array storage must NOT free the underlying accelerators (the
// config still owns and can still see them).
TEST(SelectNew, BorrowedRefsRemainValidAfterArrayFree)
{
    TempConfig tc(
        "[accelerator:0000:61:00]\n"
        "net-ip = 10.0.0.1\n"
        "[accelerator:0000:62:00]\n"
        "net-ip = 10.0.0.2\n");
    struct emu_config *cfg = nullptr;
    ASSERT_EQ(emu_config_load(tc.path(), &cfg), 0);

    struct emu_accelerator_ref_array sel = emu_accelerator_ref_array_init();
    ASSERT_EQ(emu_config_select_new(cfg, nullptr, &sel), 0);
    ASSERT_EQ(sel.len, 2u);
    // The borrowed refs must alias the config's owned accelerators.
    EXPECT_EQ(sel.d[0], cfg->accelerators.d[0]);
    EXPECT_EQ(sel.d[1], cfg->accelerators.d[1]);

    emu_accelerator_ref_array_free(&sel);

    // After freeing the ref-array, the config's accelerators are still usable.
    EXPECT_STREQ(cfg->accelerators.d[0]->bdf, "0000:61:00");
    EXPECT_NE(find(cfg, "0000:62:00"), nullptr);
    cleanup_config(cfg);
}

// Partial overlap: some running, some not.
TEST(SelectNew, PartialOverlap)
{
    TempConfig tc(
        "[accelerator:0000:61:00]\n"
        "net-ip = 10.0.0.1\n"
        "[accelerator:0000:62:00]\n"
        "net-ip = 10.0.0.2\n"
        "[accelerator:0000:63:00]\n"
        "net-ip = 10.0.0.3\n"
        "[accelerator:0000:64:00]\n"
        "net-ip = 10.0.0.4\n");
    struct emu_config *cfg = nullptr;
    ASSERT_EQ(emu_config_load(tc.path(), &cfg), 0);

    struct emu_running_set *running = nullptr;
    ASSERT_EQ(emu_running_set_new(&running), 0);
    ASSERT_EQ(emu_running_set_add(running, "0000:61:00"), 0);
    ASSERT_EQ(emu_running_set_add(running, "0000:63:00"), 0);

    struct emu_accelerator_ref_array sel = emu_accelerator_ref_array_init();
    ASSERT_EQ(emu_config_select_new(cfg, running, &sel), 0);
    ASSERT_EQ(sel.len, 2u);
    for (size_t i = 0; i < sel.len; i++) {
        EXPECT_STRNE(sel.d[i]->bdf, "0000:61:00");
        EXPECT_STRNE(sel.d[i]->bdf, "0000:63:00");
    }

    emu_accelerator_ref_array_free(&sel);
    cleanup_running_set(running);
    cleanup_config(cfg);
}

/* ---- find() null-safety ---------------------------------------------- */

TEST(ConfigFind, NullSafe)
{
    EXPECT_EQ(emu_config_find(nullptr, "0000:61:00"), nullptr);
    struct emu_config *cfg = nullptr;
    ASSERT_EQ(emu_config_load(nullptr, &cfg), 0);
    EXPECT_EQ(emu_config_find(cfg, nullptr), nullptr);
    cleanup_config(cfg);
}

/* ====================================================================== *
 * T4 rework DELTA review: find-or-create vs create-on-change.
 *
 * The parser was restructured from a new-section callback to a per-key
 * find-or-create keyed on the normalized BDF, with a parallel section_bdfs
 * array recording the *raw* section string that first created each accelerator.
 * The hazard with the rejected "create-on-section-change" alternative is that a
 * NON-ADJACENT repeated section header would double-create the same BDF and/or
 * split its keys.  These cases pin the correct behavior and would fail loudly
 * against a create-on-change implementation.
 * ====================================================================== */

// THE priority case, single accelerator: same byte-identical header reappears
// after a DIFFERENT section sits in between.  Correct: exactly ONE 0000:61:00
// accelerator that accumulated BOTH of its keys; the interloper is its own
// distinct accelerator.  A create-on-change parser would yield THREE
// accelerators (two of them 0000:61:00) -> ASSERT len==2 catches it.
TEST(ConfigLoad, NonAdjacentRepeatMergesNotDuplicates)
{
    TempConfig tc(
        "[accelerator:0000:61:00]\n"
        "net-ip = 10.0.0.61\n"
        "[accelerator:0000:62:00]\n"
        "net-ip = 10.0.0.62\n"
        "[accelerator:0000:61:00]\n"
        "net-port = 6100\n");
    struct emu_config *cfg = nullptr;
    ASSERT_EQ(emu_config_load(tc.path(), &cfg), 0);
    // Exactly two distinct accelerators -- NO phantom duplicate of 0000:61:00.
    ASSERT_EQ(cfg->accelerators.len, 2u);

    // Count how many array slots carry the 0000:61:00 BDF; must be exactly one.
    int count_61 = 0;
    for (size_t i = 0; i < cfg->accelerators.len; i++) {
        if (std::strcmp(cfg->accelerators.d[i]->bdf, "0000:61:00") == 0) {
            count_61++;
        }
    }
    EXPECT_EQ(count_61, 1);

    const struct emu_accelerator *a = find(cfg, "0000:61:00");
    ASSERT_NE(a, nullptr);
    // Both keys -- from the first and the reappearing header -- landed on it.
    EXPECT_STREQ(a->net.ip, "10.0.0.61");
    EXPECT_STREQ(a->net.port, "6100");
    cleanup_config(cfg);
}

// Reappearing header sandwiched by TWO other distinct sections.  Correct: three
// distinct accelerators, the repeated one carrying all of its scattered keys.
TEST(ConfigLoad, RepeatSandwichedByTwoOthers)
{
    TempConfig tc(
        "[accelerator:0000:61:00]\n"
        "net-ip = 10.0.0.61\n"
        "[accelerator:0000:62:00]\n"
        "net-ip = 10.0.0.62\n"
        "[accelerator:0000:63:00]\n"
        "net-ip = 10.0.0.63\n"
        "[accelerator:0000:61:00]\n"
        "net-mac = 00:00:00:00:00:61\n");
    struct emu_config *cfg = nullptr;
    ASSERT_EQ(emu_config_load(tc.path(), &cfg), 0);
    ASSERT_EQ(cfg->accelerators.len, 3u);
    const struct emu_accelerator *a = find(cfg, "0000:61:00");
    ASSERT_NE(a, nullptr);
    EXPECT_STREQ(a->net.ip, "10.0.0.61");
    EXPECT_STREQ(a->net.mac, "00:00:00:00:00:61");
    EXPECT_NE(find(cfg, "0000:62:00"), nullptr);
    EXPECT_NE(find(cfg, "0000:63:00"), nullptr);
    cleanup_config(cfg);
}

// NON-ADJACENT same-BDF but DIFFERENT spelling (short then full), with another
// section in between.  The raw section strings differ, so this is a genuine
// duplicate declaration of the same board-level BDF and MUST be rejected -- the
// section_bdfs raw-string compare is what distinguishes "same section
// continuing" from "BDF re-declared".
TEST(ConfigLoad, NonAdjacentDifferentSpellingDuplicateRejected)
{
    TempConfig tc(
        "[accelerator:61:00]\n"
        "net-ip = 10.0.0.61\n"
        "[accelerator:0000:62:00]\n"
        "net-ip = 10.0.0.62\n"
        "[accelerator:0000:61:00]\n"
        "net-mac = 00:00:00:00:00:61\n");
    struct emu_config *cfg = nullptr;
    EXPECT_EQ(emu_config_load(tc.path(), &cfg), -1);
    EXPECT_EQ(cfg, nullptr);
}

// Adjacent different-spelling same BDF must also be rejected (raw strings
// differ even though they are back-to-back).
TEST(ConfigLoad, AdjacentDifferentSpellingDuplicateRejected)
{
    TempConfig tc(
        "[accelerator:61:00]\n"
        "net-ip = 10.0.0.61\n"
        "[accelerator:0000:61:00]\n"
        "net-mac = 00:00:00:00:00:61\n");
    struct emu_config *cfg = nullptr;
    EXPECT_EQ(emu_config_load(tc.path(), &cfg), -1);
    EXPECT_EQ(cfg, nullptr);
}

// Case-only difference in the raw spelling of the same BDF, non-adjacent.  After
// normalization both are 00ab:cd:0f, but the raw section strings differ
// ("00AB:CD:0F" vs "00ab:cd:0f") -> duplicate, rejected.
TEST(ConfigLoad, NonAdjacentCaseDifferenceDuplicateRejected)
{
    TempConfig tc(
        "[accelerator:00AB:CD:0F]\n"
        "net-ip = 10.0.0.1\n"
        "[accelerator:0000:62:00]\n"
        "net-ip = 10.0.0.2\n"
        "[accelerator:00ab:cd:0f]\n"
        "net-mac = aa\n");
    struct emu_config *cfg = nullptr;
    EXPECT_EQ(emu_config_load(tc.path(), &cfg), -1);
    EXPECT_EQ(cfg, nullptr);
}

// A reappearing identical-header section that REDEFINES a key already set in its
// first appearance: last value wins, on the SAME accelerator (leak guard under
// ASan: the prior strdup must be freed by dup_into, not leaked across the gap).
TEST(ConfigLoad, ReappearingSectionRedefinesKeyLastWins)
{
    TempConfig tc(
        "[accelerator:0000:61:00]\n"
        "net-ip = 10.0.0.1\n"
        "[accelerator:0000:62:00]\n"
        "net-ip = 10.0.0.62\n"
        "[accelerator:0000:61:00]\n"
        "net-ip = 10.0.0.99\n");
    struct emu_config *cfg = nullptr;
    ASSERT_EQ(emu_config_load(tc.path(), &cfg), 0);
    ASSERT_EQ(cfg->accelerators.len, 2u);
    const struct emu_accelerator *a = find(cfg, "0000:61:00");
    ASSERT_NE(a, nullptr);
    EXPECT_STREQ(a->net.ip, "10.0.0.99");
    cleanup_config(cfg);
}

// Many interleaved reappearances must converge to exactly the distinct-BDF
// count, with every scattered key on its rightful accelerator.
TEST(ConfigLoad, InterleavedReappearancesConverge)
{
    TempConfig tc(
        "[accelerator:0000:61:00]\n"
        "net-mac = 61\n"
        "[accelerator:0000:62:00]\n"
        "net-mac = 62\n"
        "[accelerator:0000:61:00]\n"
        "net-ip = ip61\n"
        "[accelerator:0000:62:00]\n"
        "net-ip = ip62\n"
        "[accelerator:0000:61:00]\n"
        "net-port = p61\n");
    struct emu_config *cfg = nullptr;
    ASSERT_EQ(emu_config_load(tc.path(), &cfg), 0);
    ASSERT_EQ(cfg->accelerators.len, 2u);
    const struct emu_accelerator *a = find(cfg, "0000:61:00");
    const struct emu_accelerator *b = find(cfg, "0000:62:00");
    ASSERT_NE(a, nullptr);
    ASSERT_NE(b, nullptr);
    EXPECT_STREQ(a->net.mac, "61");
    EXPECT_STREQ(a->net.ip, "ip61");
    EXPECT_STREQ(a->net.port, "p61");
    EXPECT_STREQ(b->net.mac, "62");
    EXPECT_STREQ(b->net.ip, "ip62");
    EXPECT_EQ(b->net.port, nullptr);
    cleanup_config(cfg);
}

// Strong no-bleed guard for the find-or-create path: two distinct sections each
// with its OWN net-ip; assert neither inherits the other's value.  (Mirrors the
// lead's "KeysAttachToOwningSection must stay a strong guard" requirement.)
TEST(ConfigLoad, TwoSectionsNetIpNoBleed)
{
    TempConfig tc(
        "[accelerator:0000:61:00]\n"
        "net-ip = 10.0.0.61\n"
        "[accelerator:0000:62:00]\n"
        "net-ip = 10.0.0.62\n");
    struct emu_config *cfg = nullptr;
    ASSERT_EQ(emu_config_load(tc.path(), &cfg), 0);
    ASSERT_EQ(cfg->accelerators.len, 2u);
    const struct emu_accelerator *a = find(cfg, "0000:61:00");
    const struct emu_accelerator *b = find(cfg, "0000:62:00");
    ASSERT_NE(a, nullptr);
    ASSERT_NE(b, nullptr);
    EXPECT_STREQ(a->net.ip, "10.0.0.61");
    EXPECT_STREQ(b->net.ip, "10.0.0.62");
    EXPECT_STRNE(a->net.ip, b->net.ip);
    cleanup_config(cfg);
}

// A truly keyless section interleaved among keyed ones is invisible (stock
// libinih), and does NOT corrupt the section tracking of its neighbors.
TEST(ConfigLoad, KeylessInterleavedDoesNotCorruptNeighbors)
{
    TempConfig tc(
        "[accelerator:0000:61:00]\n"
        "net-ip = 10.0.0.61\n"
        "[accelerator:0000:62:00]\n"       // keyless -> invisible
        "[accelerator:0000:63:00]\n"
        "net-ip = 10.0.0.63\n");
    struct emu_config *cfg = nullptr;
    ASSERT_EQ(emu_config_load(tc.path(), &cfg), 0);
    ASSERT_EQ(cfg->accelerators.len, 2u);
    const struct emu_accelerator *a = find(cfg, "0000:61:00");
    const struct emu_accelerator *c = find(cfg, "0000:63:00");
    ASSERT_NE(a, nullptr);
    ASSERT_NE(c, nullptr);
    EXPECT_EQ(find(cfg, "0000:62:00"), nullptr);
    EXPECT_STREQ(a->net.ip, "10.0.0.61");
    EXPECT_STREQ(c->net.ip, "10.0.0.63");
    cleanup_config(cfg);
}
