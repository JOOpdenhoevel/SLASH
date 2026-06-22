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
 * @brief Unit tests for the slash-emu config parser and accelerator model (C++20 API).
 *
 * These tests exercise the C++20 config API (config.hpp / slash::emu::Config)
 * against the slash_emu_core static library:
 *   - Parsing valid configs (1 and N accelerators), including BDF normalization.
 *   - Reserved network-key round-trip.
 *   - Rejection of malformed, empty-file, missing-file, duplicate-BDF, unknown
 *     section/key, and function-suffix configs.
 *   - Standalone BDF normalization edge cases.
 *   - The RESCAN reload/merge selection (collision skip) via Config::selectNew.
 *
 * Configs are written to throwaway files under the repo .tmp scratch area
 * (SLASH_EMU_TMP_DIR, injected by CMake) so nothing touches /tmp and CTest
 * drives everything.
 */

#include <gtest/gtest.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include "config.hpp"
#include "utils.hpp"

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

    const std::string &path() const { return path_; }

private:
    std::string path_;
};

}  // namespace

using slash::emu::Config;
using slash::emu::Accelerator;
using slash::emu::SystemError;
using slash::emu::normalizeBdf;

/* ---- BDF normalization (standalone) ---------------------------------- */

TEST(BdfNormalize, FullFormPassesThrough)
{
    auto result = normalizeBdf("0000:61:00");
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(*result, "0000:61:00");
}

TEST(BdfNormalize, ShortFormExpandsDomain)
{
    auto result = normalizeBdf("61:00");
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(*result, "0000:61:00");
}

TEST(BdfNormalize, UpperCaseHexLowered)
{
    auto result = normalizeBdf("00AB:CD:0F");
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(*result, "00ab:cd:0f");
}

TEST(BdfNormalize, FunctionSuffixRejected)
{
    EXPECT_FALSE(normalizeBdf("0000:61:00.0").has_value());
    EXPECT_FALSE(normalizeBdf("0000:61:00.1").has_value());
}

TEST(BdfNormalize, MalformedRejected)
{
    EXPECT_FALSE(normalizeBdf("").has_value());
    EXPECT_FALSE(normalizeBdf("garbage").has_value());
    EXPECT_FALSE(normalizeBdf("0000:61").has_value());       // missing dev
    EXPECT_FALSE(normalizeBdf("0000:61:00:00").has_value()); // too many fields
    EXPECT_FALSE(normalizeBdf("zz:00").has_value());         // non-hex bus
    EXPECT_FALSE(normalizeBdf("000:61:00").has_value());     // short domain
    EXPECT_FALSE(normalizeBdf("0000:611:00").has_value());   // long bus
}

/* ---- Loading: defaults and valid configs ----------------------------- */

TEST(ConfigLoad, NullPathYieldsEmptyConfig)
{
    Config cfg = Config::load(std::nullopt);
    EXPECT_EQ(cfg.accelerators().size(), 0u);
    EXPECT_FALSE(cfg.sourcePath().has_value());
}

TEST(ConfigLoad, EmptyFileYieldsEmptyConfig)
{
    TempConfig tc("");
    Config cfg = Config::load(tc.path());
    EXPECT_EQ(cfg.accelerators().size(), 0u);
    ASSERT_TRUE(cfg.sourcePath().has_value());
    EXPECT_EQ(*cfg.sourcePath(), tc.path());
}

TEST(ConfigLoad, MissingFileFails)
{
    std::string missing = std::string(SLASH_EMU_TMP_DIR) + "/does_not_exist_zzz.conf";
    ::unlink(missing.c_str());
    EXPECT_THROW(Config::load(missing), SystemError);
}

TEST(ConfigLoad, SingleAccelerator)
{
    // Each accelerator section must carry >=1 key (stock libinih ignores keyless
    // sections); a reserved net-* key satisfies that.
    TempConfig tc(
        "[accelerator:0000:61:00]\n"
        "net-ip = 10.0.0.1\n");
    Config cfg = Config::load(tc.path());
    ASSERT_EQ(cfg.accelerators().size(), 1u);
    EXPECT_EQ(cfg.accelerators()[0].bdf, "0000:61:00");
    EXPECT_NE(cfg.find("0000:61:00"), nullptr);
    EXPECT_EQ(cfg.find("0000:62:00"), nullptr);
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
    Config cfg = Config::load(tc.path());
    ASSERT_EQ(cfg.accelerators().size(), 3u);
    EXPECT_NE(cfg.find("0000:61:00"), nullptr);
    EXPECT_NE(cfg.find("0000:62:00"), nullptr);
    EXPECT_NE(cfg.find("01ab:03:00"), nullptr);
}

TEST(ConfigLoad, ShortFormBdfNormalizedInSection)
{
    TempConfig tc(
        "[accelerator:61:00]\n"
        "net-ip = 10.0.0.1\n");
    Config cfg = Config::load(tc.path());
    ASSERT_EQ(cfg.accelerators().size(), 1u);
    EXPECT_EQ(cfg.accelerators()[0].bdf, "0000:61:00");
}

TEST(ConfigLoad, ReservedNetworkKeysRoundTrip)
{
    TempConfig tc(
        "[accelerator:0000:61:00]\n"
        "net-mac  = 02:00:00:00:00:01\n"
        "net-ip   = 10.0.0.1\n"
        "net-port = 4791\n");
    Config cfg = Config::load(tc.path());
    ASSERT_EQ(cfg.accelerators().size(), 1u);
    const Accelerator &acc = cfg.accelerators()[0];
    ASSERT_TRUE(acc.net.mac.has_value());
    ASSERT_TRUE(acc.net.ip.has_value());
    ASSERT_TRUE(acc.net.port.has_value());
    EXPECT_EQ(*acc.net.mac, "02:00:00:00:00:01");
    EXPECT_EQ(*acc.net.ip, "10.0.0.1");
    EXPECT_EQ(*acc.net.port, "4791");
}

TEST(ConfigLoad, MissingNetworkKeysStayEmpty)
{
    // Only net-mac is set (the required key); the other two reserved keys must
    // remain absent (empty optional).
    TempConfig tc(
        "[accelerator:0000:61:00]\n"
        "net-mac = 02:00:00:00:00:01\n");
    Config cfg = Config::load(tc.path());
    ASSERT_EQ(cfg.accelerators().size(), 1u);
    const Accelerator &acc = cfg.accelerators()[0];
    ASSERT_TRUE(acc.net.mac.has_value());
    EXPECT_EQ(*acc.net.mac, "02:00:00:00:00:01");
    EXPECT_FALSE(acc.net.ip.has_value());
    EXPECT_FALSE(acc.net.port.has_value());
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
    Config cfg = Config::load(tc.path());
    ASSERT_EQ(cfg.accelerators().size(), 1u);
    const Accelerator &a = cfg.accelerators()[0];
    ASSERT_TRUE(a.net.ip.has_value());
    ASSERT_TRUE(a.net.mac.has_value());
    EXPECT_EQ(*a.net.ip, "10.0.0.1");
    EXPECT_EQ(*a.net.mac, "02:00:00:00:00:01");
}

TEST(ConfigLoad, DuplicateBdfAfterNormalizationRejected)
{
    // Short form and full form denote the same board-level BDF.
    TempConfig tc(
        "[accelerator:61:00]\n"
        "net-ip = 10.0.0.1\n"
        "[accelerator:0000:61:00]\n"
        "net-ip = 10.0.0.2\n");
    EXPECT_THROW(Config::load(tc.path()), SystemError);
}

TEST(ConfigLoad, MalformedBdfInSectionRejected)
{
    // The section needs a key to be reported by stock libinih at all; the
    // malformed BDF must then be rejected.
    TempConfig tc(
        "[accelerator:not-a-bdf]\n"
        "net-ip = 10.0.0.1\n");
    EXPECT_THROW(Config::load(tc.path()), SystemError);
}

TEST(ConfigLoad, FunctionSuffixInSectionRejected)
{
    TempConfig tc(
        "[accelerator:0000:61:00.0]\n"
        "net-ip = 10.0.0.1\n");
    EXPECT_THROW(Config::load(tc.path()), SystemError);
}

TEST(ConfigLoad, UnknownKeyRejected)
{
    TempConfig tc(
        "[accelerator:0000:61:00]\n"
        "bogus-key = 1\n");
    EXPECT_THROW(Config::load(tc.path()), SystemError);
}

TEST(ConfigLoad, UnknownSectionRejected)
{
    TempConfig tc(
        "[role:admin]\n"
        "query-devices = yes\n");
    EXPECT_THROW(Config::load(tc.path()), SystemError);
}

TEST(ConfigLoad, TopLevelKeyRejected)
{
    TempConfig tc("stray = value\n");
    EXPECT_THROW(Config::load(tc.path()), SystemError);
}

TEST(ConfigLoad, ShippedSampleConfigParses)
{
    // The sample config shipped with the daemon must itself be valid.
    Config cfg = Config::load(std::string(SLASH_EMU_SAMPLE_CONF));
    EXPECT_GE(cfg.accelerators().size(), 1u);
}

/* ---- RESCAN reload / merge selection --------------------------------- */

TEST(SelectNew, AllSelectedWhenNothingRunning)
{
    TempConfig tc(
        "[accelerator:0000:61:00]\n"
        "net-ip = 10.0.0.1\n"
        "[accelerator:0000:62:00]\n"
        "net-ip = 10.0.0.2\n");
    Config cfg = Config::load(tc.path());

    std::vector<std::string> running;
    auto sel = cfg.selectNew(running);
    EXPECT_EQ(sel.size(), 2u);
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
    Config cfg = Config::load(tc.path());

    std::vector<std::string> running = {"0000:62:00"};
    auto sel = cfg.selectNew(running);

    // 0000:62:00 is already running -> only the other two are selected.
    ASSERT_EQ(sel.size(), 2u);
    for (const Accelerator *acc : sel) {
        EXPECT_NE(acc->bdf, "0000:62:00");
    }
}

TEST(SelectNew, EmptyRunningSelectsAll)
{
    TempConfig tc(
        "[accelerator:0000:61:00]\n"
        "net-ip = 10.0.0.1\n");
    Config cfg = Config::load(tc.path());

    std::vector<std::string> running;
    auto sel = cfg.selectNew(running);
    EXPECT_EQ(sel.size(), 1u);
}

TEST(SelectNew, NoneSelectedWhenAllRunning)
{
    TempConfig tc(
        "[accelerator:0000:61:00]\n"
        "net-ip = 10.0.0.1\n");
    Config cfg = Config::load(tc.path());

    std::vector<std::string> running = {"0000:61:00"};
    auto sel = cfg.selectNew(running);
    EXPECT_EQ(sel.size(), 0u);
}

/* ====================================================================== *
 * Adversarial / codified-audit additions (T4 review).
 *
 * Everything below was added by the adversarial reviewer to pin down the
 * documented contract.
 * ====================================================================== */

/* ---- BDF normalization: hardening edge cases ------------------------- */

// Surrounding whitespace on the raw BDF.  normalizeBdf() is the standalone
// validator; it does not (and should not) silently accept embedded spaces in a
// hex field.  " 0000:61:00" has a non-hex leading char in the domain field.
TEST(BdfNormalize, LeadingWhitespaceRejected)
{
    EXPECT_FALSE(normalizeBdf(" 0000:61:00").has_value());
}

TEST(BdfNormalize, TrailingWhitespaceRejected)
{
    // Trailing space makes the device field "00 " -> length 5 bus_dev region
    // is "00:00 " etc.; in every framing this is not 2 clean hex digits.
    EXPECT_FALSE(normalizeBdf("0000:61:00 ").has_value());
    EXPECT_FALSE(normalizeBdf("61:00 ").has_value());
}

// Internal whitespace inside a field must be rejected (not a hex digit).
TEST(BdfNormalize, InternalWhitespaceRejected)
{
    EXPECT_FALSE(normalizeBdf("0000:6 :00").has_value());
}

// A '+' / '-' sign is not a hex digit; isxdigit must reject it.
TEST(BdfNormalize, SignCharactersRejected)
{
    EXPECT_FALSE(normalizeBdf("0000:+1:00").has_value());
    EXPECT_FALSE(normalizeBdf("0000:-1:00").has_value());
}

// "0x" prefixes are not valid hex-digit runs.
TEST(BdfNormalize, HexPrefixRejected)
{
    EXPECT_FALSE(normalizeBdf("0x00:61:00").has_value());
}

// Empty fields around colons.
TEST(BdfNormalize, EmptyFieldsRejected)
{
    EXPECT_FALSE(normalizeBdf(":").has_value());
    EXPECT_FALSE(normalizeBdf("::").has_value());
    EXPECT_FALSE(normalizeBdf("0000::00").has_value());   // empty bus
    EXPECT_FALSE(normalizeBdf("0000:61:").has_value());   // empty dev
    EXPECT_FALSE(normalizeBdf(":61:00").has_value());     // empty domain
}

// Trailing colon yields 3 colons -> rejected by the colon-count switch.
TEST(BdfNormalize, TrailingColonRejected)
{
    EXPECT_FALSE(normalizeBdf("0000:61:00:").has_value());
}

// Function suffix in short form too.
TEST(BdfNormalize, ShortFormFunctionSuffixRejected)
{
    EXPECT_FALSE(normalizeBdf("61:00.0").has_value());
}

// A bare "." or a "." not part of ".F" still trips the function-suffix guard.
TEST(BdfNormalize, AnyDotRejected)
{
    EXPECT_FALSE(normalizeBdf("0000.61.00").has_value());
}

// Exactly kBdfLen must succeed; one less than required must fail cleanly.
// (The C++ API doesn't expose buffer sizing directly; we just test round-trips.)
TEST(BdfNormalize, MinimumBufferBoundary)
{
    auto result = normalizeBdf("0000:61:00");
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(*result, "0000:61:00");
    // A 10-char BDF is valid; normalizeBdf has no external buffer to size.
}

// Full-form, all-uppercase including the alpha hex digits, fully lowercased.
TEST(BdfNormalize, AllUpperLowered)
{
    auto result = normalizeBdf("FFFF:AB:CD");
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(*result, "ffff:ab:cd");
}

/* ---- Section header edge cases ---------------------------------------- */

// "[accelerator]" with no colon: not an accelerator section -> rejected once it
// carries a key (a keyless one would be silently ignored; see KeylessSection*).
TEST(ConfigLoad, AcceleratorSectionWithoutColonRejected)
{
    TempConfig tc(
        "[accelerator]\n"
        "net-ip = 10.0.0.1\n");
    EXPECT_THROW(Config::load(tc.path()), SystemError);
}

// "[accelerator:]" with empty BDF -> rejected (colon[1] == '\0').
TEST(ConfigLoad, AcceleratorSectionEmptyBdfRejected)
{
    TempConfig tc(
        "[accelerator:]\n"
        "net-ip = 10.0.0.1\n");
    EXPECT_THROW(Config::load(tc.path()), SystemError);
}

// A section whose name merely starts with "accelerator" but is a different
// token ("acceleratorX:...") must NOT be treated as an accelerator section.
TEST(ConfigLoad, AcceleratorPrefixSubstringRejected)
{
    TempConfig tc(
        "[acceleratorX:0000:61:00]\n"
        "net-ip = 10.0.0.1\n");
    EXPECT_THROW(Config::load(tc.path()), SystemError);
}

// Uppercased keyword: section keyword is matched case-sensitively;
// "[ACCELERATOR:...]" is an unknown section and must be rejected.
TEST(ConfigLoad, AcceleratorKeywordCaseSensitive)
{
    TempConfig tc(
        "[ACCELERATOR:0000:61:00]\n"
        "net-ip = 10.0.0.1\n");
    EXPECT_THROW(Config::load(tc.path()), SystemError);
}

// A non-accelerator section name "[junk]" with a key -> rejected.
TEST(ConfigLoad, EmptySectionNameRejected)
{
    TempConfig tc(
        "[junk]\n"
        "net-ip = 10.0.0.1\n");
    EXPECT_THROW(Config::load(tc.path()), SystemError);
}

/* ---- The keyless-section behavior (stock libinih) -------------------- */

// With stock libinih (INI_CALL_HANDLER_ON_NEW_SECTION OFF), a section fires no
// callback until it has a key, so a KEYLESS accelerator section is silently
// ignored -- the accelerator is NOT created and the load still succeeds.
TEST(ConfigLoad, KeylessSectionSilentlyIgnored)
{
    TempConfig tc("[accelerator:0000:61:00]\n");
    Config cfg = Config::load(tc.path());
    EXPECT_EQ(cfg.accelerators().size(), 0u);
    EXPECT_EQ(cfg.find("0000:61:00"), nullptr);
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
    Config cfg = Config::load(tc.path());
    ASSERT_EQ(cfg.accelerators().size(), 1u);
    EXPECT_NE(cfg.find("0000:62:00"), nullptr);
    EXPECT_EQ(cfg.find("0000:61:00"), nullptr);
    EXPECT_EQ(cfg.find("0000:63:00"), nullptr);
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
    Config cfg = Config::load(tc.path());
    ASSERT_EQ(cfg.accelerators().size(), 1u);
    ASSERT_TRUE(cfg.accelerators()[0].net.ip.has_value());
    EXPECT_EQ(*cfg.accelerators()[0].net.ip, "10.0.0.1");
}

// CRLF line endings must parse the same as LF.
TEST(ConfigLoad, CrlfLineEndings)
{
    TempConfig tc(
        "[accelerator:0000:61:00]\r\n"
        "net-ip = 10.0.0.1\r\n");
    Config cfg = Config::load(tc.path());
    ASSERT_EQ(cfg.accelerators().size(), 1u);
    // inih strips the trailing CR; the value must not carry a stray '\r'.
    ASSERT_TRUE(cfg.accelerators()[0].net.ip.has_value());
    EXPECT_EQ(*cfg.accelerators()[0].net.ip, "10.0.0.1");
}

// A repeated key within one section: last value wins.
// Primarily a leak guard under ASan.
TEST(ConfigLoad, RepeatedKeyLastWins)
{
    TempConfig tc(
        "[accelerator:0000:61:00]\n"
        "net-mac = aa\n"
        "net-mac = bb\n");
    Config cfg = Config::load(tc.path());
    ASSERT_EQ(cfg.accelerators().size(), 1u);
    ASSERT_TRUE(cfg.accelerators()[0].net.mac.has_value());
    EXPECT_EQ(*cfg.accelerators()[0].net.mac, "bb");
}

// Keys must attach to their own section, not bleed across sections.
TEST(ConfigLoad, KeysAttachToOwningSection)
{
    TempConfig tc(
        "[accelerator:0000:61:00]\n"
        "net-ip = 10.0.0.1\n"
        "[accelerator:0000:62:00]\n"
        "net-ip = 10.0.0.2\n");
    Config cfg = Config::load(tc.path());
    ASSERT_EQ(cfg.accelerators().size(), 2u);
    const Accelerator *a = cfg.find("0000:61:00");
    const Accelerator *b = cfg.find("0000:62:00");
    ASSERT_NE(a, nullptr);
    ASSERT_NE(b, nullptr);
    ASSERT_TRUE(a->net.ip.has_value());
    ASSERT_TRUE(b->net.ip.has_value());
    EXPECT_EQ(*a->net.ip, "10.0.0.1");
    EXPECT_EQ(*b->net.ip, "10.0.0.2");
}

// Stronger key-attribution guard: three sections, EACH with all three reserved
// keys set to per-accelerator-distinct values.
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
    Config cfg = Config::load(tc.path());
    ASSERT_EQ(cfg.accelerators().size(), 3u);

    const Accelerator *a = cfg.find("0000:61:00");
    const Accelerator *b = cfg.find("0000:62:00");
    const Accelerator *c = cfg.find("0000:63:00");
    ASSERT_NE(a, nullptr);
    ASSERT_NE(b, nullptr);
    ASSERT_NE(c, nullptr);

    ASSERT_TRUE(a->net.mac.has_value());
    ASSERT_TRUE(a->net.ip.has_value());
    ASSERT_TRUE(a->net.port.has_value());
    EXPECT_EQ(*a->net.mac, "00:00:00:00:00:61");
    EXPECT_EQ(*a->net.ip, "10.0.0.61");
    EXPECT_EQ(*a->net.port, "6100");

    ASSERT_TRUE(b->net.mac.has_value());
    ASSERT_TRUE(b->net.ip.has_value());
    ASSERT_TRUE(b->net.port.has_value());
    EXPECT_EQ(*b->net.mac, "00:00:00:00:00:62");
    EXPECT_EQ(*b->net.ip, "10.0.0.62");
    EXPECT_EQ(*b->net.port, "6200");

    ASSERT_TRUE(c->net.mac.has_value());
    ASSERT_TRUE(c->net.ip.has_value());
    ASSERT_TRUE(c->net.port.has_value());
    EXPECT_EQ(*c->net.mac, "00:00:00:00:00:63");
    EXPECT_EQ(*c->net.ip, "10.0.0.63");
    EXPECT_EQ(*c->net.port, "6300");
}

// Non-consecutive keys for the same accelerator: a section that "reappears"
// (byte-identical header) after another section is in between.
TEST(ConfigLoad, ReappearingSectionAccumulatesKeys)
{
    TempConfig tc(
        "[accelerator:0000:61:00]\n"
        "net-ip = 10.0.0.61\n"
        "[accelerator:0000:62:00]\n"
        "net-ip = 10.0.0.62\n"
        "[accelerator:0000:61:00]\n"
        "net-mac = 00:00:00:00:00:61\n");
    Config cfg = Config::load(tc.path());
    ASSERT_EQ(cfg.accelerators().size(), 2u);
    const Accelerator *a = cfg.find("0000:61:00");
    const Accelerator *b = cfg.find("0000:62:00");
    ASSERT_NE(a, nullptr);
    ASSERT_NE(b, nullptr);
    ASSERT_TRUE(a->net.ip.has_value());
    ASSERT_TRUE(a->net.mac.has_value());
    EXPECT_EQ(*a->net.ip, "10.0.0.61");
    EXPECT_EQ(*a->net.mac, "00:00:00:00:00:61");
    ASSERT_TRUE(b->net.ip.has_value());
    EXPECT_EQ(*b->net.ip, "10.0.0.62");
    EXPECT_FALSE(b->net.mac.has_value());
}

// All three reserved net-* keys round-trip and survive a reload (lifetime).
TEST(ConfigLoad, AllReservedKeysRoundTrip)
{
    TempConfig tc(
        "[accelerator:0000:61:00]\n"
        "net-mac = 02:00:00:00:00:01\n"
        "net-ip = 10.0.0.1\n"
        "net-port = 4791\n");
    Config cfg = Config::load(tc.path());
    const Accelerator &a = cfg.accelerators()[0];
    ASSERT_TRUE(a.net.mac.has_value());
    ASSERT_TRUE(a.net.ip.has_value());
    ASSERT_TRUE(a.net.port.has_value());
    EXPECT_EQ(*a.net.mac, "02:00:00:00:00:01");
    EXPECT_EQ(*a.net.ip, "10.0.0.1");
    EXPECT_EQ(*a.net.port, "4791");
}

// A malformed BDF in the *second* section must fail the whole load.
TEST(ConfigLoad, PartialParseFailureThrows)
{
    TempConfig tc(
        "[accelerator:0000:61:00]\n"
        "net-mac = 02:00:00:00:00:01\n"
        "[accelerator:not-a-bdf]\n"
        "net-ip = 10.0.0.2\n");
    EXPECT_THROW(Config::load(tc.path()), SystemError);
}

// Unknown key after a valid one in the same section must fail.
TEST(ConfigLoad, UnknownKeyAfterValidKeyFails)
{
    TempConfig tc(
        "[accelerator:0000:61:00]\n"
        "net-mac = aa\n"
        "bogus = 1\n");
    EXPECT_THROW(Config::load(tc.path()), SystemError);
}

// A key with no '=' / ':' value separator is malformed; inih rejects it (stock
// libinih ships INI_ALLOW_NO_VALUE OFF).
TEST(ConfigLoad, KeyWithoutSeparatorRejected)
{
    TempConfig tc(
        "[accelerator:0000:61:00]\n"
        "net-mac\n");
    EXPECT_THROW(Config::load(tc.path()), SystemError);
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
    EXPECT_THROW(Config::load(tc.path()), SystemError);
}

// Uppercase-vs-lowercase hex duplicate (same BDF after lower-casing).
TEST(ConfigLoad, DuplicateBdfCaseInsensitiveRejected)
{
    TempConfig tc(
        "[accelerator:00AB:CD:0F]\n"
        "net-ip = 10.0.0.1\n"
        "[accelerator:00ab:cd:0f]\n"
        "net-ip = 10.0.0.2\n");
    EXPECT_THROW(Config::load(tc.path()), SystemError);
}

/* ---- RESCAN select_new: additional coverage -------------------------- */

// Empty config -> nothing selected, regardless of running set.
TEST(SelectNew, EmptyConfigSelectsNothing)
{
    Config cfg = Config::load(std::nullopt);

    std::vector<std::string> running = {"0000:61:00"};
    auto sel = cfg.selectNew(running);
    EXPECT_EQ(sel.size(), 0u);
}

// Returned borrowed refs point into the config and stay valid for its lifetime;
// the config still owns the accelerators.
TEST(SelectNew, BorrowedRefsRemainValid)
{
    TempConfig tc(
        "[accelerator:0000:61:00]\n"
        "net-ip = 10.0.0.1\n"
        "[accelerator:0000:62:00]\n"
        "net-ip = 10.0.0.2\n");
    Config cfg = Config::load(tc.path());

    std::vector<std::string> running;
    auto sel = cfg.selectNew(running);
    ASSERT_EQ(sel.size(), 2u);
    // The borrowed refs must alias the config's owned accelerators.
    EXPECT_EQ(sel[0], &cfg.accelerators()[0]);
    EXPECT_EQ(sel[1], &cfg.accelerators()[1]);

    // After using the ref-array, the config's accelerators are still usable.
    EXPECT_EQ(cfg.accelerators()[0].bdf, "0000:61:00");
    EXPECT_NE(cfg.find("0000:62:00"), nullptr);
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
    Config cfg = Config::load(tc.path());

    std::vector<std::string> running = {"0000:61:00", "0000:63:00"};
    auto sel = cfg.selectNew(running);
    ASSERT_EQ(sel.size(), 2u);
    for (const Accelerator *acc : sel) {
        EXPECT_NE(acc->bdf, "0000:61:00");
        EXPECT_NE(acc->bdf, "0000:63:00");
    }
}

/* ---- find() null-safety ---------------------------------------------- */

TEST(ConfigFind, FindMissing)
{
    Config cfg = Config::load(std::nullopt);
    EXPECT_EQ(cfg.find("0000:61:00"), nullptr);
}

TEST(ConfigFind, FindPresent)
{
    TempConfig tc(
        "[accelerator:0000:61:00]\n"
        "net-ip = 10.0.0.1\n");
    Config cfg = Config::load(tc.path());
    const Accelerator *acc = cfg.find("0000:61:00");
    ASSERT_NE(acc, nullptr);
    EXPECT_EQ(acc->bdf, "0000:61:00");
}

/* ====================================================================== *
 * T4 rework DELTA review: find-or-create vs create-on-change.
 *
 * The parser was restructured from a new-section callback to a per-key
 * find-or-create keyed on the normalized BDF, with a parallel sectionBdfs
 * vector recording the *raw* section string that first created each accelerator.
 * The hazard with the rejected "create-on-section-change" alternative is that a
 * NON-ADJACENT repeated section header would double-create the same BDF and/or
 * split its keys.  These cases pin the correct behavior.
 * ====================================================================== */

// THE priority case, single accelerator: same byte-identical header reappears
// after a DIFFERENT section sits in between.  Correct: exactly ONE 0000:61:00
// accelerator that accumulated BOTH of its keys; the interloper is its own
// distinct accelerator.  A create-on-change parser would yield THREE
// accelerators (two of them 0000:61:00) -> ASSERT size==2 catches it.
TEST(ConfigLoad, NonAdjacentRepeatMergesNotDuplicates)
{
    TempConfig tc(
        "[accelerator:0000:61:00]\n"
        "net-ip = 10.0.0.61\n"
        "[accelerator:0000:62:00]\n"
        "net-ip = 10.0.0.62\n"
        "[accelerator:0000:61:00]\n"
        "net-port = 6100\n");
    Config cfg = Config::load(tc.path());
    // Exactly two distinct accelerators -- NO phantom duplicate of 0000:61:00.
    ASSERT_EQ(cfg.accelerators().size(), 2u);

    // Count how many array slots carry the 0000:61:00 BDF; must be exactly one.
    int count_61 = 0;
    for (const Accelerator &acc : cfg.accelerators()) {
        if (acc.bdf == "0000:61:00") {
            count_61++;
        }
    }
    EXPECT_EQ(count_61, 1);

    const Accelerator *a = cfg.find("0000:61:00");
    ASSERT_NE(a, nullptr);
    // Both keys -- from the first and the reappearing header -- landed on it.
    ASSERT_TRUE(a->net.ip.has_value());
    ASSERT_TRUE(a->net.port.has_value());
    EXPECT_EQ(*a->net.ip, "10.0.0.61");
    EXPECT_EQ(*a->net.port, "6100");
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
    Config cfg = Config::load(tc.path());
    ASSERT_EQ(cfg.accelerators().size(), 3u);
    const Accelerator *a = cfg.find("0000:61:00");
    ASSERT_NE(a, nullptr);
    ASSERT_TRUE(a->net.ip.has_value());
    ASSERT_TRUE(a->net.mac.has_value());
    EXPECT_EQ(*a->net.ip, "10.0.0.61");
    EXPECT_EQ(*a->net.mac, "00:00:00:00:00:61");
    EXPECT_NE(cfg.find("0000:62:00"), nullptr);
    EXPECT_NE(cfg.find("0000:63:00"), nullptr);
}

// NON-ADJACENT same-BDF but DIFFERENT spelling (short then full), with another
// section in between.  The raw section strings differ, so this is a genuine
// duplicate declaration of the same board-level BDF and MUST be rejected.
TEST(ConfigLoad, NonAdjacentDifferentSpellingDuplicateRejected)
{
    TempConfig tc(
        "[accelerator:61:00]\n"
        "net-ip = 10.0.0.61\n"
        "[accelerator:0000:62:00]\n"
        "net-ip = 10.0.0.62\n"
        "[accelerator:0000:61:00]\n"
        "net-mac = 00:00:00:00:00:61\n");
    EXPECT_THROW(Config::load(tc.path()), SystemError);
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
    EXPECT_THROW(Config::load(tc.path()), SystemError);
}

// Case-only difference in the raw spelling of the same BDF, non-adjacent.
TEST(ConfigLoad, NonAdjacentCaseDifferenceDuplicateRejected)
{
    TempConfig tc(
        "[accelerator:00AB:CD:0F]\n"
        "net-ip = 10.0.0.1\n"
        "[accelerator:0000:62:00]\n"
        "net-ip = 10.0.0.2\n"
        "[accelerator:00ab:cd:0f]\n"
        "net-mac = aa\n");
    EXPECT_THROW(Config::load(tc.path()), SystemError);
}

// A reappearing identical-header section that REDEFINES a key already set in its
// first appearance: last value wins, on the SAME accelerator.
TEST(ConfigLoad, ReappearingSectionRedefinesKeyLastWins)
{
    TempConfig tc(
        "[accelerator:0000:61:00]\n"
        "net-ip = 10.0.0.1\n"
        "[accelerator:0000:62:00]\n"
        "net-ip = 10.0.0.62\n"
        "[accelerator:0000:61:00]\n"
        "net-ip = 10.0.0.99\n");
    Config cfg = Config::load(tc.path());
    ASSERT_EQ(cfg.accelerators().size(), 2u);
    const Accelerator *a = cfg.find("0000:61:00");
    ASSERT_NE(a, nullptr);
    ASSERT_TRUE(a->net.ip.has_value());
    EXPECT_EQ(*a->net.ip, "10.0.0.99");
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
    Config cfg = Config::load(tc.path());
    ASSERT_EQ(cfg.accelerators().size(), 2u);
    const Accelerator *a = cfg.find("0000:61:00");
    const Accelerator *b = cfg.find("0000:62:00");
    ASSERT_NE(a, nullptr);
    ASSERT_NE(b, nullptr);
    ASSERT_TRUE(a->net.mac.has_value());
    ASSERT_TRUE(a->net.ip.has_value());
    ASSERT_TRUE(a->net.port.has_value());
    EXPECT_EQ(*a->net.mac, "61");
    EXPECT_EQ(*a->net.ip, "ip61");
    EXPECT_EQ(*a->net.port, "p61");
    ASSERT_TRUE(b->net.mac.has_value());
    ASSERT_TRUE(b->net.ip.has_value());
    EXPECT_EQ(*b->net.mac, "62");
    EXPECT_EQ(*b->net.ip, "ip62");
    EXPECT_FALSE(b->net.port.has_value());
}

// Strong no-bleed guard for the find-or-create path: two distinct sections each
// with its OWN net-ip; assert neither inherits the other's value.
TEST(ConfigLoad, TwoSectionsNetIpNoBleed)
{
    TempConfig tc(
        "[accelerator:0000:61:00]\n"
        "net-ip = 10.0.0.61\n"
        "[accelerator:0000:62:00]\n"
        "net-ip = 10.0.0.62\n");
    Config cfg = Config::load(tc.path());
    ASSERT_EQ(cfg.accelerators().size(), 2u);
    const Accelerator *a = cfg.find("0000:61:00");
    const Accelerator *b = cfg.find("0000:62:00");
    ASSERT_NE(a, nullptr);
    ASSERT_NE(b, nullptr);
    ASSERT_TRUE(a->net.ip.has_value());
    ASSERT_TRUE(b->net.ip.has_value());
    EXPECT_EQ(*a->net.ip, "10.0.0.61");
    EXPECT_EQ(*b->net.ip, "10.0.0.62");
    EXPECT_NE(*a->net.ip, *b->net.ip);
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
    Config cfg = Config::load(tc.path());
    ASSERT_EQ(cfg.accelerators().size(), 2u);
    const Accelerator *a = cfg.find("0000:61:00");
    const Accelerator *c = cfg.find("0000:63:00");
    ASSERT_NE(a, nullptr);
    ASSERT_NE(c, nullptr);
    EXPECT_EQ(cfg.find("0000:62:00"), nullptr);
    ASSERT_TRUE(a->net.ip.has_value());
    ASSERT_TRUE(c->net.ip.has_value());
    EXPECT_EQ(*a->net.ip, "10.0.0.61");
    EXPECT_EQ(*c->net.ip, "10.0.0.63");
}
