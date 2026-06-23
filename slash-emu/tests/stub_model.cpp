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
 * @file stub_model.cpp
 * @brief CI stand-in for vpp_sim: the address-keyed SIM dialect over ipc://.
 *
 * A real synthesized vpp_sim cannot run in CI, so this stub is the single source
 * of truth the daemon's SIM bridge is tested against.  It implements the same
 * wire protocol as sim.cpp (docs/bridge-protocol.md §2) -- byte-exact replies --
 * over a ZMQ_REP socket bound to the endpoint passed as argv[1] (the same CLI
 * contract the real model honours: `./vpp_sim <ZMQ URL>`).
 *
 * State: a sparse register map (addr -> u32) and a sparse byte memory map, both
 * answering reads of never-written locations as zero -- so an end-to-end
 * round-trip through the daemon is checkable (write a register / buffer, read it
 * back and get the same bytes).
 *
 * Behaviour is tweakable by env vars so the no-hang / failure paths are testable:
 *   - SLASH_EMU_STUB_NO_BIND=1   : do not bind (model that never comes up).
 *   - SLASH_EMU_STUB_HANG=1      : bind, accept, but never reply (wedge probe).
 *   - SLASH_EMU_STUB_READY_FILE  : touch this path right after bind (spawn proof).
 *   - SLASH_EMU_STUB_CRASH_AFTER_START=1 : answer the start handshake "OK", then
 *                                  SIGABRT on the NEXT request (model crashes
 *                                  mid-session after connect).
 *   - SLASH_EMU_STUB_EXIT_AFTER_START=1  : answer start "OK", then _exit(0) on the
 *                                  next request (model dies between req and reply
 *                                  -> ZMQ desync probe; the client must latch dead
 *                                  rather than reuse a half-consumed REQ socket).
 *   - SLASH_EMU_STUB_GARBAGE=1   : reply with protocol garbage to scalar/buffer
 *                                  fetches (oversized junk, non-numeric, short
 *                                  array) so the client's parsers are fed hostile
 *                                  input -- must be a clean -EPROTO, never UB.
 *   - SLASH_EMU_STUB_IGNORE_EXIT=1 : never honour `exit` (stays alive after the
 *                                  graceful teardown verb) so the bridge's
 *                                  SIGTERM->SIGKILL reap path is exercised.
 *
 * C++20 + cppzmq C++ API + JsonCpp; behaviour byte-exact with the original
 * stub_model.c.
 */

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <unistd.h>
#include <unordered_map>

#include <json/json.h>
#include <zmq.hpp>

/* ---- sparse register + memory maps --------------------------------------- */

/** Sparse register map: addr -> u32 (reads of unknown addresses return 0). */
static std::unordered_map<uint64_t, uint32_t> g_regs;

/** Sparse byte memory map: addr -> u8 (reads of unknown addresses return 0). */
static std::unordered_map<uint64_t, uint8_t> g_mem;

static void regSet(uint64_t addr, uint32_t val) { g_regs[addr] = val; }

static uint32_t regGet(uint64_t addr)
{
    auto it = g_regs.find(addr);
    return (it != g_regs.end()) ? it->second : 0u;
}

static void memSet(uint64_t addr, uint8_t val) { g_mem[addr] = val; }

static uint8_t memGet(uint64_t addr)
{
    auto it = g_mem.find(addr);
    return (it != g_mem.end()) ? it->second : 0u;
}

/* ---- helpers ------------------------------------------------------------- */

/** Send the literal two-byte "OK" reply (no NUL, no trailing newline). */
static void sendOk(zmq::socket_t &sock)
{
    sock.send(zmq::const_buffer("OK", 2), zmq::send_flags::none);
}

/**
 * Parse a JsonCpp Value from a raw frame string.
 * Returns true on success, false if the JSON is malformed.
 */
static bool parseJson(const std::string &frame, Json::Value &out)
{
    Json::CharReaderBuilder rb;
    std::string errs;
    std::istringstream ss(frame);
    return Json::parseFromStream(rb, ss, &out, &errs);
}

/* ---- main ---------------------------------------------------------------- */

int main(int argc, char **argv)
{
    if (argc != 2 || argv[1][0] == '\0') {
        std::fprintf(stderr, "Usage: %s <ZMQ URL>\n",
                     argc > 0 ? argv[0] : "stub_model");
        return 2;
    }
    const char *endpoint = argv[1];

    if (std::getenv("SLASH_EMU_STUB_NO_BIND") != nullptr) {
        /* Model that never comes up: sleep so the daemon's handshake times out,
         * then exit.  The daemon must not wedge. */
        ::sleep(30);
        return 0;
    }

    zmq::context_t ctx(1);
    zmq::socket_t sock(ctx, zmq::socket_type::rep);

    try {
        sock.bind(endpoint);
    } catch (const zmq::error_t &e) {
        std::fprintf(stderr, "stub_model: bind(%s) failed: %s\n", endpoint,
                     e.what());
        return 3;
    }

    /* Touch the ready-file if requested (spawn proof for tests). */
    const char *ready = std::getenv("SLASH_EMU_STUB_READY_FILE");
    if (ready != nullptr && ready[0] != '\0') {
        std::ofstream rf(ready);
        if (rf) {
            rf << "ready\n";
        }
    }

    bool hang = (std::getenv("SLASH_EMU_STUB_HANG") != nullptr);
    bool crash_after_start = (std::getenv("SLASH_EMU_STUB_CRASH_AFTER_START") != nullptr);
    bool exit_after_start = (std::getenv("SLASH_EMU_STUB_EXIT_AFTER_START") != nullptr);
    bool garbage = (std::getenv("SLASH_EMU_STUB_GARBAGE") != nullptr);
    bool ignore_exit = (std::getenv("SLASH_EMU_STUB_IGNORE_EXIT") != nullptr);
    bool seen_start = false;

    bool running = true;
    while (running) {
        zmq::message_t msg;
        try {
            auto res = sock.recv(msg, zmq::recv_flags::none);
            if (!res) {
                break;
            }
        } catch (const zmq::error_t &) {
            break;
        }

        std::string frame(static_cast<const char *>(msg.data()), msg.size());

        /* Check whether a binary frame 1 follows (populate payload). */
        bool more = sock.get(zmq::sockopt::rcvmore) != 0;

        /* Hostile lifecycle modes that fire once the start handshake is done: a
         * crash (SIGABRT) or a clean exit BEFORE answering the current request,
         * modelling a model that dies mid-exchange.  The daemon's REQ client must
         * not wedge: the in-flight recv times out and latches the client dead. */
        if (seen_start && (crash_after_start || exit_after_start)) {
            /* Drain a possible frame-1 payload so we do not trip ZMQ asserts. */
            if (more) {
                zmq::message_t payload;
                try {
                    (void) sock.recv(payload, zmq::recv_flags::none);
                } catch (...) {
                }
            }
            if (crash_after_start) {
                /* Model crashes after connect.  Use abort() (SIGABRT) rather than
                 * a NULL deref so the crash is a genuine fatal signal even under
                 * ASan/UBSan (which would otherwise intercept a SEGV and exit
                 * with a sanitizer status instead of dying by signal). */
                ::abort();
            }
            ::_exit(0); /* exit_after_start: die without replying */
        }

        /* Parse the JSON command from frame 0. */
        Json::Value req;
        bool valid = parseJson(frame, req);
        std::string command = (valid && req.isMember("command"))
                                  ? req["command"].asString()
                                  : "";

        if (command == "populate") {
            uint64_t addr = valid ? req.get("addr", Json::Value(0u)).asUInt64() : 0u;
            uint64_t size = valid ? req.get("size", Json::Value(0u)).asUInt64() : 0u;

            /* Drain the binary payload frame (frame 1). */
            zmq::message_t payload;
            if (more) {
                try {
                    (void) sock.recv(payload, zmq::recv_flags::none);
                    const auto *pb = static_cast<const uint8_t *>(payload.data());
                    size_t pl = payload.size();
                    for (uint64_t i = 0; i < size && i < pl; i++) {
                        memSet(addr + i, pb[i]);
                    }
                } catch (...) {
                }
            }

            if (!hang) {
                sendOk(sock);
            }
            continue;
        }

        /* No further frames expected for other commands (more is informational). */

        if (hang) {
            /* Never reply: probe that the daemon's recv timeout fires. */
            continue;
        }

        if (command == "start") {
            seen_start = true;
            sendOk(sock);
        } else if (command == "exit") {
            if (ignore_exit) {
                /* Do not honour exit: stay alive (still answer so no desync) so
                 * the bridge must fall back to SIGTERM/SIGKILL to reap us. */
                sendOk(sock);
            } else {
                sendOk(sock);
                running = false;
            }
        } else if (command == "reg") {
            uint64_t addr = valid ? req.get("addr", Json::Value(0u)).asUInt64() : 0u;
            uint32_t val = valid ? req.get("val", Json::Value(0u)).asUInt() : 0u;
            regSet(addr, val);
            sendOk(sock);
        } else if (command == "fetch") {
            if (garbage) {
                /* Protocol garbage: a long non-numeric, non-array blob.  The
                 * client's parsers must reject it cleanly (-EPROTO) and latch
                 * dead, never read out of bounds or misparse into UB. */
                std::string junk(300, 'Z');
                sock.send(zmq::const_buffer(junk.data(), junk.size()),
                          zmq::send_flags::none);
            } else {
                std::string type = (valid && req.isMember("type"))
                                       ? req["type"].asString()
                                       : "";
                uint64_t addr = valid ? req.get("addr", Json::Value(0u)).asUInt64() : 0u;

                if (type == "buffer") {
                    uint64_t size = valid ? req.get("size", Json::Value(0u)).asUInt64() : 0u;
                    /* JSON array of byte ints — byte-exact with the C original. */
                    std::string out;
                    out.reserve(static_cast<size_t>(size) * 4 + 4);
                    out += '[';
                    for (uint64_t i = 0; i < size; i++) {
                        if (i != 0) {
                            out += ',';
                        }
                        out += std::to_string(memGet(addr + i));
                    }
                    out += ']';
                    sock.send(zmq::const_buffer(out.data(), out.size()),
                              zmq::send_flags::none);
                } else {
                    /* scalar: bare decimal uint32, no trailing newline */
                    std::string out = std::to_string(regGet(addr));
                    sock.send(zmq::const_buffer(out.data(), out.size()),
                              zmq::send_flags::none);
                }
            }
        } else {
            /* Unknown command: reply something non-OK so the client surfaces it. */
            sock.send(zmq::const_buffer("ERR", 3), zmq::send_flags::none);
        }
    }

    return 0;
}
