// cadoc kernel binding: public entry points shared by the WASM and native builds.
// SPDX-License-Identifier: LGPL-2.1-only
#pragma once
#include <cstddef>
#include <cstdint>
#include <string>

namespace cadoc {
/// Answers one protocol request (JSON). Binary payloads (the STEP bytes of an
/// "import") travel beside the JSON as a raw buffer, never base64.
std::string handle(const std::string& requestJson, const uint8_t* bytes, size_t len);
/// Timing breakdown of the last handle() call, measured inside C++ (JSON).
/// Not part of the protocol; used by benchmarks.
const std::string& lastTiming();
std::string version();
/// Native JSON-lines transport: one Envelope<KernelRequest> line in, one Envelope line out,
/// the reference kernel's wire format (typed arrays as base64, kernel/reference/README.md).
std::string handleEnvelopeLine(const std::string& line);
/// WASM: typed arrays in responses go to a binary side section instead of base64
/// (kernel/README.md, "Binary responses"); lastBinary() is that section, valid until the next call.
void setBinaryResponses(bool on);
const std::string& lastBinary();
}  // namespace cadoc
