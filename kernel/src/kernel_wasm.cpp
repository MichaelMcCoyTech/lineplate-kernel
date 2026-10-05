// WASM entry points: a plain C ABI so the JS glue needs no embind.
// SPDX-License-Identifier: LGPL-2.1-only
#include "kernel.hpp"

#include <emscripten/emscripten.h>
#include <malloc.h>

static std::string g_out;
static std::string g_version;

extern "C" {
/// Returns a NUL-terminated JSON response, valid until the next call.
/// Typed arrays in the response are {"$typed": kind, "offset": o, "byteLength": n}: n bytes at
/// cadoc_binary_ptr() + o (o is a multiple of 8), valid until the next call, like the JSON.
EMSCRIPTEN_KEEPALIVE const char* cadoc_handle(const char* requestJson, const uint8_t* bytes, size_t len) {
  cadoc::setBinaryResponses(true);
  g_out = cadoc::handle(requestJson, bytes, len);
  return g_out.c_str();
}
EMSCRIPTEN_KEEPALIVE const uint8_t* cadoc_binary_ptr() {
  return reinterpret_cast<const uint8_t*>(cadoc::lastBinary().data());
}
EMSCRIPTEN_KEEPALIVE double cadoc_binary_len() { return (double)cadoc::lastBinary().size(); }
EMSCRIPTEN_KEEPALIVE const char* cadoc_last_timing() { return cadoc::lastTiming().c_str(); }
EMSCRIPTEN_KEEPALIVE const char* cadoc_version() {
  g_version = cadoc::version();
  return g_version.c_str();
}
/// Bytes currently allocated by malloc, and the allocator's peak footprint.
EMSCRIPTEN_KEEPALIVE double cadoc_heap_in_use() {
#ifdef CADOC_NO_MALLINFO  // allocators without mallinfo (e.g. -sMALLOC=mimalloc)
  return -1;
#else
  return (double)mallinfo().uordblks;
#endif
}
EMSCRIPTEN_KEEPALIVE double cadoc_heap_footprint() {
#ifdef CADOC_NO_MALLINFO
  return -1;
#else
  return (double)mallinfo().usmblks;
#endif
}
}
