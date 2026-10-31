// cadoc kernel binding: a thin, product-logic-free layer over OCCT that answers
// kernel/schema/protocol.ts messages. The same source is compiled to WASM
// (kernel_wasm.cpp entry points) and natively (kernel_cli.cpp).
//
// This file only dispatches. The handlers implement protocol v1 with the behaviour of
// the reference kernel (kernel/reference/refkernel.py), one file each:
//   import.cpp   "import"   STEP/BRep -> assembly tree + shape info
//   mesh.cpp     "mesh"     BRepMesh triangulations
//   hlr.cpp      "hlr"      exact or polygonal hidden-line removal -> Curve2D (curves.cpp)
//   spatial.cpp  "clearance", "sweep"
//   shapes.cpp   "exportShapes", "loadShapes" (binary B-rep cache for large views),
//                "importCache", "loadCache" (the whole imported document, so a file is imported once)
//   session.cpp  the current document, placements, meshes, typed-array wire forms
//
// SPDX-License-Identifier: LGPL-2.1-only
#include "kernel.hpp"

#include "session.hpp"

#include <Message.hxx>
#include <Message_Messenger.hxx>
#include <Message_Printer.hxx>
#include <OSD_ThreadPool.hxx>
#include <Standard_Failure.hxx>
#include <Standard_Version.hxx>

#include <cstdio>
#include <new>

namespace cadoc {
namespace {

std::string g_lastTiming = "{}";

void ensureInit() {
  static bool done = false;
  if (done) return;
  done = true;
  // OCCT's own algorithms run single-threaded on both builds: the WASM build has no threads, so
  // the native build must not get parallelism inside OCCT the WASM build cannot have. (The import's
  // per-shape properties are spread over threads by import.cpp itself, natively only.)
  OSD_ThreadPool::DefaultPool(1);
  // Silence OCCT's default console printer: output costs time and is not data.
  Message::DefaultMessenger()->RemovePrinters(STANDARD_TYPE(Message_Printer));
}

std::string errorJson(const std::string& code, const std::string& msg) {
  json::Writer w;
  w.beginObject().key("type").str("error").key("code").str(code).key("message").str(msg).endObject();
  return std::move(w.out);
}

std::string dispatch(const json::Value& req, const uint8_t* bytes, size_t len, json::Writer& timing, std::string& type) {
  const json::Value* t = req.get("type");
  type = t && t->kind == json::Value::Kind::String ? t->s : "?";
  if (type == "import") return doImport(req, bytes, len, timing);
  if (type == "mesh") return doMesh(req, timing);
  if (type == "hlr") return doHlr(req, timing);
  if (type == "clearance") return doClearance(req, timing);
  if (type == "sweep") return doSweep(req, timing);
  if (type == "exportShapes") return doExportShapes(req, timing);
  if (type == "loadShapes") return doLoadShapes(req, bytes, len, timing);
  if (type == "importCache") return doImportCache(req, timing);
  if (type == "loadCache") return doLoadCache(req, bytes, len, timing);
  throw bad("unknown request type " + (t && t->kind == json::Value::Kind::String ? repr(t->s) : std::string("None")));
}

/// Answers one request body (or reports why it could not be parsed). Never throws.
std::string answer(const json::Value* req, const std::string& parseError, const uint8_t* bytes, size_t len) {
  ensureInit();
  binarySection().clear();
  auto t0 = Clock::now();
  json::Writer timing;
  timing.beginObject();
  std::string out, type = "?";
  try {
    if (!req) throw bad(parseError);
    if (req->kind != json::Value::Kind::Object) throw bad("expected a request object");
    out = dispatch(*req, bytes, len, timing, type);
  } catch (const KernelError& e) {
    out = errorJson(e.code, e.what());
  } catch (const Standard_Failure& e) {
    out = errorJson("crash", std::string("OCCT: ") + e.what());
  } catch (const std::bad_alloc&) {
    out = errorJson("out-of-memory", "out of memory");
  } catch (const std::exception& e) {
    out = errorJson("bad-request", e.what());
  }
  if (out.rfind("{\"type\":\"error\"", 0) == 0) binarySection().clear();
  timing.key("type").str(type).key("callMs").num(msSince(t0)).key("responseBytes").integer((long long)out.size());
  timing.endObject();
  g_lastTiming = std::move(timing.out);
  return out;
}

}  // namespace

void setBinaryResponses(bool on) { setTypedMode(on ? TypedMode::Binary : TypedMode::Base64); }
const std::string& lastBinary() { return binarySection(); }

std::string handle(const std::string& requestJson, const uint8_t* bytes, size_t len) {
  json::Value req;
  try {
    req = json::Parser(requestJson).parse();
  } catch (const std::exception& e) {
    return answer(nullptr, std::string("invalid JSON: ") + e.what(), bytes, len);
  }
  return answer(&req, "", bytes, len);
}

std::string handleEnvelopeLine(const std::string& line) {
  json::Writer w;
  w.beginObject();
  json::Value env;
  try {
    env = json::Parser(line).parse();
  } catch (const std::exception& e) {
    w.key("id").raw("null").key("body").raw(errorJson("bad-request", std::string("invalid JSON: ") + e.what()));
    return std::move(w.endObject().out);
  }
  const json::Value* id = env.get("id");
  w.key("id");
  if (id && id->kind == json::Value::Kind::Number) w.num(id->n);
  else if (id && id->kind == json::Value::Kind::String) w.str(id->s);
  else w.raw("null");
  const json::Value* b = env.get("body");
  if (env.kind != json::Value::Kind::Object || !b || b->kind != json::Value::Kind::Object) {
    w.key("body").raw(errorJson("bad-request", "expected an Envelope {id, body}"));
    return std::move(w.endObject().out);
  }
  // Import bytes on this transport: {"$typed": "Uint8Array", "base64": ...} or a plain array of
  // numbers; or, as a native-transport extension (not protocol v1), "path": an absolute file name.
  std::string data, failure, failureCode = "bad-request";
  const json::Value* type = b->get("type");
  const bool takesBytes = type && type->kind == json::Value::Kind::String && (type->s == "import" || type->s == "loadShapes" || type->s == "loadCache");
  if (takesBytes) {
    const json::Value* bytes = b->get("bytes");
    const json::Value* path = b->get("path");
    if (bytes && bytes->kind == json::Value::Kind::Object && bytes->get("base64")) {
      try {
        data = base64Decode(bytes->get("base64")->str());
      } catch (const std::exception& e) {
        failure = e.what();
      }
    } else if (bytes && bytes->kind == json::Value::Kind::Array) {
      data.reserve(bytes->a.size());
      for (const auto& x : bytes->a) data += char(int(x.n) & 0xFF);
    } else if (!bytes && path && path->kind == json::Value::Kind::String) {
      if (path->s.empty() || path->s[0] != '/') {
        failure = type->s + ".path must be absolute";
      } else if (FILE* f = std::fopen(path->s.c_str(), "rb")) {
        char buf[1 << 16];
        for (size_t n; (n = std::fread(buf, 1, sizeof buf, f)) > 0;) data.append(buf, n);
        std::fclose(f);
      } else {
        failure = "no such file: " + path->s;
        failureCode = "read-failed";
      }
    } else {
      failure = type->s + ".bytes must be a Uint8Array";
    }
  }
  w.key("body");
  if (!failure.empty()) w.raw(errorJson(failureCode, failure));
  else w.raw(answer(b, "", reinterpret_cast<const uint8_t*>(data.data()), data.size()));
  return std::move(w.endObject().out);
}

const std::string& lastTiming() { return g_lastTiming; }

// "import rN" changes whenever an import's output (ImportResult or shapes) could change, so an
// import cache keyed by this string (protocol.ts, "import cache") is never read by a kernel that
// would import the file differently. r2: OCCT patched (kernel/build/patches), same output.
// r3: OCCT patch 0004, an exact BinTools round trip (directions and axis frames read as written,
// locations written and read exactly, format version 5), so a document restored by loadCache
// is bit-identical to the import's; an r2 cache restored shapes a few ulps off.
// r4: a part whose solids the file names one by one is split into one part per solid (import.cpp,
// namedSolids), so an r3 cache of such a file holds the wrong tree.
// "hlr rN" changes whenever exact HLR of the same shapes could answer differently (r2: OCCT
// patches 0002/0003, the outline walk), for caches of HLR results keyed by this string.
std::string version() {
#ifdef __EMSCRIPTEN__
  const char* build = "wasm";
#else
  const char* build = "native";
#endif
  return std::string("cadoc-kernel 1 (protocol v1; import r4") + (kExactBrep ? "" : ", inexact cache") + "; hlr r2; " + build +
         "); OCCT " + OCC_VERSION_COMPLETE;
}

}  // namespace cadoc
