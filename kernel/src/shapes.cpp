// "exportShapes" / "loadShapes": the shape cache for large views (protocol.ts, "shape cache").
// A fresh kernel instance that runs one HLR batch loads only that batch's shapes, as OCCT
// binary B-rep (BinTools format version 5, no triangulation), instead of re-reading a STEP
// file that can take minutes.
// "importCache" / "loadCache": the whole imported document as one blob (protocol.ts, "import
// cache"), so a file is imported once: loadCache leaves the kernel exactly as the import did and
// answers with the import's own ImportResult. Neither is part of the reference kernel.
// SPDX-License-Identifier: LGPL-2.1-only
#include "kernel.hpp"
#include "session.hpp"

#include <BinTools.hxx>
#include <NCollection_IndexedMap.hxx>
#include <TopAbs.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_ShapeMapHasher.hxx>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <istream>
#include <sstream>
#include <vector>

namespace cadoc {
namespace {

int countOf(const TopoDS_Shape& s, TopAbs_ShapeEnum t) {
  NCollection_IndexedMap<TopoDS_Shape, TopTools_ShapeMapHasher> m;
  TopExp::MapShapes(s, t, m);
  return m.Extent();
}

}  // namespace

std::string writeBrep(const TopoDS_Shape& shape) {
  if (shape.IsNull()) return std::string();
  std::ostringstream os(std::ios::out | std::ios::binary);
  BinTools::Write(shape, os, false, false, BinTools_FormatVersion(kBrepFormat));
  return os.str();
}


TopoDS_Shape readBrep(const char* data, size_t n, const std::string& what) {
  TopoDS_Shape shape;
  if (n == 0) return shape;
  MemBuf buf(reinterpret_cast<const uint8_t*>(data), n);
  std::istream in(&buf);
  BinTools::Read(shape, in);
  if (shape.IsNull()) throw KernelError("read-failed", "empty B-rep for " + what);
  return shape;
}

std::string doExportShapes(const json::Value& req, json::Writer& timing) {
  const json::Value* ids = req.get("shapeIds");
  if (!ids || ids->kind != json::Value::Kind::Array) throw bad("exportShapes.shapeIds must be an array");
  json::Writer w;
  w.beginObject().key("type").str("exportShapes:ok").key("shapes").beginArray();
  long long total = 0;
  for (const auto& sid : ids->a) {
    ShapeRec& rec = getShape(&sid);
    const std::string b = writeBrep(rec.shape);
    total += (long long)b.size();
    w.beginObject().key("shapeId").str(rec.id).key("brep");
    writeTyped(w, "Uint8Array", b.data(), b.size());
    w.endObject();
  }
  w.endArray().endObject();
  timing.key("brepBytes").integer(total);
  return std::move(w.out);
}

std::string doLoadShapes(const json::Value& req, const uint8_t* bytes, size_t len, json::Writer& timing) {
  const json::Value* list = req.get("shapes");
  if (!list || list->kind != json::Value::Kind::Array) throw bad("loadShapes.shapes must be an array");
  const json::Value* replace = req.get("replace");
  if (replace && replace->kind == json::Value::Kind::Bool && replace->b) session().clear();
  session().importJson.clear();  // the shapes are no longer exactly an import's
  int n = 0;
  for (const auto& e : list->a) {
    const json::Value* id = e.get("shapeId");
    const json::Value* off = e.get("offset");
    const json::Value* length = e.get("length");
    if (!id || id->kind != json::Value::Kind::String || !off || !length) throw bad("loadShapes entries need shapeId, offset, length");
    const double o = off->num(), l = length->num();
    if (o < 0 || l <= 0 || o + l > double(len)) throw bad("loadShapes: slice of " + repr(id->s) + " lies outside the bytes");
    auto rec = std::make_unique<ShapeRec>();
    rec->shape = readBrep(reinterpret_cast<const char*>(bytes) + size_t(o), size_t(l), repr(id->s));
    rec->id = id->s;
    rec->faceCount = countOf(rec->shape, TopAbs_FACE);
    rec->edgeCount = countOf(rec->shape, TopAbs_EDGE);
    for (TopExp_Explorer ex(rec->shape, TopAbs_SOLID); ex.More();) { rec->solid = true; break; }
    Box3 b;
    if (bboxOf(rec->shape, false, b))
      rec->diag = std::sqrt(std::pow(b.hi[0] - b.lo[0], 2) + std::pow(b.hi[1] - b.lo[1], 2) + std::pow(b.hi[2] - b.lo[2], 2));
    // volume stays 0: ShapeInfo came from the import; HLR does not need it.
    auto& s = session();
    if (auto it = s.byId.find(rec->id); it != s.byId.end()) {
      for (auto& p : s.shapes)
        if (p.get() == it->second) p = std::move(rec), it->second = p.get();
    } else {
      s.byId[rec->id] = rec.get();
      s.shapes.push_back(std::move(rec));
    }
    ++n;
  }
  json::Writer w;
  w.beginObject().key("type").str("loadShapes:ok").key("loaded").integer(n).key("held").integer((long long)session().shapes.size()).endObject();
  timing.key("loaded").integer(n);
  return std::move(w.out);
}

// ---------------------------------------------------------------- import cache
//
// Blob layout (protocol.ts, ImportCacheHeader), little-endian:
//   0   "CADOCIC1"
//   8   u32 header length H
//   12  header, UTF-8 JSON
//   D   data section, D = 12 + H rounded up to 8: the import:ok JSON, then each shape's B-rep,
//       each starting at a multiple of 8; header offsets are relative to D.

namespace {

constexpr char kMagic[8] = {'C', 'A', 'D', 'O', 'C', 'I', 'C', '1'};
constexpr int kCacheVersion = 1;


}  // namespace

std::string doImportCache(const json::Value& req, json::Writer& timing) {
  Session& s = session();
  if (s.importJson.empty()) throw bad("importCache needs an imported document (import or loadCache first)");
  auto t0 = Clock::now();
  // Each shape's B-rep, then the header (which needs their sizes), then one pass that writes the
  // blob where it goes (a file, or the response's binary section) and frees each part as it goes:
  // the WASM heap holds the B-reps about once, not once per copy.
  std::vector<std::string> breps;
  breps.reserve(s.shapes.size());
  for (const auto& rp : s.shapes) breps.push_back(writeBrep(rp->shape));
  const auto up8 = [](size_t n) { return (n + 7) / 8 * 8; };
  const size_t importLen = s.importJson.size();
  json::Writer h;
  h.beginObject().key("format").str("cadoc-import-cache").key("version").integer(kCacheVersion);
  h.key("kernel").str(version());
  h.key("import").beginObject().key("offset").integer(0).key("length").integer((long long)importLen).endObject();
  h.key("shapes").beginArray();
  size_t off = importLen;
  for (size_t i = 0; i < s.shapes.size(); ++i) {
    const ShapeRec& r = *s.shapes[i];
    off = up8(off);
    h.beginObject().key("shapeId").str(r.id).key("offset").integer((long long)off).key("length").integer((long long)breps[i].size());
    h.key("faceCount").integer(r.faceCount).key("edgeCount").integer(r.edgeCount).key("solid").boolean(r.solid);
    h.key("volume").num(r.volume).key("diag").num(r.diag).endObject();
    off += breps[i].size();
  }
  h.endArray().endObject();
  const double brepMs = msSince(t0);
  const size_t D = up8(12 + h.out.size());
  const size_t total = D + off;

  // The sink: a file (native "path"), the binary section (WASM), or a string to base64 (stdio).
  const json::Value* path = req.get("path");
  const bool toFile = path && path->kind == json::Value::Kind::String;
  if (toFile && (path->s.empty() || path->s[0] != '/')) throw bad("importCache.path must be absolute");
  FILE* f = nullptr;
  std::string local;
  std::string& bin = binarySection();
  const bool toBinary = !toFile && typedMode() == TypedMode::Binary;
  std::string& sink = toBinary ? bin : local;
  size_t base = 0;
  if (toBinary) {
    while (bin.size() % 8) bin += '\0';
    base = bin.size();
    bin.reserve(base + total);
  } else if (!toFile) {
    local.reserve(total);
  } else if (!(f = std::fopen(path->s.c_str(), "wb"))) {
    throw KernelError("read-failed", "cannot write " + path->s);
  }
  size_t written = 0;
  bool ok = true;
  const auto put = [&](const char* p, size_t n) {
    if (f) ok = ok && std::fwrite(p, 1, n, f) == n;
    else sink.append(p, n);
    written += n;
  };
  const auto pad = [&](size_t to) {
    static const char zeros[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    while (written < to) put(zeros, std::min<size_t>(8, to - written));
  };
  put(kMagic, 8);
  char hl[4];
  for (int k = 0; k < 4; ++k) hl[k] = char((uint32_t(h.out.size()) >> (8 * k)) & 0xFF);
  put(hl, 4);
  put(h.out.data(), h.out.size());
  pad(D);
  put(s.importJson.data(), importLen);
  for (auto& b : breps) {
    pad(up8(written));
    put(b.data(), b.size());
    std::string().swap(b);  // free as we go
  }
  if (f) {
    ok = std::fclose(f) == 0 && ok;
    if (!ok) throw KernelError("read-failed", "short write to " + path->s);
  }

  json::Writer w;
  w.beginObject().key("type").str("importCache:ok").key("shapes").integer((long long)s.shapes.size());
  if (toFile) {
    w.key("path").str(path->s).key("byteLength").integer((long long)total);
  } else if (toBinary) {
    w.key("cache").beginObject().key("$typed").str("Uint8Array").key("offset").integer((long long)base).key("byteLength").integer((long long)total).endObject();
  } else {
    w.key("cache");
    writeTyped(w, "Uint8Array", local.data(), local.size());
  }
  w.endObject();
  timing.key("brepMs").num(brepMs).key("cacheBytes").integer((long long)total);
  return std::move(w.out);
}

std::string doLoadCache(const json::Value& req, const uint8_t* bytes, size_t len, json::Writer& timing) {
  (void)req;
  auto t0 = Clock::now();
  if (len < 16 || std::memcmp(bytes, kMagic, 8) != 0) throw KernelError("read-failed", "not a cadoc import cache");
  const uint32_t hl = uint32_t(bytes[8]) | (uint32_t(bytes[9]) << 8) | (uint32_t(bytes[10]) << 16) | (uint32_t(bytes[11]) << 24);
  if (size_t(12) + hl > len) throw KernelError("read-failed", "import cache header is truncated");
  json::Value h;
  try {
    h = json::Parser(std::string(reinterpret_cast<const char*>(bytes) + 12, hl)).parse();
  } catch (const std::exception& e) {
    throw KernelError("read-failed", std::string("import cache header: ") + e.what());
  }
  const json::Value* ver = h.get("version");
  if (!ver || ver->kind != json::Value::Kind::Number || int(ver->n) != kCacheVersion)
    throw KernelError("read-failed", "import cache version is not " + std::to_string(kCacheVersion));
  const size_t D = (12 + size_t(hl) + 7) / 8 * 8;
  if (D > len) throw KernelError("read-failed", "import cache is truncated");
  const uint8_t* data = bytes + D;
  const size_t dataLen = len - D;
  auto slice = [&](const json::Value* e, const std::string& what) -> std::pair<size_t, size_t> {
    const json::Value* o = e ? e->get("offset") : nullptr;
    const json::Value* l = e ? e->get("length") : nullptr;
    if (!o || !l) throw KernelError("read-failed", "import cache: " + what + " has no slice");
    const double off = o->num(), n = l->num();
    if (off < 0 || n < 0 || off + n > double(dataLen)) throw KernelError("read-failed", "import cache: " + what + " lies outside the blob");
    return {size_t(off), size_t(n)};
  };
  const auto [io, il] = slice(h.get("import"), "the import result");
  const json::Value* list = h.get("shapes");
  if (!list || list->kind != json::Value::Kind::Array) throw KernelError("read-failed", "import cache: no shape table");
  std::vector<std::unique_ptr<ShapeRec>> recs;
  recs.reserve(list->a.size());
  for (const auto& e : list->a) {
    const json::Value* id = e.get("shapeId");
    if (!id || id->kind != json::Value::Kind::String) throw KernelError("read-failed", "import cache: a shape has no id");
    const auto [o, n] = slice(&e, "shape " + repr(id->s));
    auto rec = std::make_unique<ShapeRec>();
    rec->id = id->s;
    rec->shape = readBrep(reinterpret_cast<const char*>(data) + o, n, repr(id->s));
    auto num = [&](const char* k) {
      const json::Value* v = e.get(k);
      return v ? v->num() : 0.0;
    };
    rec->faceCount = int(num("faceCount"));
    rec->edgeCount = int(num("edgeCount"));
    const json::Value* solid = e.get("solid");
    rec->solid = solid && solid->kind == json::Value::Kind::Bool && solid->b;
    rec->volume = num("volume");
    rec->diag = num("diag");
    recs.push_back(std::move(rec));
  }
  // The new document replaces the session, as an import does.
  Session& s = session();
  s.clear();
  for (auto& r : recs) {
    s.byId[r->id] = r.get();
    s.shapes.push_back(std::move(r));
  }
  s.importJson.assign(reinterpret_cast<const char*>(data + io), il);
  json::Writer w;
  w.beginObject().key("type").str("loadCache:ok").key("import").raw(s.importJson).endObject();
  timing.key("loadMs").num(msSince(t0)).key("shapes").integer((long long)s.shapes.size());
  return std::move(w.out);
}

}  // namespace cadoc
