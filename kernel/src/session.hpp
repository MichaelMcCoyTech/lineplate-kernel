// Kernel session state and helpers shared by the request handlers.
//
// One document at a time, as in the reference kernel (kernel/reference/README.md):
// an "import" replaces every shape, and with it every mesh and sweep cache.
// SPDX-License-Identifier: LGPL-2.1-only
#pragma once

#include "json.hpp"

#include <BinTools_FormatVersion.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Trsf.hxx>

#include <chrono>
#include <map>
#include <memory>
#include <streambuf>
#include <stdexcept>
#include <string>
#include <vector>

namespace cadoc {

/// A failure answered as a protocol KernelError with this code.
struct KernelError : std::runtime_error {
  std::string code;
  KernelError(std::string c, const std::string& m) : std::runtime_error(m), code(std::move(c)) {}
};
inline KernelError bad(const std::string& m) { return KernelError("bad-request", m); }

using Clock = std::chrono::steady_clock;
inline double msSince(Clock::time_point t0) {
  return std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
}

// Read-only, seekable stream over the caller's buffer (no copy).
struct MemBuf : std::streambuf {
  MemBuf(const uint8_t* p, size_t n) {
    char* c = const_cast<char*>(reinterpret_cast<const char*>(p));
    setg(c, c, c + n);
  }
  pos_type seekoff(off_type off, std::ios_base::seekdir dir, std::ios_base::openmode) override {
    char* target = dir == std::ios_base::beg ? eback() + off : dir == std::ios_base::cur ? gptr() + off : egptr() + off;
    if (target < eback() || target > egptr()) return pos_type(off_type(-1));
    setg(eback(), target, egptr());
    return pos_type(target - eback());
  }
  pos_type seekpos(pos_type pos, std::ios_base::openmode m) override { return seekoff(off_type(pos), std::ios_base::beg, m); }
};

/// One distinct shape of the current document (ShapeId "s<n>").
struct ShapeRec {
  std::string id;
  TopoDS_Shape shape;  // prototype, mm, own coordinates (its label's location included)
  int faceCount = 0;
  int edgeCount = 0;
  bool solid = false;
  double diag = 0;    // optimal bbox diagonal
  double volume = 0;  // as reported in ShapeInfo
  // The triangulation currently stored on `shape` (BRepMesh writes it into the faces).
  bool meshed = false;
  double meshLin = 0, meshAng = 0;
  // Sweep mesh cache: deflection, positions (xyz), triangles (outward winding).
  double sweepDefl = -1;
  std::vector<double> sweepPos;
  std::vector<int> sweepTri;
};

struct Session {
  std::vector<std::unique_ptr<ShapeRec>> shapes;  // in ShapeId order
  std::map<std::string, ShapeRec*> byId;
  /// The import:ok response that produced these shapes (import or loadCache), for importCache.
  /// Empty once loadShapes has changed the set.
  std::string importJson;
  void clear() {
    byId.clear();
    shapes.clear();
    importJson.clear();
  }
};
Session& session();

ShapeRec& getShape(const json::Value* id);

/// A PlacedInstance of the request: the prototype moved into world coordinates.
struct Placed {
  std::string instanceId;
  ShapeRec* rec = nullptr;
  gp_Trsf trsf;
  TopoDS_Shape shape;
  /// hlr instanceFaces: the number of faces `shape` keeps of the instance, or -1 when whole.
  int partial = -1;
};
Placed parsePlaced(const json::Value* p, const std::string& what);

gp_Trsf readMat4(const json::Value* v);
void writeMat4(json::Writer& w, const gp_Trsf& t);

struct Box3 {
  double lo[3], hi[3];
};
/// BRepBndLib::Add (or AddOptimal), without triangulation. False if the box is void.
bool bboxOf(const TopoDS_Shape& s, bool optimal, Box3& out);

/// The BinTools format the kernel writes. OCCT patch 0004 (kernel/build/patches) adds version 5,
/// which writes each location exactly, and makes the reader restore directions and axis frames as
/// written: read(write(shape)) is then the same shape, bit for bit. Built against an OCCT without
/// the patch, the kernel writes the current format and its round trip is not exact (version()
/// says so, so such a kernel never shares an import cache with an exact one).
constexpr int kBrepFormat = BinTools_FormatVersion_UPPER >= 5 ? 5 : BinTools_FormatVersion_CURRENT;
constexpr bool kExactBrep = kBrepFormat >= 5;

/// BinTools write and read (format kBrepFormat, no triangulation). readBrep throws read-failed.
std::string writeBrep(const TopoDS_Shape& s);
TopoDS_Shape readBrep(const char* data, size_t n, const std::string& what);

/// Triangulates the prototype at these deflections unless it already is (reference kernel's ensure_mesh).
void ensureMesh(ShapeRec& rec, double lin, double ang);

/// Per-face triangle mesh of a prototype (reference kernel's tri_mesh): per-face vertices,
/// winding and normals flipped on REVERSED faces, face index in TopExp::MapShapes order.
struct TriMesh {
  std::vector<double> pos, nrm;  // xyz
  std::vector<uint32_t> idx;     // triangles
  std::vector<uint32_t> faceOfTriangle;
};
TriMesh triMesh(ShapeRec& rec, double lin, double ang);

/// Typed arrays in a response. On the JSON-lines transport (native CLI) they are
/// {"$typed": kind, "base64": ...}; in the WASM build they go to a binary side section
/// and the JSON says where: {"$typed": kind, "offset": byteOffset, "byteLength": n}.
void writeTyped(json::Writer& w, const char* kind, const void* data, size_t bytes);
enum class TypedMode { Base64, Binary };
void setTypedMode(TypedMode m);
TypedMode typedMode();
/// The binary section of the last response (Binary mode); offsets are 8-byte aligned.
std::string& binarySection();
std::string base64Encode(const void* data, size_t bytes);
std::string base64Decode(const std::string& b64);

// Request handlers (one per protocol request type).
std::string doImport(const json::Value& req, const uint8_t* bytes, size_t len, json::Writer& timing);
std::string doMesh(const json::Value& req, json::Writer& timing);
std::string doHlr(const json::Value& req, json::Writer& timing);
std::string doClearance(const json::Value& req, json::Writer& timing);
std::string doSweep(const json::Value& req, json::Writer& timing);
// Shape cache for large views (not in the reference kernel): BinTools B-reps out and in.
std::string doExportShapes(const json::Value& req, json::Writer& timing);
std::string doLoadShapes(const json::Value& req, const uint8_t* bytes, size_t len, json::Writer& timing);
// Import cache (not in the reference kernel): the whole imported document as one blob, and back.
std::string doImportCache(const json::Value& req, json::Writer& timing);
std::string doLoadCache(const json::Value& req, const uint8_t* bytes, size_t len, json::Writer& timing);

/// Python-repr-like quoting for messages ('x'), as the reference kernel words them.
inline std::string repr(const std::string& s) { return "'" + s + "'"; }

}  // namespace cadoc
