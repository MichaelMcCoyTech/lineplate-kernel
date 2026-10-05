// Session state, placement parsing, bounding boxes, meshing and the typed-array wire forms.
// SPDX-License-Identifier: LGPL-2.1-only
#include "session.hpp"

#include <BRepBndLib.hxx>
#include <BRepLib_ToolTriangulatedShape.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRepTools.hxx>
#include <BRep_Tool.hxx>
#include <Bnd_Box.hxx>
#include <NCollection_IndexedMap.hxx>
#include <Poly_Triangulation.hxx>
#include <Standard_Failure.hxx>
#include <TopAbs.hxx>
#include <TopExp.hxx>
#include <TopLoc_Location.hxx>
#include <TopTools_ShapeMapHasher.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>

#include <cmath>
#include <cstring>

namespace cadoc {

Session& session() {
  static Session s;
  return s;
}

ShapeRec& getShape(const json::Value* id) {
  if (id && id->kind == json::Value::Kind::String) {
    auto it = session().byId.find(id->s);
    if (it != session().byId.end()) return *it->second;
  }
  std::string shown = id && id->kind == json::Value::Kind::String ? repr(id->s) : "None";
  throw bad("unknown shapeId " + shown + " (import a file first; each import replaces the previous one)");
}

gp_Trsf readMat4(const json::Value* v) {
  if (!v || v->kind != json::Value::Kind::Array || v->a.size() != 16) throw bad("transform must be 16 numbers, column-major");
  double m[16];
  try {
    for (int i = 0; i < 16; ++i) m[i] = v->a[size_t(i)].num();
  } catch (const std::exception&) {
    throw bad("transform must be 16 numbers");
  }
  if (std::abs(m[3]) > 1e-9 || std::abs(m[7]) > 1e-9 || std::abs(m[11]) > 1e-9 || std::abs(m[15] - 1) > 1e-9)
    throw bad("transform must be affine (last row 0 0 0 1)");
  gp_Trsf t;
  try {
    // column-major: element (row r, col c) is m[c*4 + r]
    t.SetValues(m[0], m[4], m[8], m[12], m[1], m[5], m[9], m[13], m[2], m[6], m[10], m[14]);
  } catch (const Standard_Failure& e) {
    throw bad(std::string("transform is not a rigid motion (with optional uniform scale): ") + e.what());
  }
  return t;
}

void writeMat4(json::Writer& w, const gp_Trsf& t) {
  w.beginArray();
  for (int c = 1; c <= 4; ++c)
    for (int r = 1; r <= 4; ++r) w.num(r == 4 ? (c == 4 ? 1.0 : 0.0) : t.Value(r, c));
  w.endArray();
}

Placed parsePlaced(const json::Value* p, const std::string& what) {
  if (!p || p->kind != json::Value::Kind::Object) throw bad(what + " must be a PlacedInstance object");
  const json::Value* iid = p->get("instanceId");
  if (!iid || iid->kind != json::Value::Kind::String) throw bad(what + ".instanceId must be a string");
  Placed out;
  out.instanceId = iid->s;
  out.rec = &getShape(p->get("shapeId"));
  out.trsf = readMat4(p->get("transform"));
  out.shape = out.rec->shape.Moved(TopLoc_Location(out.trsf));
  return out;
}

bool bboxOf(const TopoDS_Shape& s, bool optimal, Box3& out) {
  Bnd_Box box;
  if (optimal) BRepBndLib::AddOptimal(s, box, false, false);
  else BRepBndLib::Add(s, box, false);
  if (box.IsVoid()) return false;
  box.Get(out.lo[0], out.lo[1], out.lo[2], out.hi[0], out.hi[1], out.hi[2]);
  return true;
}

static double round9(double x) { return std::round(x * 1e9) / 1e9; }

void ensureMesh(ShapeRec& rec, double lin, double ang) {
  if (rec.meshed && rec.meshLin == round9(lin) && rec.meshAng == round9(ang)) return;
  BRepTools::Clean(rec.shape);
  BRepMesh_IncrementalMesh(rec.shape, lin, false, ang, true);
  rec.meshed = true;
  rec.meshLin = round9(lin);
  rec.meshAng = round9(ang);
}

TriMesh triMesh(ShapeRec& rec, double lin, double ang) {
  ensureMesh(rec, lin, ang);
  NCollection_IndexedMap<TopoDS_Shape, TopTools_ShapeMapHasher> faces;
  TopExp::MapShapes(rec.shape, TopAbs_FACE, faces);
  TriMesh m;
  uint32_t base = 0;
  for (int fi = 1; fi <= faces.Extent(); ++fi) {
    const TopoDS_Face& face = TopoDS::Face(faces.FindKey(fi));
    TopLoc_Location loc;
    occ::handle<Poly_Triangulation> tri = BRep_Tool::Triangulation(face, loc);
    if (tri.IsNull() || tri->NbTriangles() == 0) continue;
    if (!tri->HasNormals()) BRepLib_ToolTriangulatedShape::ComputeNormals(face, tri);
    const int nn = tri->NbNodes();
    const gp_Trsf trsf = loc.Transformation();
    const bool rev = face.Orientation() == TopAbs_REVERSED;
    for (int i = 1; i <= nn; ++i) {
      gp_Pnt p = tri->Node(i).Transformed(trsf);
      m.pos.insert(m.pos.end(), {p.X(), p.Y(), p.Z()});
      gp_Dir d = tri->Normal(i);
      d.Transform(trsf);
      const double s = rev ? -1.0 : 1.0;
      m.nrm.insert(m.nrm.end(), {s * d.X(), s * d.Y(), s * d.Z()});
    }
    for (int t = 1; t <= tri->NbTriangles(); ++t) {
      int a, b, c;
      tri->Triangle(t).Get(a, b, c);
      if (rev) std::swap(b, c);
      m.idx.insert(m.idx.end(), {base + uint32_t(a - 1), base + uint32_t(b - 1), base + uint32_t(c - 1)});
      m.faceOfTriangle.push_back(uint32_t(fi - 1));
    }
    base += uint32_t(nn);
  }
  return m;
}

// ------------------------------------------------------------------ typed arrays on the wire

static TypedMode g_mode = TypedMode::Base64;
void setTypedMode(TypedMode m) { g_mode = m; }
TypedMode typedMode() { return g_mode; }
std::string& binarySection() {
  static std::string s;
  return s;
}

static const char* kB64 = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

std::string base64Encode(const void* data, size_t bytes) {
  const auto* p = static_cast<const unsigned char*>(data);
  std::string b;
  b.reserve((bytes + 2) / 3 * 4);
  size_t i = 0;
  for (; i + 2 < bytes; i += 3) {
    uint32_t v = (uint32_t(p[i]) << 16) | (uint32_t(p[i + 1]) << 8) | p[i + 2];
    b += kB64[v >> 18];
    b += kB64[(v >> 12) & 63];
    b += kB64[(v >> 6) & 63];
    b += kB64[v & 63];
  }
  if (i < bytes) {
    uint32_t v = uint32_t(p[i]) << 16;
    if (i + 1 < bytes) v |= uint32_t(p[i + 1]) << 8;
    b += kB64[v >> 18];
    b += kB64[(v >> 12) & 63];
    b += i + 1 < bytes ? kB64[(v >> 6) & 63] : '=';
    b += '=';
  }
  return b;
}

std::string base64Decode(const std::string& in) {
  static int8_t rev[256];
  static bool init = false;
  if (!init) {
    std::memset(rev, -1, sizeof rev);
    for (int i = 0; i < 64; ++i) rev[(unsigned char)kB64[i]] = int8_t(i);
    init = true;
  }
  std::string out;
  out.reserve(in.size() / 4 * 3);
  uint32_t acc = 0;
  int bits = 0;
  for (unsigned char c : in) {
    if (c == '=') break;
    int v = rev[c];
    if (v < 0) {
      if (c == '\n' || c == '\r' || c == ' ') continue;
      throw bad("invalid base64");
    }
    acc = (acc << 6) | uint32_t(v);
    bits += 6;
    if (bits >= 8) {
      bits -= 8;
      out += char((acc >> bits) & 0xFF);
    }
  }
  return out;
}

void writeTyped(json::Writer& w, const char* kind, const void* data, size_t bytes) {
  w.beginObject().key("$typed").str(kind);
  if (g_mode == TypedMode::Base64) {
    w.key("base64").str(base64Encode(data, bytes));
  } else {
    std::string& bin = binarySection();
    while (bin.size() % 8) bin += '\0';
    const size_t off = bin.size();
    bin.append(static_cast<const char*>(data), bytes);
    w.key("offset").integer((long long)off).key("byteLength").integer((long long)bytes);
  }
  w.endObject();
}

}  // namespace cadoc
