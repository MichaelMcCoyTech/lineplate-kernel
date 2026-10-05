// "mesh": BRepMesh triangulations for the viewer (protocol.ts MeshRequest -> MeshResult),
// as the reference kernel's do_mesh: the previous triangulation is cleaned first, so the
// result is deterministic; vertices are per face; winding and normals point out of a solid;
// faceOfTriangle is the 0-based face index in TopExp::MapShapes order.
// SPDX-License-Identifier: LGPL-2.1-only
#include "session.hpp"

namespace cadoc {

std::string doMesh(const json::Value& req, json::Writer& timing) {
  const json::Value* ids = req.get("shapeIds");
  if (!ids || ids->kind != json::Value::Kind::Array) throw bad("mesh.shapeIds must be an array");
  double lin = 0, ang = 0;
  try {
    if (auto* v = req.get("linearDeflection")) lin = v->num();
    if (auto* v = req.get("angularDeflection")) ang = v->num();
  } catch (const std::exception&) {
    throw bad("mesh.linearDeflection and angularDeflection must be numbers");
  }
  if (!(lin > 0 && ang > 0)) throw bad("mesh.linearDeflection and angularDeflection must be > 0");
  json::Writer w;
  w.beginObject().key("type").str("mesh:ok").key("meshes").beginArray();
  long long nTri = 0;
  for (const auto& sid : ids->a) {
    ShapeRec& rec = getShape(&sid);
    const TriMesh m = triMesh(rec, lin, ang);
    std::vector<float> pos(m.pos.begin(), m.pos.end()), nrm(m.nrm.begin(), m.nrm.end());
    w.beginObject().key("shapeId").str(rec.id);
    w.key("positions");
    writeTyped(w, "Float32Array", pos.data(), pos.size() * sizeof(float));
    w.key("normals");
    writeTyped(w, "Float32Array", nrm.data(), nrm.size() * sizeof(float));
    w.key("indices");
    writeTyped(w, "Uint32Array", m.idx.data(), m.idx.size() * sizeof(uint32_t));
    w.key("faceOfTriangle");
    writeTyped(w, "Uint32Array", m.faceOfTriangle.data(), m.faceOfTriangle.size() * sizeof(uint32_t));
    w.endObject();
    nTri += (long long)m.faceOfTriangle.size();
  }
  w.endArray().endObject();
  timing.key("triangles").integer(nTri);
  return std::move(w.out);
}

}  // namespace cadoc
