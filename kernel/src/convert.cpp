// "stlToStep": an STL mesh -> a STEP file of the same shape (protocol.ts StlToStepRequest).
//
// An STL file is triangles and nothing else, so the result is faceted: every triangle becomes a
// planar face, triangles that lie in one plane are merged back into a single face, and a closed
// piece of mesh becomes a solid. Curved surfaces stay as many small flat faces; nothing here
// fits cylinders or splines, and the protocol says so to the caller (`faceted: true`).
//
// Steps: RWStl reads the triangles (binary or ASCII, coincident nodes merged); each triangle is
// built on shared vertices and edges, so neighbouring faces are already connected (no sewing);
// ShapeFix_Shell orients the faces and splits the mesh into its connected shells; each closed
// shell becomes a solid; ShapeUpgrade_UnifySameDomain merges coplanar faces; STEPControl_Writer
// writes AP214 in millimetres. Not part of the reference kernel.
// SPDX-License-Identifier: LGPL-2.1-only
#include "kernel.hpp"
#include "session.hpp"

#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRep_Builder.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <Interface_Static.hxx>
#include <NCollection_IndexedDataMap.hxx>
#include <Poly_Triangulation.hxx>
#include <RWStl.hxx>
#include <STEPControl_Writer.hxx>
#include <ShapeFix_Shell.hxx>
#include <ShapeFix_Solid.hxx>
#include <ShapeUpgrade_UnifySameDomain.hxx>
#include <Standard_Failure.hxx>
#include <TopAbs.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_ShapeMapHasher.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Shell.hxx>
#include <TopoDS_Vertex.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <cstdint>
#include <istream>
#include <map>
#include <sstream>
#include <utility>
#include <vector>

namespace cadoc {
namespace {

/// Above this many triangles the conversion is refused: each triangle is a B-rep face, and the
/// merge of coplanar faces gets slow well before the browser runs out of memory.
constexpr int kMaxTriangles = 200000;

int countOf(const TopoDS_Shape& s, TopAbs_ShapeEnum t) {
  int n = 0;
  for (TopExp_Explorer e(s, t); e.More(); e.Next()) ++n;
  return n;
}

/// Edges used by one face only: the mesh has a hole there.
int freeEdges(const TopoDS_Shape& shell) {
  NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>, TopTools_ShapeMapHasher> m;
  TopExp::MapShapesAndAncestors(shell, TopAbs_EDGE, TopAbs_FACE, m);
  int n = 0;
  for (int i = 1; i <= m.Extent(); ++i)
    if (m.FindFromIndex(i).Extent() < 2) ++n;
  return n;
}

}  // namespace

std::string doStlToStep(const json::Value& req, const uint8_t* bytes, size_t len, json::Writer& timing) {
  if (len == 0) throw KernelError("read-failed", "the STL file is empty");
  const json::Value* nameV = req.get("name");
  const std::string name = nameV && nameV->kind == json::Value::Kind::String && !nameV->s.empty() ? nameV->s : "converted";
  const json::Value* mergeV = req.get("mergePlanar");
  const bool mergePlanar = !(mergeV && mergeV->kind == json::Value::Kind::Bool && !mergeV->b);

  auto t0 = Clock::now();
  occ::handle<Poly_Triangulation> tri;
  try {
    MemBuf buf(bytes, len);
    std::istream in(&buf);
    tri = RWStl::ReadStream(in);
  } catch (const Standard_Failure& e) {
    throw KernelError("read-failed", std::string("STL read failed: ") + e.what());
  }
  if (tri.IsNull() || tri->NbTriangles() == 0) throw KernelError("read-failed", "the file has no triangles: it isn't an STL mesh, or it is empty");
  const int nTri = tri->NbTriangles();
  if (nTri > kMaxTriangles)
    throw KernelError("too-large", "the mesh has " + std::to_string(nTri) + " triangles; the converter takes up to " + std::to_string(kMaxTriangles));
  const double readMs = msSince(t0);

  // One vertex per node and one edge per node pair, shared by the triangles that use them.
  auto t1 = Clock::now();
  std::vector<TopoDS_Vertex> verts(tri->NbNodes() + 1);
  for (int i = 1; i <= tri->NbNodes(); ++i) verts[i] = BRepBuilderAPI_MakeVertex(tri->Node(i));
  std::map<std::pair<int, int>, TopoDS_Edge> edges;
  auto edgeOf = [&](int a, int b) -> TopoDS_Edge {
    const auto key = a < b ? std::make_pair(a, b) : std::make_pair(b, a);
    auto it = edges.find(key);
    if (it == edges.end()) it = edges.emplace(key, BRepBuilderAPI_MakeEdge(verts[key.first], verts[key.second]).Edge()).first;
    // The stored edge runs first -> second; a triangle that walks it the other way uses it reversed.
    return a == key.first ? it->second : TopoDS::Edge(it->second.Reversed());
  };

  BRep_Builder bb;
  TopoDS_Shell shell;
  bb.MakeShell(shell);
  int degenerate = 0, faces = 0;
  for (int t = 1; t <= nTri; ++t) {
    int a, b, c;
    tri->Triangle(t).Get(a, b, c);
    const gp_Pnt pa = tri->Node(a), pb = tri->Node(b), pc = tri->Node(c);
    if (a == b || b == c || a == c || gp_Vec(pa, pb).Crossed(gp_Vec(pa, pc)).SquareMagnitude() < 1e-18) {
      ++degenerate;
      continue;
    }
    BRepBuilderAPI_MakeWire mw;
    mw.Add(edgeOf(a, b));
    mw.Add(edgeOf(b, c));
    mw.Add(edgeOf(c, a));
    if (!mw.IsDone()) {
      ++degenerate;
      continue;
    }
    BRepBuilderAPI_MakeFace mf(mw.Wire(), true);
    if (!mf.IsDone()) {
      ++degenerate;
      continue;
    }
    bb.Add(shell, mf.Face());
    ++faces;
  }
  if (faces == 0) throw KernelError("read-failed", "every triangle in the file is degenerate (zero area)");
  const double buildMs = msSince(t1);

  // Orient consistently and split into connected shells; close each closed one into a solid.
  auto t2 = Clock::now();
  ShapeFix_Shell fixShell(shell);
  fixShell.FixFaceOrientation(shell, true, false);
  const TopoDS_Shape shells = fixShell.Shape();
  TopoDS_Compound result;
  bb.MakeCompound(result);
  int solids = 0, openShells = 0, holeEdges = 0;
  for (TopExp_Explorer e(shells, TopAbs_SHELL); e.More(); e.Next()) {
    const TopoDS_Shell sh = TopoDS::Shell(e.Current());
    const int open = freeEdges(sh);
    if (open == 0) {
      ShapeFix_Solid fixSolid;
      const TopoDS_Solid solid = fixSolid.SolidFromShell(sh);
      if (!solid.IsNull()) {
        bb.Add(result, solid);
        ++solids;
        continue;
      }
    }
    holeEdges += open;
    ++openShells;
    bb.Add(result, sh);
  }
  TopoDS_Shape shape = result;
  const double fixMs = msSince(t2);

  auto t3 = Clock::now();
  if (mergePlanar) {
    try {
      ShapeUpgrade_UnifySameDomain unify(shape, true, true, false);
      unify.Build();
      if (!unify.Shape().IsNull()) shape = unify.Shape();
    } catch (const Standard_Failure&) {
      // Merging is a nicety: the unmerged shape is still a correct STEP.
    }
  }
  const double mergeMs = msSince(t3);

  auto t4 = Clock::now();
  std::ostringstream os(std::ios::out | std::ios::binary);
  {
    // The writer's constructor registers the STEP parameters; set them after it, or they are ignored.
    STEPControl_Writer w;
    Interface_Static::SetCVal("write.step.unit", "MM");
    Interface_Static::SetCVal("write.step.product.name", name.c_str());
    if (w.Transfer(shape, STEPControl_AsIs) != IFSelect_RetDone) throw KernelError("write-failed", "the STEP writer couldn't take the shape");
    if (w.WriteStream(os) != IFSelect_RetDone) throw KernelError("write-failed", "the STEP writer failed");
  }
  const std::string step = os.str();
  const double writeMs = msSince(t4);

  json::Writer out;
  out.beginObject().key("type").str("stlToStep:ok");
  out.key("step");
  writeTyped(out, "Uint8Array", step.data(), step.size());
  out.key("triangles").integer(nTri);
  out.key("degenerate").integer(degenerate);
  out.key("faces").integer(countOf(shape, TopAbs_FACE));
  out.key("solids").integer(solids);
  out.key("openShells").integer(openShells);
  out.key("openEdges").integer(holeEdges);
  out.key("faceted").boolean(true);
  out.endObject();
  timing.key("readMs").num(readMs).key("buildMs").num(buildMs).key("fixMs").num(fixMs).key("mergeMs").num(mergeMs).key("writeMs").num(writeMs);
  return std::move(out.out);
}

}  // namespace cadoc
