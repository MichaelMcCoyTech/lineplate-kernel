// Minimal reproduction for OCCT patch 0004 (kernel/build/patches/UPSTREAM.md): a BinTools
// write-read round trip does not give back the shape it wrote. Builds an edge on a circle and a
// face on a plane, both with oblique axes, places the face with a rotation, writes the compound
// with BinTools and reads it back, and compares the circle's and plane's frames and the
// location's transformation bit for bit. Prints one line per check; exit status 1 if any differ.
//
//   build.sh repro   (or: g++ -std=c++17 bintools-repro.cpp -I<occt>/include/opencascade
//                     -L<occt>/lib -lTKBRep -lTKG3d -lTKG2d -lTKMath -lTKernel -pthread -ldl)
// SPDX-License-Identifier: LGPL-2.1-only
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRep_Builder.hxx>
#include <BRep_Tool.hxx>
#include <BinTools.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <Geom_Circle.hxx>
#include <Geom_Plane.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <gp_Ax2.hxx>
#include <gp_Pln.hxx>
#include <gp_Trsf.hxx>

#include <cstdio>
#include <cstring>
#include <sstream>

static bool same(double a, double b) { return std::memcmp(&a, &b, sizeof a) == 0; }
static bool sameDir(const gp_Dir& a, const gp_Dir& b) { return same(a.X(), b.X()) && same(a.Y(), b.Y()) && same(a.Z(), b.Z()); }

int main() {
  // Oblique axes: their components are not exactly representable after normalisation.
  const gp_Ax2 ax(gp_Pnt(1.5, -2.25, 3.0), gp_Dir(0.3, 0.2, 1.0), gp_Dir(1.0, -0.7, 0.1));
  const TopoDS_Edge edge = BRepBuilderAPI_MakeEdge(new Geom_Circle(ax, 17.0)).Edge();
  const TopoDS_Face face = BRepBuilderAPI_MakeFace(gp_Pln(gp_Ax3(gp_Pnt(0, 0, 0), gp_Dir(-0.4, 0.9, 0.35), gp_Dir(0.9, 0.4, 0.0))), -5, 5, -5, 5).Face();
  gp_Trsf rot;
  rot.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(0.3, 0.2, 1.0)), 0.5759586531581288);
  rot.SetTranslationPart(gp_Vec(30, 5, 2));
  BRep_Builder b;
  TopoDS_Compound c;
  b.MakeCompound(c);
  b.Add(c, edge);
  b.Add(c, face.Moved(TopLoc_Location(rot)));

  // The newest format this OCCT writes (5 with the patch, 4 without).
  std::ostringstream os(std::ios::out | std::ios::binary);
  BinTools::Write(c, os, false, false, BinTools_FormatVersion(BinTools_FormatVersion_UPPER));
  std::istringstream is(os.str(), std::ios::in | std::ios::binary);
  TopoDS_Shape back;
  BinTools::Read(back, is);

  TopExp_Explorer e0(c, TopAbs_EDGE), e1(back, TopAbs_EDGE);
  double f, l;
  const auto c0 = occ::down_cast<Geom_Circle>(BRep_Tool::Curve(TopoDS::Edge(e0.Current()), f, l));
  const auto c1 = occ::down_cast<Geom_Circle>(BRep_Tool::Curve(TopoDS::Edge(e1.Current()), f, l));
  TopExp_Explorer f0(c, TopAbs_FACE), f1(back, TopAbs_FACE);
  const auto p0 = occ::down_cast<Geom_Plane>(BRep_Tool::Surface(TopoDS::Face(f0.Current())));
  const auto p1 = occ::down_cast<Geom_Plane>(BRep_Tool::Surface(TopoDS::Face(f1.Current())));
  const gp_Trsf& t0 = f0.Current().Location().Transformation();
  const gp_Trsf& t1 = f1.Current().Location().Transformation();

  bool ok = true;
  auto report = [&](const char* what, bool equal) {
    std::printf("%-44s %s\n", what, equal ? "identical" : "DIFFERS");
    ok = ok && equal;
  };
  std::printf("BinTools format %d\n", int(BinTools_FormatVersion_UPPER));
  report("circle axis direction", sameDir(c0->Position().Direction(), c1->Position().Direction()));
  report("circle X direction", sameDir(c0->Position().XDirection(), c1->Position().XDirection()));
  report("circle Y direction", sameDir(c0->Position().YDirection(), c1->Position().YDirection()));
  report("plane normal", sameDir(p0->Position().Direction(), p1->Position().Direction()));
  report("plane X direction", sameDir(p0->Position().XDirection(), p1->Position().XDirection()));
  report("plane Y direction", sameDir(p0->Position().YDirection(), p1->Position().YDirection()));
  bool m = t0.Form() == t1.Form() && same(t0.ScaleFactor(), t1.ScaleFactor());
  for (int r = 1; r <= 3; ++r) {
    m = m && same(t0.TranslationPart().Coord(r), t1.TranslationPart().Coord(r));
    for (int k = 1; k <= 3; ++k) m = m && same(t0.HVectorialPart()(r, k), t1.HVectorialPart()(r, k));
  }
  std::printf("location form %d -> %d, scale factor %.17g -> %.17g\n", int(t0.Form()), int(t1.Form()), t0.ScaleFactor(), t1.ScaleFactor());
  report("location (form, scale, matrix, translation)", m);
  return ok ? 0 : 1;
}
