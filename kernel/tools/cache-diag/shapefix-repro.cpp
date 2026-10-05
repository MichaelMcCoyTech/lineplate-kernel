// Minimal reproduction for OCCT patch 0001 (kernel/build/patches/UPSTREAM.md):
// ShapeFix_IntersectionTool::FixSelfIntersectWire is cubic in a wire's edge count. A planar face
// bounded by a regular polygon of N edges (no self-intersection at all) goes through
// ShapeFix_Wire::FixSelfIntersection with the non-adjacent check on; the time per doubling of N
// shows the order. Unpatched, each Edge(num1)/Edge(num2) pair walks the wire's linked sequence
// between the two indices inside the double loop.
// SPDX-License-Identifier: LGPL-2.1-only
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <ShapeBuild_ReShape.hxx>
#include <ShapeFix_Wire.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Wire.hxx>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <string>

int main() {
  double prev = 0;
  for (int n : {500, 1000, 2000, 4000}) {
    BRepBuilderAPI_MakePolygon poly;
    for (int i = 0; i < n; ++i) {
      const double a = 2 * M_PI * i / n;
      poly.Add(gp_Pnt(100 * std::cos(a), 100 * std::sin(a), 0));
    }
    poly.Close();
    const TopoDS_Wire wire = poly.Wire();
    const TopoDS_Face face = BRepBuilderAPI_MakeFace(wire, true).Face();
    occ::handle<ShapeFix_Wire> fix = new ShapeFix_Wire(wire, face, 1e-7);
    fix->FixNonAdjacentIntersectingEdgesMode() = 1;
    fix->SetContext(new ShapeBuild_ReShape);  // without one, FixSelfIntersectWire returns at once
    const auto t0 = std::chrono::steady_clock::now();
    fix->FixSelfIntersection();
    const double s = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    std::printf("%5d edges: %8.3f s%s\n", n, s, prev > 0 ? (std::string(" (x") + std::to_string(s / prev).substr(0, 4) + " for 2x edges)").c_str() : "");
    prev = s;
  }
  return 0;
}
