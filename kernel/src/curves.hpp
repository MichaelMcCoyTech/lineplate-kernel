// HLR output edges -> protocol Curve2D, as the reference kernel emits them
// (kernel/reference/refkernel.py, CurveEmitter; rules in kernel/reference/README.md "Curves").
// SPDX-License-Identifier: LGPL-2.1-only
#pragma once

#include "json.hpp"

#include <TopoDS_Edge.hxx>

#include <array>
#include <map>
#include <string>
#include <vector>

namespace cadoc {

using P2 = std::array<double, 2>;

struct Curve2D {
  enum class Kind { Line, Arc, Ellipse, Bezier, Polyline } kind = Kind::Line;
  P2 a{}, b{};                      // line
  P2 center{};                      // arc, ellipse
  double radius = 0;                // arc
  double major = 0, minor = 0, rotation = 0;  // ellipse
  double start = 0, end = 0;        // arc, ellipse
  bool ccw = true;
  std::vector<P2> points;           // bezier, polyline
  void write(json::Writer& w) const;
  static const char* name(Kind k);
};

class CurveEmitter {
 public:
  CurveEmitter(P2 offset, double polylineDeflection) : ox_(offset[0]), oy_(offset[1]), defl_(polylineDeflection) {}
  /// The edge as a Curve2D, or false if it is degenerate (and dropped).
  bool emit(const TopoDS_Edge& edge, Curve2D& out);
  std::map<std::string, int> stats;
  P2 p2(double x, double y) const { return {x - ox_, y - oy_}; }

 private:
  double ox_, oy_, defl_;
};

/// Joins 2-point segments that share endpoints into polylines (polygonal HLR output).
std::vector<std::vector<P2>> chainSegments(const std::vector<std::array<P2, 2>>& segs, double tol, double maxTurn);

}  // namespace cadoc
