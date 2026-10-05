// HLR output edges -> protocol Curve2D: true conics where OCCT has them, then recognised
// lines, arcs and ellipses (within 1 micron), then cubic Beziers, then polylines.
// A line-by-line port of the reference kernel's CurveEmitter (kernel/reference/refkernel.py).
// SPDX-License-Identifier: LGPL-2.1-only
#include "curves.hpp"

#include <BRepAdaptor_Curve.hxx>
#include <GCPnts_QuasiUniformDeflection.hxx>
#include <GeomConvert_ApproxCurve.hxx>
#include <GeomConvert_BSplineCurveToBezierCurve.hxx>
#include <Geom_BSplineCurve.hxx>
#include <Geom_BezierCurve.hxx>
#include <Standard_Failure.hxx>
#include <TopAbs.hxx>
#include <gp_Circ.hxx>
#include <gp_Elips.hxx>

#include <algorithm>
#include <cmath>
#include <deque>
#include <optional>

namespace cadoc {
namespace {

constexpr double CURVE_TOL = 1e-3;  // view mm: 1 micron at 1:1
constexpr double PI = 3.14159265358979323846;
constexpr double TWO_PI = 2 * PI;

double dist(const P2& a, const P2& b) { return std::hypot(a[0] - b[0], a[1] - b[1]); }
/// Python's % (the result takes the divisor's sign).
double pymod(double a, double m) { return a - m * std::floor(a / m); }

/// startAngle into (-pi, pi]; endAngle keeps the signed sweep.
void normAngles(double& s, double& e) {
  const double k = std::floor((s + PI) / TWO_PI);
  double shift = k * TWO_PI;
  double s2 = s - shift;
  if (s2 <= -PI) {
    s2 += TWO_PI;
    shift -= TWO_PI;
  }
  s = s2;
  e = e - shift;
}

P2 evalConic(const Curve2D& c, double ang) {
  if (c.kind == Curve2D::Kind::Arc) return {c.center[0] + c.radius * std::cos(ang), c.center[1] + c.radius * std::sin(ang)};
  const double x = c.major * std::cos(ang), y = c.minor * std::sin(ang);
  const double cr = std::cos(c.rotation), sr = std::sin(c.rotation);
  return {c.center[0] + x * cr - y * sr, c.center[1] + x * sr + y * cr};
}

/// Unwraps angles; nullopt unless they move strictly one way. Sets ccw.
std::optional<std::vector<double>> unwrapMonotonic(const std::vector<double>& angs, bool& ccw) {
  std::vector<double> un{angs[0]};
  for (size_t i = 1; i < angs.size(); ++i) {
    const double d = pymod(angs[i] - un.back() + PI, TWO_PI) - PI;
    un.push_back(un.back() + d);
  }
  bool allPos = true, allNeg = true;
  for (size_t i = 0; i + 1 < un.size(); ++i) {
    const double st = un[i + 1] - un[i];
    if (!(st > 0)) allPos = false;
    if (!(st < 0)) allNeg = false;
  }
  if (allPos) ccw = true;
  else if (allNeg) ccw = false;
  else return std::nullopt;
  return un;
}

// ---------------------------------------------------------------- small dense linear algebra

using M3 = std::array<std::array<double, 3>, 3>;
using V3 = std::array<double, 3>;

bool solve3(const M3& A, const M3& B, M3& X) {  // A X = B, partial pivoting
  M3 a = A, b = B;
  for (int c = 0; c < 3; ++c) {
    int p = c;
    for (int r = c + 1; r < 3; ++r)
      if (std::abs(a[r][c]) > std::abs(a[p][c])) p = r;
    if (std::abs(a[p][c]) < 1e-300) return false;
    std::swap(a[c], a[p]);
    std::swap(b[c], b[p]);
    for (int r = 0; r < 3; ++r) {
      if (r == c) continue;
      const double f = a[r][c] / a[c][c];
      for (int k = 0; k < 3; ++k) {
        a[r][k] -= f * a[c][k];
        b[r][k] -= f * b[c][k];
      }
    }
  }
  for (int r = 0; r < 3; ++r)
    for (int k = 0; k < 3; ++k) X[r][k] = b[r][k] / a[r][r];
  return true;
}

/// Real eigenvalues of a general 3x3 matrix (characteristic cubic, Newton-polished).
std::vector<double> realEigenvalues3(const M3& m) {
  const double tr = m[0][0] + m[1][1] + m[2][2];
  const double c1 = m[0][0] * m[1][1] - m[0][1] * m[1][0] + m[0][0] * m[2][2] - m[0][2] * m[2][0] + m[1][1] * m[2][2] -
                    m[1][2] * m[2][1];
  const double det = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
                     m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
  // l^3 + a l^2 + b l + c = 0
  const double a = -tr, b = c1, c = -det;
  const double q = (a * a - 3 * b) / 9, r = (2 * a * a * a - 9 * a * b + 27 * c) / 54;
  std::vector<double> roots;
  if (r * r < q * q * q) {
    const double th = std::acos(std::clamp(r / std::sqrt(q * q * q), -1.0, 1.0));
    const double sq = -2 * std::sqrt(q);
    roots = {sq * std::cos(th / 3) - a / 3, sq * std::cos((th + TWO_PI) / 3) - a / 3, sq * std::cos((th - TWO_PI) / 3) - a / 3};
  } else {
    const double A = -std::copysign(std::cbrt(std::abs(r) + std::sqrt(r * r - q * q * q)), r);
    const double B = A == 0 ? 0 : q / A;
    roots = {A + B - a / 3};
  }
  for (double& x : roots)
    for (int it = 0; it < 3; ++it) {
      const double f = ((x + a) * x + b) * x + c, df = (3 * x + 2 * a) * x + b;
      if (df == 0) break;
      x -= f / df;
    }
  return roots;
}

V3 nullVector(const M3& m, double lam) {
  M3 a = m;
  for (int i = 0; i < 3; ++i) a[i][i] -= lam;
  auto cross = [](const V3& u, const V3& v) -> V3 {
    return {u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]};
  };
  V3 best{0, 0, 0};
  double bn = -1;
  for (auto [i, j] : {std::pair{0, 1}, std::pair{0, 2}, std::pair{1, 2}}) {
    V3 v = cross(a[i], a[j]);
    const double n = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    if (n > bn) {
      bn = n;
      best = v;
    }
  }
  if (bn > 0)
    for (double& x : best) x /= bn;
  return best;
}

// ---------------------------------------------------------------- fits

std::optional<Curve2D> fitArc(const std::vector<P2>& pts) {
  const P2 a = pts[0], m = pts[pts.size() / 2], b = pts.back();
  const double ax = a[0], ay = a[1], bx = m[0], by = m[1], cx = b[0], cy = b[1];
  const double d = 2 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by));
  if (std::abs(d) < 1e-12) return std::nullopt;
  const double ux = ((ax * ax + ay * ay) * (by - cy) + (bx * bx + by * by) * (cy - ay) + (cx * cx + cy * cy) * (ay - by)) / d;
  const double uy = ((ax * ax + ay * ay) * (cx - bx) + (bx * bx + by * by) * (ax - cx) + (cx * cx + cy * cy) * (bx - ax)) / d;
  const P2 u{ux, uy};
  const double r = dist(u, a);
  const double span = std::max({dist(a, m), dist(m, b), dist(a, b)});
  if (r > 1e4 * std::max(span, 1e-9)) return std::nullopt;
  const double tol = std::max(CURVE_TOL, 1e-6 * r);
  for (const auto& p : pts)
    if (std::abs(dist(u, p) - r) > tol) return std::nullopt;
  std::vector<double> angs;
  for (const auto& p : pts) angs.push_back(std::atan2(p[1] - uy, p[0] - ux));
  bool ccw = true;
  auto un = unwrapMonotonic(angs, ccw);
  if (!un) return std::nullopt;
  double s = un->front(), e = un->back();
  normAngles(s, e);
  if (std::abs(e - s) > TWO_PI + 1e-9) return std::nullopt;
  Curve2D c;
  c.kind = Curve2D::Kind::Arc;
  c.center = u;
  c.radius = r;
  c.start = s;
  c.end = e;
  c.ccw = ccw;
  return c;
}

/// Direct least-squares conic fit (Halir and Flusser), accepted only if every sample is within tolerance.
std::optional<Curve2D> fitEllipse(const std::vector<P2>& P) {
  const size_t n = P.size();
  P2 mean{0, 0};
  for (const auto& p : P) {
    mean[0] += p[0];
    mean[1] += p[1];
  }
  mean[0] /= double(n);
  mean[1] /= double(n);
  double scale = 0;
  for (const auto& p : P) scale = std::max({scale, std::abs(p[0] - mean[0]), std::abs(p[1] - mean[1])});
  if (scale < 1e-9) return std::nullopt;
  M3 S1{}, S2{}, S3{};
  for (const auto& p : P) {
    const double x = (p[0] - mean[0]) / scale, y = (p[1] - mean[1]) / scale;
    const V3 d1{x * x, x * y, y * y}, d2{x, y, 1.0};
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j) {
        S1[i][j] += d1[i] * d1[j];
        S2[i][j] += d1[i] * d2[j];
        S3[i][j] += d2[i] * d2[j];
      }
  }
  M3 S2t{};
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) S2t[i][j] = S2[j][i];
  M3 T{};
  if (!solve3(S3, S2t, T)) return std::nullopt;
  for (auto& row : T)
    for (double& x : row) x = -x;
  M3 M{};
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) {
      M[i][j] = S1[i][j];
      for (int k = 0; k < 3; ++k) M[i][j] += S2[i][k] * T[k][j];
    }
  M3 Mp{};
  for (int j = 0; j < 3; ++j) {
    Mp[0][j] = M[2][j] / 2;
    Mp[1][j] = -M[1][j];
    Mp[2][j] = M[0][j] / 2;
  }
  std::optional<V3> a1;
  for (double lam : realEigenvalues3(Mp)) {
    const V3 v = nullVector(Mp, lam);
    if (4 * v[0] * v[2] - v[1] * v[1] > 0) {
      a1 = v;
      break;
    }
  }
  if (!a1) return std::nullopt;
  const double A = (*a1)[0], B = (*a1)[1], C = (*a1)[2];
  double Dq = 0, Eq = 0, F = 0;
  for (int k = 0; k < 3; ++k) {
    Dq += T[0][k] * (*a1)[k];
    Eq += T[1][k] * (*a1)[k];
    F += T[2][k] * (*a1)[k];
  }
  const double det = 4 * A * C - B * B;
  if (det <= 0) return std::nullopt;
  const double x0 = (B * Eq - 2 * C * Dq) / det, y0 = (B * Dq - 2 * A * Eq) / det;
  const double Fc = A * x0 * x0 + B * x0 * y0 + C * y0 * y0 + Dq * x0 + Eq * y0 + F;
  // eigh([[A, B/2], [B/2, C]]): ascending eigenvalues, unit eigenvectors.
  const double h = B / 2, mid = (A + C) / 2, rad = std::hypot((A - C) / 2, h);
  const double lam0 = mid - rad, lam1 = mid + rad;
  auto eigvec = [&](double lam) -> P2 {
    P2 v1{h, lam - A}, v2{lam - C, h};
    P2 v = std::hypot(v1[0], v1[1]) >= std::hypot(v2[0], v2[1]) ? v1 : v2;
    const double nv = std::hypot(v[0], v[1]);
    if (nv < 1e-300) return lam == lam0 ? (A <= C ? P2{1, 0} : P2{0, 1}) : (A <= C ? P2{0, 1} : P2{1, 0});
    return {v[0] / nv, v[1] / nv};
  };
  if (Fc == 0 || (-Fc / lam0) <= 0 || (-Fc / lam1) <= 0) return std::nullopt;
  const double ax0 = std::sqrt(-Fc / lam0), ax1 = std::sqrt(-Fc / lam1);
  double a, b;
  P2 v;
  if (ax0 >= ax1) {
    a = ax0;
    b = ax1;
    v = eigvec(lam0);
  } else {
    a = ax1;
    b = ax0;
    v = eigvec(lam1);
  }
  a *= scale;
  b *= scale;
  const double cx = x0 * scale + mean[0], cy = y0 * scale + mean[1];
  double lo0 = P[0][0], hi0 = P[0][0], lo1 = P[0][1], hi1 = P[0][1];
  for (const auto& p : P) {
    lo0 = std::min(lo0, p[0]);
    hi0 = std::max(hi0, p[0]);
    lo1 = std::min(lo1, p[1]);
    hi1 = std::max(hi1, p[1]);
  }
  const double span = std::max(hi0 - lo0, hi1 - lo1);
  if (a > 1e3 * std::max(span, 1e-9) || b < 1e-9) return std::nullopt;
  const double rot = std::atan2(v[1], v[0]);
  const double cr = std::cos(rot), sr = std::sin(rot);
  std::vector<double> th;
  for (const auto& p : P) {
    const double dx = p[0] - cx, dy = p[1] - cy;
    th.push_back(std::atan2((-dx * sr + dy * cr) / b, (dx * cr + dy * sr) / a));
  }
  bool ccw = true;
  auto un = unwrapMonotonic(th, ccw);
  if (!un) return std::nullopt;
  double s = un->front(), e = un->back();
  normAngles(s, e);
  if (std::abs(e - s) > TWO_PI + 1e-9) return std::nullopt;
  Curve2D c;
  c.kind = Curve2D::Kind::Ellipse;
  c.center = {cx, cy};
  c.major = a;
  c.minor = b;
  c.rotation = rot;
  c.start = s;
  c.end = e;
  c.ccw = ccw;
  const double tol = std::max(CURVE_TOL, 1e-6 * a);
  for (size_t i = 0; i < n; ++i)
    if (dist(evalConic(c, (*un)[i] - ((*un)[0] - s)), P[i]) > tol) return std::nullopt;
  return c;
}

}  // namespace

const char* Curve2D::name(Kind k) {
  switch (k) {
    case Kind::Line: return "line";
    case Kind::Arc: return "arc";
    case Kind::Ellipse: return "ellipse";
    case Kind::Bezier: return "bezier";
    default: return "polyline";
  }
}

void Curve2D::write(json::Writer& w) const {
  auto pt = [&](const P2& p) { w.beginArray().num(p[0]).num(p[1]).endArray(); };
  w.beginObject().key("kind").str(name(kind));
  switch (kind) {
    case Kind::Line:
      w.key("a"), pt(a);
      w.key("b"), pt(b);
      break;
    case Kind::Arc:
      w.key("center"), pt(center);
      w.key("radius").num(radius).key("startAngle").num(start).key("endAngle").num(end).key("ccw").boolean(ccw);
      break;
    case Kind::Ellipse:
      w.key("center"), pt(center);
      w.key("majorRadius").num(major).key("minorRadius").num(minor).key("rotation").num(rotation);
      w.key("startAngle").num(start).key("endAngle").num(end).key("ccw").boolean(ccw);
      break;
    default:
      w.key("points").beginArray();
      for (const auto& p : points) pt(p);
      w.endArray();
  }
  w.endObject();
}

bool CurveEmitter::emit(const TopoDS_Edge& edge, Curve2D& c) {
  BRepAdaptor_Curve ad(edge);
  const double f = ad.FirstParameter(), l = ad.LastParameter();
  if (!(std::isfinite(f) && std::isfinite(l)) || l - f <= 1e-12) return false;
  const bool rev = edge.Orientation() == TopAbs_REVERSED;
  auto at = [&](double t) {
    gp_Pnt p = ad.Value(t);
    return p2(p.X(), p.Y());
  };
  auto samples = [&](int n, bool reverse) {
    std::vector<P2> pts;
    for (int i = 0; i < n; ++i) pts.push_back(at(f + (l - f) * i / (n - 1)));
    if (reverse) std::reverse(pts.begin(), pts.end());
    return pts;
  };
  /// Re-evaluates start, middle and end from the emitted parameters against the real edge.
  auto check = [&](const Curve2D& cc) {
    const double s = cc.start, e = cc.end;
    if ((cc.ccw && e < s - 1e-12) || (!cc.ccw && e > s + 1e-12)) return false;
    std::vector<P2> ps{at(f), at(0.5 * (f + l)), at(l)};
    if (rev) std::reverse(ps.begin(), ps.end());
    const double ts[3] = {0.0, 0.5, 1.0};
    for (int i = 0; i < 3; ++i)
      if (dist(evalConic(cc, s + ts[i] * (e - s)), ps[size_t(i)]) > 10 * CURVE_TOL) return false;
    return true;
  };

  const GeomAbs_CurveType typ = ad.GetType();
  bool done = false;
  if (typ == GeomAbs_Line) {
    P2 a = at(f), b = at(l);
    if (rev) std::swap(a, b);
    if (dist(a, b) < 1e-9) return false;
    c = Curve2D{};
    c.kind = Curve2D::Kind::Line;
    c.a = a;
    c.b = b;
    done = true;
  } else if (typ == GeomAbs_Circle) {
    const gp_Circ circ = ad.Circle();
    const gp_Dir xd = circ.XAxis().Direction();
    const double phi0 = std::atan2(xd.Y(), xd.X());
    double s, e;
    bool ccw;
    if (circ.Axis().Direction().Z() >= 0) s = phi0 + f, e = phi0 + l, ccw = true;
    else s = phi0 - f, e = phi0 - l, ccw = false;
    if (rev) std::swap(s, e), ccw = !ccw;
    normAngles(s, e);
    Curve2D arc;
    arc.kind = Curve2D::Kind::Arc;
    arc.center = p2(circ.Location().X(), circ.Location().Y());
    arc.radius = circ.Radius();
    arc.start = s;
    arc.end = e;
    arc.ccw = ccw;
    if (check(arc)) c = arc, done = true;
  } else if (typ == GeomAbs_Ellipse) {
    const gp_Elips el = ad.Ellipse();
    const gp_Dir xd = el.XAxis().Direction();
    double s, e;
    bool ccw;
    if (el.Axis().Direction().Z() >= 0) s = f, e = l, ccw = true;
    else s = -f, e = -l, ccw = false;
    if (rev) std::swap(s, e), ccw = !ccw;
    normAngles(s, e);
    Curve2D ell;
    ell.kind = Curve2D::Kind::Ellipse;
    ell.center = p2(el.Location().X(), el.Location().Y());
    ell.major = el.MajorRadius();
    ell.minor = el.MinorRadius();
    ell.rotation = std::atan2(xd.Y(), xd.X());
    ell.start = s;
    ell.end = e;
    ell.ccw = ccw;
    if (check(ell)) c = ell, done = true;
  }
  if (!done) {
    // Everything else: recognise lines, arcs and ellipses, else cubic Beziers, else a polyline.
    const std::vector<P2> pts = samples(33, rev);
    const P2 a = pts.front(), b = pts.back();
    const double chord = dist(a, b);
    if (chord > 1e-9) {
      const double ux = (b[0] - a[0]) / chord, uy = (b[1] - a[1]) / chord;
      bool collinear = true;
      for (const auto& p : pts)
        if (std::abs((p[0] - a[0]) * uy - (p[1] - a[1]) * ux) > CURVE_TOL) collinear = false;
      if (collinear) {
        // Edge-on circles come out of HLR as degenerate B-splines that double back.
        double lo = INFINITY, hi = -INFINITY;
        for (const auto& p : pts) {
          const double t = (p[0] - a[0]) * ux + (p[1] - a[1]) * uy;
          lo = std::min(lo, t);
          hi = std::max(hi, t);
        }
        if (lo >= -CURVE_TOL && hi <= chord + CURVE_TOL) {
          ++stats["recognised"];
          c = Curve2D{};
          c.kind = Curve2D::Kind::Line;
          c.a = a;
          c.b = b;
          done = true;
        }
      }
    }
    if (!done) {
      if (auto arc = fitArc(pts)) {
        ++stats["recognised"];
        c = *arc;
        done = true;
      }
    }
    if (!done) {
      // Many exporters write circles as rational B-splines; seen at an angle they are ellipses.
      if (auto ell = fitEllipse(samples(49, rev))) {
        ++stats["recognised"];
        c = *ell;
        done = true;
      }
    }
    if (!done) {
      try {
        occ::handle<Adaptor3d_Curve> h = new BRepAdaptor_Curve(edge);
        GeomConvert_ApproxCurve approx(h, CURVE_TOL, GeomAbs_C1, 200, 3);
        if (approx.HasResult() && approx.MaxError() <= 5 * CURVE_TOL) {
          occ::handle<Geom_BSplineCurve> bs = approx.Curve();
          if (!bs->IsRational()) {
            if (bs->Degree() < 3) bs->IncreaseDegree(3);
            GeomConvert_BSplineCurveToBezierCurve conv(bs);
            std::vector<P2> poles;
            bool ok = true;
            for (int i = 1; i <= conv.NbArcs() && ok; ++i) {
              occ::handle<Geom_BezierCurve> arc = conv.Arc(i);
              if (arc->Degree() != 3) {
                ok = false;
                break;
              }
              for (int k = poles.empty() ? 1 : 2; k <= 4; ++k) {
                gp_Pnt q = arc->Pole(k);
                poles.push_back(p2(q.X(), q.Y()));
              }
            }
            if (ok && poles.size() >= 4) {
              if (rev) std::reverse(poles.begin(), poles.end());
              c = Curve2D{};
              c.kind = Curve2D::Kind::Bezier;
              c.points = std::move(poles);
              done = true;
            }
          }
        }
      } catch (const Standard_Failure&) {
        // fall through to the polyline
      }
    }
    if (!done) {
      c = Curve2D{};
      c.kind = Curve2D::Kind::Polyline;
      GCPnts_QuasiUniformDeflection q(ad, defl_, f, l);
      if (q.IsDone() && q.NbPoints() >= 2) {
        for (int i = 1; i <= q.NbPoints(); ++i) {
          gp_Pnt p = q.Value(i);
          c.points.push_back(p2(p.X(), p.Y()));
        }
        if (rev) std::reverse(c.points.begin(), c.points.end());
      } else {
        c.points = samples(33, rev);
      }
    }
  }
  ++stats[Curve2D::name(c.kind)];
  return true;
}

// ---------------------------------------------------------------- polygonal HLR: chaining

std::vector<std::vector<P2>> chainSegments(const std::vector<std::array<P2, 2>>& segs, double tol, double maxTurn) {
  std::vector<std::vector<P2>> chains;
  if (segs.empty()) return chains;
  std::map<std::pair<long long, long long>, std::vector<int>> grid;
  std::vector<P2> reps;
  auto cluster = [&](const P2& p) {
    const long long cx = (long long)std::floor(p[0] / tol), cy = (long long)std::floor(p[1] / tol);
    for (int dx = -1; dx <= 1; ++dx)
      for (int dy = -1; dy <= 1; ++dy) {
        auto it = grid.find({cx + dx, cy + dy});
        if (it == grid.end()) continue;
        for (int c : it->second)
          if (dist(reps[size_t(c)], p) <= tol) return c;
      }
    reps.push_back(p);
    grid[{cx, cy}].push_back(int(reps.size()) - 1);
    return int(reps.size()) - 1;
  };
  std::vector<std::pair<int, int>> ends;
  for (const auto& s : segs) {
    const int ca = cluster(s[0]);
    const int cb = cluster(s[1]);
    ends.push_back({ca, cb});
  }
  std::map<int, std::vector<int>> at;
  for (size_t i = 0; i < ends.size(); ++i) {
    if (ends[i].first == ends[i].second) continue;
    at[ends[i].first].push_back(int(i));
    at[ends[i].second].push_back(int(i));
  }
  std::vector<bool> used(segs.size());
  for (size_t i = 0; i < segs.size(); ++i) used[i] = ends[i].first == ends[i].second;
  auto turn = [&](int a, int b, int c) {
    const P2 u{reps[size_t(b)][0] - reps[size_t(a)][0], reps[size_t(b)][1] - reps[size_t(a)][1]};
    const P2 v{reps[size_t(c)][0] - reps[size_t(b)][0], reps[size_t(c)][1] - reps[size_t(b)][1]};
    return std::abs(std::atan2(u[0] * v[1] - u[1] * v[0], u[0] * v[0] + u[1] * v[1]));
  };
  for (size_t i = 0; i < segs.size(); ++i) {
    if (used[i]) continue;
    used[i] = true;
    std::deque<int> cs{ends[i].first, ends[i].second};
    for (bool tail : {true, false}) {
      while (true) {
        const int tip = tail ? cs.back() : cs.front();
        auto it = at.find(tip);
        if (it == at.end() || it->second.size() != 2) break;
        int nxt = -1;
        for (int j : it->second)
          if (!used[size_t(j)]) {
            nxt = j;
            break;
          }
        if (nxt < 0) break;
        const int far = ends[size_t(nxt)].first == tip ? ends[size_t(nxt)].second : ends[size_t(nxt)].first;
        const int prev = tail ? cs[cs.size() - 2] : cs[1];
        if (turn(prev, tip, far) > maxTurn) break;
        used[size_t(nxt)] = true;
        if (tail) cs.push_back(far);
        else cs.push_front(far);
        if (cs.front() == cs.back()) break;
      }
      if (cs.front() == cs.back()) break;
    }
    std::vector<P2> chain;
    for (int c : cs) chain.push_back(reps[size_t(c)]);
    chains.push_back(std::move(chain));
  }
  return chains;
}

}  // namespace cadoc
