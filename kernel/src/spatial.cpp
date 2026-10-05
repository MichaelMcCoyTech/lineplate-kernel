// "clearance" and "sweep": the spatial queries behind auto-explode (protocol.ts), with the
// reference kernel's semantics (kernel/reference/refkernel.py, do_clearance and do_sweep;
// kernel/reference/README.md). Sweep is mesh-based with an exact boolean confirming each hit.
// SPDX-License-Identifier: LGPL-2.1-only
#include "session.hpp"

#include <BRepAlgoAPI_Common.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <BRepGProp.hxx>
#include <Extrema_ExtFlag.hxx>
#include <GProp_GProps.hxx>
#include <TopLoc_Location.hxx>
#include <gp_Vec.hxx>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <queue>
#include <tuple>

namespace cadoc {
namespace {

using V = std::array<double, 3>;
V sub(const V& a, const V& b) { return {a[0] - b[0], a[1] - b[1], a[2] - b[2]}; }
V cross(const V& a, const V& b) { return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]}; }
double dot(const V& a, const V& b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
double norm(const V& a) { return std::sqrt(dot(a, a)); }

void log(const std::string& s) { std::fprintf(stderr, "[cadoc-kernel] %s\n", s.c_str()); }

double commonVolume(const TopoDS_Shape& a, const TopoDS_Shape& b) {
  BRepAlgoAPI_Common op(a, b);
  if (!op.IsDone()) {
    // A failed boolean must not read as "no interference": err on the side of contact.
    log("BRepAlgoAPI_Common failed; treating the pair as interfering");
    return INFINITY;
  }
  GProp_GProps g;
  BRepGProp::VolumeProperties(op.Shape(), g);
  return std::abs(g.Mass());
}

/// Common volume below this is boolean noise from coincident faces, not interference.
double volumeEps(const ShapeRec& a, const ShapeRec& b, double tol) {
  return std::max({1e-6 * std::min(a.volume, b.volume), tol * tol * tol, 1e-9});
}

bool penetrates(const TopoDS_Shape& a, const ShapeRec& ra, const TopoDS_Shape& b, const ShapeRec& rb, double tol) {
  if (!(ra.solid && rb.solid)) return false;
  return commonVolume(a, b) > volumeEps(ra, rb, tol);
}

double numField(const json::Value& req, const char* k, double def, const char* what) {
  const json::Value* v = req.get(k);
  if (!v) return def;
  try {
    return v->num();
  } catch (const std::exception&) {
    throw bad(std::string(what) + " must be a number");
  }
}

// ---------------------------------------------------------------- sweep

constexpr double SWEEP_GRAZE = 1e-3;  // |cos| between a facet normal and the motion below which it is "parallel"
constexpr int SWEEP_MAX_CONFIRM = 25;

struct Tri {
  V p[3];
};

struct SweepBody {
  double defl;
  std::vector<Tri> tris;
  std::vector<double> facing;  // cos(normal, d)
};

/// Cached triangle mesh for sweeps: outward winding, deflection 0.1% of the part's size.
void sweepMesh(ShapeRec& rec) {
  const double defl = std::max(1e-3, 1e-3 * rec.diag);
  if (rec.sweepDefl == defl) return;
  TriMesh m = triMesh(rec, defl, 0.3);
  std::vector<int> tri(m.idx.begin(), m.idx.end());
  if (!tri.empty()) {
    double signedV = 0;
    for (size_t i = 0; i < tri.size(); i += 3) {
      const double* p0 = &m.pos[size_t(tri[i]) * 3];
      const double* p1 = &m.pos[size_t(tri[i + 1]) * 3];
      const double* p2 = &m.pos[size_t(tri[i + 2]) * 3];
      const V c = cross({p1[0], p1[1], p1[2]}, {p2[0], p2[1], p2[2]});
      signedV += p0[0] * c[0] + p0[1] * c[1] + p0[2] * c[2];
    }
    signedV /= 6;
    if (signedV < 0) {  // inside-out shell; flip so the facing tests stay right
      log("shape " + rec.id + ": mesh signed volume < 0, flipping winding");
      for (size_t i = 0; i < tri.size(); i += 3) std::swap(tri[i + 1], tri[i + 2]);
    }
  }
  rec.sweepDefl = defl;
  rec.sweepPos = std::move(m.pos);
  rec.sweepTri = std::move(tri);
}

SweepBody sweepBody(const Placed& pl, const V& d) {
  sweepMesh(*pl.rec);
  const ShapeRec& r = *pl.rec;
  double R[3][3], t[3];
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) R[i][j] = pl.trsf.Value(i + 1, j + 1);
    t[i] = pl.trsf.Value(i + 1, 4);
  }
  SweepBody b{r.sweepDefl, {}, {}};
  auto world = [&](int vi) -> V {
    const double* p = &r.sweepPos[size_t(vi) * 3];
    V out;
    for (int i = 0; i < 3; ++i) out[size_t(i)] = p[0] * R[i][0] + p[1] * R[i][1] + p[2] * R[i][2] + t[i];
    return out;
  };
  for (size_t i = 0; i < r.sweepTri.size(); i += 3) {
    Tri tr{{world(r.sweepTri[i]), world(r.sweepTri[i + 1]), world(r.sweepTri[i + 2])}};
    const V n = cross(sub(tr.p[1], tr.p[0]), sub(tr.p[2], tr.p[0]));
    const double ln = norm(n);
    if (!(ln > 1e-14)) continue;
    b.tris.push_back(tr);
    b.facing.push_back(dot(n, d) / ln);
  }
  return b;
}

/// Triangle vertices plus points along every triangle edge at `spacing`, without duplicates.
std::vector<V> samplePoints(const std::vector<Tri>& tris, double spacing) {
  std::vector<V> pts;
  for (const auto& t : tris)
    for (const auto& p : t.p) pts.push_back(p);
  for (int e = 0; e < 3; ++e)
    for (const auto& t : tris) {
      const V& a = t.p[e];
      const V& b = t.p[(e + 1) % 3];
      const double ln = norm(sub(b, a));
      long long k = (long long)std::ceil(ln / spacing);
      k = std::min<long long>(k, 1000);
      for (long long j = 1; j < k; ++j) {
        const double frac = double(j) / double(k);
        pts.push_back({a[0] + (b[0] - a[0]) * frac, a[1] + (b[1] - a[1]) * frac, a[2] + (b[2] - a[2]) * frac});
      }
    }
  std::sort(pts.begin(), pts.end());
  pts.erase(std::unique(pts.begin(), pts.end()), pts.end());
  return pts;
}

/// Times t in [tLo, tHi] at which the points moving along +d (sign +1), or the triangles
/// moving along +d (sign -1), meet the triangles. Projection onto the plane normal to d,
/// barycentric inside test with 1e-9 slack (the reference kernel's rules), and a uniform
/// 2D grid over the triangles so the cost is not points x triangles.
std::vector<double> rayHits(const std::vector<V>& points, const std::vector<Tri>& tris, const V& d, double sign, double tLo,
                            double tHi) {
  std::vector<double> out;
  if (points.empty() || tris.empty()) return out;
  V e1 = cross(d, {1.0, 0.0, 0.0});
  if (norm(e1) < 0.5) e1 = cross(d, {0.0, 1.0, 0.0});
  const double n1 = norm(e1);
  e1 = {e1[0] / n1, e1[1] / n1, e1[2] / n1};
  const V e2 = cross(d, e1);
  struct T2 {
    double ax, ay, v0x, v0y, v1x, v1y, d00, d01, d11, den, w0, w1, w2, lox, loy, hix, hiy;
  };
  std::vector<T2> ts;
  ts.reserve(tris.size());
  double glox = INFINITY, gloy = INFINITY, ghix = -INFINITY, ghiy = -INFINITY, sumExt = 0;
  for (const auto& t : tris) {
    double x[3], y[3], w[3];
    for (int k = 0; k < 3; ++k) {
      x[k] = dot(t.p[k], e1);
      y[k] = dot(t.p[k], e2);
      w[k] = dot(t.p[k], d);
    }
    T2 q;
    q.ax = x[0];
    q.ay = y[0];
    q.v0x = x[2] - x[0];
    q.v0y = y[2] - y[0];
    q.v1x = x[1] - x[0];
    q.v1y = y[1] - y[0];
    q.d00 = q.v0x * q.v0x + q.v0y * q.v0y;
    q.d01 = q.v0x * q.v1x + q.v0y * q.v1y;
    q.d11 = q.v1x * q.v1x + q.v1y * q.v1y;
    q.den = q.d00 * q.d11 - q.d01 * q.d01;
    if (!(std::abs(q.den) > 1e-18)) continue;
    q.w0 = w[0];
    q.w1 = w[1];
    q.w2 = w[2];
    q.lox = std::min({x[0], x[1], x[2]}) - 1e-9;
    q.hix = std::max({x[0], x[1], x[2]}) + 1e-9;
    q.loy = std::min({y[0], y[1], y[2]}) - 1e-9;
    q.hiy = std::max({y[0], y[1], y[2]}) + 1e-9;
    glox = std::min(glox, q.lox);
    gloy = std::min(gloy, q.loy);
    ghix = std::max(ghix, q.hix);
    ghiy = std::max(ghiy, q.hiy);
    sumExt += std::max(q.hix - q.lox, q.hiy - q.loy);
    ts.push_back(q);
  }
  if (ts.empty()) return out;
  const double ext = std::max(ghix - glox, ghiy - gloy);
  double cell = std::max(sumExt / double(ts.size()), ext / 2048.0);
  if (!(cell > 0)) cell = 1;
  const long long nx = std::min<long long>((long long)std::floor((ghix - glox) / cell) + 1, 4096);
  const long long ny = std::min<long long>((long long)std::floor((ghiy - gloy) / cell) + 1, 4096);
  const double cx = (ghix - glox) / double(nx) + 1e-300, cy = (ghiy - gloy) / double(ny) + 1e-300;
  auto ix = [&](double x) { return std::clamp<long long>((long long)std::floor((x - glox) / cx), 0, nx - 1); };
  auto iy = [&](double y) { return std::clamp<long long>((long long)std::floor((y - gloy) / cy), 0, ny - 1); };
  std::vector<std::vector<int>> grid(size_t(nx * ny));
  for (size_t i = 0; i < ts.size(); ++i)
    for (long long a = ix(ts[i].lox); a <= ix(ts[i].hix); ++a)
      for (long long b = iy(ts[i].loy); b <= iy(ts[i].hiy); ++b) grid[size_t(a * ny + b)].push_back(int(i));
  for (const auto& p : points) {
    const double px = dot(p, e1), py = dot(p, e2), pw = dot(p, d);
    if (px < glox || px > ghix || py < gloy || py > ghiy) continue;
    for (int i : grid[size_t(ix(px) * ny + iy(py))]) {
      const T2& q = ts[size_t(i)];
      if (px < q.lox || px > q.hix || py < q.loy || py > q.hiy) continue;
      const double v2x = px - q.ax, v2y = py - q.ay;
      const double d02 = q.v0x * v2x + q.v0y * v2y, d12 = q.v1x * v2x + q.v1y * v2y;
      const double u = (q.d11 * d02 - q.d01 * d12) / q.den;
      const double v = (q.d00 * d12 - q.d01 * d02) / q.den;
      if (!(u >= -1e-9 && v >= -1e-9 && u + v <= 1 + 1e-9)) continue;
      const double wt = q.w0 + u * (q.w2 - q.w0) + v * (q.w1 - q.w0);
      const double t = sign * (wt - pw);
      if (t >= tLo && t <= tHi) out.push_back(t);
    }
  }
  return out;
}

/// Sorted candidate contact times: only the moving body's leading facets can strike the
/// obstacle's facets that face back at it, so sliding contact never registers.
std::vector<double> meshFirstContacts(const SweepBody& mv, const SweepBody& ob, const V& d, double tLo, double tHi) {
  std::vector<Tri> front, back;
  for (size_t i = 0; i < mv.tris.size(); ++i)
    if (mv.facing[i] > SWEEP_GRAZE) front.push_back(mv.tris[i]);
  for (size_t i = 0; i < ob.tris.size(); ++i)
    if (ob.facing[i] < -SWEEP_GRAZE) back.push_back(ob.tris[i]);
  if (front.empty() || back.empty()) return {};
  const double spacing = 4 * std::max(mv.defl, ob.defl);
  std::vector<double> h = rayHits(samplePoints(front, spacing), back, d, +1.0, tLo, tHi);
  std::vector<double> h2 = rayHits(samplePoints(back, spacing), front, d, -1.0, tLo, tHi);
  h.insert(h.end(), h2.begin(), h2.end());
  std::sort(h.begin(), h.end());
  return h;
}

/// t-interval (t >= 0) in which moving box mb translated by t*d overlaps box ob (mb padded).
bool rayAabbInterval(const Box3& mb, const Box3& ob, const V& d, double pad, double& tLo, double& tHi) {
  tLo = 0;
  tHi = INFINITY;
  for (int k = 0; k < 3; ++k) {
    const double m0 = mb.lo[k] - pad, m1 = mb.hi[k] + pad, o0 = ob.lo[k], o1 = ob.hi[k];
    if (std::abs(d[size_t(k)]) < 1e-15) {
      if (m1 < o0 || m0 > o1) return false;
      continue;
    }
    const double a = (o1 - m0) / d[size_t(k)], b = (o0 - m1) / d[size_t(k)];
    const double lo = d[size_t(k)] > 0 ? b : a, hi = d[size_t(k)] > 0 ? a : b;
    tLo = std::max(tLo, lo);
    tHi = std::min(tHi, hi);
    if (tLo > tHi) return false;
  }
  return true;
}

}  // namespace

std::string doClearance(const json::Value& req, json::Writer& timing) {
  Placed a = parsePlaced(req.get("a"), "a");
  Placed b = parsePlaced(req.get("b"), "b");
  const double tol = numField(req, "tolerance", 0.0, "tolerance");
  if (tol < 0) throw bad("tolerance must be >= 0");
  if (a.rec->faceCount == 0 || b.rec->faceCount == 0) throw bad("clearance needs shapes with faces");
  BRepExtrema_DistShapeShape dss;
  dss.SetMultiThread(true);
  dss.SetFlag(Extrema_ExtFlag_MIN);
  dss.LoadS1(a.shape);
  dss.LoadS2(b.shape);
  dss.Perform();
  if (!dss.IsDone()) throw KernelError("crash", "BRepExtrema_DistShapeShape failed");
  double d = dss.Value();
  bool interferes = false;
  if (dss.InnerSolution()) {
    d = 0;
    interferes = true;
  } else if (d <= tol) {
    interferes = penetrates(a.shape, *a.rec, b.shape, *b.rec, tol);
    d = 0;
  }
  json::Writer w;
  w.beginObject().key("type").str("clearance:ok").key("distance").num(d).key("interferes").boolean(interferes).endObject();
  timing.key("distance").num(d);
  return std::move(w.out);
}

std::string doSweep(const json::Value& req, json::Writer& timing) {
  Placed m = parsePlaced(req.get("moving"), "moving");
  const json::Value* dv = req.get("direction");
  V d;
  {
    if (!dv || dv->kind != json::Value::Kind::Array || dv->a.size() != 3) throw bad("direction must be [x, y, z]");
    try {
      d = {dv->a[0].num(), dv->a[1].num(), dv->a[2].num()};
    } catch (const std::exception&) {
      throw bad("direction must be numbers");
    }
    const double n = norm(d);
    if (n < 1e-12) throw bad("direction must not be zero");
    d = {d[0] / n, d[1] / n, d[2] / n};
  }
  const json::Value* obsRaw = req.get("obstacles");
  if (!obsRaw || obsRaw->kind != json::Value::Kind::Array) throw bad("obstacles must be an array");
  std::vector<Placed> obstacles;
  for (size_t i = 0; i < obsRaw->a.size(); ++i) obstacles.push_back(parsePlaced(&obsRaw->a[i], "obstacles[" + std::to_string(i) + "]"));
  const double maxD = numField(req, "maxDistance", INFINITY, "maxDistance");
  double tol = numField(req, "tolerance", 0.0, "tolerance");
  if (!(tol > 0)) tol = 1e-3;
  if (!(maxD > 0)) throw bad("maxDistance must be > 0");
  Box3 mb;
  if (!bboxOf(m.shape, false, mb) || m.rec->faceCount == 0 || !m.rec->solid) throw bad("the moving shape has no solid");

  struct Cand {
    double tIn, tOut;
    const Placed* o;
  };
  std::vector<Cand> cands;
  for (const auto& o : obstacles) {
    if (o.rec->faceCount == 0 || !o.rec->solid) continue;
    Box3 ob;
    if (!bboxOf(o.shape, false, ob)) continue;
    double lo, hi;
    if (rayAabbInterval(mb, ob, d, tol, lo, hi)) cands.push_back({lo, hi, &o});
  }
  json::Writer w;
  auto answer = [&](double free, const std::string* by) {
    w.beginObject().key("type").str("sweep:ok").key("freeDistance").num(free);
    if (by) w.key("blockedBy").str(*by);
    w.endObject();
    timing.key("freeDistance").num(free).key("candidates").integer((long long)cands.size());
    return std::move(w.out);
  };
  if (cands.empty()) return answer(INFINITY, nullptr);

  const SweepBody mv = sweepBody(m, d);
  using Item = std::tuple<double, size_t, size_t>;  // (t, index into hits, obstacle number)
  std::priority_queue<Item, std::vector<Item>, std::greater<Item>> queue;
  std::vector<std::vector<double>> hitsOf;
  for (const auto& c : cands) {
    if (c.tIn > maxD) {
      hitsOf.emplace_back();
      continue;
    }
    const SweepBody ob = sweepBody(*c.o, d);
    const double eps = std::max(tol, 2 * (mv.defl + ob.defl));
    std::vector<double> h = meshFirstContacts(mv, ob, d, -eps, std::min(c.tOut, maxD) + eps);
    for (double& x : h) x = std::max(x, 0.0);
    hitsOf.push_back(std::move(h));
    if (!hitsOf.back().empty()) queue.push({hitsOf.back()[0], 0, hitsOf.size() - 1});
  }
  int confirms = 0;
  while (!queue.empty()) {
    const auto [t, k, oi] = queue.top();
    queue.pop();
    const Placed& o = *cands[oi].o;
    const double margin = std::max(tol, 2 * (mv.defl + o.rec->sweepDefl));
    if (confirms >= SWEEP_MAX_CONFIRM) {
      log("sweep: confirmation budget spent, accepting a mesh hit");
      return answer(t, &o.instanceId);
    }
    ++confirms;
    // Exact check just past the mesh contact: a real hit has a real common volume there.
    gp_Trsf tr;
    tr.SetTranslation(gp_Vec(d[0] * (t + margin), d[1] * (t + margin), d[2] * (t + margin)));
    if (penetrates(m.shape.Moved(TopLoc_Location(tr)), *m.rec, o.shape, *o.rec, tol)) return answer(t, &o.instanceId);
    const auto& h = hitsOf[oi];
    const size_t nxt = size_t(std::upper_bound(h.begin(), h.end(), t + margin) - h.begin());
    if (nxt < h.size()) queue.push({h[nxt], nxt, oi});
  }
  // Not hit within maxDistance. Infinity only if nothing lies anywhere along the path.
  bool beyond = false;
  for (const auto& c : cands) beyond = beyond || c.tOut > maxD;
  return answer(beyond ? maxD : INFINITY, nullptr);
}

}  // namespace cadoc
