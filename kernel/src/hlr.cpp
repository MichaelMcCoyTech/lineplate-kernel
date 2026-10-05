// "hlr": hidden-line removal (protocol.ts HlrRequest -> HlrResult), exact (HLRBRep_Algo on
// the B-rep) or polygonal (HLRBRep_PolyAlgo on meshes), orthographic or perspective, with
// optional targets and per-target occluders. Behaviour matches the reference kernel's
// do_hlr (kernel/reference/refkernel.py): the same runs, the same edge order, the same curves.
// SPDX-License-Identifier: LGPL-2.1-only
#include "curves.hpp"
#include "session.hpp"

#include <BRepAdaptor_Curve.hxx>
#include <HLRAlgo_Projector.hxx>
#include <HLRBRep_Algo.hxx>
#include <HLRBRep_HLRToShape.hxx>
#include <HLRBRep_PolyAlgo.hxx>
#include <HLRBRep_PolyHLRToShape.hxx>
#include <HLRBRep_TypeOfResultingEdge.hxx>
#include <BRep_Builder.hxx>
#include <NCollection_IndexedMap.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_ShapeMapHasher.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <gp_Ax2.hxx>
#include <gp_Pnt2d.hxx>

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <functional>
#include <set>

namespace cadoc {
namespace {

using V = std::array<double, 3>;

V vec3(const json::Value* v, const std::string& what) {
  if (!v || v->kind != json::Value::Kind::Array || v->a.size() != 3) throw bad(what + " must be [x, y, z]");
  try {
    return {v->a[0].num(), v->a[1].num(), v->a[2].num()};
  } catch (const std::exception&) {
    throw bad(what + " must be numbers");
  }
}
V unit(const V& v, const std::string& what) {
  const double n = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
  if (n < 1e-12) throw bad(what + " must not be zero");
  return {v[0] / n, v[1] / n, v[2] / n};
}
V cross(const V& a, const V& b) { return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]}; }
double dot(const V& a, const V& b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }

HLRAlgo_Projector makeProjector(const json::Value* view, P2& offset) {
  if (!view || view->kind != json::Value::Kind::Object) throw bad("hlr.view must be a ViewSpec");
  const V d = unit(vec3(view->get("viewDirection"), "view.viewDirection"), "view.viewDirection");
  const V up = unit(vec3(view->get("up"), "view.up"), "view.up");
  V right = cross(up, d);
  if (std::sqrt(dot(right, right)) < 1e-9) throw bad("view.up must not be parallel to view.viewDirection");
  right = unit(right, "right");
  const json::Value* pk = view->get("projection");
  const std::string kind = pk && pk->kind == json::Value::Kind::String ? pk->s : (pk ? "?" : "orthographic");
  offset = {0, 0};
  if (kind == "orthographic") return HLRAlgo_Projector(gp_Ax2(gp_Pnt(0, 0, 0), gp_Dir(d[0], d[1], d[2]), gp_Dir(right[0], right[1], right[2])));
  if (kind == "perspective") {
    const V eye = vec3(view->get("eye"), "view.eye");
    const double focus = dot(eye, d);
    if (focus <= 1e-9) throw bad("perspective: the eye must be in front of the world origin along viewDirection");
    // Image plane through the world origin, perpendicular to viewDirection; the eye sits
    // `focus` in front of it. Output is then shifted so the world origin projects to (0, 0).
    const gp_Pnt o(eye[0] - focus * d[0], eye[1] - focus * d[1], eye[2] - focus * d[2]);
    HLRAlgo_Projector pr(gp_Ax2(o, gp_Dir(d[0], d[1], d[2]), gp_Dir(right[0], right[1], right[2])), focus);
    gp_Pnt2d q;
    pr.Project(gp_Pnt(0, 0, 0), q);
    offset = {q.X(), q.Y()};
    return pr;
  }
  throw bad("view.projection " + repr(kind) + " must be orthographic or perspective");
}

bool getBool(const json::Value& req, const char* k) {
  const json::Value* v = req.get(k);
  if (!v) return false;
  if (v->kind == json::Value::Kind::Bool) return v->b;
  if (v->kind == json::Value::Kind::Number) return v->n != 0;
  return true;
}

struct ExactClass {
  HLRBRep_TypeOfResultingEdge type;
  const char* name;
  bool smooth;
};
const ExactClass kExact[] = {
    {HLRBRep_OutLine, "outline", false},
    {HLRBRep_Sharp, "sharp", false},
    {HLRBRep_Rg1Line, "smooth", true},
    {HLRBRep_RgNLine, "seam", true},
};

}  // namespace

std::string doHlr(const json::Value& req, json::Writer& timing) {
  auto t0 = Clock::now();
  P2 offset;
  HLRAlgo_Projector projector = makeProjector(req.get("view"), offset);
  const json::Value* algV = req.get("algorithm");
  const std::string algorithm = algV ? (algV->kind == json::Value::Kind::String ? algV->s : "?") : "exact";
  if (algorithm != "exact" && algorithm != "polygonal") throw bad("hlr.algorithm must be exact or polygonal");
  const bool includeHidden = getBool(req, "includeHidden");
  const bool includeSmooth = getBool(req, "includeSmooth");
  const json::Value* raw = req.get("instances");
  if (!raw || raw->kind != json::Value::Kind::Array) throw bad("hlr.instances must be an array");
  std::vector<Placed> placed;
  std::map<std::string, size_t> index;
  for (size_t i = 0; i < raw->a.size(); ++i) {
    Placed pl = parsePlaced(&raw->a[i], "instances[" + std::to_string(i) + "]");
    if (index.count(pl.instanceId)) throw bad("duplicate instanceId " + repr(pl.instanceId));
    index[pl.instanceId] = placed.size();
    placed.push_back(std::move(pl));
  }
  // instanceFaces (protocol.ts): an instance loaded as only these faces (indices in
  // TopExp::MapShapes(FACE) order, as a mesh's faceOfTriangle), placed as the instance.
  if (const json::Value* of = req.get("instanceFaces")) {
    if (of->kind != json::Value::Kind::Object) throw bad("instanceFaces must be an object");
    for (const auto& [iid, list] : of->o) {
      auto it = index.find(iid);
      if (it == index.end()) throw bad("instanceFaces names " + repr(iid) + ", which is not an instance of the request");
      if (list.kind != json::Value::Kind::Array) throw bad("instanceFaces[" + repr(iid) + "] must be an array of face indices");
      Placed& pl = placed[it->second];
      NCollection_IndexedMap<TopoDS_Shape, TopTools_ShapeMapHasher> faces;
      TopExp::MapShapes(pl.rec->shape, TopAbs_FACE, faces);
      TopoDS_Compound c;
      BRep_Builder b;
      b.MakeCompound(c);
      int n = 0;
      for (const auto& x : list.a) {
        const int k = int(x.num());
        if (k < 0 || k >= faces.Extent()) throw bad("instanceFaces[" + repr(iid) + "]: face " + std::to_string(k) + " is out of range");
        b.Add(c, faces(k + 1));
        ++n;
      }
      pl.shape = c.Moved(TopLoc_Location(pl.trsf));
      pl.partial = n;
    }
  }
  std::vector<std::string> targets;
  const json::Value* tv = req.get("targetInstanceIds");
  if (!tv) {
    for (const auto& p : placed) targets.push_back(p.instanceId);
  } else {
    bool ok = tv->kind == json::Value::Kind::Array;
    if (ok)
      for (const auto& t : tv->a) {
        if (t.kind != json::Value::Kind::String || !index.count(t.s)) {
          ok = false;
          break;
        }
        targets.push_back(t.s);
      }
    if (!ok) throw bad("targetInstanceIds must name instances in the request");
  }
  const json::Value* occ = req.get("occludersByTarget");
  if (occ && occ->kind != json::Value::Kind::Object) throw bad("occludersByTarget must be an object");
  auto drawable = [&](const std::string& iid) {
    const Placed& p = placed[index.at(iid)];
    return p.partial >= 0 ? p.partial > 0 : p.rec->faceCount > 0;
  };
  // Faces with no edges are tessellated-only geometry (AP242 TESSELLATED_SHELL, e.g. NIST FTC 08
  // "-tg"). Both HLR algorithms crash on it (exact: a null surface; polygonal: out of range),
  // so it is refused instead of killing the kernel.
  for (const auto& p : placed)
    if (p.rec->faceCount > 0 && p.rec->edgeCount == 0)
      throw bad("instance " + repr(p.instanceId) +
                " is tessellated-only geometry (faces without B-rep edges); HLR needs B-rep geometry");

  // Runs: (instances in the run, targets extracted from it).
  std::vector<std::pair<std::vector<std::string>, std::vector<std::string>>> runs;
  std::vector<std::string> allDrawable;
  for (const auto& p : placed)
    if (drawable(p.instanceId)) allDrawable.push_back(p.instanceId);
  if (!occ) {
    std::vector<std::string> ts;
    for (const auto& t : targets)
      if (drawable(t)) ts.push_back(t);
    runs.push_back({allDrawable, ts});
  } else {
    for (const auto& t : targets) {
      if (!drawable(t)) continue;
      const json::Value* o = occ->get(t);
      std::vector<std::string> members;
      if (!o) {
        members = allDrawable;
      } else {
        bool ok = o->kind == json::Value::Kind::Array;
        if (ok)
          for (const auto& x : o->a)
            if (x.kind != json::Value::Kind::String || !index.count(x.s)) ok = false;
        if (!ok) throw bad("occludersByTarget[" + repr(t) + "] must name instances in the request");
        members.push_back(t);
        std::set<std::string> seen;
        for (const auto& x : o->a)
          if (seen.insert(x.s).second && x.s != t && drawable(x.s)) members.push_back(x.s);
      }
      runs.push_back({members, {t}});
    }
  }
  // Exact HLR with occluders: one run for all targets instead of one per target. Every
  // instance any target needs is loaded once (one Update), and only the target edges are
  // hidden, each by its own faces and by its listed occluders: the calls OCCT's Hide()
  // makes (Hide(i), then Hide(i, j)), restricted to those pairs. A target's edges are never
  // touched by another target's calls, so each comes out as it would from its own run,
  // without re-loading shared occluders per target or hiding occluders' edges nobody draws.
  std::map<std::string, std::vector<std::string>> hideBy;  // target -> its occluders, in order
  if (occ && algorithm == "exact" && !runs.empty()) {
    std::vector<std::string> unionMembers, unionTargets;
    std::set<std::string> in;
    for (const auto& [members, ts] : runs) {
      const std::string& t = ts.front();
      unionTargets.push_back(t);
      hideBy[t] = std::vector<std::string>(members.begin() + (members.front() == t ? 1 : 0), members.end());
      hideBy[t].erase(std::remove(hideBy[t].begin(), hideBy[t].end(), t), hideBy[t].end());
      for (const auto& m : members)
        if (in.insert(m).second) unionMembers.push_back(m);
    }
    runs.clear();
    runs.push_back({unionMembers, unionTargets});
  }
  long long ownHides = 0, pairHides = 0;

  double diag = 0;
  {
    bool any = false;
    Box3 all{};
    for (const auto& p : placed) {
      if (!drawable(p.instanceId)) continue;
      Box3 b;
      // The whole instance, even when instanceFaces loads part of it: the curve tolerance
      // derived from this must not depend on which faces a batch needed.
      if (!bboxOf(p.partial >= 0 ? p.rec->shape.Moved(TopLoc_Location(p.trsf)) : p.shape, false, b)) continue;
      for (int k = 0; k < 3; ++k) {
        all.lo[k] = any ? std::min(all.lo[k], b.lo[k]) : b.lo[k];
        all.hi[k] = any ? std::max(all.hi[k], b.hi[k]) : b.hi[k];
      }
      any = true;
    }
    if (any) diag = std::sqrt(std::pow(all.hi[0] - all.lo[0], 2) + std::pow(all.hi[1] - all.lo[1], 2) + std::pow(all.hi[2] - all.lo[2], 2));
  }
  CurveEmitter emitter(offset, std::max(5e-3, diag * 1e-5));
  double setupMs = msSince(t0), hlrMs = 0, extractMs = 0;

  if (algorithm == "polygonal") {
    auto t1 = Clock::now();
    const double lin = std::max(diag * 1e-3, 1e-3);
    std::set<ShapeRec*> recs;
    for (const auto& p : placed)
      if (drawable(p.instanceId) && recs.insert(p.rec).second) ensureMesh(*p.rec, lin, 0.2);
    setupMs += msSince(t1);
  }

  json::Writer w;
  w.beginObject().key("type").str("hlr:ok").key("edges").beginArray();
  long long nEdges = 0;
  auto writeEdge = [&](const std::string& iid, const char* cls, bool vis, const Curve2D& c) {
    w.beginObject().key("instanceId").str(iid).key("edgeClass").str(cls).key("visible").boolean(vis).key("curve");
    c.write(w);
    w.endObject();
    ++nEdges;
  };

  for (const auto& [members, runTargets] : runs) {
    if (runTargets.empty()) continue;
    auto t1 = Clock::now();
    Clock::time_point t2, t3;
    if (algorithm == "exact") {
      occ::handle<HLRBRep_Algo> algo = new HLRBRep_Algo();
      for (const auto& iid : members) algo->Add(placed[index.at(iid)].shape);
      algo->Projector(projector);
      t2 = Clock::now();
      algo->Update();
      if (hideBy.empty()) {
        algo->Hide();
      } else {
        std::map<std::string, int> num;  // instance -> its 1-based index in the algorithm (Add order)
        for (size_t k = 0; k < members.size(); ++k) num[members[k]] = int(k) + 1;
        for (const auto& t : runTargets) {
          auto tp = Clock::now();
          algo->Hide(num.at(t));
          ++ownHides;
          if (std::getenv("CADOC_TRACE") && msSince(tp) > 300) std::fprintf(stderr, "trace own %s: %.0f ms\n", t.c_str(), msSince(tp));
        }
        // CADOC_TRACE (native diagnosis only): slow hide calls on stderr.
        static const bool trace = std::getenv("CADOC_TRACE") != nullptr;
        for (const auto& t : runTargets)
          for (const auto& o : hideBy.at(t)) {
            auto tp = Clock::now();
            algo->Hide(num.at(t), num.at(o));
            ++pairHides;
            if (trace && msSince(tp) > 300) std::fprintf(stderr, "trace hide %s by %s: %.0f ms\n", t.c_str(), o.c_str(), msSince(tp));
          }
        algo->Select();
      }
      t3 = Clock::now();
      HLRBRep_HLRToShape h(algo);
      for (const auto& iid : runTargets) {
        const TopoDS_Shape& S = placed[index.at(iid)].shape;
        for (const auto& cls : kExact) {
          if (cls.smooth && !includeSmooth) continue;
          for (bool vis : {true, false}) {
            if (!vis && !includeHidden) continue;
            TopoDS_Shape comp = h.CompoundOfEdges(S, cls.type, vis, false);
            if (comp.IsNull()) continue;
            for (TopExp_Explorer ex(comp, TopAbs_EDGE); ex.More(); ex.Next()) {
              Curve2D c;
              if (emitter.emit(TopoDS::Edge(ex.Current()), c)) writeEdge(iid, cls.name, vis, c);
            }
          }
        }
      }
    } else {
      occ::handle<HLRBRep_PolyAlgo> algo = new HLRBRep_PolyAlgo();
      for (const auto& iid : members) algo->Load(placed[index.at(iid)].shape);
      algo->Projector(projector);
      t2 = Clock::now();
      algo->Update();
      HLRBRep_PolyHLRToShape h;
      h.Update(algo);
      t3 = Clock::now();
      using Fn = std::function<TopoDS_Shape(const TopoDS_Shape&)>;
      struct Group {
        const char* cls;
        bool vis;
        Fn fn;
      };
      std::vector<Group> groups = {{"outline", true, [&](const TopoDS_Shape& s) { return h.OutLineVCompound(s); }},
                                   {"sharp", true, [&](const TopoDS_Shape& s) { return h.VCompound(s); }}};
      if (includeSmooth) {
        groups.push_back({"smooth", true, [&](const TopoDS_Shape& s) { return h.Rg1LineVCompound(s); }});
        groups.push_back({"seam", true, [&](const TopoDS_Shape& s) { return h.RgNLineVCompound(s); }});
      }
      if (includeHidden) {
        groups.push_back({"outline", false, [&](const TopoDS_Shape& s) { return h.OutLineHCompound(s); }});
        groups.push_back({"sharp", false, [&](const TopoDS_Shape& s) { return h.HCompound(s); }});
        if (includeSmooth) {
          groups.push_back({"smooth", false, [&](const TopoDS_Shape& s) { return h.Rg1LineHCompound(s); }});
          groups.push_back({"seam", false, [&](const TopoDS_Shape& s) { return h.RgNLineHCompound(s); }});
        }
      }
      for (const auto& iid : runTargets) {
        const TopoDS_Shape& S = placed[index.at(iid)].shape;
        for (const auto& g : groups) {
          std::vector<std::array<P2, 2>> segs;
          TopoDS_Shape comp = g.fn(S);
          if (!comp.IsNull())
            for (TopExp_Explorer ex(comp, TopAbs_EDGE); ex.More(); ex.Next()) {
              BRepAdaptor_Curve ad(TopoDS::Edge(ex.Current()));
              gp_Pnt pa = ad.Value(ad.FirstParameter()), pb = ad.Value(ad.LastParameter());
              P2 a = emitter.p2(pa.X(), pa.Y()), b = emitter.p2(pb.X(), pb.Y());
              if (std::hypot(a[0] - b[0], a[1] - b[1]) > 1e-9) segs.push_back({a, b});
            }
          for (auto& chain : chainSegments(segs, 1e-4, 40.0 * 3.14159265358979323846 / 180.0)) {
            Curve2D c;
            if (chain.size() == 2) {
              c.kind = Curve2D::Kind::Line;
              c.a = chain[0];
              c.b = chain[1];
            } else {
              c.kind = Curve2D::Kind::Polyline;
              c.points = std::move(chain);
            }
            ++emitter.stats[Curve2D::name(c.kind)];
            writeEdge(iid, g.cls, g.vis, c);
          }
        }
      }
    }
    auto t4 = Clock::now();
    setupMs += std::chrono::duration<double, std::milli>(t2 - t1).count();
    hlrMs += std::chrono::duration<double, std::milli>(t3 - t2).count();
    extractMs += std::chrono::duration<double, std::milli>(t4 - t3).count();
  }
  w.endArray();
  w.key("timing").beginObject().key("setupMs").num(setupMs).key("hlrMs").num(hlrMs).key("extractMs").num(extractMs).endObject();
  w.endObject();
  timing.key("setupMs").num(setupMs).key("hlrMs").num(hlrMs).key("extractMs").num(extractMs).key("edges").integer(nEdges)
      .key("runs").integer((long long)runs.size()).key("ownHideCalls").integer(ownHides).key("pairHideCalls").integer(pairHides);
  timing.key("curves").beginObject();
  for (const auto& [k, v] : emitter.stats) timing.key(k.c_str()).integer(v);
  timing.endObject();
  return std::move(w.out);
}

}  // namespace cadoc
