// Import-cache exactness diagnostic. Imports a STEP file with the kernel's own code, then for
// each serialisation variant replaces every prototype by its write-read round trip and asks
// the same exact HLR (iso, hidden and smooth edges on, every instance), reporting:
//   - whether the HLR answer is byte-identical to the import's (timing stripped),
//   - per-class visible/hidden counts,
//   - whether the serialisation is idempotent on what it writes: bytes(write(x)) against
//     bytes(write(read(write(x)))), per shape, and in which section the first difference lies,
//   - the forms and scale factors of the locations inside the prototypes, before and after, and
//     how many come back bit for bit (form, scale factor, matrix, translation).
//
//   cadoc-cache-diag FILE.stp|FILE.brep [variant ...]   variants: bin4 bin4tri text bin5 (OCCT patch 0004)
//   CADOC_DIAG_NOHLR=1: skip the HLR comparison (large files)
//
// Native only; built against the same OCCT install as the kernel (kernel/tools/cache-diag/build.sh).
// SPDX-License-Identifier: LGPL-2.1-only
#include "kernel.hpp"
#include "session.hpp"

#include <BRepTools.hxx>
#include <BRep_Builder.hxx>
#include <BinTools.hxx>
#include <TopLoc_Location.hxx>
#include <BRep_CurveRepresentation.hxx>
#include <BRep_TEdge.hxx>
#include <BRep_PointRepresentation.hxx>
#include <BRep_TFace.hxx>
#include <BRep_TVertex.hxx>
#include <TopoDS_Iterator.hxx>
#include <gp_Trsf.hxx>

#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <vector>

using namespace cadoc;

namespace {

std::string slurp(const std::string& p) {
  std::ifstream f(p, std::ios::binary);
  std::stringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

using M4 = std::array<double, 16>;
M4 mul(const M4& a, const M4& b) {
  M4 o{};
  for (int c = 0; c < 4; ++c)
    for (int r = 0; r < 4; ++r)
      for (int k = 0; k < 4; ++k) o[c * 4 + r] += a[k * 4 + r] * b[c * 4 + k];
  return o;
}

struct Inst {
  std::string id, shapeId;
  M4 t;
};
void walk(const json::Value& n, const M4& parent, std::vector<Inst>& out) {
  M4 t{};
  const json::Value* tv = n.get("transform");
  for (int i = 0; i < 16; ++i) t[i] = tv->a[size_t(i)].num();
  const M4 w = mul(parent, t);
  const json::Value* sid = n.get("shapeId");
  if (sid && sid->kind == json::Value::Kind::String) out.push_back({n.get("id")->s, sid->s, w});
  for (const auto& c : n.get("children")->a) walk(c, w, out);
}

std::string jsonStr(const std::string& s) {
  std::string o = "\"";
  for (char c : s) {
    if (c == '"' || c == '\\') o += '\\';
    o += c;
  }
  return o + "\"";
}

std::string stripTiming(std::string s) {
  const size_t k = s.find("\"timing\":");
  if (k == std::string::npos) return s;
  size_t i = s.find('{', k), depth = 0;
  for (; i < s.size(); ++i) {
    if (s[i] == '{') ++depth;
    if (s[i] == '}' && --depth == 0) break;
  }
  size_t from = k, to = i + 1;
  if (from > 0 && s[from - 1] == ',') --from;
  else if (to < s.size() && s[to] == ',') ++to;
  return s.erase(from, to - from);
}

std::map<std::string, int> classCounts(const std::string& hlr) {
  std::map<std::string, int> m;
  const json::Value v = json::Parser(hlr).parse();
  const json::Value* edges = v.get("edges");
  if (!edges) {
    m["error:" + (v.get("code") ? v.get("code")->s : std::string("?"))] = 1;
    return m;
  }
  for (const auto& e : edges->a) m[e.get("edgeClass")->s + (e.get("visible")->b ? "/v" : "/h")]++;
  return m;
}

// ---- serialisation variants

std::string writeVariant(const TopoDS_Shape& s, const std::string& v) {
  std::ostringstream os(std::ios::out | std::ios::binary);
  if (v == "bin4") BinTools::Write(s, os, false, false, BinTools_FormatVersion_VERSION_4);
  else if (v == "bin4tri") BinTools::Write(s, os, true, true, BinTools_FormatVersion_VERSION_4);
#ifdef CADOC_HAVE_BIN5
  else if (v == "bin5") BinTools::Write(s, os, false, false, BinTools_FormatVersion_VERSION_5);
#endif
  else if (v == "text") BRepTools::Write(s, os, false, false, TopTools_FormatVersion_CURRENT);
  else throw std::runtime_error("unknown variant " + v);
  return os.str();
}

TopoDS_Shape readVariant(const std::string& bytes, const std::string& v) {
  TopoDS_Shape s;
  std::istringstream is(bytes, std::ios::in | std::ios::binary);
  if (v == "text") {
    BRep_Builder b;
    BRepTools::Read(s, is, b);
  } else {
    BinTools::Read(s, is);
  }
  return s;
}

std::string sectionAt(const std::string& bytes, size_t off) {
  // BinTools and BRepTools both write ASCII section headers before each table.
  const char* names[] = {"Locations", "Curve2ds", "Curves", "Polygon3D", "PolygonOnTriangulations", "Surfaces", "Triangulations", "TShapes"};
  std::string best = "header";
  size_t bestAt = 0;
  for (const char* n : names) {
    const size_t at = bytes.find(n);
    if (at != std::string::npos && at <= off && at >= bestAt) best = n, bestAt = at;
  }
  return best;
}

// ---- location census

const char* formName(gp_TrsfForm f) {
  switch (f) {
    case gp_Identity: return "identity";
    case gp_Rotation: return "rotation";
    case gp_Translation: return "translation";
    case gp_PntMirror: return "pntmirror";
    case gp_Ax1Mirror: return "ax1mirror";
    case gp_Ax2Mirror: return "ax2mirror";
    case gp_Scale: return "scale";
    case gp_CompoundTrsf: return "compound";
    default: return "other";
  }
}

// Every location a shape's B-rep holds, in a fixed order: each sub-shape's, each edge curve
// representation's, each face's (the surface's), each vertex point representation's. Read and original shapes have the same
// structure, so the lists line up.
void locations(const TopoDS_Shape& s, std::vector<TopLoc_Location>& out) {
  out.push_back(s.Location());
  const occ::handle<TopoDS_TShape>& ts = s.TShape();
  if (const auto e = occ::down_cast<BRep_TEdge>(ts); !e.IsNull())
    for (const auto& rep : e->Curves()) {
      out.push_back(rep->Location());
      if (rep->IsRegularity()) out.push_back(rep->Location2());
    }
  if (const auto f = occ::down_cast<BRep_TFace>(ts); !f.IsNull()) out.push_back(f->Location());
  if (const auto v = occ::down_cast<BRep_TVertex>(ts); !v.IsNull())
    for (const auto& rep : v->Points()) out.push_back(rep->Location());
  for (TopoDS_Iterator it(s, false, false); it.More(); it.Next()) locations(it.Value(), out);
}

void census(const TopoDS_Shape& s, std::map<std::string, int>& forms, int& scaleNot1) {
  std::vector<TopLoc_Location> ls;
  locations(s, ls);
  for (const auto& l : ls) {
    if (l.IsIdentity()) continue;
    const gp_Trsf& t = l.Transformation();
    forms[formName(t.Form())]++;
    if (t.ScaleFactor() != 1.0) ++scaleNot1;
  }
}

bool sameBits(double a, double b) { return std::memcmp(&a, &b, sizeof a) == 0; }

/// Locations whose transformation differs in any bit (form, scale, matrix, translation).
int locationDiffs(const TopoDS_Shape& a, const TopoDS_Shape& b, int& compared) {
  std::vector<TopLoc_Location> la, lb;
  locations(a, la);
  locations(b, lb);
  if (la.size() != lb.size()) return -1;
  int diff = 0;
  for (size_t i = 0; i < la.size(); ++i) {
    if (la[i].IsIdentity() && lb[i].IsIdentity()) continue;
    ++compared;
    const gp_Trsf &x = la[i].Transformation(), &y = lb[i].Transformation();
    bool same = x.Form() == y.Form() && sameBits(x.ScaleFactor(), y.ScaleFactor());
    for (int r = 1; r <= 3 && same; ++r) {
      same = sameBits(x.TranslationPart().Coord(r), y.TranslationPart().Coord(r));
      for (int c = 1; c <= 3 && same; ++c) same = sameBits(x.HVectorialPart()(r, c), y.HVectorialPart()(r, c));
    }
    if (!same) ++diff;
  }
  return diff;
}

}  // namespace

// --sweep FILE...: exact HLR (hidden and smooth edges on) of the whole document and of each
// shape on its own, from 14 view directions (6 axes, 8 diagonals). Before each call it writes
// "CTX <file> <view> <whole|shapeId>" to stderr, so an instrumented OCCT's own stderr lines can
// be attributed to a file, view and shape (used to find public reproductions of patches 0002
// and 0003, kernel/build/patches/UPSTREAM.md). CADOC_SWEEP_NO_WHOLE=1: each shape only;
// CADOC_SWEEP_SHAPES=<text>: only shapes whose name contains it.
int sweep(int argc, char** argv) {
  const double v[14][3] = {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}, {1, 1, 1}, {1, 1, -1},
                           {1, -1, 1}, {1, -1, -1}, {-1, 1, 1}, {-1, 1, -1}, {-1, -1, 1}, {-1, -1, -1}};
  for (int a = 2; a < argc; ++a) {
    const std::string file = argv[a], bytes = slurp(file);
    const bool brep = file.size() > 5 && file.substr(file.size() - 5) == ".brep";
    const std::string imp = handle(std::string("{\"type\":\"import\",\"format\":\"") + (brep ? "brep" : "step") + "\"}",
                                   reinterpret_cast<const uint8_t*>(bytes.data()), bytes.size());
    const json::Value iv = json::Parser(imp).parse();
    if (!iv.get("root")) {
      std::cerr << "import failed: " << file << "\n";
      continue;
    }
    std::vector<Inst> inst;
    M4 I{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
    walk(*iv.get("root"), I, inst);
    std::map<std::string, int> faces;
    const char* only = std::getenv("CADOC_SWEEP_SHAPES");  // substring of the shape names to sweep
    for (const auto& sh : iv.get("shapes")->a)
      if (!only || sh.get("name")->s.find(only) != std::string::npos) faces[sh.get("shapeId")->s] = int(sh.get("faceCount")->num());
    for (int k = 0; k < 14; ++k) {
      const double n = std::sqrt(v[k][0] * v[k][0] + v[k][1] * v[k][1] + v[k][2] * v[k][2]);
      const bool vertical = v[k][0] == 0 && v[k][1] == 0;
      std::ostringstream view;
      view.precision(17);
      view << "{\"viewDirection\":[" << v[k][0] / n << "," << v[k][1] / n << "," << v[k][2] / n << "],\"up\":" << (vertical ? "[0,1,0]" : "[0,0,1]")
           << ",\"projection\":\"orthographic\"}";
      auto run = [&](const std::string& what, const std::vector<Inst>& set) {
        std::ostringstream rq;
        rq.precision(17);
        rq << "{\"type\":\"hlr\",\"view\":" << view.str() << ",\"algorithm\":\"exact\",\"includeHidden\":true,\"includeSmooth\":true,\"instances\":[";
        bool first = true;
        for (const auto& i : set) {
          if (faces[i.shapeId] == 0) continue;
          rq << (first ? "" : ",") << "{\"instanceId\":" << jsonStr(i.id) << ",\"shapeId\":" << jsonStr(i.shapeId) << ",\"transform\":[";
          for (int q = 0; q < 16; ++q) rq << (q ? "," : "") << i.t[q];
          rq << "]}";
          first = false;
        }
        rq << "]}";
        if (first) return;
        std::fprintf(stderr, "CTX %s view%d %s\n", file.c_str(), k, what.c_str());
        std::fflush(stderr);
        handle(rq.str(), nullptr, 0);
        std::fflush(stderr);
      };
      if (!std::getenv("CADOC_SWEEP_NO_WHOLE")) run("whole", inst);
      for (const auto& [sid, nf] : faces) {
        if (nf == 0) continue;
        run(sid, {Inst{"probe", sid, I}});
      }
    }
  }
  return 0;
}

int main(int argc, char** argv) {
  if (argc >= 2 && std::string(argv[1]) == "--sweep") return sweep(argc, argv);
  if (argc < 2) {
    std::cerr << "usage: cadoc-cache-diag FILE.stp [variant ...]\n";
    return 2;
  }
  const std::string step = slurp(argv[1]);
  std::vector<std::string> variants;
  for (int i = 2; i < argc; ++i) variants.push_back(argv[i]);
  if (variants.empty()) variants = {"bin4", "text", "bin5"};

  const std::string file = argv[1];
  const bool brep = file.size() > 5 && file.substr(file.size() - 5) == ".brep";
  const std::string imp = handle(std::string("{\"type\":\"import\",\"format\":\"") + (brep ? "brep" : "step") + "\"}",
                                 reinterpret_cast<const uint8_t*>(step.data()), step.size());
  const json::Value iv = json::Parser(imp).parse();
  if (!iv.get("root")) {
    std::cerr << "import failed: " << imp.substr(0, 300) << "\n";
    return 1;
  }
  std::map<std::string, int> faces;
  for (const auto& sh : iv.get("shapes")->a) faces[sh.get("shapeId")->s] = int(sh.get("faceCount")->num());
  std::vector<Inst> inst;
  M4 I{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
  walk(*iv.get("root"), I, inst);
  std::ostringstream rq;
  rq.precision(17);
  const double c = 1 / std::sqrt(3.0);
  rq << "{\"type\":\"hlr\",\"view\":{\"viewDirection\":[" << c << "," << c << "," << c
     << "],\"up\":[0,0,1],\"projection\":\"orthographic\"},\"algorithm\":\"exact\",\"includeHidden\":true,\"includeSmooth\":true,\"instances\":[";
  bool first = true;
  for (const auto& i : inst) {
    if (faces[i.shapeId] == 0) continue;
    rq << (first ? "" : ",") << "{\"instanceId\":" << jsonStr(i.id) << ",\"shapeId\":" << jsonStr(i.shapeId) << ",\"transform\":[";
    for (int k = 0; k < 16; ++k) rq << (k ? "," : "") << i.t[k];
    rq << "]}";
    first = false;
  }
  rq << "]}";
  const std::string req = rq.str();

  Session& s = session();
  std::vector<TopoDS_Shape> orig;
  for (const auto& r : s.shapes) orig.push_back(r->shape);
  const bool noHlr = std::getenv("CADOC_DIAG_NOHLR") != nullptr;
  const std::string base = noHlr ? std::string("{\"edges\":[]}") : stripTiming(handle(req, nullptr, 0));
  const auto baseCounts = classCounts(base);

  std::map<std::string, int> forms;
  int scaleNot1 = 0;
  for (const auto& o : orig) census(o, forms, scaleNot1);
  std::cout << "file " << argv[1] << "\n  shapes " << orig.size() << ", instances " << inst.size() << "\n  locations inside prototypes (shapes, edges, faces, vertices):";
  for (const auto& [k, v] : forms) std::cout << " " << k << "=" << v;
  std::cout << " (scale != 1: " << scaleNot1 << ")\n  import HLR:";
  for (const auto& [k, v] : baseCounts) std::cout << " " << k << "=" << v;
  std::cout << "\n";

  for (const auto& v : variants) {
    int notIdem = 0;
    long long locs = 0;
    std::map<std::string, int> where;
    std::vector<TopoDS_Shape> rt;
    for (size_t i = 0; i < orig.size(); ++i) {
      const std::string a = writeVariant(orig[i], v);
      if (const size_t at = a.find("Locations "); at != std::string::npos) locs += std::atoll(a.c_str() + at + 10);
      TopoDS_Shape back = readVariant(a, v);
      const std::string b = writeVariant(back, v);
      if (a != b) {
        ++notIdem;
        size_t k = 0;
        while (k < a.size() && k < b.size() && a[k] == b[k]) ++k;
        where[sectionAt(a, k)]++;
      }
      rt.push_back(back);
    }
    std::map<std::string, int> f2;
    int s2 = 0, locCompared = 0, locDiff = 0;
    for (const auto& r : rt) census(r, f2, s2);
    for (size_t i = 0; i < orig.size(); ++i) {
      const int d = locationDiffs(orig[i], rt[i], locCompared);
      locDiff += d < 0 ? 1000000 : d;
    }
    for (size_t i = 0; i < orig.size(); ++i) s.shapes[i]->shape = rt[i], s.shapes[i]->meshed = false;
    const std::string got = noHlr ? base : stripTiming(handle(req, nullptr, 0));
    for (size_t i = 0; i < orig.size(); ++i) s.shapes[i]->shape = orig[i], s.shapes[i]->meshed = false;
    std::cout << "  [" << v << "] HLR " << (got == base ? "IDENTICAL" : "DIFFERS") << ";";
    if (got != base)
      for (const auto& [k, n] : classCounts(got)) std::cout << " " << k << "=" << n;
    std::cout << "\n    write-read-write not idempotent on " << notIdem << "/" << orig.size() << " shapes";
    for (const auto& [k, n] : where) std::cout << " " << k << "=" << n;
    std::cout << "\n    locations written (sum over shapes, incl. curve representations): " << locs;
    std::cout << "\n    locations bit-identical after the round trip: " << (locCompared - locDiff) << "/" << locCompared;
    std::cout << "\n    locations after:";
    for (const auto& [k, n] : f2) std::cout << " " << k << "=" << n;
    std::cout << " (scale != 1: " << s2 << ")\n";
  }
  return 0;
}
