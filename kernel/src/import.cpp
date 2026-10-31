// "import": a CAD file -> the assembly tree and per-shape properties (protocol.ts ImportResult).
//
// Behaviour matches the reference kernel (kernel/reference/refkernel.py, do_import):
// ids, names, colours, units, warnings and every number. The one difference is the
// formats this build reads: STEP and BRep. IGES needs OCCT's TKDEIGES, which this build
// does not include, so it is answered "unsupported-format".
// SPDX-License-Identifier: LGPL-2.1-only
#include "session.hpp"

#include <BRepGProp.hxx>
#include <BRepTools.hxx>
#include <BRep_Builder.hxx>
#include <GProp_GProps.hxx>
#include <GProp_PrincipalProps.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <Interface_Static.hxx>
#include <NCollection_IndexedMap.hxx>
#include <Quantity_Color.hxx>
#include <STEPCAFControl_Reader.hxx>
#include <STEPControl_Reader.hxx>
#include <Standard_Failure.hxx>
#include <TCollection_AsciiString.hxx>
#include <TDF_Label.hxx>
#include <TDF_Tool.hxx>
#include <TDataStd_Name.hxx>
#include <TDocStd_Document.hxx>
#include <TopAbs.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopLoc_Location.hxx>
#include <TopoDS_Iterator.hxx>
#include <TopTools_ShapeMapHasher.hxx>
#include <XCAFApp_Application.hxx>
#include <XCAFDoc_ColorTool.hxx>
#include <XCAFDoc_ColorType.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdlib>
#include <exception>
#include <istream>
#include <mutex>
#include <set>
#include <streambuf>
#include <thread>
#include <unordered_map>

namespace cadoc {
namespace {

std::string unitName(std::string u) {
  // strip + lower, then the reference kernel's exact table
  auto notSpace = [](unsigned char c) { return !std::isspace(c); };
  u.erase(u.begin(), std::find_if(u.begin(), u.end(), notSpace));
  u.erase(std::find_if(u.rbegin(), u.rend(), notSpace).base(), u.end());
  std::transform(u.begin(), u.end(), u.begin(), [](unsigned char c) { return char(std::tolower(c)); });
  static const std::map<std::string, std::string> names = {
      {"millimetre", "mm"}, {"millimeter", "mm"}, {"mm", "mm"}, {"centimetre", "cm"}, {"centimeter", "cm"},
      {"cm", "cm"},         {"metre", "m"},       {"meter", "m"}, {"m", "m"},           {"inch", "in"},
      {"in", "in"},         {"foot", "ft"},       {"feet", "ft"}, {"ft", "ft"}};
  auto it = names.find(u);
  return it == names.end() ? "unknown" : it->second;
}

/// The label's name, or "" if it has none. STEPCAF's invented "=>[0:1:1:18]" is not a name.
std::string labelName(const TDF_Label& l) {
  if (l.IsNull() || !l.IsAttribute(TDataStd_Name::GetID())) return "";
  occ::handle<TDataStd_Name> n;
  if (!l.FindAttribute(TDataStd_Name::GetID(), n)) return "";
  TCollection_AsciiString s(n->Get());  // UTF-8
  std::string out = s.ToCString();
  if (out.rfind("=>[", 0) == 0) return "";
  return out;
}

double round6(double x) { return std::round(x * 1e6) / 1e6; }

bool labelColor(const TDF_Label& l, double rgb[3]) {
  if (l.IsNull()) return false;
  Quantity_Color c;
  for (XCAFDoc_ColorType t : {XCAFDoc_ColorSurf, XCAFDoc_ColorGen, XCAFDoc_ColorCurv}) {
    if (XCAFDoc_ColorTool::GetColor(l, t, c)) {
      // The file's numbers, unconverted (STEP stores plain RGB; treated as sRGB).
      rgb[0] = round6(c.Red());
      rgb[1] = round6(c.Green());
      rgb[2] = round6(c.Blue());
      return true;
    }
  }
  return false;
}

int countOf(const TopoDS_Shape& s, TopAbs_ShapeEnum t) {
  NCollection_IndexedMap<TopoDS_Shape, TopTools_ShapeMapHasher> m;
  TopExp::MapShapes(s, t, m);
  return m.Extent();
}

bool hasSolid(const TopoDS_Shape& s) { return TopExp_Explorer(s, TopAbs_SOLID).More(); }

std::string escapeSegment(const std::string& name) {
  std::string out;
  for (char c : name) {
    if (c == '%') out += "%25";
    else if (c == '/') out += "%2F";
    else out += c;
  }
  return out;
}

struct Props {
  double volume = 0, area = 0, centroid[3] = {0, 0, 0}, moments[3] = {0, 0, 0};
};

/// volume, area, centroid, principal moments (ascending, unit density, about the centroid).
Props shapeProps(const TopoDS_Shape& shape) {
  Props p;
  GProp_GProps areaP, volP;
  BRepGProp::SurfaceProperties(shape, areaP);
  p.area = std::abs(areaP.Mass());
  BRepGProp::VolumeProperties(shape, volP);
  double volume = volP.Mass();
  GProp_GProps* use = &volP;
  if (!hasSolid(shape) || std::abs(volume) < 1e-12) {
    volume = 0;  // no solid: centroid and moments of the surface instead
    use = &areaP;
  }
  p.volume = std::abs(volume);
  if (std::abs(use->Mass()) < 1e-15) return p;
  gp_Pnt c = use->CentreOfMass();
  p.centroid[0] = c.X();
  p.centroid[1] = c.Y();
  p.centroid[2] = c.Z();
  double i1, i2, i3;
  use->PrincipalProperties().Moments(i1, i2, i3);
  double m[3] = {std::abs(i1), std::abs(i2), std::abs(i3)};
  std::sort(m, m + 3);
  std::copy(m, m + 3, p.moments);
  return p;
}

/// One distinct shape found by the tree walk. Its properties (counts, optimal bbox, mass
/// properties) are computed after the walk, independently per shape, so the native build can
/// spread them over threads; ShapeInfo and the warnings are then written in ShapeId order,
/// exactly as a sequential walk writes them.
struct Pending {
  std::string name;
  bool hasColor = false;
  double rgb[3] = {0, 0, 0};
  // Computed by computeProps():
  int faces = 0, edges = 0;
  bool bboxOk = false, solid = false;
  Box3 bb{{0, 0, 0}, {0, 0, 0}};
  Props p;
  double countMs = 0, bboxMs = 0, propsMs = 0;
};

struct ImportCtx {
  occ::handle<XCAFDoc_ShapeTool> st;
  std::vector<std::string> warnings;
  std::unordered_map<std::string, std::string> shapeOfEntry;  // label entry -> ShapeId
  std::vector<std::unique_ptr<ShapeRec>> recs;
  std::vector<Pending> pending;  // parallel to recs
  double nameColorMs = 0;
};

void computeProps(ShapeRec& rec, Pending& q) {
  auto t0 = Clock::now();
  q.faces = countOf(rec.shape, TopAbs_FACE);
  q.edges = countOf(rec.shape, TopAbs_EDGE);
  q.solid = hasSolid(rec.shape);
  auto t1 = Clock::now();
  q.bboxOk = bboxOf(rec.shape, true, q.bb);
  if (!q.bboxOk) q.bb = Box3{{0, 0, 0}, {0, 0, 0}};
  auto t2 = Clock::now();
  q.p = shapeProps(rec.shape);
  q.countMs = std::chrono::duration<double, std::milli>(t1 - t0).count();
  q.bboxMs = std::chrono::duration<double, std::milli>(t2 - t1).count();
  q.propsMs = msSince(t2);
}

/// Threads for the per-shape properties: CADOC_IMPORT_THREADS, else the hardware's, at most 8.
/// The WASM build is single-threaded.
int propThreads(size_t shapes) {
#ifdef __EMSCRIPTEN__
  (void)shapes;
  return 1;
#else
  int n = 0;
  if (const char* e = std::getenv("CADOC_IMPORT_THREADS")) n = std::atoi(e);
  if (n <= 0) n = std::min(8, int(std::thread::hardware_concurrency()));
  return std::max(1, std::min(n, int(shapes)));
#endif
}

void computeAllProps(ImportCtx& ctx, int threads) {
  const size_t n = ctx.recs.size();
  if (threads <= 1) {
    for (size_t i = 0; i < n; ++i) computeProps(*ctx.recs[i], ctx.pending[i]);
    return;
  }
#ifndef __EMSCRIPTEN__
  // Largest shapes first (by face count from a cheap count), so no thread ends with a giant.
  std::vector<size_t> order(n);
  std::vector<int> weight(n);
  for (size_t i = 0; i < n; ++i) {
    order[i] = i;
    int f = 0;
    for (TopExp_Explorer ex(ctx.recs[i]->shape, TopAbs_FACE); ex.More(); ex.Next()) ++f;
    weight[i] = f;
  }
  std::stable_sort(order.begin(), order.end(), [&](size_t a, size_t b) { return weight[a] > weight[b]; });
  std::atomic<size_t> next{0};
  std::exception_ptr failure;
  std::mutex failMu;
  auto work = [&]() {
    for (size_t k; (k = next.fetch_add(1)) < n;) {
      const size_t i = order[k];
      try {
        computeProps(*ctx.recs[i], ctx.pending[i]);
      } catch (...) {
        std::lock_guard<std::mutex> g(failMu);
        if (!failure) failure = std::current_exception();
      }
    }
  };
  std::vector<std::thread> pool;
  for (int t = 0; t < threads; ++t) pool.emplace_back(work);
  for (auto& t : pool) t.join();
  if (failure) std::rethrow_exception(failure);
#endif
}

/// ShapeInfo JSON and warnings for every shape, in ShapeId order (after computeAllProps).
void finishShapes(ImportCtx& ctx, json::Writer& w) {
  w.beginArray();
  for (size_t i = 0; i < ctx.recs.size(); ++i) {
    ShapeRec& rec = *ctx.recs[i];
    const Pending& q = ctx.pending[i];
    const std::string& sid = rec.id;
    if (!q.bboxOk) ctx.warnings.push_back("shape " + sid + " (" + q.name + ") is empty");
    if (!q.solid && q.faces > 0)
      ctx.warnings.push_back("shape " + sid + " (" + q.name + ") has faces but no solid; volume reported as 0");
    else if (q.faces == 0)
      ctx.warnings.push_back("shape " + sid + " (" + q.name +
                             ") has no faces (wireframe or empty); hlr and sweep obstacles skip it, clearance and "
                             "sweep-as-moving reject it");
    const Box3& bb = q.bb;
    const Props& p = q.p;
    w.beginObject();
    w.key("shapeId").str(sid);
    w.key("name").str(q.name);
    w.key("bbox").beginObject();
    w.key("min").beginArray().num(bb.lo[0]).num(bb.lo[1]).num(bb.lo[2]).endArray();
    w.key("max").beginArray().num(bb.hi[0]).num(bb.hi[1]).num(bb.hi[2]).endArray();
    w.endObject();
    w.key("faceCount").integer(q.faces);
    w.key("edgeCount").integer(q.edges);
    w.key("volume").num(p.volume);
    w.key("area").num(p.area);
    w.key("principalMoments").beginArray().num(p.moments[0]).num(p.moments[1]).num(p.moments[2]).endArray();
    w.key("centroid").beginArray().num(p.centroid[0]).num(p.centroid[1]).num(p.centroid[2]).endArray();
    if (q.hasColor) w.key("color").beginArray().num(q.rgb[0]).num(q.rgb[1]).num(q.rgb[2]).endArray();
    w.endObject();
    rec.faceCount = q.faces;
    rec.edgeCount = q.edges;
    rec.solid = q.solid;
    rec.volume = p.volume;
    rec.diag = std::sqrt(std::pow(bb.hi[0] - bb.lo[0], 2) + std::pow(bb.hi[1] - bb.lo[1], 2) + std::pow(bb.hi[2] - bb.lo[2], 2));
  }
  w.endArray();
}

std::string shapeIdFor(ImportCtx& ctx, const TDF_Label& target, const TDF_Label& instLbl) {
  TCollection_AsciiString entryS;
  TDF_Tool::Entry(target, entryS);
  std::string entry = entryS.ToCString();
  auto found = ctx.shapeOfEntry.find(entry);
  if (found != ctx.shapeOfEntry.end()) return found->second;
  auto t0 = Clock::now();
  const std::string sid = "s" + std::to_string(ctx.recs.size());
  ctx.shapeOfEntry.emplace(std::move(entry), sid);
  Pending q;
  q.name = labelName(target);
  if (q.name.empty()) q.name = labelName(instLbl);
  if (q.name.empty()) q.name = sid;
  q.hasColor = labelColor(target, q.rgb);
  if (!q.hasColor) {
    NCollection_Sequence<TDF_Label> subs;
    XCAFDoc_ShapeTool::GetSubShapes(target, subs);
    for (int i = 1; i <= subs.Length() && !q.hasColor; ++i) q.hasColor = labelColor(subs.Value(i), q.rgb);
  }
  if (!q.hasColor && !instLbl.IsNull()) q.hasColor = labelColor(instLbl, q.rgb);
  auto rec = std::make_unique<ShapeRec>();
  rec->id = sid;
  rec->shape = XCAFDoc_ShapeTool::GetShape(target);
  ctx.recs.push_back(std::move(rec));
  ctx.pending.push_back(std::move(q));
  ctx.nameColorMs += msSince(t0);
  return sid;
}

/// A name that only says what kind of B-rep it is ("Solid1", "Body 2", ""), or nothing.
bool typeOnlyName(std::string n) {
  while (!n.empty() && (std::isdigit((unsigned char)n.back()) || std::isspace((unsigned char)n.back()))) n.pop_back();
  std::transform(n.begin(), n.end(), n.begin(), [](unsigned char c) { return char(std::tolower(c)); });
  static const std::set<std::string> kinds = {"", "solid", "body", "shell", "compound", "compsolid", "part"};
  return kinds.count(n) > 0;
}

struct NamedSolid {
  TopoDS_Shape shape;  // located as in the prototype's shape
  TDF_Label label;     // its sub-shape label
  std::string name;
};

/// The solids of a part whose file names each one ("shroud", "bezel": Fusion 360 writes a
/// component's bodies this way; PiFinder's v2 case, 30 Oct 2026). Such a part is several
/// parts. Empty unless the shape is a compound of two or more solids and every solid carries
/// a name that is more than its kind (sub-shape names need read.stepcaf.subshapes.name).
std::vector<NamedSolid> namedSolids(const TDF_Label& target) {
  const TopoDS_Shape shape = XCAFDoc_ShapeTool::GetShape(target);
  if (shape.IsNull() || shape.ShapeType() != TopAbs_COMPOUND) return {};
  NCollection_Sequence<TDF_Label> subs;
  XCAFDoc_ShapeTool::GetSubShapes(target, subs);
  if (subs.Length() < 2) return {};
  std::vector<NamedSolid> out;
  for (TopoDS_Iterator it(shape); it.More(); it.Next()) {
    const TopoDS_Shape& s = it.Value();
    if (s.ShapeType() != TopAbs_SOLID) return {};
    NamedSolid ns{s, TDF_Label(), ""};
    for (int i = 1; i <= subs.Length(); ++i) {
      if (XCAFDoc_ShapeTool::GetShape(subs.Value(i)).TShape() == s.TShape()) {
        ns.label = subs.Value(i);
        ns.name = labelName(ns.label);
        break;
      }
    }
    if (typeOnlyName(ns.name)) return {};
    out.push_back(std::move(ns));
  }
  if (out.size() < 2) return {};
  return out;
}

/// The ShapeId of one named solid of a part (namedSolids), keyed by its sub-shape label.
std::string shapeIdForSolid(ImportCtx& ctx, const NamedSolid& ns, const TDF_Label& target, const TDF_Label& instLbl) {
  TCollection_AsciiString entryS;
  TDF_Tool::Entry(ns.label, entryS);
  std::string entry = entryS.ToCString();
  auto found = ctx.shapeOfEntry.find(entry);
  if (found != ctx.shapeOfEntry.end()) return found->second;
  auto t0 = Clock::now();
  const std::string sid = "s" + std::to_string(ctx.recs.size());
  ctx.shapeOfEntry.emplace(std::move(entry), sid);
  Pending q;
  q.name = ns.name;
  q.hasColor = labelColor(ns.label, q.rgb);
  if (!q.hasColor) q.hasColor = labelColor(target, q.rgb);
  if (!q.hasColor && !instLbl.IsNull()) q.hasColor = labelColor(instLbl, q.rgb);
  auto rec = std::make_unique<ShapeRec>();
  rec->id = sid;
  rec->shape = ns.shape;
  ctx.recs.push_back(std::move(rec));
  ctx.pending.push_back(std::move(q));
  ctx.nameColorMs += msSince(t0);
  return sid;
}

TDF_Label referred(const TDF_Label& l) {
  TDF_Label t;
  if (XCAFDoc_ShapeTool::IsReference(l) && XCAFDoc_ShapeTool::GetReferredShape(l, t)) return t;
  return l;
}

std::string nodeName(const TDF_Label& l, const TDF_Label& target) {
  std::string n = labelName(l);
  if (n.empty()) n = labelName(target);
  return n.empty() ? "unnamed" : n;
}

/// Segment ids for siblings: "<k>:" only where names repeat, k counting from 0 in file order.
std::vector<std::string> segments(const std::vector<std::string>& names) {
  std::map<std::string, int> counts, seen;
  for (const auto& n : names) ++counts[n];
  std::vector<std::string> out;
  for (const auto& n : names) {
    std::string seg = escapeSegment(n);
    if (counts[n] > 1) seg = std::to_string(seen[n]++) + ":" + seg;
    out.push_back(seg);
  }
  return out;
}

void build(ImportCtx& ctx, json::Writer& w, const TDF_Label& lbl, bool isComponent, const std::string& nodeId,
           const std::string& name) {
  const TDF_Label target = referred(lbl);
  gp_Trsf transform = isComponent ? XCAFDoc_ShapeTool::GetLocation(lbl).Transformation()
                                  : XCAFDoc_ShapeTool::GetShape(lbl).Location().Transformation();
  const bool assembly = XCAFDoc_ShapeTool::IsAssembly(target);
  std::string sid;
  const std::vector<NamedSolid> solids = assembly ? std::vector<NamedSolid>() : namedSolids(target);
  if (!solids.empty()) {
    if (!isComponent) transform = gp_Trsf();  // as a free simple shape: the solids carry its location
  } else if (!assembly) {
    // The prototype keeps whatever location its label carries; that is its "own coordinates".
    sid = shapeIdFor(ctx, target, isComponent ? lbl : TDF_Label());
    if (!isComponent) transform = gp_Trsf();  // a free simple shape: its location is in the prototype
  }
  w.beginObject();
  w.key("id").str(nodeId);
  w.key("name").str(name);
  w.key("transform");
  writeMat4(w, transform);
  w.key("children").beginArray();
  if (assembly) {
    NCollection_Sequence<TDF_Label> comps;
    XCAFDoc_ShapeTool::GetComponents(target, comps, false);
    std::vector<TDF_Label> kids;
    std::vector<std::string> names;
    for (int i = 1; i <= comps.Length(); ++i) {
      kids.push_back(comps.Value(i));
      names.push_back(nodeName(kids.back(), referred(kids.back())));
    }
    const auto segs = segments(names);
    for (size_t i = 0; i < kids.size(); ++i)
      build(ctx, w, kids[i], true, nodeId.empty() ? segs[i] : nodeId + "/" + segs[i], names[i]);
  } else if (!solids.empty()) {
    std::vector<std::string> names;
    for (const auto& ns : solids) names.push_back(ns.name);
    const auto segs = segments(names);
    for (size_t i = 0; i < solids.size(); ++i) {
      const std::string childSid = shapeIdForSolid(ctx, solids[i], target, isComponent ? lbl : TDF_Label());
      w.beginObject();
      w.key("id").str(nodeId.empty() ? segs[i] : nodeId + "/" + segs[i]);
      w.key("name").str(names[i]);
      w.key("transform");
      writeMat4(w, gp_Trsf());
      w.key("children").beginArray().endArray();
      w.key("shapeId").str(childSid);
      w.endObject();
    }
  }
  w.endArray();
  if (!assembly && solids.empty()) w.key("shapeId").str(sid);
  w.endObject();
}

std::string listRepr(const std::set<std::string>& s) {
  std::string out = "[";
  for (const auto& x : s) out += (out.size() > 1 ? ", " : "") + repr(x);
  return out + "]";
}

}  // namespace

std::string doImport(const json::Value& req, const uint8_t* bytes, size_t len, json::Writer& timing) {
  const json::Value* fmtV = req.get("format");
  const std::string fmt = fmtV && fmtV->kind == json::Value::Kind::String ? fmtV->s : "";
  if (fmt != "step" && fmt != "iges" && fmt != "brep")
    throw KernelError("unsupported-format", "format " + (fmtV ? repr(fmt) : std::string("None")) + " is not supported");
  if (fmt == "iges")
    throw KernelError("unsupported-format", "format 'iges' is not supported by this kernel build (it reads STEP and BRep)");

  auto t0 = Clock::now();
  std::vector<std::string> warnings;
  std::string sourceUnit = "unknown";
  occ::handle<TDocStd_Document> doc;
  XCAFApp_Application::GetApplication()->NewDocument("MDTV-XCAF", doc);
  XCAFDoc_DocumentTool::SetLengthUnit(doc, 1.0, UnitsMethods_LengthUnit_Millimeter);
  MemBuf buf(bytes, len);
  std::istream in(&buf);
  double readMs = 0, transferMs = 0;
  struct CloseDoc {
    occ::handle<TDocStd_Document>& d;
    ~CloseDoc() {
      if (!d.IsNull()) XCAFApp_Application::GetApplication()->Close(d);
    }
  } closer{doc};

  if (fmt == "step") {
    STEPCAFControl_Reader r;
    Interface_Static::SetCVal("xstep.cascade.unit", "MM");
    r.SetColorMode(true);
    r.SetNameMode(true);
    r.SetLayerMode(false);
    r.SetPropsMode(false);
    // Names of solids inside a part, onto their sub-shape labels (namedSolids).
    Interface_Static::SetIVal("read.stepcaf.subshapes.name", 1);
    const IFSelect_ReturnStatus st = r.ReadStream("input.step", in);
    if (st != IFSelect_RetDone) throw KernelError("read-failed", "STEP reader status " + std::to_string(int(st)));
    readMs = msSince(t0);
    {
      NCollection_Sequence<TCollection_AsciiString> a, b, c;
      r.ChangeReader().FileUnits(a, b, c);
      std::set<std::string> units;
      for (int i = 1; i <= a.Length(); ++i) units.insert(unitName(a.Value(i).ToCString()));
      if (units.size() == 1) sourceUnit = *units.begin();
      else if (units.size() > 1)
        warnings.push_back("file declares several length units " + listRepr(units) + "; each item is converted from its own");
    }
    auto t1 = Clock::now();
    if (!r.Transfer(doc)) throw KernelError("read-failed", "STEP transfer failed");
    transferMs = msSince(t1);
  } else {  // brep
    TopoDS_Shape shape;
    try {
      BRep_Builder bld;
      BRepTools::Read(shape, in, bld);
    } catch (const Standard_Failure& e) {
      throw KernelError("read-failed", std::string("BRep read failed: ") + e.what());
    }
    if (shape.IsNull()) throw KernelError("read-failed", "BRep file contained no shape");
    XCAFDoc_DocumentTool::ShapeTool(doc->Main())->AddShape(shape, false);
    warnings.push_back("BRep files carry no units; millimetres assumed");
    readMs = msSince(t0);
  }

  auto t2 = Clock::now();
  ImportCtx ctx;
  ctx.st = XCAFDoc_DocumentTool::ShapeTool(doc->Main());
  ctx.warnings = std::move(warnings);
  NCollection_Sequence<TDF_Label> roots;
  ctx.st->GetFreeShapes(roots);
  if (roots.Length() == 0) throw KernelError("read-failed", "the file contains no shapes");

  json::Writer tree;
  if (roots.Length() == 1) {
    const TDF_Label lbl = roots.Value(1);
    const std::string nm = nodeName(lbl, lbl);
    build(ctx, tree, lbl, false, escapeSegment(nm), nm);
  } else {
    tree.beginObject().key("id").str("").key("name").str("").key("transform");
    writeMat4(tree, gp_Trsf());
    tree.key("children").beginArray();
    std::vector<TDF_Label> labels;
    std::vector<std::string> names;
    for (int i = 1; i <= roots.Length(); ++i) {
      labels.push_back(roots.Value(i));
      names.push_back(nodeName(labels.back(), labels.back()));
    }
    const auto segs = segments(names);
    for (size_t i = 0; i < labels.size(); ++i) build(ctx, tree, labels[i], false, segs[i], names[i]);
    tree.endArray().endObject();
  }
  const double walkMs = msSince(t2);
  auto t3 = Clock::now();
  const int threads = propThreads(ctx.recs.size());
  computeAllProps(ctx, threads);
  const double propsWallMs = msSince(t3);
  json::Writer infos;
  finishShapes(ctx, infos);
  const double treeMs = msSince(t2);
  double countMs = 0, bboxMs = 0, propsMs = 0;
  for (const auto& q : ctx.pending) countMs += q.countMs, bboxMs += q.bboxMs, propsMs += q.propsMs;

  // The new document replaces the session (and every cache with it).
  Session& s = session();
  s.clear();
  for (auto& r : ctx.recs) {
    s.byId[r->id] = r.get();
    s.shapes.push_back(std::move(r));
  }

  json::Writer w;
  w.beginObject();
  w.key("type").str("import:ok");
  w.key("protocolVersion").integer(1);
  w.key("sourceUnit").str(sourceUnit);
  w.key("root").raw(tree.out);
  w.key("shapes").raw(infos.out);
  w.key("warnings").beginArray();
  for (const auto& x : ctx.warnings) w.str(x);
  w.endArray();
  w.endObject();
  s.importJson = w.out;
  timing.key("readMs").num(readMs).key("transferMs").num(transferMs).key("treeAndInfoMs").num(treeMs)
      .key("shapes").integer((long long)s.shapes.size());
  // treeAndInfoMs, split: the tree walk (names and colours inside it), then the per-shape
  // properties (wall, on `propThreads` threads), and their summed thread time by kind.
  timing.key("walkMs").num(walkMs).key("nameColorMs").num(ctx.nameColorMs).key("propsWallMs").num(propsWallMs)
      .key("propThreads").integer(threads).key("countCpuMs").num(countMs).key("bboxCpuMs").num(bboxMs)
      .key("massCpuMs").num(propsMs);
  return std::move(w.out);
}

}  // namespace cadoc
