#!/usr/bin/env python3
"""cadoc reference native kernel: protocol v1 (kernel/schema/protocol.ts) over JSON lines.

OCCT 8.0.1 through the OCP bindings (cadquery-ocp). Spike infrastructure: the
pipeline spikes use it until the C++ WASM and native kernels exist. See README.md
in this directory for the wire format and the known limitations.

stdin:  one JSON `Envelope<KernelRequest>` per line.
stdout: one JSON `Envelope<KernelResult | KernelError>` per line, same `id`.
stderr: logs, and anything OCCT itself prints.
"""

from __future__ import annotations

import os
import sys

# OCCT's C++ code prints straight to fd 1 (the STEP reader's "***" banners, for
# example). Keep a private copy of the real stdout for the protocol and point
# fd 1 at stderr, so nothing but protocol lines can ever reach the pipe.
_PROTO = os.fdopen(os.dup(1), "w", encoding="utf-8", buffering=1)
os.dup2(2, 1)
sys.stdout = sys.stderr

import base64  # noqa: E402
import heapq  # noqa: E402
import faulthandler  # noqa: E402
import json  # noqa: E402
import math  # noqa: E402
import tempfile  # noqa: E402
import time  # noqa: E402
import traceback  # noqa: E402
from dataclasses import dataclass, field  # noqa: E402
from typing import Any  # noqa: E402

import numpy as np  # noqa: E402

faulthandler.enable(file=sys.stderr)

from OCP.Bnd import Bnd_Box  # noqa: E402
from OCP.BRep import BRep_Builder, BRep_Tool  # noqa: E402
from OCP.BRepAdaptor import BRepAdaptor_Curve  # noqa: E402
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common  # noqa: E402
from OCP.BRepBndLib import BRepBndLib  # noqa: E402
from OCP.BRepExtrema import BRepExtrema_DistShapeShape  # noqa: E402
from OCP.BRepGProp import BRepGProp  # noqa: E402
from OCP.BRepLib import BRepLib_ToolTriangulatedShape  # noqa: E402
from OCP.BRepMesh import BRepMesh_IncrementalMesh  # noqa: E402
from OCP.BRepTools import BRepTools  # noqa: E402
from OCP.Extrema import Extrema_ExtFlag  # noqa: E402
from OCP.GCPnts import GCPnts_QuasiUniformDeflection  # noqa: E402
from OCP.GeomAbs import GeomAbs_CurveType, GeomAbs_Shape  # noqa: E402
from OCP.GeomConvert import GeomConvert_ApproxCurve, GeomConvert_BSplineCurveToBezierCurve  # noqa: E402
from OCP.GProp import GProp_GProps  # noqa: E402
from OCP.gp import gp_Ax2, gp_Dir, gp_Pnt, gp_Pnt2d, gp_Trsf, gp_Vec  # noqa: E402
from OCP.HLRAlgo import HLRAlgo_Projector  # noqa: E402
from OCP.HLRBRep import (  # noqa: E402
    HLRBRep_Algo,
    HLRBRep_HLRToShape,
    HLRBRep_PolyAlgo,
    HLRBRep_PolyHLRToShape,
    HLRBRep_TypeOfResultingEdge,
)
from OCP.IFSelect import IFSelect_ReturnStatus  # noqa: E402
from OCP.Interface import Interface_Static  # noqa: E402
from OCP.OCP.collections import (  # noqa: E402
    IndexedMap_TopoDS_Shape_TopTools_ShapeMapHasher as ShapeIndexedMap,
    Sequence_TCollection_AsciiString,
    Sequence_TDF_Label,
)
from OCP.Quantity import Quantity_Color  # noqa: E402
from OCP.STEPCAFControl import STEPCAFControl_Reader  # noqa: E402
from OCP.TCollection import TCollection_AsciiString, TCollection_ExtendedString  # noqa: E402
from OCP.TDataStd import TDataStd_Name  # noqa: E402
from OCP.TDF import TDF_Label  # noqa: E402
from OCP.TDocStd import TDocStd_Document  # noqa: E402
from OCP.TopAbs import TopAbs_COMPOUND, TopAbs_EDGE, TopAbs_FACE, TopAbs_Orientation, TopAbs_SOLID  # noqa: E402
from OCP.TopExp import TopExp, TopExp_Explorer  # noqa: E402
from OCP.TopLoc import TopLoc_Location  # noqa: E402
from OCP.TopoDS import TopoDS, TopoDS_Iterator, TopoDS_Shape  # noqa: E402
from OCP.UnitsMethods import UnitsMethods_LengthUnit  # noqa: E402
from OCP.XCAFDoc import XCAFDoc_ColorType, XCAFDoc_DocumentTool, XCAFDoc_ShapeTool  # noqa: E402

PROTOCOL_VERSION = 1


def log(*args: Any) -> None:
    print("[refkernel]", *args, file=sys.stderr, flush=True)


# ------------------------------------------------------------------ errors


class KernelFailure(Exception):
    def __init__(self, code: str, message: str):
        super().__init__(message)
        self.code = code
        self.message = message


def bad(message: str) -> KernelFailure:
    return KernelFailure("bad-request", message)


# ------------------------------------------------------------------ wire encoding
#
# Typed arrays travel as {"$typed": "<Float32Array|Uint32Array|Uint8Array>", "base64": "<little-endian bytes>"}.
# Non-finite numbers travel as {"$float": "Infinity" | "-Infinity" | "NaN"}.
# Everything else is plain JSON. See README.md.

_TYPED_DTYPES = {"Float32Array": "<f4", "Float64Array": "<f8", "Uint32Array": "<u4", "Int32Array": "<i4", "Uint8Array": "u1"}


class Typed:
    """Marks a numpy array for typed-array encoding."""

    __slots__ = ("kind", "array")

    def __init__(self, kind: str, array: np.ndarray):
        self.kind = kind
        self.array = np.ascontiguousarray(array, dtype=_TYPED_DTYPES[kind])


def to_wire(v: Any) -> Any:
    if isinstance(v, float):
        if math.isfinite(v):
            return v
        return {"$float": "NaN" if math.isnan(v) else ("Infinity" if v > 0 else "-Infinity")}
    if isinstance(v, dict):
        return {k: to_wire(x) for k, x in v.items()}
    if isinstance(v, (list, tuple)):
        return [to_wire(x) for x in v]
    if isinstance(v, Typed):
        return {"$typed": v.kind, "base64": base64.b64encode(v.array.tobytes()).decode("ascii")}
    if isinstance(v, (np.floating,)):
        return to_wire(float(v))
    if isinstance(v, (np.integer,)):
        return int(v)
    return v


def from_wire(v: Any) -> Any:
    if isinstance(v, dict):
        if "$typed" in v and "base64" in v:
            kind = v["$typed"]
            if kind not in _TYPED_DTYPES:
                raise bad(f"unknown typed array kind {kind!r}")
            return np.frombuffer(base64.b64decode(v["base64"]), dtype=_TYPED_DTYPES[kind])
        if "$float" in v and len(v) == 1:
            return float(v["$float"].replace("Infinity", "inf"))
        return {k: from_wire(x) for k, x in v.items()}
    if isinstance(v, list):
        return [from_wire(x) for x in v]
    return v


# ------------------------------------------------------------------ small geometry helpers


def mat4_to_trsf(m: Any) -> gp_Trsf:
    if not isinstance(m, (list, tuple)) or len(m) != 16:
        raise bad("transform must be 16 numbers, column-major")
    try:
        m = [float(x) for x in m]
    except (TypeError, ValueError):
        raise bad("transform must be 16 numbers")
    if abs(m[3]) > 1e-9 or abs(m[7]) > 1e-9 or abs(m[11]) > 1e-9 or abs(m[15] - 1) > 1e-9:
        raise bad("transform must be affine (last row 0 0 0 1)")
    t = gp_Trsf()
    try:
        # column-major: element (row r, col c) is m[c*4 + r]
        t.SetValues(m[0], m[4], m[8], m[12], m[1], m[5], m[9], m[13], m[2], m[6], m[10], m[14])
    except Exception as e:  # gp_Trsf only represents similarities
        raise bad(f"transform is not a rigid motion (with optional uniform scale): {e}")
    return t


def trsf_to_mat4(t: gp_Trsf) -> list[float]:
    out = [0.0] * 16
    for c in range(4):
        for r in range(3):
            out[c * 4 + r] = float(t.Value(r + 1, c + 1))
    out[15] = 1.0
    return out


def vec3(v: Any, what: str) -> tuple[float, float, float]:
    if not isinstance(v, (list, tuple)) or len(v) != 3:
        raise bad(f"{what} must be [x, y, z]")
    try:
        return (float(v[0]), float(v[1]), float(v[2]))
    except (TypeError, ValueError):
        raise bad(f"{what} must be numbers")


def unit(v: tuple[float, float, float], what: str) -> tuple[float, float, float]:
    n = math.sqrt(v[0] ** 2 + v[1] ** 2 + v[2] ** 2)
    if n < 1e-12:
        raise bad(f"{what} must not be zero")
    return (v[0] / n, v[1] / n, v[2] / n)


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def bbox_of(shape: TopoDS_Shape, optimal: bool = False) -> tuple[tuple[float, float, float], tuple[float, float, float]] | None:
    box = Bnd_Box()
    if optimal:
        BRepBndLib.AddOptimal_s(shape, box, False, False)
    else:
        BRepBndLib.Add_s(shape, box, False)
    if box.IsVoid():
        return None
    a, b = box.CornerMin(), box.CornerMax()
    return (a.X(), a.Y(), a.Z()), (b.X(), b.Y(), b.Z())


def index_map(shape: TopoDS_Shape, kind) -> ShapeIndexedMap:
    m = ShapeIndexedMap()
    TopExp.MapShapes_s(shape, kind, m)
    return m


def has_solid(shape: TopoDS_Shape) -> bool:
    return TopExp_Explorer(shape, TopAbs_SOLID).More()


# ------------------------------------------------------------------ session state


@dataclass
class ShapeRec:
    shape_id: str
    shape: TopoDS_Shape  # prototype, mm, own coordinates
    info: dict
    face_count: int
    solid: bool
    diag: float
    mesh_key: tuple | None = None
    sweep_mesh: tuple | None = None  # (deflection, positions, triangles) for sweep


@dataclass
class Session:
    shapes: dict[str, ShapeRec] = field(default_factory=dict)


SESSION = Session()


def get_shape(shape_id: Any) -> ShapeRec:
    rec = SESSION.shapes.get(shape_id) if isinstance(shape_id, str) else None
    if rec is None:
        raise bad(f"unknown shapeId {shape_id!r} (import a file first; each import replaces the previous one)")
    return rec


def ensure_mesh(rec: ShapeRec, lin: float, ang: float) -> None:
    key = (round(lin, 9), round(ang, 9))
    if rec.mesh_key == key:
        return
    BRepTools.Clean_s(rec.shape)
    BRepMesh_IncrementalMesh(rec.shape, lin, False, ang, True)
    rec.mesh_key = key


@dataclass
class Placed:
    instance_id: str
    rec: ShapeRec
    trsf: gp_Trsf
    shape: TopoDS_Shape  # rec.shape moved into world


def parse_placed(p: Any, what: str) -> Placed:
    if not isinstance(p, dict):
        raise bad(f"{what} must be a PlacedInstance object")
    iid = p.get("instanceId")
    if not isinstance(iid, str):
        raise bad(f"{what}.instanceId must be a string")
    rec = get_shape(p.get("shapeId"))
    t = mat4_to_trsf(p.get("transform"))
    return Placed(iid, rec, t, rec.shape.Moved(TopLoc_Location(t)))


# ------------------------------------------------------------------ import

_UNIT_NAMES = {
    "millimetre": "mm", "millimeter": "mm", "mm": "mm",
    "centimetre": "cm", "centimeter": "cm", "cm": "cm",
    "metre": "m", "meter": "m", "m": "m",
    "inch": "in", "in": "in",
    "foot": "ft", "feet": "ft", "ft": "ft",
}


def label_name(lbl: TDF_Label) -> str | None:
    if lbl.IsNull():
        return None
    # FindAttribute segfaults (OCP 8.0.1) on some labels that have no name, e.g. the
    # unnamed shape labels of NIST CTC 02; asking IsAttribute first avoids the call.
    if not lbl.IsAttribute(TDataStd_Name.GetID_s()):
        return None
    n = TDataStd_Name()
    if lbl.FindAttribute(TDataStd_Name.GetID_s(), n):
        s = TCollection_AsciiString(n.Get()).ToCString()
        # STEPCAF invents "=>[0:1:1:18]" for unnamed instances; that is not a name.
        if s and not s.startswith("=>["):
            return s
    return None


def label_color(lbl: TDF_Label) -> tuple[float, float, float] | None:
    c = Quantity_Color()
    for t in (XCAFDoc_ColorType.XCAFDoc_ColorSurf, XCAFDoc_ColorType.XCAFDoc_ColorGen, XCAFDoc_ColorType.XCAFDoc_ColorCurv):
        if XCAFDoc_DocumentTool.ColorTool_s(lbl).GetColor_s(lbl, t, c):
            # The file's numbers, unconverted (STEP stores plain RGB; we treat it as sRGB).
            return (round(c.Red(), 6), round(c.Green(), 6), round(c.Blue(), 6))
    return None


def shape_props(shape: TopoDS_Shape) -> tuple[float, float, tuple, tuple]:
    """volume, area, centroid, principal moments (ascending, unit density, about the centroid)."""
    area_p = GProp_GProps()
    BRepGProp.SurfaceProperties_s(shape, area_p)
    area = abs(area_p.Mass())
    vol_p = GProp_GProps()
    BRepGProp.VolumeProperties_s(shape, vol_p)
    volume = vol_p.Mass()
    use = vol_p
    if not has_solid(shape) or abs(volume) < 1e-12:
        # No solid: centroid and moments of the surface instead (documented).
        volume = 0.0
        use = area_p
    if abs(use.Mass()) < 1e-15:
        return abs(volume), area, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)
    c = use.CentreOfMass()
    moments = tuple(sorted(abs(x) for x in use.PrincipalProperties().Moments()))
    return abs(volume), area, (c.X(), c.Y(), c.Z()), moments


def escape_segment(name: str) -> str:
    return name.replace("%", "%25").replace("/", "%2F")


_TYPE_ONLY_NAMES = {"", "solid", "body", "shell", "compound", "compsolid", "part"}


def type_only_name(n: str | None) -> bool:
    """A name that only says what kind of B-rep it is ("Solid1", "Body 2", ""), or none."""
    return (n or "").rstrip("0123456789 \t\r\n").lower() in _TYPE_ONLY_NAMES


def named_solids(target: TDF_Label) -> list:
    """The solids of a part whose file names each one ("shroud", "bezel": Fusion 360 writes a
    component's bodies this way; PiFinder's v2 case, 30 Oct 2026), as (sub-shape label, solid,
    name). Such a part is several parts. Empty unless the shape is a compound of two or more
    solids and every solid carries a name that is more than its kind."""
    st_ = XCAFDoc_ShapeTool
    shape = st_.GetShape_s(target)
    if shape.IsNull() or shape.ShapeType() != TopAbs_COMPOUND:
        return []
    subs = Sequence_TDF_Label()
    st_.GetSubShapes_s(target, subs)
    if subs.Length() < 2:
        return []
    sub_list = [subs.Value(i) for i in range(1, subs.Length() + 1)]
    out = []
    it = TopoDS_Iterator(shape)
    while it.More():
        s = it.Value()
        if s.ShapeType() != TopAbs_SOLID:
            return []
        lab, name = TDF_Label(), None
        for sl in sub_list:
            if st_.GetShape_s(sl).TShape() == s.TShape():
                lab, name = sl, label_name(sl)
                break
        if type_only_name(name):
            return []
        out.append((lab, s, name))
        it.Next()
    return out if len(out) >= 2 else []


def read_document(fmt: str, path: str, warnings: list[str]):
    doc = TDocStd_Document(TCollection_ExtendedString("XmlXCAF"))
    XCAFDoc_DocumentTool.SetLengthUnit_s(doc, 1.0, UnitsMethods_LengthUnit.UnitsMethods_LengthUnit_Millimeter)
    Interface_Static.SetCVal_s("xstep.cascade.unit", "MM")
    source_unit = "unknown"
    if fmt == "step":
        r = STEPCAFControl_Reader()
        r.SetColorMode(True)
        r.SetNameMode(True)
        r.SetLayerMode(False)
        r.SetPropsMode(False)
        # Names of solids inside a part, onto their sub-shape labels (named_solids).
        Interface_Static.SetIVal_s("read.stepcaf.subshapes.name", 1)
        st = r.ReadFile(path)
        if st != IFSelect_ReturnStatus.IFSelect_RetDone:
            raise KernelFailure("read-failed", f"STEP reader status {st}")
        try:
            a, b, c = Sequence_TCollection_AsciiString(), Sequence_TCollection_AsciiString(), Sequence_TCollection_AsciiString()
            r.Reader().FileUnits(a, b, c)
            units = {_UNIT_NAMES.get(a.Value(i).ToCString().strip().lower(), "unknown") for i in range(1, a.Length() + 1)}
            if len(units) == 1:
                source_unit = units.pop()
            elif len(units) > 1:
                warnings.append(f"file declares several length units {sorted(units)}; each item is converted from its own")
        except Exception as e:  # pragma: no cover - diagnostic only
            warnings.append(f"could not read file units: {e}")
        if not r.Transfer(doc):
            raise KernelFailure("read-failed", "STEP transfer failed")
    elif fmt == "iges":
        from OCP.IGESCAFControl import IGESCAFControl_Reader

        r = IGESCAFControl_Reader()
        r.SetColorMode(True)
        r.SetNameMode(True)
        st = r.ReadFile(path)
        if st != IFSelect_ReturnStatus.IFSelect_RetDone:
            raise KernelFailure("read-failed", f"IGES reader status {st}")
        try:
            gs = r.IGESModel().GlobalSection()
            source_unit = _UNIT_NAMES.get(gs.UnitName().ToCString().strip().lower().replace("'", ""), "unknown")
        except Exception:
            pass
        if not r.Transfer(doc):
            raise KernelFailure("read-failed", "IGES transfer failed")
    elif fmt == "brep":
        shape = TopoDS_Shape()
        try:
            BRepTools.Read_s(shape, path, BRep_Builder())
        except Exception as e:
            raise KernelFailure("read-failed", f"BRep read failed: {e}")
        if shape.IsNull():
            raise KernelFailure("read-failed", "BRep file contained no shape")
        XCAFDoc_DocumentTool.ShapeTool_s(doc.Main()).AddShape(shape, False)
        warnings.append("BRep files carry no units; millimetres assumed")
    else:
        raise KernelFailure("unsupported-format", f"format {fmt!r} is not supported")
    return doc, source_unit


def do_import(req: dict) -> dict:
    fmt = req.get("format")
    data = req.get("bytes")
    # Native-transport extension (not in protocol v1): {"path": "/abs/file.step"} instead of
    # bytes, so a large file never crosses the pipe. The kernel only reads the file.
    src_path = req.get("path")
    if data is None and isinstance(src_path, str):
        if not os.path.isabs(src_path):
            raise bad("import.path must be absolute")
        if not os.path.isfile(src_path):
            raise KernelFailure("read-failed", f"no such file: {src_path}")
    else:
        src_path = None
        if isinstance(data, list):
            data = np.asarray(data, dtype=np.uint8)
        if not isinstance(data, np.ndarray):
            raise bad("import.bytes must be a Uint8Array")
    if fmt not in ("step", "iges", "brep"):
        raise KernelFailure("unsupported-format", f"format {fmt!r} is not supported")
    warnings: list[str] = []
    if src_path is not None:
        doc, source_unit = read_document(fmt, src_path, warnings)
    else:
        suffix = {"step": ".stp", "iges": ".igs", "brep": ".brep"}[fmt]
        with tempfile.NamedTemporaryFile(suffix=suffix, delete=False) as f:
            f.write(data.tobytes())
            tmp = f.name
        try:
            doc, source_unit = read_document(fmt, tmp, warnings)
        finally:
            os.unlink(tmp)

    st = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    shapes: dict[str, ShapeRec] = {}
    shape_of_tag: dict[str, str] = {}

    def shape_record(sid: str, shp: TopoDS_Shape, name: str, color) -> ShapeRec:
        faces = index_map(shp, TopAbs_FACE).Size()
        edges = index_map(shp, TopAbs_EDGE).Size()
        bb = bbox_of(shp, optimal=True)
        if bb is None:
            bb = ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
            warnings.append(f"shape {sid} ({name}) is empty")
        solid = has_solid(shp)
        if not solid and faces > 0:
            warnings.append(f"shape {sid} ({name}) has faces but no solid; volume reported as 0")
        elif faces == 0:
            warnings.append(f"shape {sid} ({name}) has no faces (wireframe or empty); hlr and sweep obstacles skip it, clearance and sweep-as-moving reject it")
        volume, area, centroid, moments = shape_props(shp)
        info = {
            "shapeId": sid,
            "name": name,
            "bbox": {"min": list(bb[0]), "max": list(bb[1])},
            "faceCount": faces,
            "edgeCount": edges,
            "volume": volume,
            "area": area,
            "principalMoments": list(moments),
            "centroid": list(centroid),
        }
        if color is not None:
            info["color"] = list(color)
        diag = math.dist(bb[0], bb[1])
        return ShapeRec(sid, shp, info, faces, solid, diag)

    def shape_id_for(target: TDF_Label, inst_lbl: TDF_Label) -> str:
        key = TCollection_AsciiString()
        from OCP.TDF import TDF_Tool

        TDF_Tool.Entry_s(target, key)
        entry = key.ToCString()
        if entry in shape_of_tag:
            return shape_of_tag[entry]
        sid = f"s{len(shapes)}"
        shape_of_tag[entry] = sid
        shp = st.GetShape_s(target)
        name = label_name(target) or label_name(inst_lbl) or sid
        color = label_color(target)
        if color is None:
            subs = Sequence_TDF_Label()
            st.GetSubShapes_s(target, subs)
            for i in range(1, subs.Length() + 1):
                color = label_color(subs.Value(i))
                if color is not None:
                    break
        if color is None and not inst_lbl.IsNull():
            color = label_color(inst_lbl)
        shapes[sid] = shape_record(sid, shp, name, color)
        return sid

    def shape_id_for_solid(sub_lbl: TDF_Label, solid: TopoDS_Shape, name: str, target: TDF_Label, inst_lbl: TDF_Label) -> str:
        key = TCollection_AsciiString()
        from OCP.TDF import TDF_Tool

        TDF_Tool.Entry_s(sub_lbl, key)
        entry = key.ToCString()
        if entry in shape_of_tag:
            return shape_of_tag[entry]
        sid = f"s{len(shapes)}"
        shape_of_tag[entry] = sid
        color = label_color(sub_lbl)
        if color is None:
            color = label_color(target)
        if color is None and not inst_lbl.IsNull():
            color = label_color(inst_lbl)
        shapes[sid] = shape_record(sid, solid, name, color)
        return sid

    def node_name(lbl: TDF_Label, target: TDF_Label) -> str:
        return label_name(lbl) or label_name(target) or "unnamed"

    def build(lbl: TDF_Label, is_component: bool, node_id: str, name: str) -> dict:
        target = TDF_Label()
        if st.IsReference_s(lbl):
            st.GetReferredShape_s(lbl, target)
        else:
            target = lbl
        if is_component:
            transform = trsf_to_mat4(st.GetLocation_s(lbl).Transformation())
        else:
            transform = trsf_to_mat4(st.GetShape_s(lbl).Location().Transformation())
        node: dict = {"id": node_id, "name": name, "transform": transform, "children": []}
        if st.IsAssembly_s(target):
            comps = Sequence_TDF_Label()
            st.GetComponents_s(target, comps, False)
            kids = [comps.Value(i) for i in range(1, comps.Length() + 1)]
            names = []
            for k in kids:
                kt = TDF_Label()
                if st.IsReference_s(k):
                    st.GetReferredShape_s(k, kt)
                else:
                    kt = k
                names.append(node_name(k, kt))
            counts: dict[str, int] = {}
            for n in names:
                counts[n] = counts.get(n, 0) + 1
            seen: dict[str, int] = {}
            for k, n in zip(kids, names):
                seg = escape_segment(n)
                if counts[n] > 1:
                    seg = f"{seen.get(n, 0)}:{seg}"
                    seen[n] = seen.get(n, 0) + 1
                child_id = f"{node_id}/{seg}" if node_id else seg
                node["children"].append(build(k, True, child_id, n))
        elif solids := named_solids(target):
            if not is_component:
                node["transform"] = trsf_to_mat4(gp_Trsf())  # as a free simple shape: the solids carry its location
            names = [n for _, _, n in solids]
            counts = {}
            for n in names:
                counts[n] = counts.get(n, 0) + 1
            seen = {}
            for sub_lbl, solid, n in solids:
                seg = escape_segment(n)
                if counts[n] > 1:
                    seg = f"{seen.get(n, 0)}:{seg}"
                    seen[n] = seen.get(n, 0) + 1
                sid = shape_id_for_solid(sub_lbl, solid, n, target, lbl if is_component else TDF_Label())
                node["children"].append({"id": f"{node_id}/{seg}" if node_id else seg, "name": n, "transform": trsf_to_mat4(gp_Trsf()), "children": [], "shapeId": sid})
        else:
            # The prototype keeps whatever location its label carries; that is its "own coordinates".
            proto_lbl = target
            sid = shape_id_for(proto_lbl, lbl if is_component else TDF_Label())
            node["shapeId"] = sid
            if not is_component:
                # A free simple shape: its own location is already part of the prototype.
                node["transform"] = trsf_to_mat4(gp_Trsf())
        return node

    roots = Sequence_TDF_Label()
    st.GetFreeShapes(roots)
    if roots.Length() == 0:
        raise KernelFailure("read-failed", "the file contains no shapes")
    root_labels = [roots.Value(i) for i in range(1, roots.Length() + 1)]
    if len(root_labels) == 1:
        lbl = root_labels[0]
        nm = node_name(lbl, lbl)
        root = build(lbl, False, escape_segment(nm), nm)
    else:
        root = {"id": "", "name": "", "transform": trsf_to_mat4(gp_Trsf()), "children": []}
        names = [node_name(lb, lb) for lb in root_labels]
        counts: dict[str, int] = {}
        for n in names:
            counts[n] = counts.get(n, 0) + 1
        seen: dict[str, int] = {}
        for lb, n in zip(root_labels, names):
            seg = escape_segment(n)
            if counts[n] > 1:
                seg = f"{seen.get(n, 0)}:{seg}"
                seen[n] = seen.get(n, 0) + 1
            root["children"].append(build(lb, False, seg, n))

    SESSION.shapes = shapes
    return {
        "type": "import:ok",
        "protocolVersion": PROTOCOL_VERSION,
        "sourceUnit": source_unit,
        "root": root,
        "shapes": [r.info for r in shapes.values()],
        "warnings": warnings,
    }


# ------------------------------------------------------------------ mesh


def tri_mesh(rec: ShapeRec, lin: float, ang: float):
    """Triangulate a prototype. Returns positions (n,3), vertex normals (n,3), triangles (m,3),
    and face index per triangle. Vertices are per face (not shared across faces); winding
    and normals are flipped on REVERSED faces, so cross(p1-p0, p2-p0) points out of a solid.
    The face index is the 0-based position in TopExp.MapShapes(shape, FACE) order."""
    ensure_mesh(rec, lin, ang)
    faces = index_map(rec.shape, TopAbs_FACE)
    pos_parts, nrm_parts, idx_parts, fot_parts = [], [], [], []
    base = 0
    for fi in range(1, faces.Size() + 1):
        face = TopoDS.Face(faces.FindKey(fi))
        loc = TopLoc_Location()
        tri = BRep_Tool.Triangulation_s(face, loc)
        if tri is None or tri.NbTriangles() == 0:
            continue
        if not tri.HasNormals():
            BRepLib_ToolTriangulatedShape.ComputeNormals_s(face, tri)
        n_nodes = tri.NbNodes()
        trsf = loc.Transformation()
        pts = np.empty((n_nodes, 3), dtype=np.float64)
        nrm = np.empty((n_nodes, 3), dtype=np.float64)
        for i in range(1, n_nodes + 1):
            p = tri.Node(i).Transformed(trsf)
            pts[i - 1] = (p.X(), p.Y(), p.Z())
            d = tri.Normal(i)
            d.Transform(trsf)
            nrm[i - 1] = (d.X(), d.Y(), d.Z())
        n_tri = tri.NbTriangles()
        idx = np.empty((n_tri, 3), dtype=np.int64)
        for t in range(1, n_tri + 1):
            idx[t - 1] = tri.Triangle(t).Get()
        idx -= 1
        if face.Orientation() == TopAbs_Orientation.TopAbs_REVERSED:
            idx = idx[:, [0, 2, 1]]
            nrm = -nrm
        pos_parts.append(pts)
        nrm_parts.append(nrm)
        idx_parts.append(idx + base)
        fot_parts.append(np.full(n_tri, fi - 1, dtype=np.uint32))
        base += n_nodes
    if not pos_parts:
        return np.zeros((0, 3)), np.zeros((0, 3)), np.zeros((0, 3), dtype=np.int64), np.zeros(0, dtype=np.uint32)
    return np.concatenate(pos_parts), np.concatenate(nrm_parts), np.concatenate(idx_parts), np.concatenate(fot_parts)


def do_mesh(req: dict) -> dict:
    ids = req.get("shapeIds")
    if not isinstance(ids, list):
        raise bad("mesh.shapeIds must be an array")
    lin = float(req.get("linearDeflection", 0))
    ang = float(req.get("angularDeflection", 0))
    if not (lin > 0 and ang > 0):
        raise bad("mesh.linearDeflection and angularDeflection must be > 0")
    out = []
    for sid in ids:
        pos, nrm, idx, fot = tri_mesh(get_shape(sid), lin, ang)
        out.append({
            "shapeId": sid,
            "positions": Typed("Float32Array", pos.reshape(-1)),
            "normals": Typed("Float32Array", nrm.reshape(-1)),
            "indices": Typed("Uint32Array", idx.reshape(-1)),
            "faceOfTriangle": Typed("Uint32Array", fot),
        })
    return {"type": "mesh:ok", "meshes": out}


# ------------------------------------------------------------------ curve emission (HLR output -> Curve2D)

# Tolerances in view mm. HLR output is at 1:1, so 1 micron is far below anything a drawing can show.
CURVE_TOL = 1e-3
TWO_PI = 2 * math.pi


class CurveEmitter:
    def __init__(self, offset: tuple[float, float], polyline_deflection: float):
        self.ox, self.oy = offset
        self.defl = polyline_deflection
        self.stats = {"line": 0, "arc": 0, "ellipse": 0, "bezier": 0, "polyline": 0, "recognised": 0}

    def p2(self, p: gp_Pnt) -> tuple[float, float]:
        return (p.X() - self.ox, p.Y() - self.oy)

    def emit(self, edge) -> dict | None:
        ad = BRepAdaptor_Curve(edge)
        f, l = ad.FirstParameter(), ad.LastParameter()
        if not (math.isfinite(f) and math.isfinite(l)) or l - f <= 1e-12:
            return None
        rev = edge.Orientation() == TopAbs_Orientation.TopAbs_REVERSED
        typ = ad.GetType()
        c = None
        if typ == GeomAbs_CurveType.GeomAbs_Line:
            a, b = self.p2(ad.Value(f)), self.p2(ad.Value(l))
            if rev:
                a, b = b, a
            if math.dist(a, b) < 1e-9:
                return None
            c = {"kind": "line", "a": list(a), "b": list(b)}
        elif typ == GeomAbs_CurveType.GeomAbs_Circle:
            c = self._circle(ad, f, l, rev)
        elif typ == GeomAbs_CurveType.GeomAbs_Ellipse:
            c = self._ellipse(ad, f, l, rev)
        if c is None:
            c = self._general(ad, f, l, rev)
        self.stats[c["kind"]] += 1
        return c

    # -- exact conics -----------------------------------------------------

    def _circle(self, ad, f, l, rev):
        circ = ad.Circle()
        ctr = circ.Location()
        cx, cy = ctr.X() - self.ox, ctr.Y() - self.oy
        r = circ.Radius()
        xd = circ.XAxis().Direction()
        phi0 = math.atan2(xd.Y(), xd.X())
        if circ.Axis().Direction().Z() >= 0:
            s, e, ccw = phi0 + f, phi0 + l, True
        else:
            s, e, ccw = phi0 - f, phi0 - l, False
        if rev:
            s, e, ccw = e, s, not ccw
        s, e = self._norm(s, e)
        arc = {"kind": "arc", "center": [cx, cy], "radius": r, "startAngle": s, "endAngle": e, "ccw": ccw}
        return arc if self._check(arc, ad, f, l, rev) else None

    def _ellipse(self, ad, f, l, rev):
        el = ad.Ellipse()
        ctr = el.Location()
        xd = el.XAxis().Direction()
        rot = math.atan2(xd.Y(), xd.X())
        if el.Axis().Direction().Z() >= 0:
            s, e, ccw = f, l, True
        else:
            s, e, ccw = -f, -l, False
        if rev:
            s, e, ccw = e, s, not ccw
        s, e = self._norm(s, e)
        c = {
            "kind": "ellipse",
            "center": [ctr.X() - self.ox, ctr.Y() - self.oy],
            "majorRadius": el.MajorRadius(),
            "minorRadius": el.MinorRadius(),
            "rotation": rot,
            "startAngle": s,
            "endAngle": e,
            "ccw": ccw,
        }
        return c if self._check(c, ad, f, l, rev) else None

    @staticmethod
    def _norm(s: float, e: float) -> tuple[float, float]:
        """startAngle into (-pi, pi]; endAngle keeps the signed sweep (|sweep| <= 2pi)."""
        k = math.floor((s + math.pi) / TWO_PI)
        shift = k * TWO_PI
        s2 = s - shift
        if s2 <= -math.pi:
            s2 += TWO_PI
            shift -= TWO_PI
        return s2, e - shift

    @staticmethod
    def eval_conic(c: dict, ang: float) -> tuple[float, float]:
        if c["kind"] == "arc":
            return (c["center"][0] + c["radius"] * math.cos(ang), c["center"][1] + c["radius"] * math.sin(ang))
        a, b, rot = c["majorRadius"], c["minorRadius"], c["rotation"]
        x, y = a * math.cos(ang), b * math.sin(ang)
        cr, sr = math.cos(rot), math.sin(rot)
        return (c["center"][0] + x * cr - y * sr, c["center"][1] + x * sr + y * cr)

    def _check(self, c: dict, ad, f, l, rev) -> bool:
        """Re-evaluate start, middle and end from the emitted parameters against the real edge."""
        s, e = c["startAngle"], c["endAngle"]
        if c["ccw"] and e < s - 1e-12 or (not c["ccw"]) and e > s + 1e-12:
            return False
        ps = [ad.Value(f), ad.Value(0.5 * (f + l)), ad.Value(l)]
        if rev:
            ps.reverse()
        for t, p in zip((0.0, 0.5, 1.0), ps):
            q = self.eval_conic(c, s + t * (e - s))
            if math.dist(q, self.p2(p)) > 10 * CURVE_TOL:
                return False
        return True

    # -- everything else: recognise lines and circular arcs, else cubic Beziers, else polyline ---------

    def _samples(self, ad, f, l, rev, n=33):
        pts = [self.p2(ad.Value(f + (l - f) * i / (n - 1))) for i in range(n)]
        if rev:
            pts.reverse()
        return pts

    def _general(self, ad, f, l, rev):
        pts = self._samples(ad, f, l, rev)
        a, b = pts[0], pts[-1]
        chord = math.dist(a, b)
        if chord > 1e-9:
            ux, uy = (b[0] - a[0]) / chord, (b[1] - a[1]) / chord
            if all(abs((p[0] - a[0]) * uy - (p[1] - a[1]) * ux) <= CURVE_TOL for p in pts):
                # Collinear. Edge-on circles come out of HLR as degenerate B-splines that
                # double back; emit the extent actually covered, not just the endpoints.
                ts = [(p[0] - a[0]) * ux + (p[1] - a[1]) * uy for p in pts]
                lo, hi = min(ts), max(ts)
                if lo >= -CURVE_TOL and hi <= chord + CURVE_TOL:
                    self.stats["recognised"] += 1
                    return {"kind": "line", "a": list(a), "b": list(b)}
        arc = self._fit_arc(pts)
        if arc is not None:
            self.stats["recognised"] += 1
            return arc
        # Many exporters write circles as (rational) B-splines; seen at an angle they
        # project to B-splines that are exactly ellipses. Recognise them.
        ell = self._fit_ellipse(self._samples(ad, f, l, rev, 49))
        if ell is not None:
            self.stats["recognised"] += 1
            return ell
        bez = self._bezier(ad, f, l, rev)
        if bez is not None:
            return bez
        return self._polyline(ad, f, l, rev)

    def _fit_arc(self, pts):
        a, m, b = pts[0], pts[len(pts) // 2], pts[-1]
        ax, ay = a
        bx, by = m
        cx, cy = b
        d = 2 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by))
        if abs(d) < 1e-12:
            return None
        ux = ((ax * ax + ay * ay) * (by - cy) + (bx * bx + by * by) * (cy - ay) + (cx * cx + cy * cy) * (ay - by)) / d
        uy = ((ax * ax + ay * ay) * (cx - bx) + (bx * bx + by * by) * (ax - cx) + (cx * cx + cy * cy) * (bx - ax)) / d
        r = math.dist((ux, uy), a)
        span = max(math.dist(a, m), math.dist(m, b), math.dist(a, b))
        if r > 1e4 * max(span, 1e-9):
            return None
        tol = max(CURVE_TOL, 1e-6 * r)
        if any(abs(math.dist((ux, uy), p) - r) > tol for p in pts):
            return None
        # Unwrap angles; they must move monotonically in one direction.
        angs = [math.atan2(p[1] - uy, p[0] - ux) for p in pts]
        un = [angs[0]]
        for x in angs[1:]:
            dlt = x - un[-1]
            dlt = (dlt + math.pi) % TWO_PI - math.pi
            un.append(un[-1] + dlt)
        steps = [un[i + 1] - un[i] for i in range(len(un) - 1)]
        if all(s > 0 for s in steps):
            ccw = True
        elif all(s < 0 for s in steps):
            ccw = False
        else:
            return None
        s, e = self._norm(un[0], un[-1])
        if abs(e - s) > TWO_PI + 1e-9:
            return None
        return {"kind": "arc", "center": [ux, uy], "radius": r, "startAngle": s, "endAngle": e, "ccw": ccw}

    @staticmethod
    def _unwrap_monotonic(angs: list[float]) -> tuple[list[float], bool] | None:
        un = [angs[0]]
        for x in angs[1:]:
            dlt = (x - un[-1] + math.pi) % TWO_PI - math.pi
            un.append(un[-1] + dlt)
        steps = [un[i + 1] - un[i] for i in range(len(un) - 1)]
        if all(s > 0 for s in steps):
            return un, True
        if all(s < 0 for s in steps):
            return un, False
        return None

    def _fit_ellipse(self, pts):
        """Direct least-squares conic fit (Halir and Flusser), accepted only if every sample is within tolerance."""
        P = np.asarray(pts, dtype=np.float64)
        mean = P.mean(axis=0)
        scale = float(np.abs(P - mean).max())
        if scale < 1e-9:
            return None
        Q = (P - mean) / scale
        x, y = Q[:, 0], Q[:, 1]
        D1 = np.column_stack([x * x, x * y, y * y])
        D2 = np.column_stack([x, y, np.ones_like(x)])
        S1, S2, S3 = D1.T @ D1, D1.T @ D2, D2.T @ D2
        try:
            T = -np.linalg.solve(S3, S2.T)
            M = S1 + S2 @ T
            M = np.array([M[2] / 2, -M[1], M[0] / 2])
            w, V = np.linalg.eig(M)
        except np.linalg.LinAlgError:
            return None
        V = np.real(V)
        cond = 4 * V[0] * V[2] - V[1] ** 2
        ok = np.where(cond > 0)[0]
        if len(ok) == 0:
            return None
        a1 = V[:, ok[0]]
        A, B, C = a1
        Dq, Eq, F = T @ a1
        det = 4 * A * C - B * B
        if det <= 0:
            return None
        x0 = (B * Eq - 2 * C * Dq) / det
        y0 = (B * Dq - 2 * A * Eq) / det
        Fc = A * x0 * x0 + B * x0 * y0 + C * y0 * y0 + Dq * x0 + Eq * y0 + F
        lam, vec = np.linalg.eigh(np.array([[A, B / 2], [B / 2, C]]))
        if Fc == 0 or (-Fc / lam[0]) <= 0 or (-Fc / lam[1]) <= 0:
            return None
        ax0, ax1 = math.sqrt(-Fc / lam[0]), math.sqrt(-Fc / lam[1])
        # major axis: the larger semi-axis
        if ax0 >= ax1:
            a, b, v = ax0, ax1, vec[:, 0]
        else:
            a, b, v = ax1, ax0, vec[:, 1]
        a, b = a * scale, b * scale
        cx, cy = x0 * scale + mean[0], y0 * scale + mean[1]
        span = float(np.ptp(P, axis=0).max())
        if a > 1e3 * max(span, 1e-9) or b < 1e-9:
            return None
        rot = math.atan2(v[1], v[0])
        cr, sr = math.cos(rot), math.sin(rot)
        dx, dy = P[:, 0] - cx, P[:, 1] - cy
        u = (dx * cr + dy * sr) / a
        w2 = (-dx * sr + dy * cr) / b
        th = np.arctan2(w2, u)
        mono = self._unwrap_monotonic([float(t) for t in th])
        if mono is None:
            return None
        un, ccw = mono
        s, e = self._norm(un[0], un[-1])
        if abs(e - s) > TWO_PI + 1e-9:
            return None
        c = {"kind": "ellipse", "center": [cx, cy], "majorRadius": a, "minorRadius": b, "rotation": rot,
             "startAngle": s, "endAngle": e, "ccw": ccw}
        tol = max(CURVE_TOL, 1e-6 * a)
        for t, p in zip(un, pts):
            q = self.eval_conic(c, t - (un[0] - s))
            if math.dist(q, p) > tol:
                return None
        return c

    def _bezier(self, ad, f, l, rev):
        try:
            approx = GeomConvert_ApproxCurve(ad, CURVE_TOL, GeomAbs_Shape.GeomAbs_C1, 200, 3)
            if not approx.HasResult() or approx.MaxError() > 5 * CURVE_TOL:
                return None
            bs = approx.Curve()
            if bs.IsRational():
                return None
            if bs.Degree() < 3:
                bs.IncreaseDegree(3)
            conv = GeomConvert_BSplineCurveToBezierCurve(bs)
            pts: list[tuple[float, float]] = []
            for i in range(1, conv.NbArcs() + 1):
                arc = conv.Arc(i)
                if arc.Degree() != 3:
                    return None
                poles = [self.p2(arc.Pole(k)) for k in range(1, 5)]
                pts.extend(poles if not pts else poles[1:])
            if rev:
                pts.reverse()
            if len(pts) < 4:
                return None
            return {"kind": "bezier", "points": [list(p) for p in pts]}
        except Exception as e:
            log("bezier approximation failed:", e)
            return None

    def _polyline(self, ad, f, l, rev):
        q = GCPnts_QuasiUniformDeflection(ad, self.defl, f, l)
        if q.IsDone() and q.NbPoints() >= 2:
            pts = [self.p2(q.Value(i)) for i in range(1, q.NbPoints() + 1)]
        else:
            pts = self._samples(ad, f, l, False, 33)
            if rev:
                pts.reverse()
            return {"kind": "polyline", "points": [list(p) for p in pts]}
        if rev:
            pts.reverse()
        return {"kind": "polyline", "points": [list(p) for p in pts]}


# ------------------------------------------------------------------ hlr

_EXACT_CLASSES = [
    (HLRBRep_TypeOfResultingEdge.HLRBRep_OutLine, "outline", False),
    (HLRBRep_TypeOfResultingEdge.HLRBRep_Sharp, "sharp", False),
    (HLRBRep_TypeOfResultingEdge.HLRBRep_Rg1Line, "smooth", True),
    (HLRBRep_TypeOfResultingEdge.HLRBRep_RgNLine, "seam", True),
]


def make_projector(view: Any) -> tuple[HLRAlgo_Projector, tuple[float, float]]:
    if not isinstance(view, dict):
        raise bad("hlr.view must be a ViewSpec")
    d = unit(vec3(view.get("viewDirection"), "view.viewDirection"), "view.viewDirection")
    up = unit(vec3(view.get("up"), "view.up"), "view.up")
    right = cross(up, d)
    if math.sqrt(dot(right, right)) < 1e-9:
        raise bad("view.up must not be parallel to view.viewDirection")
    right = unit(right, "right")
    proj_kind = view.get("projection", "orthographic")
    if proj_kind == "orthographic":
        return HLRAlgo_Projector(gp_Ax2(gp_Pnt(0, 0, 0), gp_Dir(*d), gp_Dir(*right))), (0.0, 0.0)
    if proj_kind == "perspective":
        eye = vec3(view.get("eye"), "view.eye")
        focus = dot(eye, d)
        if focus <= 1e-9:
            raise bad("perspective: the eye must be in front of the world origin along viewDirection")
        # Image plane passes through the world origin, perpendicular to viewDirection; the eye
        # sits `focus` in front of it. Then shift so the world origin projects to (0, 0).
        o = (eye[0] - focus * d[0], eye[1] - focus * d[1], eye[2] - focus * d[2])
        pr = HLRAlgo_Projector(gp_Ax2(gp_Pnt(*o), gp_Dir(*d), gp_Dir(*right)), focus)
        q = gp_Pnt2d()
        pr.Project(gp_Pnt(0, 0, 0), q)
        return pr, (q.X(), q.Y())
    raise bad(f"view.projection {proj_kind!r} must be orthographic or perspective")


def edges_of(compound: TopoDS_Shape):
    if compound is None or compound.IsNull():
        return
    ex = TopExp_Explorer(compound, TopAbs_EDGE)
    while ex.More():
        yield TopoDS.Edge(ex.Current())
        ex.Next()


def turn_angle(a, b, c) -> float:
    u = (b[0] - a[0], b[1] - a[1])
    v = (c[0] - b[0], c[1] - b[1])
    return abs(math.atan2(u[0] * v[1] - u[1] * v[0], u[0] * v[0] + u[1] * v[1]))


def chain_segments(segs: list[tuple[tuple[float, float], tuple[float, float]]], tol: float, max_turn: float = math.radians(40)) -> list[list[tuple[float, float]]]:
    """Join 2-point segments that share endpoints (within tol) into polylines (polygonal HLR output).

    Chains continue only through joints where exactly two segments meet and the
    direction turns by less than max_turn, so mesh-faceted curves become one
    polyline while real corners stay separate edges.
    """
    if not segs:
        return []
    # Cluster endpoints: a grid of cell size tol, looking in the 3x3 neighbourhood.
    grid: dict[tuple[int, int], list[int]] = {}
    reps: list[tuple[float, float]] = []

    def cluster(p) -> int:
        cx, cy = int(math.floor(p[0] / tol)), int(math.floor(p[1] / tol))
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for c in grid.get((cx + dx, cy + dy), ()):
                    if math.dist(reps[c], p) <= tol:
                        return c
        reps.append(p)
        grid.setdefault((cx, cy), []).append(len(reps) - 1)
        return len(reps) - 1

    ends = [(cluster(a), cluster(b)) for a, b in segs]
    at: dict[int, list[int]] = {}
    for i, (ca, cb) in enumerate(ends):
        if ca == cb:
            continue
        at.setdefault(ca, []).append(i)
        at.setdefault(cb, []).append(i)
    used = [ends[i][0] == ends[i][1] for i in range(len(segs))]
    chains = []
    for i in range(len(segs)):
        if used[i]:
            continue
        used[i] = True
        cs = [ends[i][0], ends[i][1]]
        for tail in (True, False):  # grow at the tail, then at the head
            while True:
                tip = cs[-1] if tail else cs[0]
                if len(at.get(tip, ())) != 2:
                    break  # an end, or a junction of three or more
                nxt = next((j for j in at[tip] if not used[j]), None)
                if nxt is None:
                    break
                far = ends[nxt][1] if ends[nxt][0] == tip else ends[nxt][0]
                prev = cs[-2] if tail else cs[1]
                if turn_angle(reps[prev], reps[tip], reps[far]) > max_turn:
                    break
                used[nxt] = True
                if tail:
                    cs.append(far)
                else:
                    cs.insert(0, far)
                if cs[0] == cs[-1]:
                    break  # closed loop
            if cs[0] == cs[-1]:
                break
        chains.append([reps[c] for c in cs])
    return chains


def do_hlr(req: dict) -> dict:
    t0 = time.perf_counter()
    projector, offset = make_projector(req.get("view"))
    algorithm = req.get("algorithm", "exact")
    if algorithm not in ("exact", "polygonal"):
        raise bad("hlr.algorithm must be exact or polygonal")
    include_hidden = bool(req.get("includeHidden", False))
    include_smooth = bool(req.get("includeSmooth", False))
    raw = req.get("instances")
    if not isinstance(raw, list):
        raise bad("hlr.instances must be an array")
    placed: dict[str, Placed] = {}
    for i, p in enumerate(raw):
        pl = parse_placed(p, f"instances[{i}]")
        if pl.instance_id in placed:
            raise bad(f"duplicate instanceId {pl.instance_id!r}")
        placed[pl.instance_id] = pl
    targets = req.get("targetInstanceIds")
    if targets is None:
        targets = list(placed)
    elif not isinstance(targets, list) or any(t not in placed for t in targets):
        raise bad("targetInstanceIds must name instances in the request")
    occ = req.get("occludersByTarget")
    if occ is not None and not isinstance(occ, dict):
        raise bad("occludersByTarget must be an object")
    drawable = lambda iid: placed[iid].rec.face_count > 0  # noqa: E731
    # Faces with no edges are tessellated-only geometry (AP242 TESSELLATED_SHELL, e.g.
    # NIST FTC 08 "-tg"). Both HLR algorithms crash on it (exact: segfault; polygonal:
    # NCollection_Array1 out of range), so refuse it instead of killing the process.
    tess = [i for i in placed if drawable(i) and placed[i].rec.info.get("edgeCount") == 0]
    if tess:
        raise bad(f"instance {tess[0]!r} is tessellated-only geometry (faces without B-rep edges); HLR needs B-rep geometry")

    # Runs: (shapes in the run, targets extracted from it).
    runs: list[tuple[list[str], list[str]]] = []
    if occ is None:
        runs.append(([i for i in placed if drawable(i)], [t for t in targets if drawable(t)]))
    else:
        for t in targets:
            if not drawable(t):
                continue
            o = occ.get(t)
            if o is None:
                members = [i for i in placed if drawable(i)]
            else:
                if not isinstance(o, list) or any(x not in placed for x in o):
                    raise bad(f"occludersByTarget[{t!r}] must name instances in the request")
                members = [t] + [x for x in dict.fromkeys(o) if x != t and drawable(x)]
            runs.append((members, [t]))

    diag = 0.0
    boxes = [bbox_of(placed[i].shape) for i in placed if drawable(i)]
    boxes = [b for b in boxes if b is not None]
    if boxes:
        lo = [min(b[0][k] for b in boxes) for k in range(3)]
        hi = [max(b[1][k] for b in boxes) for k in range(3)]
        diag = math.dist(lo, hi)
    emitter = CurveEmitter(offset, max(5e-3, diag * 1e-5))
    setup_ms = (time.perf_counter() - t0) * 1000
    hlr_ms = 0.0
    extract_ms = 0.0
    edges_out: list[dict] = []

    if algorithm == "polygonal":
        lin = max(diag * 1e-3, 1e-3)
        t1 = time.perf_counter()
        for rec in {placed[i].rec.shape_id: placed[i].rec for i in placed if drawable(i)}.values():
            ensure_mesh(rec, lin, 0.2)
        setup_ms += (time.perf_counter() - t1) * 1000

    for members, run_targets in runs:
        if not run_targets:
            continue
        t1 = time.perf_counter()
        if algorithm == "exact":
            algo = HLRBRep_Algo()
            for iid in members:
                algo.Add(placed[iid].shape)
            algo.Projector(projector)
            t2 = time.perf_counter()
            algo.Update()
            algo.Hide()
            t3 = time.perf_counter()
            h = HLRBRep_HLRToShape(algo)
            for iid in run_targets:
                S = placed[iid].shape
                for typ, cls, smooth in _EXACT_CLASSES:
                    if smooth and not include_smooth:
                        continue
                    for vis in (True, False) if include_hidden else (True,):
                        for e in edges_of(h.CompoundOfEdges(S, typ, vis, False)):
                            c = emitter.emit(e)
                            if c is not None:
                                edges_out.append({"instanceId": iid, "edgeClass": cls, "visible": vis, "curve": c})
        else:
            algo = HLRBRep_PolyAlgo()
            for iid in members:
                algo.Load(placed[iid].shape)
            algo.Projector(projector)
            t2 = time.perf_counter()
            algo.Update()
            h = HLRBRep_PolyHLRToShape()
            h.Update(algo)
            t3 = time.perf_counter()
            for iid in run_targets:
                S = placed[iid].shape
                groups = [("outline", True, h.OutLineVCompound), ("sharp", True, h.VCompound)]
                if include_smooth:
                    groups += [("smooth", True, h.Rg1LineVCompound), ("seam", True, h.RgNLineVCompound)]
                if include_hidden:
                    groups += [("outline", False, h.OutLineHCompound), ("sharp", False, h.HCompound)]
                    if include_smooth:
                        groups += [("smooth", False, h.Rg1LineHCompound), ("seam", False, h.RgNLineHCompound)]
                for cls, vis, fn in groups:
                    segs = []
                    for e in edges_of(fn(S)):
                        ad = BRepAdaptor_Curve(e)
                        a = emitter.p2(ad.Value(ad.FirstParameter()))
                        b = emitter.p2(ad.Value(ad.LastParameter()))
                        if math.dist(a, b) > 1e-9:
                            segs.append((a, b))
                    for chain in chain_segments(segs, 1e-4):
                        if len(chain) == 2:
                            c = {"kind": "line", "a": list(chain[0]), "b": list(chain[1])}
                        else:
                            c = {"kind": "polyline", "points": [list(p) for p in chain]}
                        emitter.stats[c["kind"]] += 1
                        edges_out.append({"instanceId": iid, "edgeClass": cls, "visible": vis, "curve": c})
        t4 = time.perf_counter()
        setup_ms += (t2 - t1) * 1000
        hlr_ms += (t3 - t2) * 1000
        extract_ms += (t4 - t3) * 1000

    log(f"hlr {algorithm}: {len(edges_out)} edges, {len(runs)} run(s), curves {emitter.stats}")
    return {
        "type": "hlr:ok",
        "edges": edges_out,
        "timing": {"setupMs": setup_ms, "hlrMs": hlr_ms, "extractMs": extract_ms},
    }


# ------------------------------------------------------------------ clearance and sweep


def common_volume(a: TopoDS_Shape, b: TopoDS_Shape) -> float:
    op = BRepAlgoAPI_Common(a, b)
    if not op.IsDone():
        # A failed boolean must not read as "no interference": err on the side of contact.
        log("BRepAlgoAPI_Common failed; treating the pair as interfering")
        return math.inf
    g = GProp_GProps()
    BRepGProp.VolumeProperties_s(op.Shape(), g)
    return abs(g.Mass())


def volume_eps(a: ShapeRec, b: ShapeRec, tol: float) -> float:
    """Common volume below this is boolean noise from coincident faces, not interference."""
    v = min(a.info["volume"], b.info["volume"])
    return max(1e-6 * v, tol ** 3, 1e-9)


def distance(a: TopoDS_Shape, b: TopoDS_Shape) -> tuple[float, bool]:
    d = BRepExtrema_DistShapeShape()
    d.SetMultiThread(True)
    d.SetFlag(Extrema_ExtFlag.Extrema_ExtFlag_MIN)
    d.LoadS1(a)
    d.LoadS2(b)
    d.Perform()
    if not d.IsDone():
        raise KernelFailure("crash", "BRepExtrema_DistShapeShape failed")
    return d.Value(), bool(d.InnerSolution())


def penetrates(a: Placed | TopoDS_Shape, ra: ShapeRec, b: TopoDS_Shape, rb: ShapeRec, tol: float) -> bool:
    if not (ra.solid and rb.solid):
        return False
    return common_volume(a, b) > volume_eps(ra, rb, tol)


def do_clearance(req: dict) -> dict:
    a = parse_placed(req.get("a"), "a")
    b = parse_placed(req.get("b"), "b")
    tol = float(req.get("tolerance", 0.0))
    if tol < 0:
        raise bad("tolerance must be >= 0")
    if a.rec.face_count == 0 or b.rec.face_count == 0:
        raise bad("clearance needs shapes with faces")
    d, inner = distance(a.shape, b.shape)
    interferes = False
    if inner:
        d, interferes = 0.0, True
    elif d <= tol:
        interferes = penetrates(a.shape, a.rec, b.shape, b.rec, tol)
        d = 0.0
    return {"type": "clearance:ok", "distance": d, "interferes": interferes}


def ray_aabb_interval(mb, ob, d, pad) -> tuple[float, float] | None:
    """t-interval (t >= 0) in which moving box mb translated by t*d overlaps box ob (both padded)."""
    t_lo, t_hi = 0.0, math.inf
    for k in range(3):
        m0, m1 = mb[0][k] - pad, mb[1][k] + pad
        o0, o1 = ob[0][k], ob[1][k]
        if abs(d[k]) < 1e-15:
            if m1 < o0 or m0 > o1:
                return None
            continue
        # overlap when m0 + t d <= o1 and m1 + t d >= o0
        a = (o1 - m0) / d[k]
        b = (o0 - m1) / d[k]
        lo, hi = (b, a) if d[k] > 0 else (a, b)
        t_lo, t_hi = max(t_lo, lo), min(t_hi, hi)
        if t_lo > t_hi:
            return None
    return t_lo, t_hi


def moved_by(shape: TopoDS_Shape, d, t: float) -> TopoDS_Shape:
    tr = gp_Trsf()
    tr.SetTranslation(gp_Vec(d[0] * t, d[1] * t, d[2] * t))
    return shape.Moved(TopLoc_Location(tr))


# Sweep is mesh-based with exact confirmation. Exact conservative advancement was
# correct but too slow: every B-rep distance or boolean costs 0.2-0.7 s on real
# files, and parts that slide along each other (a pin in its hole) need hundreds.

SWEEP_GRAZE = 1e-3  # |cos| between a facet normal and the motion below which the facet is "parallel"
SWEEP_MAX_CONFIRM = 25


def sweep_mesh(rec: ShapeRec):
    """Cached triangle mesh for sweeps: (deflection, positions, triangles), outward winding."""
    defl = max(1e-3, 1e-3 * rec.diag)
    if rec.sweep_mesh is None or rec.sweep_mesh[0] != defl:
        pos, _, tri, _ = tri_mesh(rec, defl, 0.3)
        if len(tri):
            p0, p1, p2 = pos[tri[:, 0]], pos[tri[:, 1]], pos[tri[:, 2]]
            signed = float(np.einsum("ij,ij->i", p0, np.cross(p1, p2)).sum()) / 6
            if signed < 0:  # inside-out shell; flip so the facing tests stay right
                log(f"shape {rec.shape_id}: mesh signed volume {signed:.3g} < 0, flipping winding")
                tri = tri[:, [0, 2, 1]]
        rec.sweep_mesh = (defl, pos, tri)
    return rec.sweep_mesh


def trsf_arrays(t: gp_Trsf) -> tuple[np.ndarray, np.ndarray]:
    R = np.array([[t.Value(r, c) for c in (1, 2, 3)] for r in (1, 2, 3)])
    return R, np.array([t.Value(1, 4), t.Value(2, 4), t.Value(3, 4)])


@dataclass
class SweepBody:
    defl: float
    tris: np.ndarray  # (m,3,3) world-space triangles
    facing: np.ndarray  # (m,) cos(normal, d)


def sweep_body(pl: Placed, d: np.ndarray) -> SweepBody:
    defl, pos, tri = sweep_mesh(pl.rec)
    R, t = trsf_arrays(pl.trsf)
    world = pos @ R.T + t
    tris = world[tri]
    n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    ln = np.linalg.norm(n, axis=1)
    ok = ln > 1e-14
    tris, n, ln = tris[ok], n[ok], ln[ok]
    return SweepBody(defl, tris, (n @ d) / ln)


def sample_points(tris: np.ndarray, spacing: float) -> np.ndarray:
    """Triangle vertices plus points along every triangle edge at `spacing`."""
    if len(tris) == 0:
        return np.zeros((0, 3))
    verts = tris.reshape(-1, 3)
    a = np.concatenate([tris[:, 0], tris[:, 1], tris[:, 2]])
    b = np.concatenate([tris[:, 1], tris[:, 2], tris[:, 0]])
    ln = np.linalg.norm(b - a, axis=1)
    k = np.ceil(ln / spacing).astype(np.int64)
    k = np.minimum(k, 1000)
    many = k > 1
    a, b, k = a[many], b[many], k[many]
    if len(k) == 0:
        return np.unique(verts, axis=0)
    rep = np.repeat(np.arange(len(k)), k - 1)
    starts = np.cumsum(k - 1) - (k - 1)
    j = np.arange(len(rep)) - np.repeat(starts, k - 1) + 1
    frac = (j / k[rep])[:, None]
    mids = a[rep] + (b[rep] - a[rep]) * frac
    return np.unique(np.concatenate([verts, mids]), axis=0)


def ray_hits(points: np.ndarray, tris: np.ndarray, d: np.ndarray, sign: float, t_lo: float, t_hi: float) -> np.ndarray:
    """Times t in [t_lo, t_hi] at which points moving by sign*t*d... meet the triangles.

    Points move along +d (sign=+1: t = w_tri - w_pt) or the triangles do (sign=-1:
    t = w_pt - w_tri). Brute force in chunks, after a 2D bounding-box prefilter.
    """
    if len(points) == 0 or len(tris) == 0:
        return np.zeros(0)
    e1 = np.cross(d, [1.0, 0.0, 0.0])
    if np.linalg.norm(e1) < 0.5:
        e1 = np.cross(d, [0.0, 1.0, 0.0])
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(d, e1)
    B = np.stack([e1, e2], axis=1)  # (3,2)
    P2, Pw = points @ B, points @ d
    T2, Tw = tris @ B, tris @ d  # (m,3,2), (m,3)
    lo, hi = T2.min(axis=(0, 1)), T2.max(axis=(0, 1))
    keep = np.all((P2 >= lo - 1e-9) & (P2 <= hi + 1e-9), axis=1)
    P2, Pw = P2[keep], Pw[keep]
    plo, phi = P2.min(axis=0) if len(P2) else lo, P2.max(axis=0) if len(P2) else hi
    tk = np.all((T2.max(axis=1) >= plo - 1e-9) & (T2.min(axis=1) <= phi + 1e-9), axis=1)
    T2, Tw = T2[tk], Tw[tk]
    if len(P2) == 0 or len(T2) == 0:
        return np.zeros(0)
    a, v0, v1 = T2[:, 0], T2[:, 2] - T2[:, 0], T2[:, 1] - T2[:, 0]
    d00 = np.einsum("ij,ij->i", v0, v0)
    d01 = np.einsum("ij,ij->i", v0, v1)
    d11 = np.einsum("ij,ij->i", v1, v1)
    den = d00 * d11 - d01 * d01
    good = np.abs(den) > 1e-18
    a, v0, v1, d00, d01, d11, den, Tw = a[good], v0[good], v1[good], d00[good], d01[good], d11[good], den[good], Tw[good]
    tmin2, tmax2 = T2[good].min(axis=1), T2[good].max(axis=1)
    out = []
    chunk = max(1, 4_000_000 // max(len(a), 1))
    for s in range(0, len(P2), chunk):
        p, pw = P2[s:s + chunk], Pw[s:s + chunk]
        inb = np.all((p[:, None, :] >= tmin2[None] - 1e-9) & (p[:, None, :] <= tmax2[None] + 1e-9), axis=2)
        pi, ti = np.nonzero(inb)
        if len(pi) == 0:
            continue
        v2 = p[pi] - a[ti]
        d02 = np.einsum("ij,ij->i", v0[ti], v2)
        d12 = np.einsum("ij,ij->i", v1[ti], v2)
        u = (d11[ti] * d02 - d01[ti] * d12) / den[ti]
        v = (d00[ti] * d12 - d01[ti] * d02) / den[ti]
        inside = (u >= -1e-9) & (v >= -1e-9) & (u + v <= 1 + 1e-9)
        pi, ti, u, v = pi[inside], ti[inside], u[inside], v[inside]
        wt = Tw[ti, 0] + u * (Tw[ti, 2] - Tw[ti, 0]) + v * (Tw[ti, 1] - Tw[ti, 0])
        t = sign * (wt - pw[pi])
        out.append(t[(t >= t_lo) & (t <= t_hi)])
    return np.concatenate(out) if out else np.zeros(0)


def mesh_first_contacts(mv: SweepBody, ob: SweepBody, d: np.ndarray, t_lo: float, t_hi: float) -> np.ndarray:
    """Sorted candidate contact times of `mv` moving along d into `ob`.

    Only the moving body's leading facets (normal along d) can strike the obstacle's
    facets that face back at it (normal against d); trailing and parallel facets
    can never be the first contact, and skipping them removes the chord-versus-arc
    slivers that tight fits (a rod in its hole) would otherwise report as hits.
    """
    front = mv.tris[mv.facing > SWEEP_GRAZE]
    back = ob.tris[ob.facing < -SWEEP_GRAZE]
    if len(front) == 0 or len(back) == 0:
        return np.zeros(0)
    spacing = 4 * max(mv.defl, ob.defl)
    t1 = ray_hits(sample_points(front, spacing), back, d, +1.0, t_lo, t_hi)
    t2 = ray_hits(sample_points(back, spacing), front, d, -1.0, t_lo, t_hi)
    return np.sort(np.concatenate([t1, t2]))


def do_sweep(req: dict) -> dict:
    m = parse_placed(req.get("moving"), "moving")
    d = unit(vec3(req.get("direction"), "direction"), "direction")
    obs_raw = req.get("obstacles")
    if not isinstance(obs_raw, list):
        raise bad("obstacles must be an array")
    obstacles = [parse_placed(p, f"obstacles[{i}]") for i, p in enumerate(obs_raw)]
    max_d = float(req.get("maxDistance", math.inf))
    tol = float(req.get("tolerance", 0.0))
    if not tol > 0:
        tol = 1e-3
    if not max_d > 0:
        raise bad("maxDistance must be > 0")
    mb = bbox_of(m.shape)
    if mb is None or m.rec.face_count == 0 or not m.rec.solid:
        raise bad("the moving shape has no solid")
    cands = []
    for o in obstacles:
        if o.rec.face_count == 0 or not o.rec.solid:
            continue
        ob = bbox_of(o.shape)
        if ob is None:
            continue
        iv = ray_aabb_interval(mb, ob, d, tol)
        if iv is not None:
            cands.append((iv[0], iv[1], o))
    if not cands:
        return {"type": "sweep:ok", "freeDistance": math.inf}
    dv = np.array(d)
    mv = sweep_body(m, dv)
    queue = []  # (t, index into hits, obstacle number)
    hits_of: list[np.ndarray] = []
    for t_in, t_out, o in cands:
        if t_in > max_d:
            hits_of.append(np.zeros(0))
            continue
        ob = sweep_body(o, dv)
        eps = max(tol, 2 * (mv.defl + ob.defl))
        h = mesh_first_contacts(mv, ob, dv, -eps, min(t_out, max_d) + eps)
        hits_of.append(np.maximum(h, 0.0))
        if len(h):
            heapq.heappush(queue, (float(hits_of[-1][0]), 0, len(hits_of) - 1))
    confirms = 0
    while queue:
        t, k, oi = heapq.heappop(queue)
        o = cands[oi][2]
        margin = max(tol, 2 * (mv.defl + o.rec.sweep_mesh[0]))
        if confirms >= SWEEP_MAX_CONFIRM:
            log(f"sweep: confirmation budget spent, accepting mesh hit at {t:.4g}")
            return {"type": "sweep:ok", "freeDistance": t, "blockedBy": o.instance_id}
        confirms += 1
        # Exact check just past the mesh contact: a real hit has a real common volume there.
        if penetrates(moved_by(m.shape, d, t + margin), m.rec, o.shape, o.rec, tol):
            return {"type": "sweep:ok", "freeDistance": t, "blockedBy": o.instance_id}
        log(f"sweep: mesh hit on {o.instance_id} at {t:.4g} not confirmed by the exact check; skipping it")
        h = hits_of[oi]
        nxt = int(np.searchsorted(h, t + margin, side="right"))
        if nxt < len(h):
            heapq.heappush(queue, (float(h[nxt]), nxt, oi))
    # Not hit within maxDistance. Report Infinity only if nothing lies anywhere along the path.
    return {"type": "sweep:ok", "freeDistance": max_d if any(c[1] > max_d for c in cands) else math.inf}


# ------------------------------------------------------------------ main loop

HANDLERS = {"import": do_import, "mesh": do_mesh, "hlr": do_hlr, "clearance": do_clearance, "sweep": do_sweep}


def handle(line: str) -> dict:
    env_id: Any = None
    try:
        try:
            env = json.loads(line)
        except json.JSONDecodeError as e:
            raise bad(f"invalid JSON: {e}")
        if not isinstance(env, dict) or not isinstance(env.get("body"), dict):
            raise bad("expected an Envelope {id, body}")
        env_id = env.get("id")
        body = from_wire(env["body"])
        kind = body.get("type")
        fn = HANDLERS.get(kind)
        if fn is None:
            raise bad(f"unknown request type {kind!r}")
        t0 = time.perf_counter()
        result = fn(body)
        log(f"#{env_id} {kind} ok in {(time.perf_counter() - t0) * 1000:.0f} ms")
        return {"id": env_id, "body": result}
    except KernelFailure as e:
        log(f"#{env_id} {e.code}: {e.message}")
        return {"id": env_id, "body": {"type": "error", "code": e.code, "message": e.message}}
    except MemoryError:
        return {"id": env_id, "body": {"type": "error", "code": "out-of-memory", "message": "out of memory"}}
    except Exception as e:  # OCCT Standard_Failure and anything unexpected
        traceback.print_exc(file=sys.stderr)
        return {"id": env_id, "body": {"type": "error", "code": "crash", "message": f"{type(e).__name__}: {e}"}}


def main() -> None:
    log(f"ready (protocol v{PROTOCOL_VERSION}, pid {os.getpid()})")
    for line in sys.stdin:
        if not line.strip():
            continue
        resp = handle(line)
        _PROTO.write(json.dumps(to_wire(resp), allow_nan=False, separators=(",", ":")) + "\n")
        _PROTO.flush()


if __name__ == "__main__":
    main()
