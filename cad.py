"""
CAD side of step2urdf: read a STEP file (with assembly structure, names and colours),
and for every part compute exact B-rep mass properties, a triangle mesh and the
cylindrical features that are candidates for joint axes.

All geometry returned by this module is in millimetres (OpenCascade converts every STEP
file to mm on import, whatever unit it was authored in).
"""
from __future__ import annotations

import logging
import re
from dataclasses import dataclass, field

import numpy as np

from OCP.BRep import BRep_Tool
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.BRepBndLib import BRepBndLib
from OCP.BRepGProp import BRepGProp
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.Bnd import Bnd_Box
from OCP.GeomAbs import GeomAbs_Cylinder
from OCP.GProp import GProp_GProps
from OCP.IFSelect import IFSelect_RetDone
from OCP.Quantity import Quantity_Color
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.STEPControl import STEPControl_Reader
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDataStd import TDataStd_Name
from OCP.TDF import TDF_Label
try:  # OCP >= 7.8 / 8.x
    from OCP.collections import Sequence_TDF_Label as TDF_LabelSequence
except ImportError:  # older OCP
    from OCP.TDF import TDF_LabelSequence
from OCP.TDocStd import TDocStd_Document
from OCP.TopAbs import TopAbs_FACE, TopAbs_REVERSED, TopAbs_SOLID
from OCP.TopExp import TopExp_Explorer
from OCP.TopLoc import TopLoc_Location
from OCP.TopoDS import TopoDS, TopoDS_Shape

_to_face = getattr(TopoDS, "Face_s", None) or TopoDS.Face


def _static(cls, name):
    """OCP exposes static methods as Name_s (older) or Name (newer)."""
    return getattr(cls, name + "_s", None) or getattr(cls, name)
from OCP.XCAFDoc import (XCAFDoc_ColorGen, XCAFDoc_ColorSurf, XCAFDoc_ColorTool,
                         XCAFDoc_DocumentTool, XCAFDoc_ShapeTool)

log = logging.getLogger("step2urdf")


@dataclass
class Cylinder:
    """A cylindrical face: a likely shaft, pin, bore or bearing seat."""
    point: np.ndarray      # a point on the axis (mm)
    direction: np.ndarray  # unit axis direction
    radius: float          # mm
    area: float            # mm^2


@dataclass
class Part:
    name: str
    shape: TopoDS_Shape
    color: tuple | None = None           # (r, g, b) 0..1
    volume: float = 0.0                  # mm^3
    com: np.ndarray = field(default_factory=lambda: np.zeros(3))    # mm, world
    inertia: np.ndarray = field(default_factory=lambda: np.zeros((3, 3)))  # mm^5, unit density, about COM
    bbox_min: np.ndarray = field(default_factory=lambda: np.zeros(3))
    bbox_max: np.ndarray = field(default_factory=lambda: np.zeros(3))
    cylinders: list = field(default_factory=list)
    approximate: bool = False            # True if not a closed solid -> bbox-based mass props
    _mesh: tuple | None = None

    @property
    def size(self) -> float:
        return float(np.linalg.norm(self.bbox_max - self.bbox_min))

    def mesh(self, quality: float = 1.0):
        """Return (vertices[N,3] mm, faces[M,3] int). quality>1 = finer."""
        if self._mesh is None or self._mesh[0] != quality:
            self._mesh = (quality, tessellate(self.shape, self.size, quality))
        return self._mesh[1]


# --------------------------------------------------------------------------- STEP reading

def _label_name(label: TDF_Label) -> str:
    attr = TDataStd_Name()
    if label.FindAttribute(_static(TDataStd_Name, "GetID")(), attr):
        s = attr.Get().ToExtString()
        if s and not s.startswith("=>"):
            return s.strip()
    return ""


def _label_color(color_tool, label: TDF_Label):
    c = Quantity_Color()
    get = getattr(XCAFDoc_ColorTool, "GetColor_s", None)
    for kind in (XCAFDoc_ColorSurf, XCAFDoc_ColorGen):
        try:
            ok = get(label, kind, c) if get else color_tool.GetColor(label, kind, c)
        except TypeError:
            ok = False
        if ok:
            return (c.Red(), c.Green(), c.Blue())
    return None


def _read_xde(path: str):
    doc = TDocStd_Document(TCollection_ExtendedString("XmlOcaf"))
    reader = STEPCAFControl_Reader()
    reader.SetNameMode(True)
    reader.SetColorMode(True)
    if reader.ReadFile(path) != IFSelect_RetDone:
        raise ValueError(f"Could not read STEP file: {path}")
    reader.Transfer(doc)
    st = _static(XCAFDoc_DocumentTool, "ShapeTool")(doc.Main())
    ct = _static(XCAFDoc_DocumentTool, "ColorTool")(doc.Main())

    out: list[tuple[str, TopoDS_Shape, tuple | None]] = []

    def walk(label, loc, inherited_color, hint_name):
        name = _label_name(label) or hint_name
        color = _label_color(ct, label) or inherited_color
        if _static(XCAFDoc_ShapeTool, "IsAssembly")(label):
            comps = TDF_LabelSequence()
            _static(XCAFDoc_ShapeTool, "GetComponents")(label, comps, False)
            for i in range(1, comps.Length() + 1):
                comp = comps.Value(i)
                ref = TDF_Label()
                _static(XCAFDoc_ShapeTool, "GetReferredShape")(comp, ref)
                cloc = _static(XCAFDoc_ShapeTool, "GetLocation")(comp)
                ccolor = _label_color(ct, comp) or color
                walk(ref, loc.Multiplied(cloc), ccolor, _label_name(comp) or name)
        else:
            shape = _static(XCAFDoc_ShapeTool, "GetShape")(label)
            if not shape.IsNull():
                out.append((name or "part", shape.Moved(loc), color))

    roots = TDF_LabelSequence()
    st.GetFreeShapes(roots)
    for i in range(1, roots.Length() + 1):
        walk(roots.Value(i), TopLoc_Location(), None, "")
    return out


def _read_plain(path: str):
    reader = STEPControl_Reader()
    if reader.ReadFile(path) != IFSelect_RetDone:
        raise ValueError(f"Could not read STEP file: {path}")
    reader.TransferRoots()
    return [("part", reader.OneShape(), None)]


def _solids(shape):
    exp, out = TopExp_Explorer(shape, TopAbs_SOLID), []
    while exp.More():
        out.append(exp.Current())
        exp.Next()
    return out


def sanitize(name: str) -> str:
    s = re.sub(r"[^A-Za-z0-9_]+", "_", name).strip("_")
    if not s:
        s = "part"
    if s[0].isdigit():
        s = "p_" + s
    return s


def load_step(path: str) -> list[Part]:
    """Read a STEP file into a flat list of positioned parts with mass properties."""
    try:
        raw = _read_xde(path)
    except Exception as e:  # pragma: no cover - exotic files
        log.warning("Assembly-aware reader failed (%s); falling back to plain reader", e)
        raw = []
    if not raw:
        raw = _read_plain(path)

    # A single multi-body part (common for STEP exports without assembly info): split bodies.
    if len(raw) == 1:
        name, shape, color = raw[0]
        solids = _solids(shape)
        if len(solids) > 1:
            raw = [(f"{name}_body{i+1}", s, color) for i, s in enumerate(solids)]

    parts, seen = [], {}
    for name, shape, color in raw:
        base = sanitize(name)
        n = seen.get(base, 0)
        seen[base] = n + 1
        uname = base if n == 0 else f"{base}_{n+1}"
        p = Part(uname, shape, color)
        analyse(p)
        if p.volume <= 0 and p.size <= 0:
            log.warning("Skipping empty shape %s", uname)
            continue
        parts.append(p)
    # Rename first instance to *_1 when duplicates exist, so names read consistently.
    for base, count in seen.items():
        if count > 1:
            for p in parts:
                if p.name == base:
                    p.name = f"{base}_1"
    if not parts:
        raise ValueError("No geometry found in STEP file")
    return parts


# --------------------------------------------------------------------------- analysis

def _gp(p):
    return np.array([p.X(), p.Y(), p.Z()])


def analyse(p: Part):
    box = Bnd_Box()
    _static(BRepBndLib, "Add")(p.shape, box)
    if box.IsVoid():
        return
    p.bbox_min, p.bbox_max = _gp(box.CornerMin()), _gp(box.CornerMax())

    props = GProp_GProps()
    _static(BRepGProp, "VolumeProperties")(p.shape, props)
    vol = props.Mass()
    sign = -1.0 if vol < 0 else 1.0     # badly oriented shells give negative volume
    vol = abs(vol)
    box_vol = float(np.prod(p.bbox_max - p.bbox_min))

    if vol > 1e-6 * max(box_vol, 1e-12):
        m = props.MatrixOfInertia()      # about the centre of mass, density 1
        p.volume = vol
        p.com = _gp(props.CentreOfMass())
        p.inertia = sign * np.array([[m.Value(i, j) for j in (1, 2, 3)] for i in (1, 2, 3)])
    else:
        # Surface/sheet model: no enclosed volume. Approximate as a solid bounding box.
        p.approximate = True
        d = p.bbox_max - p.bbox_min
        p.volume = max(box_vol, 1e-9)
        p.com = (p.bbox_min + p.bbox_max) / 2
        p.inertia = p.volume / 12.0 * np.diag([d[1]**2 + d[2]**2, d[0]**2 + d[2]**2, d[0]**2 + d[1]**2])
        log.warning("Part %s is not a closed solid; mass properties approximated from its bounding box", p.name)

    p.cylinders = find_cylinders(p.shape)


def find_cylinders(shape) -> list[Cylinder]:
    out = []
    exp = TopExp_Explorer(shape, TopAbs_FACE)
    while exp.More():
        face = _to_face(exp.Current())
        try:
            surf = BRepAdaptor_Surface(face)
            if surf.GetType() == GeomAbs_Cylinder:
                c = surf.Cylinder()
                ax = c.Axis()
                gp = GProp_GProps()
                _static(BRepGProp, "SurfaceProperties")(face, gp)
                d = _gp(ax.Direction())
                out.append(Cylinder(_gp(ax.Location()), d / np.linalg.norm(d), c.Radius(), abs(gp.Mass())))
        except Exception:  # pragma: no cover
            pass
        exp.Next()
    return out


def tessellate(shape, size: float, quality: float = 1.0):
    """Triangulate a shape. Deflection scales with part size so small and large parts both look right."""
    lin = max(size * 0.002 / quality, 0.005)
    BRepMesh_IncrementalMesh(shape, lin, False, 0.35 / quality, True)
    verts, faces, offset = [], [], 0
    exp = TopExp_Explorer(shape, TopAbs_FACE)
    while exp.More():
        face = _to_face(exp.Current())
        loc = TopLoc_Location()
        tri = _static(BRep_Tool, "Triangulation")(face, loc)
        if tri is not None:
            trsf = loc.Transformation()
            n = tri.NbNodes()
            v = np.empty((n, 3))
            for i in range(n):
                q = tri.Node(i + 1).Transformed(trsf)
                v[i] = (q.X(), q.Y(), q.Z())
            t = np.empty((tri.NbTriangles(), 3), dtype=np.int64)
            for i in range(tri.NbTriangles()):
                t[i] = tri.Triangle(i + 1).Get()
            t -= 1
            if face.Orientation() == TopAbs_REVERSED:
                t = t[:, [0, 2, 1]]
            verts.append(v)
            faces.append(t + offset)
            offset += n
        exp.Next()
    if not verts:
        return np.zeros((0, 3)), np.zeros((0, 3), dtype=np.int64)
    return np.vstack(verts), np.vstack(faces)
