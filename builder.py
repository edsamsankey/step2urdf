"""
Turn a list of CAD parts + a kinematic config into a URDF robot package.

Frame convention used for every generated link
----------------------------------------------
* The root link's frame is the CAD origin.
* Every other link's frame sits at its joint origin (a point in CAD space) and is
  axis-aligned with the CAD frame. So joint <origin> is a pure translation and the
  joint <axis> is just the CAD direction of rotation/sliding.
* Meshes are re-expressed in their link frame and written in metres, so no <scale> is needed.
"""
from __future__ import annotations

import fnmatch
import io
import json
import logging
import math
import os
import struct
import zipfile
from dataclasses import asdict, dataclass, field
from xml.dom import minidom
from xml.etree import ElementTree as ET

import numpy as np

from .cad import Part, sanitize

log = logging.getLogger("step2urdf")

JOINT_TYPES = ("fixed", "revolute", "continuous", "prismatic")


# --------------------------------------------------------------------------- config model

@dataclass
class LinkSpec:
    name: str
    parts: list[str]                 # part names or glob patterns, e.g. "M8_Screw*"
    density: float | None = None     # kg/m^3, overrides the global default
    mass: float | None = None        # kg, overrides density-based mass (inertia is scaled)
    color: list[float] | None = None  # rgba 0..1


@dataclass
class JointSpec:
    name: str
    type: str
    parent: str
    child: str
    origin: object = "auto"          # "auto" | "com" | [x, y, z] in CAD units (mm)
    axis: object = "auto"            # "auto" | [x, y, z]
    lower: float | None = None       # rad (revolute) or m (prismatic)
    upper: float | None = None
    effort: float = 10.0             # N*m or N
    velocity: float = 1.0            # rad/s or m/s


@dataclass
class Config:
    robot_name: str = "robot"
    density: float = 2700.0          # kg/m^3 (aluminium)
    unit_scale: float = 0.001        # CAD unit -> metre (OpenCascade always imports in mm)
    mesh_quality: float = 1.0        # >1 finer, <1 coarser
    mesh_path: str = "package"       # "package" -> package://name/meshes, "relative" -> ../meshes
    collision: str = "mesh"          # "mesh" | "box" | "none"
    root: str | None = None
    links: list[LinkSpec] = field(default_factory=list)
    joints: list[JointSpec] = field(default_factory=list)

    @staticmethod
    def from_dict(d: dict) -> "Config":
        d = dict(d)
        links = [LinkSpec(**l) for l in d.pop("links", [])]
        joints = [JointSpec(**j) for j in d.pop("joints", [])]
        known = {k: v for k, v in d.items() if k in Config.__dataclass_fields__}
        return Config(links=links, joints=joints, **known)

    def to_dict(self) -> dict:
        d = asdict(self)
        # drop empty optional keys for a tidy YAML file
        d["links"] = [{k: v for k, v in l.items() if v is not None} for l in d["links"]]
        d["joints"] = [{k: v for k, v in j.items() if v is not None} for j in d["joints"]]
        return d


def default_config(parts: list[Part], robot_name="robot", mode="per_part", density=2700.0) -> Config:
    """
    mode="per_part": one link per part, all fixed to the heaviest part (a scaffold to edit).
    mode="single":   everything merged into one rigid link.
    """
    cfg = Config(robot_name=sanitize(robot_name), density=density)
    if mode == "single":
        cfg.links = [LinkSpec("base_link", ["*"])]
        cfg.root = "base_link"
        return cfg
    root = max(parts, key=lambda p: p.volume)
    cfg.root = root.name
    cfg.links = [LinkSpec(p.name, [p.name]) for p in parts]
    cfg.joints = [JointSpec(f"{p.name}_joint", "fixed", root.name, p.name, origin="com")
                  for p in parts if p is not root]
    return cfg


# --------------------------------------------------------------------------- built model

@dataclass
class Link:
    name: str
    parts: list[Part]
    frame: np.ndarray = field(default_factory=lambda: np.zeros(3))  # CAD units, world
    mass: float = 0.0                 # kg
    com: np.ndarray = field(default_factory=lambda: np.zeros(3))    # CAD units, world
    inertia: np.ndarray = field(default_factory=lambda: np.eye(3))  # kg m^2 about COM, world axes
    color: list[float] = field(default_factory=lambda: [0.7, 0.7, 0.7, 1.0])


@dataclass
class Joint:
    spec: JointSpec
    origin_world: np.ndarray          # CAD units
    axis: np.ndarray
    note: str = ""


@dataclass
class Robot:
    config: Config
    links: dict[str, Link]
    joints: list[Joint]
    root: str
    warnings: list[str]

    def children(self, link):
        return [j for j in self.joints if j.spec.parent == link]


def _match(pattern: str, names: list[str]) -> list[str]:
    if pattern in names:
        return [pattern]
    return [n for n in names if fnmatch.fnmatchcase(n, pattern)]


def _line_hits_box(p, d, lo, hi) -> bool:
    """Does the infinite line p + t*d pass through the box [lo, hi]?"""
    tmin, tmax = -math.inf, math.inf
    for i in range(3):
        if abs(d[i]) < 1e-12:
            if p[i] < lo[i] or p[i] > hi[i]:
                return False
        else:
            a, b = (lo[i] - p[i]) / d[i], (hi[i] - p[i]) / d[i]
            tmin, tmax = max(tmin, min(a, b)), min(tmax, max(a, b))
    return tmax >= tmin


def _bbox(parts, pad=0.0):
    lo = np.min([p.bbox_min for p in parts], axis=0)
    hi = np.max([p.bbox_max for p in parts], axis=0)
    m = pad * np.linalg.norm(hi - lo)
    return lo - m, hi + m


def detect_axis(parent: Link, child: Link, axis_hint=None):
    """
    Find the most plausible rotation axis between two links from cylindrical faces:
    a shaft/pin/bore on either link whose axis passes through the other link.
    Returns (point_on_axis, unit_direction, description) or None.
    """
    candidates = []
    for owner, other in ((child, parent), (parent, child)):
        lo, hi = _bbox(other.parts, pad=0.02)
        for part in owner.parts:
            for c in part.cylinders:
                if axis_hint is not None and abs(float(np.dot(c.direction, axis_hint))) < 0.995:
                    continue
                if _line_hits_box(c.point, c.direction, lo, hi):
                    candidates.append((c.area, c, part.name))
    if not candidates:
        return None
    _, c, pname = max(candidates, key=lambda t: t[0])
    # Place the origin on the axis where the two links meet: project the centre of their
    # bounding-box overlap (or of the gap between them) onto the axis line.
    (plo, phi), (clo, chi) = _bbox(parent.parts), _bbox(child.parts)
    contact = (np.maximum(plo, clo) + np.minimum(phi, chi)) / 2
    d = c.direction
    pt = c.point + np.dot(contact - c.point, d) * d
    return pt, d, f"cylinder r={c.radius:.2f} on {pname}"


def build(parts: list[Part], cfg: Config) -> Robot:
    warnings: list[str] = []
    names = [p.name for p in parts]
    by_name = {p.name: p for p in parts}
    s = cfg.unit_scale

    # 1. assign parts to links -------------------------------------------------------
    claimed: dict[str, str] = {}
    links: dict[str, Link] = {}
    for spec in cfg.links:
        lname = sanitize(spec.name)
        if lname in links:
            raise ValueError(f"Duplicate link name '{lname}'")
        got = []
        for pat in spec.parts:
            hits = _match(pat, names)
            if not hits:
                warnings.append(f"Link '{lname}': pattern '{pat}' matched no parts")
            for h in hits:
                if h in claimed:
                    if claimed[h] != lname:
                        warnings.append(f"Part '{h}' already in link '{claimed[h]}', ignored for '{lname}'")
                    continue
                claimed[h] = lname
                got.append(by_name[h])
        links[lname] = Link(lname, got)

    if not links:
        raise ValueError("Config defines no links")

    child_names = {sanitize(j.child) for j in cfg.joints}
    root = sanitize(cfg.root) if cfg.root else next((n for n in links if n not in child_names), None)
    if root is None or root not in links:
        raise ValueError("Could not determine the root link (every link is a joint child?)")

    unassigned = [p for p in parts if p.name not in claimed]
    if unassigned:
        links[root].parts.extend(unassigned)
        warnings.append(f"{len(unassigned)} unassigned part(s) merged into root link '{root}': "
                        + ", ".join(p.name for p in unassigned[:8]) + ("..." if len(unassigned) > 8 else ""))

    empty = [n for n, l in links.items() if not l.parts]
    for n in empty:
        warnings.append(f"Link '{n}' has no geometry; it gets a tiny placeholder mass")

    # 2. mass properties per link (SI) ------------------------------------------------
    density_of = {sanitize(l.name): (l.density or cfg.density) for l in cfg.links}
    for name, link in links.items():
        rho = density_of.get(name, cfg.density)
        if not link.parts:
            link.mass, link.inertia = 1e-3, np.eye(3) * 1e-6
            continue
        ms = np.array([p.volume * s**3 * rho for p in link.parts])
        coms = np.array([p.com for p in link.parts])
        M = ms.sum()
        com = (ms[:, None] * coms).sum(0) / M
        I = np.zeros((3, 3))
        for m, p in zip(ms, link.parts):
            I += p.inertia * rho * s**5                       # own inertia about own COM
            dv = (p.com - com) * s
            I += m * (np.dot(dv, dv) * np.eye(3) - np.outer(dv, dv))  # parallel-axis shift
        link.mass, link.com, link.inertia = float(M), com, I
        col = next((p.color for p in link.parts if p.color), None)   # CAD colour of first coloured part
        if col:
            link.color = [*col, 1.0]
        if any(p.approximate for p in link.parts):
            warnings.append(f"Link '{name}' contains non-solid geometry; its inertia is approximate")

    spec_by_name = {sanitize(l.name): l for l in cfg.links}
    for name, spec in spec_by_name.items():
        if spec.color:                                   # explicit colour wins over CAD colour
            links[name].color = list(spec.color) + [1.0] * (4 - len(spec.color))
        if spec.mass and links[name].mass > 0:
            k = spec.mass / links[name].mass
            links[name].mass, links[name].inertia = spec.mass, links[name].inertia * k

    # 3. joints -----------------------------------------------------------------------
    joints: list[Joint] = []
    parent_of: dict[str, str] = {}
    for js in cfg.joints:
        js.parent, js.child, js.name = sanitize(js.parent), sanitize(js.child), sanitize(js.name)
        if js.type not in JOINT_TYPES:
            raise ValueError(f"Joint '{js.name}': type must be one of {JOINT_TYPES}")
        for end in (js.parent, js.child):
            if end not in links:
                raise ValueError(f"Joint '{js.name}' references unknown link '{end}'")
        if js.child == root:
            raise ValueError(f"Joint '{js.name}': the root link '{root}' cannot be a child")
        if js.child in parent_of:
            raise ValueError(f"Link '{js.child}' has two parent joints")
        parent_of[js.child] = js.parent

        parent, child = links[js.parent], links[js.child]
        hint = None
        if not (isinstance(js.axis, str) and js.axis == "auto"):
            hint = np.asarray(js.axis, float)
            if np.linalg.norm(hint) < 1e-9:
                raise ValueError(f"Joint '{js.name}': axis is zero")
            hint = hint / np.linalg.norm(hint)
        note, axis = "", hint if hint is not None else np.array([0.0, 0.0, 1.0])

        if isinstance(js.origin, str) and js.origin == "auto" and js.type != "fixed" and child.parts:
            found = detect_axis(parent, child, hint)
            if found:
                origin, d, note = found
                if hint is None:
                    axis = d
                else:  # keep the user's sign
                    axis = hint
                note = "auto: " + note
            else:
                origin = child.com
                note = "auto: no shaft/bore found, used child centre of mass"
                warnings.append(f"Joint '{js.name}': no cylindrical feature found for the axis; "
                                f"origin set to child COM - please set origin/axis manually")
        elif isinstance(js.origin, str):           # "com" or "auto" on fixed joints
            origin = child.com
        else:
            origin = np.asarray(js.origin, float)
        if js.type == "prismatic" and hint is None and not note.startswith("auto: cyl"):
            warnings.append(f"Joint '{js.name}': prismatic axis defaulted to {axis.round(3).tolist()}")
        joints.append(Joint(js, np.asarray(origin, float), axis / np.linalg.norm(axis), note))
        child.frame = np.asarray(origin, float)

    # every non-root link must be reachable from the root
    for name in links:
        seen, n = set(), name
        while n != root:
            if n in seen:
                raise ValueError(f"Kinematic loop involving link '{name}' (URDF must be a tree)")
            seen.add(n)
            if n not in parent_of:
                js = JointSpec(f"{name}_fixed", "fixed", root, name, origin="com")
                joints.append(Joint(js, links[name].com.copy(), np.array([0.0, 0.0, 1.0])))
                links[name].frame = links[name].com.copy()
                parent_of[name] = root
                warnings.append(f"Link '{name}' had no parent joint; fixed it to '{root}'")
                break
            n = parent_of[n]

    return Robot(cfg, links, joints, root, warnings)


# --------------------------------------------------------------------------- meshes

def link_mesh(link: Link, scale: float, quality: float = 1.0):
    """Vertices in the link frame, in metres."""
    vs, fs, off = [], [], 0
    for p in link.parts:
        v, f = p.mesh(quality)
        if len(f):
            vs.append((v - link.frame) * scale)
            fs.append(f + off)
            off += len(v)
    if not vs:
        return np.zeros((0, 3)), np.zeros((0, 3), dtype=np.int64)
    return np.vstack(vs), np.vstack(fs)


def stl_bytes(v, f, name="mesh") -> bytes:
    tri = v[f]                                   # (M,3,3)
    n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    ln = np.linalg.norm(n, axis=1, keepdims=True)
    n = np.divide(n, ln, out=np.zeros_like(n), where=ln > 0)
    rec = np.zeros(len(f), dtype=[("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])
    rec["n"], rec["v"] = n, tri
    header = f"step2urdf {name}".encode()[:80].ljust(80, b" ")
    return header + struct.pack("<I", len(f)) + rec.tobytes()


# --------------------------------------------------------------------------- URDF

def _fmt(x):
    return " ".join(f"{float(a):.9g}" if abs(a) > 1e-12 else "0" for a in x)


def urdf_xml(robot: Robot) -> str:
    cfg, s = robot.config, robot.config.unit_scale
    rname = sanitize(cfg.robot_name)
    root = ET.Element("robot", name=rname)
    root.append(ET.Comment(" Generated by step2urdf from a STEP file. Units: metres, kilograms, radians. "))

    def mesh_uri(link):
        if cfg.mesh_path == "relative":
            return f"../meshes/{link}.stl"
        return f"package://{rname}/meshes/{link}.stl"

    for name, link in robot.links.items():
        le = ET.SubElement(root, "link", name=name)
        inr = ET.SubElement(le, "inertial")
        ET.SubElement(inr, "origin", xyz=_fmt((link.com - link.frame) * s), rpy="0 0 0")
        ET.SubElement(inr, "mass", value=f"{max(link.mass, 1e-6):.9g}")
        I = link.inertia.copy()
        I[np.abs(I) < 1e-9 * max(np.abs(np.diag(I)).max(), 1e-30)] = 0.0   # strip float noise
        ev = np.linalg.eigvalsh(I) if link.parts else [1]
        if min(ev) <= 0:  # degenerate geometry (e.g. a flat sheet): nudge to positive definite
            I = I + np.eye(3) * (abs(min(ev)) + 1e-9)
        ET.SubElement(inr, "inertia", ixx=f"{I[0,0]:.9g}", ixy=f"{I[0,1]:.9g}", ixz=f"{I[0,2]:.9g}",
                      iyy=f"{I[1,1]:.9g}", iyz=f"{I[1,2]:.9g}", izz=f"{I[2,2]:.9g}")
        if not link.parts:
            continue
        vis = ET.SubElement(le, "visual")
        ET.SubElement(vis, "origin", xyz="0 0 0", rpy="0 0 0")
        ET.SubElement(ET.SubElement(vis, "geometry"), "mesh", filename=mesh_uri(name))
        mat = ET.SubElement(vis, "material", name=f"{name}_material")
        ET.SubElement(mat, "color", rgba=_fmt(link.color))
        if cfg.collision == "mesh":
            col = ET.SubElement(le, "collision")
            ET.SubElement(col, "origin", xyz="0 0 0", rpy="0 0 0")
            ET.SubElement(ET.SubElement(col, "geometry"), "mesh", filename=mesh_uri(name))
        elif cfg.collision == "box":
            lo, hi = _bbox(link.parts)
            col = ET.SubElement(le, "collision")
            ET.SubElement(col, "origin", xyz=_fmt(((lo + hi) / 2 - link.frame) * s), rpy="0 0 0")
            ET.SubElement(ET.SubElement(col, "geometry"), "box", size=_fmt((hi - lo) * s))

    for j in robot.joints:
        js = j.spec
        parent = robot.links[js.parent]
        if j.note:
            root.append(ET.Comment(f" {js.name}: {j.note} "))
        je = ET.SubElement(root, "joint", name=js.name, type=js.type)
        ET.SubElement(je, "parent", link=js.parent)
        ET.SubElement(je, "child", link=js.child)
        ET.SubElement(je, "origin", xyz=_fmt((j.origin_world - parent.frame) * s), rpy="0 0 0")
        if js.type != "fixed":
            ET.SubElement(je, "axis", xyz=_fmt(j.axis))
        if js.type in ("revolute", "prismatic"):
            lo = js.lower if js.lower is not None else (-math.pi if js.type == "revolute" else -0.1)
            hi = js.upper if js.upper is not None else (math.pi if js.type == "revolute" else 0.1)
            ET.SubElement(je, "limit", lower=f"{lo:.9g}", upper=f"{hi:.9g}",
                          effort=f"{js.effort:.9g}", velocity=f"{js.velocity:.9g}")
        elif js.type == "continuous":
            ET.SubElement(je, "limit", effort=f"{js.effort:.9g}", velocity=f"{js.velocity:.9g}")

    xml = minidom.parseString(ET.tostring(root, encoding="unicode")).toprettyxml(indent="  ")
    return xml.replace('<?xml version="1.0" ?>', '<?xml version="1.0"?>', 1)


PACKAGE_XML = """<?xml version="1.0"?>
<package format="3">
  <name>{name}</name>
  <version>0.1.0</version>
  <description>URDF description of {name}, generated from CAD by step2urdf.</description>
  <maintainer email="you@example.com">you</maintainer>
  <license>TODO</license>
  <buildtool_depend condition="$ROS_VERSION == 1">catkin</buildtool_depend>
  <buildtool_depend condition="$ROS_VERSION == 2">ament_cmake</buildtool_depend>
  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>joint_state_publisher_gui</exec_depend>
  <exec_depend>rviz2</exec_depend>
  <export>
    <build_type condition="$ROS_VERSION == 1">catkin</build_type>
    <build_type condition="$ROS_VERSION == 2">ament_cmake</build_type>
  </export>
</package>
"""

CMAKE = """cmake_minimum_required(VERSION 3.8)
project({name})
if("$ENV{{ROS_VERSION}}" STREQUAL "2")
  find_package(ament_cmake REQUIRED)
  install(DIRECTORY urdf meshes DESTINATION share/${{PROJECT_NAME}})
  ament_package()
else()
  find_package(catkin REQUIRED)
  catkin_package()
  install(DIRECTORY urdf meshes DESTINATION ${{CATKIN_PACKAGE_SHARE_DESTINATION}})
endif()
"""


def summary(robot: Robot) -> str:
    lines = [f"Robot '{robot.config.robot_name}': {len(robot.links)} links, {len(robot.joints)} joints, root '{robot.root}'"]
    total = sum(l.mass for l in robot.links.values())
    for n, l in robot.links.items():
        lines.append(f"  link  {n:<24} {l.mass:9.4f} kg  ({len(l.parts)} part(s))")
    for j in robot.joints:
        lines.append(f"  joint {j.spec.name:<24} {j.spec.type:<10} {j.spec.parent} -> {j.spec.child}"
                     + (f"  axis {_fmt(j.axis)}" if j.spec.type != 'fixed' else "")
                     + (f"   [{j.note}]" if j.note else ""))
    lines.append(f"  total mass {total:.4f} kg")
    return "\n".join(lines)


def package_files(robot: Robot) -> dict[str, bytes]:
    """All files of the robot package, keyed by relative path."""
    cfg = robot.config
    name = sanitize(cfg.robot_name)
    files = {f"{name}/urdf/{name}.urdf": urdf_xml(robot).encode()}
    for lname, link in robot.links.items():
        if link.parts:
            v, f = link_mesh(link, cfg.unit_scale, cfg.mesh_quality)
            files[f"{name}/meshes/{lname}.stl"] = stl_bytes(v, f, lname)
    files[f"{name}/package.xml"] = PACKAGE_XML.format(name=name).encode()
    files[f"{name}/CMakeLists.txt"] = CMAKE.format(name=name).encode()
    try:
        import yaml
        files[f"{name}/step2urdf_config.yaml"] = yaml.safe_dump(_plain(cfg.to_dict()), sort_keys=False).encode()
    except ImportError:  # pragma: no cover
        files[f"{name}/step2urdf_config.json"] = json.dumps(_plain(cfg.to_dict()), indent=2).encode()
    from .viewer import preview_html   # local import avoids a circular dependency
    files[f"{name}/preview.html"] = preview_html(robot).encode()
    files[f"{name}/conversion_report.txt"] = (summary(robot) + "\n\nWarnings:\n"
                                              + "\n".join("  - " + w for w in robot.warnings or ["none"])).encode()
    return files


def _plain(o):
    if isinstance(o, dict):
        return {k: _plain(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [_plain(v) for v in o]
    if isinstance(o, np.ndarray):
        return o.tolist()
    if isinstance(o, np.generic):
        return o.item()
    return o


def write_package(robot: Robot, out_dir: str) -> str:
    for rel, data in package_files(robot).items():
        path = os.path.join(out_dir, rel)
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "wb") as fh:
            fh.write(data)
    name = sanitize(robot.config.robot_name)
    return os.path.join(out_dir, name, "urdf", f"{name}.urdf")


def zip_package(robot: Robot) -> bytes:
    buf = io.BytesIO()
    with zipfile.ZipFile(buf, "w", zipfile.ZIP_DEFLATED) as z:
        for rel, data in package_files(robot).items():
            z.writestr(rel, data)
    return buf.getvalue()
