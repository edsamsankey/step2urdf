"""
STEP to URDF web app.   Run with:   streamlit run app.py
"""
import hashlib
import os
import re
import tempfile

import numpy as np
import pandas as pd
import streamlit as st
import streamlit.components.v1 as components
import yaml

from step2urdf.builder import (JOINT_TYPES, Config, JointSpec, LinkSpec, _match, _plain, build,
                               summary, urdf_xml, zip_package)
from step2urdf.cad import load_step, sanitize
from step2urdf.viewer import preview_html

# Work on both older and newer Streamlit releases.
_ver = tuple(int(x) for x in re.findall(r"\d+", st.__version__)[:2])
STRETCH = {"width": "stretch"} if _ver >= (1, 46) else {"use_container_width": True}


def show_html(html: str, height: int):
    if hasattr(st, "iframe") and _ver >= (1, 60):
        st.iframe(html, height=height)
    else:
        components.html(html, height=height)


st.set_page_config(page_title="STEP to URDF", page_icon="⚙️", layout="wide")
st.markdown("""<style>
 .block-container{padding-top:2rem;max-width:1400px}
 h1{font-weight:600;letter-spacing:-.01em}
 div[data-testid="stDataEditor"]{border:1px solid #d5dbe1;border-radius:4px}
</style>""", unsafe_allow_html=True)


@st.cache_resource(show_spinner=False, max_entries=3)
def parse_step(digest: str, _data: bytes):
    with tempfile.NamedTemporaryFile(suffix=".step", delete=False) as fh:
        fh.write(_data)
        path = fh.name
    try:
        return load_step(path)
    finally:
        os.unlink(path)


def vec_text(value) -> str:
    if isinstance(value, str):
        return value
    return ", ".join(f"{float(v):g}" for v in value)


def parse_vec(text, field, joint):
    t = str(text or "auto").strip().lower()
    if t in ("auto", "com"):
        return t
    nums = [x for x in re.split(r"[,\s;]+", t.strip("[]()")) if x]
    try:
        vals = [float(x) for x in nums]
    except ValueError:
        vals = []
    if len(vals) != 3:
        raise ValueError(f"Joint '{joint}': {field} must be 'auto'"
                         + (", 'com'" if field == "origin" else "") + " or three numbers like 0, 0, 1")
    return vals


def num_or_none(v):
    return None if v is None or pd.isna(v) else float(v)


# ------------------------------------------------------------------ state helpers

def reset_links(parts, mode):
    if mode == "single":
        link_col = ["base_link"] * len(parts)
    else:
        link_col = [p.name for p in parts]
    st.session_state.links_df = pd.DataFrame({
        "Part": [p.name for p in parts],
        "Volume (cm³)": [round(p.volume / 1000, 3) for p in parts],
        "Link": link_col,
    })
    st.session_state.ver += 1
    st.session_state.joint_rows = {}
    st.session_state.root = None


def apply_config(cfg: Config, parts):
    names = [p.name for p in parts]
    assign = {}
    for ls in cfg.links:
        for pat in ls.parts:
            for n in _match(pat, names):
                assign.setdefault(n, sanitize(ls.name))
    reset_links(parts, "per_part")
    st.session_state.links_df["Link"] = [assign.get(n, "") for n in names]
    st.session_state.joint_rows = {
        sanitize(j.child): {"Joint": j.name, "Type": j.type, "Parent": sanitize(j.parent), "Child": sanitize(j.child),
                            "Origin": vec_text(j.origin), "Axis": vec_text(j.axis), "Lower": j.lower,
                            "Upper": j.upper, "Effort": j.effort, "Velocity": j.velocity}
        for j in cfg.joints}
    st.session_state.root = sanitize(cfg.root) if cfg.root else None
    st.session_state.link_extras = {sanitize(l.name): l for l in cfg.links}
    for k in ("density", "mesh_path", "collision"):
        st.session_state[f"opt_{k}"] = getattr(cfg, k)
    st.session_state.opt_name = cfg.robot_name
    st.session_state.opt_density = float(cfg.density)


def default_joint(child, root):
    return {"Joint": f"{child}_joint", "Type": "fixed", "Parent": root, "Child": child, "Origin": "auto",
            "Axis": "auto", "Lower": None, "Upper": None, "Effort": 10.0, "Velocity": 1.0}


# ------------------------------------------------------------------ page

st.title("STEP to URDF")
st.write("Upload a CAD assembly, tell it which parts move together and how they are jointed, "
         "and download a URDF package with meshes, inertias and a ROS package file.")

uploaded = st.file_uploader("STEP file", type=["step", "stp", "STEP", "STP"])
if not uploaded:
    st.info("Drop a .step or .stp file above to begin. Assemblies work best: each part keeps its name, "
            "colour and position from your CAD tool.")
    st.stop()

data = uploaded.getvalue()
digest = hashlib.sha1(data).hexdigest()
with st.spinner("Reading geometry and computing mass properties..."):
    try:
        parts = parse_step(digest, data)
    except Exception as e:
        st.error(f"Could not read this STEP file: {e}. Check that it exports solids (not just surfaces) "
                 "and try re-exporting as AP214 or AP242.")
        st.stop()

if st.session_state.get("digest") != digest:
    st.session_state.digest = digest
    st.session_state.ver = 0
    st.session_state.link_extras = {}
    st.session_state.opt_name = sanitize(os.path.splitext(uploaded.name)[0])
    reset_links(parts, "per_part")
for k, v in (("opt_density", 2700.0), ("opt_mesh_path", "package"), ("opt_collision", "mesh")):
    st.session_state.setdefault(k, v)
if st.session_state.get("pending_cfg") is not None:      # apply a loaded config before widgets exist
    try:
        apply_config(Config.from_dict(st.session_state.pending_cfg), parts)
        st.toast("Config applied")
    except Exception as e:
        st.error(f"Config not applied: {e}")
    st.session_state.pending_cfg = None

# ---- sidebar settings
with st.sidebar:
    st.header("Settings")
    robot_name = st.text_input("Robot name", key="opt_name")
    density = st.number_input("Default density (kg/m³)", 1.0, 30000.0, step=50.0, key="opt_density",
                              help="Aluminium 2700, steel 7850, PLA 1240, ABS 1050. Override per link in a config file.")
    mesh_path = st.radio("Mesh paths", ["package", "relative"], key="opt_mesh_path",
                         format_func=lambda x: {"package": "package:// (ROS, RViz, Gazebo)",
                                                "relative": "../meshes (PyBullet, MuJoCo, Isaac)"}[x])
    collision = st.radio("Collision geometry", ["mesh", "box", "none"], key="opt_collision",
                         format_func=lambda x: {"mesh": "Same mesh as visual", "box": "Bounding box per link",
                                                "none": "None"}[x])
    quality = st.slider("Mesh detail", 0.25, 4.0, 1.0, 0.25, help="Higher is smoother but produces larger STL files.")
    st.divider()
    cfg_file = st.file_uploader("Load a saved config (.yaml)", type=["yaml", "yml", "json"])
    if cfg_file and st.button("Apply config"):
        try:
            st.session_state.pending_cfg = yaml.safe_load(cfg_file.getvalue())
            st.rerun()
        except yaml.YAMLError as e:
            st.error(f"That file is not valid YAML: {e}")

c1, c2, c3 = st.columns(3)
c1.metric("Parts", len(parts))
c2.metric("Estimated mass", f"{sum(p.volume for p in parts) * 1e-9 * density:.3f} kg", help="At the default density")
extent = np.max([p.bbox_max for p in parts], 0) - np.min([p.bbox_min for p in parts], 0)
c3.metric("Size (mm)", " × ".join(f"{e:.0f}" for e in extent))
if any(p.approximate for p in parts):
    st.warning("Some parts are surfaces rather than closed solids; their mass properties are estimated from bounding boxes.")

# ---- 1. links
st.subheader("1. Group parts into links")
st.write("Parts with the same link name are welded together into one rigid body. "
         "Leave a link name blank to merge that part into the root link.")
b1, b2, _ = st.columns([1, 1, 3])
if b1.button("One link per part"):
    reset_links(parts, "per_part"); st.rerun()
if b2.button("Everything as one link"):
    reset_links(parts, "single"); st.rerun()

links_df = st.data_editor(
    st.session_state.links_df, key=f"links_{st.session_state.ver}", hide_index=True, **STRETCH,
    disabled=["Part", "Volume (cm³)"], height=min(38 + 35 * len(parts), 420),
    column_config={"Link": st.column_config.TextColumn("Link", help="Type the same name on several rows to group them")})

link_names = list(dict.fromkeys(sanitize(x) for x in links_df["Link"] if str(x or "").strip()))
if not link_names:
    st.error("Give at least one part a link name.")
    st.stop()

vol_by_link = {}
for _, r in links_df.iterrows():
    if str(r["Link"] or "").strip():
        vol_by_link[sanitize(r["Link"])] = vol_by_link.get(sanitize(r["Link"]), 0) + r["Volume (cm³)"]
default_root = st.session_state.root if st.session_state.root in link_names else max(vol_by_link, key=vol_by_link.get)
root = st.selectbox("Root link (fixed to the world)", link_names, index=link_names.index(default_root))
st.session_state.root = root

# ---- 2. joints
st.subheader("2. Define joints")
st.write("Each non-root link gets one joint to its parent. Origin and axis are in CAD millimetres. "
         "**auto** finds the shaft or bore the two links share; **com** uses the child's centre of mass; "
         "or type a point such as `0, 0, 60`. Limits are in radians (or metres for prismatic).")

children = [l for l in link_names if l != root]
rows = st.session_state.joint_rows
jkey = f"joints_{st.session_state.ver}_{hashlib.md5(('|'.join(link_names) + '#' + root).encode()).hexdigest()[:8]}"
if jkey not in st.session_state.setdefault("joint_base", {}):
    base = []
    for ch in children:
        r = dict(rows.get(ch) or default_joint(ch, root))
        if r["Parent"] not in link_names or r["Parent"] == ch:
            r["Parent"] = root
        r["Child"] = ch
        base.append(r)
    st.session_state.joint_base[jkey] = pd.DataFrame(
        base, columns=["Joint", "Type", "Parent", "Child", "Origin", "Axis", "Lower", "Upper", "Effort", "Velocity"])

if children:
    joints_df = st.data_editor(
        st.session_state.joint_base[jkey], key=jkey, hide_index=True, **STRETCH, num_rows="fixed",
        disabled=["Child"],
        column_config={
            "Type": st.column_config.SelectboxColumn("Type", options=list(JOINT_TYPES), required=True),
            "Parent": st.column_config.SelectboxColumn("Parent", options=link_names, required=True),
            "Origin": st.column_config.TextColumn("Origin (mm)", help="auto, com, or x, y, z"),
            "Axis": st.column_config.TextColumn("Axis", help="auto, or a direction like 0, 0, 1"),
            "Lower": st.column_config.NumberColumn("Lower", format="%.3f"),
            "Upper": st.column_config.NumberColumn("Upper", format="%.3f"),
            "Effort": st.column_config.NumberColumn("Effort", format="%.2f"),
            "Velocity": st.column_config.NumberColumn("Velocity", format="%.2f"),
        })
    for _, r in joints_df.iterrows():
        rows[r["Child"]] = r.to_dict()
else:
    joints_df = pd.DataFrame()
    st.caption("Only one link, so there are no joints. The URDF will describe a single rigid body.")

# ---- assemble config
try:
    extras = st.session_state.link_extras
    links = []
    for ln in link_names:
        pats = [p for p, l in zip(links_df["Part"], links_df["Link"]) if str(l or "").strip() and sanitize(l) == ln]
        ex = extras.get(ln)
        links.append(LinkSpec(ln, pats, density=ex.density if ex else None, mass=ex.mass if ex else None,
                              color=ex.color if ex else None))
    joints = []
    for _, r in joints_df.iterrows():
        jn = sanitize(str(r["Joint"] or f"{r['Child']}_joint"))
        joints.append(JointSpec(jn, r["Type"] or "fixed", r["Parent"] or root, r["Child"],
                                origin=parse_vec(r["Origin"], "origin", jn), axis=parse_vec(r["Axis"], "axis", jn),
                                lower=num_or_none(r["Lower"]), upper=num_or_none(r["Upper"]),
                                effort=num_or_none(r["Effort"]) or 10.0, velocity=num_or_none(r["Velocity"]) or 1.0))
    if len({j.name for j in joints}) != len(joints):
        raise ValueError("Joint names must be unique")
    cfg = Config(robot_name=sanitize(robot_name or "robot"), density=density, mesh_quality=quality,
                 mesh_path=mesh_path, collision=collision, root=root, links=links, joints=joints)
except ValueError as e:
    st.error(str(e))
    st.stop()

# ---- 3. build
st.subheader("3. Check and download")
signature = hashlib.md5(repr(_plain(cfg.to_dict())).encode()).hexdigest()
if st.button("Build URDF", type="primary"):
    with st.spinner("Meshing links and writing URDF..."):
        try:
            robot = build(parts, cfg)
            st.session_state.result = {"sig": signature, "zip": zip_package(robot), "urdf": urdf_xml(robot),
                                       "summary": summary(robot), "warnings": robot.warnings,
                                       "preview": preview_html(robot), "name": cfg.robot_name,
                                       "config": yaml.safe_dump(_plain(cfg.to_dict()), sort_keys=False)}
        except ValueError as e:
            st.session_state.result = None
            st.error(str(e))

res = st.session_state.get("result")
if res:
    if res["sig"] != signature:
        st.info("Settings changed since the last build. Build again to update the download.")
    for w in res["warnings"]:
        st.warning(w)
    d1, d2, d3 = st.columns(3)
    d1.download_button("Download URDF package (.zip)", res["zip"], f"{res['name']}.zip", "application/zip", type="primary")
    d2.download_button("Download .urdf only", res["urdf"], f"{res['name']}.urdf", "application/xml")
    d3.download_button("Download config (.yaml)", res["config"], f"{res['name']}_config.yaml", "text/yaml",
                       help="Reuse with the command line tool or load it here next time")
    view_col, info_col = st.columns([3, 2])
    with view_col:
        show_html(res["preview"], 560)
        st.caption("Move the sliders to check that every joint rotates or slides about the right axis.")
    with info_col:
        st.code(res["summary"], language=None)
        with st.expander("URDF source"):
            st.code(res["urdf"], language="xml")
