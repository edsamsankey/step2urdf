"""Edge cases: unnamed multi-body STEP, prismatic joints, invalid kinematic trees."""
import os, sys, tempfile
import numpy as np, yourdfpy
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox
from OCP.BRep import BRep_Builder
from OCP.TopoDS import TopoDS_Compound
from OCP.gp import gp_Pnt
from OCP.STEPControl import STEPControl_Writer, STEPControl_AsIs
from step2urdf.cad import load_step
from step2urdf.builder import Config, JointSpec, LinkSpec, build, default_config, write_package

def plain_multibody_step(path):
    """A STEP with two loose solids, no assembly, no names (like many 'export all' files)."""
    comp, bld = TopoDS_Compound(), BRep_Builder()
    bld.MakeCompound(comp)
    bld.Add(comp, BRepPrimAPI_MakeBox(gp_Pnt(0, 0, 0), 200, 200, 20).Shape())    # rail
    bld.Add(comp, BRepPrimAPI_MakeBox(gp_Pnt(20, 80, 20), 40, 40, 30).Shape())   # carriage
    w = STEPControl_Writer(); w.Transfer(comp, STEPControl_AsIs); w.Write(path)

def test_multibody_and_prismatic():
    with tempfile.TemporaryDirectory() as tmp:
        step = os.path.join(tmp, "slider.step"); plain_multibody_step(step)
        parts = load_step(step)
        assert len(parts) == 2, [p.name for p in parts]
        big, small = sorted(parts, key=lambda p: -p.volume)
        # default scaffold: works with zero config
        r = build(parts, default_config(parts, "slider"))
        assert len(r.links) == 2 and r.joints[0].spec.type == "fixed"
        cfg = Config(robot_name="slider", root="rail", mesh_path="relative",
                     links=[LinkSpec("rail", [big.name]), LinkSpec("carriage", [small.name])],
                     joints=[JointSpec("slide", "prismatic", "rail", "carriage", origin="com",
                                       axis=[1, 0, 0], lower=0.0, upper=0.14)])
        robot = build(parts, cfg)
        u = yourdfpy.URDF.load(write_package(robot, tmp))
        u.update_cfg({"slide": 0.1})
        p = u.get_transform("carriage", "rail")[:3, 3]
        assert np.allclose(p, [0.04 + 0.1, 0.1, 0.035], atol=1e-9), p

def test_errors_are_clear():
    with tempfile.TemporaryDirectory() as tmp:
        step = os.path.join(tmp, "s.step"); plain_multibody_step(step)
        parts = load_step(step)
        a, b = parts[0].name, parts[1].name
        bad = [
            (Config(links=[LinkSpec("x", [a]), LinkSpec("y", [b])],
                    joints=[JointSpec("j1", "revolute", "x", "y"), JointSpec("j2", "revolute", "y", "x")]), "root"),
            (Config(root="x", links=[LinkSpec("x", [a]), LinkSpec("y", [b])],
                    joints=[JointSpec("j", "hinge", "x", "y")]), "type must be"),
            (Config(root="x", links=[LinkSpec("x", [a]), LinkSpec("y", [b])],
                    joints=[JointSpec("j", "fixed", "x", "nope")]), "unknown link"),
        ]
        for cfg, msg in bad:
            try:
                build(parts, cfg); raise AssertionError("expected failure: " + msg)
            except ValueError as e:
                assert msg in str(e), (msg, str(e))

if __name__ == "__main__":
    for t in (test_multibody_and_prismatic, test_errors_are_clear):
        t(); print("PASS", t.__name__)
