"""End-to-end check: STEP -> URDF -> load with yourdfpy -> forward kinematics must reproduce the CAD."""
import math, os, sys, tempfile
import numpy as np, trimesh, yourdfpy
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from step2urdf.cad import load_step
from step2urdf.builder import Config, build, write_package
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
STEP = os.path.join(HERE, "..", "examples", "test_arm.step")
CFG = os.path.join(HERE, "..", "examples", "test_arm.yaml")

def load_robot(tmp, mesh_path="relative"):
    parts = load_step(STEP)
    cfg = Config.from_dict(yaml.safe_load(open(CFG))); cfg.mesh_path = mesh_path
    robot = build(parts, cfg)
    urdf = write_package(robot, tmp)
    return parts, robot, yourdfpy.URDF.load(urdf, build_collision_scene_graph=True, load_collision_meshes=True)

def test_zero_pose_matches_cad():
    with tempfile.TemporaryDirectory() as tmp:
        parts, robot, u = load_robot(tmp)
        assert set(u.link_map) == {"base_link", "upper_arm", "forearm", "gripper"}
        assert len(u.actuated_joint_names) == 2
        for lname, link in robot.links.items():
            T = u.get_transform(lname, "base_link")
            mesh = list(u.scene.geometry.values())  # ensure meshes loaded
            lo, hi = np.min([p.bbox_min for p in link.parts], 0), np.max([p.bbox_max for p in link.parts], 0)
            geom = u.link_map[lname].visuals[0].geometry.mesh.filename
            m = trimesh.load(os.path.join(tmp, "test_arm", "urdf", geom))
            m.apply_transform(T)
            assert np.allclose(m.bounds[0], lo / 1000, atol=2e-4), (lname, m.bounds, lo)
            assert np.allclose(m.bounds[1], hi / 1000, atol=2e-4), (lname, m.bounds, hi)
            # inertial COM expressed back in world must equal CAD COM
            com_local = u.link_map[lname].inertial.origin[:3, 3]
            com_world = (T @ np.r_[com_local, 1])[:3]
            assert np.allclose(com_world, link.com / 1000, atol=1e-6), lname

def test_joint_motion():
    with tempfile.TemporaryDirectory() as tmp:
        _, robot, u = load_robot(tmp)
        u.update_cfg({"shoulder_yaw": math.pi / 2, "elbow_pitch": 0.0})
        # forearm extends along +x at zero; after 90 deg yaw it must extend along +y
        T = u.get_transform("gripper", "base_link")
        p = T[:3, 3]
        assert np.allclose(p, [0, 0.135, 0.275], atol=1e-9), p
        # elbow pitch should rotate the gripper about the pin at z=0.26
        u.update_cfg({"shoulder_yaw": 0.0, "elbow_pitch": math.pi / 2})
        p = u.get_transform("gripper", "base_link")[:3, 3]
        r0 = np.hypot(0.135, 0.275 - 0.26)
        assert abs(np.hypot(p[0], p[2] - 0.26) - r0) < 1e-6, p

def test_mass_properties():
    with tempfile.TemporaryDirectory() as tmp:
        _, robot, u = load_robot(tmp)
        fa = u.link_map["forearm"].inertial
        assert abs(fa.mass - 135000e-9 * 7850) < 1e-9
        # box 150x30x30 mm about its COM: Ixx = m/12 (b^2 + c^2)
        m = fa.mass
        assert math.isclose(fa.inertia[0, 0], m / 12 * (0.03**2 + 0.03**2), rel_tol=1e-7)
        assert math.isclose(fa.inertia[1, 1], m / 12 * (0.15**2 + 0.03**2), rel_tol=1e-7)
        assert abs(u.link_map["gripper"].inertial.mass - 0.25) < 1e-12
        for l in u.link_map.values():
            assert np.all(np.linalg.eigvalsh(l.inertial.inertia) > 0)

if __name__ == "__main__":
    for t in (test_zero_pose_matches_cad, test_joint_motion, test_mass_properties):
        t(); print("PASS", t.__name__)
