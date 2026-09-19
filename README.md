# step2urdf

**Convert a STEP CAD assembly into a ready-to-use URDF robot package.**

Upload your CAD, group the parts that move together, define the joints, and get back a URDF with
meshes, exact mass and inertia computed from the CAD solids, and a ROS package — plus a 3D preview
with joint sliders so you can verify the kinematics before you use it.

![step2urdf web app](docs/screenshot.png)

## Why

Exporting CAD to URDF by hand means measuring joint positions, computing inertia tensors and
writing XML — slow, and easy to get wrong. This does it from the CAD file itself:

- **Exact mass properties** from the B-rep solids, not estimated from meshes
- **Automatic joint axes** — it finds the shaft or bore that two links share
- **Verify before you trust it** — a browser preview with one slider per joint
- **Repeatable** — save the config, re-run it after every CAD revision

Tested end to end: the reassembled URDF matches the source CAD to within 0.2 mm, inertias match
analytic formulas, and the output loads and simulates in MuJoCo.

## Install

**Python 3.12 or 3.13.** The OpenCascade CAD kernel has no build for Python 3.14 yet,
so `pip install` will fail on it. Check with `python --version`.

```bash
python -m venv venv
# Windows:  venv\Scripts\python.exe -m pip install -r requirements.txt
# Linux/macOS:
./venv/bin/pip install -r requirements.txt
```

## Web app (upload STEP, download URDF)

```bash
# Windows
venv\Scripts\python.exe -m streamlit run app.py
# Linux/macOS
./venv/bin/python -m streamlit run app.py
```

Calling the venv's python directly means you never have to "activate" it,
which avoids PowerShell's script-execution block on Windows.

1. Upload a `.step` / `.stp` file. Each part keeps its CAD name, colour and position.
2. **Group parts into links.** Give parts the same link name to weld them into one rigid body
   (e.g. a bracket and its screws). Blank = merge into the root link.
3. **Define joints.** One row per non-root link: type, parent, origin, axis, limits.
   Leave origin and axis as `auto` and the tool finds the shaft/bore shared by the two links.
4. Click **Build URDF**, move the sliders in the preview to check each joint, then download the zip.
   Save the config (.yaml) so you can reload it after your next CAD revision.

## Command line

```bash
# Quick: every part becomes a link, all fixed to the heaviest part
python -m step2urdf robot.step -o out/

# Proper workflow: write a config, edit it, build
python -m step2urdf robot.step --init-config robot.yaml
python -m step2urdf robot.step -c robot.yaml -o out/ --zip

# One rigid body (e.g. a gripper or a static fixture)
python -m step2urdf part.step --single -o out/
```

Options: `--density 7850`, `--mesh-path relative` (PyBullet/MuJoCo/Isaac) or `package` (ROS),
`--collision mesh|box|none`, `--quality 2` for smoother meshes.
`pip install -e .` also gives you a `step2urdf` command.

## Config file

See `examples/test_arm.yaml`. Lengths are CAD millimetres; limits are radians (revolute) or metres (prismatic).

```yaml
robot_name: my_arm
density: 2700               # default, kg/m^3
root: base_link
links:
  - name: base_link
    parts: [Base, "M8_Screw*"]   # names or glob patterns
  - name: upper_arm
    parts: [UpperArm]
    density: 7850                # per-link override
  - name: gripper
    parts: [Gripper]
    mass: 0.25                   # measured mass wins; inertia is scaled to match
joints:
  - name: shoulder
    type: revolute               # fixed | revolute | continuous | prismatic
    parent: base_link
    child: upper_arm
    origin: auto                 # auto | com | [x, y, z]
    axis: auto                   # auto | [x, y, z]
    lower: -3.14
    upper: 3.14
    effort: 10
    velocity: 1
```

## Output

```
my_arm/
  urdf/my_arm.urdf          the robot description
  meshes/<link>.stl         one binary STL per link, in metres, in the link frame
  preview.html              open in a browser: 3D view with joint sliders
  package.xml, CMakeLists.txt   drop into a ROS 1 or ROS 2 workspace as-is
  step2urdf_config.yaml     the exact config used (re-run after CAD changes)
  conversion_report.txt     masses, joints, and any warnings
```

Use in ROS 2: copy the folder into `ws/src`, `colcon build`, then
`ros2 launch urdf_tutorial display.launch.py model:=$(ros2 pkg prefix my_arm)/share/my_arm/urdf/my_arm.urdf`.

## How it works

- **Geometry** is read with OpenCascade (the kernel behind FreeCAD), walking the STEP assembly tree
  so nested sub-assemblies and repeated part instances are placed correctly.
- **Mass properties** come from the exact B-rep solids, not the meshes. Parts in a link are combined
  with the parallel-axis theorem. Units are converted to SI (m, kg, kg·m²).
- **Frames**: the root link frame is the CAD origin. Every other link frame sits at its joint origin,
  aligned with the CAD axes, so joint origins are pure translations and axes read like CAD directions.
- **Auto joint axes**: the tool looks for cylindrical faces (pins, shafts, bores, bearing seats) on either
  link whose axis passes through the other link, picks the largest, and places the origin on that axis
  where the two links meet. The URDF records what it picked as a comment next to each joint.

## Limits worth knowing

- STEP files do not store joints, so kinematics always come from you (or `auto`). Check `auto` results
  in the preview; if a link has several shafts, set the axis direction to disambiguate or type the origin.
- Mass assumes uniform density per link. Use `mass:` for parts with motors, batteries, or hollow castings.
- Surface-only models (no closed solids) get bounding-box mass estimates; you'll see a warning.
- Collision meshes are the visual meshes. For fast physics, choose `box` or replace them with convex hulls.
- Link frames are CAD-axis-aligned (rpy = 0). If your controller expects a specific joint frame
  convention (e.g. DH), rotate in your controller or edit the URDF.

## Tests

```bash
pip install yourdfpy trimesh
python tests/test_roundtrip.py && python tests/test_edge_cases.py
```

`examples/make_test_arm.py` builds the sample STEP assembly used by the tests.
