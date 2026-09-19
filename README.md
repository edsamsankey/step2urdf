# step2urdf

**Convert a STEP CAD assembly into a ready-to-use URDF robot package.**

Upload your CAD file, group the parts that move together, define the joints, and get back a URDF
with meshes, exact mass and inertia computed from the CAD solids, and a ROS package — plus a 3D
preview with a slider per joint so you can verify the kinematics before you trust them.

![step2urdf web app](docs/screenshot.png)

---

## Why this exists

Going from CAD to URDF by hand means measuring joint positions off drawings, computing inertia
tensors, converting units, and writing XML. It is slow and easy to get subtly wrong — and a wrong
inertia tensor produces a simulation that looks fine and behaves nothing like your robot.

step2urdf reads the geometry itself:

- **Exact mass properties** computed from the B-rep solids, not approximated from meshes
- **Automatic joint axes** — it finds the shaft, pin or bore that two links share and puts the joint there
- **Verify before you trust it** — a browser preview with one slider per joint
- **Repeatable** — save the config and re-run it after every CAD revision instead of starting over

Verified end to end: the reassembled URDF matches the source CAD to within 0.2 mm, inertias match
analytic formulas for known shapes, and the output loads and simulates correctly in MuJoCo.

---

## Requirements

- **Python 3.12 or 3.13.** The OpenCascade CAD kernel has no build for 3.14 yet, so installation
  will fail on it. Check with `python --version`.
- About 500 MB of disk for the CAD kernel.
- Any OS: Windows, macOS or Linux.

---

## Install

```bash
git clone https://github.com/YOUR-USERNAME/step2urdf.git
cd step2urdf
python -m venv venv
```

Then install into that environment:

```bash
# Windows
venv\Scripts\python.exe -m pip install -r requirements.txt

# macOS / Linux
./venv/bin/pip install -r requirements.txt
```

Calling the venv's Python directly means you never have to "activate" it, which sidesteps
PowerShell's script-execution block on Windows.

---

## Run the web app

```bash
# Windows
venv\Scripts\python.exe -m streamlit run app.py

# macOS / Linux
./venv/bin/python -m streamlit run app.py
```

Your browser opens at `http://localhost:8501`. Windows users can also just double-click
`run_windows.bat`, which handles setup and launch in one step.

### Using it

1. **Upload** a `.step` or `.stp` file. Assemblies work best — each part keeps its CAD name,
   colour and position.

2. **Group parts into links.** Type the same link name on several rows to weld those parts into one
   rigid body. A bracket, its bearings, its spacers and the servos bolted to it all move together,
   so they are one link. Leave a name blank to merge that part into the root link.

3. **Define joints.** One row per non-root link — type, parent, origin, axis and limits. Leave
   origin and axis as `auto` and the tool finds the shaft or bore the two links share.
   The **parent** column is the one people get wrong: a leg is a chain (body → hip → thigh → shin),
   not three things all hanging off the body.

4. **Build and check.** Drag every slider in the preview. A thigh joint should carry the shin with
   it. If a joint spins about the wrong axis, replace `auto` with an explicit direction like
   `0, 1, 0` and rebuild.

5. **Download** the zip, and download the config too — that config rebuilds everything in one click
   after your next CAD revision.

---

## Command line

```bash
# Quick: every part becomes a link, all fixed to the heaviest part
python -m step2urdf robot.step -o out/

# Normal workflow: generate a config, edit it, build
python -m step2urdf robot.step --init-config robot.yaml
python -m step2urdf robot.step -c robot.yaml -o out/ --zip

# One rigid body, e.g. a gripper or a static fixture
python -m step2urdf part.step --single -o out/
```

| Option | Meaning |
|---|---|
| `--density 7850` | Default density in kg/m³ (2700 aluminium, 7850 steel, 1240 PLA, 1050 ABS) |
| `--mesh-path relative` | `../meshes` for PyBullet, MuJoCo, Isaac. `package` for ROS, RViz, Gazebo |
| `--collision mesh\|box\|none` | Collision geometry. `box` is much faster for physics |
| `--quality 2` | Finer meshes (larger files) |
| `--name my_robot` | Robot name |

`pip install -e .` also installs a `step2urdf` command.

---

## Config file

Lengths are in CAD millimetres. Limits are radians for revolute joints, metres for prismatic.

```yaml
robot_name: my_arm
density: 2700               # default kg/m^3
mesh_path: package          # or "relative"
collision: mesh             # mesh | box | none
root: base_link

links:
  - name: base_link
    parts: [Base, "M8_Screw*"]     # exact names or glob patterns
  - name: upper_arm
    parts: [UpperArm]
    density: 7850                  # per-link override
  - name: gripper
    parts: [Gripper]
    mass: 0.25                     # measured mass wins; inertia is scaled to match
    color: [0.8, 0.1, 0.1]         # overrides the CAD colour

joints:
  - name: shoulder
    type: revolute                 # fixed | revolute | continuous | prismatic
    parent: base_link
    child: upper_arm
    origin: auto                   # auto | com | [x, y, z]
    axis: auto                     # auto | [x, y, z]
    lower: -3.14
    upper: 3.14
    effort: 10                     # N*m (revolute) or N (prismatic)
    velocity: 1                    # rad/s or m/s
```

**Joint types:** `fixed` for parts that never move relative to each other, `revolute` for a servo or
motor with end stops, `continuous` for a wheel or anything that spins freely, `prismatic` for a
linear slide or actuator.

**Origin modes:** `auto` looks for cylindrical faces — shafts, pins, bores, bearing seats — on
either link whose axis passes through the other link, picks the largest, and places the origin
where the two links meet. `com` uses the child's centre of mass. Or give an explicit point in CAD
coordinates. The URDF records what `auto` picked as a comment beside each joint.

---

## What you get

```
my_robot/
  urdf/my_robot.urdf          the robot description
  meshes/<link>.stl           one binary STL per link, in metres, in the link frame
  preview.html                open in a browser: 3D view with joint sliders
  package.xml                 drop the folder into a ROS 1 or ROS 2 workspace as-is
  CMakeLists.txt
  step2urdf_config.yaml       the exact config used, for the next CAD revision
  conversion_report.txt       masses, joints and any warnings
```

Using it in ROS 2:

```bash
cp -r my_robot ~/ws/src/ && cd ~/ws && colcon build
ros2 launch urdf_tutorial display.launch.py \
  model:=$(ros2 pkg prefix my_robot)/share/my_robot/urdf/my_robot.urdf
```

---

## Example: a 12-DOF quadruped

`examples/quadruped_12dof.yaml` is a working config for a four-legged robot with 12 MG996R servos:
53 CAD parts grouped into 13 links, with 12 revolute joints — hip abduction, thigh and knee for
each leg.

```yaml
- name: leg1_abduction
  type: revolute
  parent: base_link
  child: leg1_hip
  origin: auto          # finds the hip bearing axis
  axis: auto
  lower: -0.7           # +/- 40 degrees sideways
  upper: 0.7
  effort: 1.0           # MG996R stall torque at 6V, N*m
  velocity: 5.2         # no-load speed, rad/s
```

Two things that example demonstrates:

- **Servos belong to the link they are bolted to, not the one they drive.** The hip servo sits on
  the chassis and is part of `base_link`; the two servos it carries around are part of `leg1_hip`.
- **Use real actuator numbers.** The defaults (effort 10, velocity 1) are placeholders. An MG996R
  delivers about 1.0 N·m and 5.2 rad/s at 6V. Leaving the default tells your simulator the servos
  are ten times stronger than they are, and any gait you tune will fail on the real robot.

`examples/test_arm.step` is a small two-joint arm used by the test suite, with
`examples/test_arm.yaml` as its config — a good thirty-second check that your install works.

---

## How it works

**Geometry** is read with OpenCascade, the kernel behind FreeCAD. It walks the STEP assembly tree,
so nested sub-assemblies and repeated part instances land in the right place, and part names and
colours survive.

**Mass properties** are computed from the exact solids, not the meshes. Parts sharing a link are
combined with the parallel-axis theorem. Everything is converted to SI: metres, kilograms, kg·m².

**Frames.** The root link's frame is the CAD origin. Every other link's frame sits at its joint
origin and stays aligned with the CAD axes, so joint origins are pure translations and joint axes
read like CAD directions. Meshes are re-expressed in their link frame and written in metres, so no
scale factor is needed anywhere.

---

## Limitations

- **STEP files contain no kinematics.** Joint structure always comes from you, or from `auto`.
  Always check `auto` results in the preview.
- **URDF cannot represent closed loops.** If your robot has a four-bar linkage or a push rod, break
  the loop: attach the rod to one link and drive the joint directly. The kinematics stay correct;
  the rod just looks detached when it moves. Add the loop constraint in your simulator if you need it.
- **Uniform density per link.** Use `mass:` for links containing motors, batteries or hollow prints.
  3D-printed parts with infill are considerably lighter than the solid-material figure.
- **Surface-only models** (no closed solids) fall back to bounding-box mass estimates, with a warning.
- **Collision meshes are the visual meshes** by default. Use `box`, or swap in convex hulls, for
  fast physics.
- **Link frames are CAD-axis-aligned** (rpy = 0). If your controller expects a particular convention
  such as DH parameters, handle that in the controller.

---

## Performance

Measured on synthetic assemblies of simple solids:

| Assembly | Triangles | Time | Output |
|---|---|---|---|
| 144 parts | 216 k | 3 s | 17 MB |
| 1,000 parts | 1.1 M | 16 s | 57 MB |
| 3,000 parts | 3.4 M | 50 s | 170 MB |

Real CAD with fillets and threads is slower and denser than this. For very large assemblies,
group aggressively, set `collision: box`, and lower `--quality`, or the resulting URDF will be
too heavy for a simulator to run in real time.

---

## Tests

```bash
./venv/bin/pip install yourdfpy trimesh
./venv/bin/python tests/test_roundtrip.py
./venv/bin/python tests/test_edge_cases.py
```

`test_roundtrip.py` builds a URDF, loads it with an independent parser, runs forward kinematics and
checks the result against the original CAD geometry and against analytic inertia formulas.
`test_edge_cases.py` covers unnamed multi-body STEP files, prismatic joints, and invalid
kinematic trees.

`examples/make_test_arm.py` regenerates the sample STEP assembly.

---

## Troubleshooting

**`pip install` fails on cadquery-ocp** — you are almost certainly on Python 3.14. Install 3.12,
delete the `venv` folder, and set it up again. On Windows, `py install 3.12` then
`py -3.12 -m venv venv`.

**`streamlit: command not found`** — use `venv\Scripts\python.exe -m streamlit run app.py` rather
than calling `streamlit` directly.

**PowerShell blocks `activate`** — you do not need to activate anything. Call the venv's
`python.exe` directly as shown above.

**"An Application Control policy has blocked this file"** — Windows Smart App Control is refusing
to load unsigned Python binaries. Move the project out of Downloads, run
`Get-ChildItem -Recurse | Unblock-File`, and rebuild the venv. If it persists, run the project under
WSL. Turning Smart App Control off also works, but it cannot be turned back on without reinstalling
Windows.

**The robot explodes or sinks in simulation** — check the masses in `conversion_report.txt` against
your real robot. A default density meant for aluminium applied to printed plastic is the usual cause.

**A joint rotates about the wrong axis** — replace `auto` in that joint's axis with an explicit
direction such as `0, 1, 0`. The origin detector still runs, but only considers axes parallel to
your hint.

---

## Contributing

Issues and pull requests are welcome. If a STEP file converts badly, opening an issue with the file
(or a cut-down version of it) is the most useful thing you can do.

## License

MIT — see [LICENSE](LICENSE).

## Built with

[OpenCascade](https://dev.opencascade.org/) via [OCP](https://github.com/CadQuery/OCP) for CAD
geometry, [Streamlit](https://streamlit.io/) for the web app, and
[three.js](https://threejs.org/) for the 3D preview.
