"""
Command line interface.

  step2urdf model.step -o out/                    # scaffold: every part a link, all fixed
  step2urdf model.step --init-config arm.yaml     # write an editable config and stop
  step2urdf model.step -c arm.yaml -o out/        # build with your links & joints
  step2urdf model.step --single -o out/           # one rigid link
"""
import argparse
import logging
import os
import sys

import yaml

from .builder import Config, build, default_config, summary, write_package, zip_package, _plain
from .cad import load_step


def main(argv=None):
    ap = argparse.ArgumentParser(prog="step2urdf", description="Convert a STEP CAD file into a URDF robot package.")
    ap.add_argument("step", help="input .step / .stp file")
    ap.add_argument("-o", "--out", default="urdf_out", help="output directory (default: urdf_out)")
    ap.add_argument("-c", "--config", help="YAML/JSON config describing links and joints")
    ap.add_argument("-n", "--name", help="robot name (default: file name)")
    ap.add_argument("--init-config", metavar="FILE", help="write a starter config listing every part, then exit")
    ap.add_argument("--single", action="store_true", help="merge all parts into one rigid link")
    ap.add_argument("--density", type=float, help="default density kg/m^3 (default 2700, aluminium)")
    ap.add_argument("--mesh-path", choices=["package", "relative"], help="mesh URIs: package:// (ROS) or ../meshes (PyBullet, MuJoCo)")
    ap.add_argument("--collision", choices=["mesh", "box", "none"], help="collision geometry")
    ap.add_argument("--quality", type=float, help="mesh fineness multiplier (default 1.0)")
    ap.add_argument("--zip", action="store_true", help="also write a .zip of the package")
    ap.add_argument("-q", "--quiet", action="store_true")
    a = ap.parse_args(argv)

    logging.basicConfig(level=logging.ERROR if a.quiet else logging.WARNING, format="warning: %(message)s")
    if not os.path.isfile(a.step):
        ap.error(f"file not found: {a.step}")

    name = a.name or os.path.splitext(os.path.basename(a.step))[0]
    print(f"Reading {a.step} ...", file=sys.stderr)
    parts = load_step(a.step)
    print(f"Found {len(parts)} part(s):", file=sys.stderr)
    for p in parts:
        print(f"  {p.name:<30} volume {p.volume/1000:10.2f} cm^3", file=sys.stderr)

    if a.config:
        with open(a.config) as fh:
            cfg = Config.from_dict(yaml.safe_load(fh))
        if a.name:
            cfg.robot_name = a.name
    else:
        cfg = default_config(parts, name, "single" if a.single else "per_part")
    for attr, val in (("density", a.density), ("mesh_path", a.mesh_path),
                      ("collision", a.collision), ("mesh_quality", a.quality)):
        if val is not None:
            setattr(cfg, attr, val)

    if a.init_config:
        with open(a.init_config, "w") as fh:
            fh.write("# step2urdf config. Lengths in CAD units (mm); limits in rad (revolute) or m (prismatic).\n"
                     "# Group parts into links (globs allowed, e.g. 'M8_Screw*'), then connect links with joints.\n"
                     "# origin: auto  -> find the shaft/bore axis between parent and child\n"
                     "# origin: com   -> child's centre of mass;  origin: [x, y, z] -> explicit point\n")
            yaml.safe_dump(_plain(cfg.to_dict()), fh, sort_keys=False)
        print(f"Wrote config template to {a.init_config}. Edit it, then run with -c {a.init_config}", file=sys.stderr)
        return 0

    robot = build(parts, cfg)
    urdf = write_package(robot, a.out)
    print(summary(robot))
    for w in robot.warnings:
        print("warning:", w, file=sys.stderr)
    if a.zip:
        zp = os.path.join(a.out, f"{cfg.robot_name}.zip")
        with open(zp, "wb") as fh:
            fh.write(zip_package(robot))
        print(f"Zip: {zp}")
    print(f"URDF: {urdf}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
