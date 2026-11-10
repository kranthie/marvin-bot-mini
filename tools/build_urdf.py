"""Build the robot's URDF from the description YAML files.

Reads description/robot.yaml (joints, ranges, servos), actuators.yaml (torque, speed, mass,
size), mass_budget.yaml (non-servo masses) and skeleton.yaml (frames, where masses sit,
collision boxes), mirrors the left side to the right, and writes
description/urdf/marvin_bot_mini.urdf. Adapted from Marvin Bot's tools/build_urdf.py.

Model:
- each link's inertia combines its structure boxes (skeleton.yaml "bodies") and the servo cases
  mounted on it, as solid boxes with the shaft along the joint axis;
- joint effort and velocity limits are the servo's stall torque and no-load speed at its rated
  voltage (the simulations derate them with the battery voltage);
- the code-CAD parts (description/cad_model.yaml, from cad/build.py, once it exists) replace
  skeleton.yaml's estimate boxes for the budget items they cover, with their computed masses
  and inertias and their meshes as visuals;
- collision geometry is the skeleton's boxes; visuals are the estimate boxes (except the
  harness and hardware, spread through their links), the CAD meshes and the servo cases.

Usage:
  python tools/build_urdf.py           # write the URDF and print the mass summary
  python tools/build_urdf.py --check   # exit 1 if the URDF is out of date (used by CI)
"""

import argparse
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parent.parent
DESC = ROOT / "description"
URDF_OUT = DESC / "urdf" / "marvin_bot_mini.urdf"
CAD_MODEL = DESC / "cad_model.yaml"  # written by cad/build.py
SIDES = ("left", "right")
MOTOR_COLOURS = {"STS3250": "0.88 0.66 0.0 1", "STS3215-C018": "0.23 0.49 0.85 1"}
BODY_COLOUR = "0.19 0.20 0.23 1"
# budget items spread through their links (a box per link for the mass and inertia), not drawn
NOT_DRAWN = {"harness", "hardware"}
# printed parts by material: white PETG, grey nylon-carbon, black TPU
CAD_COLOURS = {"petg": "0.90 0.91 0.92 1", "pa_cf": "0.42 0.44 0.47 1", "tpu": "0.08 0.08 0.09 1"}


# ---------------------------------------------------------------------------------- helpers


def mirror_name(name: str) -> str:
    return "right_" + name[len("left_") :] if name.startswith("left_") else name


def mirror_xyz(v: list[float]) -> list[float]:
    return [v[0], -v[1], v[2]]


def mirror_axis(a: list[float]) -> list[float]:
    """Pitch axes (y) stay; roll (x) and yaw (z) axes flip, so ranges keep their meaning."""
    return [-a[0], a[1], -a[2]]


def fmt(v) -> str:
    return " ".join(f"{x:.6g}" for x in v)


def mat_add(a, b):
    return [[a[i][j] + b[i][j] for j in range(3)] for i in range(3)]


def scale(a, s):
    return [[a[i][j] * s for j in range(3)] for i in range(3)]


def eye():
    return [[1.0 if i == j else 0.0 for j in range(3)] for i in range(3)]


def outer(u, v):
    return [[u[i] * v[j] for j in range(3)] for i in range(3)]


def box_inertia(m: float, size: list[float]):
    x, y, z = size
    return [
        [m * (y * y + z * z) / 12, 0, 0],
        [0, m * (x * x + z * z) / 12, 0],
        [0, 0, m * (x * x + y * y) / 12],
    ]


def combine(parts):
    """Mass, centre of mass and inertia about it of (mass, com, inertia-about-own-com) parts."""
    m = sum(p[0] for p in parts)
    com = [sum(p[0] * p[1][k] for p in parts) / m for k in range(3)]
    inertia = [[0.0] * 3 for _ in range(3)]
    for pm, pc, pi in parts:
        d = [pc[k] - com[k] for k in range(3)]
        dd = sum(x * x for x in d)
        inertia = mat_add(
            inertia, mat_add(pi, scale(mat_add(scale(eye(), dd), scale(outer(d, d), -1)), pm))
        )
    return m, com, inertia


# ------------------------------------------------------------------------------ the model


def expand(entries: list[dict]) -> list[dict]:
    """robot.yaml entries with "{side}" become a left and a right copy."""
    out = []
    for e in entries:
        sides = SIDES if any("{side}" in str(v) for v in e.values()) else (None,)
        for side in sides:

            def sub(v, side=side):
                if isinstance(v, str):
                    return v.replace("{side}", side) if side else v
                if isinstance(v, list):
                    return [sub(x) for x in v]
                return v

            out.append({k: sub(v) for k, v in e.items()})
    return out


def mirrored(items: dict | list, key_is_name: bool):
    """Add right-side copies of left-side skeleton entries."""
    if key_is_name:
        out = dict(items)
        for name, e in items.items():
            if name.startswith("left_"):
                r = dict(e)
                for k in ("parent", "child", "link"):
                    if k in r:
                        r[k] = mirror_name(r[k])
                r["xyz"] = mirror_xyz(e["xyz"])
                if "axis" in r:
                    r["axis"] = mirror_axis(e["axis"])
                out[mirror_name(name)] = r
        return out
    out = list(items)
    for e in items:
        if e["link"].startswith("left_"):
            out.append({**e, "link": mirror_name(e["link"]), "xyz": mirror_xyz(e["xyz"])})
    return out


def servo_box(axis: list[float], envelope_mm: list[float]) -> list[float]:
    """A servo case's size along x, y, z (m): its height (the second envelope figure) along the
    shaft, its length vertical (or along x for a vertical shaft) and its width across."""
    length, height, width = (v / 1000 for v in envelope_mm)
    a = [abs(round(v)) for v in axis]
    if a[2]:
        return [length, width, height]
    if a[0]:
        return [height, width, length]
    return [width, height, length]


def motor_placements() -> list[dict]:
    """Every servo case: name, model, the link carrying it, centre in that link's frame, size,
    and the joint it drives. A case sits on the parent link of its joint, centred on the joint,
    unless skeleton.yaml's motor_mounts moves it."""
    robot = yaml.safe_load((DESC / "robot.yaml").read_text())
    acts = yaml.safe_load((DESC / "actuators.yaml").read_text())
    skel = yaml.safe_load((DESC / "skeleton.yaml").read_text())
    joints = mirrored(skel["joints"], key_is_name=True)
    mounts = mirrored(skel["motor_mounts"], key_is_name=True)
    out = []
    for m in expand(robot["motors"]):
        (jn,) = m["drives"]
        j = joints[jn]
        mount = mounts.get(m["name"], {"link": j["parent"], "xyz": j["xyz"]})
        out.append(
            dict(
                name=m["name"],
                model=m["model"],
                link=mount["link"],
                xyz=mount["xyz"],
                size=servo_box(j["axis"], acts[m["model"]]["envelope_mm"]),
                joint=jn,
            )
        )
    return out


def build() -> tuple[str, dict]:
    robot = yaml.safe_load((DESC / "robot.yaml").read_text())
    acts = yaml.safe_load((DESC / "actuators.yaml").read_text())
    budget = yaml.safe_load((DESC / "mass_budget.yaml").read_text())["items"]
    skel = yaml.safe_load((DESC / "skeleton.yaml").read_text())

    ranges = {j["name"]: j["range"] for j in expand(robot["joints"])}
    joints = mirrored(skel["joints"], key_is_name=True)
    cad = yaml.safe_load(CAD_MODEL.read_text()) if CAD_MODEL.exists() else {}
    cad = {k: v for k, v in cad.items() if not budget[v["budget"]].get("add_on")}
    cad_items = {p["budget"] for p in cad.values()}
    bodies = [
        b for b in mirrored(skel["bodies"], key_is_name=False) if b["budget"] not in cad_items
    ]
    collisions = mirrored(skel["collisions"], key_is_name=False)
    errors = []

    # skeleton lengths must match robot.yaml
    dims = robot["dimensions"]
    z = {n: -j["xyz"][2] for n, j in joints.items()}
    checks = {
        "thigh": z["left_hip_roll"] + z["left_hip_pitch"] + z["left_hip_yaw"] + z["left_knee"],
        "shin": z["left_ankle_pitch"],
        "upper_arm": z["left_shoulder_roll"] + z["left_shoulder_yaw"] + z["left_elbow"],
        "shoulder_height": dims["hip_axis_height"] - z["waist_yaw"] - z["left_shoulder_pitch"],
    }
    for k, v in checks.items():
        if abs(v - dims[k]) > 1e-6:
            errors.append(f"skeleton gives {k} = {v:.3f} m, robot.yaml says {dims[k]:.3f} m")
    if set(joints) != set(ranges):
        errors.append(f"joints differ: {sorted(set(joints) ^ set(ranges))}")

    # every budget item fully placed, except the add-ons (the CAD places those, later)
    placed: dict[str, float] = {}
    for b in [*bodies, *cad.values()]:
        placed[b["budget"]] = placed.get(b["budget"], 0.0) + b["mass"]
    for k, v in budget.items():
        if v.get("add_on"):
            continue
        if abs(placed.get(k, 0.0) - v["mass"]) > 5e-4:  # half a gram: budgets are rounded
            errors.append(
                f"budget {k}: {v['mass']} kg, skeleton places {placed.get(k, 0.0):.3f} kg"
            )

    links: dict[str, list] = {"pelvis": []}
    for j in joints.values():
        links.setdefault(j["parent"], [])
        links.setdefault(j["child"], [])
    visuals: dict[str, list[str]] = {n: [] for n in links}
    for b in bodies:
        links[b["link"]].append((b["mass"], b["xyz"], box_inertia(b["mass"], b["size"])))
        if b["budget"] in NOT_DRAWN:
            continue
        visuals[b["link"]].append(
            f'<visual><origin xyz="{fmt(b["xyz"])}"/><geometry><box size="{fmt(b["size"])}"/>'
            f'</geometry><material name="body"><color rgba="{BODY_COLOUR}"/></material></visual>'
        )
    for name, c in cad.items():
        i = c["inertia"]  # ixx iyy izz ixy ixz iyz
        inertia = [[i[0], i[3], i[4]], [i[3], i[1], i[5]], [i[4], i[5], i[2]]]
        links[c["link"]].append((c["mass"], c["com"], inertia))
        visuals[c["link"]].append(
            f'<visual><geometry><mesh filename="../meshes/{name}.stl"/></geometry>'
            f'<material name="{c["material"]}"><color rgba="{CAD_COLOURS[c["material"]]}"/>'
            "</material></visual>"
        )

    # servos: the case's mass on its link; the joint's limits from the servo
    effort: dict[str, float] = {}
    velocity: dict[str, float] = {}
    for m in motor_placements():
        a = acts[m["model"]]
        links[m["link"]].append((a["mass"], m["xyz"], box_inertia(a["mass"], m["size"])))
        visuals[m["link"]].append(
            f'<visual><origin xyz="{fmt(m["xyz"])}"/><geometry><box size="{fmt(m["size"])}"/>'
            f'</geometry><material name="{m["model"]}">'
            f'<color rgba="{MOTOR_COLOURS[m["model"]]}"/></material></visual>'
        )
        effort[m["joint"]] = a["stall_torque"]
        velocity[m["joint"]] = a["no_load_speed_rpm"] * 2 * math.pi / 60

    if errors:
        for e in errors:
            print(f"ERROR: {e}")
        sys.exit(1)

    # write
    out = [f'<robot name="{robot["robot"]["name"]}">']
    summary = {}
    for name, parts in links.items():
        if not parts:
            parts = [(0.002, [0.0, 0.0, 0.0], box_inertia(0.002, [0.01, 0.01, 0.01]))]
        m, com, i = combine(parts)
        summary[name] = m
        out.append(f'  <link name="{name}">')
        out.append(
            f'    <inertial><origin xyz="{fmt(com)}"/><mass value="{m:.6g}"/>'
            f'<inertia ixx="{i[0][0]:.6g}" ixy="{i[0][1]:.6g}" ixz="{i[0][2]:.6g}" '
            f'iyy="{i[1][1]:.6g}" iyz="{i[1][2]:.6g}" izz="{i[2][2]:.6g}"/></inertial>'
        )
        out += [f"    {v}" for v in visuals[name]]
        for c in collisions:
            if c["link"] == name:
                out.append(
                    f'    <collision><origin xyz="{fmt(c["xyz"])}"/><geometry>'
                    f'<box size="{fmt(c["size"])}"/></geometry></collision>'
                )
        out.append("  </link>")
    for name, j in joints.items():
        lo, hi = map(math.radians, ranges[name])
        out += [
            f'  <joint name="{name}" type="revolute">',
            f'    <parent link="{j["parent"]}"/><child link="{j["child"]}"/>',
            f'    <origin xyz="{fmt(j["xyz"])}"/><axis xyz="{fmt(j["axis"])}"/>',
            f'    <limit lower="{lo:.6g}" upper="{hi:.6g}" effort="{effort[name]:.6g}" '
            f'velocity="{velocity[name]:.6g}"/>',
            "  </joint>",
        ]
    out.append("</robot>")
    header = (
        "<?xml version='1.0' encoding='utf-8'?>\n"
        "<!-- Generated by tools/build_urdf.py from description/*.yaml. Do not edit. -->\n"
    )
    ET.fromstring("\n".join(out))  # well-formed
    return header + "\n".join(out) + "\n", summary


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--check", action="store_true", help="fail if the URDF is out of date")
    args = ap.parse_args()
    text, summary = build()
    if args.check:
        if not URDF_OUT.exists() or URDF_OUT.read_text() != text:
            print(f"{URDF_OUT.relative_to(ROOT)} is out of date: run python tools/build_urdf.py")
            return 1
        print("URDF up to date")
        return 0
    URDF_OUT.parent.mkdir(parents=True, exist_ok=True)
    URDF_OUT.write_text(text)
    print(
        f"wrote {URDF_OUT.relative_to(ROOT)}: {len(summary)} links, {sum(summary.values()):.3f} kg"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
