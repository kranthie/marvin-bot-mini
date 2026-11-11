"""Clearance check: move every joint through its range and report parts that collide.

Each joint is taken to both ends of its range and half of each end (the others at zero), plus a
few combined poses (a deep squat, arms forward and in, a reach across the chest). In each pose
every part and servo case is compared with those on other links it can reach: within a limb,
a limb against the body it hangs from, and the pelvis against the upper body (as Marvin Bot's
cad/covers.py check). A pair fails when their overlap is more than MIN_CM3: mating faces (a
horn plate on its horn, a bearing on its pin's boss) touch without overlapping. Every part must
also be one piece, and no shell may cut into the frame it covers.

Usage:
  python -m cad.check            # frame and shells
  python -m cad.check --frame    # the frame only
"""

from __future__ import annotations

import argparse
import sys

import numpy as np

from . import servo as sv
from .layout import JOINTS, RANGES, ROBOT, SERVOS, to_link
from .lib import Part

MIN_CM3 = 0.05


def _rot(axis, q: float) -> np.ndarray:
    a = np.asarray(axis, float)
    a = a / np.linalg.norm(a)
    k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(q) * k + (1 - np.cos(q)) * k @ k


def link_poses(q: dict) -> dict[str, np.ndarray]:
    """World 4x4 of every link (left side and centre) for joint angles q (degrees)."""
    out = {"pelvis": np.eye(4)}
    pending = dict(JOINTS)
    while pending:
        for name, j in list(pending.items()):
            if j["parent"] not in out:
                continue
            T = np.eye(4)
            T[:3, 3] = j["xyz"]
            R = np.eye(4)
            R[:3, :3] = _rot(j["axis"], np.radians(q.get(name, 0.0)))
            out[j["child"]] = out[j["parent"]] @ T @ R
            del pending[name]
    return out


def group(link: str) -> str:
    if link.startswith("left_"):
        rest = link[5:]
        return (
            "arm" if rest.startswith(("shoulder", "upper_arm", "forearm", "palm", "jaw")) else "leg"
        )
    return "pelvis" if link == "pelvis" else "upper"


def checked(a: str, b: str) -> bool:
    ga, gb = group(a), group(b)
    if ga == gb:
        return True
    pair = {ga, gb}
    return pair in ({"pelvis", "upper"}, {"leg", "pelvis"}, {"arm", "upper"})


def adjacent_ok(a: str, b: str) -> bool:
    """Links joined by a joint: their parts meet at the joint by design (horn and pin), so
    only overlaps beyond that count; they are still checked."""
    return True


COUPLED = [
    {k: (v.replace("{side}", "left") if isinstance(v, str) else v) for k, v in c.items()}
    for c in ROBOT.get("coupled_limits", [])
]


def lowest(joint: str, q: dict) -> float:
    """A joint's lowest angle in pose q: its range, raised by any coupled limit."""
    lo = RANGES[joint][0]
    for c in COUPLED:
        if c["joint"] == joint:
            pts = np.array(c["min"], float)
            lo = max(lo, float(np.interp(q.get(c["by"], 0.0), pts[:, 0], pts[:, 1])))
    return lo


def poses() -> list[tuple[str, dict]]:
    out = [("zero", {})]
    for name, (lo, hi) in RANGES.items():
        if name not in JOINTS:
            continue
        for q in (lo, lo / 2, hi / 2, hi):
            if q and q >= lowest(name, {}):
                out.append((f"{name} {q:+.0f}", {name: q}))
    for c in COUPLED:
        pts = c["min"]
        mids = [[(a[0] + b[0]) / 2, (a[1] + b[1]) / 2] for a, b in zip(pts, pts[1:], strict=False)]
        for by, q in pts + mids:
            out.append(
                (f"{c['joint']} {q:+.0f} at {c['by']} {by:+.0f}", {c["by"]: by, c["joint"]: q})
            )
    out += [
        ("squat", {"left_hip_pitch": 100, "left_knee": 120, "left_ankle_pitch": -30}),
        ("arm forward, elbow bent", {"left_shoulder_pitch": 90, "left_elbow": 120}),
        (
            "arm across the chest",
            {
                "left_shoulder_pitch": 60,
                "left_shoulder_roll": -15,
                "left_shoulder_yaw": 60,
                "left_elbow": 100,
            },
        ),
        ("arm up", {"left_shoulder_pitch": 180}),
        ("arm out, elbow bent", {"left_shoulder_roll": 90, "left_elbow": 130}),
        ("leg out and twisted", {"left_hip_roll": 45, "left_hip_yaw": 45}),
        ("wrist and grip", {"left_elbow": 90, "left_wrist_yaw": 150, "left_gripper": 60}),
        ("head down and turned", {"neck_yaw": 80, "neck_pitch": 45}),
        ("head up and turned", {"neck_yaw": -80, "neck_pitch": -20}),
    ]
    return out


def solids(parts: list[Part]) -> list[tuple[str, str, object]]:
    """(name, link, solid in link frame) for the parts and every servo case."""
    out = [(p.name, p.link, p.solid) for p in parts]
    for s in SERVOS.values():
        out.append((f"{s.name} servo", s.link, to_link(sv.place(s, sv.case()), s.link)))
    return out


def own_link(parts: list[Part]) -> list[str]:
    """A shell must not cut into anything on its own link (the frame it covers, or a servo):
    only its mounts' hooks and bosses meet the frame, and those are part of the shell's or the
    frame's solid already, so any overlap left is a shell through a part."""
    items = solids(parts)
    errors = []
    for p in parts:
        if p.process != "cover":
            continue
        for name, link, m in items:
            if (
                link != p.link
                or name == p.name
                or name.endswith("_cover")
                or "cover_" in name
                or name == "head_visor"
            ):
                continue
            v = (p.solid ^ m).volume() / 1000.0
            if v > MIN_CM3:
                errors.append(f"{'own link':32} {p.name} x {name}: {v:.2f} cm³")
    return errors


def check(parts: list[Part], verbose: bool = False) -> list[str]:
    items = solids(parts)
    pairs = [
        (i, k)
        for i in range(len(items))
        for k in range(i + 1, len(items))
        if items[i][1] != items[k][1] and checked(items[i][1], items[k][1])
    ]
    errors = []
    seen = set()
    for label, q in poses():
        T = link_poses(q)
        placed = [m.transform(T[link][:3, :4].tolist()) for _, link, m in items]
        boxes = [np.array(p.bounding_box()) for p in placed]
        for i, k in pairs:
            a, b = boxes[i], boxes[k]
            if np.any(a[3:] < b[:3]) or np.any(b[3:] < a[:3]):
                continue
            v = (placed[i] ^ placed[k]).volume() / 1000.0
            if v > MIN_CM3:
                key = (items[i][0], items[k][0])
                msg = f"{label:32} {items[i][0]} x {items[k][0]}: {v:.2f} cm³"
                if verbose or key not in seen:
                    errors.append(msg)
                seen.add(key)
    return errors


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--frame", action="store_true", help="the frame only, no shells")
    ap.add_argument("-v", "--verbose", action="store_true", help="every pose a pair collides in")
    args = ap.parse_args()
    from . import frame

    parts = frame.parts()
    if not args.frame:
        from . import shells

        framed, covers = shells.build(parts)
        parts = framed + covers
    # every part one piece: a boss or hook over a window, or a cut that splits a part, shows here
    errors = [
        f"{'pieces':32} {p.name}: {n} separate pieces"
        for p in parts
        if (n := len(p.solid.decompose())) > 1
    ]
    errors += (own_link(parts) if not args.frame else []) + check(parts, args.verbose)
    for e in errors:
        print(e)
    print(f"{len(poses())} poses, {len(errors)} collisions")
    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(main())
