"""Give the retargeted clips the wrists' turn and the pincers' grip.

No keypoint fixes the mini's wrist yaw or pincer (the retargeter matches positions, and a wrist's
position does not change as it turns), so retarget_mini.py holds them at rest and this sets them,
as add_head_motion.py sets the neck:

- wrist yaw: the person's hand twist against the forearm, about the forearm's axis, from Kimodo's
  joint rotations, less its average in the idle clip (where the hand hangs, palm in: the pincer's
  rest, its jaws closing front to back); clipped to the joint's range;
- grip: Kimodo's hands do not open or close (every finger keeps its rest pose), so each clip that
  works an object has a schedule in GRIP: open on the way to it, closed on it while held, open to
  let go, worked out from the hand's path. Every other clip keeps the pincer closed (0°).

Run it after add_head_motion.py, before the conversion to ProtoMotions' format.

Usage (any environment with numpy):
  python sim/motion/add_hand_motion.py <Kimodo npz dir> <retargeted dir> <retargeting URDF>
"""

import argparse
import re
from pathlib import Path

import numpy as np
import yaml

DESC = Path(__file__).resolve().parents[2] / "description"
# SOMA joint indices in Kimodo's 77-joint output (SOMASkeleton77.bone_order_names)
HIPS, HEAD = 0, 6
FOREARM = {"left": 13, "right": 41}
HAND = {"left": 14, "right": 42}
FPS = 30.0
OPEN, HOLD = 45.0, 10.0  # degrees: the pincer open, and closed on a cup or handle
RAMP = 0.3  # s to open or close


def twist(npz: dict, side: str) -> np.ndarray:
    """The hand's twist against the forearm about the forearm's axis (rad, right-handed about
    the axis pointing down the forearm), per frame."""
    r, p = npz["global_rot_mats"], npz["posed_joints"]
    f, h = FOREARM[side], HAND[side]
    axis = np.einsum("tji,tj->ti", r[:, f], p[:, h] - p[:, f]).mean(0)
    axis /= np.linalg.norm(axis)
    rel = np.einsum("tji,tjk->tik", r[:, f], r[:, h])
    w = np.sqrt(np.clip(1 + rel[:, 0, 0] + rel[:, 1, 1] + rel[:, 2, 2], 1e-9, None)) / 2
    v = np.stack(
        [rel[:, 2, 1] - rel[:, 1, 2], rel[:, 0, 2] - rel[:, 2, 0], rel[:, 1, 0] - rel[:, 0, 1]], 1
    ) / (4 * w[:, None])
    return 2 * np.arctan2(v @ axis, w)


def smooth(x: np.ndarray, n: int = 5) -> np.ndarray:
    k = np.ones(n) / n
    return np.convolve(np.pad(x, (n // 2, n // 2), mode="edge"), k, mode="valid")


def ramped(on: np.ndarray, high: float, low: float) -> np.ndarray:
    """A grip angle per frame: `high` where on is False, `low` where True, eased over RAMP."""
    target = np.where(on, low, high).astype(float)
    n = max(1, int(RAMP * FPS))
    return smooth(target, n if n % 2 else n + 1)


def hand_path(npz: dict, side: str = "right") -> tuple[np.ndarray, np.ndarray]:
    """The hand's height above the floor and its reach ahead of the hips (m), per frame."""
    p = npz["posed_joints"]
    h = p[:, HAND[side]]
    floor = p[:, :, 1].min()
    return h[:, 1] - floor, h[:, 2] - p[:, HIPS, 2]


def grip_pick_up(npz):
    """Open on the way down, closed from the lowest point on: it picks something up."""
    height, _ = hand_path(npz)
    t_low = int(np.argmin(height))
    return ramped(np.arange(len(height)) >= t_low, OPEN, HOLD)


def grip_held(npz):
    """Closed on a cup throughout."""
    return np.full(len(npz["posed_joints"]), HOLD)


def grip_reach(npz, let_go: bool = True):
    """Open while reaching, closed while the hand is out at its object (within 80% of its
    furthest reach), open again as it lets go."""
    _, reach = hand_path(npz)
    lo, hi = np.percentile(reach, 5), reach.max()
    at = reach > lo + 0.8 * (hi - lo)
    if not let_go:
        at = np.maximum.accumulate(at)
    return ramped(at, OPEN, HOLD)


def grip_overhead(npz):
    """Closed on the bulb while the hand is above the head."""
    height, _ = hand_path(npz)
    head = npz["posed_joints"][:, HEAD, 1] - npz["posed_joints"][:, :, 1].min()
    return ramped(height > head, OPEN, HOLD)


def grip_hand_over(npz):
    """Holding the object until the hand is furthest out, then open: it lets go of it."""
    _, reach = hand_path(npz)
    t_out = int(np.argmax(reach))
    return ramped(np.arange(len(reach)) < t_out, OPEN, HOLD)


GRIP = {
    "pick_up": grip_pick_up,
    "pour": grip_held,
    "drink": grip_held,
    "turn_knob": grip_reach,
    "screw_bulb": grip_overhead,
    "hand_over": grip_hand_over,
}


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("npz_dir", type=Path)
    ap.add_argument("retargeted", type=Path)
    ap.add_argument("urdf", type=Path)
    a = ap.parse_args()
    text = a.urdf.read_text()
    joints = re.findall(r'<joint name="([^"]+)" type="([^"]+)"', text)
    order = [n for n, t in joints if t != "fixed"]
    robot = yaml.safe_load((DESC / "robot.yaml").read_text())
    ranges = {
        j["name"].replace("{side}", s): np.radians(j["range"])
        for j in robot["joints"]
        for s in ("left", "right")
    }
    idle = dict(np.load(a.npz_dir / "idle.npz"))
    neutral = {s: float(np.mean(twist(idle, s))) for s in ("left", "right")}
    for path in sorted(a.retargeted.glob("*_retargeted.npz")):
        clip = re.sub(r"^(soma_)?(proto_)?|_keypoints_retargeted\.npz$", "", path.name)
        src = a.npz_dir / f"{clip}.npz"
        if not src.is_file():
            print(f"{clip}: no source, wrists and pincers left at rest")
            continue
        s = dict(np.load(src))
        d = dict(np.load(path))
        q = d["joint_angles"]
        t = np.arange(len(q)) / float(d["fps"])
        ts = np.arange(len(s["posed_joints"])) / FPS
        out = []
        for side, sign in (("left", 1.0), ("right", -1.0)):
            # the left joint turns about the forearm's downward axis, the right (mirrored) about
            # its upward one
            yaw = sign * smooth(twist(s, side) - neutral[side])
            name = f"{side}_wrist_yaw"
            lo, hi = ranges[name]
            q[:, order.index(name)] = np.clip(np.interp(t, ts, yaw), lo, hi)
            out.append(f"{side[0]} {np.degrees(yaw.min()):+4.0f}..{np.degrees(yaw.max()):+4.0f}°")
        grip = GRIP.get(clip)
        g = np.radians(grip(s)) if grip else np.zeros(len(ts))
        q[:, order.index("right_gripper")] = np.interp(t, ts, g)
        q[:, order.index("left_gripper")] = 0.0
        d["joint_angles"] = q
        np.savez(path, **d)
        print(f"{clip:14s} wrist yaw {', '.join(out)}; grip {np.degrees(g.max()):3.0f}° max")


if __name__ == "__main__":
    main()
