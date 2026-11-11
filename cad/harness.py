"""The servo buses' wiring: the daisy chains, each cable's length and the joints it crosses.

Each Feetech bus is a daisy chain of three-wire cables (ground, 12 V, data) with 5264-type
three-pin plugs, each servo carrying two sockets at the far end of its case (the cradles'
connector notches, cad/servo.py). A leg's chain starts at its bus adapter on the pelvis's front;
the upper body's three chains (each arm, the neck) start at the power board high in the torso's
back, which also feeds the waist servo and carries 12 V down through the waist to the leg
adapters.

A cable's length is its run between the two connectors (the straight distance at the zero pose,
times ROUTE for going round the frame), plus a service loop for every joint it crosses: the
loop's arc at LOOP_R from the axis over half the joint's range, plus LOOP_SLACK. The loops lie
in the gaps the shells leave at the joints and are tied either side of the joint through the
frame's cable-tie slot pairs (cad/frame.py). Lengths are rounded up to the next STOCK length.

Usage:
  python -m cad.harness          # print the table
"""

from __future__ import annotations

import math
import sys

import numpy as np

from .layout import CASE_D, CASE_X, JOINTS, RANGES, SERVOS

ROUTE = 1.35
LOOP_R, LOOP_SLACK = 15.0, 25.0  # mm
STOCK = (100, 150, 200, 300, 400, 600, 900)  # mm, the common lengths of 5264 three-pin leads

LEG_ADAPTER = np.array([28.0, 0.0, 12.0])  # pelvis front (skeleton.yaml: electronics)
POWER_BOARD = np.array([-24.0, 0.0, 160.0])  # torso back, high (world, zero pose)

CHAINS = {
    "left leg": (
        LEG_ADAPTER,
        [
            "left_hip_roll",
            "left_hip_pitch",
            "left_hip_yaw",
            "left_knee",
            "left_ankle_pitch",
            "left_ankle_roll",
        ],
    ),
    "left arm": (
        POWER_BOARD,
        [
            "left_shoulder_pitch",
            "left_shoulder_roll",
            "left_shoulder_yaw",
            "left_elbow",
            "left_wrist_yaw",
            "left_gripper",
        ],
    ),
    "neck": (POWER_BOARD, ["neck_yaw", "neck_pitch"]),
    "waist": (POWER_BOARD, ["waist_yaw"]),
}


def connector(j: str) -> np.ndarray:
    """Where a servo's bus sockets are: the far end of its case, near the floor."""
    return SERVOS[j].at([CASE_X[1] + 2.0, 0.0, -CASE_D + 5.0])


def link_of_point(name) -> str:
    return (
        "pelvis"
        if name is LEG_ADAPTER
        else ("torso_link" if name is POWER_BOARD else SERVOS[name].link)
    )


def path(a: str, b: str) -> list[str]:
    """The joints between two links (through their common ancestor)."""
    parent = {j["child"]: (n, j["parent"]) for n, j in JOINTS.items()}

    def up(link):
        out = []
        while link in parent:
            n, link = parent[link]
            out.append((n, link))
        return out

    ua, ub = up(a), up(b)
    la = [a] + [p for _, p in ua]
    lb = [b] + [p for _, p in ub]
    common = next(x for x in la if x in lb)
    joints = [n for n, p in ua[: la.index(common)]] + [n for n, p in ub[: lb.index(common)]]
    return joints


def loop_mm(j: str) -> float:
    lo, hi = RANGES[j]
    return LOOP_R * math.radians(hi - lo) / 2 + LOOP_SLACK


def table() -> list[dict]:
    rows = []
    for chain, (start, servos) in CHAINS.items():
        prev_p, prev_link, prev_name = (
            start,
            link_of_point(start),
            "adapter" if start is LEG_ADAPTER else "power board",
        )
        for s in servos:
            p = connector(s)
            crossed = path(prev_link, SERVOS[s].link)
            run = float(np.linalg.norm(p - prev_p)) * ROUTE
            loops = sum(loop_mm(j) for j in crossed)
            need = run + loops
            rows.append(
                dict(
                    chain=chain,
                    frm=prev_name,
                    to=s,
                    crosses=crossed,
                    mm=need,
                    stock=next((x for x in STOCK if x >= need), None),
                )
            )
            prev_p, prev_link, prev_name = p, SERVOS[s].link, s
    # 12 V and ground for the legs, from the power board through the waist to the pelvis
    run = float(np.linalg.norm(LEG_ADAPTER - POWER_BOARD)) * ROUTE + loop_mm("waist_yaw")
    rows.append(
        dict(
            chain="leg power",
            frm="power board",
            to="leg adapters",
            crosses=["waist_yaw"],
            mm=run,
            stock=None,
        )
    )
    return rows


def counts(rows) -> dict:
    """Leads by stock length, both sides (the left chains doubled for the right)."""
    out: dict = {}
    for r in rows:
        if r["stock"] is None:
            continue
        n = 2 if r["chain"].startswith("left") else 1
        out[r["stock"]] = out.get(r["stock"], 0) + n
    return dict(sorted(out.items()))


def main() -> int:
    rows = table()
    print(f"{'chain':10} {'from':20} {'to':20} {'mm':>5} {'lead':>5}  joints crossed")
    for r in rows:
        stock = f"{r['stock']}" if r["stock"] else "wire"
        crossed = ", ".join(r["crosses"]) or "-"
        print(f"{r['chain']:10} {r['frm']:20} {r['to']:20} {r['mm']:5.0f} {stock:>5}  {crossed}")
    print("leads (both sides):", ", ".join(f"{n} x {k} mm" for k, n in counts(rows).items()))
    return 0


if __name__ == "__main__":
    sys.exit(main())
