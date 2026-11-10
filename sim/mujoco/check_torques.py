"""Torque check in MuJoCo against robot.yaml's servo torque rule: the phase 1 order gate.

The robot is held by its simulated servos (sim/servo_model.py) on a pack at its empty voltage,
through cases that load the joints hardest short of walking:

- stand: robot.yaml's stance, held for 3 s;
- squat: from the stance down to a deep squat (knee 120°) and back up over 8 s, the hips
  chosen at each depth to keep the centre of mass over the feet with the trunk upright;
- push-up: lying face down on the hands and toes, elbows bent 60°, as when getting up from the
  front;
- sit-up: sitting on the floor leaning far back on straight arms, as when getting up from the
  back;
- crouch-rise: from a crouch with the hands on the floor (knee 130°, the pose a get-up passes
  through) up to the stance over 3 s.

Each case reports per joint the peak and RMS torque, against the rule's peak limit (60% of
stall at the empty voltage) and the servo's rated torque. These are quasi-static holds and slow
moves without a balance controller; the dynamic peaks of a trained get-up policy are checked
again when it exists. Passes if every case stays within the rule and stand and squat stay up.

Usage:
  python sim/mujoco/check_torques.py
  python sim/mujoco/check_torques.py --case squat --verbose

Needs: pip install -e ".[sim]" and the URDF (python tools/build_urdf.py).
"""

import argparse
import math
import sys
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from sim.mujoco.model import ROBOT, Robot  # noqa: E402
from sim.servo_model import joint_servos  # noqa: E402

RULE = ROBOT["torque_rule"]
V_EMPTY = ROBOT["power"]["pack"][RULE["at"]]
STANCE = ROBOT["stance"]


def foot_flat_quat(r: Robot) -> np.ndarray:
    """The base orientation that puts the left foot flat on the floor, for the current joints."""
    mujoco.mj_kinematics(r.model, r.data)
    foot = r.data.xquat[r.model.body("left_foot_link").id].copy()
    base = r.data.qpos[3:7].copy()
    inv = np.zeros(4)
    mujoco.mju_negQuat(inv, foot)
    q = np.zeros(4)
    mujoco.mju_mulQuat(q, inv, base)
    return q


def balanced_squat(r: Robot, knee: float) -> dict:
    """A squat pose at this knee angle (deg) with the feet flat and the centre of mass over the
    middle of the feet. The trunk stays upright (ankle = knee - hip) while the ankle's range
    allows; deeper, the ankle stays at its range (less 3°) and the trunk leans forward. The hip
    pitch is scanned in 0.5° steps for the centre of mass crossing the middle of the foot."""
    ankle_max = next(j["range"][1] for j in ROBOT["joints"] if j["name"].endswith("ankle_pitch"))
    ankle_max -= 3

    def com_offset(hip, ankle):
        pose = {**STANCE, "knee": knee, "hip_pitch": hip, "ankle_pitch": ankle}
        r.set_pose(pose)
        r.data.qpos[3:7] = foot_flat_quat(r)
        mujoco.mj_forward(r.model, r.data)
        foot = r.data.geom_xpos[foot_geom(r)]
        return r.com()[0] - foot[0], pose

    for upright in (True, False):
        best = None
        for hip in np.arange(0.0, 115.5, 0.5):
            ankle = knee - hip if upright else ankle_max
            if upright and not -ankle_max <= ankle <= ankle_max:
                continue
            f, pose = com_offset(hip, ankle)
            if best is None or abs(f) < best[0]:
                best = (abs(f), pose)
        if best[0] < 0.004:
            return best[1]
    raise ValueError(f"no balanced squat at knee {knee}°")


def foot_geom(r: Robot) -> int:
    body = r.model.body("left_foot_link").id
    return next(
        g
        for g in range(r.model.ngeom)
        if r.model.geom_bodyid[g] == body and r.model.geom_contype[g]
    )


def run(r: Robot, goals_at, duration: float, settle: float = 0.5):
    """Step the robot towards goals_at(t) for duration seconds; torques after settle."""
    n = int(duration / r.model.opt.timestep)
    taus = []
    for i in range(n):
        t = i * r.model.opt.timestep
        tau = r.step(goals_at(t))
        if t >= settle:
            taus.append(tau)
    return np.abs(np.array(taus))


def case_stand(r: Robot):
    g = r.set_pose(STANCE)
    r.place_on_floor()
    taus = run(r, lambda t: g, 3.0)
    return taus, r.tilt_deg() < 15


def case_squat(r: Robot, depth: float = 120.0):
    knees = np.linspace(STANCE["knee"], depth, 21)
    poses = [r.pose_vector(balanced_squat(r, k)) for k in knees]
    start = poses[0]
    r.set_pose(balanced_squat(r, knees[0]))
    r.data.qpos[3:7] = foot_flat_quat(r)
    mujoco.mj_forward(r.model, r.data)
    r.place_on_floor()
    path = np.array(poses + poses[::-1])

    def goals(t):
        if t < 0.5:
            return start
        s = min((t - 0.5) / 8.0, 1.0) * (len(path) - 1)
        i = min(int(s), len(path) - 2)
        return path[i] + (s - i) * (path[i + 1] - path[i])

    taus = run(r, goals, 9.0)
    return taus, r.tilt_deg() < 20


def quat_pitch(deg: float) -> np.ndarray:
    a = math.radians(deg) / 2
    return np.array([math.cos(a), 0.0, math.sin(a), 0.0])


def case_pushup(r: Robot):
    # face down, the body tilted onto the hands (base pitched forward past horizontal), the arms
    # reaching down to the floor: shoulder pitch 90° is the arm forward, here towards the floor
    pose = {"shoulder_pitch": 70, "elbow": 60, "shoulder_roll": 10, "ankle_pitch": 30}
    g = r.set_pose(pose, root_quat=quat_pitch(75))
    r.place_on_floor()
    taus = run(r, lambda t: g, 2.0)
    return taus, True


def case_situp(r: Robot):
    # sitting on the floor leaning far back (trunk about 45° from vertical) on the hands, arms
    # 30° behind the trunk: the hardest of the sitting poses scanned for the arms
    pose = {"hip_pitch": 30, "shoulder_pitch": -30, "shoulder_roll": 10}
    g = r.set_pose(pose, root_quat=quat_pitch(-60))
    r.place_on_floor()
    taus = run(r, lambda t: g, 2.0)
    return taus, True


def case_crouch_rise(r: Robot):
    crouch = {**balanced_squat(r, 130.0), "shoulder_pitch": 40, "elbow": 10}
    start = r.set_pose(crouch)
    r.data.qpos[3:7] = foot_flat_quat(r)
    mujoco.mj_forward(r.model, r.data)
    r.place_on_floor()
    end = r.pose_vector(STANCE)

    def goals(t):
        s = min(max(t - 0.5, 0.0) / 3.0, 1.0)
        return start + s * (end - start)

    taus = run(r, goals, 4.0)
    return taus, r.tilt_deg() < 20


CASES = {
    "stand": case_stand,
    "squat": case_squat,
    "push-up": case_pushup,
    "sit-up": case_situp,
    "crouch-rise": case_crouch_rise,
}
MUST_STAY_UP = {"stand", "squat"}


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--case", choices=sorted(CASES), action="append")
    ap.add_argument("--verbose", action="store_true", help="every joint, not only the worst")
    args = ap.parse_args()
    servos = joint_servos()
    errors = []
    for name in args.case or CASES:
        r = Robot(v_supply=V_EMPTY)
        taus, upright = CASES[name](r)
        peak, rms = taus.max(axis=0), np.sqrt((taus**2).mean(axis=0))
        limit = np.array(
            [
                servos[n][2]["stall_torque"]
                * V_EMPTY
                / servos[n][2]["v_rated"]
                * RULE["peak_share_of_stall"]
                for n in r.names
            ]
        )
        rated = np.array([servos[n][2]["rated_torque"] for n in r.names])
        use = peak / limit
        print(f"\n{name}: {'upright' if upright else 'FELL'}, tilt {r.tilt_deg():.0f}°")
        print(f"  {'joint':22} {'peak':>5} {'limit':>5} {'use':>5} {'rms':>5} {'rated':>5}")
        order = np.argsort(-use)
        shown = order if args.verbose else order[:6]
        for i in shown:
            flag = "  <--" if peak[i] > limit[i] or rms[i] > rated[i] else ""
            print(
                f"  {r.names[i]:22} {peak[i]:5.2f} {limit[i]:5.2f} {use[i]:5.0%} "
                f"{rms[i]:5.2f} {rated[i]:5.2f}{flag}"
            )
        for i in range(len(r.names)):
            if peak[i] > limit[i]:
                errors.append(f"{name}: {r.names[i]} peaks {peak[i]:.2f} > {limit[i]:.2f} N·m")
            if RULE["rms_within_rated"] and rms[i] > rated[i]:
                errors.append(f"{name}: {r.names[i]} RMS {rms[i]:.2f} > rated {rated[i]:.2f} N·m")
        if name in MUST_STAY_UP and not upright:
            errors.append(f"{name}: the robot fell")
    print(f"\nservos at {V_EMPTY} V (the pack's {RULE['at']}), mass {Robot().mass:.2f} kg")
    for e in errors:
        print(f"ERROR: {e}")
    if errors:
        return 1
    print("OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
