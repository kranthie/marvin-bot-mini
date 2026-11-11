"""Render the robot in MuJoCo: a still of the stance and animations of the torque check's cases.

Writes docs/media/robot.png (the stance from four sides, the bare frame and with the shells) and an
animated WebP per moving case of sim/mujoco/check_torques.py (docs/media/<case>.webp), played by the
simulated servos at the pack's empty voltage, as the check runs them.

Usage:
  MUJOCO_GL=egl python tools/render_robot.py              # still and every animation
  MUJOCO_GL=egl python tools/render_robot.py --case squat  # one animation

Needs: pip install -e ".[sim]" pillow; MUJOCO_GL=egl without a display.
"""

import argparse
import sys
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from sim.mujoco import check_torques  # noqa: E402
from sim.mujoco.model import Robot  # noqa: E402

MEDIA = ROOT / "docs" / "media"
W, H = 360, 360
FPS = 15


def camera(azimuth: float, elevation: float = -12, distance: float = 0.95, z: float = 0.28):
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.azimuth, cam.elevation, cam.distance = azimuth, elevation, distance
    cam.lookat[:] = [0, 0, z]
    return cam


def frame(r: Robot, renderer: mujoco.Renderer, cam: mujoco.MjvCamera, follow=True) -> np.ndarray:
    if follow:
        cam.lookat[:2] = r.data.qpos[:2]
    renderer.update_scene(r.data, cam)
    return renderer.render()


def still() -> Path:
    """The stance from four sides, the bare frame above and with the clip-on shells below."""
    rows = []
    for covers in (False, True):
        r = Robot(visual=True, covers=covers)
        r.set_pose(check_torques.STANCE)
        r.place_on_floor()
        with mujoco.Renderer(r.model, H, W) as rd:
            rows.append(
                np.concatenate([frame(r, rd, camera(az)) for az in (150, 90, 180, 30)], axis=1)
            )
    out = MEDIA / "robot.png"
    Image.fromarray(np.concatenate(rows, axis=0)).save(out)
    return out


def animate(case: str, azimuth: float = 140) -> Path:
    r = Robot(v_supply=check_torques.V_EMPTY, visual=True)
    frames = []
    every = int(round(1 / (FPS * r.model.opt.timestep)))
    rd = mujoco.Renderer(r.model, H, W)
    cam = camera(azimuth, z=0.2 if case in ("push-up", "sit-up") else 0.27)
    plain_run = check_torques.run

    def recording_run(robot, goals_at, duration, settle=0.5):
        n = int(duration / robot.model.opt.timestep)
        taus = []
        for i in range(n):
            t = i * robot.model.opt.timestep
            tau = robot.step(goals_at(t))
            if i % every == 0:
                frames.append(Image.fromarray(frame(robot, rd, cam)))
            if t >= settle:
                taus.append(tau)
        return np.abs(np.array(taus))

    check_torques.run = recording_run
    try:
        check_torques.CASES[case](r)
    finally:
        check_torques.run = plain_run
        rd.close()
    out = MEDIA / f"{case}.webp"
    frames[0].save(
        out, save_all=True, append_images=frames[1:], duration=1000 // FPS, loop=0, quality=70
    )
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--case", choices=sorted(check_torques.CASES), action="append")
    args = ap.parse_args()
    MEDIA.mkdir(parents=True, exist_ok=True)
    if not args.case:
        print(f"wrote {still().relative_to(ROOT)}")
    for case in args.case or [c for c in check_torques.CASES if c != "stand"]:
        print(f"wrote {animate(case).relative_to(ROOT)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
