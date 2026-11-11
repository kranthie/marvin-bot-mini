"""Where everything sits: link origins and servo frames at the zero pose, from description/.

All parts are drawn in the world frame at the zero pose (the pelvis frame, mm; left side and
centre) and then moved into their link's frame. Link frames carry no rotation at the zero pose
(skeleton.yaml), so that move is a translation.

A servo's frame: origin on its shaft axis at the case's top face; z along the output (the horn
side), x along the case's length (robot.yaml's length: the shaft sits SHAFT_OFF from the case's
centre that way), y = z x x. In it the case spans x CASE_X, y +-CASE_W/2, z -CASE_D..0, and the
spline and horn rise to HORN_Z, the face a horn plate bolts to.
"""

from __future__ import annotations

import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "tools"))
import build_urdf  # noqa: E402

DESC = ROOT / "description"
SKEL = yaml.safe_load((DESC / "skeleton.yaml").read_text())
ROBOT = yaml.safe_load((DESC / "robot.yaml").read_text())
ACTS = yaml.safe_load((DESC / "actuators.yaml").read_text())

# the case, from actuators.yaml's shape (both servo models share it)
_sh = ACTS["STS3250"]["shape"]
CASE_L, CASE_W = _sh["box"]
CASE_D = -_sh["axial"][0]
HORN_Z = _sh["axial"][1]
SHAFT_OFF = _sh["shaft_offset"]
CASE_X = (SHAFT_OFF - CASE_L / 2, SHAFT_OFF + CASE_L / 2)


def _origins() -> dict[str, np.ndarray]:
    joints = SKEL["joints"]
    out = {"pelvis": np.zeros(3)}
    pending = dict(joints)
    while pending:
        for k, v in list(pending.items()):
            if v["parent"] in out:
                out[v["child"]] = out[v["parent"]] + np.array(v["xyz"]) * 1000
                del pending[k]
    return out


ORIGIN = _origins()
JOINTS = {k: dict(v, xyz=np.array(v["xyz"]) * 1000) for k, v in SKEL["joints"].items()}
RANGES = {
    j["name"].replace("{side}", "left"): j["range"]
    for j in ROBOT["joints"]
    if "right" not in j["name"]
}


@dataclass
class Servo:
    name: str  # the joint it drives
    model: str
    link: str  # the link carrying the case
    M: np.ndarray  # 4x4, servo frame -> world (mm)

    def at(self, p) -> np.ndarray:
        """A point in the servo frame, in world coordinates."""
        return (self.M @ np.r_[np.asarray(p, float), 1.0])[:3]

    @property
    def axis_w(self) -> np.ndarray:
        return self.M[:3, 2]


def _servos() -> dict[str, Servo]:
    out = {}
    for m in build_urdf.motor_placements():
        if m["name"].startswith("right_"):
            continue
        z = np.array(m["output"], float)
        x = np.array(m["length"], float)
        y = np.cross(z, x)
        M = np.eye(4)
        M[:3, 0], M[:3, 1], M[:3, 2] = x, y, z
        M[:3, 3] = ORIGIN[m["link"]] + np.array(m["top"]) * 1000
        out[m["joint"]] = Servo(m["joint"], m["model"], m["link"], M)
    return out


SERVOS = _servos()


def to_link(m, link: str):
    """A solid drawn in world coordinates (zero pose), in its link's frame."""
    return m.translate(list(-ORIGIN[link]))
