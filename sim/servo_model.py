"""The servo model shared by the MuJoCo checks and Isaac Lab training.

A Feetech STS servo takes a goal position; its firmware turns the position error into a PWM
duty cycle, and the motor turns the resulting voltage into torque against its back-EMF. After
BAM (Rhoban, Apache-2.0, https://github.com/Rhoban/bam, arXiv:2410.08650):

    duty   = clip(firmware_kp * error_gain * (q_target - q), -max_pwm, max_pwm)
    torque = kt * (duty * v_supply - kt * dq) / R  -  friction

kt and R are effective output-side values (description/actuators.yaml "model"). The friction
here is Coulomb plus viscous; BAM's Stribeck and load-dependent terms come with the phase 2
fit. The functions use only arithmetic and the array's own clip, so they work on floats, numpy
arrays and torch tensors alike.

Usage (as a library):
    from sim.servo_model import ServoParams, servo_torque
    p = ServoParams.from_actuator(acts["STS3250"])
    tau = servo_torque(p, q_target, q, dq, v_supply=11.1)
"""

from dataclasses import dataclass
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
DESC = ROOT / "description"


@dataclass(frozen=True)
class ServoParams:
    kt: float  # N·m/A = V·s/rad, output side
    R: float  # ohm
    firmware_kp: float
    error_gain: float
    max_pwm: float
    friction_coulomb: float = 0.0  # N·m
    friction_viscous: float = 0.0  # N·m·s/rad

    @classmethod
    def from_actuator(cls, a: dict) -> "ServoParams":
        m = a["model"]
        return cls(
            kt=m["kt"],
            R=m["R"],
            firmware_kp=m["firmware_kp"],
            error_gain=m["error_gain"],
            max_pwm=m["max_pwm"],
            friction_coulomb=m.get("friction_coulomb", 0.0),
            friction_viscous=m.get("friction_viscous", 0.0),
        )

    @property
    def stiffness_per_volt(self) -> float:
        """Small-error stiffness (N·m/rad) per volt of supply, before the duty saturates."""
        return self.kt * self.firmware_kp * self.error_gain / self.R


def _clip(x, lo, hi):
    if hasattr(x, "clip"):
        return x.clip(lo, hi)
    return min(max(x, lo), hi)


def _sign(x, eps=1e-3):
    """A smooth sign, so friction does not chatter at rest."""
    return x / (abs(x) + eps)


def servo_torque(p: ServoParams, q_target, q, dq, v_supply: float):
    """Output torque (N·m) of the servo for a goal angle, joint angle and speed (rad, rad/s)."""
    duty = _clip(p.firmware_kp * p.error_gain * (q_target - q), -p.max_pwm, p.max_pwm)
    tau = p.kt * (duty * v_supply - p.kt * dq) / p.R
    return tau - p.friction_coulomb * _sign(dq) - p.friction_viscous * dq


def stall_torque(p: ServoParams, v_supply: float) -> float:
    """Torque at full duty and no speed: the servo's stall torque at this supply."""
    return p.kt * p.max_pwm * v_supply / p.R


def joint_servos() -> dict[str, tuple[str, ServoParams, dict]]:
    """Per joint (robot.yaml names, sides expanded): the servo model name, its parameters and its
    actuators.yaml entry."""
    robot = yaml.safe_load((DESC / "robot.yaml").read_text())
    acts = yaml.safe_load((DESC / "actuators.yaml").read_text())
    out = {}
    for m in robot["motors"]:
        for side in ("left", "right") if "{side}" in m["name"] else (None,):
            (j,) = (d.replace("{side}", side) if side else d for d in m["drives"])
            a = acts[m["model"]]
            out[j] = (m["model"], ServoParams.from_actuator(a), a)
    return out
