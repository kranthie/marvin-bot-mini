# Portions adapted from Isaac Lab (https://github.com/isaac-sim/IsaacLab), Copyright (c) 2022-2026,
# The Isaac Lab Project Developers, BSD-3-Clause (LICENSES/BSD-3-Clause-IsaacLab.txt). Modified for
# Marvin Bot Mini; the rest Apache-2.0.
# SPDX-License-Identifier: Apache-2.0 AND BSD-3-Clause
"""Isaac Lab articulation config for Marvin Bot Mini.

Isaac Lab 3.0 API. Everything joint-related comes from the description files: joint order and
default stance from description/robot.yaml, the servos and their model from actuators.yaml.
sim/isaaclab/convert_urdf.py turns description/urdf/marvin_bot_mini.urdf into the USD loaded here.

The servos are position-controlled with no torque mode, so the actuator copies the servo's own
firmware loop (the model of sim/servo_model.py, after BAM): the policy's target angle becomes a
PWM duty cycle through the firmware's P gain, and the motor's torque follows from the supply
voltage and its back-EMF. Each robot gets its own supply voltage (the pack between empty and
full), its own spread of the motor constants, and a command delay of 5-20 ms for the inference
and the serial bus round trip. The model's constants are datasheet-derived placeholders until the
phase 2 bench fit.
"""

import math
from pathlib import Path

import isaaclab.sim as sim_utils
import torch
import yaml
from isaaclab.actuators import IdealPDActuator, IdealPDActuatorCfg
from isaaclab.assets import ArticulationCfg
from isaaclab.utils import DelayBuffer, configclass
from isaaclab_newton.sim.schemas import NewtonArticulationCfg
from isaaclab_physx.sim.schemas import PhysxArticulationCfg, PhysxRigidBodyCfg

ROOT = Path(__file__).resolve().parents[3]
DESC = ROOT / "description"
URDF_PATH = DESC / "urdf" / "marvin_bot_mini.urdf"
# USD converted from the URDF by convert_urdf.py (git-ignored); loading it needs no Isaac Sim
USD_DIR = Path(__file__).resolve().parents[1] / "usd"
USD_PATH = USD_DIR / "marvin_bot_mini" / "marvin_bot_mini" / "marvin_bot_mini.usda"

_robot = yaml.safe_load((DESC / "robot.yaml").read_text())
_acts = yaml.safe_load((DESC / "actuators.yaml").read_text())
SIDES = ("left", "right")
PACK = _robot["power"]["pack"]


def _expand(name: str) -> list[str]:
    return [name.replace("{side}", s) for s in SIDES] if "{side}" in name else [name]


# Joint order of every policy and of the robot, on every physics backend: robot.yaml's order,
# left before right.
JOINT_ORDER = tuple(n for j in _robot["joints"] for n in _expand(j["name"]))
STANCE_DEG = _robot["stance"]
STANCE_PELVIS_Z = _robot["stance_pelvis_height"]
SOLE_BELOW_FOOT_FRAME = _robot["dimensions"]["ankle_height"]

LEG_JOINTS = [
    ".*_hip_pitch",
    ".*_hip_roll",
    ".*_hip_yaw",
    ".*_knee",
    ".*_ankle_pitch",
    ".*_ankle_roll",
]
ARM_JOINTS = [".*_shoulder_pitch", ".*_shoulder_roll", ".*_shoulder_yaw", ".*_elbow"]
FEET = ".*_foot_link"

# IMU on the pelvis, as the MuJoCo model's site
IMU_BODY = "pelvis"
IMU_OFFSET = (0.0, 0.0, 0.02)


def joint_models() -> dict[str, str]:
    """The servo model driving each joint."""
    return {j: m["model"] for m in _robot["motors"] for j in _expand(m["drives"][0])}


def joint_rated() -> dict[str, float]:
    """Continuous (rated) torque per joint, N·m."""
    return {j: _acts[m]["rated_torque"] for j, m in joint_models().items()}


def joint_rule_peak() -> dict[str, float]:
    """The torque rule's peak per joint, N·m: robot.yaml's share of stall at the pack's empty
    voltage (60% at 10.5 V)."""
    rule = _robot["torque_rule"]
    v = PACK[rule["at"]]
    return {
        j: _acts[m]["stall_torque"] * v / _acts[m]["v_rated"] * rule["peak_share_of_stall"]
        for j, m in joint_models().items()
    }


class ServoActuator(IdealPDActuator):
    """A Feetech STS servo: its firmware's P loop on the position error drives the motor's PWM
    duty, and the motor's torque follows from the supply voltage and its back-EMF:

        duty = clip(firmware_kp * error_gain * (q_target - q), -max_pwm, max_pwm)
        tau  = kt * (duty * v - kt * dq) / R - friction

    The target reaches the servo a random number of physics steps late (a lag per episode, as
    Isaac Lab's DelayedPDActuator does). Per robot, at each reset: the supply voltage is drawn
    from v_range and kt, R and the firmware gain are scaled by up to +-param_spread.
    """

    def __init__(self, cfg, *args, **kwargs):
        super().__init__(cfg, *args, **kwargs)
        shape = (self._num_envs, self.num_joints)
        self._v = torch.full(shape, cfg.v_range[1], device=self._device)
        self._kt = torch.full(shape, cfg.kt, device=self._device)
        self._R = torch.full(shape, cfg.R, device=self._device)
        self._gain = torch.full(shape, cfg.firmware_kp * cfg.error_gain, device=self._device)
        self._delay = DelayBuffer(cfg.max_delay, self._num_envs, device=self._device)

    def reset(self, env_ids):
        super().reset(env_ids)
        if env_ids is None:
            env_ids = slice(None)
        n = len(range(self._num_envs)[env_ids]) if isinstance(env_ids, slice) else len(env_ids)
        cfg = self.cfg
        lags = torch.randint(
            cfg.min_delay, cfg.max_delay + 1, (n,), dtype=torch.int, device=self._device
        )
        self._delay.set_time_lag(lags, env_ids)
        self._delay.reset(env_ids)

        def spread(base):
            r = torch.rand((n, self.num_joints), device=self._device) * 2 - 1
            return base * (1 + cfg.param_spread * r)

        lo, hi = cfg.v_range
        self._v[env_ids] = lo + (hi - lo) * torch.rand((n, 1), device=self._device)
        self._kt[env_ids] = spread(cfg.kt)
        self._R[env_ids] = spread(cfg.R)
        self._gain[env_ids] = spread(cfg.firmware_kp * cfg.error_gain)

    def compute(self, control_action, joint_pos, joint_vel):
        target = self._delay.compute(control_action.joint_positions)
        duty = torch.clamp(self._gain * (target - joint_pos), -self.cfg.max_pwm, self.cfg.max_pwm)
        tau = self._kt * (duty * self._v - self._kt * joint_vel) / self._R
        tau = tau - self.cfg.friction_viscous * joint_vel
        self.computed_effort = tau
        self.applied_effort = self._clip_effort(tau, joint_vel)
        control_action.joint_efforts = self.applied_effort
        control_action.joint_positions = None
        control_action.joint_velocities = None
        return control_action


@configclass
class ServoActuatorCfg(IdealPDActuatorCfg):
    class_type: type = ServoActuator
    kt: float = 1.0  # N·m/A, effective, output side
    R: float = 1.0  # ohm, effective
    firmware_kp: float = 32.0
    error_gain: float = 0.166
    max_pwm: float = 0.97
    friction_viscous: float = 0.0  # N·m·s/rad, on top of the back-EMF
    v_range: tuple[float, float] = (PACK["v_empty"], PACK["v_full"])
    param_spread: float = 0.1  # kt, R and the firmware gain, +-10% per robot
    min_delay: int = 1  # physics steps: 5 ms
    max_delay: int = 4  # 20 ms


def _actuators() -> dict[str, ServoActuatorCfg]:
    models = joint_models()
    out = {}
    for model in sorted(set(models.values())):
        a = _acts[model]
        m = a["model"]
        joints = [j for j in JOINT_ORDER if models[j] == model]
        stall = a["stall_torque"] * PACK["v_full"] / a["v_rated"]
        speed = a["no_load_speed_rpm"] * 2 * math.pi / 60 * PACK["v_full"] / a["v_rated"]
        out[model.lower().replace("-", "_")] = ServoActuatorCfg(
            joint_names_expr=joints,
            kt=m["kt"],
            R=m["R"],
            firmware_kp=m["firmware_kp"],
            error_gain=m["error_gain"],
            max_pwm=m["max_pwm"],
            # the model itself caps torque at stall; this only guards the solver
            actuator_effort_limit=stall * 1.2,
            joint_effort_limit=stall * 1.2,
            actuator_velocity_limit=speed * 1.5,
            joint_velocity_limit=speed * 1.5,
            stiffness=0.0,
            damping=0.0,
            armature=m["armature"],
            friction=0.0,
        )
    return out


def _stance() -> dict[str, float]:
    out = {}
    for j in JOINT_ORDER:
        key = j.split("_", 1)[1] if j.startswith(SIDES) else j
        if key in STANCE_DEG:
            out[j] = math.radians(STANCE_DEG[key])
    return out


MARVIN_BOT_MINI_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=str(USD_PATH),
        activate_contact_sensors=True,
        rigid_props=PhysxRigidBodyCfg(max_depenetration_velocity=1.0),
        articulation_props=[
            PhysxArticulationCfg(
                enabled_self_collisions=True,
                solver_position_iteration_count=8,
                solver_velocity_iteration_count=4,
            ),
            NewtonArticulationCfg(self_collision_enabled=True),
        ],
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, STANCE_PELVIS_Z + 0.005),
        joint_pos=_stance(),  # every other joint at 0
        joint_vel={".*": 0.0},
    ),
    joint_ordering=JOINT_ORDER,
    soft_joint_pos_limit_factor=0.95,
    actuators=_actuators(),
)
