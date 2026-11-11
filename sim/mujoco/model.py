"""The robot in MuJoCo with its servos modelled, shared by the MuJoCo checks.

Loads description/urdf/marvin_bot_mini.urdf (run tools/build_urdf.py first), adds a floor, a
free base and a torque actuator per joint, and drives each actuator with the servo model
(sim/servo_model.py) at a given supply voltage: the policy or check sets goal angles, as on the
real robot, and the servo firmware's loop is simulated at the physics rate.

Usage (as a library):
    from sim.mujoco.model import Robot
    r = Robot(v_supply=10.5)
    r.set_pose({"knee": 20, ...}); r.place_on_floor()
    r.step(goals)  # goal angles (rad) in actuator order
"""

import math
import sys
from pathlib import Path

import mujoco
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from sim.servo_model import joint_servos, servo_torque  # noqa: E402

DESC = ROOT / "description"
URDF = DESC / "urdf" / "marvin_bot_mini.urdf"
COVERS_URDF = DESC / "urdf" / "marvin_bot_mini_covers.urdf"  # with the add-on shells
ROBOT = yaml.safe_load((DESC / "robot.yaml").read_text())


def side_key(name: str) -> str:
    """left_knee -> knee; waist_yaw -> waist_yaw."""
    return name.split("_", 1)[1] if name.startswith(("left_", "right_")) else name


class Robot:
    def __init__(
        self,
        v_supply: float | None = None,
        timestep: float = 0.001,
        visual: bool = False,
        covers: bool = False,
    ):
        text = (COVERS_URDF if covers else URDF).read_text()
        if visual:  # keep the URDF's visual geoms (MuJoCo drops them by default), for renders
            text = text.replace(
                f'<robot name="{ROBOT["robot"]["name"]}">',
                f'<robot name="{ROBOT["robot"]["name"]}"><mujoco>'
                f'<compiler discardvisual="false" meshdir="{URDF.parent}"/></mujoco>',
                1,
            )
        spec = mujoco.MjSpec.from_string(text)
        spec.option.timestep = timestep
        spec.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
        spec.compiler.autolimits = True
        world = spec.worldbody
        world.add_geom(
            name="floor",
            type=mujoco.mjtGeom.mjGEOM_PLANE,
            size=[3, 3, 0.05],
            friction=[1, 0.005, 1e-4],
        )
        world.add_light(pos=[0, 0, 3], dir=[0, 0, -1], type=mujoco.mjtLightType.mjLIGHT_DIRECTIONAL)
        if visual:  # a checked floor (10 cm squares) and a plain sky, for scale in renders
            spec.add_texture(
                name="grid",
                type=mujoco.mjtTexture.mjTEXTURE_2D,
                builtin=mujoco.mjtBuiltin.mjBUILTIN_CHECKER,
                width=512,
                height=512,
                rgb1=[0.82, 0.83, 0.85],
                rgb2=[0.74, 0.75, 0.78],
            )
            mat = spec.add_material(name="grid", texrepeat=[15, 15])
            mat.textures[mujoco.mjtTextureRole.mjTEXROLE_RGB] = "grid"
            spec.geom("floor").material = "grid"
            spec.add_texture(
                name="sky",
                type=mujoco.mjtTexture.mjTEXTURE_SKYBOX,
                builtin=mujoco.mjtBuiltin.mjBUILTIN_GRADIENT,
                width=256,
                height=256,
                rgb1=[0.93, 0.95, 0.98],
                rgb2=[0.70, 0.76, 0.84],
            )
            spec.visual.headlight.ambient = [0.45, 0.45, 0.45]
        pelvis = spec.body("pelvis")
        pelvis.add_freejoint(name="root")
        pelvis.add_site(name="imu", pos=[0, 0, 0.02])
        for g in spec.geoms:
            if g.contype and g.name != "floor":
                g.group = 3  # collision boxes hidden in renders
        spec.visual.global_.offwidth = 1280
        spec.visual.global_.offheight = 960
        servos = joint_servos()
        for j in spec.joints:
            if j.name == "root":
                continue
            _, p, a = servos[j.name]
            j.armature = a["model"].get("armature", 0.0)
            act = spec.add_actuator(name=j.name, target=j.name, trntype=mujoco.mjtTrn.mjTRN_JOINT)
            act.gainprm[0] = 1.0  # a torque motor; the servo model computes the torque
            act.forcelimited = mujoco.mjtLimited.mjLIMITED_FALSE
        self.model = spec.compile()
        self.data = mujoco.MjData(self.model)
        self.v_supply = v_supply or ROBOT["power"]["pack"]["v_nominal"]
        m = self.model
        self.names = [m.actuator(i).name for i in range(m.nu)]
        jids = [m.actuator(i).trnid[0] for i in range(m.nu)]
        self.qadr = np.array([m.jnt_qposadr[j] for j in jids])
        self.dadr = np.array([m.jnt_dofadr[j] for j in jids])
        self.params = [servos[n][1] for n in self.names]
        self.models = [servos[n][0] for n in self.names]
        self.lo = np.array([m.jnt_range[j][0] for j in jids])
        self.hi = np.array([m.jnt_range[j][1] for j in jids])
        self.mass = float(sum(m.body_mass))

    # ------------------------------------------------------------------ poses
    def pose_vector(self, pose_deg: dict) -> np.ndarray:
        """Goal angles (rad, actuator order) from a side-less pose in degrees; joints not
        given are at 0. Keys may also be full joint names, which win over side-less ones."""
        q = np.array(
            [math.radians(pose_deg.get(n, pose_deg.get(side_key(n), 0.0))) for n in self.names]
        )
        return np.clip(q, self.lo, self.hi)

    def set_pose(self, pose_deg: dict, root_quat=(1, 0, 0, 0)) -> np.ndarray:
        q = self.pose_vector(pose_deg)
        d = self.data
        d.qpos[:] = 0
        d.qpos[3:7] = root_quat
        d.qpos[self.qadr] = q
        d.qvel[:] = 0
        mujoco.mj_forward(self.model, d)
        return q

    def lowest_point(self) -> float:
        """The lowest z of any collision box corner."""
        m, d = self.model, self.data
        low = np.inf
        for g in range(m.ngeom):
            if m.geom_type[g] != mujoco.mjtGeom.mjGEOM_BOX or m.geom_contype[g] == 0:
                continue
            if m.geom_bodyid[g] == 0:
                continue
            h = m.geom_size[g]
            corners = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
            pts = d.geom_xpos[g] + (corners * h) @ d.geom_xmat[g].reshape(3, 3).T
            low = min(low, pts[:, 2].min())
        return low

    def place_on_floor(self, clearance: float = 0.0005) -> None:
        """Lift or lower the free base so the lowest collision box just clears the floor."""
        self.data.qpos[2] += clearance - self.lowest_point()
        mujoco.mj_forward(self.model, self.data)

    # ------------------------------------------------------------------ stepping
    def torques(self, goals: np.ndarray) -> np.ndarray:
        q = self.data.qpos[self.qadr]
        dq = self.data.qvel[self.dadr]
        return np.array(
            [
                servo_torque(p, g, qi, dqi, self.v_supply)
                for p, g, qi, dqi in zip(self.params, goals, q, dq, strict=True)
            ]
        )

    def step(self, goals: np.ndarray) -> np.ndarray:
        """One physics step towards the goal angles; returns the servo torques applied."""
        tau = self.torques(np.clip(goals, self.lo, self.hi))
        self.data.ctrl[:] = tau
        mujoco.mj_step(self.model, self.data)
        return tau

    def tilt_deg(self) -> float:
        """Angle of the pelvis's z axis from vertical."""
        z = self.data.xmat[self.model.body("pelvis").id].reshape(3, 3)[:, 2]
        return math.degrees(math.acos(max(-1.0, min(1.0, z[2]))))

    def com(self) -> np.ndarray:
        return self.data.subtree_com[self.model.body("pelvis").id].copy()
