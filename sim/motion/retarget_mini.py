"""Retarget human keypoints to Marvin Bot Mini with ProtoMotions' PyRoki retargeter.

Marvin Bot's retargeter (retarget_marvin_bot.py) with the mini's sizes, links and held joints.

Runs ProtoMotions' G1 retargeter (pyroki/batch_retarget_to_g1_from_keypoints.py) unchanged
except for what is robot-specific:
- which robot frame each human keypoint is matched to (LINKS), on the retargeting URDF that
  setup_protomotions.py writes (it adds the toe, hand and torso frames);
- the scale from the human source to the mini in place of the retargeter's G1 scale
  (scale_keypoints), with the feet moved apart to the mini's hip spacing (WIDEN);
- the rest pose the solver starts from and pulls towards, the standing pose instead of the
  middle of each joint's range;
- self-collision of the arms against the trunk, legs and other arm (COLLISION_EDITS), which
  upstream leaves off;
- the joints held near their rest angle (the neck, the wrists and the pincers; add_head_motion.py
  and add_hand_motion.py set them afterwards).

It checks that PyRoki's joint order is the MJCF's, since the converter that follows reads the
retargeted joint vector in MJCF order.

Keypoints converted from other motion capture into the same convention (convert_100style.py)
take --converted.

Usage (in the PyRoki environment), then continue with ProtoMotions' converter:
  python sim/motion/retarget_mini.py --protomotions /path/to/ProtoMotions \\
      --keypoints-folder-path keypoints/ --output-dir retargeted/ --source-type soma --no-visualize
"""

import ast
import re
import sys
from pathlib import Path

import numpy as onp

# human keypoint -> the mini's frame, in the retargeter's order
LINKS = [
    ("pelvis", "pelvis"),  # the pelvis origin is on the hip axes, where SOMA's pelvis point is
    ("left_hip", "left_hip_pitch_link"),
    ("right_hip", "right_hip_pitch_link"),
    ("left_knee", "left_shin_link"),
    ("right_knee", "right_shin_link"),
    ("left_ankle", "left_foot_link"),
    ("right_ankle", "right_foot_link"),
    ("left_foot", "left_toe_link"),
    ("right_foot", "right_toe_link"),
    ("left_shoulder", "left_shoulder_pitch_link"),
    ("right_shoulder", "right_shoulder_pitch_link"),
    ("left_elbow", "left_forearm_link"),
    ("right_elbow", "right_forearm_link"),
    ("left_wrist", "left_hand_link"),
    ("right_wrist", "right_hand_link"),
]
# The retargeter scales its SOMA source for G1 (legs and upper body about the pelvis keypoint,
# by axis); scale_keypoints undoes that and scales for the mini instead
G1_SCALE = {"lower": (0.8, 0.8, 0.807), "upper": (0.8, 0.8, 0.7)}
# The robot's sizes, from description/ (the scales below follow them; read with re, since the
# retargeter's environment has no YAML parser)
DESC = Path(__file__).resolve().parents[2] / "description"
_ROBOT = (DESC / "robot.yaml").read_text()
_DIMS = {
    k: float(re.search(rf"^  {k}: ([0-9.]+)", _ROBOT, re.M).group(1))
    for k in ("thigh", "shin", "shoulder_height", "hip_axis_height", "upper_arm", "forearm")
}
_SKEL = (DESC / "skeleton.yaml").read_text()


def _xyz(joint: str) -> list[float]:
    m = re.search(rf"^  {joint}:.*?xyz: \[([^\]]+)\]", _SKEL, re.M)
    return [float(v) for v in m.group(1).split(",")]


_ROLL_DROP = -_xyz("left_shoulder_roll")[2]
# shoulder joints' distance apart (pitch joint plus the roll offset, both sides)
_SHOULDER_SPAN = 2 * (_xyz("left_shoulder_pitch")[1] + _xyz("left_shoulder_roll")[1])
_HIP_SPAN = 2 * _xyz("left_hip_roll")[1]
# Legs: uniform, the robot's straight hip-to-ankle length over the source's standing one (knee
# 5-7 deg bent; 0.853 m). The knee is sensitive to this near straight: a target leg 1.5 cm short
# of the robot's holds the knee at 26 deg
LEG = (_DIMS["thigh"] + _DIMS["shin"]) / 0.853
# Shoulders about the hips: width the robot's shoulder span over the source's (0.51 m, from
# Marvin Bot's 0.62 for its 0.316 m); height 0.75 for shoulders 0.365 m above the hip axis
# (0.34-0.39 m across the clips), in proportion for the robot's. Arms: uniform about each
# shoulder, the robot's straight-arm reach to the hand's end over the source's to the wrist
# (0.56 m)
_SHOULDER_ABOVE_HIP = _DIMS["shoulder_height"] - _DIMS["hip_axis_height"]
_W = _SHOULDER_SPAN / 0.51
TORSO = (_W, _W, 0.75 * _SHOULDER_ABOVE_HIP / 0.365)
# and forward (m, along the hips' heading): the source's shoulders are 2.5-4 cm behind its hips
# and the mini's are over its own; Marvin Bot's 6.5 cm, scaled by the leg
SHOULDER_FORWARD = 0.065 * 0.24 / 0.64
_REACH = _ROLL_DROP + _DIMS["upper_arm"] + _DIMS["forearm"]
ARM = (_REACH + 0.005) / 0.56  # 0.44 for the mini's 0.24 m reach
# The source's pelvis keypoint is 8 cm above its hips; the robot's pelvis origin is on the hip
# axis, so its target is put midway between the hip targets (as on Marvin Bot).
# Scaled down, the human's feet end up about 5 cm apart, and the mini's feet are 5.5 cm wide and
# its hips 9 cm apart: the knees (keypoints 3, 4) and feet (5-8) move out by these (m) each,
# half and all of the hip spacing the scaling leaves short (left keypoints are odd)
_SHORT = max(0.0, (_HIP_SPAN - 0.18 * LEG) / 2)
WIDEN = {3: _SHORT / 2, 4: _SHORT / 2, 5: _SHORT, 6: _SHORT, 7: _SHORT, 8: _SHORT}
# held near rest: the neck (add_head_motion.py sets it afterwards) and the wrists and pincers,
# which no keypoint fixes (add_hand_motion.py sets them)
HOLD = ["neck_yaw", "neck_pitch"] + [
    f"{s}_{j}" for s in ("left", "right") for j in ("wrist_yaw", "gripper")
]


# Self-collision. Upstream builds no collision model and leaves the cost out ("turned off for
# now"), and the retargeted arms passed through the body: hands 4-10 cm into the thighs in
# 18-92% of walking frames, the palms 9 cm through each other in the clap. These edits to the
# upstream script (each must match once) turn it on, for the arms against the trunk, the legs
# and the other arm only: links that already touch in the standing pose are left out
SELF_COLLISION = 10.0
COLLISION_EDITS = [
    (
        "    # robot_coll = pk.collision.RobotCollision.from_urdf(urdf)\n    robot_coll = None\n",
        "    robot_coll = pk.collision.RobotCollision.from_urdf(\n"
        "        urdf, user_ignore_pairs=marvin_collision_ignore(urdf)\n    )\n",
    ),
    (
        "        self_collision=0.0,  # turned off for now\n",
        f"        self_collision={SELF_COLLISION},\n",
    ),
    (
        "        # pk.costs.self_collision_cost(\n"
        "        #     jax.tree.map(lambda x: x[None], robot),\n"
        "        #     jax.tree.map(lambda x: x[None], robot_coll),\n"
        "        #     var_joints,\n"
        "        #     margin=0.01,\n"
        '        #     weight=weights["self_collision"],\n'
        "        # ),\n",
        "        pk.costs.self_collision_cost(\n"
        "            jax.tree.map(lambda x: x[None], robot),\n"
        "            jax.tree.map(lambda x: x[None], robot_coll),\n"
        "            var_joints,\n"
        "            margin=0.02,\n"
        '            weight=weights["self_collision"],\n'
        "        ),\n",
    ),
]
ARM_LINKS = ("upper_arm_link", "forearm_link")
BODY_LINKS = ("pelvis", "torso_body", "thigh_link", "shin_link")


def marvin_collision_ignore(urdf) -> tuple[tuple[str, str], ...]:
    """Every pair of links except an arm's lower links against the trunk, the legs and the other
    arm. All links are listed: the URDF passed in is loaded without its collision meshes."""
    links = list(urdf.link_map)

    def arm_side(n):
        return n.split("_")[0] if n.endswith(ARM_LINKS) else None

    def checked(a, b):
        for x, y in ((a, b), (b, a)):
            side = arm_side(x)
            if side and (y.endswith(BODY_LINKS) or (arm_side(y) and arm_side(y) != side)):
                return True
        return False

    return tuple((a, b) for i, a in enumerate(links) for b in links[i + 1 :] if not checked(a, b))


def load_retargeter(pm: Path):
    """ProtoMotions' G1 retargeting script with COLLISION_EDITS applied, as a module."""
    import types  # noqa: PLC0415

    path = pm / "pyroki" / "batch_retarget_to_g1_from_keypoints.py"
    src = path.read_text()
    for old, new in COLLISION_EDITS:
        if src.count(old) != 1:
            sys.exit(f"{path} has changed upstream; update COLLISION_EDITS:\n{old}")
        src = src.replace(old, new)
    module = types.ModuleType("batch_retarget_to_g1_from_keypoints")
    module.__file__ = str(path)
    module.marvin_collision_ignore = marvin_collision_ignore
    sys.modules[module.__name__] = module
    exec(compile(src, str(path), "exec"), module.__dict__)
    return module


def scale_keypoints(kp: onp.ndarray) -> onp.ndarray:
    """The retargeter's SOMA keypoints (T, 18, 3; pelvis, legs 1-8, shoulders, elbows, wrists
    9-14 with left first, hand and torso aids 15-17), rescaled from G1's size to the mini's."""
    root = kp[:, :1]
    src = kp.copy()  # the unscaled source
    src[:, 0] = root[:, 0] / G1_SCALE["lower"]
    src[:, 1:9] = src[:, :1] + (kp[:, 1:9] - root) / G1_SCALE["lower"]
    src[:, 9:] = src[:, :1] + (kp[:, 9:] - root) / G1_SCALE["upper"]
    new = src * LEG
    hips_src, hips = src[:, 1:3].mean(1, keepdims=True), new[:, 1:3].mean(1, keepdims=True)
    new[:, 0] = hips[:, 0]
    side = new[:, 1, :2] - new[:, 2, :2]  # along the hip axis, to the left
    side /= onp.linalg.norm(side, axis=-1, keepdims=True)
    forward = onp.stack([side[:, 1], -side[:, 0], onp.zeros(len(side))], -1)[:, None]
    new[:, 9:11] = hips + (src[:, 9:11] - hips_src) * TORSO + SHOULDER_FORWARD * forward
    new[:, 17] = hips[:, 0] + (src[:, 17] - hips_src[:, 0]) * TORSO
    for arm, shoulder in ((11, 9), (13, 9), (15, 9), (12, 10), (14, 10), (16, 10)):
        new[:, arm] = new[:, shoulder] + (src[:, arm] - src[:, shoulder]) * ARM
    # move the knees and feet outwards along the hip axis (left is +)
    for k, w in WIDEN.items():
        new[:, k, :2] += onp.sign(k % 2 - 0.5) * w * side
    return new


def main() -> None:
    argv = sys.argv[1:]
    i = argv.index("--protomotions")
    pm = Path(argv.pop(i + 1))
    argv.pop(i)
    # --converted: keypoints from other motion capture, already in the SOMA keypoint convention
    # with the pelvis keypoint on the hips (not extracted from a ProtoMotions SOMA motion)
    converted = "--converted" in argv
    if converted:
        argv.remove("--converted")
    sys.path.insert(0, str(pm / "pyroki"))
    r = load_retargeter(pm)
    import pyroki as pk  # noqa: PLC0415
    import yourdfpy  # noqa: PLC0415

    urdf = pm / "protomotions/data/assets/urdf/for_retargeting/marvin_bot_mini.urdf"
    mjcf = pm / "protomotions/data/assets/mjcf/marvin_bot_mini.xml"
    robot = pk.Robot.from_urdf(yourdfpy.URDF.load(urdf))
    mj_order = [n for n in re.findall(r'<joint name="([^"]+)"', mjcf.read_text()) if n != "root"]
    if list(robot.joints.actuated_names) != mj_order:
        sys.exit(f"PyRoki joint order {robot.joints.actuated_names} is not the MJCF's {mj_order}")

    # PyRoki's default pose, which the solver starts from and its rest cost pulls every joint
    # towards, is the middle of each joint's range: on Marvin Bot a 65 deg knee and 45 deg hip
    # pitch. Near a straight leg that small pull outweighs a centimetre of keypoint error, and
    # it held the knees at 23 deg or more through a walk. Use the standing pose instead
    config = (pm / "protomotions/robot_configs/marvin_bot_mini.py").read_text()
    stance = ast.literal_eval(re.search(r"DEFAULT_JOINT_POS = (\{.*\})", config).group(1))
    rest = onp.array([stance.get(n, 0.0) for n in mj_order])
    from_urdf = pk.Robot.from_urdf
    pk.Robot.from_urdf = lambda urdf, **kw: from_urdf(urdf, default_joint_cfg=rest, **kw)

    def indices():
        names = [h for h, _ in LINKS]
        return names, r.jnp.array([r.G1_LINK_NAMES.index(link) for _, link in LINKS])

    load = r.load_motion_data

    def load_motion_data(motion_path, source_type, *args, **kwargs):
        out = list(load(motion_path, source_type, *args, **kwargs))
        if source_type == "soma":
            out[0] = scale_keypoints(out[0])
        return tuple(out)

    if converted:
        r.soma_pelvis_keypoint_on_hips = lambda positions, orientations: positions
    r.get_humanoid_retarget_indices = indices
    r.load_motion_data = load_motion_data
    r.JOINTS_TO_MOVE_LESS = {"soma": HOLD, "smpl": HOLD}
    sys.argv = [sys.argv[0], "--urdf-path", str(urdf), *argv]
    r.main()


if __name__ == "__main__":
    main()
