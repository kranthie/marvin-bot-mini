# Isaac Lab

Marvin Bot Mini's Isaac Lab 3.0 package: the articulation config with the servo actuator (`marvin_bot_mini/robot_cfg.py`), the walking tasks, and scripts to convert the URDF, check the robot and export policies. It is Marvin Bot's package scaled to the mini, with the DC motor model replaced by the servo model. Isaac Lab runs on an RTX desktop GPU, not on the Jetson.

| File | What it does |
| --- | --- |
| `marvin_bot_mini/robot_cfg.py` | `MARVIN_BOT_MINI_CFG`: USD path, `ServoActuator` (the servo model of `sim/servo_model.py` with a 5-20 ms command delay and per-robot supply voltage and motor-constant spread), joint order and default stance from `robot.yaml`, IMU mount |
| `convert_urdf.py` | Converts `description/urdf/marvin_bot_mini.urdf` to `usd/` (git-ignored) with Isaac Sim's URDF importer |
| `check_robot.py` | Checks mass, joint order and limits, kinematics against MuJoCo, and that the servos hold the default stance |
| `marvin_bot_mini/tasks/` | `MarvinBotMini-Velocity-Flat` and `MarvinBotMini-Velocity-Rough` (rough not trained yet) |
| `export_policy.py` | Exports a checkpoint to `policy.onnx` plus `policy.json` (joint order, default pose, action scale, observation layout) |
| `../mujoco/check_policy.py` | The MuJoCo cross-check of an exported policy with the same servo model: walk, turn, stop, fast walk or steps; `--render` saves an animation |

## The servo actuator

The policy outputs target angles at 50 Hz. Each servo turns its position error into a PWM duty
through its firmware's P gain, and the motor's torque follows from the supply voltage and its
back-EMF (BAM's model structure: `description/actuators.yaml`). In training, each robot draws its
pack voltage between empty (10.5 V) and full (12.6 V), spreads kt, R and the firmware gain by
±10%, and takes its targets 5-20 ms late. The constants are datasheet-derived until the phase 2
bench fit replaces them. It runs in Isaac Lab's Python actuator path (`use_newton_actuators = False`).

## Setup

As for Marvin Bot: Isaac Lab 3.0 (Isaac Sim 6.2, Python 3.12) installed with uv outside this repo,
commit `961760bcb91e3af48e7d116292e57e8ecaefc31e` of `release/3.0.0`, on one RTX PRO 6000
Blackwell (96 GB).

```bash
git clone https://github.com/isaac-sim/IsaacLab.git && cd IsaacLab
git checkout 961760bcb91e3af48e7d116292e57e8ecaefc31e
uv sync --extra isaacsim --extra importers
uv pip install --no-deps -e <path to this repo>/sim/isaaclab
export OMNI_KIT_ACCEPT_EULA=YES              # only if you have read and accept NVIDIA's Isaac Sim licence
```

Isaac Sim is NVIDIA software under its own licence, not open source. `uv sync` removes packages it
doesn't manage, so reinstall this package after any sync and use `uv run --no-sync` otherwise.

## Convert and check

The URDF refers to the part meshes, which are generated: first, in this repo's environment,
`pip install -e ".[cad]" && python -m cad.build`.

```bash
uv run --no-sync python <repo>/sim/isaaclab/convert_urdf.py      # after any URDF change
uv run --no-sync python <repo>/sim/isaaclab/check_robot.py       # Newton
uv run --no-sync python <repo>/sim/isaaclab/check_robot.py --backend physx
```

Results (2026-11-09, the 2.92 kg model):

| Check | Newton (MuJoCo-Warp) | PhysX |
| --- | --- | --- |
| Total mass | 2.924 kg, as the URDF | 2.924 kg |
| Joint order, ranges, effort and velocity guards | match | match |
| Link positions vs MuJoCo FK, 8 random poses | worst 0.10 mm | worst 2.78 mm |
| Default stance, 5 s, on the servo actuators | holds, pelvis 0.286 m, tilt 1.0°, knees 0.34 N·m (13% of the rule) | holds, 0.286 m, 0.9°, knees 0.33 N·m |

## Walking tasks

Marvin Bot's walking tasks (the decoupled whole-body-control split: the policy drives the 12 leg
joints, the waist and arm targets jump to random poses every 1-3 s), scaled to the mini:

- **Commands:** forward −0.15 m/s up to a curriculum limit that starts at 0.25 m/s and grows to
  0.5 m/s, sideways ±0.15 m/s, yaw ±1.0 rad/s; 20% of robots stand still. (Marvin Bot's speeds
  scaled by about the root of the leg length.)
- **Gait shaping:** 3.5 cm swing clearance, single-support air time 0.25 s, step width 8 ± 1.5 cm,
  fall below 0.15 m pelvis height or past 1 rad of tilt.
- **Randomization:** pushes up to 0.3 m/s, torso mass ±15% and CoM ±1.5 cm, friction 0.4-1.0,
  plus the servo spread above.
- **Torque penalty:** G1's scale raised by about the square of the torque ratio (servo torques up
  to 4.9 N·m against G1's 120 N·m).
- **Rough ground (not trained yet):** steps of 1.5-6 cm with 12 cm treads, boxes to 4 cm, rough
  ground to 2 cm, slopes to 0.25 rad, a 0.6 × 0.3 m terrain map at 5 cm.

```bash
uv run --no-sync isaaclab train --rl_library rsl_rl --task MarvinBotMini-Velocity-Flat --num_envs 4096 --max_iterations 3000
uv run --no-sync python <repo>/sim/isaaclab/export_policy.py --checkpoint logs/rsl_rl/marvin_bot_mini_flat/<run>/model_2999.pt
# then from this repo, in its own venv:
python sim/mujoco/check_policy.py <IsaacLab>/logs/rsl_rl/marvin_bot_mini_flat/<run>/exported
```

A run takes about 0.47 s an iteration on the GPU above (about 25 minutes for 3000), with rsl_rl's
default seed. Trained policies are not committed.

## Results

| Policy | Training | MuJoCo cross-check (`sim/mujoco/check_policy.py`) |
| --- | --- | --- |
| flat v1 | 3000 iterations, Marvin Bot's reward weights | learned to fall within 0.25 s: the heat penalty outweighed the tracking rewards (decisions 2026-11-09) |
| flat v2 | 3000 iterations from scratch, rescaled rewards | walks, turns and stops; 4.5 steps/s, 10 cm steps, 1.5 cm lift; knees up to 171% of the torque rule's peak, 109% of rated RMS at 0.3 m/s |
| flat v3 | 2000 iterations from v2, torque-rule penalty, clearance weight 2 | passes at 11.1 V / 10 ms, 10.5 V / 20 ms, full noise, 0.3 m/s on 10.5 V; 2.3 steps/s, 19 cm steps, 3.4 cm lift, knees 60% of rated RMS at 0.2 m/s and 81% at 0.3 m/s; above the torque rule under 1% of the time, at most 20 ms. Fails on 12.6 V / 5 ms: 0.26 m/s for 0.20, a knee above the rule for 56 ms |

The checkpoint of flat v3 resumed flat v2 (`--checkpoint` with v2's `model_2999.pt`); the task
config at each stage is in the git history.
