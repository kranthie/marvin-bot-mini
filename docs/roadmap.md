# Roadmap

Current phase: **1, robot in simulation** (started 2026-11-09). Phases and pass checks are defined in [plan.md](plan.md).

| Phase | Status |
|---|---|
| 0 Foundations | done |
| 1 Robot in simulation | in progress |
| 2 Servo bench | not started |
| 3 Print and build | not started |
| 4 Walking | not started |
| 5 Skills | not started |

## Phase 0

- [x] Repository, licences, acknowledgements, plan, requirements, safety rules
- [x] v0.1 joint layout, servo assignment, buses and power (`description/robot.yaml`)
- [x] Servo specifications and the provisional servo model (`description/actuators.yaml`)
- [x] Mass budget (2.85 kg bare, 3.05 kg with the shells) and draft BOM (`bom/bom.xlsx`)
- [x] Description check: joints, servos, bus timing, torque budget, mass, BOM counts (`tools/check_description.py`)
- [x] Licence check of the open-source references (`ACKNOWLEDGMENTS.md`)
- [x] v0.1 layout agreed by the owner
- [x] GitHub repository created (private until the first public release)

## Phase 1

- [x] Skeleton (`description/skeleton.yaml`) and URDF generated from the description files (`tools/build_urdf.py`, adapted from Marvin Bot's)
- [x] Servo model after BAM, shared by MuJoCo and (to come) Isaac Lab (`sim/servo_model.py`), unit-tested against the datasheet stall torque and no-load speed
- [x] The robot in MuJoCo driven by the servo model (`sim/mujoco/model.py`)
- [x] Torque check, the order gate (`sim/mujoco/check_torques.py`): stand, squat, push-up, sit-up, crouch-rise within the rule; shoulder pitch moved to the STS3250 and arms lengthened to 0.24 m (decisions 2026-11-09)
- [x] Code-CAD first pass of the legs, pelvis and torso (`cad/parts.py`, `cad/build.py`): meshes and masses in the URDF; 2.92 kg bare
- [x] Renders and animations of the torque check (`tools/render_robot.py`)
- [x] Priced order list (`bom/bom.xlsx`, listings of 2026-11-10): $1,370 to buy, servos from WowRobo (decisions 2026-11-10)
- [ ] Order placed
- [x] Isaac Lab package (`sim/isaaclab/`): `ServoActuator` (the servo model with a 5-20 ms command delay and per-robot pack voltage and motor-constant spread), Marvin Bot's walking tasks scaled to the mini; robot check passes on Newton and PhysX
- [x] Mirror rule checked (`sim/mujoco/check_symmetry.py`)
- [x] Flat walking: v3 passes the MuJoCo cross-check with the servo model (`sim/mujoco/check_policy.py`): walk 0.2 m/s, turn, stop; 0.3 m/s on an empty pack; knees at 60-81% of rated RMS (decisions 2026-11-10)
- [ ] Flat v3 on a full pack with a short delay: walks 0.26 m/s for 0.20 and holds a knee above the torque rule for 56 ms (randomize the pack voltage in the command response, or train with it observed)
- [x] Motion pipeline (`sim/motion/`): one tracker plays all 25 skills upright in Newton and in the MuJoCo check on the servo model, the hardest-working joint at 46-79% of rated RMS ([skills](skills.md), decisions 2026-11-10)
- [ ] Squat and pick-up to the reference's depth (the tracker stops 12 cm high, knees at stall)
- [ ] Rough-ground task (configured, not trained), get-up task
- [ ] Arms and head CAD; fasteners, horn bolt patterns and bearing seats; stress check of the printed parts (Marvin Bot's `cad/fea.py` with printed-material properties); joint sweep for part clashes
- [ ] Harness generator (`electrical/`)

## Open items to verify before ordering

1. STS3250 and STS3215-C018 figures from the manufacturer: stall current, firmware gains (P, D, I registers), return delay, sync read support by firmware version. The STS3215-C018 figures are from resellers so far.
2. The Orin Nano Super developer kit's DC input range (forum replies cite 9–20 V and 7–20 V; the carrier board specification decides) and its mass; it sets the buck-boost from the 3S pack.
3. IMU: a BMI088 on a microcontroller (low latency, raw data) or a BNO085 (fused orientation, simpler).
4. CSI camera model and its driver on the JetPack release the Orin Nano runs.
5. ~~Shoulder torque getting up~~: settled, the shoulder pitch is an STS3250 (decisions 2026-11-09).
6. The STS3250's 75 rpm no-load speed caps walking speed: flat v3 walks 0.3-0.36 m/s within its ratings; faster is untried.
