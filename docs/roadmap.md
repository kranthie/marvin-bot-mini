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
- [x] Mass budget (2.85 kg bare, 3.05 kg with the shells) and draft BOM (`bom/bom.yaml`)
- [x] Description check: joints, servos, bus timing, torque budget, mass, BOM counts (`tools/check_description.py`)
- [x] Licence check of the open-source references (`ACKNOWLEDGMENTS.md`)
- [x] v0.1 layout agreed by the owner
- [ ] GitHub repository created (owner's OK)

## Phase 1

- [x] Skeleton (`description/skeleton.yaml`) and URDF generated from the description files (`tools/build_urdf.py`, adapted from Marvin Bot's)
- [x] Servo model after BAM, shared by MuJoCo and (to come) Isaac Lab (`sim/servo_model.py`), unit-tested against the datasheet stall torque and no-load speed
- [x] The robot in MuJoCo driven by the servo model (`sim/mujoco/model.py`)
- [x] Torque check, the order gate (`sim/mujoco/check_torques.py`): stand, squat, push-up, sit-up, crouch-rise within the rule; shoulder pitch moved to the STS3250 and arms lengthened to 0.24 m (decisions 2026-11-09)
- [x] Code-CAD first pass of the legs, pelvis and torso (`cad/parts.py`, `cad/build.py`): meshes and masses in the URDF; 2.92 kg bare
- [x] Renders and animations of the torque check (`tools/render_robot.py`)
- [ ] Priced order list for the owner's OK (prices in `bom/bom.yaml` are estimates)
- [ ] Isaac Lab: robot config with a servo actuator class, walking task, first policy; MuJoCo cross-check of the policy
- [ ] Arms and head CAD; fasteners, horn bolt patterns and bearing seats; stress check of the printed parts (Marvin Bot's `cad/fea.py` with printed-material properties); joint sweep for part clashes
- [ ] Harness generator (`electrical/`)

## Open items to verify before ordering

1. STS3250 and STS3215-C018 figures from the manufacturer: stall current, firmware gains (P, D, I registers), return delay, sync read support by firmware version. The STS3215-C018 figures are from resellers so far.
2. The Orin Nano Super developer kit's DC input range (forum replies cite 9–20 V and 7–20 V; the carrier board specification decides) and its mass; it sets the buck-boost from the 3S pack.
3. IMU: a BMI088 on a microcontroller (low latency, raw data) or a BNO085 (fused orientation, simpler).
4. CSI camera model and its driver on the JetPack release the Orin Nano runs.
5. ~~Shoulder torque getting up~~: settled, the shoulder pitch is an STS3250 (decisions 2026-11-09).
6. The STS3250's 75 rpm no-load speed caps walking speed; phase 1's policy check reports joint speeds against it.
