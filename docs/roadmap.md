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
- [x] Code-CAD first pass of the legs, pelvis and torso: meshes and masses in the URDF; 2.92 kg bare
- [x] Frame and shells redrawn assembly ready (`cad/frame.py`, `cad/shells.py`): servos placed by their real shaft offset, cradles, horn plates and far-side bearings, wrists and pincers, Marvin Bot-style shells hooked and screwed on, clear through every joint's range (`python -m cad.check`, 69 poses); 3.30 kg bare, 3.60 kg with the shells, torque checks pass ([assembly](assembly.md), decisions 2026-11-10)
- [x] Flat walking retrained on the 27-joint model (flat v4, staged: without the torque penalties, with them, then a broader clearance reward); passes the MuJoCo cross-check with and without the shells (decisions 2026-11-11)
- [ ] Flat v4 on an empty pack at 0.3 m/s with the shells (3.86 kg): an ankle pitch above the torque rule for 60 ms (allowed 50); fine-tune on the final mass
- [ ] Measure a servo's horn screw circle, case mounting holes and shaft offset, and correct `cad/servo.py` before printing
- [x] Renders and animations of the torque check (`tools/render_robot.py`)
- [x] Priced order list (`bom/bom.xlsx`, listings of 2026-11-10): $1,370 to buy, servos from WowRobo (decisions 2026-11-10)
- [ ] Order placed
- [x] Isaac Lab package (`sim/isaaclab/`): `ServoActuator` (the servo model with a 5-20 ms command delay and per-robot pack voltage and motor-constant spread), Marvin Bot's walking tasks scaled to the mini; robot check passes on Newton and PhysX
- [x] Mirror rule checked (`sim/mujoco/check_symmetry.py`)
- [x] Flat walking: v3 passes the MuJoCo cross-check with the servo model (`sim/mujoco/check_policy.py`): walk 0.2 m/s, turn, stop; 0.3 m/s on an empty pack; knees at 60-81% of rated RMS (decisions 2026-11-10)
- [ ] Flat v3 on a full pack with a short delay: walks 0.26 m/s for 0.20 and holds a knee above the torque rule for 56 ms (randomize the pack voltage in the command response, or train with it observed)
- [x] Motion pipeline (`sim/motion/`): one tracker plays 30 skills, five for the wrists and pincers, upright in Newton in all and in the MuJoCo check on the servo model in 29, the hardest-working joint at 48-86% of rated RMS ([skills](skills.md), decisions 2026-11-11)
- [ ] Two-hand wave: falls in MuJoCo (feet drift together), holds in Newton; try randomising foot friction and servo gains in training
- [ ] Squat and pick-up to the reference's depth (the tracker stops 12 cm high, knees at stall)
- [ ] Rough-ground task (configured, not trained), get-up task
- [x] Stress check of the printed parts (`python -m cad.fea`, Marvin Bot's): the frame strengthened until every part but one reaches a safety factor of 2.2 on the strength between layers, the neck servos at a 30% torque limit; 3.59 kg bare, 3.86 kg with the shells (decisions 2026-11-11)
- [ ] Hip pitch bracket at a safety factor of 2.0 under the hip yaw's stall torque (at the idler's bearing seat): test it first in print
- [x] Harness: bus chains, lead lengths and service loops (`python -m cad.harness`)
- [ ] Power wiring and the power board's layout (`electrical/`)

## Open items to verify before ordering

1. STS3250 and STS3215-C018 figures from the manufacturer: stall current, firmware gains (P, D, I registers), return delay, sync read support by firmware version. The STS3215-C018 figures are from resellers so far.
2. The Orin Nano Super developer kit's DC input range (forum replies cite 9–20 V and 7–20 V; the carrier board specification decides) and its mass; it sets the buck-boost from the 3S pack.
3. IMU: a BMI088 on a microcontroller (low latency, raw data) or a BNO085 (fused orientation, simpler).
4. CSI camera model and its driver on the JetPack release the Orin Nano runs.
5. ~~Shoulder torque getting up~~: settled, the shoulder pitch is an STS3250 (decisions 2026-11-09).
6. The STS3250's 75 rpm no-load speed caps walking speed: flat v3 walks 0.3-0.36 m/s within its ratings; faster is untried.
