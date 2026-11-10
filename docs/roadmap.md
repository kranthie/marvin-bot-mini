# Roadmap

Current phase: **0, foundations** (started 2026-11-09). Phases and pass checks are defined in [plan.md](plan.md).

| Phase | Status |
|---|---|
| 0 Foundations | in progress |
| 1 Robot in simulation | not started |
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
- [ ] v0.1 layout agreed by the owner; GitHub repository created (owner's OK)

## Phase 1

- [ ] Copy and adapt from Marvin Bot: `tools/build_urdf.py`, the skeleton file, `cad/build.py`, `cad/covers.py --check`, `cad/fea.py` (printed-material properties), the MuJoCo checks
- [ ] build123d CAD of the legs, pelvis and torso; URDF from the CAD masses
- [ ] MuJoCo standing, squat and get-up torque checks under the torque rule (the order gate)
- [ ] Priced order list for the owner's OK
- [ ] Servo actuator model in Isaac Lab and MuJoCo (BAM's model structure, unit-tested)
- [ ] Isaac Lab walking task and first policy; MuJoCo cross-check
- [ ] Arms and head CAD; harness generator

## Open items to verify before ordering

1. STS3250 and STS3215-C018 figures from the manufacturer: stall current, firmware gains (P, D, I registers), return delay, sync read support by firmware version. The STS3215-C018 figures are from resellers so far.
2. The Orin Nano Super developer kit's DC input range (forum replies cite 9–20 V and 7–20 V; the carrier board specification decides) and its mass; it sets the buck-boost from the 3S pack.
3. IMU: a BMI088 on a microcontroller (low latency, raw data) or a BNO085 (fused orientation, simpler).
4. CSI camera model and its driver on the JetPack release the Orin Nano runs.
5. Getting up is the worst torque case for the shoulders (STS3215: 1.54 N·m under the rule): if phase 1's check fails, the shoulder pitch moves to an STS3250.
6. The STS3250's 75 rpm no-load speed caps walking speed; phase 1's policy check reports joint speeds against it.
