# Marvin Bot Mini

A ~0.6 m, ~3.6 kg, 27-DOF open humanoid on Feetech STS serial-bus servos (STS3250 legs, STS3215 12 V upper body, wrists and pincer grippers), printed frame, Jetson Orin Nano Super 8 GB onboard, 3S battery. The miniature of Marvin Bot (the 1.41 m humanoid), with the same architecture. Currently in phase 1 (the robot in simulation).

## Read first
- `README.md`: overview and layout
- `docs/plan.md`: phases, pass checks and spend per phase
- `docs/decisions.md`: settled decisions, newest first. Don't re-open them without a reason; add new ones here.
- `docs/roadmap.md`: current phase and open items

## Conventions
- `description/robot.yaml` is the single source of truth for joints, ranges, sizes, servos, buses, power and the torque rule; `actuators.yaml` for servo specs and their simulation model; `mass_budget.yaml` for non-servo masses. Everything else (URDF, CAD masses, BOM counts, harness) is generated from or checked against them.
- Joint names follow Marvin Bot's, so its motion pipeline carries over.
- SI units in code and configs; joint ranges in `robot.yaml` are degrees.
- Stack: Isaac Lab 3.0 for training with a fitted servo actuator model (BAM-style), MuJoCo as a cross-check, ProtoMotions for motion tracking, ONNX policies in a 50 Hz control loop on the Orin, outside ROS.

## Rules
- Keep `tools/check_all.sh` passing (CI runs it).
- Write for future open-source readers: no personal names, local or machine-specific paths, or private links. Keep fabrication tooling generic (no specific printer brands).
- Refer only to open-source projects: no commercial robots as benchmarks, comparisons or style references. Bought parts are named as parts. Check a project's licence before drawing on it and credit it in `ACKNOWLEDGMENTS.md`.
- Do not commit vendor CAD unless its licence allows redistribution. Motion data and research notes stay outside the repo.
- Ask before ordering parts.
- Code marked untested stays marked until it has run.
