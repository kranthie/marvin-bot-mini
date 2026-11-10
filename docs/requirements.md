# Requirements

"Must" items gate the design; "should" items are targets that may move.

## Size and mass

| ID | Requirement | Target | Level |
|---|---|---|---|
| R1 | Standing height | about 0.60 m | must |
| R2 | Total mass with battery | ≤3.0 kg target bare; ≤3.5 kg hard limit as it runs, add-on shells included | must |
| R3 | Joint names and layout | Marvin Bot's joints without the wrists and hands, so its motion pipeline carries over | must |

## Mobility

| ID | Requirement | Target | Level |
|---|---|---|---|
| R10 | Walk on flat ground, turn in place, stop on command | 0.3 m/s walk, 0.5 rad/s turn | must |
| R11 | Rough ground | 1 cm obstacles, ±5° slopes | should |
| R12 | Push recovery | recovers from a light push while standing and walking | should |
| R13 | Fall recovery | gets up unaided from face up and face down, 8 of 10 times each | must |

## Skills

| ID | Requirement | Target | Level |
|---|---|---|---|
| R20 | Motion tracking | plays clips generated with Kimodo (gestures, squat, kick, dance) on the robot, upright | must |
| R21 | Ball kick | walks to a ball seen by the head camera and kicks it, from 1 m, 7 of 10 times | should |
| R22 | Follow | follows a person or a tag at walking pace | should |

## Compute and control

| ID | Requirement | Target | Level |
|---|---|---|---|
| R30 | Onboard control | 50 Hz control loop on the Orin Nano, under 5 ms jitter; 100 Hz if the buses allow | must |
| R31 | Onboard vision | AprilTag and one small detector at 10 Hz or more alongside the control loop | should |
| R32 | Simulation | every policy passes a MuJoCo cross-check with the fitted servo model before it runs on the robot | must |

## Power, cost, build

| ID | Requirement | Target | Level |
|---|---|---|---|
| R40 | Run time | 25 minutes of mixed standing and walking per pack | should |
| R41 | Cost | about $1.5k in parts, the computer included; priced at $1.8k on 2026-11-10 (decisions) | should |
| R42 | Fabrication | a desktop 3D printer and hand tools; no machining | must |
| R43 | Servo loads | peak torque within 60% of stall at the pack's empty voltage, RMS within rated (`robot.yaml` `torque_rule`) | must |
