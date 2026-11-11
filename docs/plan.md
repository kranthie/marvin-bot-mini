# Plan

Marvin Bot Mini is built in stages, like Marvin Bot. Each stage ends with a pass check that decides whether to move on. At this size the parts are cheap, so the whole kit is bought at once, but only after simulation has confirmed the servo choice.

## Principles

1. **Simulation before hardware.** The servo assignment must pass the torque checks in simulation (standing, squatting, getting up) before the kit is ordered.
2. **Model the servos, don't guess them.** The servos are position-controlled with no torque mode, so the simulation copies their firmware loop, and the model is fitted from bench measurements before training for the real robot.
3. **One order, after the check.** One priced order list once phase 1's torque checks pass; nothing is bought without the owner's OK.
4. **Measure, then update the model.** Every hardware stage measures the real thing (servo response, heat, bus timing, mass) and feeds it back into `description/` and the simulation.

## Phases

| # | Phase | Work | Pass check | Indicative time (part-time) |
|---|---|---|---|---|
| 0 | Foundations | Repo, licences, plan, decisions, requirements, safety; description v0.1; mass budget; draft BOM; licence check of the open-source references | Layout and v0 numbers agreed | days |
| 1 | Robot in simulation | code-CAD of the frame (computer and battery bays), arms, wrists and pincers, head and clip-on shells, assembly ready; URDF from the CAD masses; MuJoCo standing, squat and get-up torque checks under the torque rule; the servo model (datasheet-derived) in Isaac Lab and MuJoCo; first walking policy and its MuJoCo cross-check | **Order gate:** torque checks pass with the chosen servos, then a priced order list for the owner's OK. **Phase pass:** walks, turns and stops in MuJoCo with the provisional servo model | 2–3 weeks |
| 2 | Servo bench | Fit the servo model from pendulum tests (BAM method) on an STS3250 and an STS3215; bus timing with three buses and 27 servos on the Orin Nano; knee servo heating at its walking RMS torque; retrain walking with the fitted model | Model matches logged motion; 50 Hz loop with under 5 ms jitter; no thermal cutoff at walking load | 1–2 weeks after delivery |
| 3 | Print and build | Print and assemble, zero calibration, harness, power and e-stop; hang the robot on a stand and replay simulated trajectories in the air | Every joint tracks its simulated trajectory in the air; mass within 10% of the budget | 2–3 weeks |
| 4 | Walking | Stand, walk on a tether, walk free, push recovery; train and test get-up face up and face down | Walks at about 0.3 m/s, turns, stops, recovers from light pushes; gets up 8 of 10 times each way | 2–4 weeks |
| 5 | Skills | Motion-tracking clips on the robot; walk to and kick a ball and follow with the camera; skills page and website | Clips play upright on the robot; kicks a ball from 1 m 7 of 10 times | ongoing |

## Spend

About $1.37k in parts besides the computer ($1.6k with it), at listed prices of 2026-11-10 with the servos from WowRobo's shop, all in the phase 1 order: `bom/bom.xlsx`, totalled by `tools/bom/summary.py`. Not included: the 3D printer, the desktop GPU, shipping, import duties and taxes.

## Repository growth

Still to come, as phases reach them: `cad/`, the generated URDF, `sim/mujoco/`, `sim/isaaclab/`, `sim/motion/` and a harness generator in `electrical/` (phase 1); `bench/` and `robot/` (Feetech driver, control loop, safety; phase 2); build guides in `docs/build/` (phase 3); the website with a 3-D viewer once there is a model to show. Trained policies are not committed.
