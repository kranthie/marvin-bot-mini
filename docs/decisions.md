# Decisions

Settled decisions, newest first, with the reason and the alternatives considered.

## 2026-11-09: the order gate passes; shoulder pitch on the STS3250; longer arms

`sim/mujoco/check_torques.py` holds the robot with its simulated servos on an empty pack (10.5 V) through five cases: standing, a squat to knee 120° and back with the centre of mass kept over the feet, a push-up face down on the hands and toes, sitting leaning far back on the hands, and rising from a hands-down crouch (knee 130°). With the first CAD pass (2.92 kg bare) every joint stays within the torque rule; the worst are the elbows in the push-up (0.72 N·m, 47% of the rule's 1.54 N·m) and the shoulder pitch in the sit-up (1.13 N·m, 44% of 2.57 N·m). Two changes came out of it:

- **Shoulder pitch on the STS3250** (open item 5 of phase 0). On the STS3215 the far-lean sit-up held it at 1.43 N·m, 93% of the rule, and 1.23 N·m RMS against 0.98 N·m rated. Cost: 39 g and about $54.
- **Arms 0.24 m** (upper arm 0.10, forearm with the hand stub 0.14; were 0.09 and 0.11). At 0.20 m the hands did not reach the floor sitting, so the robot could not push itself up from its back. The arm is now 0.40 of the height, as Marvin Bot's.

These are quasi-static holds and slow moves with no balance controller; a trained get-up policy's dynamic peaks are checked again when it exists. Alternatives: a static hand calculation per pose (misses how the weight shares between hands and feet), or waiting for the get-up policy (later than the order should wait).

## 2026-11-09: a reflected rotor inertia in the servo model; printed parts' first pass

The servo model's back-EMF damping (kt²/R, about 0.6 N·m·s/rad on an STS3250) on the small links' bare inertia was numerically unstable at a 1 ms step. A real servo's rotor, geared 345:1, adds a large reflected inertia: BAM's STS3215 fit gives 0.022 kg·m², used as a placeholder (0.02) for both servos until the bench fit. kt and R are now derived at the firmware's 97% duty cap, so the model reproduces the datasheet's stall torque and no-load speed exactly.

The first CAD pass (`cad/parts.py`) draws the legs, pelvis and torso as plates and shells round the servo cases: nylon-carbon hip brackets, thighs, shins and ankle cross pieces, PETG feet, pelvis and torso frame, TPU soles. It weighs 0.43 kg for the legs and pelvis and 0.07 kg for the torso frame (estimates were 0.37 and 0.10), modelled at wall thickness as solid perimeters. No fasteners, horn bolt patterns or bearing seats yet, and no stress check. The frame budget item is split into leg, torso, arm and head frames so the CAD takes over the first two while the arms and head stay estimates.

## 2026-11-09: v0.1 numbers: 0.6 m, 3 kg, three servo buses, a torque rule for servos

Proportions follow Marvin Bot's: hip axis 0.29 m (thigh 0.12, shin 0.12, ankle 0.05), shoulders at 0.48 m, 0.20 m wide. The estimate is 2.85 kg bare and 3.05 kg with the add-on shells (servos 1.50 kg); the target is 3.0 kg bare and the limit 3.5 kg. Three buses (one per leg, one for the upper body) keep each bus busy 18–25% of a 50 Hz cycle with a sync write and a sync read (`tools/check_description.py`), leaving room for 100 Hz. The servo torque rule replaces Marvin Bot's 70%-of-peak rule: peak torque within 60% of stall at the pack's empty voltage (10.5 V: 2.57 N·m on an STS3250, 1.54 N·m on an STS3215) and RMS within the rated torque. A first hand check: a deep double-support squat needs about 1.3 N·m per knee and single support while walking about 0.9 N·m, both inside the rule; getting up is checked in phase 1 before ordering.

## 2026-11-09: a servo model fitted on the bench, after BAM

The servos have no torque mode: the policy sends target angles and the servo's firmware closes the loop. Simulation copies that loop with BAM's model structure (Rhoban, Apache-2.0): a duty cycle from the firmware's P gain and the position error, a motor with an effective torque constant and resistance on the supply voltage, and friction. Until the bench fit, kt and R are derived from the datasheet stall torque and no-load speed, and the gains from BAM's STS3215 figures (`description/actuators.yaml`). Phase 2 fits the model from pendulum tests and training randomises around the fit. Alternatives: Isaac Lab's ideal PD with heavy randomisation (fast to start, likely to need repeated real-world tuning) and a learned actuator network (needs much more bench data; a later option).

## 2026-11-09: onboard compute, power, camera

The Orin Nano Super 8 GB (owned) runs a 50 Hz control loop outside ROS (servo I/O, ONNX policies, safety) and light vision (AprilTags, a ball or person detector). Training, teleoperation and any vision-language-action model stay on the desktop; a small (~0.5B) VLA onboard is a later experiment. A 3S Li-ion pack of about 3 Ah (35 Wh) feeds the 12 V servos directly through a fuse and the e-stop, with a buck-boost for the computer, for 25–40 minutes. A wide-angle CSI RGB camera sits on the pan-tilt neck; depth comes later if stairs or obstacles become goals. Alternatives: a tether until walking works (the robot then behaves differently untethered), 4S with a 12 V buck (needs a 20–30 A converter), a stereo depth camera (mass high up, cost).

## 2026-11-09: a printed frame

Everything is printed on a desktop FDM printer: PETG or PLA+ brackets and shells, nylon-carbon at the hips and knees, steel pins and small bearings on the far side of loaded joints, heat-set inserts. No mill. The stress check (copied from Marvin Bot) uses printed-material properties with a derating for layer direction. Alternatives: milled aluminium hip and pelvis plates (brings back the mill), bought aluminium servo brackets (vendor geometry, generic look).

## 2026-11-09: Feetech STS servos, 23 joints with Marvin Bot's names

Feetech STS3250 (4.9 N·m stall, 75 rpm, 75 g) on the twelve leg joints and STS3215 12 V (C018, 2.9 N·m stall, 45 rpm) on the arms, waist and neck: magnetic 12-bit encoders, a 1 Mbit/s bus, readable position, load, voltage and temperature, the same protocol as Marvin Bot's neck servos. The layout keeps Marvin Bot's joints except the wrists and hands: 6 per leg (roll-first hip, knee, ankle pitch and roll driven directly, no linkage), 4 per arm, waist yaw, 2 neck. The same names let the motion pipeline carry over. No hands in v1; a 1-DOF gripper may follow. Alternatives: Dynamixel X-series (a real current mode and less backlash, but 3–5 times the servo cost), STS3215 throughout (a 0.45 m robot with thin knee margins), potentiometer bus servos (low resolution and range), 27 joints with wrists (little gain without hands), 12 leg joints first (cheapest, but no upper-body skills).

## 2026-11-09: a separate repository, tools copied from Marvin Bot

Marvin Bot's Lite variant works because it is Max's upper body unchanged. The mini shares no parts, scale, actuators or fabrication with Max, so it is its own repository with the same layout and conventions, and Marvin Bot's tools are copied, trimmed and adapted for servos and printing, credited back. Alternatives: a third variant in Marvin Bot (most tools would need servo and print branches, and Max's CI would grow) and a shared tools package for both (cleaner long term, but means refactoring Marvin Bot first).

## 2026-11-09: built in stages, one order after the simulation check

As Marvin Bot, simulation comes first, but at about $1.4k the kit is bought in one order once phase 1's torque checks confirm the servo assignment, and the bench fit runs on the first servos to arrive. Nothing is ordered without the owner's OK.
