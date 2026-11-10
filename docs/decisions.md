# Decisions

Settled decisions, newest first, with the reason and the alternatives considered.

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
