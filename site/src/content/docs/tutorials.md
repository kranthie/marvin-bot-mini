---
title: Learn
description: "Learning humanoid robotics: Marvin Bot's interactive guide covers the ideas behind this robot too."
---

Marvin Bot Mini is the miniature of [Marvin Bot](https://kranthie.github.io/marvin-bot/), and the
ideas behind both are the same. **Marvin Bot's interactive guide** teaches humanoid robotics from
first principles in 23 playgrounds with sliders and live pictures, from degrees of freedom and
balance to reinforcement learning, motion tracking and sim-to-real. Its numbers are Marvin Bot's
(1.41 m, 40 kg, quasi-direct-drive actuators); what changes at the mini's scale is below.

[Open Marvin Bot's guide →](https://kranthie.github.io/marvin-bot/guide/)

## What is different at 0.6 m

| Topic | Marvin Bot | Marvin Bot Mini | Read more |
|---|---|---|---|
| Actuators | RobStride quasi-direct-drive motors, torque-controlled over CAN | Feetech STS bus servos: position-controlled by their own firmware, no torque mode | [decisions](/marvin-bot-mini/decisions/), guide [04](https://kranthie.github.io/marvin-bot/guide/04-actuators/) |
| Actuator model in training | a DC motor's torque-speed line, with PD gains | the servo's firmware loop: duty cycle from the position error, torque from the supply voltage and back-EMF (BAM) | [Isaac Lab](/marvin-bot-mini/simulation/isaac-lab/), guide [20](https://kranthie.github.io/marvin-bot/guide/20-sim-to-real/) |
| Torque rule | 70% of peak | 60% of stall at the empty pack for held poses; brief saturation allowed when moving | [requirements](/marvin-bot-mini/requirements/) |
| Structure | machined aluminium plates | printed PETG and nylon-carbon | [overview](/marvin-bot-mini/overview/) |
| Power | 48 V, 630 Wh | 3S Li-ion, 39 Wh, straight to the 12 V servos | [safety](/marvin-bot-mini/build/safety/) |
| Compute | Jetson AGX Orin 64 GB | Jetson Orin Nano Super 8 GB | [plan](/marvin-bot-mini/plan/) |
| Cost | about $17.6k | about $1.8k | [bill of materials](/marvin-bot-mini/build/bom/) |
