# Acknowledgments

Marvin Bot Mini is designed from scratch, but it learns from open projects, published specifications and tools. This file credits them. Nothing from them is copied into this repository unless a line below says so and the licence allows it. Licences were checked on 2026-11-09.

## Marvin Bot

- **Marvin Bot**: https://github.com/kranthie/marvin-bot (software Apache-2.0, hardware CERN-OHL-S-2.0), by the same contributors. The architecture, conventions, the description file format, the description check and the BOM summary are carried over from it, under the same licences.

## Small open humanoids and servo models (ideas and methods)

- **ToddlerBot** (Stanford University): https://github.com/hshi74/toddlerbot (MIT). A low-cost open servo humanoid; reference for zero-point calibration, system identification of servos and sim-to-real on a small robot. Ideas only.
- **Open Duck Mini**: https://github.com/apirrone/Open_Duck_Mini (Apache-2.0). An open small biped on STS3215 servos walking with a policy trained in simulation; reference for the servo model fitting workflow. Ideas only.
- **Zeroth bot**: https://github.com/zeroth-robotics/zeroth-bot (MIT). A small open humanoid on STS3215 servos; reference for scale and servo choice. Ideas only.
- **Berkeley Humanoid Lite** (UC Berkeley Hybrid Robotics): https://github.com/HybridRobotics/Berkeley-Humanoid-Lite (MIT). Lessons on printed structures that route load through metal fasteners.
- **BAM: Better Actuator Models** (Rhoban; M. Duclusaud, G. Passault): https://github.com/Rhoban/bam (Apache-2.0) and "Extended Friction Models for the Physics Simulation of Servo Actuators", arXiv:2410.08650. The provisional servo model's structure (firmware duty cycle, effective torque constant and resistance, friction) and its STS3215 firmware-gain figures (`description/actuators.yaml`); the pendulum fitting method planned for phase 2.

## Software and models

- **NVIDIA Isaac Lab** (https://github.com/isaac-sim/IsaacLab, BSD-3-Clause) and **Newton**: training environments. The walking tasks, PPO settings and terrain config in `sim/isaaclab/` come from Marvin Bot's, which adapt Isaac Lab's G1 velocity tasks; those files keep Isaac Lab's notice (`LICENSES/BSD-3-Clause-IsaacLab.txt`).
- **rsl_rl** (ETH Zurich Robotic Systems Lab, https://github.com/leggedrobotics/rsl_rl, BSD-3-Clause): PPO training, used as a library.
- **MuJoCo** (Google DeepMind, https://github.com/google-deepmind/mujoco, Apache-2.0): the simulation cross-checks.
- **ProtoMotions** (NVIDIA, https://github.com/NVlabs/ProtoMotions, Apache-2.0): the motion pipeline in `sim/motion/` runs its keypoint extraction, PyRoki retargeter (G1 version, with the mini's link map and scaling), converters, motion library, motion-tracking training and ONNX export; `setup_protomotions.py` writes an edited copy of its BeyondMimic experiment into the user's checkout. Its own sample motions (BONES-SEED) are not used.
- **Kimodo** (NVIDIA, https://github.com/nv-tlabs/kimodo, code Apache-2.0; the Kimodo-SOMA-RP model under the NVIDIA Open Model License, trained on Bones Studio's motion capture): generates the human motions the trackers learn from (`sim/motion/generate_kimodo.py`). The motions are not redistributed.
- **PyRoki** (https://github.com/chungmin99/pyroki, MIT): the kinematics optimiser behind the retargeter.
- **BeyondMimic** (https://github.com/HybridRobotics/whole_body_tracking, arXiv:2508.08241): the motion-tracking formulation ProtoMotions' tracker follows.
- **LeRobot** (Hugging Face, https://github.com/huggingface/lerobot, Apache-2.0): its Feetech bus driver is the reference for the onboard driver (phase 2).
- **HoST** (https://github.com/InternRobotics/HoST, MIT; arXiv:2502.08378): the planned get-up training approach.

## Tools

- **build123d** (https://github.com/gumyr/build123d, Apache-2.0) on **OCP** (https://github.com/CadQuery/OCP, Apache-2.0) and **Open CASCADE Technology** (https://dev.opencascade.org, LGPL-2.1 with the OCCT exception): the code-CAD in `cad/`. **trimesh** (https://github.com/mikedh/trimesh, MIT): meshes and mass properties.
- **openpyxl** (MIT): the BOM workbook. **NumPy**, **PyYAML**, **Pillow**, **ONNX Runtime**: used as libraries.
- **Astro** and **Starlight** (https://github.com/withastro/astro, https://github.com/withastro/starlight, MIT): the documentation website in `site/`. **three.js** (https://github.com/mrdoob/three.js, MIT): the 3-D model on the website. The site, its 3-D viewers and the 3-D model page are Marvin Bot's, adapted.

## Parts

- **Feetech** STS3250 datasheet (ST-3250-C001) and STS3215-C018 reseller listings: the servo figures in `description/actuators.yaml`.
- **NVIDIA** Jetson Orin Nano Super developer kit: the onboard computer.

## Development

- Built with **Claude** and **Claude Code** (Anthropic): the design, code and documentation were developed with them; commits they contributed to are marked `Co-Authored-By: Claude`.
