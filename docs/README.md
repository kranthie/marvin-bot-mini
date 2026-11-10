# Documentation

| Document | What it is |
|---|---|
| [plan.md](plan.md) | Phases, their pass checks and the repository's planned layout |
| [roadmap.md](roadmap.md) | Where each phase stands and the open items |
| [requirements.md](requirements.md) | What the robot must and should do |
| [decisions.md](decisions.md) | Every design decision, newest first, with reasons and alternatives |
| [skills.md](skills.md) | What the robot does in simulation: walking and the tracked skills, with their clips |
| [safety.md](safety.md) | Safety rules for servos, batteries and walking tests |
| [../sim/isaaclab/README.md](../sim/isaaclab/README.md) | The servo actuator, the walking tasks and every run's results |
| [../sim/motion/README.md](../sim/motion/README.md) | The motion-tracking pipeline |
| [../bom/README.md](../bom/README.md) | Bill of materials and how its prices are sourced |

## Glossary

- **DOF:** degree of freedom, one powered joint axis. Marvin Bot Mini has 23.
- **Bus servo:** a servo with its own controller, addressed by id on a shared serial bus; the Feetech STS3250 and STS3215 used here read back position, speed, load, voltage and temperature.
- **Position control, firmware loop:** the servo takes a goal angle and its own firmware turns the error into a PWM duty cycle; there is no torque mode. The simulations model that loop (`sim/servo_model.py`).
- **Stall torque, rated torque:** the torque at full duty and no speed, and the torque the servo can hold continuously. Both scale roughly with the supply voltage.
- **Torque rule:** robot.yaml's limit for the servos: held poses within 60% of stall at the empty pack, brief saturation allowed when moving, RMS within rated.
- **BAM:** Better Actuator Models (Rhoban), the open servo-model project whose model structure is used here.
- **URDF, MJCF, USD:** robot description formats: URDF is generated here from `description/`; MuJoCo reads URDF or MJCF; Isaac Sim uses USD.
- **Code-CAD:** parts written as Python code (`cad/`) instead of drawn in a CAD program, so they regenerate from the description.
- **CoM:** centre of mass.
- **IMU:** inertial measurement unit (gyro and accelerometer) in the pelvis.
- **BMS:** battery management system (protection board).
- **PPO:** the reinforcement-learning algorithm used to train the walking policies.
- **Policy:** the trained neural network that turns sensor readings into joint targets.
- **Flat v3:** the 3rd flat-ground training run; each resumed or replaced the one before after a change.
- **Sim-to-real:** making a policy trained in simulation work on the real robot.
- **Cross-check:** replaying a policy trained in Isaac Lab in a second simulator (MuJoCo) to catch simulator-specific tricks.
- **Motion tracking (BeyondMimic):** training a policy to follow a reference motion, such as a retargeted human walk.
- **Retargeting:** converting a human motion to the robot's body and joints.
