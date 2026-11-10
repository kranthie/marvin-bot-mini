# Safety

Marvin Bot Mini weighs about 3 kg and runs on a 3S lithium battery. It is far less dangerous than a full-size humanoid, but its servos can pinch fingers, it can fall onto things or off a table, and a damaged or mischarged lithium pack can start a fire. These rules apply from the first servo bench test onward and will grow with each phase.

> [!WARNING]
> Nothing described below has been built or tested yet. These notes are a starting point, not a
> complete safety procedure, and do not replace your own risk assessment or the component makers'
> instructions. You build and run the robot at your own risk; see the licences' warranty
> disclaimers.

## Servos

- Clamp a servo or limb to the bench before powering it. Keep fingers out of the range of motion while torque is enabled.
- Start with low torque limits and low firmware gains, and raise them only after the motion has been checked.
- Set each servo's angle limits and temperature limit in its memory before it goes into the robot.
- The bench scripts and the driver are untested until marked otherwise: check every command at low limits first.

## E-stop and power

- The e-stop cuts power to every servo; the computer stays powered so logs and state survive.
- The servo rail has its own fuse.
- A watchdog in the control loop disables servo torque if commands stop arriving.
- On a bench supply, set a current limit.

## Battery

- Charge only with a balance charger, attended, on a non-flammable surface, in a charging bag.
- Store packs at about 50% charge in a fire-resistant container.
- Never use a pack that is swollen, punctured or has been in a hard fall until it has been inspected.
- The control loop reads the pack voltage and sits the robot down below the empty voltage in `description/robot.yaml`.

## Walking tests

- Early standing and walking happens on a tether or a hand-held lifting strap that catches the robot before it hits the floor.
- Test on the floor, never on a table, in a clear area.
- Policies run in simulation first (Isaac Lab and the MuJoCo cross-check) with the same limits, rates and latency as the robot.

## Software safety layers

- Commands from any high-level source (policies, teleoperation) are clamped to joint and speed limits.
- If no new command arrives within a timeout, the robot holds its pose; if the controller itself stops, servo torque is disabled.
- Fall detection relaxes the joints to a low-torque mode.
- Servo faults (over-temperature, overload, voltage) are logged and stop the robot.
