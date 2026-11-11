# Decisions

Settled decisions, newest first, with the reason and the alternatives considered.

## 2026-11-11: five skills for the wrists and pincers, the tracker retrained

With a wrist yaw and a pincer on each arm, five clips were added for them (pouring, drinking,
turning a door knob, screwing in a light bulb overhead, handing something over), and the tracker
was retrained on all 30. No keypoint fixes a wrist's turn or a grip, so `add_hand_motion.py` sets
them after retargeting: the wrist from the person's hand twist against the forearm, less its idle
average; the right pincer from a schedule per clip worked out from the hand's path (open on the
way to the object, closed on it while held, open to let go), since Kimodo's hands never open or
close. The left pincer stays closed. The tracker scores a 1 g hand body on each palm, so its
wrist turn is tracked as the hand's orientation.

It trained 3,000 epochs from scratch with the servo-load penalty at 0.5, resumed once after the
GPU's display watchdog stopped it at epoch 1,560, then 600 more on the strengthened frame's mass:
the first replay, of a tracker trained on the 3.51 kg model, ran on the 3.59 kg one. In MuJoCo
the wrist clips put the hand 3-9 cm from the reference and every clip but one stays upright: the
two-hand wave falls in its first second as the feet drift together and catch, in MuJoCo only and
whatever the noise or command delay. It is left as an open item (roadmap) rather than held up for:
the 25-clip tracker kept it, and the likely cure, randomising the feet's friction and the servo
gains in training, is a longer run. Alternatives: dropping the two-hand wave from the set (it
hides the gap rather than closing it), and grips from a hand-pose model (Kimodo has none for these
hands; a schedule does for a one-servo pincer).

## 2026-11-11: the frame stress-checked and strengthened, walking retrained

**Stress check.** Marvin Bot's finite-element check (`cad/fea.py`: quadratic tetrahedra, each part
held where it is driven and loaded where the next link hangs on it) now runs on every printed
part, with each part's contact patches recorded by the CAD: a landing on one foot at three body
weights, a sideways fall onto the knee, a hand catching half the body weight, the head dropped at
10 g and each servo's stall torque, against the printed material's strength between layers
(25 MPa) for a safety factor of 2.2. The first pass found the frame far too weak in places: the
pelvis at 0.9 (landing), the hip pitch brackets at 0.9, the torso's floor at 0.4 and the neck yoke
at 0.5 (both bent round a servo horn by a stall torque), the shoulder yoke at 1.4, the foot at 1.6.
What changed:

- **Horn plates with skirts.** A 3-4 mm plate on a horn bends round the horn's 20 mm face when a
  moment crosses it. Where it did, the plate gets a skirt under it, down to just above the
  servo's top face and open round the horn (the hip pitch brackets' floors, the torso's waist
  hub, the neck yoke), and is thicker where the next servo's sweep leaves room (`swept()` trims
  it). Plates over 3.2 mm are counterbored so the M2 x 6 horn screws still reach the horn.
- **The pelvis** carries the hips' loads to the waist servo's case through a channel (two walls
  and a floor) between the cradles, gussets down the hip roll cradles' inner walls, and thicker
  waist cradle sides; the hip roll bracket's swing trims them.
- **The torso** has a front rib across its floor, gusseted up the side frames, and an 8 mm hub on
  the waist horn. The shoulder yokes' bottoms are 5 mm, the hip pitch idlers 5 mm and wider, the
  shins' back webs 7 mm, and the feet gain gussets up the back of the ankle's idler lug.
- **The neck servos run at a 30% torque limit** (robot.yaml `torque_limit`, set in the servo at
  setup). The neck holds a 0.2 kg head with under 10% of stall; at full stall torque its yoke
  would need to be much heavier.

Results (`python -m cad.fea`; the left side, the right is its mirror):

| Part | Worst case | Safety factor |
|---|---|---|
| torso frame | shoulder pitch stall torque | 2.2 |
| pelvis | landing | 2.3 |
| hip roll bracket | landing | 4.7 |
| hip pitch bracket | hip yaw stall torque | **2.0** |
| thigh | landing | 4.2 |
| shin | ankle pitch stall torque | 2.3 |
| ankle bracket | ankle roll stall torque | 3.1 |
| foot | landing | 2.3 |
| shoulder bracket | shoulder roll stall torque | 5.0 |
| shoulder yoke | hand catch | 2.4 |
| upper arm | hand catch | 8.7 |
| forearm | wrist yaw stall torque | 2.5 |
| palm | gripper stall torque | 11.2 |
| neck yoke | neck pitch at its 30% limit | 4.0 |

The hip pitch bracket stays at 2.0: its peak is at the idler's bearing seat, where the model ties
the bearing rigidly to its bore, and a thicker idler meets the hip girdle. It is the part to test
first in print. Alternatives considered: nylon-carbon for the PETG parts (the same 25 MPa between
layers, so no gain in this check), and thicker plates throughout (over the mass limit).

**Mass.** The frame gained 77 g, the shells lost 37 g (1.4 mm walls, from 1.6: they carry no
load): 3.59 kg bare, 3.86 kg with the shells. The target is now 3.6 kg bare; the 3.9 kg limit
stays. The torque checks pass with the shells (`check_torques.py --covers`): the elbow in the
push-up peaks at 66% of the rule and runs at 92% of its rated torque RMS.

**Walking retrained (flat v4).** The 27-joint robot's policy had to be trained from scratch (its
observations hold every joint). With the torque penalties on from the start it learned to throw
itself over within 0.2 s, as flat v1 did; trained 3,000 iterations without them, 2,000 with them
and 1,500 more with a broader foot clearance reward (the sharp one paid nothing to a policy
shuffling with 0.5 cm of lift), it walks, turns and stops in the MuJoCo cross-check with and
without the shells ([sim/isaaclab/README.md](../sim/isaaclab/README.md)).

## 2026-11-10: the frame and shells redrawn, assembly ready, with wrists and pincers

The first CAD pass drew the legs, pelvis and torso as plain plates and boxes round servo cases
centred on their joints, and several cases overlapped (the shoulder roll and yaw servos, the ankle
pitch and roll servos). The robot is now drawn from scratch in `cad/` on manifold3d:

- **Servos where they really sit.** Each case is placed by its top face on the shaft's axis, its
  output and length directions (robot.yaml `output`, `length`, Marvin Bot's form) and the shaft's
  11.5 mm offset from the case's centre (actuators.yaml `shape`, Marvin Bot's STS3250 figures).
  Joint axes are unchanged except two: the ankle roll axis is 22 mm below the pitch axis (with
  them crossing, either servo sweeps into the other within the ankle's range; the roll servo now
  lies in front of the pitch servo), and nothing else moves.
- **One set of joint details.** A cradle round the case (M2 screws into its ends, a notch for its
  bus sockets), a horn plate (four M2 into the horn) and, at the hips, knees, ankle pitch,
  shoulder roll, elbows and neck pitch, an idler plate on a 683 bearing turning on an M3 pin in a
  heat-set insert: carried on both sides, as Marvin Bot's far-side bearings.
- **Wrists and pincers (option 1 of three studied):** a wrist yaw and a one-servo pincer per arm,
  one jaw fixed, one on the gripper servo's horn (the open-source SO-ARM100's arrangement).
  Chosen over parallel jaws (more parts for the same grip on a cup in the animations) and a
  pitch-and-roll wrist (a third servo per arm, all 15 STS3215s used, awkward high-elbow poses at
  this arm length). 13 of the 15 STS3215s are used, two spare.
- **Shells in Marvin Bot's language** (the slim diamond-faceted design chosen from rendered
  options): the torso faceted by Marvin Bot's rules, its side walls inside the shoulder servos,
  which show; octagonal upper-arm sleeves with cooling slots, a faceted forearm, box-section thigh
  and shin shells, a dark hip girdle, toe caps, a rounded-box head with a dark visor. Each is
  sized from what it covers and fastened by a snap hook into a frame slot and M2 screws into
  heat-set inserts on bosses grown from the frame to it.
- **Wiring:** the cradles' socket notches face the next servo in each chain; leads cross joints
  in service loops in the shells' gaps, tied through paired slots; `python -m cad.harness` sizes
  them (27 leads, 100-300 mm).
- **Checked:** `python -m cad.check` moves every joint through its range (69 poses) and fails if
  any part, servo or shell meets another, or a shell cuts into the frame it covers. To pass it,
  ankle roll is +-15° (from 25°), shoulder yaw -45..90° (from +-90°), neck pitch -20..30° (from
  -30..45°), and shoulder roll goes below -2° only with the arm forward (a coupled limit).

The robot is 3.30 kg bare and 3.60 kg with the shells (from 2.92 and 3.12): the arms gained two
servos each and their printed parts are now drawn. The torque checks pass at 3.60 kg (peaks at
most 63% of the rule, the elbow in the push-up at 89% of its rated torque RMS), so the target is
now 3.3 kg bare and the limit 3.7 kg with the shells. The flat walking policy, trained on the
old model, still walks the new one in MuJoCo without falling but too fast (0.27 m/s for 0.20):
the walking and tracking policies are to be retrained on it. The servo's horn screw circle and
case mounting holes are datasheet figures, to measure on a servo before printing
([assembly](assembly.md)).

## 2026-11-10: a servo-load penalty in the motion tracker

The first tracker, trained with Marvin Bot's BeyondMimic-style rewards, held both hip rolls 10° past where they stood, pressing the legs together through the floor: 4.4 N·m each standing still in the MuJoCo check, 2.8 times the STS3250's rated torque, which would overheat them in minutes. Nothing in those rewards sees torque, and a stiff stance costs nothing in simulation; Marvin Bot's larger motors had the margin to hide it. The tracker now pays for each joint's torque beyond half its servo's rated torque, squared (`compute_servo_load_rew`, worked out from the PD target since the simulator's built-in PD does not report its torque). At a weight of 0.1 the hip rolls still held 92% of rated standing; warm-starting 1,000 more epochs at 0.5 brought the hardest-working joint to 46-79% of rated RMS across the 25 clips, and the walking joint error from 20° to 9°. Every clip stays upright in Newton and in MuJoCo; the squat and pick-up stop 12 cm above the reference's depth. Alternatives: lowering the servo gains (the stiffness is the servo firmware's, so the sim would no longer match the servo), or a torque term only in the MuJoCo check (it would catch the problem, not fix it).

## 2026-11-10: both servo models from WowRobo, $1,370 to buy

WowRobo, one of the SO-ARM100 kit sellers its repository lists, sells both servo models on their own: STS3215-C018s at $15.99 ($13.99 each in tens; $27.99 at the retailer priced before) and STS3250-C002s at $43 ($46 at the supplier listing). C002 is the STS3250 without the metal bracket that C001 comes with; the frame's printed U-brackets hold the servos by horn and idler, so the bracket would go unused and add 10-20 g a joint. Spares: one STS3250 rather than two (every STS3250 joint takes the same servo, and the spare is first the bench unit for the servo model fit), and 15 STS3215s, a ten-pack and a five-pack ($214.85): 9 used, one spare and five kept for wrists, which would carry little load (a 55 g servo at the forearm's end adds about 0.13 N·m at the shoulder pitch, which peaks at 1.22 of 2.57 N·m in the sit-up; an estimate, not simulated) but are not designed until there is a hand or gripper to point. That takes the order to $1,370 to buy and $1,619 with the computer, close to R41's $1.5k. Shipping and duties are not included and are added at checkout. To ask the seller before ordering: whether each servo comes with its horn and screws, and whether the servos take a full 3S pack's 12.6 V (both listings give up to 12 V). Nothing else on the list is in its catalogue. Alternatives: the supplier listing and the retailer, as before.

## 2026-11-10: Marvin Bot's BOM workbook, website and motion pipeline

To match Marvin Bot, three of its pieces are carried over:

- **The BOM is a workbook** (`bom/bom.xlsx`, kept tidy by Marvin Bot's `tools/bom/book.py` without its Lite column and actuator-bolt sheet), in place of `bom/bom.yaml`: a Summary of formulas, one sheet per group with a status dropdown, a Sourcing sheet and a Made parts sheet that `python -m cad.export` fills from the CAD. Every status ships empty, as on Marvin Bot, so the total ($1,619) is the full cost to replicate.
- **The website** (`site/`): Marvin Bot's Astro Starlight site with the 3-D home page and the 3-D model page (layers, assembly order, scripted skills with servo loads, joints by hand), on the mini's model; the docs are synced from the repository's Markdown. Marvin Bot's interactive guide is linked rather than ported: its ideas apply, its numbers are Marvin Bot's.
- **The motion pipeline** (`sim/motion/`): the same 25 Kimodo motions retargeted to the mini and tracked by one ProtoMotions policy, checked in MuJoCo on the servo model (`sim/motion/README.md` says what changes for the mini).

## 2026-11-10: the order priced: $1,554 to buy, STS3250s from a supplier listing

Every line of `bom/bom.xlsx` priced from listings of 2026-11-10, with its source. The STS3250 costs $74-97 at retail, not the $45 estimated: at retail the order came to $2,002. They come from a supplier listing at about $46 instead, for $1,554 to buy and $1,803 with the computer, against R41's $1.5k. The rest came close to the estimates: STS3215-C018 $27.99, servo adapters $9.05, BNO085 IMU $29.50 (on the computer's UART, no microcontroller), wide-angle IMX219 camera $25.99, 3S 3.5 Ah packs $37.28 (167 g, under the 0.22 kg budgeted; no protection board stated, so a 3S BMS each), charger $59.95, buck-boost $8.90. Two spares of each servo, as planned. Shipping, duties and taxes are not included. Alternatives: retail STS3250s ($2,002), a bench pair first and the bulk after the bench fit, and fewer spares (about $50-100 less each).

## 2026-11-10: first walking policy (flat v3); the torque rule allows brief saturation when moving

Flat v2 (3000 iterations with the rescaled rewards) walked, turned and stopped in the MuJoCo cross-check, but at 4.5 steps a second with 10 cm steps and 1.5 cm of foot lift, its knees and hip pitch peaking at up to 171% of the torque rule's 60%-of-stall limit, and its knee at 109% of rated RMS when walking at 0.3 m/s. Flat v3 continued from v2 for 2000 iterations with a penalty on torque above the rule (`torque_over_rule`) and twice the foot-clearance weight: 2.3 steps a second, 19 cm steps, 3.4 cm of lift (target 3.5), knees at 60% of rated RMS walking at 0.2 m/s and 81% at 0.3 m/s. Its peaks still reach 126-183% of the rule, but for under 1% of the time and at most 20 ms at a stretch, at touchdowns and stops: a position servo saturates at stall whenever its target steps more than about 10° (0.97 duty at 32 × 0.166 per radian), which the model reproduces, capped at stall for the pack's voltage.

So the rule now has two parts (`robot.yaml` `torque_rule`): held poses keep the 60%-of-stall peak (the order gate), and moving policies may saturate briefly but stay above that limit no longer than 50 ms at a stretch and no more than 5% of a segment; RMS within rating applies to both. Alternatives: capping each servo's torque-limit register at about 60% and modelling it (the rule holds by construction, but a weaker robot for getting up and pushes) and keeping the strict peak with a stronger penalty and filtered targets (more training, likely a stiffer gait).

v3 passes the cross-check at 11.1 V with a 10 ms delay, on an empty pack with 20 ms, at full sensor noise, and walking at 0.3 m/s on an empty pack. On a full pack (12.6 V) with a 5 ms delay it walks 0.26 m/s for 0.20 and holds a knee above the rule for 56 ms: open.

## 2026-11-09: walking rewards rescaled for servos; flat v1 learned to fall

Marvin Bot's walking task carried over with its weights, and flat v1 (3000 iterations) learned to fall: every episode ended on tilt within 12.6 control steps (0.25 s), faster than random actions fall (33 steps). The heat penalty (`sustained_torque`, the running average of (torque / rated)² over 0.8) cost about 6 a second, against at most about 2.5 a second from command tracking, so ending the episode was the best return. A servo on a light limb saturates near stall, about three times its rated torque, whenever the policy jitters, so the term that only shaped Marvin Bot's stair climbing dominated here. Its weight is now a tenth of Marvin Bot's (−0.1 flat, −0.2 rough). Two other widths were still Max-sized: the speed-tracking width (0.25 m/s, as wide as the mini's whole command range, so standing still earned a third of the walking reward; now 0.1 m/s) and the foot-clearance width (scaled by (3.5 / 8)² with the 3.5 cm lift). Zero actions hold the stance in the training environment with the upper body moving (0 of 256 robots fell in 3 s).

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

The Orin Nano Super 8 GB runs a 50 Hz control loop outside ROS (servo I/O, ONNX policies, safety) and light vision (AprilTags, a ball or person detector). Training, teleoperation and any vision-language-action model stay on the desktop; a small (~0.5B) VLA onboard is a later experiment. A 3S Li-ion pack of about 3 Ah (35 Wh) feeds the 12 V servos directly through a fuse and the e-stop, with a buck-boost for the computer, for 25–40 minutes. A wide-angle CSI RGB camera sits on the pan-tilt neck; depth comes later if stairs or obstacles become goals. Alternatives: a tether until walking works (the robot then behaves differently untethered), 4S with a 12 V buck (needs a 20–30 A converter), a stereo depth camera (mass high up, cost).

## 2026-11-09: a printed frame

Everything is printed on a desktop FDM printer: PETG or PLA+ brackets and shells, nylon-carbon at the hips and knees, steel pins and small bearings on the far side of loaded joints, heat-set inserts. No mill. The stress check (copied from Marvin Bot) uses printed-material properties with a derating for layer direction. Alternatives: milled aluminium hip and pelvis plates (brings back the mill), bought aluminium servo brackets (vendor geometry, generic look).

## 2026-11-09: Feetech STS servos, 23 joints with Marvin Bot's names

Feetech STS3250 (4.9 N·m stall, 75 rpm, 75 g) on the twelve leg joints and STS3215 12 V (C018, 2.9 N·m stall, 45 rpm) on the arms, waist and neck: magnetic 12-bit encoders, a 1 Mbit/s bus, readable position, load, voltage and temperature, the same protocol as Marvin Bot's neck servos. The layout keeps Marvin Bot's joints except the wrists and hands: 6 per leg (roll-first hip, knee, ankle pitch and roll driven directly, no linkage), 4 per arm, waist yaw, 2 neck. The same names let the motion pipeline carry over. No hands in v1; a 1-DOF gripper may follow. Alternatives: Dynamixel X-series (a real current mode and less backlash, but 3–5 times the servo cost), STS3215 throughout (a 0.45 m robot with thin knee margins), potentiometer bus servos (low resolution and range), 27 joints with wrists (little gain without hands), 12 leg joints first (cheapest, but no upper-body skills).

## 2026-11-09: a separate repository, tools copied from Marvin Bot

Marvin Bot's Lite variant works because it is Max's upper body unchanged. The mini shares no parts, scale, actuators or fabrication with Max, so it is its own repository with the same layout and conventions, and Marvin Bot's tools are copied, trimmed and adapted for servos and printing, credited back. Alternatives: a third variant in Marvin Bot (most tools would need servo and print branches, and Max's CI would grow) and a shared tools package for both (cleaner long term, but means refactoring Marvin Bot first).

## 2026-11-09: built in stages, one order after the simulation check

As Marvin Bot, simulation comes first, but at about $1.4k the kit is bought in one order once phase 1's torque checks confirm the servo assignment, and the bench fit runs on the first servos to arrive. Nothing is ordered without the owner's OK.
