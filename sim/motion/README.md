# Motion pipeline: human motion to a Marvin Bot Mini tracking policy

Marvin Bot's motion pipeline ([its README](https://github.com/kranthie/marvin-bot/blob/main/sim/motion/README.md)
explains every step and why), run on the mini. The same 25 human motions generated with Kimodo
(locomotion, sidesteps, turns, a jog, a march, a kick, squatting and picking up, a bow, a dance
and arm gestures), and five of its own for the wrists and pincers (pouring, drinking, a door
knob, a light bulb, handing over), are retargeted to the mini's size, and one BeyondMimic-style
tracking policy learns to follow all of them in ProtoMotions. A new skill is then a new clip, not new reward code.
The clips, with the reference as a ghost, are on the [skills page](../../docs/skills.md).

| Step | Tool | Here |
|---|---|---|
| Motions | [Kimodo](https://github.com/nv-tlabs/kimodo) text to motion, with a root path per clip | `generate_kimodo.py` (Marvin Bot's, unchanged) |
| Keypoints | ProtoMotions' `convert_soma23_npz_to_proto.py`, `extract_retargeting_input_keypoints_from_packaged_motionlib.py` | |
| Retarget | ProtoMotions' PyRoki retargeter | `retarget_mini.py` |
| Neck | the human's head orientation | `add_head_motion.py` |
| Wrists, grip | the human's hand twist; a grip schedule per clip | `add_hand_motion.py` |
| Convert, package | ProtoMotions' `convert_pyroki_retargeted_robot_motions_to_proto.py`, `motion_lib.py` | `fix_sole_height.py` |
| Track | [ProtoMotions](https://github.com/NVlabs/ProtoMotions) on Newton, BeyondMimic-style rewards | `setup_protomotions.py`, `check_tracking.py` |
| Check | the tracker in MuJoCo on the servo model | `replay_mujoco.py` |
| Clips | renders with the reference as a ghost | `render_tracking.py`, `check_clips.py` |

## What changes for the mini

Everything that is not robot-specific is Marvin Bot's (the tracker's rewards, its terminations,
the contact penalty, the root paths, the head and sole fixes). What changes:

- **Scale.** `retarget_mini.py` scales the human keypoints to the mini: the legs by 0.28 (its
  0.24 m straight leg over the source's 0.853 m), the shoulders 0.37 as wide (its 0.19 m shoulder
  span over the source's 0.51 m) and 0.39 as high above the hips, the arms by 0.44 about each
  shoulder (its 0.24 m reach), the shoulders 2.4 cm forward. Scaled down, the human's feet end up
  about 5 cm apart against the mini's 9 cm hips, so the feet move out 2 cm each and the knees 1 cm.
  Marvin Bot's figures are 0.69, 0.62, 0.79 and 6.5 cm. The speeds scale with the size, so a
  1.2 m/s human walk becomes about 0.34 m/s.
- **Hands.** The retargeter matches the human's wrist to the mini's wrist (the wrist servo's
  output face), and the tracking model carries a 1 g body there on the palm, which the tracker
  scores (its position and, as it turns with the wrist, its orientation) and the checks measure
  as the hand. The shoulder and elbow angles are left out of the joint-angle term, as on Marvin
  Bot; the wrists and pincers are scored by it.
- **Wrists and grip.** No keypoint fixes a wrist's turn or a grip, so the retargeter holds them and
  `add_hand_motion.py` sets them: each wrist from the person's hand twist against the forearm,
  less its idle average; the pincer from a schedule per clip (Kimodo's hands never open or close):
  open on the way to an object, closed on it while held, open to let go. Five clips were added for
  them (`generate_kimodo.py`'s wrist group): pouring, drinking, turning a door knob, screwing in
  a light bulb overhead and handing something over.
- **Servos.** ProtoMotions drives joints with its built-in PD. At small errors the servo model
  (`sim/servo_model.py`) is a PD too: stiffness kt × firmware_kp × error_gain × v / R (25 N·m/rad
  for an STS3250, 15 for an STS3215 at 11.1 V), damping kt² / R (0.62 N·m·s/rad), torque capped at
  stall, armature 0.02 kg·m², and that is what `setup_protomotions.py` gives it. BeyondMimic's
  action scale (0.25 × effort / stiffness) would then be 0.046 rad; Marvin Bot softened its gains
  to raise it, but the servo's stiffness is its firmware's, so here the action scale is raised to
  0.4 rad instead and the gains are left as they are.
- **Servo load.** A penalty on each joint's torque beyond half its servo's rated torque (the
  joint PD's torque, worked out from the target, since the simulator's built-in PD does not
  report it). Without it the first tracker pressed the legs together, both hip rolls held 10°
  past where they stood, 4.4 N·m each standing still: near stall and 2.8 times rated, which
  would overheat them in minutes. Nothing in BeyondMimic's rewards sees torque, and a stiff
  stance costs nothing in simulation.
- **The MuJoCo check** (`replay_mujoco.py`) drives each joint with the full servo model (the duty
  cycle saturating at large errors, the back-EMF, the pack voltage, a command delay), which the
  PD in training only approximates.

## Environments

As for Marvin Bot: one parent folder outside this repo with the ProtoMotions, PyRoki and Kimodo
checkouts, three Python environments and a work folder; Kimodo's text encoder is gated (Meta
Llama 3 8B Instruct). The pipeline was run on ProtoMotions
`f69fdcaabff31846b4d2247207417b57dc788828`, PyRoki `388e43e1fc0d0ee382968d3dd72970fd62a0450c`
and Kimodo `e1c219c99b4b94ba48be1fb6a1cbc64ecee4c954`; the setup commands are in
[Marvin Bot's README](https://github.com/kranthie/marvin-bot/blob/main/sim/motion/README.md#environments).
`setup_protomotions.py` edits the ProtoMotions checkout (it adds five generated files and
registers `marvin_bot_mini` in `robot_configs/factory.py`), so rerun it after updating
ProtoMotions or the description.

## Steps

From the ProtoMotions checkout (`P=../venvs/proto/bin/python; R=../venvs/pyroki/bin/python`;
`$M` is this repo, `$W` the work folder):

```bash
../venvs/kimodo/bin/python $M/sim/motion/generate_kimodo.py $W/npz locomotion skills wrist
$P $M/sim/motion/setup_protomotions.py .
$P data/scripts/convert_soma23_npz_to_proto.py --input-dir $W/npz --output-dir $W/soma \
    --input-fps 30 --output-fps 30 --ignore-motion-filter
$P protomotions/components/motion_lib.py --motion-path $W/soma --output-file $W/soma.pt
$P data/scripts/extract_retargeting_input_keypoints_from_packaged_motionlib.py $W/soma.pt \
    --output-path $W/keypoints --skeleton-format soma --start-idx 0 --skip-freq 1
for f in $W/keypoints/*.npy; do
  d=$W/keypoints_split/$(basename $f .npy); mkdir -p $d; cp $f $d/
  ( $R $M/sim/motion/retarget_mini.py --protomotions . --keypoints-folder-path $d \
      --output-dir $W/retargeted --source-type soma --no-visualize &&
    $R $M/sim/motion/retarget_mini.py --protomotions . --keypoints-folder-path $d \
      --output-dir $W/retargeted --source-type soma --no-visualize \
      --save-contacts-only --contacts-dir $W/contacts ) &
done; wait
$P $M/sim/motion/add_head_motion.py $W/npz $W/retargeted \
    protomotions/data/assets/urdf/for_retargeting/marvin_bot_mini.urdf
$P $M/sim/motion/add_hand_motion.py $W/npz $W/retargeted \
    protomotions/data/assets/urdf/for_retargeting/marvin_bot_mini.urdf
$P data/scripts/convert_pyroki_retargeted_robot_motions_to_proto.py \
    --retargeted-motion-dir $W/retargeted --output-dir $W/proto --robot-type marvin_bot_mini \
    --contact-labels-dir $W/contacts --apply-motion-filter --force-remake
$P $M/sim/motion/fix_sole_height.py . $W/proto
$P protomotions/components/motion_lib.py --motion-path $W/proto --output-file $W/mini_motions.pt
$P protomotions/train_agent.py --robot-name marvin_bot_mini --simulator newton \
    --experiment-path examples/experiments/mimic/mlp_bm_marvin_bot_mini.py \
    --experiment-name mini_track --motion-file $W/mini_motions.pt \
    --num-envs 16384 --batch-size 65536 --headless
$P $M/sim/motion/check_tracking.py \
    results/mini_track/results/predicted_motion_lib_epoch_<N>.pt $W/mini_motions.pt
```

Then the MuJoCo check and the clips (`pip install onnx onnxruntime` in the ProtoMotions
environment first):

```bash
$P deployment/export_bm_tracker_onnx.py --checkpoint results/mini_track/last.ckpt --output $W/onnx
$P $M/sim/motion/replay_mujoco.py $W/onnx/unified_pipeline.onnx $W/mini_motions.pt $W/mujoco.pt
$P $M/sim/motion/check_tracking.py $W/mujoco.pt $W/mini_motions.pt
$P $M/sim/motion/render_tracking.py export <Newton rollout> $W/mini_motions.pt $W/v.npz
MUJOCO_GL=egl python sim/motion/render_tracking.py render $W/v.npz docs/media/skills   # this repo's env
```

The human clips were checked before Marvin Bot was trained on them (`check_clips.py human`; all 25
pass); `check_clips.py robot` reruns the skill checks on the mini's references.

## Results

One tracker plays all 30 clips (the [skills page](../../docs/skills.md) has each one). It was
trained 3,000 epochs from scratch with the servo-load penalty at a weight of 0.5, then 600 more
warm-started from it on the strengthened frame's mass (3.59 kg; `--checkpoint`,
`--training-max-iterations 600`), 16,384 robots on one GPU, about 5 hours in all. A warm start
counts its epochs from zero, so `--training-max-iterations` is the number still to run; the long
run was resumed once that way after a GPU timeout on a machine also driving a display. Every clip
stays upright in Newton; in the MuJoCo check on the servo model, 29 of 30 do:

| | 25 clips, weight 0.1 then 0.5 | 30 clips, final mass |
|---|---|---|
| Hardest-working joint, worst clip (RMS / rated) | 79% | 86% (arms raise) |
| Joint error, walk | 8.7° | 16.2° |
| Hands, feet, walk | 3.6, 3.7 cm | 4.0, 3.5 cm |
| Hands, wrist clips | | 3.0-8.6 cm |

The earlier, 25-clip tracker's development showed what the load penalty does: without it the
tracker held both hip rolls at 4.4 N·m standing still (280% of rated), and with it the legs stopped
pressing against each other and the joints followed their targets more closely. What is still off:

- **The two-hand wave falls in MuJoCo** in its first second: the policy lets its feet drift
  together and the soles catch, with noise and command delay off too. It holds in Newton, and the
  25-clip tracker held it in MuJoCo, so this is a gap between the simulators that the five new
  clips opened; domain randomisation of the feet's friction and the servo gains is the next thing
  to try.
- **Squat and pick-up** stop with the pelvis at 0.27 m against the reference's 0.15 m (and
  `replay_mujoco.py` flags them, since it counts 0.10 m below the reference as a fall). The torque
  check takes the robot down to a deep squat (knee 120°) within the torque rule, so the tracker
  may be short of what the servos allow; a squat-weighted curriculum is next to try.
- **Sidesteps and walking back** cover 43-55% of the reference's distance.
- **Arm gestures** (arms raise, clap) put the hands 12-13 cm off, and the overhead bulb 9 cm: the
  arms' poses are scaled to a 0.24 m reach.

## Data licences

As for Marvin Bot: motion data is not committed. Kimodo's models are under the NVIDIA Open Model
License (NVIDIA claims no rights in the motions they generate) and were trained on Bones Studio's
motion capture; its text encoder is Meta's Llama 3 8B Instruct under the Meta Llama 3 Community
License and the LLM2Vec adapters (MIT). Each user accepts those licences when downloading the
models; nothing from them is redistributed here. ProtoMotions' own sample motions (the BONES-SEED
dataset) are not used. ProtoMotions' and PyRoki's code is Apache-2.0 and MIT.
