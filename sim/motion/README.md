# Motion pipeline: human motion to a Marvin Bot Mini tracking policy

Marvin Bot's motion pipeline ([its README](https://github.com/kranthie/marvin-bot/blob/main/sim/motion/README.md)
explains every step and why), run on the mini. The same 25 human motions generated with Kimodo
(locomotion, sidesteps, turns, a jog, a march, a kick, squatting and picking up, a bow, a dance
and arm gestures) are retargeted to the mini's size, and one BeyondMimic-style tracking policy
learns to follow all of them in ProtoMotions. A new skill is then a new clip, not new reward code.
The clips, with the reference as a ghost, are on the [skills page](../../docs/skills.md).

| Step | Tool | Here |
|---|---|---|
| Motions | [Kimodo](https://github.com/nv-tlabs/kimodo) text to motion, with a root path per clip | `generate_kimodo.py` (Marvin Bot's, unchanged) |
| Keypoints | ProtoMotions' `convert_soma23_npz_to_proto.py`, `extract_retargeting_input_keypoints_from_packaged_motionlib.py` | |
| Retarget | ProtoMotions' PyRoki retargeter | `retarget_mini.py` |
| Neck | the human's head orientation | `add_head_motion.py` |
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
- **Hands.** The mini has no wrists or hands, so the retargeter matches the human's wrist to a
  frame at the end of each forearm, and the tracking model carries a 1 g body there, which the
  tracker scores and the checks measure as the hand. The arms' joint angles are left out of the
  joint-angle term, as on Marvin Bot.
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
../venvs/kimodo/bin/python $M/sim/motion/generate_kimodo.py $W/npz locomotion skills
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

One tracker plays all 25 clips (the [skills page](../../docs/skills.md) has each one). It was
trained 2,400 epochs from scratch with the servo-load penalty at a weight of 0.1, then 1,000 more
warm-started from it at 0.5 (`--checkpoint`, `--training-max-iterations 1000`), 16,384 robots on
one GPU, about 3 hours in all. Every clip stays upright in Newton and in the MuJoCo check. In
MuJoCo, on the servo model:

| | No load penalty | Weight 0.1 | Weight 0.5 |
|---|---|---|---|
| Hip rolls standing (idle) | 4.4 N·m, 280% of rated | 1.45 N·m, 92% | 63% RMS |
| Hardest-working joint, worst clip (RMS / rated) | over 200% | 147% | 79% |
| Joint error, walk | | 20° | 8.7° |
| Hands, feet, walk | | 3.9, 4.0 cm | 3.6, 3.7 cm |

Penalising the load also tightened the tracking: with the legs no longer pressing against each
other the joints follow their targets more closely. What is still off:

- **Squat and pick-up** stop with the pelvis at 0.28 m against the reference's 0.15 m (and
  `replay_mujoco.py` flags them, since it counts 0.10 m below the reference as a fall); the knees
  reach stall on the way down. The torque check takes the robot down to a deep squat (knee 120°) within the
  torque rule, so the tracker may be
  short of what the servos allow; a longer run or a squat-weighted curriculum is next to try.
- **Sidesteps and walking back** cover 72-76% of the reference's distance.
- **Arm gestures** (arms raise, the two-hand wave, clap) put the hands 15 cm off: the arms' poses
  are scaled to a 0.24 m reach and the shoulder servos peak at stall.

## Data licences

As for Marvin Bot: motion data is not committed. Kimodo's models are under the NVIDIA Open Model
License (NVIDIA claims no rights in the motions they generate) and were trained on Bones Studio's
motion capture; its text encoder is Meta's Llama 3 8B Instruct under the Meta Llama 3 Community
License and the LLM2Vec adapters (MIT). Each user accepts those licences when downloading the
models; nothing from them is redistributed here. ProtoMotions' own sample motions (the BONES-SEED
dataset) are not used. ProtoMotions' and PyRoki's code is Apache-2.0 and MIT.
