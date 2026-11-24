# <div align="center">3DGS Mapper and Evaluation Design</div>

Status: **design, approved direction (2026-11-14), not implemented.** Scope: indoor room only.

This document covers the off-board 3D Gaussian Splatting (3DGS) mapper and how one codebase serves two modes:

- **Online:** on the lab's dual-4090 machine, closes the exploration loop.
- **Offline:** batch retrains on NRP that produce every reported metric.

For the research question and the fair-comparison rules, see [CLAUDE.md](../CLAUDE.md).

## 1. Decisions

| Decision | Choice | Why |
|---|---|---|
| Mapper | Custom mapper on [gsplat](https://github.com/nerfstudio-project/gsplat) (Apache-2.0) | Takes external poses (no tracking to turn off), we own the per-step iteration budget, Gaussian parameters are plain tensors, and gsplat is actively maintained. MonoGS (Imperial non-commercial licence, no modification without permission) and SplaTAM (Inria non-commercial rasterizer, file-only input) were rejected. GauSS-MI is ROS 1 and built on MonoGS, so it serves as the **reference implementation** for the uncertainty score, not as code we run. |
| Uncertainty | GauSS-MI-style per-Gaussian probability plus mutual information (MI) per candidate view, reimplemented on gsplat | The method the proposal cites, and it scores candidate views without rendering ground truth. FisherRF-style scoring is the fallback. |
| Pose source | GLIM 6-DoF everywhere: live GLIM online, **offline GLIM re-run** for every retrain | Rule 1. Running offline GLIM on every arm's own bag makes the pose estimator identical across arms. |
| Middleware | Fast DDS (Jazzy default), unchanged | Already working. Raw images never cross Wi-Fi in this design. |
| Metrics | Computed **only** from offline retrains | Wi-Fi drops and online compute limits can't affect the scores. |
| Platform | Ubuntu 24.04 + ROS 2 Jazzy on the Jetson and the lab machine. NRP images need no ROS | The offline retrain reads a ROS-free "run package" (§6). |
| Compute placement | **Online mapping on the lab's 4090s. Offline retrains on NRP A100s.** No real-time mapping on NRP | §6.2. A live session needs a GPU pod up and waiting for the rover the whole time. That breaks the A100 access terms (temporary pods only, ≤ 1 h each, no idle waiting) and NRP's utilization rules. It would also route DDS through the internet and Kubernetes networking. Online map quality isn't scored anyway; it only has to be good enough to plan with. The A100s' 80 GB earns its keep in the offline retrains that produce the metrics. |

## 2. Prerequisites in the onboard stack

| Item | Status |
|---|---|
| RGB-D registered pair: depth aligned to color on the device, `Sync` by device timestamp, capture-time stamps, `CameraInfo` from each frame's transformation, `16UC1` depth | **Done in `oakd_publisher` (2026-11-14), not yet run on hardware.** Verify with the checks in §2.1 |
| Exposure and white-balance lock (`rgb_exposure_us`, `rgb_white_balance_k`) | Parameters exist, default auto. Pick values per session (§7.2) |
| GLIM on live sensors indoors | **Passed** in manual teleop mapping (2026-11). Not yet used during autonomous exploration |
| `glim_map` ↔ `map` bridge, and GLIM poses in `keyframe_bridge` | Not started. Bridge approach approved (§4.3) |
| `keyframe_bridge` forwards `/oakd/color/camera_info` as `/keyframe/camera_info` | Not started |
| `exploration_msgs/ViewpointGoal` + onboard goal relay | Not started (§4.2) |
| Benchmark recording profile (§5.1) | Not started. `scripts/record.sh` already records the needed topics |

### 2.1 Hardware checks for the OAK-D change

1. The log shows `color intrinsics 1280x720 ...` and `depth (aligned) intrinsics 1280x720 ...` with identical numbers, and no "size differs" warning.
2. `ros2 topic hz` gives ~15 Hz on all four `/oakd/{color,depth}/{image_raw,camera_info}` topics. `ros2 topic echo --once /oakd/depth/image_raw --field encoding` prints `16UC1`.
3. **Alignment overlay:** save one color and one depth frame, colorize the depth, and blend them 50/50. Edges (door frames, table legs) must line up across the whole image, corners included. A mismatch only near the corners means the color undistortion and the reported `K` disagree. Then compare against `rgb_undistort:=false`, where `CameraInfo` carries the distortion model instead.
4. **Stamp check:** rotate the rover in place and watch RViz with fixed frame `map`. The depth point cloud (through `depth_image_proc`) should stay put on the walls, not swing with the turn.
5. Does the IR dot pattern appear in the color image with `ir_dot_projector_ma > 0`? If it doesn't, turn the projector on for runs (better depth on blank walls). If it does, leave it off.

## 3. Module layout

```
ros2_ws/src/exploration_msgs/            # built on BOTH machines (rover needs the message)
  msg/ViewpointGoal.msg
  msg/ExplorationStatus.msg
ros2_ws/src/robot_slam/...               # + goal relay node, + glim↔map tree bridge node (onboard)

ros2_ws_offboard/
  src/gs_explore/                        # ament_python package, also pip-installable for NRP
    gs_explore/
      core/                              # pure Python + PyTorch + gsplat; NO rclpy imports
        config.py                        # schema dataclasses, YAML load, ${ENV} expansion, validation, hash
        frames.py                        # Frame(stamp_ns, rgb, depth_m, K, T_world_cam)
        keyframes.py                     # the ONE keyframe-selection rule (both modes, all arms)
        gaussians.py                     # GaussianModel: parameter tensors, init from RGB-D, save/load .ply
        trainer.py                       # Trainer.step(n): the ONE training loop (losses, gsplat strategy)
        uncertainty.py                   # per-Gaussian information state + view MI score
        viewpoints.py                    # candidate generation in 2D free space, scoring, selection
        metrics.py                       # PSNR / SSIM / LPIPS, coverage
        determinism.py                   # seeding + deterministic flags
      online/
        mapper_node.py                   # rclpy node: ingest, cycle, publish ViewpointGoal
      offline/
        export.py                        # sensor bag + GLIM dump -> run package   (lab machine, needs ROS)
        align.py                         # ICP: run map -> held-out session map      (lab machine)
        retrain.py                       # run package -> model + metrics            (NRP, no ROS)
      cli.py                             # `gs-explore export|align|retrain|report`
    config/
      online.yaml
      offline.yaml
  docker/Dockerfile.retrain              # CUDA + PyTorch + gsplat + gs_explore[offline]; no ROS
  deploy/nrp/retrain-job.yaml            # Kubernetes Job template
```

**Rule:** `core/` is the only place the model, the losses and the keyframe rule live. Both modes call `Trainer.step(n)`; only the YAML differs. Nothing in `core/` knows which arm produced the data.

## 4. Online mode (lab machine, closed loop)

### 4.1 Data flow

```
ROVER (Jetson)                                      LAB MACHINE (2× RTX 4090)
oakd_publisher ─┐
GLIM ───────────┼─> keyframe_bridge ──/keyframe/{color,depth,pose,camera_info}──> mapper_node
                │      (GLIM 6-DoF pose)              best-effort, Wi-Fi            │ GPU0: Trainer.step
slam_toolbox ───┼───────────────────────────/map (occupancy grid)─────────────────> │ GPU1: uncertainty + scoring
                │                                                                    │
goal_relay <────┼──────────────────────────/exploration/viewpoint_goal───────────── ┘
  │ NavigateToPose                         /exploration/status ───────────────────>
  v
Nav2 (RPP, Ackermann)
```

### 4.2 Messages

`exploration_msgs/ViewpointGoal`:

```
std_msgs/Header header            # stamp = publish time; frame_id = "map" (slam_toolbox frame Nav2 uses)
uint32 goal_id                    # increments per goal
uint32 cycle                      # planning cycle index
geometry_msgs/Pose pose           # planar goal; z, roll, pitch ignored by Nav2
float32 score                     # MI of the chosen view
float32 runner_up_score
uint32 num_candidates
builtin_interfaces/Time newest_keyframe_stamp   # freshness of the map behind this decision
float32 plan_time_s               # train + score wall time this cycle
```

`exploration_msgs/ExplorationStatus` (rover → lab, 1 Hz and on change):

```
std_msgs/Header header
uint32 goal_id
uint8 state                       # IDLE=0, NAVIGATING=1, ARRIVED=2, FAILED=3, TIMED_OUT=4, ENDED=5
float32 distance_travelled_m      # odometry, for live monitoring only (reported path length is offline)
```

**goal_relay (onboard)** turns each `ViewpointGoal` into a `NavigateToPose` action call, the same way `frontier_explorer` does today. It reports progress on `/exploration/status`. If no new goal arrives within `goal_wait_timeout_s` after `ARRIVED` (Wi-Fi drop, mapper crash), it publishes `ENDED` and leaves the rover stopped. It never wanders. Localization and control never depend on the link, as the architecture requires.

### 4.3 Pose frames

The online map and the candidate poses live in GLIM's frame, but Nav2 navigates in slam_toolbox's `map`. A new onboard `tree_bridge` node publishes a TF edge `map → glim_map`. It takes the two pose estimates of the robot at the same timestamp, `T_map_base(t)` from the robot tree and `T_glimmap_base(t)` from GLIM's tree (through `glim_lidar` and the URDF's `base_link → livox_frame`), and computes `T_map_glimmap = T_map_base · T_glimmap_base⁻¹`. The result is low-pass filtered (the two SLAM systems drift differently) and published at 10 Hz.

`glim_map` has no parent today, so this joins the two trees without giving `odom` or `base_link` a second parent. `keyframe_bridge` then computes the camera pose as `T_glimmap_cam = T_glimmap_lidar(t) · T_lidar_cam`. `T_lidar_cam` comes from `/tf_static`: `livox_frame → base_link → oakd_link → oakd_camera_optical_frame`.

### 4.4 Planning cycle (stop-and-go, ~30–90 s)

1. The rover reaches its goal (`ARRIVED`) and holds still.
2. `mapper_node` takes every keyframe received since the last cycle and adds new Gaussians, initialized from each keyframe's depth (`core/gaussians.py`).
3. `Trainer.step(train.iters_per_cycle)` runs on GPU0. A wall-clock guard (`train.cycle_time_limit_s`) cuts the cycle short if it runs over and logs that it did.
4. GPU1 copies the Gaussian tensors and updates the per-Gaussian information state.
5. **Candidate generation:** sample poses in known-free cells of `/map`, inflated by the robot footprint, at `planner.candidate_spacing_m`, with `planner.headings` yaw values each. Drop candidates closer than `planner.min_goal_separation_m` to recently visited goals. The OAK-D's height and pitch are fixed, so candidates are planar.
6. **Scoring:** render each candidate at `planner.render_downscale` resolution and compute the GauSS-MI-style score. Optionally divide by an estimated path cost (`planner.distance_weight`).
7. Publish the best candidate as a `ViewpointGoal`. Log everything (§5.2).
8. **Termination:** stop at `mission.time_budget_s`, or after `mission.low_score_cycles` cycles in a row below `mission.min_score`.

The frontier arm runs the same `mapper_node` with `planner.publish_goals: false`. It still trains and scores, so the decision bag shows what the uncertainty planner would have chosen. Goals come from `frontier_explorer` as today.

### 4.5 GPU use

GPU0 trains, GPU1 renders candidates. Device indices come from the config (`devices.train`, `devices.score`). A single-GPU machine works by setting both to `cuda:0`.

## 5. Logging

Each run produces two bags with the same `run_id`, written into the bag metadata and the folder name (`<date>_<arm>_<nn>`).

### 5.1 Sensor bag, recorded on the Jetson's NVMe

This is the **only input** to the offline retrain, so a Wi-Fi drop can't change what a policy gets scored on.

| Topic | Purpose |
|---|---|
| `/livox/lidar`, `/livox/imu` | Offline GLIM re-run → poses, path length, run map |
| `/oakd/color/image_raw`, `/oakd/depth/image_raw`, `/oakd/{color,depth}/camera_info` | Training frames (registered RGB-D) |
| `/tf_static` | `T_lidar_cam` and the URDF mount offsets |
| `/tf`, `/vesc_odom`, `/oakd/imu` | Debugging; the online-pose comparison |
| `/mission/control`, `/exploration/status`, `/exploration/viewpoint_goal`, `navigate_to_pose/_action/status` | Exploration start and end times, goal outcomes |

**Rate and size:** for benchmark sessions, launch the OAK-D at `fps:=5`. Raw 1280×720 bgr8 plus 16UC1 depth is 4.5 MB per pair, ≈ 23 MB/s, or about 14 GB per 10-minute run before MCAP zstd. That's within NVMe bandwidth and needs no new code. Hardware JPEG on the OAK-D (`VideoEncoder`) can come later if needed. Check frame counts after every run with `ros2 bag info`.

**Clock:** the Livox driver stamps packets with the host clock on arrival when PTP isn't set up ([`pub_handler.cpp`](../ros2_ws/src/livox_ros_driver2/src/comm/pub_handler.cpp)). The OAK-D stamps are host-synced capture times. LiDAR poses and images therefore share one clock, with a few ms of LiDAR network latency.

### 5.2 Decision bag, recorded on the lab machine

`/keyframe/*` (what actually arrived), `/exploration/*`, and `/gs_explore/cycle` (per cycle: keyframe count, Gaussian count, iterations run, train and score time, all candidate poses and scores). Every `online.snapshot_every_cycles` cycles, the mapper also writes a map snapshot `.ply` beside the bag.

Latency (capture stamp → arrival → goal publish) is computed from this bag, which addresses the proposal's "communication latency" gap.

## 6. Offline mode

### 6.1 Pipeline

```
sensor bag ──[export, lab machine, ROS]──> run package ──[align, lab machine]──> + alignment.yaml
                                               │
                                               └──(copy to NRP storage)──> [retrain Job × seeds, NRP] ──> results
```

**export** (`gs-explore export --bag <bag> --glim-config <dir> --out <runs_root>`) runs once per bag:

1. Run `glim_rosbag` with the pinned **offline GLIM config** (stock quality values, see [glim.md](glim.md) §4). Save `traj_lidar.txt` and export the map `.ply`.
2. For each color/depth pair in the bag, interpolate `T_glimmap_lidar` at the image stamp (linear translation, SLERP rotation), then compose with `T_lidar_cam`. Drop frames more than `export.max_pose_gap_s` from a trajectory sample, and frames outside the exploration window (mission `start` → `end`).
3. Write the run package. Images stay lossless: color as PNG unless the bag already holds JPEG (then copy the bytes as-is), depth as 16-bit PNG in mm.

**Run package (the bag → Job interface):**

```
<runs_root>/<run_id>/
  manifest.yaml        # schema_version, run_id, arm (frontier|nbv|heldout), session_id,
                       # bag sha256, GLIM version + config hash, exporter git sha,
                       # exploration_start_ns, exploration_end_ns, frame count
  intrinsics.yaml      # width, height, K, distortion: none
  frames.csv           # idx, stamp_ns, rgb, depth, tx ty tz qx qy qz qw (T_world_cam, GLIM frame)
  rgb/000000.png ...
  depth/000000.png ...
  trajectory.txt       # GLIM traj_lidar (TUM) → path length
  map.ply              # GLIM map → alignment, coverage reference
  alignment.yaml       # written by `align`: T_heldout_run, ICP fitness, RMSE
```

The run package holds every frame. **Keyframe selection runs inside the retrain** with the config's rule, so all arms get identical selection.

**align** (`gs-explore align --run <run> --reference <heldout_run>`): point-to-plane ICP of `map.ply` against the held-out session's map. The initial guess is identity, since every run uses the same start pose by rule 3. The step fails loudly if fitness < `align.min_fitness` or RMSE > `align.max_rmse_m`.

**retrain** (`gs-explore retrain --config offline.yaml --run <run> --heldout <heldout_run> --out <dir> (--seed <s> | --seed-from-env JOB_COMPLETION_INDEX)`):

1. Load and validate the config, then record its sha256. Seed everything (§6.3).
2. Select keyframes (`keyframes.*`), initialize Gaussians from their depth, and run exactly `train.iters` iterations. No early stopping.
3. Save `point_cloud.ply`.
4. Evaluate. For each held-out view:
   - Map its pose into the run's frame through `alignment.yaml`.
   - Refine the pose for `eval.pose_refine_iters` with the Gaussians frozen (the same for every arm).
   - Render, then compute PSNR, SSIM and LPIPS.
5. Compute coverage (§7.3), path length and exploration time.
6. Write the results, then the `DONE` marker last, and exit 0. Any failure exits non-zero and writes no `DONE`.

**Results layout** (one directory per run and seed):

```
<results_root>/<run_id>/seed_<s>/
  config.resolved.yaml   config_sha256.txt   env.json   # git sha, torch/gsplat/CUDA versions, GPU name
  point_cloud.ply
  metrics.json           # psnr/ssim/lpips mean+std, coverage, path_length_m, exploration_time_s, iters, train_time_s
  per_view.csv           # per held-out view metrics + refined pose delta
  train_log.csv
  DONE
```

**Idempotent:** if `DONE` exists, the job exits 0 right away. Results are written to `<dir>.tmp` and renamed when finished, so a preempted pod can't leave half-written results.

### 6.2 NRP Job

Template: [`ros2_ws_offboard/deploy/nrp/retrain-job.yaml`](../ros2_ws_offboard/deploy/nrp/retrain-job.yaml). It's one **Indexed Job per run**, where the completion index selects the seed.

**Project rules for NRP.** These are the conditions of our A100 access, and they bind every manifest, script and person using the namespace:

1. **No `sleep`, ever.** Not in the container command, not in a wrapper script, not to "keep a pod up for debugging".
2. **Temporary pods only.** Every pod comes from a Job, does its work and exits. No interactive pods, no Deployments, no `kubectl exec` sessions into running work, no pod waiting for data or input.
3. **One hour per pod at most.** The template sets pod-level `activeDeadlineSeconds: 3600`. Work that can't finish in an hour is split into stages that checkpoint to the PVC (for example `train` then `eval`), each a separate Job whose pod also stays under 1 h. Never raise the deadline.
4. **Clean up.** `ttlSecondsAfterFinished` deletes finished Jobs and pods. Results live on the PVC, not in pods.
5. **Debug locally, not on NRP.** Retrains are debugged on the lab machine with the same container image and CLI. NRP only runs finished, non-interactive jobs.

**How the template applies them:**

- The container command is the retrain CLI and nothing else.
- **Two timeouts:** the pod-level deadline (1 h) and `train.wall_clock_limit_s` (45 min, leaving ~15 min for loading and evaluation). Hitting either is a **failure** (exit non-zero), never a quietly shorter run. A Job-level `activeDeadlineSeconds` caps the whole Job including retries.
- **Resources:** 1 GPU per pod, since gsplat training is single-GPU. On NRP, A100s are a special resource (`nvidia.com/a100`), not `nvidia.com/gpu`, and are gated by a per-namespace quota. The template takes the resource name from `GPU_RESOURCE`. CPU and memory limits stay within 20% of requests (NRP policy). NRP flags GPUs used below 40%, so frames are decoded once into host RAM to keep data loading ahead of training.
- **One settings file, one command.** All NRP settings live in `ros2_ws_offboard/deploy/nrp/nrp.env`: namespace, kube context, image, PVC, GPU resource and product, CPU and memory, default seed count, Job deadline, config path. Create it by copying `nrp.env.example`; it is git-ignored, like `docker/.env`. Then submit:

  ```bash
  scripts/nrp_retrain.sh <run_id> --heldout <heldout_run_id> --dry-run   # render + check only
  scripts/nrp_retrain.sh <run_id> --heldout <heldout_run_id>             # submit
  ```

  The script fills the template, saves the rendered manifest to `/tmp/rover/nrp/`, and **refuses to submit** anything that breaks the rules above. It checks for: a Job (not a bare pod), `restartPolicy: Never`, a pod deadline ≤ 3600 s, no `sleep` in any command, no stdin/tty, exactly 1 GPU, and limits within 20% of requests. The offline config is uploaded as a ConfigMap named after its sha256, so editing `offline.yaml` can never change a Job that's already running. The 1 h pod limit is deliberately not a setting.
- **Same GPU product for every benchmark retrain:** NRP has more than one A100 variant (e.g. SXM4 80 GB), so pin one via `GPU_PRODUCT` per benchmark batch. It is recorded in `env.json`.
- **Retrain time on an A100 is unmeasured.** The first pilot retrain sets `train.iters` so training fits in 45 min with margin.

### 6.3 Determinism

- Seed Python, NumPy and torch (CPU and CUDA). Set `torch.use_deterministic_algorithms(True, warn_only=True)` and `CUBLAS_WORKSPACE_CONFIG=:4096:8`. Shuffle data with a seeded `torch.Generator`. Train for a fixed iteration count, not a wall-clock budget.
- **Bit-exact reruns are not expected.** CUDA splatting rasterizers usually accumulate gradients with floating-point atomics, so the same seed may produce slightly different results; this is unconfirmed for gsplat and will be measured. The protocol: **3 training seeds per run**, report mean ± std, and measure the noise floor once by training the same run with the same seed twice. A difference between arms smaller than that noise floor doesn't count as a result.
- The analysis script refuses to compare results whose `config_sha256` differ.

## 7. Evaluation protocol

### 7.1 Session structure

One session is one block of time with unchanged lighting and furniture and no people in the room:

1. Warm up the camera, then lock exposure and white balance (§7.2).
2. **Held-out pass:** teleop a slow loop of the room, covering places neither policy is guaranteed to visit. This is recorded with the same sensor bag profile and has arm `heldout`.
3. Exploration runs, alternating arms (frontier, nbv, frontier, nbv, …), each from the same marked start pose with the same `mission.time_budget_s`.

### 7.2 Exposure lock

At the start of each session, run auto-exposure for ~10 s while pointed at the room's average scene. Read the settled exposure time, ISO and color temperature from the frame metadata, then relaunch `oakd_publisher` with those as `rgb_exposure_us`, `rgb_iso` and `rgb_white_balance_k` for every pass in the session. Logging the settled values needs a small addition to `oakd_publisher`. `ImgFrame` exposes `getExposureTime()`, `getSensitivity()` and `getColorTemperature()`.

### 7.3 Metrics (offline retrains only)

| Metric | Definition |
|---|---|
| PSNR, SSIM, LPIPS | On the held-out views after pose refinement. LPIPS uses AlexNet, the usual 3DGS convention. Mean ± std over views, then over seeds |
| Observed coverage vs time | Voxelize the held-out session's GLIM map at `eval.coverage.voxel_m` inside the room bounds. A voxel counts as observed once a training frame's depth (≤ `depth_max_m`) lands in it. This gives a curve over exploration time, and its area under the curve. It depends on the policy only, not the splat |
| Reconstruction completeness | Share of reference voxels within `eval.coverage.surface_dist_m` of a Gaussian with opacity ≥ `eval.coverage.min_opacity` |
| Path length | Summed from the offline GLIM trajectory between mission start and end (the same estimator for every arm) |
| Exploration time | Mission `start` → `end`, or budget exhausted |

## 8. Config schema

One YAML per mode, both under `ros2_ws_offboard/src/gs_explore/config/`. Shared sections have the same meaning in both. Paths never appear in the configs; they come from CLI arguments. Strings may use `${ENV_VAR}`. `schema_version` is checked at load time.

| Section | Online | Offline | Notes |
|---|---|---|---|
| `schema_version`, `mode`, `seed` | ✓ | ✓ | `seed` in offline mode is overridden by `--seed` |
| `devices` | train, score | train | e.g. `cuda:0` |
| `camera` | ✓ | ✓ | depth scale, depth min/max, image downscale |
| `keyframes` | ✓ | ✓ | the one selection rule |
| `model` | ✓ | ✓ | SH degree, init stride, max Gaussians |
| `strategy` | ✓ | ✓ | `default` or `mcmc`, with gsplat strategy parameters |
| `train` | `iters_per_cycle`, `cycle_time_limit_s` | `iters`, `wall_clock_limit_s` | loss weights, learning rates |
| `uncertainty`, `planner`, `mission`, `ros`, `online` | ✓ | | |
| `export`, `align`, `eval` | | ✓ | |

See the two files for every key and its default.

## 9. Validation plan

1. **OAK-D:** the §2.1 checks.
2. **Offline first:** export a teleop bag and retrain it at a small budget. Check that the PSNR on its own training views rises and that renders look right.
3. **Noise floor:** the same run, same seed, trained twice, then 3 seeds.
4. **Uncertainty spike:** reimplement the GauSS-MI score on gsplat. Compare its candidate ranking against the GauSS-MI reference code on their example rosbag (rank correlation).
5. **Online dry run:** teleop with the `mapper_node` publishing goals that aren't executed. Check cycle times and goal sanity.
6. **Closed loop**, then benchmark sessions.

## 10. Open questions

1. **NRP access.** Namespace name, storage (a CephFS PVC vs NRP's S3), and how run packages get from the lab machine to NRP.
2. **Which A100 variant** to pin for the benchmark batch (`kubectl get nodes -L nvidia.com/gpu.product` lists them). It must be the same for every retrain.
3. **Budgets.** The offline `train.iters` (draft: 30k, must fit 45 min on the pinned A100) and the online `iters_per_cycle`, both set after pilot runs.
4. **Runs per arm and time budget.** Draft: 3 physical runs per arm per session, 10 min each, 3 training seeds per run.
5. **Frontier baseline pacing.** It drives continuously while the NBV arm stops to plan, and both count toward exploration time. Keep the baseline as built (draft), or add matching pauses?
6. **Arm B (3D frontier):** still pending advisor approval. If it goes ahead, it reuses all of this unchanged.
7. **Candidate source.** Planar candidates from slam_toolbox's `/map` free space (draft) vs candidates from GLIM's 3D map.
8. **IR dot projector** on or off for benchmark runs, decided by check §2.1-5.

**Resolved (2026-11-23):**
- GLIM passed live indoor teleop mapping, so the online mapper uses GLIM poses from the start. No slam_toolbox fallback.
- The `map → glim_map` TF bridge (§4.3) is approved.
- Compute placement: online on the lab 4090s, offline retrains on NRP A100s, under the rules in §6.2.
