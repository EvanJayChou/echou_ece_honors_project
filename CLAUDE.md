# CLAUDE.md

ECE Honors project (FA26 → WI27, advisors Prof. Silberman and Prof. Nguyen): **Uncertainty-Guided Autonomous Exploration via Online 3D Gaussian Splatting** on an Ackermann rover (Jetson Orin Nano, Livox Mid-360, OAK-D Pro, VESC). Original proposal: [docs/proposal.pdf](docs/proposal.pdf). This file records the plan as refined since then; where they differ, this file is current.

## Research question

We benchmark **exploration policies**, not SLAM systems. The arms differ only in *which map drives goal selection*; everything else is held fixed.

| Arm | Goal selection driven by | Status |
|-----|--------------------------|--------|
| A. 2D frontier (baseline) | slam_toolbox occupancy grid (`/map`) | Implemented: `ros2_ws/src/frontier_explorer` |
| B. 3D frontier (extension) | GLIM 3D point-cloud map (voxel frontiers / unseen 3D regions) | Not started. Added scope beyond the proposal; confirm with Prof. Silberman before building |
| C. Uncertainty-guided NBV (contribution) | Per-Gaussian reconstruction uncertainty of the live splat (GauSS-MI-style) | Q2 (WI27) |

SLAM and 3DGS are not competitors: SLAM supplies poses, and 3DGS consumes them to build the splat. Don't frame results as "GLIM vs. 3DGS" or "SLAM Toolbox vs. 3DGS".

### Fair-comparison rules (apply to every arm)

1. **Same localization in every arm.** Planned pose source for keyframes: GLIM 6-DoF. The EKF runs `two_d_mode`, so its camera poses are planar (fixed z, roll and pitch), which is poor for 3DGS.
2. **The 3DGS mapper runs in every arm**, even when it doesn't pick goals, because the headline metrics need a splat.
3. **Same rover, Nav2 stack (Regulated Pure Pursuit, Ackermann), room, start pose and time budget.** Exploration goals reach Nav2 as ordinary goal poses; no custom motion planner.
4. **Metrics:** coverage efficiency (explored area/volume vs. time and distance), reconstruction quality (PSNR on held-out views, geometric accuracy). PSNR doesn't apply to GLIM's point cloud, so a GLIM-map vs. splat geometry comparison is at most a secondary result.

## Architecture

The design decision behind the split (and part of the honors design requirement): **only the compute-heavy 3DGS work leaves the rover**, so a Wi-Fi drop can't break localization or control.

| Where | Runs |
|-------|------|
| Onboard (Jetson) | Sensor drivers, EKF (`robot_localization`), slam_toolbox, GLIM, Nav2 (planner + controller), exploration target manager (frontier → later NBV goals), `keyframe_bridge` |
| Off-board (GPU machine) | 3DGS mapper, uncertainty estimator, NBV selection, evaluation harness |

**Migration status (2026-11-13):** slam_toolbox, the full Nav2 stack and `frontier_explorer` now run on the Jetson by default (`onboard.launch.py use_nav2:=true use_explorer:=true`, launched by `scripts/explore.sh`). `offboard_nav` was removed; `ros2_ws_offboard/` is empty and reserved for 3DGS. Off-board SLAM remains opt-in: `--offboard-slam` on the rover scripts plus `scripts/offboard.sh slam`, which reads `robot_slam/config/slam_toolbox.yaml` (the only copy). **Not yet run on hardware.** Remaining:
- First on-Jetson run of `scripts/explore.sh`: check that all Nav2 lifecycle nodes reach `active`, then send a manual 2D Goal Pose before starting the explorer.
- Measure memory with GLIM + slam_toolbox + Nav2 + explorer together (`jtop`/`tegrastats` on the host). GLIM alone peaked at 3.1 GB of 7.3 GB on the demo bag. If it doesn't fit, move GLIM off-board, not SLAM.
- Update `docs/software_systems.md` and `docs/full_stage_testing.md` to match.

### TF trees

```
robot: map ─(slam_toolbox)→ odom ─(ekf_local)→ base_link ─(URDF)→ livox_frame, oakd_link, oakd_camera_optical_frame
GLIM:  glim_map → glim_odom → glim_imu → glim_lidar   (separate tree on purpose; see docs/glim.md "TF frames")
```

`keyframe_bridge` currently looks up `map → oakd_camera_optical_frame`, so it uses slam_toolbox/EKF poses. Using GLIM poses for keyframes (rule 1) needs a bridge between the trees or GLIM re-framed as described in docs/glim.md. Never give `odom` or `base_link` two TF parents.

## Roadmap

**Q1 (FA26): mapping + baseline**
- [x] Onboard stack on Orin Nano / JetPack 7.2 / Ubuntu 24.04 / ROS 2 Jazzy, in Docker
- [x] Frontier-exploration baseline (arm A)
- [x] GLIM integrated (validated on a demo bag only)
- [ ] GLIM on live sensors: indoor closed-loop test (docs/glim.md §5 "Still to do")
- [x] Move SLAM/Nav2/explorer onboard (code + docs; hardware run pending)
- [ ] GLIM 6-DoF poses → `keyframe_bridge`
- [ ] Launch `keyframe_bridge` from the scripts and stream posed RGB-D keyframes off-board
- [ ] Online 3DGS mapper on the GPU machine; validate a live splat while teleoperating
- [x] RViz config for GLIM output: `robot_slam/rviz/glim.rviz` (fixed frame `glim_map`, `/glim_ros/map` with Transient Local durability)

**Q2 (WI27): close the loop**
- [ ] Per-Gaussian uncertainty + NBV ranking (arm C), goals to Nav2
- [ ] (Optional, pending advisor OK) 3D frontier over GLIM's map (arm B)
- [ ] Evaluation harness and benchmark runs in a shared indoor room

## Repo layout

- `ros2_ws/src/`: onboard packages. `robot_slam` (launch files, EKF/Nav2/slam_toolbox/GLIM configs, URDF, RViz, `scan_relay_node`, `cmd_vel_to_ackermann_node`), `frontier_explorer`, `livox_bringup`, `livox_ros_driver2` (git submodule), `oakd_publisher`, `keyframe_bridge`, `ackermann_mux`, `vesc`, `ucsd_robocar_hub2`
- `ros2_ws_offboard/`: empty, reserved for the 3DGS mapper / uncertainty / NBV packages.
- `scripts/`: one-command launchers per test; logs go to `/tmp/rover/`
- `docker/`: dev container (`docker/dev.sh up|shell|down`)
- `docs/`: see the README's Documentation list. `glim.md`, `software_systems.md` and `full_stage_testing.md` are the most relevant to this plan.

## Common commands

```bash
docker/dev.sh up && docker/dev.sh shell          # dev container on the Jetson
scripts/build.sh onboard                          # offboard is a no-op until 3DGS lands
scripts/teleop_slam.sh [--joy] [--rviz]           # teleop + onboard slam_toolbox
scripts/explore.sh [--auto-start] [--rviz]        # frontier exploration, all on the Jetson
scripts/{teleop_slam,explore}.sh --offboard-slam  # opt-in: pair with scripts/offboard.sh slam on the GPU machine
ros2 topic pub --once /mission/control std_msgs/msg/String "data: start"   # start exploring ("end" to stop)
ros2 launch robot_slam glim.launch.py             # GLIM (GPU); use_gpu:=false for CPU fallback
scripts/record.sh [--no-images]                   # MCAP bag into bags/ for offline GLIM/3DGS
```

## Conventions

- **IMUs:** the EKF fuses the **OAK-D Pro IMU** (`/oakd/imu`, yaw rate only; gyro bias calibrated at startup, so keep the rover still). GLIM uses the **Mid-360's built-in IMU** (`/livox/imu`), which shares the LiDAR's clock and housing; tight LiDAR-inertial coupling needs that. Don't swap GLIM onto the OAK-D IMU without a time-sync and extrinsic calibration plan.

- Mount offsets are in `robot_slam/urdf/robot.urdf.xacro`; keep `glim.launch.py`'s fallback static TF in sync with it.
- There is one `slam_toolbox.yaml` and one Nav2 params file (`nav2_params_onboard.yaml`), both in `robot_slam/config/`. Don't reintroduce off-board copies.
- GLIM config changes go in `robot_slam/config/glim/` with a `[rover]` comment, and are listed in its README.
- Keep CUDA and GLIM inside Docker; never install them on the Jetson host.
- Nav2 drives the base via `/cmd_vel` → `cmd_vel_to_ackermann_node` → `/ackermann_cmd`. Keyboard teleop does not override Nav2.
