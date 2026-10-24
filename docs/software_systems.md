# <div align="center">Software Systems</div>

## Overall Architecture

![image](https://raw.githubusercontent.com/UCSD-ECEMAE-148/148sp26-spring-2026-final-project-team-4/main/docs/media/rosgraph_nodes_only.png)

*(The graph above predates the Livox / OAK-D Pro IMU migration.)*

The stack runs natively on a Jetson Orin Nano (JetPack 7.2 / Ubuntu 24.04) with ROS 2 Jazzy, with no Docker container. This was a deliberate choice over the UCSD RoboCar Hub2 Docker image, for direct hardware access and lower latency. The GPU machine must also run Jazzy.

It is split across two ROS 2 workspaces that share a `ROS_DOMAIN_ID`:

| Workspace | Machine | Runs |
|-----------|---------|------|
| `ros2_ws/` | Jetson Orin Nano (rover) | Sensor drivers, VESC, scan relay, EKF, Nav2 controller (latency-critical) |
| `ros2_ws_offboard/` | GPU machine | slam_toolbox, Nav2 global planning / BT, frontier_explorer (Q2: 3DGS mapper + NBV) |

SLAM can also run on the Jetson for self-contained tests (`onboard.launch.py use_slam:=true`). Each test has its own launch script in [`scripts/`](../scripts). See [full_stage_testing.md](full_stage_testing.md).

---

## Sensors

### Livox Mid-360 (`livox_bringup`)

`livox_ros_driver2` (git submodule, built against Livox-SDK2) publishes `/livox/lidar` as a standard PCL `PointXYZI` `PointCloud2` (`xfer_format: 2`) at 10 Hz in `livox_frame`. `pointcloud_to_laserscan` flattens a ±0.1 m Z slice of that cloud into `/scan`: a 360° `LaserScan` with a fixed `angle_increment` of 0.0087 rad (~723 beams). The rest of the 2D stack consumes `/scan`, so the LiDAR can be swapped without touching SLAM or Nav2.

The Mid-360's own IMU (`/livox/imu`) is published by the driver but not used.

### OAK-D Pro (`oakd_publisher`)

A single DepthAI v3 pipeline on the OAK-D Pro produces:

| Topic | Type | Frame | Rate |
|-------|------|-------|------|
| `/oakd/color/image_raw` | `sensor_msgs/Image` (bgr8, 1280×720) | `oakd_camera_optical_frame` | 15 Hz |
| `/oakd/depth/image_raw` | `sensor_msgs/Image` (mono16 mm, 1280×720) | `oakd_camera_optical_frame` | 15 Hz |
| `/oakd/imu` | `sensor_msgs/Imu` | `oakd_imu_frame` | 200 Hz |

**IMU pipeline.** The node requests `ACCELEROMETER_RAW` and `GYROSCOPE_RAW` reports. That is the subset shared by the BMI270 (current OAK-D Pro) and the BNO086 (early units). Neither chip has a magnetometer, so no orientation is published (`orientation_covariance[0] = -1`). Each sample is:

1. **Rotated** into `oakd_imu_frame`, which is axis-aligned with the camera body `oakd_link` (REP-103: x forward, y left, z up). The rotation is the IMU→`CAM_A` extrinsic stored in the device EEPROM, composed with the optical→body rotation. If the EEPROM has no IMU extrinsics, the `imu_rpy` parameter is used instead. The URDF then only needs the camera mount pose.
2. **De-biased.** For the first `gyro_bias_calibration_s` (5 s), samples are averaged into a gyro bias and nothing is published, so the EKF never ingests a biased rate. The rover must be stationary during this window.
3. **Stamped** with the device's host-synced capture time, shifted onto the ROS clock, rather than the time it was dequeued.

The OAK-D Pro's IR dot projector and flood light are off by default (`ir_dot_projector_ma`, `ir_flood_light_ma`).

> `multi_cam` and `ucsd_robocar_sensor2_pkg camera_oakd.launch.py` are legacy UCSD drivers (RGB-only, multi-device). The rover stack doesn't use them, and they can't share the camera with `oakd_publisher`.

---

## State estimation and SLAM

### Data flow

```
[Livox Mid-360] → /livox/lidar → pointcloud_to_laserscan → /scan (~723 beams, 360°)
                                                              ↓
                                                     scan_relay_node
                                                              ↓
                                               /scan_fixed (540 beams, 270° front FOV)
                                                              ↓
[VESC]      → /vesc_odom  (vx)        ─┐
[OAK-D Pro] → /oakd/imu   (yaw rate)  ─┴→ ekf_local_node → odom→base_link TF + /odometry/filtered (50 Hz)
                                                              ↓
                                          slam_toolbox → /map + map→odom TF
```

### scan_relay_node (`robot_slam`)

Crops `/scan` to a 270° front-facing FOV centred on +X (±135°; the rear 90° faces the rover body) and resamples it to exactly 540 beams on every message. slam_toolbox rejects scans whose beam count differs from the first one it saw, so this fixed count is required. It is also why two relays must never run at once. Angles are normalised to [-π, π] first, so the relay works with either angle convention (the old LD06 used 0→2π).

### Extended Kalman Filter (`ekf_local.yaml`)

`robot_localization`'s `ekf_local_node` runs at 50 Hz in `two_d_mode`:

| Source | Topic | Fused | Why |
|--------|-------|-------|-----|
| VESC wheel odometry (`vesc_to_odom_node`) | `/vesc_odom` | `vx` | eRPM-derived speed is accurate for translation |
| OAK-D Pro IMU | `/oakd/imu` | `vyaw` | Gyro yaw rate, bias-corrected at startup. There's no absolute yaw (no magnetometer) |

Heading is the integral of the gyro yaw rate. slam_toolbox's scan matching corrects any drift through `map→odom`. `robot_localization` transforms the IMU sample from `oakd_imu_frame` into `base_link` using the URDF TF. `use_control: true` uses `/cmd_vel` (published only when Nav2 is driving) as a motion-model prediction.

This replaces the earlier XIAO nRF52840 setup, which fused the XIAO's dead-reckoned `vtheta` and differential quaternion yaw.

### slam_toolbox

Online async mode on `/scan_fixed` with a 5 cm grid, publishing `/map` with `TRANSIENT_LOCAL` QoS. A new scan is added after ≥ 0.05 m of travel or ≥ 0.05 rad of rotation, at most every 0.1 s (`minimum_time_interval`). The map republishes every 0.5 s and uses returns out to 20 m. The onboard launch delays it by 5 s so `odom→base_link` exists before the first scan lookup. Config: `robot_slam/config/slam_toolbox.yaml` (onboard) or `offboard_nav/config/slam_toolbox.yaml` (off-board). Keep the two in sync.

---

## Rates and compute budget (Jetson Orin Nano)

The Raspberry Pi 5 couldn't run SLAM and Nav2 together, so several rates were held down. These are the values for the Orin Nano and the reasoning behind each:

| Setting | Pi 5 | Orin Nano | Notes |
|---------|------|-----------|-------|
| EKF `frequency` | 25 Hz | **50 Hz** | `ekf_local.yaml`; `sensor_timeout` 0.3 → 0.1 s |
| OAK-D IMU | 100 Hz | **200 Hz** | `imu_rate_hz` launch arg |
| OAK-D color/depth | 640×480 @ 15 Hz | **1280×720 @ 15 Hz** | Resolution for 3DGS keyframes. The rate stays at 15 Hz because each raw frame pair is ~4.5 MB through the middleware and `keyframe_bridge` only republishes on motion |
| slam_toolbox `minimum_time_interval` | 0.5 s (default) | **0.1 s** | The default silently capped scan matching at 2 Hz |
| slam_toolbox `map_update_interval` | 1.0 s | **0.5 s** | |
| slam_toolbox `max_laser_range` / scan buffer | 12 m / 20 scans, 10 m | **20 m / 30 scans, 20 m** | |
| slam_toolbox `loop_search_maximum_distance` | 4 m | **6 m** | |
| Nav2 `controller_frequency` / velocity smoother | 20 Hz | **30 Hz** | |
| Local costmap update / publish / size | 5 Hz / 2 Hz / 3 m | **10 Hz / 5 Hz / 5 m** | 10 Hz = every LiDAR scan |
| Global costmap update / publish | 1 Hz | **2 Hz** | Tracks the 0.5 s map updates |
| `frontier_explorer` replan period | 2 s | **1 s** | Frontier detection vectorised with NumPy (~10× faster, identical output) |
| slam_toolbox start delay | 10 s | **5 s** | |
| SLAM + Nav2 + explorer on one machine | not possible | **`scripts/explore.sh --all-onboard`** | |

Deliberately **unchanged**:
- **Livox `publish_freq` (10 Hz).** The Mid-360 emits a fixed ~200k points/s, so a higher frame rate means sparser frames and gappier 2D scans. It's a launch argument if you want to experiment.
- **`scan_relay_node` 270° crop.** The rear of the scan is blocked by the rover body. That's geometry, not CPU.
- **5 cm map resolution.** 2.5 cm would 4× the cells processed by the explorer and costmaps, for little gain on a car this size.

### Checking the budget on the Jetson

Run the heaviest configuration (`scripts/explore.sh --all-onboard`) and watch:

```bash
sudo tegrastats                       # per-core CPU, GPU, RAM, temps (Ctrl+C to stop)
top -c                                # which nodes use the CPU
ros2 topic hz /odometry/filtered      # should hold ~50 Hz
grep -c "Failed to meet update rate" /tmp/rover/onboard.log      # EKF overruns
grep -c "Control loop missed its desired rate" /tmp/rover/onboard.log
```

An occasional overrun is harmless. If these counts climb steadily, or `/odometry/filtered` drops well below 50 Hz, back off in this order:
1. OAK-D to 640×480 (`rgb_width:=640 rgb_height:=480`). The Python image publishing is the largest avoidable load.
2. IMU to 100 Hz (`imu_rate_hz:=100`).
3. Controller and velocity smoother to 20 Hz.
4. EKF to 30 Hz.

Use the highest Jetson power mode (`sudo nvpmodel -q`; the scripts warn if it isn't MAXN), and optionally run `sudo jetson_clocks`.

---

## Driving

### Teleop

```
keyboard_teleop_node → /ackermann_cmd → ackermann_to_vesc_node → vesc_driver_node → VESC
```

`keyboard_teleop.launch.py` (in `ucsd_robocar_control2_pkg`) starts `ackermann_to_vesc_node`, `vesc_driver_node` and `vesc_to_odom_node`, which remaps `/odom` to `/vesc_odom` with `publish_tf: false` so the EKF owns `odom→base_link`. The `vesc_port` launch argument overrides the port in `keyboard_teleop_config.yaml`.

### Navigation and frontier exploration

Nav2 is split by latency. The controller stays on the Jetson, and everything that plans runs off-board:

| Node | Where | Notes |
|------|-------|-------|
| `controller_server` + local costmap | Jetson | Regulated Pure Pursuit, 30 Hz; 5 m rolling costmap at 10 Hz, `/scan` obstacle layer |
| `velocity_smoother` | Jetson | `cmd_vel_nav` → `/cmd_vel` |
| `cmd_vel_to_ackermann_node` | Jetson | `/cmd_vel` → `/ackermann_cmd` |
| `planner_server` + global costmap | GPU | NavFn, `allow_unknown: true` |
| `bt_navigator`, `behavior_server`, `smoother_server`, `waypoint_follower` | GPU | |
| `frontier_explorer` | GPU | Clusters free/unknown boundary cells in `/map`, sends `NavigateToPose` goals, blacklists failures |

**Ackermann constraints.** The base can't rotate in place, so:
- The controller is Regulated Pure Pursuit with `use_rotate_to_heading: false` and `allow_reversing: false`. The previous DWB configuration sampled pure-rotation commands.
- `yaw_goal_tolerance` is 2π, since frontier goals have arbitrary headings.
- `cmd_vel_to_ackermann_node` applies the bicycle model, `δ = atan(L·ω / v)` with L = 0.34 m, clamped to ±0.4 rad. A pure-rotation command (v ≈ 0) is sent as a stop with a throttled warning.
- Nav2's default `Spin` recovery can't execute and will simply fail through to `BackUp`/`Wait`.

`frontier_explorer` is idle until `/mission/control` receives `start` (or `explorer_auto_start:=true`). It publishes progress on `/mission/state` and sends `end` itself once no frontiers remain for 5 consecutive cycles.

---

## Keyframe bridge (Q2 groundwork)

`keyframe_bridge` pairs `/oakd/color` and `/oakd/depth` frames by timestamp. It looks up `map → oakd_camera_optical_frame` at the image stamp and republishes a keyframe on `/keyframe/{color,depth,pose}` (best-effort QoS) whenever the camera has moved ≥ 0.15 m or rotated ≥ 0.2 rad. These topics will feed the off-board 3D Gaussian Splatting mapper. Start it with `ros2 launch keyframe_bridge keyframe_bridge.launch.py`. None of the test scripts launch it yet.
