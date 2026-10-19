# ROS 2 Testing Guide

How to build, bring up and verify each part of the rover stack: the sensors (Livox Mid-360, OAK-D Pro camera and IMU), keyboard teleop, SLAM, and frontier exploration. Each test builds on the one before it, so work through them in order the first time.

All launch scripts live in [`scripts/`](../scripts). See [full_stage_testing.md](full_stage_testing.md) for the script reference and troubleshooting.

| Test | Rover | GPU machine |
|------|-------|-------------|
| Sensors only | `scripts/sensors.sh` | — |
| Teleop | `scripts/teleop.sh` | — |
| Teleop + SLAM (on the Jetson) | `scripts/teleop_slam.sh` | optional: RViz |
| Teleop + SLAM (off-board) | `scripts/teleop_slam.sh --offboard-slam` | `scripts/offboard.sh slam` |
| Frontier exploration | `scripts/explore.sh` | `scripts/offboard.sh explore` |
| Frontier exploration (Jetson only) | `scripts/explore.sh --all-onboard` | — |

---

## 1. Build

On the rover (Jetson Orin Nano, ROS 2 Humble):

```bash
cd ~/scout-survey-rover
git submodule update --init ros2_ws/src/livox_ros_driver2
scripts/build.sh onboard
```

On the GPU machine:

```bash
scripts/build.sh offboard
```

`scripts/build.sh onboard` builds only the packages the scripts use (`--all` builds everything in `ros2_ws/src`). It also handles a quirk of the Livox driver: `livox_ros_driver2` ships `package_ROS2.xml` but no `package.xml`, so a plain `colcon build` skips it. The script copies the manifest into place and passes `-DROS_EDITION=ROS2 -DDISTRO_ROS=humble`. Don't use the driver's own `build.sh`, because it deletes the whole workspace's `build/` and `install/`.

Builds use `--symlink-install`, so edits to Python nodes, launch files and YAML take effect without rebuilding. C++ packages still need a rebuild. Never edit `build/` or `install/` directly.

For a manual shell, source both layers:

```bash
source /opt/ros/humble/setup.bash
source ~/scout-survey-rover/ros2_ws/install/setup.bash
```

---

## 2. Sensor tests

```bash
scripts/sensors.sh            # LiDAR + OAK-D
scripts/sensors.sh --lidar    # LiDAR only
scripts/sensors.sh --oakd     # OAK-D only
```

### LiDAR (Livox Mid-360)

The Mid-360 connects over Ethernet: the sensor is at `192.168.1.12` and the Jetson is statically `192.168.1.50` (see [robot_hardware.md](robot_hardware.md)). `livox_bringup` starts `livox_ros_driver2` and flattens its 3D point cloud into a 2D `LaserScan`:

```
livox_ros_driver2 → /livox/lidar (PointCloud2, livox_frame)
pointcloud_to_laserscan (±0.1 m Z slice) → /scan (LaserScan, ~723 beams, -π..π)
```

Verify:

```bash
ping -c 1 192.168.1.12
ros2 topic hz /livox/lidar     # ~10 Hz
ros2 topic hz /scan            # ~10 Hz
ros2 topic echo /scan --once --field angle_min   # ≈ -3.14
```

**Forward-direction check:** put a box about 1 m directly in front of the rover. In RViz (fixed frame `livox_frame`, LaserScan `/scan`), it should appear on the +X axis. If it appears somewhere else, the sensor is mounted rotated. Fix the `rpy` of `lidar_joint` in `robot_slam/urdf/robot.urdf.xacro`, not the scan relay.

**Self-hits:** if the chassis or camera mast shows up as a ring of points close to the sensor, raise `range_min` (for example `ros2 launch livox_bringup livox_mid360.launch.py range_min:=0.3`) or move the Z slice with `scan_height_min`/`scan_height_max`.

### OAK-D Pro camera

`oakd_publisher` is the OAK-D Pro driver. It uses the DepthAI v3 API: RGB comes from `CAM_A` and stereo depth from `CAM_B`/`CAM_C`.

```bash
ros2 launch oakd_publisher oakd_publisher.launch.py
ros2 topic hz /oakd/color/image_raw   # ~15 Hz, 1280x720
ros2 topic hz /oakd/depth/image_raw   # ~15 Hz, 1280x720
```

The OAK-D Pro's active IR is off by default. To enable it for dark or textureless scenes:

```bash
ros2 launch oakd_publisher oakd_publisher.launch.py ir_dot_projector_ma:=200.0 ir_flood_light_ma:=100.0
```

Only one process can own the device. Don't run `multi_cam` or `ucsd_robocar_sensor2_pkg camera_oakd.launch.py` at the same time; both are legacy drivers this stack doesn't use.

### OAK-D Pro IMU

The same node publishes the camera's onboard IMU (BMI270 on current units, BNO086 on early ones) on `/oakd/imu` at about 200 Hz. This IMU replaced the XIAO nRF52840. Only angular velocity and linear acceleration are published. Orientation is not published (`orientation_covariance[0] = -1`) because there is no magnetometer to give absolute heading.

**Keep the rover still for the first ~5 s after launch.** The node averages the gyro to estimate its bias and doesn't publish `/oakd/imu` until that finishes. Look for this in the log:

```
[oakd_publisher_node]: Gyro bias calibrated: [0.0012, -0.0008, 0.0003] rad/s
```

Samples are rotated into `oakd_imu_frame`, which is axis-aligned with `oakd_link` (x forward, y left, z up). The rotation comes from the IMU extrinsics in the device EEPROM. If those are missing, the node logs `Falling back to the imu_rpy parameter` and uses `imu_rpy` instead.

**Axis check (do this once per camera mount):**

```bash
ros2 topic echo /oakd/imu --field linear_acceleration
```

1. Rover flat and still: `z ≈ +9.8` and `x, y ≈ 0`.
2. Lift the front of the rover: `x` goes **negative** (gravity tilts toward the back).

```bash
ros2 topic echo /oakd/imu --field angular_velocity
```

3. Turn the rover counter-clockwise when viewed from above: `z` goes **positive**.

If any of these fail, override the IMU-to-camera rotation (roll, pitch, yaw in radians) until all three checks pass, then copy the values into `oakd_publisher.launch.py`:

```bash
ros2 run oakd_publisher oakd_publisher_node --ros-args \
  -p imu_rotation_source:=manual -p "imu_rpy:=[0.0, 0.0, 1.5708]"
```

If the camera itself is mounted tilted, put that tilt in the URDF `oakd_joint` instead.

---

## 3. Keyboard teleop

```bash
scripts/teleop.sh
```

```
keyboard_teleop_node → /ackermann_cmd → ackermann_to_vesc_node → commands/motor/speed + commands/servo/position
                                                                 → vesc_driver_node → VESC
vesc_driver_node → /sensors/core → vesc_to_odom_node → /vesc_odom
```

Keys: `w`/`s` step the speed forward or back, `a`/`d` steer, `space` stops, `r`/`f` and `e`/`c` change the step sizes, and `q` quits and stops everything.

Verify:

```bash
ros2 topic hz /sensors/core      # ~50 Hz VESC telemetry
ros2 topic echo /vesc_odom --field twist.twist.linear.x   # tracks commanded speed while driving
```

**VESC port:** the scripts use `/dev/vesc` if it exists (from [`udev/99-rover.rules`](../udev/99-rover.rules)), otherwise `/dev/ttyACM0`. Override with `VESC_PORT=/dev/ttyACM1 scripts/teleop.sh`.

**First-time setup:** servo output must be enabled in VESC Tool (App Settings → General → Use Servo Output → Write App Configuration).

### Steering calibration

Tune these values in `ucsd_robocar_control2_pkg/config/keyboard_teleop_config.yaml`:

```yaml
steering_angle_to_servo_gain: -0.6    # flip sign if steering is reversed
steering_angle_to_servo_offset: 0.5   # servo value (0–1) that centers the wheels
```

To find the offset, publish straight to the servo:

```bash
ros2 run vesc_driver vesc_driver_node --ros-args -p port:=/dev/vesc
ros2 topic pub /commands/servo/position std_msgs/msg/Float64 "data: 0.5" --once
# adjust until the wheels are straight → that value is the offset
```

Then `gain = (servo_full_left - offset) / max_steering_angle_radians`. Positive `steering_angle` must turn **left**, because Nav2's `cmd_vel_to_ackermann_node` and the EKF both assume REP-103.

---

## 4. SLAM

### Architecture

```
[Livox Mid-360] → /scan → scan_relay_node → /scan_fixed (500 beams, 250° front FOV)
                                                  ↓
[VESC]     → /vesc_odom (vx)        ─┐
[OAK-D Pro]→ /oakd/imu  (yaw rate)  ─┴→ ekf_local_node → odom→base_link TF, /odometry/filtered
                                                  ↓
[robot_state_publisher] → base_link→{livox_frame, oakd_link, oakd_imu_frame, oakd_camera_optical_frame}
                                                  ↓
                                   slam_toolbox (online async) → /map + map→odom TF
```

`scan_relay_node` (now in `robot_slam`) resamples every scan to a fixed beam count, because slam_toolbox rejects scans whose length changes after initialisation. Never run two instances.

### Option A: SLAM on the Jetson (self-contained)

```bash
scripts/teleop_slam.sh
```

This starts the LiDAR, OAK-D, VESC and `onboard.launch.py use_slam:=true`, then runs keyboard teleop in the foreground. `slam_toolbox` starts 5 s after the onboard launch so the EKF's TF exists before the first scan arrives.

### Option B: SLAM off-board

```bash
# rover
scripts/teleop_slam.sh --offboard-slam
# GPU machine (same ROS_DOMAIN_ID, DDS discovery working)
scripts/offboard.sh slam
```

### Verify

```bash
ros2 topic hz /scan_fixed           # ~10 Hz
ros2 topic hz /odometry/filtered    # ~50 Hz
ros2 run tf2_ros tf2_echo map base_link
```

`/map` is `TRANSIENT_LOCAL`, so `ros2 topic hz /map` is unreliable. Use the service instead:

```bash
ros2 service call /slam_toolbox/dynamic_map nav_msgs/srv/GetMap
```

A healthy map has `width`/`height` > 0. Expected log lines (in `/tmp/rover/onboard.log` for onboard SLAM):

```
[scan_relay_node]: Relaying /scan → /scan_fixed | 500 beams | FOV 250° centred at 0°
Registering sensor: [Custom Described Lidar]
```

One `Message Filter dropping message` at startup is normal. Repeated drops mean the TF chain is broken (see [full_stage_testing.md](full_stage_testing.md#diagnostics)).

**Odometry sanity check before trusting the map:** drive about 2 m straight and compare `ros2 topic echo /odometry/filtered --field pose.pose.position` against a tape measure. Then drive one full circle back to the start pose and check the EKF yaw returns close to where it began. A bad VESC `speed_to_erpm_gain` or wrong IMU axes show up here first.

### RViz

Prefer running it on another machine:

```bash
export ROS_DOMAIN_ID=<same as rover>
rviz2 -d ~/scout-survey-rover/ros2_ws/src/robot_slam/rviz/mapping.rviz
```

Or use `scripts/teleop_slam.sh --rviz` (Jetson display, CPU heavy), or `scripts/offboard.sh slam --rviz`.

### Save the map

```bash
ros2 run nav2_map_server map_saver_cli -f ~/scout-survey-rover/ros2_ws/src/robot_slam/ros_data/maps/custom/stage1_map
```

---

## 5. Frontier exploration

Exploration is split across both machines. The Jetson runs the low-latency controller, and the GPU machine runs SLAM, global planning and frontier selection:

```
[GPU] frontier_explorer → NavigateToPose → bt_navigator → planner_server (NavFn, global costmap)
                                                ↓ FollowPath action (over DDS)
[Jetson]  controller_server (Regulated Pure Pursuit, local costmap) → cmd_vel_nav
      velocity_smoother → /cmd_vel → cmd_vel_to_ackermann_node → /ackermann_cmd → VESC
```

The base is Ackermann (car-like), so the stack is configured to never rotate in place:
- The controller is Regulated Pure Pursuit with `use_rotate_to_heading: false`. DWB would sample spin-in-place commands, which a car can't execute.
- `yaw_goal_tolerance` is 2π, because frontier goals carry an arbitrary heading.
- `cmd_vel_to_ackermann_node` converts using `steering = atan(wheelbase · ω / v)`, clamped to ±0.4 rad. Any spin-in-place command that does arrive is turned into a stop with a warning.

### Run

```bash
# rover
scripts/explore.sh
# GPU machine
scripts/offboard.sh explore
# either machine, once both are up:
ros2 topic pub --once /mission/control std_msgs/msg/String "data: start"
```

Stop exploring (cancels the active goal):

```bash
ros2 topic pub --once /mission/control std_msgs/msg/String "data: end"
```

To keep SLAM on the Jetson instead: `scripts/explore.sh --onboard-slam` with `scripts/offboard.sh explore --no-slam`.

To run everything on the Jetson, with no GPU machine: build both workspaces on it (`scripts/build.sh onboard && scripts/build.sh offboard`), then run `scripts/explore.sh --all-onboard`. The off-board launch output goes to `/tmp/rover/offboard.log`. See [software_systems.md](software_systems.md#checking-the-budget-on-the-jetson) for how to check CPU headroom.

### Verify

```bash
ros2 lifecycle get /controller_server     # active [3]   (rover)
ros2 lifecycle get /bt_navigator          # active [3]   (GPU)
ros2 action list | grep -E "follow_path|navigate_to_pose"
ros2 topic echo /mission/state            # exploration_goal_sent / _reached / _failed / _complete
ros2 topic echo /ackermann_cmd            # non-zero while a goal is active
```

Before the first autonomous run, send one manual goal with RViz's **2D Goal Pose** tool about 1.5 m ahead in open space. That checks the whole planner → controller → converter → VESC chain without the explorer.

**Safety:** keyboard teleop doesn't override Nav2 here, since both publish `/ackermann_cmd`. Stop with `data: end`, Ctrl+C on `scripts/explore.sh`, or the power switch.

---

## 6. Diagnostics

```bash
ros2 node list
```

Expected for `scripts/teleop_slam.sh` (onboard SLAM): `/livox_lidar_publisher`, `/livox_pointcloud_to_laserscan`, `/oakd_publisher_node`, `/ackermann_to_vesc_node`, `/vesc_driver_node`, `/vesc_to_odom_node`, `/scan_relay_node`, `/ekf_local_node`, `/robot_state_publisher`, `/slam_toolbox`.

`scripts/explore.sh` adds `/controller_server`, `/local_costmap/local_costmap`, `/velocity_smoother`, `/cmd_vel_to_ackermann_node` and `/lifecycle_manager_onboard`, and drops `/slam_toolbox` (it runs off-board).

Per-component logs are written to `/tmp/rover/<component>.log`. Scan them all with:

```bash
grep -E "ERROR|process has died|Caught exception|Message Filter dropping" /tmp/rover/*.log
```
