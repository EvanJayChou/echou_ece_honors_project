# Launch Scripts & Troubleshooting Reference

The scripts in [`scripts/`](../scripts) replace the old `stage1_start.sh`. Each one brings up exactly what one test needs, writes a log per component to `/tmp/rover/` (override with `LOG_DIR`), and stops everything on Ctrl+C or when teleop quits. See [ros2_testing.md](ros2_testing.md) for what to verify in each test.

## Scripts

| Script | Runs on | Starts |
|--------|---------|--------|
| `build.sh {onboard\|offboard} [--all] [--clean]` | either | `colcon build` with the Livox driver workarounds |
| `sensors.sh [--lidar\|--oakd]` | rover | Livox Mid-360, OAK-D Pro (RGB-D + IMU) |
| `teleop.sh [--joy]` | rover | VESC stack + keyboard teleop (`--joy`: Logitech F710 gamepad) |
| `teleop_slam.sh [--offboard-slam] [--headless] [--rviz] [--joy]` | rover | sensors + VESC + scan relay + EKF + slam_toolbox (unless `--offboard-slam`) + keyboard teleop (`--joy`: F710 gamepad) |
| `explore.sh [--onboard-slam\|--all-onboard] [--rviz]` | rover | sensors + VESC + scan relay + EKF + Nav2 controller + `cmd_vel_to_ackermann_node`. `--all-onboard` also runs SLAM, the Nav2 global side and frontier_explorer on the Jetson (no GPU machine) |
| `offboard.sh slam [--rviz]` | GPU machine | slam_toolbox |
| `offboard.sh explore [--no-slam] [--auto-start] [--rviz]` | GPU machine | slam_toolbox + Nav2 global side + frontier_explorer |

Pairings: `teleop_slam.sh --offboard-slam` ↔ `offboard.sh slam`, and `explore.sh` ↔ `offboard.sh explore`.

Environment overrides (all scripts): `VESC_PORT` (default `/dev/vesc`, else `/dev/ttyACM0`), `LIVOX_IP` (default `192.168.1.12`), `ROS_DISTRO_NAME` (default `$ROS_DISTRO`, else `jazzy`), `LOG_DIR` (default `/tmp/rover`).

Both machines must use the same `ROS_DOMAIN_ID`. The scripts print the one in use.

## Prerequisites

- ROS 2 Jazzy at `/opt/ros/jazzy` on both machines (Ubuntu 24.04; JetPack 7.2 on the Jetson), workspaces built with `scripts/build.sh`
- Livox Mid-360 reachable at `192.168.1.12` (Jetson Ethernet statically `192.168.1.50`)
- OAK-D Pro on a **USB 3** port
- VESC on USB, with servo output enabled in VESC Tool (App Settings → General → Use Servo Output → Write App Configuration)
- `udev/99-rover.rules` installed (gives the VESC a stable `/dev/vesc` name)

## Startup sequence (`teleop_slam.sh` / `explore.sh`)

| Time | Event |
|------|-------|
| t=0 s | LiDAR, OAK-D and VESC start. **Keep the rover still**: OAK-D gyro bias calibration runs for 5 s |
| t=3 s | `onboard.launch.py`: scan relay, EKF, robot_state_publisher (+ Nav2 controller for `explore.sh`) |
| t=8 s | `slam_toolbox` starts (onboard SLAM only; 5 s `TimerAction`) |
| t=11 s | Script checks `/scan_fixed`, `/oakd/imu`, `/vesc_odom`, `/odometry/filtered` and reports each one |

## Logs

| Component | Log file |
|-----------|----------|
| LiDAR (`livox_bringup`) | `/tmp/rover/lidar.log` |
| OAK-D (`oakd_publisher`) | `/tmp/rover/oakd.log` |
| VESC stack | `/tmp/rover/vesc.log` |
| Onboard launch (scan relay, EKF, SLAM, Nav2 controller) | `/tmp/rover/onboard.log` |
| Off-board launch | `/tmp/rover/offboard.log` |

## All topic rates at once

```bash
for t in /scan /scan_fixed /oakd/imu /vesc_odom /odometry/filtered /oakd/color/image_raw; do
  rate=$(timeout 4 ros2 topic hz $t 2>&1 | grep "average rate" | awk '{print $3}')
  echo "  $t: ${rate:-NOT PUBLISHING}"
done
```

Expected: `/scan` and `/scan_fixed` ~10 Hz, `/oakd/imu` ~200 Hz, `/vesc_odom` ~50 Hz, `/odometry/filtered` ~50 Hz, color/depth ~15 Hz (1280×720), `/local_costmap/costmap` ~5 Hz.

## Expected TF tree

```
map ──(slam_toolbox)──▶ odom ──(ekf_local_node)──▶ base_link ──(robot_state_publisher, static)──▶ livox_frame
                                                             └──▶ oakd_link ──▶ oakd_camera_optical_frame
                                                                           └──▶ oakd_imu_frame
```

```bash
ros2 run tf2_tools view_frames      # writes frames.pdf
```

## Diagnostics

### Repeated message-filter drops in slam_toolbox

One drop at startup is normal. Repeated drops mean slam_toolbox can't transform the scan.

- `odom → base_link` missing: the EKF isn't publishing. Check `/vesc_odom` and `/oakd/imu` rates. `/oakd/imu` stays silent until the gyro calibration finishes.
- `base_link → livox_frame` missing: `robot_state_publisher` died, or the scan's `frame_id` doesn't match the URDF. Both must be `livox_frame` (`livox_bringup` `lidar_frame` argument).
- Off-board SLAM only: the GPU machine can't see the rover's `/tf`. Check `ROS_DOMAIN_ID` and DDS discovery.

### slam_toolbox: "LaserRangeScan contains X readings, expected Y"

Two `scan_relay_node` instances are running. Stop everything and relaunch through a script. Never run `scan_relay_node` by hand next to `onboard.launch.py`.

### No `/scan` from the Livox

- `ping 192.168.1.12` fails: check the cable and the Jetson's static IP (`nmcli con show livox`, `ip -br addr`).
- Ping works but there's no data: the host IPs in `livox_bringup/config/MID360_config.json` must match the Jetson's address, and a firewall must not block UDP 56100–56501.
- `/livox/lidar` publishes but `/scan` is empty: the ±0.1 m Z slice may miss everything if the sensor is tilted. Widen it with `scan_height_min`/`scan_height_max`.

### OAK-D not found / `/oakd/imu` silent

- `lsusb | grep 03e7` shows nothing: re-seat the cable, use a USB 3 port, and install the udev rule.
- `X_LINK_DEVICE_NOT_FOUND` or `device already in use`: another process owns the camera (`multi_cam`, an old `camera_oakd.launch.py`, a DepthAI demo). Kill it.
- Color works but there's no IMU: wait for `Gyro bias calibrated` in `/tmp/rover/oakd.log`. If the log shows an IMU sensor error, the requested report types or rate aren't supported by that IMU chip. Try `imu_rate_hz:=50`.

### Map smears or rotates while driving

Usually the yaw rate is wrong. Re-run the IMU axis check in [ros2_testing.md §2](ros2_testing.md#oak-d-pro-imu). Also make sure the rover was still during gyro calibration. If it moved, restart the OAK-D node.

### Map stretches or shrinks along the direction of travel

The VESC `speed_to_erpm_gain` is wrong. Calibrate it by driving a measured distance and comparing it with `/vesc_odom`.

### VESC not connecting

```bash
ls -l /dev/vesc /dev/ttyACM*
python3 -c "from serial.tools import list_ports; [print(p.device, hex(p.vid or 0), hex(p.pid or 0)) for p in list_ports.comports()]"
# VESC shows as vid=0x483 pid=0x5740
```

### Exploration: goals sent but the rover doesn't move

- `ros2 topic echo /cmd_vel`: if it's empty, the controller isn't receiving `FollowPath`. Check that `/follow_path` is visible from the GPU machine.
- `/cmd_vel` has values but `/ackermann_cmd` is zero with "Rotate-in-place" warnings: the controller is asking for spin-in-place. Check that `use_rotate_to_heading: false` is set in `nav2_params_onboard.yaml`.
- Goals keep failing with `exploration_goal_failed`: frontiers may sit inside the inflated costmap, or the car can't turn tightly enough. Try a larger `min_frontier_cluster_size` or a smaller `inflation_radius`.

## Out of scope

- Stage 2 localization (`launch_stage_2.launch.py`, AMCL against a saved map). It hasn't been updated for the Ackermann controller.
- 3DGS mapping, uncertainty estimation and NBV selection (Q2, off-board). `keyframe_bridge` is ready for it but no script starts it yet.
