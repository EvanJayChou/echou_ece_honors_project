# Stage 1 Verification & Troubleshooting Reference

Stage 1 brings up the full mapping stack — LiDAR, IMU/EKF, SLAM, VESC, LED feedback, and optionally the survey camera and mission report web server. The launch script handles sequencing; this document covers how to verify the stack is healthy and how to diagnose failures.

## Launching

```bash
# Keyboard teleop in this terminal (interactive)
./stage1_start.sh

# Headless — all nodes, no teleop (e.g. drive via ROS topic)
./stage1_start.sh --headless

# Full stack — headless + survey camera + mission report web server
./stage1_start.sh --full

# Enable RViz on the Pi display (CPU warning — see below)
./stage1_start.sh --rviz
```

`--full` implies `--headless`. The web controller at `http://<rover-ip>:3000` replaces keyboard teleop when `--full` is used.

## Prerequisites

- ROS 2 Jazzy installed at `/opt/ros/jazzy`
- `slam_ros2_ws` built: `cd slam_ros2_ws && colcon build --symlink-install`
- Livox Mid-360 LiDAR reachable at `192.168.1.12` (Pi Ethernet statically assigned `192.168.1.50`)
- XIAO nRF52840 Sense on `/dev/ttyACM0`
- VESC on `/dev/ttyACM1` — servo output must be enabled in VESC Tool: App Settings → General → Use Servo Output → Write App Configuration
- Pico LED bridge on `/dev/ttyACM2` (optional — LED feedback disabled if absent)

## Expected startup sequence

| Time | Event |
|------|-------|
| t=0 s | Pico hw server starts (retries internally up to 10 s) |
| t=0 s | LiDAR starts |
| t=3 s | IMU bridge starts |
| t=7 s | SLAM stack starts — `scan_relay_node`, `ekf_local_node`, `robot_state_publisher` up immediately |
| t=12 s | VESC driver starts |
| t=15 s | LED state monitor starts |
| t=17 s | `slam_toolbox` activates (10 s internal `TimerAction` from SLAM launch) |

## Verifying each subsystem

### LiDAR
```bash
ros2 topic hz /scan           # expect ~10 Hz
```

### IMU / odometry
```bash
ros2 topic hz /odom           # expect ~50 Hz
ros2 topic hz /imu            # expect ~50 Hz
```

### EKF and TF chain
```bash
ros2 topic hz /odometry/filtered          # expect ~10 Hz
ros2 topic echo /tf --once | grep -E "frame_id|child_frame"
ros2 topic echo /tf_static --once | grep child_frame
```

Expected TF frames:
- `/tf`: `odom → base_link` (published by EKF)
- `/tf_static`: `base_link → laser`, `base_link → camera_link`, `base_link → base_imu_link`

### SLAM map
`/map` uses `TRANSIENT_LOCAL` QoS — `ros2 topic hz /map` always shows "not published" even when the map is live. Use the service instead:

```bash
ros2 service call /slam_toolbox/dynamic_map nav_msgs/srv/GetMap
```

A valid response has `width` and `height` > 0 and a `data` array with `-1` (unknown), `0` (free), and `100` (occupied) values. If `width: 0`, wait 30 s and retry — `slam_toolbox` activates 10 s after the SLAM launch and needs a few more seconds to accept the first scan.

### VESC
```bash
ros2 topic hz /sensors/core   # expect ~50 Hz VESC telemetry
```

### All topic rates at once
```bash
for t in /scan /scan_fixed /odom /imu /odometry/filtered; do
  rate=$(timeout 4 ros2 topic hz $t 2>&1 | grep "average rate" | awk '{print $3}')
  echo "  $t: ${rate:-NOT PUBLISHING}"
done
```

### All nodes present
```bash
ros2 node list
# Expected:
#   /pico_hw_server  /led_state_monitor
#   /livox_lidar_publisher  /livox_pointcloud_to_laserscan  /serial_bridge_node  /scan_relay_node
#   /ekf_local_node  /robot_state_publisher  /slam_toolbox
#   /ackermann_to_vesc_node  /vesc_driver_node
```

## Watching the map grow

```bash
watch -n 5 "ros2 service call /slam_toolbox/dynamic_map nav_msgs/srv/GetMap 2>&1 | grep -E 'width|height'"
```

SLAM adds a pose node when the robot moves ≥ 0.1 m or rotates ≥ 0.2 rad — short, slow movements may not trigger updates.

## Saving the map

```bash
ros2 run nav2_map_server map_saver_cli \
  -f ~/scout-survey-rover/slam_ros2_ws/src/robot_slam/ros_data/maps/custom/stage1_map
```

Verify:
```bash
ls -lh ~/scout-survey-rover/slam_ros2_ws/src/robot_slam/ros_data/maps/custom/
# stage1_map.yaml  stage1_map.pgm
```

## RViz (optional, remote recommended)

```bash
./stage1_start.sh --rviz
```

**CPU warning**: RViz is heavy on the Pi. If the map stalls or EKF lag increases, close RViz. Preferred: run RViz on a separate machine on the same network:

```bash
export ROS_DOMAIN_ID=0
source /opt/ros/jazzy/setup.bash
rviz2 -d <path-to>/robot_slam/rviz/mapping.rviz
```

## Logs

| Component | Log file |
|-----------|----------|
| Pico hw server | `/tmp/stage1_pico.log` |
| LiDAR | `/tmp/stage1_lidar.log` |
| IMU bridge | `/tmp/stage1_imu.log` |
| SLAM stack | `/tmp/stage1_slam.log` |
| VESC | `/tmp/stage1_vesc.log` |
| LED monitor | `/tmp/stage1_led.log` |
| Survey camera (`--full`) | `/tmp/stage1_survey_camera.log` |
| Mission report server (`--full`) | `/tmp/stage1_mission_report.log` |

## Diagnostics

### Repeated message-filter drops in slam_toolbox

One drop at startup is normal. Repeated drops every few seconds mean the TF chain is broken.

```bash
ros2 topic echo /tf --once | grep "frame_id\|child_frame"
```

- `odom → base_link` missing → EKF is not publishing. Check `/odom` and `/imu` rates.
- `base_link → laser` missing → `robot_state_publisher` crashed. Check `/tmp/stage1_slam.log`.

### slam_toolbox: "LaserRangeScan contains X readings, expected Y"

Two `scan_relay_node` instances are running. Do not start `scan_relay_node` manually — the SLAM launch includes its own. Kill everything and relaunch via the script:

```bash
pkill -9 -f "scan_relay_node|slam_toolbox|ekf_node|robot_state_publisher"
./stage1_start.sh
```

### XIAO bridge not connecting

```bash
ls /dev/ttyACM*
python3 -c "from serial.tools import list_ports; [print(p.device, hex(p.vid or 0), hex(p.pid or 0)) for p in list_ports.comports()]"
# XIAO shows as vid=0x2886 pid=0x8045
```

If the device appears but the bridge still fails, replug USB and wait 5 s — the bridge reconnects automatically.

### Why `publish_tf:=false` on the IMU bridge

The serial bridge defaults to publishing the `odom → base_link` TF at 50 Hz from raw accelerometer dead-reckoning. If left on, it overwrites the EKF's position estimate and makes `slam_toolbox` think the robot never moves. The EKF owns that TF exclusively.

## What is intentionally out of scope for Stage 1

- Nav2 autonomous navigation (`use_nav2:=true` exists but is not tested in Stage 1 — it saturates CPU alongside SLAM)
- Frontier exploration planner
- Oak-D YOLO safety layer
