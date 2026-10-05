# <div align="center">Software Systems</div>

## Overall Architecture

![image](https://raw.githubusercontent.com/UCSD-ECEMAE-148/148sp26-spring-2026-final-project-team-4/main/docs/media/rosgraph_nodes_only.png)

The stack runs natively on Ubuntu 24.04 with ROS 2 Jazzy — no Docker container. This was a deliberate choice over the UCSD RoboCar Hub2 Docker image to get direct hardware access and lower latency on the Raspberry Pi 5. All dependencies (ROS packages, Python libraries, Node.js) are installed system-wide.

The full system is launched with a single script:

```bash
./stage1_start.sh --full
```

This coordinates timed startup of every subsystem (see [full_stage_testing.md](full_stage_testing.md) for the timing sequence) and handles clean shutdown with LED feedback via the Pico bridge.

---

## IMU Odometry — XIAO nRF52840 Sense

### Firmware (`xiao_serial_bridge/firmware/xiao_odom/`)

The XIAO runs custom Arduino firmware on its onboard LSM6DS3 IMU. At startup it collects 500 gyroscope samples (~10 seconds) to compute a static bias offset per axis. The robot must remain stationary during this window — any movement corrupts the bias estimate and degrades heading accuracy for the entire session.

After calibration the firmware integrates accelerometer and gyroscope data at ~50 Hz using a complementary filter for orientation and trapezoidal integration for position. Each tick it emits a single CSV line over USB CDC serial:

```
<seq>,<timestamp>,<ax>,<ay>,<az>,<gx>,<gy>,<gz>,<x>,<y>,<theta>,<vx>,<vtheta>
```

All values are in SI units. The 13-field format is fixed — the ROS bridge validates field count and rejects malformed packets.

### ROS driver (`xiao_serial_bridge` package)

`serial_bridge_node` runs a background thread that reads the serial port and pushes packets to a queue. A 50 Hz ROS timer drains the queue (keeping only the latest packet per tick) and publishes:

- `/odom` (`nav_msgs/Odometry`) — pose (x, y, θ) and velocities (vx, vθ) with tuned covariance. Translational velocity covariance is set high (`0.5`) because accel-based velocity is noisy; angular velocity covariance is low (`0.01`).
- `/imu` (`sensor_msgs/Imu`) — raw accelerometer + bias-corrected gyroscope. `/imu` is intentionally **not published during the 10-second calibration window** so the EKF never ingests biased gyro values.

The node auto-detects the XIAO by USB VID/PID (`0x2886:0x8045`) if `/dev/ttyACM0` is absent, and recovers automatically on replug.

**`publish_tf:=false` is set at launch.** The firmware dead-reckoning position is too noisy to own the `odom→base_link` transform — the EKF is the sole publisher of that TF.

---

## SLAM — Extended Kalman Filter + slam_toolbox

### Data flow

```
[Livox Mid-360]  →  /livox/lidar (PointCloud2, xfer_format=2)
                     ↓
      livox_pointcloud_to_laserscan (livox_bringup)
                     ↓
              /scan (~720 beams, 0–360°)
                     ↓
              scan_relay_node
                     ↓
           /scan_fixed (300 beams, 250° front FOV)
                     ↓
[XIAO IMU]  →  /odom, /imu  ──┐
[VESC]      →  /vesc_odom   ──┤→  ekf_local_node  →  odom→base_link TF (25 Hz)
                               ↓
                        slam_toolbox  →  /map  +  map→odom TF
```

### scan_relay_node

The Mid-360's 3D point cloud is flattened to 2D by `pointcloud_to_laserscan` (a thin Z-slice around the sensor's height, configured in `livox_bringup/launch/livox_mid360.launch.py`), producing a full 360° `LaserScan` at a fixed ~720-beam resolution (`angle_increment: 0.0087` rad). `scan_relay_node` then crops that to a 250° front-facing FOV and resamples it to exactly 300 beams on every tick. The fixed beam count is required by `slam_toolbox` — it rejects scans that don't match the count it saw at initialisation. This is also why two `scan_relay_node` instances must never run simultaneously.

### Extended Kalman Filter (`ekf_local.yaml`)

`robot_localization`'s `ekf_local_node` fuses three sensor streams at 25 Hz:

| Source | Topic | What's fused | Why |
|--------|-------|-------------|-----|
| XIAO dead-reckoning | `/odom` | `vtheta` only | `vx` is accel-derived and unreliable at constant speed |
| VESC wheel encoder | `/vesc_odom` | `vx` only | eRPM-based velocity is accurate for translation |
| XIAO IMU | `/imu` | Yaw (differential) | Differential mode converts consecutive quaternions into delta-yaw, avoiding direct gyro injection which has a ~0.8°/s bias at rest |

`two_d_mode: true` forces the EKF to treat the robot as planar (ignores roll, pitch, Z). `use_control: true` feeds the `ackermann_cmd` as a motion model prediction step to smooth out the estimate between sensor ticks.

### slam_toolbox

`slam_toolbox` runs in online async mode on `/scan_fixed`, building an occupancy grid and publishing it on `/map` with `TRANSIENT_LOCAL` QoS (new subscribers receive the last map immediately on connection). It adds a pose graph node when the robot moves ≥ 0.1 m or rotates ≥ 0.2 rad. The node is held for 10 seconds after the SLAM launch via a `TimerAction` to give the EKF time to establish the `odom→base_link` TF before the first scan lookup — without this delay, `slam_toolbox` drops the first several scans due to a missing TF.

---

## Pico Hardware Bridge — LED Strip + Camera Servo

### Firmware (`firmware/pico_hardware_control/`)

The Pico 2W runs Arduino firmware that listens for newline-terminated ASCII commands over USB CDC serial at 115200 baud and replies with `ACK` or `PONG`. Two hardware outputs are controlled:

- **WS2812B LED strip** (GPIO 2) via Adafruit NeoPixel — four states mapped to rover status
- **Camera pan servo** (GPIO 3) — PWM position control in degrees (0–180)

| Command | Effect |
|---------|--------|
| `PING` | Responds `PONG` — used for connection health checks |
| `LED:SUCCESS` | Strip → green (rover idle) |
| `LED:UNKNOWN` | Strip → blue (rover moving) |
| `LED:FAILURE` | Strip → red (startup error / unexpected shutdown) |
| `LED:OFF` | Strip → off (clean shutdown) |
| `C_SERVO:CENTER` | Pan camera to configured center angle |
| `C_SERVO:<deg>` | Pan camera to angle in degrees |

### ROS driver (`pico_hw_bridge` package)

`pico_hw_server` opens the serial port at startup with up to 10 retry attempts (1 s apart), then waits 2 seconds for the Pico to finish its USB CDC reboot cycle before serving requests. It exposes the Pico's command set as ROS 2 services:

| Service | Type | Effect |
|---------|------|--------|
| `/pico/ping` | `std_srvs/Trigger` | Send `PING`, verify `PONG` |
| `/pico/led_success/failure/unknown/off` | `std_srvs/Trigger` | Set LED state |
| `/pico/set_led` | `pico_interfaces/SetLed` | Set LED by string status |
| `/pico/camera_center` | `std_srvs/Trigger` | Center the camera servo |
| `/pico/set_camera_angle` | `pico_interfaces/SetCameraAngle` | Pan to specific angle |

`led_state_monitor` subscribes to `/ackermann_cmd` and calls `/pico/led_success` (green) when speed ≈ 0 or `/pico/led_unknown` (blue) when `|speed| > 0.01 m/s`, giving a real-time visual indicator of rover motion.

The `stage1_start.sh` cleanup trap calls `/pico/led_off` on clean exit or `/pico/led_failure` if startup never completed, so the LED always reflects the last known state even after the ROS graph shuts down.

---

## Web Controller and Dashboard

### Architecture

```
[OAK-D Pro]  →  camera_node  →  /survey_camera/image_raw
                                        ↓
                               web_bridge_node (:8080)
                                 │  GET  /video        MJPEG stream
                                 │  GET  /map_image    OccupancyGrid → PNG
                                 │  POST /drive        → /ackermann_cmd
                                 │  POST /capture      → /survey_camera/capture
                                 │  POST /camera_angle → /pico/set_camera_angle
                                 │  POST /save_map     → saves PNG to public/
                                 ↓
[slam_toolbox]  →  /map (TRANSIENT_LOCAL)
[ekf_local]     →  map→base_link TF

[Next.js dashboard]  →  browser  →  :3000
```

### Camera node (`survey_camera/camera_node.py`)

Uses the DepthAI v3 API to stream frames from the OAK-D Pro at ~30 Hz and publishes them as `sensor_msgs/Image` on `/survey_camera/image_raw`. It also exposes a `/survey_camera/capture` service that saves a JPEG snapshot to `mission_report/public/captures/inspection_<timestamp>.jpg`, making it directly accessible at `http://<Pi-IP>:3000/captures/<filename>`.

### Web bridge node (`survey_camera/web_bridge_node.py`)

A `ThreadingHTTPServer` runs on port 8080 in a background thread alongside the ROS node. The ROS node subscribes to the camera image and `/map` topics, storing the latest JPEG frame and `OccupancyGrid` in memory under locks. The HTTP threads read from those shared buffers.

**MJPEG stream** (`/video`): Loops over the latest JPEG frame, writing multipart HTTP chunks. Each browser connection holds this connection open indefinitely.

**Map image** (`/map_image`): On each GET, the bridge renders the current `OccupancyGrid` into a colour-coded PNG (slate-200 for free, slate-950 for occupied, slate-600 for unknown) at 4× pixel scale. It then looks up the `map→base_link` TF and overlays the robot's position as a blue dot with a heading arrow. Returned as `image/png` with `no-cache` headers.

**Drive** (`POST /drive`): Converts `{linear_x, angular_z}` JSON into an `AckermannDriveStamped` message published directly on `/ackermann_cmd`. The frontend repeats key-held commands every 100 ms to keep the ackermann mux alive (it drops commands after 500 ms of silence).

**Camera angle** (`POST /camera_angle`): Calls `pico/set_camera_angle` as a fire-and-forget async service call to avoid blocking the HTTP thread.

### Next.js dashboard (`mission_report/`)

Built with Next.js, React 19, and Tailwind CSS v4. The backend URL is derived from `window.location.hostname` at runtime, so the same build works whether accessed from the Pi locally or remotely over the network.

| Control | Action |
|---------|--------|
| W / S | Forward / reverse at 0.3 m/s |
| A / D | Steer left / right at 0.6 rad/s |
| Space | Emergency stop |
| J / L | Pan camera left / right |
| Enter | Capture inspection photo |

The SLAM map panel polls `/map_image` every 500 ms and renders the PNG inline, showing the live occupancy grid with the robot's position and heading as the rover explores.
