# SLAM Debugging Log & Lessons Learned

> **Historical record.** This log was written for the LD06 LiDAR + XIAO nRF52840 IMU setup on a Raspberry Pi 5 (ROS 2 Jazzy) and the old `slam_ros2_ws` layout. The rover now runs ROS 2 Jazzy on a Jetson Orin Nano (JetPack 7.2). Since then the LiDAR is a Livox Mid-360, the IMU is the OAK-D Pro's onboard IMU, `scan_relay_node` lives in `robot_slam`, and `slam.launch.py` was folded into `onboard.launch.py use_slam:=true`. The root-cause notes (fixed beam count, TF timing, single scan relay) still apply. For current procedures see [ros2_testing.md](../ros2_testing.md).

This document records every significant finding, change, and lesson from bringing the Stage 1 SLAM pipeline to a working state. It is intended as a reference for future debugging and onboarding.

---

## System Overview (at time of debugging)

```
[LD06 LiDAR]  →  /scan  →  scan_relay_node  →  /scan_fixed
[XIAO IMU]    →  /odom, /imu  →  ekf_local_node  →  odom→base_link TF
[robot_state_publisher]  →  base_link→laser TF (static)
slam_toolbox  →  /map  +  map→odom TF
```

ROS 2 distro: **Jazzy** on Raspberry Pi 5 (arm64, Ubuntu 24.04).

---

## Changes Made

### 1. `scan_relay_node` — full rewrite

**File:** `slam_ros2_ws/src/xiao_serial_bridge/xiao_serial_bridge/scan_relay_node.py`

**Before:** Simple passthrough relay that forwarded `/scan` to `/scan_fixed` with a fixed beam count resampler. No FOV filtering.

**After:** Added a 250° front-FOV crop with wraparound handling, then nearest-neighbour resample to a fixed beam count.

Key parameters added:

| Parameter | Default | Purpose |
|-----------|---------|---------|
| `target_beams` | 300 | Output beam count after resampling |
| `fov_deg` | 250.0 | FOV to keep, centred on `fov_center_deg` |
| `fov_center_deg` | 0.0 | Centre of FOV in degrees (0 = robot forward) |

Algorithm:
1. Normalize each beam's raw angle to `[-π, π]` using: `(raw - center + π) % 2π - π`
2. Discard beams outside `±half_fov`
3. Sort remaining beams by normalized angle (handles 0→2π wraparound cleanly)
4. Nearest-neighbour resample to `target_beams`

**Why 250° and not 360°:** Rear 110° is occluded by the rover body and only adds noise to SLAM. Fewer beams also reduces slam_toolbox CPU load on the Pi.

---

### 2. `slam.launch.py` — scan relay parameters

**File:** `slam_ros2_ws/src/robot_slam/launch/slam.launch.py`

Updated the `scan_relay_node` parameters block from:
```python
parameters=[{'target_beams': 450}]
```
to:
```python
parameters=[{
    'target_beams': 300,
    'fov_deg': 250.0,
    'fov_center_deg': 0.0,
}]
```

The `TimerAction` (10 s delay before slam_toolbox starts) was already in place from prior work and was left unchanged — it is load-bearing (see Lessons Learned §2).

---

### 3. IMU bridge — replaced `razor_imu_9dof` with `xiao_serial_bridge`

**Old approach:** `ros2 launch razor_imu_9dof imu_razor_m0.launch.py` — a generic IMU bridge not matched to the XIAO hardware or CSV packet format.

**New approach:** `ros2 run xiao_serial_bridge serial_bridge_node` — custom bridge that parses the exact CSV format the XIAO firmware outputs, auto-detects the device by VID/PID (0x2886:0x8045), and recovers automatically after USB replug.

---

### 4. Documentation rewrites

- `docs/ros2_testing.md` — rewritten to cover correct IMU bridge command, SLAM architecture diagram, 3-terminal launch sequence, `/map` QoS explanation, expected log output, and diagnostics.
- `docs/full_stage_testing.md` — rewritten as a 9-step end-to-end Stage 1 test guide with explicit verify steps, RViz option, and diagnostics section.

---

## Bugs Found and Fixed

### Bug 1: `/map` appeared unpublished — false alarm

**Symptom:** `ros2 topic hz /map` printed "not published" even after slam_toolbox was clearly running and logging pose updates.

**Root cause:** slam_toolbox publishes `/map` with **TRANSIENT_LOCAL QoS**. Any subscriber that joins after the first publish — including `ros2 topic hz`, `ros2 topic echo` without explicit QoS flags — misses all messages and reports nothing. This is a ROS 2 QoS mismatch, not a slam_toolbox failure.

**Fix:** Use the service to verify map data:
```bash
ros2 service call /slam_toolbox/dynamic_map nav_msgs/srv/GetMap
```
A valid response has `width` and `height` > 0 and a `data` array with `-1`, `0`, and `100` values.

**Lesson:** Never use `ros2 topic hz` to confirm a TRANSIENT_LOCAL topic is alive. Always check via service or `--qos-durability transient_local` flag.

---

### Bug 2: Stale launch file — beam count not updating

**Symptom:** After changing `slam.launch.py` to request 300 beams, the launch terminal still showed "450 beams." Map was receiving 450-beam scans.

**Root cause:** `colcon build --symlink-install` symlinks Python source files but **copies** launch files for non-Python packages. After editing `slam.launch.py` in the source tree, the installed copy at `install/robot_slam/share/robot_slam/launch/slam.launch.py` was still the old version.

**Fix:** Rebuild the affected package:
```bash
colcon build --packages-select robot_slam --symlink-install
```

**Lesson:** `--symlink-install` only symlinks Python node source files. Launch files in ament packages (non-Python executables) must be rebuilt to propagate to `install/`. When a launch change seems to have no effect, always check `install/<pkg>/share/<pkg>/launch/` directly.

---

### Bug 3: Duplicate `scan_relay_node` — beam-count mismatch in slam_toolbox

**Symptom:** slam_toolbox logs: `LaserRangeScan contains 450 range readings, expected 300` (or vice versa). Appeared after SLAM had been running correctly, following a manual launch of `scan_relay_node` in a separate terminal.

**Root cause:** Two `scan_relay_node` instances were running simultaneously:
- One started manually (PID 7586/7593, 450 beams — old default)
- One started by the launch (PID 7775, 300 beams — new)

Both published to `/scan_fixed`. slam_toolbox registered on the first scan it received (300 beams), then rejected all subsequent scans from the other instance (450 beams) as size mismatches.

**Fix:** Kill the manually-started instances and leave the launch-managed one:
```bash
pkill -9 -f scan_relay_node
# Then restart Step 4 (launch) only — LiDAR and IMU bridge can stay running
```

**Lesson:** Never start a node manually if the launch already starts it. The duplicate causes interleaved, inconsistent message streams. Document this warning prominently in the run guides (done).

---

### Bug 4: Message filter drop at slam_toolbox startup — expected, not a bug

**Symptom:** `[slam_toolbox]: Message Filter dropping message: frame 'laser' at time X for reason 'the timestamp on the message is earlier than all the data in the transform cache'`

**Root cause:** slam_toolbox starts and immediately tries to look up the `laser` frame in the TF buffer. At `t=0` the buffer is momentarily empty. The very first scan is dropped.

**Status:** This is **normal** and **expected**. The 10 s `TimerAction` delay ensures EKF has been publishing `odom→base_link` TF for 10 s before slam_toolbox activates, so the buffer is populated for all scans after the first.

**When it is a real problem:** If the drop repeats every few seconds after startup, the TF chain is broken (EKF not publishing, robot_state_publisher crashed, or `base_link→laser` static TF missing). Check:
```bash
ros2 topic echo /tf --once | grep -E "frame_id|child_frame"
ros2 topic echo /tf_static --once | grep child_frame
```

---

### Bug 5: `robot.urdf.xacro` parsed without xacro processor

**Symptom (potential):** `robot_state_publisher` was given the raw `.xacro` file content via `open(...).read()` — not processed through the xacro tool.

**Finding:** The file uses the `xmlns:xacro` namespace declaration but contains **no actual xacro macros**. It is valid URDF XML. `robot_state_publisher` parsed it correctly and published all static TFs (`base_link→laser`, `base_link→camera_link`, `base_link→base_imu_link`).

**Risk:** If xacro macros are ever added to the URDF, the launch will silently break because raw file content will no longer be valid XML. Fix preemptively by replacing `open(...).read()` with a proper xacro subprocess call if macros are added.

---

## Lessons Learned

### 1. TRANSIENT_LOCAL topics require service-call verification

`/map`, and potentially other slam_toolbox topics, use TRANSIENT_LOCAL durability. Standard `ros2 topic hz` and `ros2 topic echo` use VOLATILE and will always show nothing unless they were subscribed before the first publish. Always verify these topics via their corresponding service.

### 2. The 10 s `TimerAction` for slam_toolbox is load-bearing

Do not remove or shorten the delay. slam_toolbox looks up TF immediately on activation. If `odom→base_link` is not yet in the buffer, every scan is dropped and the map never builds. On the Pi, EKF takes a few seconds to stabilise after startup — 10 s gives comfortable margin.

### 3. `--symlink-install` does not symlink launch files

Only Python source files (`.py` nodes) are symlinked. Launch files, YAML configs, URDF files, and other share-directory resources are copied. After editing them, rebuild the package. A quick check: `diff src/<pkg>/launch/foo.launch.py install/<pkg>/share/<pkg>/launch/foo.launch.py`.

### 4. LD06 angle convention

The LD06 outputs angles from `0` to `2π` counterclockwise, with `0` pointing forward along the robot x-axis (confirmed from URDF `rpy="0 0 0"`). The FOV filter uses `fov_center_deg=0.0` to keep the front arc. This is not obvious from the LD06 datasheet — the frame convention came from the URDF, not the sensor docs.

### 5. Nav2 disabled for Stage 1 — intentional

Running Nav2 (controller_server, planner_server, etc.) alongside SLAM on the Pi saturated CPU, causing EKF TF lag and slam_toolbox crashes. Nav2 is gated behind `use_nav2:=false` (default). Do not re-enable until SLAM map generation is stable and a CPU budget analysis has been done.

### 6. RViz is CPU-heavy on the Pi

`use_rviz:=true` works but competes with SLAM and EKF for CPU. If the map stalls or EKF lag increases when RViz is open, run RViz on a separate machine on the same network with the same `ROS_DOMAIN_ID`.

---

## Current Working State (as of 2026-06-05)

| Component | Status | Notes |
|-----------|--------|-------|
| `serial_bridge_node` | Working | `/odom` 50 Hz, `/imu` 50 Hz; VID/PID auto-detect |
| `scan_relay_node` | Working | 300 beams, 250° front FOV, single instance |
| `ekf_local_node` | Working | `odom→base_link` TF at 10 Hz |
| `robot_state_publisher` | Working | `base_link→laser` static TF |
| `slam_toolbox` | Working | Map confirmed via `dynamic_map` service; one startup drop normal |
| XIAO firmware | Needs flash | `while (!Serial) {}` → `while (!Serial && millis() < 5000) {}` fix written but not yet flashed — requires DFU mode (double-press reset) |

---

## Pending Items

- **Flash updated XIAO firmware**: The `while (!Serial) {}` hang fix was written in a prior session but never flashed. The XIAO must be put into DFU mode by double-pressing the reset button before `arduino-cli upload` will work. Without this fix, the bridge node cannot open the serial port until the XIAO's USB CDC is up — the bridge's retry loop compensates but adds startup delay.
