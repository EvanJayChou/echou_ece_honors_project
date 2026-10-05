# <div align="center">Project Reproduction Guide</div>

This guide walks through reproducing the Scout Survey Rover from scratch: OS setup, firmware flashing, ROS 2 workspace build, udev rules, and first launch.

---

## Table of Contents

1. [Hardware requirements](#1-hardware-requirements)
2. [OS setup — Ubuntu 24.04 native on Raspberry Pi 5](#2-os-setup--ubuntu-2404-native-on-raspberry-pi-5)
3. [ROS 2 Jazzy installation](#3-ros-2-jazzy-installation)
4. [System dependencies](#4-system-dependencies)
5. [Clone the repository](#5-clone-the-repository)
6. [udev rules — persistent device names](#6-udev-rules--persistent-device-names)
7. [Firmware — XIAO nRF52840 Sense (IMU)](#7-firmware--xiao-nrf52840-sense-imu)
8. [Firmware — Raspberry Pi Pico 2W (LED strip + servo)](#8-firmware--raspberry-pi-pico-2w-led-strip--servo)
9. [VESC configuration](#9-vesc-configuration)
10. [Build the ROS 2 workspace](#10-build-the-ros-2-workspace)
11. [Node.js 20 — mission report web server](#11-nodejs-20--mission-report-web-server)
12. [First launch](#12-first-launch)
13. [Verifying the stack](#13-verifying-the-stack)
14. [Calibration](#14-calibration)

---

## 1. Hardware requirements

See [robot_hardware.md](robot_hardware.md) for the full parts list and USB device assignments. All hardware from the **Final project additions** section is required for full stack operation.

---

## 2. OS setup — Ubuntu 24.04 native on Raspberry Pi 5

We run Ubuntu 24.04 natively, **not** inside Docker. This avoids the overhead of the UCSD RoboCar Hub2 Docker image and gives direct hardware access.

### Flash the SD card

1. Download the [Raspberry Pi Imager](https://www.raspberrypi.com/software/).
2. Select **Other general-purpose OS → Ubuntu → Ubuntu Server 24.04 LTS (64-bit)**.
3. Click the gear icon → set hostname, username (`rover`), password, SSH, and Wi-Fi SSID before writing.
4. Write to the SD card.

### First boot

```bash
# On first SSH connection:
sudo apt update && sudo apt full-upgrade -y
sudo reboot
```

### Enable serial ports (disable Bluetooth UART conflict)

The Pi 5 does not have the same UART/Bluetooth conflict as the Pi 4, but confirm serial devices appear after boot:

```bash
ls /dev/ttyACM*        # should list XIAO, VESC, Pico after all three are plugged in
ping -c 1 192.168.1.12 # Livox Mid-360, after network setup (section 6)
```

---

## 3. ROS 2 Jazzy installation

Follow the [official ROS 2 Jazzy installation guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html) for Ubuntu 24.04. Abbreviated:

```bash
# Locale
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# ROS 2 apt source
sudo apt install -y software-properties-common curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list

sudo apt update
sudo apt install -y ros-jazzy-desktop ros-dev-tools
```

Add to `~/.bashrc`:

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 4. System dependencies

### Python packages

```bash
pip3 install pyserial flask opencv-python-headless
```

> `opencv-python-headless` is used by `cv_bridge` and the web bridge. If you need GUI windows (e.g. for debugging), use `opencv-python` instead, but avoid it on the Pi to save RAM.

### ROS 2 packages (apt)

```bash
sudo apt install -y \
    ros-jazzy-robot-localization \
    ros-jazzy-slam-toolbox \
    ros-jazzy-nav2-map-server \
    ros-jazzy-ackermann-msgs \
    ros-jazzy-cv-bridge \
    ros-jazzy-image-transport \
    ros-jazzy-tf2-ros \
    ros-jazzy-tf2-geometry-msgs
```

### DepthAI (OAK-D camera)

```bash
pip3 install depthai
# USB rules for OAK-D:
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' \
    | sudo tee /etc/udev/rules.d/80-movidius.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

### nvm (Node.js version manager) — needed for web server

```bash
curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.7/install.sh | bash
source ~/.bashrc
nvm install 20
nvm use 20
node --version   # should print v20.x.x
```

---

## 5. Clone the repository

```bash
cd ~
git clone https://github.com/UCSD-ECEMAE-148/148sp26-spring-2026-final-project-team-4.git scout-survey-rover
cd scout-survey-rover
```

All paths in launch files and scripts assume the repo lives at `~/scout-survey-rover`. If you clone elsewhere, update `SCRIPT_DIR` references in `stage1_start.sh` and the capture path in `survey_camera/camera_node.py`.

---

## 6. Livox Mid-360 — network setup and driver build

The Mid-360 replaced the LD06 LiDAR and connects over Ethernet rather than USB-serial, so
there's no udev rule for it — instead, statically assign the Pi's Ethernet interface:

```bash
sudo ip addr add 192.168.1.50/24 dev eth0   # make persistent via netplan/NetworkManager
ping 192.168.1.12                            # confirm the sensor (default IP) is reachable
```

`livox_ros_driver2` (vendored as a git submodule at `ros2_ws/src/livox_ros_driver2`) depends on
the Livox-SDK2 C++ library, which is not packaged for apt and must be built and installed once:

```bash
git clone https://github.com/Livox-SDK/Livox-SDK2.git ~/Livox-SDK2
cd ~/Livox-SDK2 && mkdir build && cd build
cmake .. && make -j$(nproc)
sudo make install
```

Also install `pointcloud_to_laserscan`, which `livox_bringup` uses to flatten the Mid-360's
point cloud into the `/scan` `LaserScan` the rest of the SLAM stack expects:

```bash
sudo apt install ros-jazzy-pointcloud-to-laserscan
```

Init the submodule after cloning this repo (or `git submodule update --init` if already cloned):

```bash
git submodule update --init ros2_ws/src/livox_ros_driver2
```

### VESC

```bash
sudo cp slam_ros2_ws/src/vesc/vesc_driver/scripts/99-vesc6.rules /etc/udev/rules.d/99-vesc6.rules
```

### Apply rules

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

### Add user to `dialout` group

Required for read/write access to all serial devices without `sudo`:

```bash
sudo usermod -aG dialout $USER
# Log out and back in (or reboot) for group membership to take effect.
```

### Verify

Plug in all USB devices, then:

```bash
ls -la /dev/ttyACM0 /dev/ttyACM1 /dev/ttyACM2
ping -c 1 192.168.1.12   # Livox Mid-360
```

---

## 7. Firmware — XIAO nRF52840 Sense (IMU)

The XIAO runs custom firmware (`xiao_odom`) that reads the onboard LSM6DS3 IMU, performs gyro bias calibration at startup, integrates accelerometer and gyroscope data for dead-reckoning, and streams CSV packets at ~50 Hz over USB CDC serial.

### Install Arduino IDE

Download [Arduino IDE 2.x](https://www.arduino.cc/en/software) on your development machine (not the Pi).

### Add Seeed nRF52840 board support

1. Open Arduino IDE → **File → Preferences**.
2. Add to **Additional boards manager URLs**:
   ```
   https://files.seeedstudio.com/arduino/package_seeeduino_boards_index.json
   ```
3. **Tools → Board → Boards Manager** → search `Seeed nRF52` → install **Seeed nRF52 Boards**.

### Install required libraries

**Tools → Manage Libraries** → install:

| Library | Version |
|---------|---------|
| `Seeed Arduino LSM6DS3` | latest |
| `Adafruit LSM6DS` | latest (fallback if above is unavailable) |

### Flash the firmware

1. Open `slam_ros2_ws/src/xiao_serial_bridge/firmware/xiao_odom/xiao_odom.ino` in Arduino IDE.
2. **Tools → Board** → select **Seeed XIAO nRF52840 Sense**.
3. **Tools → Port** → select the XIAO's COM/tty port.
4. Click **Upload**.

### Verify firmware output

After flashing, open the **Serial Monitor** (baud rate: `115200`). You should see:

```
Calibrating gyro bias... (hold still for 10 s)
Gyro bias: bx=0.0012 by=-0.0008 bz=0.0003
0.000,0.000,0.000,0.000,0.000,0.000,0.000,0.000,0.000
```

The CSV columns are: `ax, ay, az, gx, gy, gz, x, y, theta` at ~50 Hz. Hold the XIAO still for the full 10-second calibration — movement during calibration degrades odometry accuracy.

### Serial packet format

The ROS bridge (`serial_bridge_node.py`) expects this exact CSV format on each line:

```
<ax>,<ay>,<az>,<gx>,<gy>,<gz>,<x>,<y>,<theta>
```

All values are SI units: m/s², rad/s, m, m, rad. The bridge computes velocities from consecutive poses and publishes `/odom` (Odometry) and `/imu` (Imu) at the packet rate.

---

## 8. Firmware — Raspberry Pi Pico 2W (LED strip + servo)

The Pico runs firmware that listens for text commands over USB CDC serial at 115200 baud and controls a WS2812B LED strip and a camera pan servo.

### Install Arduino IDE board support for Pico 2W

1. **File → Preferences** → **Additional boards manager URLs**, add:
   ```
   https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json
   ```
2. **Tools → Board → Boards Manager** → search `Raspberry Pi Pico` → install **Raspberry Pi RP2040 Boards** by Earle Philhower.

### Install required libraries

**Tools → Manage Libraries** → install:

| Library | Notes |
|---------|-------|
| `Adafruit NeoPixel` | WS2812B LED strip |

### Hardware wiring

| Pico pin | Connected to |
|----------|-------------|
| GPIO 0 (TX) | Not used for serial — USB CDC is used |
| GPIO 2 | WS2812B LED strip data line |
| GPIO 3 | Camera pan servo signal |
| 3V3 | Servo power (if drawing < 500 mA; otherwise use external 5 V) |
| GND | Shared ground |

Update `firmware/pico_hardware_control/config.h` if your wiring differs:

```cpp
#define LED_PIN        2
#define LED_COUNT      30      // number of WS2812B LEDs on your strip
#define SERVO_PIN      3
#define SERVO_CENTER   90      // degrees — adjust until camera points forward
#define BAUD_RATE      115200
```

### Flash the firmware

1. Open `firmware/pico_hardware_control/pico_hardware_control.ino` in Arduino IDE.
2. **Tools → Board** → select **Raspberry Pi Pico 2W**.
3. Hold the **BOOTSEL** button on the Pico while plugging it in via USB — it mounts as a USB mass storage device.
4. Click **Upload** (Arduino IDE handles the UF2 transfer automatically).

### Verify firmware output

Open the Serial Monitor (115200 baud) and send:

```
LED:SUCCESS
```

The LED strip should turn green. Other valid commands:

| Command | LED state |
|---------|-----------|
| `LED:SUCCESS` | Green — idle |
| `LED:UNKNOWN` | Blue — moving |
| `LED:FAILURE` | Red — error |
| `LED:OFF` | Off |
| `SERVO:CENTER` | Pan camera to center position |
| `SERVO:<deg>` | Pan camera to angle in degrees (0–180) |
| `PING` | Responds `PONG` |

The `pico_hw_server` ROS node communicates via these exact commands over the USB serial port at 115200 baud.

---

## 9. VESC configuration

The VESC must have servo output enabled before the ROS driver can control steering.

1. Download [VESC Tool](https://vesc-project.com/vesc_tool) on your laptop.
2. Connect the VESC via USB.
3. **App Settings → General → Use Servo Output** → enable → **Write App Configuration**.
4. Verify motor direction and steering direction with VESC Tool's built-in test controls.
5. Note the firmware version shown on connection — the driver expects VESC firmware ≥ 5.x.

The VESC driver connects to `/dev/ttyACM1` at runtime. If the VESC enumerates on a different port, either re-plug in the correct order or set the `port` parameter in `ucsd_robocar_control2_pkg/config/vesc_config.yaml`.

---

## 10. Build the ROS 2 workspace

```bash
cd ~/scout-survey-rover/slam_ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

`--symlink-install` means edits to Python nodes and launch files take effect immediately without rebuilding. C++ packages still need a rebuild after source changes.

### Add workspace source to `.bashrc`

```bash
echo "source ~/scout-survey-rover/slam_ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Verify the build

```bash
ros2 pkg list | grep -E "xiao_serial_bridge|pico_hw_bridge|robot_slam|survey_camera|pico_interfaces"
```

All five packages should appear.

### Common build failures

**`robot_localization` build conflict** — if a locally-built version of `robot_localization` conflicts with the system-installed one:

```bash
rm -rf build/robot_localization install/robot_localization
colcon build --symlink-install --packages-skip robot_localization
```

The system package installed via apt in section 4 is used instead.

**Missing `pico_interfaces`** — `pico_hw_bridge` depends on the custom service definitions in `pico_interfaces`. Build `pico_interfaces` first if you need to build selectively:

```bash
colcon build --packages-select pico_interfaces
colcon build --packages-select pico_hw_bridge
```

---

## 11. Node.js 20 — mission report web server

```bash
cd ~/scout-survey-rover/mission_report
nvm use 20
npm install
```

Test that the dev server starts:

```bash
npm run dev
# Open http://localhost:3000 — should load the dashboard UI
# Ctrl+C to stop
```

In production (or when running headless), the `--full` flag on `stage1_start.sh` starts this automatically.

---

## 12. First launch

With all firmware flashed, udev rules installed, workspace built, and Node.js installed:

```bash
cd ~/scout-survey-rover

# Keyboard teleop mode (interactive terminal)
./stage1_start.sh

# Headless mode (drive via web dashboard)
./stage1_start.sh --headless

# Full stack — SLAM + survey camera + web server (recommended for normal use)
./stage1_start.sh --full
```

The script runs preflight hardware checks and will warn (but not abort) if devices are missing. Allow ~20 seconds for `slam_toolbox` to activate and begin mapping.

**Web dashboard** (when using `--full`): open `http://<Pi-IP>:3000` from any browser on the same network. Find the Pi's IP with `hostname -I`.

---

## 13. Verifying the stack

See [full_stage_testing.md](full_stage_testing.md) for the complete verification and troubleshooting reference. Quick checks:

```bash
# All nodes running
ros2 node list

# Sensor rates
ros2 topic hz /scan             # ~10 Hz
ros2 topic hz /odom             # ~50 Hz
ros2 topic hz /odometry/filtered  # ~10 Hz (EKF output)
ros2 topic hz /sensors/core     # ~50 Hz (VESC telemetry)

# Map building (use service, not topic hz)
ros2 service call /slam_toolbox/dynamic_map nav_msgs/srv/GetMap
```

---

## 14. Calibration

### Steering offset

The servo center value is the most common thing to tune per-robot:

```bash
# Find the value that centers the wheels visually:
ros2 topic pub /commands/servo/position std_msgs/msg/Float64 "data: 0.5" --once
# Adjust 0.5 up/down until wheels are straight
```

Edit `ucsd_robocar_control2_pkg/config/keyboard_teleop_config.yaml`:

```yaml
steering_angle_to_servo_gain:   -0.6   # flip sign if steering direction is reversed
steering_angle_to_servo_offset:  0.5   # servo value (0.0–1.0) that centers the wheels
```

Rebuild after editing:
```bash
colcon build --packages-select ucsd_robocar_control2_pkg
```

### IMU gyro bias calibration

Bias calibration runs automatically for 10 seconds every time the XIAO firmware boots. Place the rover on a flat, still surface before powering on, and avoid moving it until the bridge reports calibration complete in the `/tmp/stage1_imu.log`.

### SLAM parameters

EKF and SLAM tuning parameters live in `slam_ros2_ws/src/robot_slam/config/`:

| File | What it controls |
|------|-----------------|
| `ekf_local.yaml` | Process noise, sensor covariances, fusion weights |
| `slam_toolbox.yaml` | Map resolution, loop closure thresholds, scan matching |
| `nav2_params.yaml` | Nav2 costmap and planner (not active in Stage 1) |

Changes to these files take effect immediately without rebuilding (symlink install). Restart the stack after editing.
