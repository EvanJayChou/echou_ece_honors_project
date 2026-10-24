# <div align="center">Project Reproduction Guide</div>

This guide walks through reproducing the Scout Survey Rover from scratch: OS setup, ROS 2 and sensor SDK installation, workspace build, udev and network setup, and first launch.

> **Prefer containers?** [docker.md](docker.md) replaces sections 3, 4, the SDK build in 6, and 9 with a single image build. You still need JetPack 7.2 (§2), the Livox network link (§6), udev rules (§7) and VESC configuration (§8) on the host.

---

## Table of Contents

1. [Hardware requirements](#1-hardware-requirements)
2. [OS setup — JetPack 7.2 (Ubuntu 24.04) on Jetson Orin Nano](#2-os-setup--jetpack-72-ubuntu-2404-on-jetson-orin-nano)
3. [ROS 2 Jazzy installation](#3-ros-2-jazzy-installation)
4. [System dependencies](#4-system-dependencies)
5. [Clone the repository](#5-clone-the-repository)
6. [Livox Mid-360 — network setup and SDK](#6-livox-mid-360--network-setup-and-sdk)
7. [udev rules — VESC and OAK-D Pro](#7-udev-rules--vesc-and-oak-d-pro)
8. [VESC configuration](#8-vesc-configuration)
9. [Build the ROS 2 workspaces](#9-build-the-ros-2-workspaces)
10. [First launch](#10-first-launch)
11. [Calibration](#11-calibration)

---

## 1. Hardware requirements

See [robot_hardware.md](robot_hardware.md) for the full parts list and USB device assignments. The rover stack needs the Livox Mid-360, the OAK-D Pro, and the VESC. Frontier exploration also needs a second (GPU) machine on the same network.

---

## 2. OS setup — JetPack 7.2 (Ubuntu 24.04) on Jetson Orin Nano

The onboard computer is an NVIDIA Jetson Orin Nano running **JetPack 7.2.x** (Jetson Linux / L4T 39.2.x), which is Ubuntu 24.04. The ROS 2 release for 24.04 is **Jazzy**. (JetPack 6 is Ubuntu 22.04 and can't install Jazzy from apt; on a JetPack 6 host use the CPU-only container build in [docker.md](docker.md#2-build-the-image) instead.) Everything runs natively, **not** inside Docker. This avoids the overhead of the UCSD RoboCar Hub2 Docker image and gives direct hardware access.

### Flash JetPack 7.2

Follow NVIDIA's [Jetson Orin Nano Developer Kit Getting Started guide](https://developer.nvidia.com/embedded/learn/get-started-jetson-orin-nano-devkit): use the JetPack 7.2 image, or SDK Manager for NVMe. The QSPI bootloader firmware has to match the release. A dev kit still on JetPack 5 or 6 firmware needs a firmware update before a JetPack 7.2 image will boot. The guide covers this. Moving from JetPack 6 is a full reflash, not an in-place `apt` upgrade.

During first-boot setup, create the user (`rover`), set the hostname, and enable SSH.

### First boot

```bash
cat /etc/nv_tegra_release      # R39.x → JetPack 7.2
lsb_release -rs                # 24.04
sudo apt update && sudo apt full-upgrade -y
sudo nvpmodel -q               # check the power mode; pick the highest one (MAXN) for SLAM
sudo reboot
```

Confirm devices appear after boot:

```bash
ls /dev/ttyACM*        # VESC, once plugged in
ping -c 1 192.168.1.12 # Livox Mid-360, after network setup (section 6)
```

---

## 3. ROS 2 Jazzy installation

Follow the [official ROS 2 Jazzy installation guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debians.html) for Ubuntu 24.04 (arm64 is Tier 1). Abbreviated:

```bash
# Locale
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# ROS 2 apt source
sudo apt install -y software-properties-common curl
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list

sudo apt update
sudo apt install -y ros-jazzy-ros-base ros-dev-tools   # ros-jazzy-desktop if you want RViz on the Jetson
```

Add to `~/.bashrc`:

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

The GPU machine must also run **Jazzy** (Ubuntu 24.04). Message definitions changed between Humble and Jazzy (Nav2 actions in particular), so a Humble GPU machine can't drive a Jazzy rover. Don't mix distros between the two machines.

---

## 4. System dependencies

### ROS 2 packages (apt)

On the rover:

```bash
sudo apt install -y \
    ros-jazzy-robot-localization \
    ros-jazzy-slam-toolbox \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-pointcloud-to-laserscan \
    ros-jazzy-pcl-ros \
    ros-jazzy-ackermann-msgs \
    ros-jazzy-joy \
    ros-jazzy-cv-bridge \
    ros-jazzy-tf2-ros \
    ros-jazzy-nav2-map-server \
    ros-jazzy-nav2-lifecycle-manager \
    ros-jazzy-nav2-controller \
    ros-jazzy-nav2-velocity-smoother \
    ros-jazzy-nav2-regulated-pure-pursuit-controller \
    ros-jazzy-nav2-costmap-2d \
    python3-serial python3-pip
```

On the GPU machine (Ubuntu 24.04 + ROS 2 Jazzy):

```bash
sudo apt install -y ros-jazzy-navigation2 ros-jazzy-slam-toolbox ros-jazzy-rviz2
```

`rosdep install --from-paths src --ignore-src -r -y` in either workspace catches anything missing from these lists.

### DepthAI v3 (OAK-D Pro)

`oakd_publisher` uses the DepthAI **v3** API (`dai.Pipeline(device)`, `Camera.build()`, `requestOutput()`). It doesn't run on depthai 2.x. Pin NumPy below 2: Jazzy's `cv_bridge` is compiled against Ubuntu 24.04's NumPy 1.26 and fails to import (`_ARRAY_API not found`) if pip upgrades it.

Ubuntu 24.04 marks the system Python as externally managed (PEP 668), so a plain `pip install` is refused. Install into your user site with `--break-system-packages`. ROS nodes run on the system interpreter, so a venv would hide the ROS Python packages.

```bash
pip3 install --user --break-system-packages "depthai>=3" "numpy<2"
python3 -c "import depthai, numpy, cv_bridge; print(depthai.__version__, numpy.__version__)"   # 3.x, 1.x
```

---

## 5. Clone the repository

```bash
cd ~
git clone https://github.com/UCSD-ECEMAE-148/148sp26-spring-2026-final-project-team-4.git scout-survey-rover
cd scout-survey-rover
git submodule update --init ros2_ws/src/livox_ros_driver2
```

The scripts find the repo relative to their own location, so it can live anywhere. Docs use `~/scout-survey-rover`.

---

## 6. Livox Mid-360 — network setup and SDK

The Mid-360 replaced the LD06 LiDAR and connects over Ethernet rather than USB-serial, so
it doesn't need a udev rule. Instead, statically assign the Jetson's Ethernet interface:

```bash
ip -br link                                  # find the wired interface (eth0 / enP8p1s0 on Orin Nano)
sudo nmcli con add type ethernet ifname <iface> con-name livox \
    ipv4.method manual ipv4.addresses 192.168.1.50/24 ipv4.never-default yes
sudo nmcli con up livox
ping 192.168.1.12                            # confirm the sensor (default IP) is reachable
```

`ipv4.never-default` keeps the Jetson's default route (and internet) on Wi-Fi.

If your sensor isn't at the factory IP, its address is `192.168.1.1XX`, where XX is the last two digits of its serial number. Update `ros2_ws/src/livox_bringup/config/MID360_config.json` and set `LIVOX_IP` for the scripts.

`livox_ros_driver2` depends on the Livox-SDK2 C++ library, which isn't packaged for apt and must be built and installed once:

```bash
git clone --branch v1.2.5 https://github.com/Livox-SDK/Livox-SDK2.git ~/Livox-SDK2
cd ~/Livox-SDK2 && mkdir build && cd build
cmake .. -DCMAKE_CXX_FLAGS="-include cstdint" && make -j$(nproc)
sudo make install && sudo ldconfig
```

The `-include cstdint` flag is needed on Ubuntu 24.04. Some SDK sources use `uint8_t`/`uint32_t` without including `<cstdint>`, and GCC 13 no longer includes it implicitly (`'uint8_t' was not declared in this scope`).


---

## 7. udev rules — VESC and OAK-D Pro

```bash
sudo cp udev/99-rover.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
sudo usermod -aG dialout $USER     # serial access without sudo; log out and back in
```

Verify, with the devices plugged in:

```bash
ls -l /dev/vesc                    # symlink to the VESC's ttyACM device
lsusb | grep 03e7                  # OAK-D Pro (Movidius)
```

---

## 8. VESC configuration

The VESC must have servo output enabled before the ROS driver can control steering.

1. Download [VESC Tool](https://vesc-project.com/vesc_tool) on your laptop.
2. Connect the VESC via USB.
3. **App Settings → General → Use Servo Output** → enable → **Write App Configuration**.
4. Verify motor direction and steering direction with VESC Tool's built-in test controls.
5. Note the firmware version shown on connection. The driver expects VESC firmware ≥ 5.x.

The scripts pass the VESC port to the driver (`/dev/vesc`, else `/dev/ttyACM0`; override with `VESC_PORT`).

---

## 9. Build the ROS 2 workspaces

Rover:

```bash
cd ~/scout-survey-rover
source /opt/ros/jazzy/setup.bash
(cd ros2_ws && rosdep install --from-paths src --ignore-src -r -y)
scripts/build.sh onboard
```

GPU machine:

```bash
(cd ros2_ws_offboard && rosdep install --from-paths src --ignore-src -r -y)
scripts/build.sh offboard
```

`scripts/build.sh onboard` copies `livox_ros_driver2/package_ROS2.xml` to `package.xml` (the driver ships no default manifest), passes `-DROS_EDITION=ROS2 -DDISTRO_ROS=jazzy`, and builds the packages the scripts need. Add `--all` to build every package in `ros2_ws/src`.

### Verify the build

```bash
source ros2_ws/install/setup.bash
ros2 pkg list | grep -E "^(robot_slam|livox_bringup|livox_ros_driver2|oakd_publisher|keyframe_bridge|vesc_driver|vesc_ackermann|ucsd_robocar_control2_pkg)$"
```

All eight packages should appear.

### Common build failures

- **`Could not find a package configuration file provided by "livox_sdk"`**: Livox-SDK2 isn't installed (section 6), or `ldconfig` hasn't picked it up yet (`sudo ldconfig`).
- **livox_ros_driver2 isn't built at all**: its `package.xml` is missing. Use `scripts/build.sh`, not a bare `colcon build`.
- **`'uint8_t' was not declared` while building Livox-SDK2**: rebuild it with `-DCMAKE_CXX_FLAGS="-include cstdint"` (section 6).
- **`error: externally-managed-environment` from pip**: add `--break-system-packages` (section 4).
- **`robot_localization` build conflict**: a source copy is shadowing the apt package. Remove it with `rm -rf build/robot_localization install/robot_localization`.

---

## 10. First launch

Work through the tests in order. [ros2_testing.md](ros2_testing.md) explains what to check at each step.

```bash
cd ~/scout-survey-rover
scripts/sensors.sh            # LiDAR + OAK-D (camera + IMU)
scripts/teleop.sh             # keyboard driving only
scripts/teleop_slam.sh        # keyboard driving while mapping on the Jetson
scripts/explore.sh            # + scripts/offboard.sh explore on the GPU machine
```

Keep the rover still for the first ~5 s of any script that starts the OAK-D, while the gyro bias is calibrated.

---

## 11. Calibration

### Steering offset

The servo center value is the most common thing to tune per robot:

```bash
ros2 topic pub /commands/servo/position std_msgs/msg/Float64 "data: 0.5" --once
# Adjust 0.5 up/down until wheels are straight
```

Edit `ucsd_robocar_control2_pkg/config/keyboard_teleop_config.yaml`:

```yaml
steering_angle_to_servo_gain:   -0.6   # flip sign if steering direction is reversed
steering_angle_to_servo_offset:  0.5   # servo value (0.0–1.0) that centers the wheels
```

A positive `steering_angle` must steer **left**. Nav2's `cmd_vel_to_ackermann_node` relies on it.

### Speed (eRPM gain)

Drive a measured 2 m with teleop and compare it with the distance integrated from `/vesc_odom`. Scale `speed_to_erpm_gain` in the same YAML by `odom_distance / true_distance`.

### OAK-D Pro IMU

Gyro bias calibration runs automatically for 5 s every time `oakd_publisher` starts (`gyro_bias_calibration_s`). Check the IMU axes once per mount using the procedure in [ros2_testing.md §2](ros2_testing.md#oak-d-pro-imu).

### Sensor mount poses

Measure the LiDAR and camera positions relative to `base_link` (rear-axle center at ground level is a good convention for a car), then update `ros2_ws/src/robot_slam/urdf/robot.urdf.xacro`.

### SLAM / EKF / Nav2 parameters

| File | What it controls |
|------|-----------------|
| `robot_slam/config/ekf_local.yaml` | EKF inputs, process noise |
| `robot_slam/config/slam_toolbox.yaml` | Onboard SLAM (keep in sync with `offboard_nav/config/slam_toolbox.yaml`) |
| `robot_slam/config/nav2_params_onboard.yaml` | Controller (RPP), local costmap, velocity smoother |
| `offboard_nav/config/nav2_params_offboard.yaml` | Planner, BT navigator, global costmap |

Changes take effect on the next launch without rebuilding (symlink install).
