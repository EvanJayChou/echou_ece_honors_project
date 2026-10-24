# <div align="center">Robot Hardware</div>

## Parts List

### Base kit

| Component | Notes |
|-----------|-------|
| Traxxas chassis | Steering servo + sensored brushless DC motor |
| NVIDIA Jetson Orin Nano (Developer Kit) | JetPack 7.2.x → Ubuntu 24.04 + ROS 2 Jazzy. Replaced the Raspberry Pi 5 |
| Livox Mid-360 LiDAR | Ethernet; Jetson static IP `192.168.1.50`, sensor at `192.168.1.12` (see `livox_bringup/config/MID360_config.json`) |
| OAK-D Pro camera | USB 3 port required. RGB + stereo depth with active IR (dot projector + flood light). Its **onboard IMU** (BMI270 on current units, BNO086 on early ones) is the rover's IMU |
| VESC motor controller | USB-serial (VID `0x0483` / PID `0x5740`) → `/dev/vesc` via `udev/99-rover.rules` |
| 2× DC-DC converters | 12 V rail for VESC; the second rail must match the Jetson dev kit's DC barrel-jack input (check NVIDIA's spec; the old 5 V Pi rail is **not** enough) |
| Anti-spark switch with power switch | |
| 4-cell LiPo battery | |
| Battery voltage checker/alarm | |
| Micro SD card | ≥ 32 GB, A2 class recommended |
| SD card adapter/reader | |
| XT60, XT30, MR60 connectors | |

### Final project additions

| Component | Notes |
|-----------|-------|
| 35 kg servo | Camera pan mount (no software in this repo drives it yet) |
| Raspberry Pi AI Hat+ | Obtained but not used (Pi-only, not compatible with the Jetson) |

### Retired hardware

| Component | Replaced by |
|-----------|-------------|
| LD06 2D LiDAR | Livox Mid-360 (Ethernet, 360° × 59° 3D) |
| OAK-D Lite | OAK-D Pro |
| Seeed XIAO nRF52840 Sense (custom IMU firmware) | OAK-D Pro onboard IMU. The `xiao_serial_bridge` package and its firmware were removed (still in git history) |
| Raspberry Pi Pico 2W + WS2812B LED strip | Its `pico_hw_bridge` software (and the `survey_camera` / `mission_report` web dashboard) was removed in `9f8795d`, so the status LED isn't driven |

### Sensor frames

All mount poses live in `ros2_ws/src/robot_slam/urdf/robot.urdf.xacro`. The current offsets are estimates, so **measure them on the rover**:

| Frame | Parent | Default offset (x, y, z) m | Source of data |
|-------|--------|---------------------------|----------------|
| `livox_frame` | `base_link` | 0.10, 0.00, 0.20 | `/livox/lidar`, `/scan` |
| `oakd_link` | `base_link` | 0.15, 0.00, 0.25 | camera body (x forward, y left, z up) |
| `oakd_camera_optical_frame` | `oakd_link` | rotation only | `/oakd/color/image_raw`, `/oakd/depth/image_raw` |
| `oakd_imu_frame` | `oakd_link` | identity | `/oakd/imu` (rotated into this frame by `oakd_publisher`) |

### USB device assignments

With the XIAO and Pico gone, the VESC is the only USB-serial device, so it usually comes up as `/dev/ttyACM0`. Install the udev rule so it always gets a stable name:

```bash
sudo cp udev/99-rover.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
ls -l /dev/vesc        # → ttyACM0 (or similar)
```

The same rule file grants access to the OAK-D Pro (USB vendor `03e7`). Plug the OAK-D into one of the Jetson 5's **blue USB 3** ports. On USB 2, depth plus IMU streaming drops frames.

### Livox Mid-360 network assignment

The Mid-360 connects over Ethernet, not USB-serial. The Jetson's Ethernet interface must be
statically assigned `192.168.1.50/24` to reach the sensor's default IP `192.168.1.12`:

```
sudo nmcli con add type ethernet ifname <iface> con-name livox \
    ipv4.method manual ipv4.addresses 192.168.1.50/24 ipv4.never-default yes
```

Find `<iface>` with `ip -br link`. See [reproduction.md §6](reproduction.md#6-livox-mid-360--network-setup-and-sdk).
Host- and lidar-side IPs are also set in `ros2_ws/src/livox_bringup/config/MID360_config.json`.
The Mid-360 needs its own 9–27 V supply through the aviation connector. It doesn't draw power over Ethernet.

## Mechanical Design Highlight

__Camera Stand__

![image](https://raw.githubusercontent.com/UCSD-ECEMAE-148/148sp26-spring-2026-final-project-team-4/main/docs/media/camera.png)

__VESC and Electronics Mounting__

![image](https://raw.githubusercontent.com/UCSD-ECEMAE-148/148sp26-spring-2026-final-project-team-4/main/docs/media/electronics.png)

__Full Assembly__

![image](https://raw.githubusercontent.com/UCSD-ECEMAE-148/148sp26-spring-2026-final-project-team-4/main/docs/media/full_mount.png)
