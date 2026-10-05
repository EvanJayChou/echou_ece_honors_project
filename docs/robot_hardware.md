# <div align="center">Robot Hardware</div>

## Parts List

### Base kit

| Component | Notes |
|-----------|-------|
| Traxxas chassis | Steering servo + sensored brushless DC motor |
| Raspberry Pi 5 | 4 GB or 8 GB RAM recommended |
| Livox Mid-360 LiDAR | Ethernet; Pi static IP `192.168.1.50`, sensor at `192.168.1.12` (see `livox_bringup/config/MID360_config.json`) |
| OAK-D Pro camera | USB 3.0 port required; active IR stereo (dot projector + flood light) in addition to passive RGB-D |
| VESC motor controller | USB-serial; maps to `/dev/ttyACM1` |
| 2× DC-DC converters | 12 V rail for VESC; 5 V rail for Pi |
| Anti-spark switch with power switch | |
| 4-cell LiPo battery | |
| Battery voltage checker/alarm | |
| Micro SD card | ≥ 32 GB, A2 class recommended |
| SD card adapter/reader | |
| XT60, XT30, MR60 connectors | |

### Final project additions

| Component | Notes |
|-----------|-------|
| Seeed Studio XIAO nRF52840 Sense | USB CDC; maps to `/dev/ttyACM0` — runs custom IMU firmware |
| Raspberry Pi Pico 2W | USB CDC; maps to `/dev/ttyACM2` — controls LED strip and camera servo |
| 35 kg servo | Camera pan mount |
| WS2812B LED strip | Connected to Pico GPIO 2 |
| Raspberry Pi AI Hat+ | Obtained but not used |

### USB device assignments

The ACM device numbers are assigned by the kernel in enumeration order at boot:

```
/dev/ttyACM0   ←  XIAO nRF52840 Sense  (VID 0x2886 / PID 0x8045)
/dev/ttyACM1   ←  VESC                  (VID 0x0483 / PID 0x5740)
/dev/ttyACM2   ←  Pico 2W               (VID 0x2e8a)
```

If the ACM assignment is wrong after a reboot, plug devices in a different order or add device-specific udev symlink rules.

### Livox Mid-360 network assignment

The Mid-360 connects over Ethernet, not USB-serial. The Pi's Ethernet interface must be
statically assigned `192.168.1.50/24` to reach the sensor's default IP `192.168.1.12`:

```
sudo ip addr add 192.168.1.50/24 dev eth0
```

Make this persistent via netplan/NetworkManager rather than running it by hand each boot.
Host- and lidar-side IPs are also set in `ros2_ws/src/livox_bringup/config/MID360_config.json`.

## Mechanical Design Highlight

__Camera Stand__

![image](https://raw.githubusercontent.com/UCSD-ECEMAE-148/148sp26-spring-2026-final-project-team-4/main/docs/media/camera.png)

__VESC and Electronics Mounting__

![image](https://raw.githubusercontent.com/UCSD-ECEMAE-148/148sp26-spring-2026-final-project-team-4/main/docs/media/electronics.png)

__Full Assembly__

![image](https://raw.githubusercontent.com/UCSD-ECEMAE-148/148sp26-spring-2026-final-project-team-4/main/docs/media/full_mount.png)
