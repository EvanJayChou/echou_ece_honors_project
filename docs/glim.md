# <div align="center">GLIM: 3D LiDAR-Inertial SLAM</div>

[GLIM](https://github.com/koide3/glim) ([docs](https://koide3.github.io/glim/), ROS 2 wrapper [glim_ros2](https://github.com/koide3/glim_ros2)) runs onboard as real-time LiDAR-inertial odometry and mapping from the Livox Mid-360. The same raw data can be recorded and re-mapped at higher quality on a workstation. It runs **alongside** the 2D stack (EKF + slam_toolbox + Nav2) without taking over its TF frames.

| Piece | Where |
|-------|-------|
| GLIM install (koide3 PPA, `-cuda13.1` build) | [`docker/Dockerfile`](../docker/Dockerfile), "GLIM" step |
| Config (copied from GLIM 1.2.2 and tuned) | [`robot_slam/config/glim/`](../ros2_ws/src/robot_slam/config/glim/); [its README](../ros2_ws/src/robot_slam/config/glim/README.md) lists every change and why |
| Launch | [`robot_slam/launch/glim.launch.py`](../ros2_ws/src/robot_slam/launch/glim.launch.py) |
| Config directory builder | [`robot_slam/glim_config.py`](../ros2_ws/src/robot_slam/robot_slam/glim_config.py) (`ros2 run robot_slam glim_config`) |
| Raw-sensor recorder | [`scripts/record.sh`](../scripts/record.sh) |

---

## 1. Install (in the image)

GLIM is installed in the Docker image, not on the Jetson host:

```
koide3 PPA (ubuntu2404)  →  libgtsam-points-cuda13.1-dev 1.2.2
                             ros-jazzy-glim-cuda13.1 / ros-jazzy-glim-ros-cuda13.1 1.2.2-0noble
                             libiridescence-dev libboost-all-dev libglfw3-dev libmetis-dev
```

**CUDA stays inside Docker.** The image's base is `nvidia/cuda:13.2.1-runtime-ubuntu24.04`: CUDA 13.2 runtime libraries with no compiler. The PPA has no 13.2 build, but the 13.1 build works on this image without adding anything:

- The packages declare no CUDA dependency, and a dry run (`apt-get install -s`) pulled in no CUDA toolkit and removed nothing.
- They link `libcudart.so.13`, which resolves to the image's `/usr/local/cuda/lib64/libcudart.so.13` (13.2). CUDA 13.x keeps one ABI.
- `libgtsam_points_cuda.so` embeds kernels for `sm_75/80/90/100/110/120` but not `sm_87`. The `sm_80` code runs unchanged on any `sm_8x` GPU, the Orin included. A full demo-bag run logged no `no kernel image is available` or other CUDA errors.
- Nothing on the host was installed, upgraded or reconfigured (JetPack 7.2.1, CUDA 13.2 driver 595.78). If GLIM ever needs a source build (e.g. native `sm_87`), do it in a `nvidia/cuda:13.2.1-devel` stage of the image with `-DCMAKE_CUDA_ARCHITECTURES=87 -DBUILD_WITH_MARCH_NATIVE=OFF -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF`, never on the host.

On the CPU-only base (`--build-arg BASE_IMAGE=ros:jazzy-ros-base`), the Dockerfile installs the plain `ros-jazzy-glim-ros` instead; launch it with `use_gpu:=false`. Versions are pinned with `GLIM_VERSION` / `GTSAM_POINTS_VERSION` ([docker.md §2](docker.md#2-build-the-image)).

In an already-running `rover-dev` container, rebuilding the image picks this up. To try it without a rebuild, the same `apt-get` commands from the Dockerfile work as-is with `sudo` inside the container. They're lost when the container is recreated.

## 2. Configuration summary

Full details, with sources, are in [config/glim/README.md](../ros2_ws/src/robot_slam/config/glim/README.md). In short:

- **Input:** `/livox/lidar` + `/livox/imu` (the Mid-360's own IMU, `acc_scale` 9.80665 because it reports in g). `livox_bringup` now uses `xfer_format: 0`, so every point carries a timestamp for deskewing.
- **Extrinsic:** `T_lidar_imu = [0.011, 0.02329, -0.04412, 0, 0, 0, 1]`, from the Livox Mid-360 user manual (IMU at 11.0, 23.29, −44.12 mm in the point cloud frame, axes aligned).
- **Headless:** no Iridescence viewer module. Output goes over ROS for a remote RViz.
- **Memory:** stock settings hit 84 % of the 8 GB shared pool on the demo bag. Halved per-frame points, a 40 m range cap, a shorter smoother window, fewer keyframes, and smaller submaps bring it down (§5).

### TF frames

GLIM publishes its own tree, separate from the robot's:

```
robot:  map ─(slam_toolbox)→ odom ─(ekf_local)→ base_link ─(URDF)→ livox_frame, oakd_link, …
GLIM:   glim_map ─→ glim_odom ─→ glim_imu ─→ glim_lidar
```

GLIM always broadcasts `map → odom → base` itself. Using the robot's frame names would give `base_link` and `odom` two parents. In RViz, set the fixed frame to `glim_map` to view GLIM's output.

#### Making GLIM the primary TF source

Not done yet. Steps if GLIM should replace the EKF and slam_toolbox for localization:

1. In `config_ros.json`: `"imu_frame_id": "livox_frame"`, `"lidar_frame_id": "livox_frame"`, `"base_frame_id": "base_link"`, `"odom_frame_id": "odom"`, `"map_frame_id": "map"`, `"publish_imu2lidar": false`. With `base_link` set, GLIM looks up `livox_frame → base_link` from the URDF.
2. Set `publish_tf: false` in `ekf_local.yaml` and don't run slam_toolbox (`use_slam:=false`).
3. Nav2 still needs a 2D occupancy map; GLIM's map is a 3D point cloud.

## 3. Launch

```bash
# Sensors (Mid-360 at xfer_format 0) and the base stack, as usual
scripts/sensors.sh --lidar            # or any teleop/explore script
ros2 launch robot_slam glim.launch.py               # GPU (default)
ros2 launch robot_slam glim.launch.py use_gpu:=false  # CPU configs, see the caveat in §5
```

| Argument | Default | |
|----------|---------|---|
| `use_gpu` | `true` | `false` selects `config_cpu.json` (CPU odometry, VGICP CPU mapping) |
| `dump_path` | `/tmp/rover/glim_dump` | Map and trajectory dump written on Ctrl+C; overwritten each run |
| `use_sim_time` | `false` | `true` with `ros2 bag play --clock` |
| `publish_lidar_tf` | `false` | Static `base_link → livox_frame` (0.1, 0, 0.2), only when `onboard.launch.py` isn't running |

At startup the launch file runs `robot_slam.glim_config`, which builds `/tmp/rover/glim_config_<gpu|cpu>/` (symlinks to the package config plus the matching `config.json`) and passes it as `config_path`. On Ctrl+C GLIM finishes global optimization and writes the dump. That takes a few seconds, so the launch file gives it 60 s before escalating to SIGTERM.

### Published topics (node `/glim_ros`)

| Topic | Type | Frame | Notes |
|-------|------|-------|-------|
| `/glim_ros/odom` | `nav_msgs/Odometry` | `glim_odom` → `glim_imu` | Real-time odometry, one per LiDAR frame (10 Hz) |
| `/glim_ros/pose` | `geometry_msgs/PoseStamped` | `glim_map` | Same pose in the map frame |
| `/glim_ros/odom_corrected`, `/glim_ros/pose_corrected` | | | Re-published after the fixed-lag smoother refines a frame |
| `/glim_ros/lidar_odom`, `/glim_ros/lidar_pose` (+ `_corrected`, `_scanend`) | | `glim_lidar` | LiDAR-frame versions |
| `/glim_ros/points`, `/glim_ros/aligned_points` (+ `_corrected`) | `sensor_msgs/PointCloud2` | `glim_lidar` / `glim_map` | Deskewed scan; scan transformed into the map |
| `/glim_ros/map` | `sensor_msgs/PointCloud2` | `glim_map` | Global map, republished as submaps are added |
| `/tf` | | | `glim_map → glim_odom → glim_imu → glim_lidar` |

Remote RViz: fixed frame `glim_map`, then add `/glim_ros/map` and `/glim_ros/aligned_points`. `/glim_ros/map` is published reliable + transient-local (depth 1), so set the RViz display's Durability to *Transient Local* to get the latest map on connect. It grows large, so view it sparingly over Wi-Fi.

Logs: the launch output, plus GLIM's own `glim_*.log` files in `/tmp/rover/` (`config_logging.json`).

## 4. Recording and offline mapping

### Record on the rover

```bash
scripts/record.sh                 # MCAP, zstd_fast chunk compression → <repo>/bags/rover_<date>_<time>/
scripts/record.sh --no-images     # LiDAR + IMU + TF + wheel odom only
scripts/record.sh --no-compress   # if the compressor falls behind
```

Topics: `/livox/lidar`, `/livox/imu`, `/oakd/imu`, `/vesc_odom`, `/tf`, `/tf_static`, plus with images `/oakd/color/image_raw`, `/oakd/depth/image_raw` and `/oakd/{color,depth}/camera_info`. Start the stack before recording so latched `/tf_static` is captured. Bags land in `<repo>/bags/`, which is git-ignored, excluded from the Docker build context, and mounted into both containers.

Compression is MCAP's chunk-level zstd (`--storage-preset-profile zstd_fast`), not rosbag2's per-message compression, which costs more CPU and makes replay slower. LiDAR + IMU measured about **2.4 MB/s** compressed. Raw OAK-D RGB-D adds about 70 MB/s before compression (1280×720 bgr8 + mono16 at 15 Hz). After a run, check for drops with `ros2 bag info <bag>`: counts should match about 10 Hz for `/livox/lidar`, 200 Hz for `/livox/imu`, and 15 Hz for the images. If image counts fall short, use `--no-compress`, or lower the OAK-D resolution/fps.

### Re-run GLIM on a workstation (x86, NVIDIA GPU)

1. Install GLIM from the same PPA (https://koide3.github.io/ppa/), choosing the CUDA build that matches the workstation's toolkit: `ros-jazzy-glim-ros-cuda12.6` or `-cuda13.1`, plus `libgtsam-points-cuda*-dev` and `libiridescence-dev`. Or use this repo's Docker image built on the workstation (§1 / [docker.md](docker.md)).
2. Build `robot_slam` (or copy `config/glim/`), then build a config directory. The Jetson memory limits don't apply here: for best quality, set the values in the config README's tuning table back to stock (the "Stock" column), e.g. `random_downsample_target` 10000, `submap_target_num_points` 50000, `randomsampling_rate` 1.0, `max_implicit_loop_distance` 100. To view the map interactively, add `"libstandard_viewer.so"` back to `extension_modules`.
   ```bash
   ros2 run robot_slam glim_config --mode gpu --out ~/glim_cfg
   ```
3. Run on the bag (a bag directory with `metadata.yaml`, or a `.mcap` file):
   ```bash
   ros2 run glim_ros glim_rosbag ~/bags/rover_20261030_225319 \
       --ros-args -p config_path:=$HOME/glim_cfg -p dump_path:=$HOME/glim_dump -p auto_quit:=true
   ```
   `glim_rosbag` reads as fast as it can (`playback_speed`, default 100×, set in `config_ros.json`'s `glim_ros` section). Other parameters: `start_offset`, `playback_duration`, `playback_until`.
4. Inspect or export: `ros2 run glim_ros offline_viewer ~/glim_dump` (GUI; it can re-optimize, add manual loop closures, and export the map as PLY).

### What's in a dump (`dump_path`, default `/tmp/dump` for `glim_rosbag`, `/tmp/rover/glim_dump` for `glim.launch.py`)

| File | Contents |
|------|----------|
| `traj_lidar.txt`, `traj_imu.txt` | Optimized trajectory after global mapping, TUM format: `stamp x y z qx qy qz qw` |
| `odom_lidar.txt`, `odom_imu.txt` | Raw odometry trajectory (before global optimization) |
| `000000/`, `000001/`, … | One folder per submap: points, covariances, keyframe data |
| `graph.bin`, `graph.txt`, `values.bin` | GTSAM factor graph and optimized values (for `offline_viewer` re-optimization) |
| `config/` | Snapshot of the config used |

Note that `/tmp/rover` is on the host's `/tmp`, so copy dumps you want to keep (e.g. into `bags/`).

## 5. Validation (2026-10-30)

Live sensors were **not** connected for this validation (Mid-360 unreachable, no OAK-D on USB). Instead, everything was checked on GLIM's official Mid-360 demo bag ([Zenodo 14841855](https://zenodo.org/records/14841855), outdoor drive, 277 s, about 1.08 km). The bag was played at 1× with `ros2 bag play --clock` into `glim.launch.py`, the same node and config path as live. The Jetson was in the `rover-dev` container, GLIM 1.2.2 `-cuda13.1`.

| | GPU (`use_gpu:=true`) | CPU (`use_gpu:=false`) |
|---|---|---|
| Odometry messages / LiDAR frames | 2761 / 2772 (10 Hz; the 11 missing frames are the 1 s IMU initialization window) | 2741 / 2772 (3 s initialization window in the CPU config) |
| GPU load (sysfs `17000000.gpu/load`) | 13 % avg, 100 % peaks | 0 % |
| `glim_rosnode` CPU (avg, % of one core) | ~48 % | ~102 % |
| `glim_rosnode` RSS peak | 3.1 GB | 2.7 GB |
| System RAM used (start → peak) | 3.7 → 5.8 GB of 7.3 GB | 3.5 → 5.2 GB |
| `[mem]` warnings (>80 %) | none (stock config: 84 %) | none |
| CUDA errors | none | n/a |
| Trajectory vs stock-config GPU run (best-fit rigid alignment) | **0.40 m RMSE**, 0.65 m max | 18.6 m RMSE, 35.8 m max (see below) |

`scripts/record.sh --no-images` ran during the GPU test (alongside GLIM). It recorded 2666 LiDAR frames over 266.6 s (exactly 10 Hz), 53315 IMU messages (200 Hz), no drops, and 640 MB. Replaying that MCAP offline with `glim_rosbag` + the GPU config took 64 s and matched the live run to 2.1 m RMSE (3.5 m max). The bag started about 10 s into the drive, so GLIM initialized while the vehicle was moving.

**CPU mode drifts on this bag.** Even GLIM's stock CPU config ends 45 m below the start over 1 km (13.0 m RMSE vs. the GPU run), and the rover's tuned CPU config is worse (17.7 m RMSE offline). Treat `use_gpu:=false` as a fallback for the CPU-only image, not an equal alternative. It has not been tested indoors.

### Still to do on the rover

- **Live sensors:** `scripts/sensors.sh --lidar`, then `glim.launch.py`. Check `ros2 topic hz /glim_ros/odom` (expect about 10 Hz), `ros2 topic hz /scan` (pointcloud_to_laserscan on the new `xfer_format: 0` cloud), and memory with `jtop`/`tegrastats` on the host (neither is in the image).
- **Closed loop:** drive a loop around a room back to the start mark, stop, Ctrl+C. Compare the first and last poses in `/tmp/rover/glim_dump/traj_lidar.txt`. A few centimetres apart is good; tens of centimetres means a bad extrinsic or IMU timing.
- **Extrinsic check:** `T_lidar_imu` comes from the datasheet. If the loop test is poor, check `imu_time_offset` and the IMU validation lines GLIM logs (`IMU better ratios rot=… trans=…`).
