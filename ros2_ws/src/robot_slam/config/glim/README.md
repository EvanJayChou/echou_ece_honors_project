GLIM config (rover)
===================

Copied from GLIM 1.2.2's `share/glim/config` (`ros-jazzy-glim-ros-cuda13.1`) and edited for the Livox Mid-360 on the Jetson Orin Nano. Every changed line carries a `// [rover]` comment. Never edit the installed copy under `/opt/ros/jazzy/share/glim/`. For launching, recording and offline mapping, see [docs/glim.md](../../../../../docs/glim.md).

`glim.launch.py` doesn't point GLIM at this folder directly, because GLIM always reads `<config_path>/config.json`. Instead `robot_slam.glim_config` builds `/tmp/rover/glim_config_<gpu|cpu>/`: symlinks to the files here, plus `config.json` → `config_gpu.json` or `config_cpu.json`.

| File | Selected by |
|------|-------------|
| `config_gpu.json` / `config_cpu.json` | `use_gpu:=true` / `false`: picks the `*_gpu` or `*_cpu` odometry, sub-mapping and global-mapping files |
| `config_ros.json`, `config_sensors.json`, `config_preprocess.json`, `config_logging.json` | both modes |
| `config_odometry_ct.json`, `config_global_mapping_pose_graph.json`, `config_sub_mapping_passthrough.json` | unused; stock alternatives kept for experiments |

`config_viewer.json` was dropped, and so was the `config_viewer` entry in the selectors (see Headless below).

## Sensor and ROS settings

| Key | Stock | Rover | Why |
|-----|-------|-------|-----|
| `glim_ros.points_topic` | `/os_cloud_node/points` | `/livox/lidar` | Mid-360 driver output. Needs `xfer_format: 0` in `livox_bringup` for per-point timestamps (deskewing) |
| `glim_ros.imu_topic` | `/os_cloud_node/imu` | `/livox/imu` | The Mid-360's built-in IMU: 200 Hz and hardware-synced with the LiDAR. The OAK-D IMU's clock is shifted from a different device |
| `glim_ros.acc_scale` | `0.0` (auto) | `9.80665` | The Livox IMU reports acceleration in g; GLIM's docs give this value for Livox sensors |
| `sensors.T_lidar_imu` | Ouster OS0 values | `[0.011, 0.02329, -0.04412, 0, 0, 0, 1]` | Livox Mid-360 user manual: "IMU position in the point cloud coordinate system: (11.0, 23.29, -44.12) mm", axes aligned with the point cloud frame. GLIM's convention is `p_lidar = T_lidar_imu * p_imu`, so the translation is the IMU origin in LiDAR coordinates |

Per-point time is left on `autoconf_perpoint_times: true`. The driver's `timestamp` field holds absolute FLOAT64 nanoseconds, which GLIM detects and converts (it logs a warning saying so on the first frame).

### TF frames

GLIM always broadcasts `map_frame_id → odom_frame_id → base_frame_id` (+ `imu → lidar`). In the rover stack, `ekf_local_node` already owns `odom → base_link`, slam_toolbox owns `map → odom`, and robot_state_publisher owns `base_link → livox_frame`. Reusing those names would give a frame two parents. The driver also uses `livox_frame` for both points and IMU, so stock `imu → lidar` would be a self-transform (GLIM logs `TF_SELF_TRANSFORM` errors on the demo bag).

GLIM therefore publishes its own parallel tree, which never touches the robot's:

```
glim_map → glim_odom → glim_imu → glim_lidar
```

`base_frame_id` is blank, so it defaults to `glim_imu` and GLIM never needs a TF lookup into the robot tree. In remote RViz, set the fixed frame to `glim_map` to view GLIM's output. The odometry in `/glim_ros/odom` is the pose of the Mid-360's IMU; `T_lidar_imu` above and the URDF's `base_link → livox_frame` relate it to `base_link`. To make GLIM the rover's localization source instead, see "Making GLIM the primary TF source" in docs/glim.md.

### Headless

`libstandard_viewer.so` (the Iridescence OpenGL window) is removed from `extension_modules`, and the selectors don't reference `config_viewer.json`. Still loaded:

- `libmemory_monitor.so`: logs a warning when CPU or GPU memory passes about 80 %. On the Orin both are the same 8 GB pool.
- `librviz_viewer.so`: despite its name it opens no window. It publishes `/glim_ros/*` topics and TF for a remote RViz.

## Memory tuning (8 GB shared CPU/GPU)

With stock settings, GLIM reached **84 % of the 7.5 GB pool** (GPU 6283 MB) replaying the 277 s Mid-360 demo bag. GPU memory on the Orin *is* system RAM, so that leaves nothing for Nav2, the OAK-D, or a recorder. Every change trims either per-frame point count or how much history stays resident:

| File | Key | Stock | Rover | Effect |
|------|-----|-------|-------|--------|
| `config_preprocess.json` | `distance_far_thresh` | 100 m | 40 m | Drops far points that are sparse and noisy anyway. Matches `livox_bringup`'s `range_max` |
| | `random_downsample_target` | 10000 | 5000 | Halves the points per frame that go into odometry |
| `config_odometry_gpu.json` | `smoother_lag` | 5.0 s | 3.0 s | Shorter fixed-lag smoother window, so fewer frames and factors are kept |
| | `max_num_keyframes` | 15 | 10 | Fewer keyframe voxelmaps on the GPU |
| `config_odometry_cpu.json` | `smoother_lag` | 5.0 s | 3.0 s | Same as GPU |
| `config_sub_mapping_gpu.json` | `max_num_keyframes` | 15 | 10 | Smaller submaps |
| | `submap_target_num_points` | 50000 | 20000 | Every submap stays resident for global mapping; this is what grows with run length |
| `config_sub_mapping_cpu.json` | `max_num_keyframes` | 15 | 10 | Smaller submaps |
| `config_global_mapping_gpu.json` | `randomsampling_rate` | 1.0 | 0.5 | Half the points per submap in each loop-closure factor |
| | `max_implicit_loop_distance` | 100 m | 30 m | Fewer submap-pair factors. Plenty for rooms and corridors; raise it for large outdoor loops |
| `config_global_mapping_cpu.json` | `max_implicit_loop_distance` | 100 m | 30 m | Same as GPU |
| `config_logging.json` | `log_dir` | `/tmp` | `/tmp/rover` | Next to the other component logs (mounted on the host) |

Global mapping still grows with run length, because every submap is kept. For long runs, watch the `[mem]` warnings in the GLIM log. If memory runs short, either set `enable_global_mapping: false` in `config_ros.json` (odometry + submaps only), or record a bag and do the full mapping offline on the workstation (docs/glim.md §4), where none of this tuning is needed.
