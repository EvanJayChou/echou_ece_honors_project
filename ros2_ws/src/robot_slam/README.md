robot_slam
==========

Onboard state estimation, SLAM and Nav2-controller launch package for the rover, plus two small nodes:

| Node | Purpose |
|------|---------|
| `scan_relay_node` | `/scan` → `/scan_fixed`: crops to a 270° front FOV and resamples to a fixed beam count (slam_toolbox requires a constant count) |
| `cmd_vel_to_ackermann_node` | Nav2 `/cmd_vel` (Twist) → `/ackermann_cmd` via the bicycle model, for `ackermann_to_vesc` |
| `glim_config` | Not a node: builds the GLIM config directory for `glim.launch.py` or an offline `glim_rosbag` run (`ros2 run robot_slam glim_config --mode gpu`) |

**Launch files**

- [launch/onboard.launch.py](launch/onboard.launch.py): the main entry point. Always starts `scan_relay_node`, `ekf_local_node` (VESC `vx` + OAK-D IMU yaw rate) and `robot_state_publisher`. Options:

  | Argument | Default | Adds |
  |----------|---------|------|
  | `use_slam` | `false` | slam_toolbox on the Jetson, 5 s after start. Leave `false` when SLAM runs off-board |
  | `use_nav2` | `false` | `controller_server` (Regulated Pure Pursuit), `velocity_smoother`, `cmd_vel_to_ackermann_node`, lifecycle manager |
  | `use_rviz` | `false` | RViz with `rviz/mapping.rviz` |

  ```bash
  ros2 launch robot_slam onboard.launch.py use_slam:=true
  ```

- [launch/glim.launch.py](launch/glim.launch.py): GLIM 3D LiDAR-inertial odometry and mapping on `/livox/lidar` + `/livox/imu`, headless, publishing on `/glim_ros/*` in its own `glim_map → glim_odom → glim_imu` TF tree. Arguments: `use_gpu` (`true`; `false` = CPU configs), `dump_path` (`/tmp/rover/glim_dump`), `use_sim_time`, `publish_lidar_tf` (`false`; static `base_link → livox_frame` for when `onboard.launch.py` isn't running). See [docs/glim.md](../../../docs/glim.md).

  ```bash
  ros2 launch robot_slam glim.launch.py              # GPU
  ros2 launch robot_slam glim.launch.py use_gpu:=false
  ```

- [launch/launch_stage_1.launch.py](launch/launch_stage_1.launch.py): Stage 1 wrapper around `onboard.launch.py` that passes the same arguments through.
- [launch/launch_stage_2.launch.py](launch/launch_stage_2.launch.py) / [launch/localization.launch.py](launch/localization.launch.py): Stage 2 AMCL localization against a saved map. **Not yet updated** for the Ackermann controller or the cmd_vel converter.

Sensor drivers (`livox_bringup`, `oakd_publisher`) and the VESC stack are started separately. The scripts in [`scripts/`](../../../scripts) combine everything per test, e.g. `scripts/teleop_slam.sh` or `scripts/explore.sh`.

**Config**

| File | Used by |
|------|---------|
| `config/ekf_local.yaml` | `ekf_local_node` |
| `config/slam_toolbox.yaml` | onboard slam_toolbox (`use_slam:=true`) |
| `config/nav2_params_onboard.yaml` | onboard controller / local costmap / velocity smoother |
| `config/nav2_params.yaml` | Stage 2 all-onboard Nav2 (legacy) |
| `config/glim/` | `glim.launch.py`. GLIM configs with rover changes; [config/glim/README.md](config/glim/README.md) lists each change |
| `urdf/robot.urdf.xacro` | `base_link` → `livox_frame`, `oakd_link`, `oakd_camera_optical_frame`, `oakd_imu_frame` |

**RViz layouts**

- Mapping: `rviz/mapping.rviz`
- Localization: `rviz/localization.rviz`
