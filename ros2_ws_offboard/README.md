# Off-board Workspace (GPU cluster)

Runs the compute-heavy half of the stack: SLAM Toolbox occupancy mapping, the
Nav2 global-planning side (`bt_navigator`, `planner_server`, `smoother_server`,
`behavior_server`, `waypoint_follower`, global costmap), and `frontier_explorer`
(Q1 exploration baseline). The 3DGS mapper, uncertainty estimator, and
next-best-view selector will be added here in Q2.

The onboard rover stack (sensors, EKF, Nav2 `controller_server`, and the
`cmd_vel` → `/ackermann_cmd` converter) lives in `../ros2_ws` and talks to this
workspace over shared-domain ROS 2 topics/actions. See
`../ros2_ws/src/robot_slam/launch/onboard.launch.py` and
`src/offboard_nav/launch/offboard.launch.py`.

Both machines must share a `ROS_DOMAIN_ID` and be able to discover each other
(static peer list or a DDS discovery server). Default UDP multicast is often
blocked on lab Wi-Fi, so test it before relying on it.

## Build and run

```bash
scripts/build.sh offboard              # from the repo root
scripts/offboard.sh slam               # SLAM only     (rover: scripts/teleop_slam.sh --offboard-slam)
scripts/offboard.sh explore            # SLAM + Nav2 + frontier_explorer (rover: scripts/explore.sh)
```

Or by hand:

```bash
cd ros2_ws_offboard
colcon build --symlink-install
source install/setup.bash
ros2 launch offboard_nav offboard.launch.py                       # everything
ros2 launch offboard_nav offboard.launch.py use_nav2:=false use_explorer:=false   # SLAM only
```

| Argument | Default | Effect |
|----------|---------|--------|
| `use_slam` | `true` | slam_toolbox here. Set `false` if the rover maps onboard |
| `use_nav2` | `true` | Nav2 global-planning servers + lifecycle manager |
| `use_explorer` | `true` | `frontier_explorer` |
| `explorer_auto_start` | `false` | Explore without waiting for `/mission/control` `start` |

Start and stop exploration:

```bash
ros2 topic pub --once /mission/control std_msgs/msg/String "data: start"
ros2 topic pub --once /mission/control std_msgs/msg/String "data: end"
```
