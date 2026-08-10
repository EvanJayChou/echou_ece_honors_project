# Off-board Workspace (GPU cluster)

Runs the compute-heavy half of the stack: SLAM Toolbox occupancy mapping, the
Nav2 global-planning side (`bt_navigator`, `planner_server`, `smoother_server`,
`behavior_server`, `waypoint_follower`, global costmap), and `frontier_explorer`
(Q1 exploration baseline). The 3DGS mapper, uncertainty estimator, and
next-best-view selector will be added here in Q2.

The onboard rover stack (sensors, EKF, Nav2 `controller_server`) lives in
`../ros2_ws` and talks to this workspace over shared-domain ROS 2
topics/actions — see `../ros2_ws/src/robot_slam/launch/onboard.launch.py` and
`src/offboard_nav/launch/offboard.launch.py`.

Both machines must share a `ROS_DOMAIN_ID` and be able to discover each other
(static peer list or a DDS discovery server — default UDP multicast is often
blocked on lab Wi-Fi, so don't assume it works untested).

Build like any ROS 2 workspace:

```bash
cd ros2_ws_offboard
colcon build --symlink-install
source install/setup.bash
ros2 launch offboard_nav offboard.launch.py
```
