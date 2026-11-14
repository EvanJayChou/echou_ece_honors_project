# Off-board Workspace (GPU machine)

Home of the compute-heavy, off-board half of the project: the online 3DGS
mapper, the per-Gaussian uncertainty estimator, and next-best-view selection.
It receives posed RGB-D keyframes from the rover's `keyframe_bridge`
(`/keyframe/{color,depth,pose}`) and returns next-best-view goals to Nav2 on the
rover. **Empty until that pipeline lands**; `scripts/build.sh offboard` is a
no-op until then.

Everything else runs on the Jetson in `../ros2_ws`: sensors, EKF, slam_toolbox,
the full Nav2 stack and `frontier_explorer` (see
`../ros2_ws/src/robot_slam/launch/onboard.launch.py`). That way a Wi-Fi drop can't
break localization or control.

## Opt-in: SLAM on this machine

For comparison runs only. It needs ROS 2 Jazzy and slam_toolbox installed, but no
build: `scripts/offboard.sh` reads `ros2_ws/src/robot_slam/config/slam_toolbox.yaml`
from this checkout.

```bash
scripts/offboard.sh slam [--rviz]          # this machine
scripts/teleop_slam.sh --offboard-slam     # rover, teleop
scripts/explore.sh --offboard-slam         # or rover, exploration
```

Both machines must share a `ROS_DOMAIN_ID` and be able to discover each other
(static peer list or a DDS discovery server). Default UDP multicast is often
blocked on lab Wi-Fi, so test it before relying on it.
