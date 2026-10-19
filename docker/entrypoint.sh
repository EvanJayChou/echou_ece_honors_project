#!/usr/bin/env bash
# Container entrypoint: source ROS 2 and whichever workspaces are built, then run
# the command (default: bash). Missing workspaces are skipped so the dev service,
# which mounts an unbuilt checkout over /opt/rover, still starts.
set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
[ -f /opt/rover/ros2_ws/install/setup.bash ] && source /opt/rover/ros2_ws/install/setup.bash
[ -f /opt/rover/ros2_ws_offboard/install/local_setup.bash ] && source /opt/rover/ros2_ws_offboard/install/local_setup.bash
cd /opt/rover
exec "$@"
