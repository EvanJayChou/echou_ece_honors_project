#!/usr/bin/env bash
# offboard.sh — GPU-machine side of the stack.
#
# SLAM, Nav2 and frontier_explorer run on the Jetson by default; this script is
# only for the opt-in off-board SLAM mode. (The Q2 3DGS mapper / NBV pipeline in
# ros2_ws_offboard will be launched from here once it exists.)
#
# Must share ROS_DOMAIN_ID with the rover and be able to discover it over DDS.
# Needs only ROS 2 + slam_toolbox installed: the config is read from this
# checkout's robot_slam package, so there is a single slam_toolbox.yaml.
#
# Usage:
#   scripts/offboard.sh slam [--rviz]   slam_toolbox here; pairs with
#                                       teleop_slam.sh --offboard-slam or explore.sh --offboard-slam

_TAG=offboard
source "$(dirname "$0")/common.sh"

MODE="${1:-}"
shift || true
USE_RVIZ=false
for arg in "$@"; do
    case "$arg" in
        --rviz) USE_RVIZ=true ;;
        *) err "Unknown flag: $arg  (valid: --rviz)"; exit 1 ;;
    esac
done

case "$MODE" in
    slam) ;;
    explore)
        err "Exploration runs on the Jetson now: scripts/explore.sh (add --offboard-slam to pair with 'offboard.sh slam')"
        exit 1 ;;
    *) err "Usage: scripts/offboard.sh slam [--rviz]"; exit 1 ;;
esac

if [ ! -f "/opt/ros/$ROS_DISTRO_NAME/setup.bash" ]; then
    err "/opt/ros/$ROS_DISTRO_NAME/setup.bash not found — is ROS 2 $ROS_DISTRO_NAME installed?"
    exit 1
fi
set +u
source "/opt/ros/$ROS_DISTRO_NAME/setup.bash"
log "Sourced ROS 2 $ROS_DISTRO_NAME  (ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0})"

SLAM_PARAMS="$ONBOARD_WS/src/robot_slam/config/slam_toolbox.yaml"

log "Waiting for the rover's /scan_fixed..."
check_topic /scan_fixed 10 "/scan_fixed visible from this machine" \
    "/scan_fixed not visible — is the rover stack up, same ROS_DOMAIN_ID, and DDS discovery working?"

start_bg offboard ros2 launch slam_toolbox online_async_launch.py \
    slam_params_file:="$SLAM_PARAMS" use_sim_time:=false

if [ "$USE_RVIZ" = true ]; then
    start_bg rviz rviz2 -d "$ONBOARD_WS/src/robot_slam/rviz/mapping.rviz"
fi

log "Ctrl+C to stop."
wait
