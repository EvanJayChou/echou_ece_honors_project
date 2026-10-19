#!/usr/bin/env bash
# offboard.sh — GPU-machine side of the stack (ros2_ws_offboard).
#
# Must share ROS_DOMAIN_ID with the rover and be able to discover it over DDS.
#
# Usage:
#   scripts/offboard.sh slam                 slam_toolbox only (pairs with teleop_slam.sh --offboard-slam)
#   scripts/offboard.sh explore              slam_toolbox + Nav2 global side + frontier_explorer (pairs with explore.sh)
#   scripts/offboard.sh explore --no-slam    as above, but SLAM runs on the Jetson (explore.sh --onboard-slam)
#   scripts/offboard.sh explore --auto-start begin exploring without waiting for /mission/control "start"
#   add --rviz to any mode to open RViz with the mapping layout

_TAG=offboard
source "$(dirname "$0")/common.sh"

MODE="${1:-}"
shift || true
USE_SLAM=true
AUTO_START=false
USE_RVIZ=false
for arg in "$@"; do
    case "$arg" in
        --no-slam)    USE_SLAM=false ;;
        --auto-start) AUTO_START=true ;;
        --rviz)       USE_RVIZ=true ;;
        *) err "Unknown flag: $arg  (valid: --no-slam, --auto-start, --rviz)"; exit 1 ;;
    esac
done

case "$MODE" in
    slam)    USE_NAV2=false; USE_EXPLORER=false ;;
    explore) USE_NAV2=true;  USE_EXPLORER=true ;;
    *) err "Usage: scripts/offboard.sh {slam|explore} [--no-slam] [--auto-start] [--rviz]"; exit 1 ;;
esac

source_ws "$OFFBOARD_WS"

log "Waiting for the rover's /scan_fixed..."
check_topic /scan_fixed 10 "/scan_fixed visible from this machine" \
    "/scan_fixed not visible — is the rover stack up, same ROS_DOMAIN_ID, and DDS discovery working?"

start_bg offboard ros2 launch offboard_nav offboard.launch.py \
    use_slam:="$USE_SLAM" use_nav2:="$USE_NAV2" use_explorer:="$USE_EXPLORER" \
    explorer_auto_start:="$AUTO_START"

if [ "$USE_RVIZ" = true ]; then
    start_bg rviz rviz2 -d "$ONBOARD_WS/src/robot_slam/rviz/mapping.rviz"
fi

if [ "$USE_EXPLORER" = true ] && [ "$AUTO_START" = false ]; then
    log "Start exploring with:"
    log "  ros2 topic pub --once /mission/control std_msgs/msg/String \"data: start\""
fi
log "Ctrl+C to stop."
wait
