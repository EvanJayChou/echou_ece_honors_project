#!/usr/bin/env bash
# explore.sh — onboard half of autonomous frontier exploration.
#
# Runs the sensors, EKF, VESC driver, the onboard Nav2 controller
# (controller_server + velocity_smoother) and the cmd_vel → /ackermann_cmd
# converter. By default SLAM, the global planner/BT and frontier_explorer run
# off-board:
#
#   GPU machine:  scripts/offboard.sh explore
#
# With --all-onboard the Jetson runs everything itself (needs ros2_ws_offboard
# built on the Jetson too: scripts/build.sh offboard). No second machine needed.
#
# Start exploring once both sides are up:
#   ros2 topic pub --once /mission/control std_msgs/msg/String "data: start"
# Stop (cancels the active Nav2 goal):
#   ros2 topic pub --once /mission/control std_msgs/msg/String "data: end"
#
# Usage:
#   scripts/explore.sh                 off-board SLAM + planning (default)
#   scripts/explore.sh --onboard-slam  run slam_toolbox on the Jetson instead (then use `offboard.sh explore --no-slam`)
#   scripts/explore.sh --all-onboard   SLAM, Nav2 and frontier_explorer all on the Jetson
#   scripts/explore.sh --rviz

_TAG=explore
source "$(dirname "$0")/common.sh"

USE_SLAM=false
USE_RVIZ=false
ALL_ONBOARD=false
for arg in "$@"; do
    case "$arg" in
        --onboard-slam) USE_SLAM=true ;;
        --all-onboard)  USE_SLAM=true; ALL_ONBOARD=true ;;
        --rviz)         USE_RVIZ=true ;;
        *) err "Unknown flag: $arg  (valid: --onboard-slam, --all-onboard, --rviz)"; exit 1 ;;
    esac
done

log "Preflight..."
check_power_mode
check_lidar
check_oakd
resolve_vesc_port
source_ws "$ONBOARD_WS"
[ "$ALL_ONBOARD" = true ] && source_overlay "$OFFBOARD_WS"

start_lidar
start_oakd
start_vesc
sleep 3
start_onboard use_slam:="$USE_SLAM" use_nav2:=true use_rviz:="$USE_RVIZ"
if [ "$ALL_ONBOARD" = true ]; then
    # Global planning + explorer from ros2_ws_offboard; SLAM is already in onboard.launch.py.
    start_bg offboard ros2 launch offboard_nav offboard.launch.py use_slam:=false
fi

sleep 8
check_topic /scan_fixed 5 "/scan_fixed publishing" "/scan_fixed missing — see $LOG_DIR/lidar.log and $LOG_DIR/onboard.log"
check_topic /oakd/imu 5 "/oakd/imu publishing" "/oakd/imu missing — see $LOG_DIR/oakd.log"
check_topic /odometry/filtered 5 "EKF /odometry/filtered publishing" "EKF output missing — see $LOG_DIR/onboard.log"
if ros2 action list 2>/dev/null | grep -q "^/follow_path$"; then
    ok "Nav2 controller up (/follow_path)"
else
    warn "/follow_path action not found — see $LOG_DIR/onboard.log"
fi

echo ""
if [ "$ALL_ONBOARD" = true ]; then
    log "Full exploration stack running on the Jetson."
else
    log "Onboard exploration stack running. On the GPU machine (ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}):"
    log "  scripts/offboard.sh explore"
fi
log "Then start:  ros2 topic pub --once /mission/control std_msgs/msg/String \"data: start\""
log "Stop:        ros2 topic pub --once /mission/control std_msgs/msg/String \"data: end\""
warn "Keep a hand on the power switch — keyboard teleop does not override Nav2 here."
log "Ctrl+C to stop all onboard nodes."
wait
