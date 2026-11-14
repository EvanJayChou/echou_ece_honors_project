#!/usr/bin/env bash
# explore.sh — autonomous frontier exploration (Q1 baseline), all on the Jetson.
#
# Runs the sensors, EKF, VESC driver, slam_toolbox, the full Nav2 stack
# (planner, BT navigator, behaviors, controller, velocity smoother), the
# cmd_vel → /ackermann_cmd converter, and frontier_explorer. No second machine
# needed; only the Q2 3DGS pipeline runs off-board.
#
# Start exploring once the stack is up:
#   ros2 topic pub --once /mission/control std_msgs/msg/String "data: start"
# Stop (cancels the active Nav2 goal):
#   ros2 topic pub --once /mission/control std_msgs/msg/String "data: end"
#
# Usage:
#   scripts/explore.sh                  everything on the Jetson (default)
#   scripts/explore.sh --offboard-slam  slam_toolbox off-board instead (GPU machine: scripts/offboard.sh slam)
#   scripts/explore.sh --auto-start     begin exploring without waiting for /mission/control "start"
#   scripts/explore.sh --rviz           RViz on the Jetson display (CPU heavy)

_TAG=explore
source "$(dirname "$0")/common.sh"

USE_SLAM=true
USE_RVIZ=false
AUTO_START=false
for arg in "$@"; do
    case "$arg" in
        --offboard-slam) USE_SLAM=false ;;
        --auto-start)    AUTO_START=true ;;
        --rviz)          USE_RVIZ=true ;;
        --all-onboard|--onboard-slam)
            warn "$arg is the default now; ignoring it" ;;
        *) err "Unknown flag: $arg  (valid: --offboard-slam, --auto-start, --rviz)"; exit 1 ;;
    esac
done

log "Preflight..."
check_power_mode
check_lidar
check_oakd
resolve_vesc_port
source_ws "$ONBOARD_WS"

start_lidar
start_oakd
start_vesc
sleep 3
start_onboard use_slam:="$USE_SLAM" use_nav2:=true use_explorer:=true \
    explorer_auto_start:="$AUTO_START" use_rviz:="$USE_RVIZ"

sleep 8
check_topic /scan_fixed 5 "/scan_fixed publishing" "/scan_fixed missing — see $LOG_DIR/lidar.log and $LOG_DIR/onboard.log"
check_topic /oakd/imu 5 "/oakd/imu publishing" "/oakd/imu missing — see $LOG_DIR/oakd.log"
check_topic /odometry/filtered 5 "EKF /odometry/filtered publishing" "EKF output missing — see $LOG_DIR/onboard.log"
ACTIONS=$(ros2 action list 2>/dev/null)
for action in /follow_path /navigate_to_pose; do
    if echo "$ACTIONS" | grep -qx "$action"; then
        ok "Nav2 action $action up"
    else
        warn "$action action not found — see $LOG_DIR/onboard.log"
    fi
done

echo ""
if [ "$USE_SLAM" = true ]; then
    log "Exploration stack running on the Jetson (slam_toolbox starts ~5 s after the onboard launch)."
else
    log "Exploration stack running without SLAM. On the GPU machine (ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}):"
    log "  scripts/offboard.sh slam"
fi
if [ "$AUTO_START" = false ]; then
    log "Start:       ros2 topic pub --once /mission/control std_msgs/msg/String \"data: start\""
fi
log "Stop:        ros2 topic pub --once /mission/control std_msgs/msg/String \"data: end\""
warn "Keep a hand on the power switch — keyboard teleop does not override Nav2 here."
log "Ctrl+C to stop all onboard nodes."
wait
