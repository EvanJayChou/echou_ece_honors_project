#!/usr/bin/env bash
# teleop_slam.sh — keyboard teleop while mapping with SLAM.
#
#   livox_bringup     → /scan                    (Mid-360 point cloud flattened to 2D)
#   oakd_publisher    → /oakd/imu (+ RGB-D)      (OAK-D Pro onboard IMU)
#   VESC stack        → /vesc_odom, drives the car from /ackermann_cmd
#   onboard.launch.py → scan_relay (/scan_fixed), ekf_local (odom→base_link), robot_state_publisher
#   slam_toolbox      → /map + map→odom          (on the Jetson by default, or off-board)
#   keyboard_teleop   → /ackermann_cmd           (foreground)
#
# Usage:
#   scripts/teleop_slam.sh                   SLAM on the Jetson, keyboard teleop here
#   scripts/teleop_slam.sh --offboard-slam   no SLAM on the Jetson; run `scripts/offboard.sh slam` on the GPU machine
#   scripts/teleop_slam.sh --headless        no teleop; drive from another terminal
#   scripts/teleop_slam.sh --rviz            RViz on the Jetson display (CPU heavy)

_TAG=teleop_slam
source "$(dirname "$0")/common.sh"

USE_SLAM=true
HEADLESS=false
USE_RVIZ=false
for arg in "$@"; do
    case "$arg" in
        --offboard-slam) USE_SLAM=false ;;
        --headless)      HEADLESS=true ;;
        --rviz)          USE_RVIZ=true ;;
        *) err "Unknown flag: $arg  (valid: --offboard-slam, --headless, --rviz)"; exit 1 ;;
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
start_onboard use_slam:="$USE_SLAM" use_rviz:="$USE_RVIZ"

sleep 8
check_topic /scan_fixed 5 "/scan_fixed publishing" "/scan_fixed missing — see $LOG_DIR/lidar.log and $LOG_DIR/onboard.log"
check_topic /oakd/imu 5 "/oakd/imu publishing" "/oakd/imu missing — see $LOG_DIR/oakd.log"
check_topic /vesc_odom 5 "/vesc_odom publishing" "/vesc_odom missing — see $LOG_DIR/vesc.log"
check_topic /odometry/filtered 5 "EKF /odometry/filtered publishing" "EKF output missing — see $LOG_DIR/onboard.log"

echo ""
if [ "$USE_SLAM" = true ]; then
    log "slam_toolbox starts ~5 s after the onboard launch. Check the map with:"
    log "  ros2 service call /slam_toolbox/dynamic_map nav_msgs/srv/GetMap"
    log "Save it with:"
    log "  mkdir -p $ONBOARD_WS/src/robot_slam/ros_data/maps/custom"
    log "  ros2 run nav2_map_server map_saver_cli -f $ONBOARD_WS/src/robot_slam/ros_data/maps/custom/stage1_map"
else
    log "Off-board SLAM: on the GPU machine (same ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}) run:"
    log "  scripts/offboard.sh slam"
fi

if [ "$HEADLESS" = true ]; then
    log "Headless — drive from another terminal with:"
    log "  ros2 run ucsd_robocar_control2_pkg keyboard_teleop_node"
    log "Ctrl+C to stop all nodes."
    wait
else
    run_teleop_foreground
fi
