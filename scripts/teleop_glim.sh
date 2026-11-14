#!/usr/bin/env bash
# teleop_glim.sh — keyboard (or F710) teleop while mapping in 3D with GLIM.
#
#   livox_bringup    → /livox/lidar + /livox/imu   (Mid-360, xfer_format 0)
#   VESC stack       → drives the car from /ackermann_cmd, /vesc_odom
#   glim.launch.py   → /glim_ros/odom, /glim_ros/map, TF glim_map → … → glim_lidar
#   keyboard_teleop  → /ackermann_cmd              (foreground; or F710 gamepad with --joy)
#   record.sh        → raw bag for offline re-mapping (with --record)
#
# GLIM runs alongside nothing else: no EKF, slam_toolbox or OAK-D (see docs/glim.md).
# Ctrl+C (or q in keyboard teleop) stops driving, then lets GLIM finish global
# optimization and write its map dump before exiting.
#
# Usage:
#   scripts/teleop_glim.sh             keyboard teleop, GLIM on the GPU
#   scripts/teleop_glim.sh --joy       drive with the Logitech F710 gamepad
#   scripts/teleop_glim.sh --record    also record LiDAR/IMU/TF/odom (record.sh --no-images)
#   scripts/teleop_glim.sh --cpu       GLIM CPU configs (fallback; drifts more, docs/glim.md §5)
#   scripts/teleop_glim.sh --headless  no teleop; drive from another terminal
#
# The map dump goes to ros_data/maps/custom/glim_<date>_<time>/ (the host's maps/,
# so it survives reboots); override with GLIM_DUMP_PATH. View or export it with:
#   ros2 run glim_ros offline_viewer <dump>

_TAG=teleop_glim
source "$(dirname "$0")/common.sh"

USE_GPU=true
USE_JOY=false
USE_RECORD=false
HEADLESS=false
for arg in "$@"; do
    case "$arg" in
        --joy)      USE_JOY=true ;;
        --record)   USE_RECORD=true ;;
        --cpu)      USE_GPU=false ;;
        --headless) HEADLESS=true ;;
        *) err "Unknown flag: $arg  (valid: --joy, --record, --cpu, --headless)"; exit 1 ;;
    esac
done

DUMP_PATH="${GLIM_DUMP_PATH:-$ONBOARD_WS/src/robot_slam/ros_data/maps/custom/glim_$(date +%Y%m%d_%H%M%S)}"
GLIM_SAVE_TIMEOUT=90   # s; glim.launch.py itself escalates to SIGTERM after 60 s

# ── Processes that must shut down cleanly ─────────────────────────────────────
# common.sh's cleanup SIGKILLs everything 2 s after SIGINT. GLIM needs longer to
# optimize and save its map, and ros2 bag record to close the MCAP, so these are
# tracked separately and waited for. Signal only the top process: ros2 launch
# forwards SIGINT to glim_rosnode, and a second SIGINT kills it mid-save.
GRACEFUL_PIDS=()

start_graceful() {
    spawn_bg "$@"
    GRACEFUL_PIDS+=("$SPAWNED_PID")
}

glim_cleanup() {
    trap - EXIT INT TERM
    local pid
    for pid in "${GRACEFUL_PIDS[@]}"; do
        kill -INT "$pid" 2>/dev/null || true
    done
    cleanup   # LiDAR, VESC, joy: SIGINT, then SIGKILL after 2 s
    [ "${#GRACEFUL_PIDS[@]}" -eq 0 ] && return

    log "Waiting for GLIM to optimize and save the map (up to ${GLIM_SAVE_TIMEOUT} s)..."
    ( sleep "$GLIM_SAVE_TIMEOUT"
      for pid in "${GRACEFUL_PIDS[@]}"; do kill -9 -- "-$pid" 2>/dev/null; done ) &
    local watchdog=$!
    for pid in "${GRACEFUL_PIDS[@]}"; do
        wait "$pid" 2>/dev/null
        kill -9 -- "-$pid" 2>/dev/null   # anything the launch left behind
    done
    kill "$watchdog" 2>/dev/null

    if [ -f "$DUMP_PATH/graph.bin" ]; then
        ok "GLIM map saved: $DUMP_PATH"
        log "  view / export PLY: ros2 run glim_ros offline_viewer $DUMP_PATH"
    else
        err "No GLIM dump in $DUMP_PATH — see $LOG_DIR/glim.log"
    fi
}
trap glim_cleanup EXIT INT TERM

# ── Bring-up ──────────────────────────────────────────────────────────────────
log "Preflight..."
check_power_mode
check_lidar
[ "$USE_JOY" = true ] && check_f710
resolve_vesc_port
source_ws "$ONBOARD_WS"
mkdir -p "$(dirname "$DUMP_PATH")"

start_lidar
start_vesc
sleep 8
check_topic /livox/lidar 5 "/livox/lidar publishing" "/livox/lidar missing — see $LOG_DIR/lidar.log"
check_topic /livox/imu 5 "/livox/imu publishing" "/livox/imu missing — see $LOG_DIR/lidar.log"
check_topic /vesc_odom 5 "/vesc_odom publishing" "/vesc_odom missing — see $LOG_DIR/vesc.log"

warn "Keep the rover STILL until GLIM is up — it estimates the initial IMU state from the first frames"
start_graceful glim ros2 launch robot_slam glim.launch.py use_gpu:="$USE_GPU" dump_path:="$DUMP_PATH"
sleep 6
check_topic /glim_ros/odom 8 "GLIM odometry publishing (/glim_ros/odom)" \
    "/glim_ros/odom missing — see $LOG_DIR/glim.log and /tmp/rover/glim_*.log"

if [ "$USE_RECORD" = true ]; then
    start_graceful record "$SCRIPTS_DIR/record.sh" --no-images
fi

echo ""
log "Map dump on exit: $DUMP_PATH"
log "Remote RViz (ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}):"
log "  rviz2 -d \$(ros2 pkg prefix robot_slam)/share/robot_slam/rviz/glim.rviz"
log "Drive slowly and finish where you started so GLIM can close the loop."

if [ "$HEADLESS" = true ]; then
    log "Headless — drive from another terminal with:"
    log "  ros2 run ucsd_robocar_control2_pkg keyboard_teleop_node"
    log "  or: ros2 launch ucsd_robocar_control2_pkg f710_teleop.launch.py"
    log "Ctrl+C to stop and save the map."
    wait
elif [ "$USE_JOY" = true ]; then
    start_joy
    sleep 2
    check_topic /joy 3 "/joy publishing (F710)" \
        "/joy not detected — check the receiver, the X/D switch, and $LOG_DIR/joy.log"
    run_joy_foreground
else
    run_teleop_foreground
fi
