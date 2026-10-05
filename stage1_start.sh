#!/bin/bash
# stage1_start.sh — One-shot Stage 1 mapping stack launcher
#
# Starts everything needed for Stage 1 manual mapping in a single terminal:
#
#   pico_hw_server         /dev/ttyACM2       → Pico serial bridge (LED strip)
#   LiDAR (Livox Mid-360)  Ethernet 192.168.1.12 → /scan      (via pointcloud_to_laserscan, ~10 Hz)
#   IMU bridge (XIAO)      /dev/ttyACM0       → /odom + /imu  (~50 Hz)
#   scan_relay_node        (inside SLAM launch) → /scan_fixed  (300 beams, 250° FOV)
#   ekf_local_node         (inside SLAM launch) → odom→base_link TF (25 Hz)
#   robot_state_publisher  (inside SLAM launch) → static TFs
#   slam_toolbox           (inside SLAM launch, +10 s delay) → /map
#   ackermann_to_vesc_node (inside VESC launch) → /commands/motor + /commands/servo
#   vesc_driver_node       (inside VESC launch) /dev/ttyACM1
#   led_state_monitor                           → watches /ackermann_cmd, drives LED strip
#   keyboard_teleop_node   foreground — captures keys in this terminal
#
# LED strip states (WS2812B via Pico on /dev/ttyACM2):
#   GREEN  (LED:SUCCESS) — rover idle, no drive command
#   BLUE   (LED:UNKNOWN) — rover actively moving / exploring
#   RED    (LED:FAILURE) — startup error or unexpected shutdown
#   OFF    (LED:OFF)     — clean shutdown (user quit / Ctrl+C)
#
# CPU constraints (Raspberry Pi 5):
#   Nav2 is always OFF — it saturates CPU alongside SLAM.
#   RViz is OFF by default — run on a remote machine instead (same ROS_DOMAIN_ID).
#
# Timing sequence:
#   t=0s   Pico hw server + LiDAR start
#   t=3s   IMU bridge starts
#   t=7s   SLAM stack starts → scan_relay + EKF + robot_state_publisher up immediately
#   t=17s  slam_toolbox activates (10 s internal TimerAction in slam.launch.py)
#   t=12s  VESC driver starts
#   t=15s  LED state monitor starts
#
# Usage:
#   ./stage1_start.sh              full stack; keyboard teleop in foreground
#   ./stage1_start.sh --headless   start all nodes, skip interactive teleop
#   ./stage1_start.sh --rviz       enable RViz on local display (Pi CPU warning)
#   ./stage1_start.sh --full       headless + survey_camera launch + mission_report web server (nvm 20 / npm run dev)

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WS="$SCRIPT_DIR/slam_ros2_ws"

# ── Parse flags ────────────────────────────────────────────────────────────────
HEADLESS=false
USE_RVIZ=false
FULL=false
for arg in "$@"; do
    case "$arg" in
        --headless) HEADLESS=true ;;
        --rviz)     USE_RVIZ=true ;;
        --full)     FULL=true; HEADLESS=true ;;
        *) echo "Unknown flag: $arg  (valid: --headless, --rviz, --full)"; exit 1 ;;
    esac
done

# ── Colours ───────────────────────────────────────────────────────────────────
RED='\033[0;31m'
YEL='\033[1;33m'
GRN='\033[0;32m'
CYN='\033[0;36m'
NC='\033[0m'
log()  { echo -e "${CYN}[stage1]${NC} $*"; }
warn() { echo -e "${YEL}[stage1 WARN]${NC} $*"; }
ok()   { echo -e "${GRN}[stage1]${NC}   ✓ $*"; }
err()  { echo -e "${RED}[stage1 ERROR]${NC} $*"; }

# ── Startup state ─────────────────────────────────────────────────────────────
# Set to "true" once all nodes are running. The cleanup trap uses this to
# decide whether to send LED:FAILURE (startup never completed) or LED:OFF (clean exit).
STARTUP_OK=false

# ── Cleanup trap ──────────────────────────────────────────────────────────────
PIDS=()

_ALL_NODES="livox_ros_driver2_node|livox_pointcloud_to_laserscan|serial_bridge_node|scan_relay_node|ekf_node|slam_toolbox|\
robot_state_publisher|ackermann_to_vesc_node|vesc_driver_node|\
keyboard_teleop_node|pico_hw_server|led_state_monitor|survey_camera"

cleanup() {
    echo ""
    log "Shutting down..."

    # Set LED state while pico_hw_server is still alive.
    # LED:FAILURE if startup never completed; LED:OFF for any clean exit.
    if [ "$STARTUP_OK" = "true" ]; then
        log "LED → OFF (clean shutdown)"
        timeout 3 ros2 service call /pico/led_off std_srvs/srv/Trigger '{}' \
            > /dev/null 2>&1 || true
    else
        log "LED → FAILURE (startup did not complete)"
        timeout 3 ros2 service call /pico/led_failure std_srvs/srv/Trigger '{}' \
            > /dev/null 2>&1 || true
    fi

    # Graceful SIGINT to all background processes, then SIGKILL after 2 s
    for pid in "${PIDS[@]}"; do
        kill "$pid" 2>/dev/null || true
    done
    pkill -SIGINT -f "$_ALL_NODES" 2>/dev/null || true
    sleep 2
    pkill -9 -f "$_ALL_NODES" 2>/dev/null || true
    log "All nodes stopped."
}
trap cleanup EXIT INT TERM

# ── Preflight hardware checks ─────────────────────────────────────────────────
log "Preflight hardware checks..."
PREFAIL=0

if ! ping -c 1 -W 1 192.168.1.12 > /dev/null 2>&1; then
    warn "Livox Mid-360 (192.168.1.12) not reachable — check Ethernet link and host static IP (192.168.1.50)"
    PREFAIL=$((PREFAIL + 1))
fi

if ! ls /dev/ttyACM* > /dev/null 2>&1; then
    warn "No /dev/ttyACM* devices — XIAO, VESC, and Pico may not be connected"
    PREFAIL=$((PREFAIL + 1))
else
    [ -e /dev/ttyACM1 ] || warn "/dev/ttyACM1 not found — VESC may not be connected"
    [ -e /dev/ttyACM2 ] || warn "/dev/ttyACM2 not found — Pico LED bridge may not be connected (LED feedback disabled)"
fi

if [ "$PREFAIL" -gt 0 ]; then
    err "$PREFAIL preflight check(s) failed. Check hardware connections."
    err "Continuing in 5 s — expect node failures for missing devices."
    sleep 5
else
    ok "Hardware devices present (Livox Mid-360, /dev/ttyACM0-2)"
fi

# ── Source ROS 2 environment ──────────────────────────────────────────────────
log "Sourcing ROS 2 Jazzy and workspace..."
if [ ! -f /opt/ros/jazzy/setup.bash ]; then
    err "/opt/ros/jazzy/setup.bash not found — is ROS 2 Jazzy installed?"
    exit 1
fi
if [ ! -f "$WS/install/setup.bash" ]; then
    err "$WS/install/setup.bash not found — run: cd $WS && colcon build --symlink-install"
    exit 1
fi
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"

# ── Kill stale nodes from any previous run ────────────────────────────────────
log "Clearing stale nodes from any previous run..."
pkill -9 -f "$_ALL_NODES" 2>/dev/null || true
sleep 2

# ── [0/5] Pico hardware server ───────────────────────────────────────────────
# Start early — pico_hw_server has its own 10-attempt retry loop (up to 10 s)
# plus a 2 s post-connect wait. Launching it now gives it time to connect
# while LiDAR and IMU are spinning up, so LED:SUCCESS fires as soon as the
# rover is ready.
log "[0/5] Starting Pico hardware server (LED strip + camera servo)..."
ros2 run pico_hw_bridge pico_hw_server \
    > /tmp/stage1_pico.log 2>&1 &
PIDS+=($!)
# No blocking wait — pico_hw_server retries internally. Continue launching other nodes.

# ── [1/5] LiDAR (Livox Mid-360) ──────────────────────────────────────────────
log "[1/5] Starting LiDAR (Livox Mid-360)..."
ros2 launch livox_bringup livox_mid360.launch.py \
    > /tmp/stage1_lidar.log 2>&1 &
PIDS+=($!)
sleep 3

if timeout 5 ros2 topic hz /scan 2>&1 | grep -q "average rate"; then
    ok "/scan publishing (~10 Hz)"
else
    warn "/scan not yet detected — check Livox Ethernet link and MID360_config.json IPs"
fi

# ── [2/5] IMU bridge (XIAO nRF52840 Sense) ───────────────────────────────────
log "[2/5] Starting IMU bridge (XIAO nRF52840)..."
# publish_tf:=false — the EKF owns odom→base_link TF exclusively.
# If the serial bridge also publishes it (default: true) it wins at 50 Hz and
# overwrites the EKF's position with near-zero accel dead-reckoning, making
# slam_toolbox think the robot never moves.
ros2 run xiao_serial_bridge serial_bridge_node \
    --ros-args -p publish_tf:=false \
    > /tmp/stage1_imu.log 2>&1 &
PIDS+=($!)
sleep 4

if timeout 5 ros2 topic hz /odom 2>&1 | grep -q "average rate"; then
    ok "/odom publishing (~50 Hz)"
else
    warn "/odom not yet detected — replug XIAO USB and wait ~5 s for CDC port to reappear"
    warn "The bridge reconnects automatically; retrying is not required."
fi

# ── [3/5] SLAM stack ─────────────────────────────────────────────────────────
# scan_relay_node, ekf_local_node, and robot_state_publisher start immediately.
# slam_toolbox is held by a 10 s TimerAction inside slam.launch.py so the EKF
# has time to publish odom→base_link TF before the first scan lookup.
#
# WARNING: do NOT run scan_relay_node manually before or alongside this launch.
# Two instances send interleaved scans with mismatched beam counts to slam_toolbox.
log "[3/5] Starting SLAM stack (scan_relay + EKF + slam_toolbox in ~10 s)..."

if [ "$USE_RVIZ" = "true" ]; then
    warn "RViz enabled — this competes with SLAM and EKF for Pi CPU."
    warn "If the map stalls, close RViz and use the service call to verify the map."
    RVIZ_ARG="true"
else
    RVIZ_ARG="false"
fi

ros2 launch robot_slam launch_stage_1.launch.py \
    use_rviz:="$RVIZ_ARG" \
    > /tmp/stage1_slam.log 2>&1 &
PIDS+=($!)
sleep 5  # give EKF and scan_relay time to start before VESC

# ── [4/5] VESC driver (driving / steering) ───────────────────────────────────
# Starts ackermann_to_vesc_node and vesc_driver_node.
# Independent of SLAM — no ordering constraint relative to EKF.
log "[4/5] Starting VESC driver (ackermann_to_vesc + vesc_driver)..."
ros2 launch ucsd_robocar_control2_pkg keyboard_teleop.launch.py \
    > /tmp/stage1_vesc.log 2>&1 &
PIDS+=($!)
sleep 3

if timeout 5 ros2 topic hz /sensors/core 2>&1 | grep -q "average rate"; then
    ok "/sensors/core publishing (~50 Hz VESC telemetry)"
else
    warn "/sensors/core not detected — check VESC USB (/dev/ttyACM1)"
    warn "Ensure servo output is enabled in VESC Tool: App Settings → General → Use Servo Output → Write App Configuration"
fi

# ── [5/5] LED state monitor ───────────────────────────────────────────────────
# Subscribes to /ackermann_cmd and calls pico/led_success or pico/led_unknown
# based on whether the rover is moving.
#   speed ≈ 0  → LED:SUCCESS (green)
#   |speed| > 0.01 m/s → LED:UNKNOWN (blue / exploring)
log "[5/5] Starting LED state monitor..."
ros2 run pico_hw_bridge led_state_monitor \
    > /tmp/stage1_led.log 2>&1 &
PIDS+=($!)

# All core nodes are running — mark startup complete.
# Cleanup will now send LED:OFF instead of LED:FAILURE.
STARTUP_OK=true

# ── Status banner ─────────────────────────────────────────────────────────────
echo ""
echo -e "${CYN}════════════════════════════════════════════════════════════════${NC}"
echo -e "${GRN}  Stage 1 stack is running${NC}"
echo -e "${CYN}════════════════════════════════════════════════════════════════${NC}"
echo "  slam_toolbox will activate ~10 s after SLAM launch (timer running)"
echo ""
echo "  LED strip (Pico /dev/ttyACM2):"
echo "    GREEN  — rover idle (not moving)"
echo "    BLUE   — rover driving / exploring"
echo "    RED    — failure / unexpected shutdown"
echo ""
echo "  Expected nodes:"
echo "    /pico_hw_server  /led_state_monitor"
echo "    /livox_lidar_publisher  /livox_pointcloud_to_laserscan  /serial_bridge_node  /scan_relay_node"
echo "    /ekf_local_node  /robot_state_publisher  /slam_toolbox"
echo "    /ackermann_to_vesc_node  /vesc_driver_node"
echo ""
echo "  Logs:"
echo "    Pico   → /tmp/stage1_pico.log"
echo "    LiDAR  → /tmp/stage1_lidar.log"
echo "    IMU    → /tmp/stage1_imu.log"
echo "    SLAM   → /tmp/stage1_slam.log"
echo "    VESC   → /tmp/stage1_vesc.log"
echo "    LED    → /tmp/stage1_led.log"
if [ "$FULL" = "true" ]; then
echo "    Camera → /tmp/stage1_survey_camera.log"
echo "    Web    → /tmp/stage1_mission_report.log"
fi
echo ""
echo "  Verify map is building (~20 s after this script started):"
echo "    ros2 service call /slam_toolbox/dynamic_map nav_msgs/srv/GetMap"
echo ""
echo "  Save map when done:"
echo "    ros2 run nav2_map_server map_saver_cli \\"
echo "      -f ~/scout-survey-rover/slam_ros2_ws/src/robot_slam/ros_data/maps/custom/stage1_map"
echo -e "${CYN}════════════════════════════════════════════════════════════════${NC}"
echo ""

# ── [--full] Survey camera + mission report web server ────────────────────────
if [ "$FULL" = "true" ]; then
    log "[full] Starting survey camera..."
    ros2 launch survey_camera survey_camera.launch.py \
        > /tmp/stage1_survey_camera.log 2>&1 &
    PIDS+=($!)
    ok "survey_camera launched → /tmp/stage1_survey_camera.log"

    log "[full] Starting mission report web server (nvm 20 / npm run dev)..."
    # nvm is a shell function — must source nvm.sh before use.
    bash -c 'source ~/.nvm/nvm.sh && nvm use 20 && cd "'"$SCRIPT_DIR/mission_report"'" && npm run dev' \
        > /tmp/stage1_mission_report.log 2>&1 &
    PIDS+=($!)
    ok "mission_report dev server launched → /tmp/stage1_mission_report.log"

    _NET_IP=$(hostname -I 2>/dev/null | awk '{print $1}')
    echo ""
    echo -e "  ${GRN}Web dashboard:${NC}"
    echo    "    Local:    http://localhost:3000"
    [ -n "$_NET_IP" ] && echo "    Network:  http://${_NET_IP}:3000"
    echo ""
fi

# ── Keyboard teleop or headless wait ─────────────────────────────────────────
if [ "$HEADLESS" = "true" ]; then
    log "Headless mode — keyboard teleop not started."
    log "To drive manually, open a new terminal and run:"
    log "  ros2 run ucsd_robocar_control2_pkg keyboard_teleop_node"
    echo ""
    log "Press Ctrl+C to stop all nodes (LED will go OFF)."
    wait
else
    log "Starting keyboard teleop (this terminal captures keystrokes)."
    echo ""
    echo "  Keys:  w / s     = forward / backward"
    echo "         a / d     = steer left / right"
    echo "         space     = stop immediately"
    echo "         r / f     = increase / decrease speed step"
    echo "         e / c     = increase / decrease steer step"
    echo "         q         = quit (sends LED:OFF, stops all nodes)"
    echo ""
    # Foreground — blocks until user presses q or Ctrl+C.
    # EXIT trap fires on return: sends LED:OFF and kills all background nodes.
    ros2 run ucsd_robocar_control2_pkg keyboard_teleop_node
fi
