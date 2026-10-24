#!/usr/bin/env bash
# common.sh — shared helpers for the rover launch scripts. Source it, don't run it.
#
# Provides: logging, workspace sourcing, background-process bookkeeping with a
# cleanup trap, topic-rate checks, and hardware preflight checks.
#
# Environment overrides:
#   ROS_DISTRO_NAME   ROS 2 distro under /opt/ros     (default: $ROS_DISTRO, else jazzy)
#   LOG_DIR           where per-component logs go      (default: /tmp/rover)
#   VESC_PORT         VESC serial device               (default: /dev/vesc, else /dev/ttyACM0)
#   LIVOX_IP          Mid-360 address for preflight    (default: 192.168.1.12)

SCRIPTS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPTS_DIR/.." && pwd)"
ONBOARD_WS="$REPO_ROOT/ros2_ws"
OFFBOARD_WS="$REPO_ROOT/ros2_ws_offboard"

ROS_DISTRO_NAME="${ROS_DISTRO_NAME:-${ROS_DISTRO:-jazzy}}"
LOG_DIR="${LOG_DIR:-/tmp/rover}"
LIVOX_IP="${LIVOX_IP:-192.168.1.12}"
mkdir -p "$LOG_DIR"

# ── Logging ───────────────────────────────────────────────────────────────────
RED='\033[0;31m'
YEL='\033[1;33m'
GRN='\033[0;32m'
CYN='\033[0;36m'
NC='\033[0m'
_TAG="${_TAG:-rover}"
log()  { echo -e "${CYN}[${_TAG}]${NC} $*"; }
warn() { echo -e "${YEL}[${_TAG} WARN]${NC} $*"; }
ok()   { echo -e "${GRN}[${_TAG}]${NC}   ✓ $*"; }
err()  { echo -e "${RED}[${_TAG} ERROR]${NC} $*"; }

# ── Workspace sourcing ────────────────────────────────────────────────────────
# source_ws <workspace dir>
source_ws() {
    local ws="$1"
    if [ ! -f "/opt/ros/$ROS_DISTRO_NAME/setup.bash" ]; then
        err "/opt/ros/$ROS_DISTRO_NAME/setup.bash not found — is ROS 2 $ROS_DISTRO_NAME installed?"
        exit 1
    fi
    if [ ! -f "$ws/install/setup.bash" ]; then
        err "$ws/install/setup.bash not found — build it first (scripts/build.sh)"
        exit 1
    fi
    # setup.bash references unset variables; don't let a caller's `set -u` trip on it.
    set +u
    source "/opt/ros/$ROS_DISTRO_NAME/setup.bash"
    source "$ws/install/setup.bash"
    log "Sourced ROS 2 $ROS_DISTRO_NAME + $(basename "$ws")  (ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0})"
}

# source_overlay <workspace dir> — add a second workspace on top of the current
# environment (local_setup.bash leaves already-sourced workspaces in place).
source_overlay() {
    local ws="$1"
    if [ ! -f "$ws/install/local_setup.bash" ]; then
        err "$ws/install/local_setup.bash not found — build it first (scripts/build.sh)"
        exit 1
    fi
    set +u
    source "$ws/install/local_setup.bash"
    log "Overlaid $(basename "$ws")"
}

# ── Background processes + cleanup ────────────────────────────────────────────
PIDS=()

# start_bg <name> <command...> — run in the background, logging to $LOG_DIR/<name>.log
start_bg() {
    local name="$1"; shift
    "$@" > "$LOG_DIR/$name.log" 2>&1 &
    PIDS+=($!)
    log "started $name  (log: $LOG_DIR/$name.log)"
}

cleanup() {
    trap - EXIT INT TERM
    echo ""
    log "Shutting down..."
    for pid in "${PIDS[@]}"; do
        kill -INT "$pid" 2>/dev/null || true
    done
    sleep 2
    for pid in "${PIDS[@]}"; do
        kill -9 "$pid" 2>/dev/null || true
    done
    log "All nodes stopped."
}
trap cleanup EXIT INT TERM

# ── Checks ────────────────────────────────────────────────────────────────────
# check_topic <topic> <timeout_s> <ok message> <warn message>
check_topic() {
    if timeout "$2" ros2 topic hz "$1" 2>&1 | grep -q "average rate"; then
        ok "$3"
    else
        warn "$4"
    fi
}

# Jetson only: report the power mode. SLAM + Nav2 onboard wants the highest
# mode (MAXN); `sudo jetson_clocks` additionally pins clocks at max.
check_power_mode() {
    command -v nvpmodel > /dev/null 2>&1 || return 0
    local mode
    mode=$(timeout 3 nvpmodel -q 2>/dev/null | grep -m1 "Power Mode" | sed 's/.*Power Mode: *//')
    if [ -z "$mode" ]; then
        return 0
    elif echo "$mode" | grep -qi "MAXN"; then
        ok "Jetson power mode: $mode"
    else
        warn "Jetson power mode is $mode — for SLAM/Nav2 use the highest mode (sudo nvpmodel -m <MAXN id>; see nvpmodel -p --verbose)"
    fi
}

check_lidar() {
    if ping -c 1 -W 1 "$LIVOX_IP" > /dev/null 2>&1; then
        ok "Livox Mid-360 reachable at $LIVOX_IP"
    else
        warn "Livox Mid-360 ($LIVOX_IP) not reachable — check Ethernet link and host static IP (192.168.1.50)"
    fi
}

# Logitech F710 receiver: 046d:c21f in X (XInput) mode, 046d:c219 in D mode.
check_f710() {
    if ! command -v lsusb > /dev/null 2>&1; then
        warn "lsusb not installed — skipping F710 presence check"
    elif lsusb | grep -q "046d:c21f"; then
        ok "Logitech F710 receiver detected (X mode)"
    elif lsusb | grep -q "046d:c219"; then
        warn "Logitech F710 is in D mode — slide the switch on the back of the controller to X"
    else
        warn "Logitech F710 receiver not found on USB — plug in the nano receiver"
    fi
}

check_oakd() {
    if ! command -v lsusb > /dev/null 2>&1; then
        warn "lsusb not installed — skipping OAK-D presence check"
    elif lsusb | grep -q "03e7:"; then
        ok "OAK-D Pro detected on USB"
    else
        warn "OAK-D Pro not found on USB (vendor 03e7) — use a USB 3 port and check the cable"
    fi
}

# Sets VESC_PORT if the caller didn't.
resolve_vesc_port() {
    if [ -z "${VESC_PORT:-}" ]; then
        if [ -e /dev/vesc ]; then
            VESC_PORT=/dev/vesc
        else
            VESC_PORT=/dev/ttyACM0
        fi
    fi
    if [ -e "$VESC_PORT" ]; then
        ok "VESC port: $VESC_PORT"
    else
        warn "VESC port $VESC_PORT not found — plug in the VESC, install udev/99-rover.rules, or set VESC_PORT"
    fi
}

# ── Common stack pieces ───────────────────────────────────────────────────────
start_lidar() {
    start_bg lidar ros2 launch livox_bringup livox_mid360.launch.py
}

start_oakd() {
    start_bg oakd ros2 launch oakd_publisher oakd_publisher.launch.py
    warn "Keep the rover STILL for ~8 s — OAK-D boot + 5 s gyro bias calibration"
}

start_vesc() {
    start_bg vesc ros2 launch ucsd_robocar_control2_pkg keyboard_teleop.launch.py \
        vesc_port:="$VESC_PORT"
}

# start_onboard <extra launch args...> — scan relay + EKF + robot_state_publisher (+ options)
start_onboard() {
    start_bg onboard ros2 launch robot_slam onboard.launch.py "$@"
}

print_teleop_keys() {
    echo ""
    echo "  Keys:  w / s     = forward / backward (each press steps speed)"
    echo "         a / d     = steer left / right"
    echo "         space     = stop immediately"
    echo "         r / f     = increase / decrease speed step"
    echo "         e / c     = increase / decrease steer step"
    echo "         q         = quit (stops all nodes)"
    echo ""
}

run_teleop_foreground() {
    print_teleop_keys
    ros2 run ucsd_robocar_control2_pkg keyboard_teleop_node
}

# Logitech F710 gamepad: joy game_controller_node + f710_teleop_node → /ackermann_cmd
start_joy() {
    start_bg joy ros2 launch ucsd_robocar_control2_pkg f710_teleop.launch.py
}

print_joy_controls() {
    echo ""
    echo "  Logitech F710 (switch on X; press the Logitech button if the LED is off):"
    echo "         left stick  ←/→   = steer"
    echo "         right stick ↑/↓   = throttle forward / reverse"
    echo "         A                 = emergency stop (center throttle stick to re-arm)"
    echo "         Back              = constant throttle on / off"
    echo "         D-pad ↑ / ↓       = increase / decrease max speed"
    echo "         Ctrl+C            = quit (stops all nodes)"
    echo "  Log: $LOG_DIR/joy.log"
    echo ""
}

run_joy_foreground() {
    print_joy_controls
    wait
}
