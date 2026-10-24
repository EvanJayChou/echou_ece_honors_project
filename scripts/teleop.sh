#!/usr/bin/env bash
# teleop.sh — drive the rover with the keyboard. VESC only, no sensors.
#
#   keyboard_teleop_node → /ackermann_cmd → ackermann_to_vesc → vesc_driver → VESC
#     (or with --joy: game_controller_node → /joy → f710_teleop_node → /ackermann_cmd)
#   vesc_to_odom → /vesc_odom (wheel odometry, used by the EKF in the other scripts)
#
# Usage:
#   scripts/teleop.sh              keyboard teleop in this terminal
#   scripts/teleop.sh --joy        Logitech F710 gamepad teleop
#   VESC_PORT=/dev/ttyACM1 scripts/teleop.sh

_TAG=teleop
source "$(dirname "$0")/common.sh"

USE_JOY=false
for arg in "$@"; do
    case "$arg" in
        --joy) USE_JOY=true ;;
        *) err "Unknown flag: $arg  (valid: --joy)"; exit 1 ;;
    esac
done

log "Preflight..."
[ "$USE_JOY" = true ] && check_f710
resolve_vesc_port
source_ws "$ONBOARD_WS"

start_vesc
sleep 3
check_topic /sensors/core 5 "/sensors/core publishing (VESC telemetry)" \
    "/sensors/core not detected — check VESC USB, and that servo output is enabled in VESC Tool"

if [ "$USE_JOY" = true ]; then
    start_joy
    sleep 2
    check_topic /joy 3 "/joy publishing (F710)" \
        "/joy not detected — check the receiver, the X/D switch, and $LOG_DIR/joy.log"
    run_joy_foreground
else
    run_teleop_foreground
fi
