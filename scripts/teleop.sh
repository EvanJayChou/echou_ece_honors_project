#!/usr/bin/env bash
# teleop.sh — drive the rover with the keyboard. VESC only, no sensors.
#
#   keyboard_teleop_node → /ackermann_cmd → ackermann_to_vesc → vesc_driver → VESC
#   vesc_to_odom → /vesc_odom (wheel odometry, used by the EKF in the other scripts)
#
# Usage:
#   scripts/teleop.sh              keyboard teleop in this terminal
#   VESC_PORT=/dev/ttyACM1 scripts/teleop.sh

_TAG=teleop
source "$(dirname "$0")/common.sh"

log "Preflight..."
resolve_vesc_port
source_ws "$ONBOARD_WS"

start_vesc
sleep 3
check_topic /sensors/core 5 "/sensors/core publishing (VESC telemetry)" \
    "/sensors/core not detected — check VESC USB, and that servo output is enabled in VESC Tool"

run_teleop_foreground
