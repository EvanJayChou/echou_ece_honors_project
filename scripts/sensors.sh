#!/usr/bin/env bash
# sensors.sh — bring up only the sensors (Livox Mid-360 + OAK-D Pro) for bench checks.
#
#   livox_bringup  → /livox/lidar (PointCloud2), /scan (LaserScan)
#   oakd_publisher → /oakd/color/image_raw, /oakd/depth/image_raw, /oakd/imu
#
# Usage:
#   scripts/sensors.sh             both sensors, Ctrl+C to stop
#   scripts/sensors.sh --lidar     LiDAR only
#   scripts/sensors.sh --oakd      OAK-D only

_TAG=sensors
source "$(dirname "$0")/common.sh"

USE_LIDAR=true
USE_OAKD=true
for arg in "$@"; do
    case "$arg" in
        --lidar) USE_OAKD=false ;;
        --oakd)  USE_LIDAR=false ;;
        *) err "Unknown flag: $arg  (valid: --lidar, --oakd)"; exit 1 ;;
    esac
done

log "Preflight..."
[ "$USE_LIDAR" = true ] && check_lidar
[ "$USE_OAKD" = true ] && check_oakd
source_ws "$ONBOARD_WS"

[ "$USE_LIDAR" = true ] && start_lidar
[ "$USE_OAKD" = true ] && start_oakd
sleep 8

if [ "$USE_LIDAR" = true ]; then
    check_topic /livox/lidar 5 "/livox/lidar publishing" "/livox/lidar not detected — see $LOG_DIR/lidar.log"
    check_topic /scan 5 "/scan publishing" "/scan not detected — see $LOG_DIR/lidar.log"
fi
if [ "$USE_OAKD" = true ]; then
    check_topic /oakd/color/image_raw 5 "/oakd/color/image_raw publishing" "no OAK-D color — see $LOG_DIR/oakd.log"
    check_topic /oakd/imu 5 "/oakd/imu publishing" "no /oakd/imu — calibration may still be running; see $LOG_DIR/oakd.log"
fi

log "Sensors running. Ctrl+C to stop."
wait
