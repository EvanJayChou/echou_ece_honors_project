#!/usr/bin/env bash
# record.sh — record raw sensor topics to an MCAP bag for offline GLIM mapping.
#
# Start the sensors first (scripts/sensors.sh, or any teleop/explore script),
# then run this in a second terminal. Ctrl+C stops and closes the bag.
#
#   /livox/lidar, /livox/imu          Mid-360 points (xfer_format 0, per-point time) + IMU
#   /oakd/color/image_raw, depth      OAK-D Pro RGB-D (~70 MB/s raw at 1280x720 @ 15 fps)
#   /oakd/*/camera_info               recorded if a publisher appears (oakd_publisher has none yet)
#   /oakd/imu, /vesc_odom             secondary IMU and wheel odometry
#   /tf, /tf_static                   static TF is latched, so start the stack before this
#
# Usage:
#   scripts/record.sh                 MCAP with zstd_fast chunk compression
#   scripts/record.sh --no-compress   plain MCAP (if the compressor can't keep up — see docs/glim.md)
#   scripts/record.sh --no-images     LiDAR/IMU/TF only (small; enough for GLIM)
#
# Bags go to $BAG_DIR (default: <repo>/bags), named rover_<date>_<time>.

_TAG=record
source "$(dirname "$0")/common.sh"

BAG_DIR="${BAG_DIR:-$REPO_ROOT/bags}"
PRESET=zstd_fast
USE_IMAGES=true
for arg in "$@"; do
    case "$arg" in
        --no-compress) PRESET=none ;;
        --no-images)   USE_IMAGES=false ;;
        *) err "Unknown flag: $arg  (valid: --no-compress, --no-images)"; exit 1 ;;
    esac
done

TOPICS=(/livox/lidar /livox/imu /oakd/imu /vesc_odom /tf /tf_static)
if [ "$USE_IMAGES" = true ]; then
    TOPICS+=(/oakd/color/image_raw /oakd/depth/image_raw /oakd/color/camera_info /oakd/depth/camera_info)
fi

source_ws "$ONBOARD_WS"
check_topic /livox/lidar 5 "/livox/lidar publishing" "/livox/lidar not detected — start the sensors first (scripts/sensors.sh)"
check_topic /livox/imu 5 "/livox/imu publishing" "/livox/imu not detected"

mkdir -p "$BAG_DIR"
OUT="$BAG_DIR/rover_$(date +%Y%m%d_%H%M%S)"
log "Recording ${#TOPICS[@]} topics to $OUT  (storage preset: $PRESET)"
log "Ctrl+C to stop. Afterwards: ros2 bag info $OUT"
# Foreground, not start_bg: ros2 bag record gets Ctrl+C directly and closes the
# MCAP cleanly; common.sh's 2 s SIGKILL fallback could truncate a large bag.
ros2 bag record -s mcap --storage-preset-profile "$PRESET" -o "$OUT" --topics "${TOPICS[@]}"
