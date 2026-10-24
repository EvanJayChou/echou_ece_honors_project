#!/usr/bin/env bash
# run.sh — start the rover container without Docker Compose.
#
#   docker/run.sh                          shell in the container
#   docker/run.sh scripts/teleop_slam.sh   run a launch script
#   IMAGE=rover:jazzy ROS_DOMAIN_ID=5 docker/run.sh scripts/explore.sh --all-onboard
#
# Build the image first, from the repo root:
#   docker build -f docker/Dockerfile -t rover:jazzy .
#
# Uses the NVIDIA runtime for GPU access; DOCKER_RUNTIME=runc runs without it.

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
IMAGE="${IMAGE:-rover:jazzy}"
mkdir -p /tmp/rover "$REPO_ROOT/maps/custom"

exec docker run --rm -it --init \
    --runtime "${DOCKER_RUNTIME:-nvidia}" \
    --network host --ipc host --privileged \
    -v /dev:/dev \
    -v /tmp/rover:/tmp/rover \
    -v "$REPO_ROOT/maps:/opt/rover/ros2_ws/src/robot_slam/ros_data/maps" \
    -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
    ${VESC_PORT:+-e VESC_PORT="$VESC_PORT"} \
    ${LIVOX_IP:+-e LIVOX_IP="$LIVOX_IP"} \
    "$IMAGE" "$@"
