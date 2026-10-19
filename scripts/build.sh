#!/usr/bin/env bash
# build.sh — build the onboard (rover) or off-board (GPU machine) workspace.
#
# Usage:
#   scripts/build.sh onboard          packages the rover stack needs (default)
#   scripts/build.sh onboard --all    every package in ros2_ws/src
#   scripts/build.sh offboard         ros2_ws_offboard
#   scripts/build.sh onboard --clean  wipe build/ install/ log/ first
#   scripts/build.sh onboard --no-symlink   copy files into install/ (Docker image builds;
#                                           lets build/ be deleted afterwards)
#
# Onboard prerequisites: Livox-SDK2 installed (docs/reproduction.md §6) and the
# livox_ros_driver2 submodule initialised.

_TAG=build
source "$(dirname "$0")/common.sh"
trap - EXIT INT TERM   # nothing to clean up

TARGET="${1:-onboard}"
shift || true
ALL=false
CLEAN=false
INSTALL_MODE=(--symlink-install)
for arg in "$@"; do
    case "$arg" in
        --all)        ALL=true ;;
        --clean)      CLEAN=true ;;
        --no-symlink) INSTALL_MODE=() ;;
        *) err "Unknown flag: $arg  (valid: --all, --clean, --no-symlink)"; exit 1 ;;
    esac
done

# Packages the scripts in scripts/ launch (plus their dependencies).
ONBOARD_PACKAGES=(
    robot_slam
    livox_bringup
    livox_ros_driver2
    oakd_publisher
    keyframe_bridge
    ucsd_robocar_control2_pkg
    vesc_driver
    vesc_ackermann
)

if [ ! -f "/opt/ros/$ROS_DISTRO_NAME/setup.bash" ]; then
    err "/opt/ros/$ROS_DISTRO_NAME/setup.bash not found"
    exit 1
fi
set +u
source "/opt/ros/$ROS_DISTRO_NAME/setup.bash"

case "$TARGET" in
    onboard)
        WS="$ONBOARD_WS"
        LIVOX="$WS/src/livox_ros_driver2"
        if [ ! -f "$LIVOX/package_ROS2.xml" ]; then
            err "livox_ros_driver2 submodule is empty — run: git submodule update --init ros2_ws/src/livox_ros_driver2"
            exit 1
        fi
        # The driver ships ROS1/ROS2 manifests side by side and expects its own
        # build.sh to copy one into place; that script also wipes the whole
        # workspace's build/install, so do just the copy here instead.
        if ! cmp -s "$LIVOX/package_ROS2.xml" "$LIVOX/package.xml"; then
            cp "$LIVOX/package_ROS2.xml" "$LIVOX/package.xml"
            log "Installed livox_ros_driver2 package.xml (ROS 2 manifest)"
        fi
        CMAKE_ARGS=(--cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS="$ROS_DISTRO_NAME")
        if [ "$ALL" = true ]; then
            SELECT=()
        else
            SELECT=(--packages-up-to "${ONBOARD_PACKAGES[@]}")
        fi
        ;;
    offboard)
        WS="$OFFBOARD_WS"
        CMAKE_ARGS=()
        SELECT=()
        ;;
    *)
        err "Usage: scripts/build.sh {onboard|offboard} [--all] [--clean]"
        exit 1
        ;;
esac

cd "$WS" || exit 1
if [ "$CLEAN" = true ]; then
    log "Removing $WS/{build,install,log}"
    rm -rf build install log
fi

log "Building $(basename "$WS")..."
colcon build "${INSTALL_MODE[@]}" "${SELECT[@]}" "${CMAKE_ARGS[@]}" || { err "colcon build failed"; exit 1; }
ok "Build complete — source $WS/install/setup.bash"
