#!/usr/bin/env bash

WORKSPACE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCRIPT_MODE="executed"

if [[ "${BASH_SOURCE[0]}" != "${0}" ]]; then
  SCRIPT_MODE="sourced"
fi

die() {
  echo "$1" >&2
  if [[ "$SCRIPT_MODE" == "sourced" ]]; then
    return 1
  fi
  exit 1
}

source_ros2_pkg() {
  if [[ -z "${ROS_DISTRO:-}" ]]; then
    die "ROS_DISTRO is not set. Source your ROS 2 environment first."
  fi

  source "/opt/ros/${ROS_DISTRO}/setup.bash" || die "Failed to source /opt/ros/${ROS_DISTRO}/setup.bash"
}

source_ros2() {
  source_ros2_pkg || return 1
  cd "$WORKSPACE_ROOT" || die "Failed to enter workspace root: $WORKSPACE_ROOT"
  source install/setup.bash || die "Failed to source $WORKSPACE_ROOT/install/setup.bash"
}

build_ros2() {
  source_ros2_pkg || return 1
  cd "$WORKSPACE_ROOT" || die "Failed to enter workspace root: $WORKSPACE_ROOT"
  colcon build --symlink-install || die "colcon build --symlink-install failed"
  source install/setup.bash || die "Failed to source $WORKSPACE_ROOT/install/setup.bash"
}

build_ros2_pkg() {
  source_ros2_pkg || return 1
  cd "$WORKSPACE_ROOT" || die "Failed to enter workspace root: $WORKSPACE_ROOT"
  colcon build --symlink-install --packages-select "$@" || die "colcon build --symlink-install --packages-select $* failed"
  source install/setup.bash || die "Failed to source $WORKSPACE_ROOT/install/setup.bash"
}

complete -W "robot_slam" build_ros2_pkg

main() {
  build_ros2 || return 1
}

main "$@"
