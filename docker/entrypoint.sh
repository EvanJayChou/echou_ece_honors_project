#!/usr/bin/env bash
# Container entrypoint: source ROS 2 and whichever workspaces are built, then run
# the command (default: bash). Runs as root (rover service) or as the non-root
# echou user (dev service); it only reads files.
#
# ROVER_WORKSPACE is the checkout to use: /opt/rover (baked into the image) by
# default, /opt/echou-honors-dev (the host checkout) in the dev service. Each
# workspace's install/ there wins; until you build it, the image's copy under
# /opt/rover is used instead. Missing workspaces are skipped.
#
# /etc/bash.bashrc also sources this file, for interactive shells that don't
# start through the entrypoint; it stops before the cd/exec in that case.
set +u
ROVER_WORKSPACE="${ROVER_WORKSPACE:-/opt/rover}"
source "/opt/ros/${ROS_DISTRO}/setup.bash"
_rover_source_ws() {  # <workspace> <setup file>
    if [ -f "$ROVER_WORKSPACE/$1/install/$2" ]; then
        source "$ROVER_WORKSPACE/$1/install/$2"
    elif [ -f "/opt/rover/$1/install/$2" ]; then
        source "/opt/rover/$1/install/$2"
        echo "[entrypoint] $ROVER_WORKSPACE/$1 isn't built; using the image's copy (scripts/build.sh builds yours)" >&2
    fi
}
_rover_source_ws ros2_ws setup.bash
_rover_source_ws ros2_ws_offboard local_setup.bash
unset -f _rover_source_ws
export ROVER_ENV_SOURCED=1
[ "${BASH_SOURCE[0]}" != "$0" ] && return
cd "$ROVER_WORKSPACE"
exec "$@"
