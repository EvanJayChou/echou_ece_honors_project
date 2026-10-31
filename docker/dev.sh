#!/usr/bin/env bash
# dev.sh — manage the persistent development container (the `dev` service).
#
#   docker/dev.sh up            start it in the background (builds the image first if missing)
#   docker/dev.sh shell         bash inside it, with ROS 2 and the workspaces sourced
#   docker/dev.sh shell CMD...  run one command inside it instead
#   docker/dev.sh down          stop and remove it (the rover-home volume, with your logins, stays)
#   docker/dev.sh status        show whether it's running
#   docker/dev.sh reset-home    delete the rover-home volume (wipes GitHub/Claude logins, git config)
#
# Optional overrides (USER_UID, USER_GID, ROS_DOMAIN_ID) are read from docker/.env
# if it exists; see docker/.env.example.

set -euo pipefail

DOCKER_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$DOCKER_DIR/.." && pwd)"
CONTAINER=rover-dev
VOLUME=rover-home

COMPOSE=(docker compose -f "$DOCKER_DIR/compose.yaml")
[ -f "$DOCKER_DIR/.env" ] && COMPOSE+=(--env-file "$DOCKER_DIR/.env")

usage() { sed -n '2,12p' "$0" | sed 's/^# \{0,1\}//'; }

running() { [ "$(docker inspect -f '{{.State.Running}}' "$CONTAINER" 2>/dev/null)" = true ]; }

case "${1:-}" in
    up)
        # Create the bind-mount sources as you; Docker would create missing ones as root.
        mkdir -p /tmp/rover "$REPO_ROOT/maps/custom"
        "${COMPOSE[@]}" up -d dev
        echo "Running. Open a shell with: docker/dev.sh shell"
        ;;
    shell)
        shift
        if ! running; then
            echo "$CONTAINER isn't running. Start it with: docker/dev.sh up" >&2
            exit 1
        fi
        TTY=(-i)
        [ -t 0 ] && [ -t 1 ] && TTY=(-it)
        exec docker exec "${TTY[@]}" "$CONTAINER" /entrypoint.sh "${@:-bash}"
        ;;
    down)
        "${COMPOSE[@]}" rm --stop --force dev
        echo "Stopped. The $VOLUME volume (logins, home directory) is kept."
        ;;
    status)
        if running; then
            echo "$CONTAINER: running ($(docker inspect -f '{{.Config.Image}}, started {{.State.StartedAt}}' "$CONTAINER"))"
        elif docker inspect "$CONTAINER" >/dev/null 2>&1; then
            echo "$CONTAINER: stopped"
        else
            echo "$CONTAINER: not created"
        fi
        if docker volume inspect "$VOLUME" >/dev/null 2>&1; then
            echo "$VOLUME volume: present"
        else
            echo "$VOLUME volume: absent (created on next up)"
        fi
        ;;
    reset-home)
        if ! docker volume inspect "$VOLUME" >/dev/null 2>&1; then
            echo "No $VOLUME volume; nothing to delete."
            exit 0
        fi
        echo "This deletes the $VOLUME volume: /home/echou, including GitHub and Claude Code"
        echo "logins, git config and shell history. The checkout in $REPO_ROOT is not touched."
        read -r -p "Delete it? [y/N] " answer
        case "$answer" in
            [yY]|[yY][eE][sS]) ;;
            *) echo "Cancelled."; exit 0 ;;
        esac
        # The volume can't be removed while a container uses it.
        "${COMPOSE[@]}" rm --stop --force dev
        docker volume rm "$VOLUME"
        echo "Deleted. The next docker/dev.sh up starts with a fresh home directory."
        ;;
    -h|--help|help)
        usage
        ;;
    *)
        usage >&2
        exit 1
        ;;
esac
