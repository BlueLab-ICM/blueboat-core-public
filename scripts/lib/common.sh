# Shared setup for blueboat-core scripts: sets CORE_ROOT and loads config/vehicle.env.
# Source it, do not run it. Callers keep their own set -e/-u choices.
# shellcheck shell=bash

CORE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
# shellcheck source=../../config/vehicle.env
source "$CORE_ROOT/config/vehicle.env"

# Print the active mission directory; return 1 when there is none or it no longer exists.
current_mission_dir() {
    local file="$DEPLOY_DATA_ROOT/.current_mission" dir
    [[ -f "$file" ]] || return 1
    dir="$(cat "$file")"
    [[ -n "$dir" && -d "$dir" ]] || return 1
    printf '%s\n' "$dir"
}

# Return 0 if the container exists and is running.
container_running() {
    [[ "$(docker inspect --format '{{.State.Running}}' "$1" 2>/dev/null)" == "true" ]]
}
