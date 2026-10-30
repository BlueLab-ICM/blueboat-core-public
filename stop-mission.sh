#!/bin/bash

source "$(dirname "${BASH_SOURCE[0]}")/scripts/lib/common.sh"
CORE_COMPOSE="$CORE_ROOT/docker/docker-compose.yml"
MODULE_DIR="$CORE_ROOT/src/modules"

eval "$(python3 "$CORE_ROOT/scripts/lib/mission_config.py" exports)"
MISSION_DIR="$(current_mission_dir)" || MISSION_DIR=""
export APPLICATION_DIR="$MODULE_DIR/$APPLICATION"

run_hook() {
    local hook="$APPLICATION_DIR/hooks/$1.sh"
    [[ -n "$APPLICATION" && -x "$hook" && -n "$MISSION_DIR" ]] || return 0
    CORE_COMPOSE="$CORE_COMPOSE" "$hook" "$MISSION_DIR"
}

# First, so the bag holds the shutdown of everything else and closes properly.
if container_running core_logger; then
    echo "Closing the recording..."
    docker stop --time 30 core_logger > /dev/null || true
fi

# Without journald, logs live in the containers and must be read before they are removed.
if [[ -n "$MISSION_DIR" && "$DOCKER_LOG_DRIVER" != "journald" ]]; then
    "$CORE_ROOT/scripts/data/collect-logs.sh" "$MISSION_DIR"
fi

# A persistent module, such as the LoRaWAN tracker, keeps running after the mission.
python3 "$CORE_ROOT/scripts/lib/mission_config.py" modules | while read -r NAME _DATA _SHARED PERSISTENT; do
    [[ "$PERSISTENT" == "yes" ]] && echo "Leaving $NAME running" && continue
    echo "Stopping module: $NAME"
    docker compose -p "$NAME" down --timeout 5
done

run_hook pre-stop

if [[ -n "$MISSION_DIR" && -f "$MISSION_DIR/system/.motion_pid" ]]; then
    MOTION_PID="$(cat "$MISSION_DIR/system/.motion_pid")"
    echo "Stopping motion logger (pid=$MOTION_PID)..."
    kill "$MOTION_PID" 2>/dev/null || true
    sleep 1
fi

echo "Stopping core..."
# --remove-orphans also takes down the SITL container of a --sim mission.
docker compose -f "$CORE_COMPOSE" --profile record down --timeout 5 --remove-orphans

if [[ -n "$MISSION_DIR" && -d "$MISSION_DIR" ]]; then
    [[ "$DOCKER_LOG_DRIVER" == "journald" ]] && "$CORE_ROOT/scripts/data/collect-logs.sh" "$MISSION_DIR"
    python3 "$CORE_ROOT/scripts/data/mission-manifest.py" end "$MISSION_DIR" || true
    python3 "$CORE_ROOT/scripts/data/mission-verify.py" "$MISSION_DIR" || true
    # Drop any mission subfolder nothing wrote to.
    find "$MISSION_DIR" -mindepth 1 -type d -empty -delete
    # File the mission under lab/ or water/ by how much the boat actually moved.
    python3 "$CORE_ROOT/scripts/data/mission-venue.py" file "$MISSION_DIR" || \
        echo "WARNING: could not file mission, left at $MISSION_DIR"
fi

docker ps
