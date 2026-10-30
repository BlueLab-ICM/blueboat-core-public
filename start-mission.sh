#!/bin/bash
set -e

source "$(dirname "${BASH_SOURCE[0]}")/scripts/lib/common.sh"
CORE_COMPOSE="$CORE_ROOT/docker/docker-compose.yml"
MISSION_COMPOSE="$CORE_ROOT/docker/docker-compose.mission.yml"
SITL_COMPOSE="$CORE_ROOT/docker/docker-compose.sitl.yml"
MODULE_DIR="$CORE_ROOT/src/modules"

# --sim: simulated frontseat (SITL). Filed under sim/, never classified or uploaded by default.
SIM=false
case "${1:-}" in
    --sim) SIM=true ;;
    "") ;;
    *) echo "Usage: $0 [--sim]" >&2; exit 2 ;;
esac

if ! $SIM && [[ "$VEHICLE_NAME" == "unnamed" ]]; then
    echo "ERROR: VEHICLE_NAME is not set. Add it to config/vehicle.local.env:" >&2
    echo '  : "${VEHICLE_NAME:=<name>}"' >&2
    exit 1
fi

if container_running core_helm; then
    echo "ERROR: a mission is already running. Stop it with stop-mission.sh." >&2
    exit 1
fi

# A mission that never reached stop-mission.sh still has an open folder.
if PREVIOUS="$(current_mission_dir)" && ! grep -q '^ended_at:' "$PREVIOUS/manifest.yaml" 2>/dev/null; then
    "$CORE_ROOT/scripts/data/recover-mission.sh" "$PREVIOUS" || true
fi

eval "$(python3 "$CORE_ROOT/scripts/lib/mission_config.py" exports)"

# Compose file for this board: the Jetson variant when there is one.
module_compose_file() {
    local file=""
    [[ "$BACKSEAT_PLATFORM" == "jetson" ]] && file=$(find -L "$MODULE_DIR/$1" -name "docker-compose.module.jetson.yml" 2>/dev/null | head -1)
    [[ -z "$file" ]] && file=$(find -L "$MODULE_DIR/$1" -name "docker-compose.module.yml" 2>/dev/null | head -1)
    echo "$file"
}

# A module service without ROS_DOMAIN_ID from the environment runs on domain 0, unseen by core.
for NAME in $MODULES; do
    COMPOSE_FILE="$(module_compose_file "$NAME")"
    [[ -f "$COMPOSE_FILE" ]] || continue
    if ! docker compose -f "$COMPOSE_FILE" config --no-interpolate --format json | python3 -c "
import json, sys
def domain(service):
    env = service.get('environment') or {}
    if isinstance(env, list):
        env = dict(item.partition('=')[::2] for item in env)
    return str(env.get('ROS_DOMAIN_ID') or '')
services = json.load(sys.stdin)['services'].values()
sys.exit(any(not domain(s).startswith('\${ROS_DOMAIN_ID') for s in services))
"; then
        echo "ERROR: module $NAME does not take ROS_DOMAIN_ID from the environment ($COMPOSE_FILE)." >&2
        echo "Add to the environment of each of its services:" >&2
        echo '  - ROS_DOMAIN_ID=${ROS_DOMAIN_ID:?source config/vehicle.env first}' >&2
        exit 1
    fi
done

MISSION_ID="$(python3 -c 'import uuid; print(uuid.uuid4().hex[:12])')"
MISSION_TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
MISSION_DIR="$DEPLOY_DATA_ROOT/${MISSION_TIMESTAMP}_${MISSION_NAME}"
mkdir -p "$MISSION_DIR"/{rosbags,system,control,experiment_logs}
echo "$MISSION_DIR" > "$DEPLOY_DATA_ROOT/.current_mission"

export MISSION_ID MISSION_DIR BAG_FORMAT BAG_SPLIT_S BAG_EXCLUDE_REGEX
export MISSION_ROSBAG_DIR="$MISSION_DIR/rosbags"
export MISSION_EXPERIMENT_LOG_DIR="$MISSION_DIR/experiment_logs"
export MISSION_SYSTEM_DIR="$MISSION_DIR/system"
export MISSION_CONTROL_DIR="$MISSION_DIR/control"

SIM_FLAG=()
$SIM && SIM_FLAG=(--sim)
python3 "$CORE_ROOT/scripts/data/mission-manifest.py" start "$MISSION_DIR" \
    --core-root "$CORE_ROOT" --mission-id "$MISSION_ID" --name "$MISSION_NAME" \
    --platform "$BACKSEAT_PLATFORM" --vehicle "$VEHICLE_NAME" \
    --application "$APPLICATION" --modules "$MODULES" \
    --recording "$RECORDING_ENABLED" --bag-format "$BAG_FORMAT" "${SIM_FLAG[@]}"

echo "Platform: $BACKSEAT_PLATFORM | Recording: $RECORDING_ENABLED | Simulation: $SIM"
echo "Mission $MISSION_ID: $MISSION_DIR"

COMPOSE_FILES=(-f "$CORE_COMPOSE" -f "$MISSION_COMPOSE")
if $SIM; then
    COMPOSE_FILES+=(-f "$SITL_COMPOSE")
    export SIM_MODE=true
fi
PROFILES=()
[[ "$RECORDING_ENABLED" == "TRUE" ]] && PROFILES=(--profile record)

# Applications build on the core image, so that comes first.
docker compose -f "$CORE_COMPOSE" build
if [[ -n "$APPLICATION" ]]; then
    export APPLICATION_DIR="$MODULE_DIR/$APPLICATION"
    APPLICATION_COMPOSE="$APPLICATION_DIR/docker/docker-compose.application.yml"
    [[ -f "$APPLICATION_COMPOSE" ]] && COMPOSE_FILES+=(-f "$APPLICATION_COMPOSE")
fi
run_hook() {
    local hook="${APPLICATION_DIR:-}/hooks/$1.sh"
    [[ -n "${APPLICATION_DIR:-}" && -x "$hook" ]] || return 0
    SIM="$SIM" CORE_COMPOSE="$CORE_COMPOSE" "$hook" "$MISSION_DIR"
}
run_hook pre-start
docker compose "${COMPOSE_FILES[@]}" "${PROFILES[@]}" up -d --build

python3 "$CORE_ROOT/scripts/lib/mission_config.py" modules | while read -r NAME DATA_DIR SHARED_DIR _PERSISTENT; do
    COMPOSE_FILE="$(module_compose_file "$NAME")"
    if [[ ! -f "$COMPOSE_FILE" ]]; then
        echo "WARNING: No compose file for module $NAME"
        continue
    fi
    MODULE_DATA_DIR=""
    MODULE_SHARED_DIR=""
    if [[ "$DATA_DIR" != "-" ]]; then
        MODULE_DATA_DIR="$MISSION_DIR/$DATA_DIR"
        mkdir -p "$MODULE_DATA_DIR"
    fi
    if [[ "$SHARED_DIR" != "-" ]]; then
        MODULE_SHARED_DIR="$DEPLOY_DATA_ROOT/$SHARED_DIR"
        mkdir -p "$MODULE_SHARED_DIR"
    fi
    echo "Starting module: $NAME"
    MODULE_DATA_DIR="$MODULE_DATA_DIR" MODULE_SHARED_DIR="$MODULE_SHARED_DIR" \
        docker compose -f "$COMPOSE_FILE" -p "$NAME" up -d --build
done

run_hook post-start
python3 "$CORE_ROOT/scripts/data/mission-manifest.py" images "$MISSION_DIR"

if ! $SIM; then
    echo "Starting motion logger (venue classification)..."
    nohup python3 "$CORE_ROOT/scripts/data/mission-venue.py" log "$MISSION_DIR" \
        >> "$MISSION_DIR/system/motion-logger.log" 2>&1 &
    echo "$!" > "$MISSION_DIR/system/.motion_pid"
fi

docker ps --format "table {{.Names}}\t{{.Status}}\t{{.Image}}"
