#!/bin/bash
# Save a mission's container logs into <mission>/system/. Reads journald, which keeps them after
# the containers are gone; with another log driver it reads the containers that still exist.
#
# Usage: collect-logs.sh <mission_dir>
set -uo pipefail

MISSION_DIR="$1"
mkdir -p "$MISSION_DIR/system"
MISSION_ID="$(python3 -c "
import sys, yaml
print((yaml.safe_load(open(sys.argv[1])) or {}).get('mission_id', ''))
" "$MISSION_DIR/manifest.yaml" 2>/dev/null || true)"

if [[ -n "$MISSION_ID" ]] && command -v journalctl >/dev/null; then
    NAMES="$(journalctl --no-pager -o json --output-fields=CONTAINER_NAME \
        MISSION_ID="$MISSION_ID" 2>/dev/null | python3 -c "
import json, sys
print(' '.join(sorted({json.loads(line).get('CONTAINER_NAME', '') for line in sys.stdin})))
" || true)"
    if [[ -n "$NAMES" ]]; then
        for C in $NAMES; do
            journalctl --no-pager -o short-iso-precise \
                MISSION_ID="$MISSION_ID" CONTAINER_NAME="$C" > "$MISSION_DIR/system/$C.log" || true
        done
        exit 0
    fi
fi

echo "No journald logs for this mission; reading containers." >&2
for C in $(docker ps -a --filter "label=com.docker.compose.project" --format '{{.Names}}'); do
    docker logs "$C" &> "$MISSION_DIR/system/$C.log" || true
done
