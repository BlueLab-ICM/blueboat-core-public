#!/bin/bash
# Finish a mission that never reached stop-mission.sh (crash or power cut): save its logs, close
# its bags, stamp its end, check it and file it.
#
# Usage: recover-mission.sh <mission_dir>
set -uo pipefail

source "$(dirname "${BASH_SOURCE[0]}")/../lib/common.sh"
MISSION_DIR="$1"

echo "Recovering mission that did not stop cleanly: $MISSION_DIR"
"$CORE_ROOT/scripts/data/collect-logs.sh" "$MISSION_DIR"

for BAG in "$MISSION_DIR"/rosbags/*/; do
    [[ -d "$BAG" && ! -f "$BAG/metadata.yaml" ]] || continue
    echo "Reindexing $BAG"
    docker run --rm -u "$(id -u):$(id -g)" -v "$BAG:/bag" blueboat-core bash -c \
        'source /opt/ros/jazzy/setup.bash && ros2 bag reindex /bag -s mcap' || \
        echo "WARNING: could not reindex $BAG"
done

python3 "$CORE_ROOT/scripts/data/mission-manifest.py" end "$MISSION_DIR" --recovered
python3 "$CORE_ROOT/scripts/data/mission-verify.py" "$MISSION_DIR" || true
python3 "$CORE_ROOT/scripts/data/mission-venue.py" file "$MISSION_DIR" || \
    echo "WARNING: could not file mission, left at $MISSION_DIR"
