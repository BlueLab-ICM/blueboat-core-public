#!/bin/bash
# Entrypoint for the `logger` service (docker-compose.yml). Builds the
# `ros2 bag record` argument list from env vars supplied by compose:
#   BAG_FORMAT          storage format ("mcap" | "sqlite3"), default mcap
#   BAG_EXCLUDE_REGEX   anchored alternation regex of topics to skip
#                       (--exclude-regex; ros2 bag record has no -x short flag),
#                       set in deployment/.env or the shell.
#                       Empty -> record every topic.
# Kept as a script (not an inline compose command) so the exclusion regex,
# which contains shell-special characters (^ $ |), is passed to ros2 as a
# single safely-quoted argument.
set -e
source /opt/ros/jazzy/setup.bash

ARGS=(-a -s "${BAG_FORMAT:-mcap}")
if [ -n "${BAG_EXCLUDE_REGEX}" ]; then
    ARGS+=(--exclude-regex "${BAG_EXCLUDE_REGEX}")
    echo "record-bag: excluding topics matching '${BAG_EXCLUDE_REGEX}'"
fi

exec ros2 bag record "${ARGS[@]}" \
    -o "/app/backseat_ws/logs/rosbag_$(date +%Y%m%d_%H%M%S)"
