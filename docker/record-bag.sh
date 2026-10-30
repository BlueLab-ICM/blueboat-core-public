#!/bin/bash
# Entrypoint of the logger service. Records from mission start to mission end, in files of
# BAG_SPLIT_S seconds, so a power cut costs one file at most.
#   BAG_FORMAT          storage format, default mcap
#   BAG_EXCLUDE_REGEX   topics to skip, built from recording.exclude_topics
#   BAG_SPLIT_S         seconds per file, default 300
set -e
source /opt/ros/jazzy/setup.bash
# Custom message types come from the workspace; without it rosbag2 drops those topics.
source /app/backseat_ws/install/setup.bash

ARGS=(-a -s "${BAG_FORMAT:-mcap}" --max-bag-duration "${BAG_SPLIT_S:-300}")
if [ -n "${BAG_EXCLUDE_REGEX}" ]; then
    ARGS+=(--exclude-regex "${BAG_EXCLUDE_REGEX}")
    echo "record-bag: excluding topics matching '${BAG_EXCLUDE_REGEX}'"
fi

exec ros2 bag record "${ARGS[@]}" -o "/app/backseat_ws/logs/rosbag_$(date +%Y%m%d_%H%M%S)"
