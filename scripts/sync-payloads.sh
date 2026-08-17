#!/bin/bash
# =============================================================================
# Fetch and build every payload listed in deployment/payload-registry.yaml.
#
#   ./scripts/sync-payloads.sh [--force]
#
# Payloads are cloned into src/payloads/<name>/ (git-ignored) and their Docker
# images are built ahead of time. Run this at the dock, while you still have
# bandwidth: bringing the stack up expects the images to exist, and a cellular link
# offshore is a poor place to discover a missing base image.
# =============================================================================
set -euo pipefail

BASE_DIR="$(cd "$(dirname "$0")/.." && pwd)"
PAYLOAD_DIR="$BASE_DIR/src/payloads"
REGISTRY_FILE="$BASE_DIR/deployment/payload-registry.yaml"

FORCE=false
while [[ $# -gt 0 ]]; do
    case "$1" in
        --force)
            # Discard local changes in payload checkouts and reset to the
            # registry version. Useful on a vehicle where someone debugged in
            # place and the working tree has drifted.
            FORCE=true
            ;;
        -h|--help)
            sed -n '2,12p' "$0" | sed 's/^# \?//'
            exit 0
            ;;
        *)
            echo "Unknown option: $1" >&2
            exit 1
            ;;
    esac
    shift
done

[[ -f /etc/nv_tegra_release ]] && PLATFORM="jetson" || PLATFORM="generic"
echo "Platform: $PLATFORM"

mkdir -p "$PAYLOAD_DIR"

PAYLOADS="$(python3 - "$REGISTRY_FILE" <<'PY'
import sys

import yaml

try:
    with open(sys.argv[1], 'r', encoding='utf-8') as handle:
        data = yaml.safe_load(handle) or {}
except OSError:
    sys.exit(0)

for payload in data.get('payloads') or []:
    if not payload.get('enabled', True):
        continue
    print(f"{payload['name']} {payload['repo']} {payload.get('version', 'main')}")
PY
)"

if [[ -z "$PAYLOADS" ]]; then
    echo "No enabled payloads in $REGISTRY_FILE — nothing to do."
    exit 0
fi

while read -r NAME REPO VERSION; do
    [[ -z "$NAME" ]] && continue
    TARGET="$PAYLOAD_DIR/$NAME"
    echo "---------------------------------------------------"
    echo "Payload: $NAME (version: $VERSION)"

    if [[ -d "$TARGET/.git" ]]; then
        git -C "$TARGET" fetch --all --quiet
    elif [[ -d "$TARGET" ]]; then
        echo "ERROR: $TARGET exists but is not a git repository." >&2
        exit 1
    else
        git clone "$REPO" "$TARGET"
    fi

    if $FORCE; then
        # Show what is about to be discarded rather than destroying work
        # silently — a payload checkout on a vehicle may hold the only copy
        # of a field fix.
        CHANGES="$(git -C "$TARGET" status --porcelain --untracked-files=no)"
        if [[ -n "$CHANGES" ]]; then
            echo "  Discarding local changes:"
            sed 's/^/    /' <<<"$CHANGES"
        fi
        git -C "$TARGET" checkout -f "$VERSION" --quiet
        git -C "$TARGET" reset --hard "origin/$VERSION" --quiet 2>/dev/null || true
    else
        git -C "$TARGET" checkout "$VERSION" --quiet
        git -C "$TARGET" pull --ff-only origin "$VERSION" 2>/dev/null || \
            echo "  (no fast-forward available; leaving checkout as is)"
    fi

    # Prefer a platform-specific compose file when the payload provides one:
    # GPU payloads need a CUDA base image that will not run on a Pi.
    COMPOSE_FILE=""
    if [[ "$PLATFORM" == "jetson" && -f "$TARGET/docker/docker-compose.payload.jetson.yml" ]]; then
        COMPOSE_FILE="$TARGET/docker/docker-compose.payload.jetson.yml"
    elif [[ -f "$TARGET/docker/docker-compose.payload.yml" ]]; then
        COMPOSE_FILE="$TARGET/docker/docker-compose.payload.yml"
    fi

    if [[ -n "$COMPOSE_FILE" ]]; then
        echo "  Building image ($(basename "$COMPOSE_FILE"))..."
        docker compose -f "$COMPOSE_FILE" -p "$NAME" build
    else
        echo "  ERROR: no docker/docker-compose.payload*.yml in $TARGET" >&2
    fi
done <<<"$PAYLOADS"

echo "---------------------------------------------------"
echo "Payload sync complete."
