#!/bin/bash
# ==============================================================================
# SOFTWARE UPDATE & BUILD (Requires Internet)
# Reads config/module-registry.yaml to download and compile all modules.
# ==============================================================================

set -euo pipefail

usage() {
    cat <<'EOF'
Usage: scripts/modules/sync-modules.sh [--force] [--hard-reset]

Options:
  --force       Reset all existing module repos to origin/main after showing
                which tracked files would be overwritten.
  --hard-reset  Do everything in --force and also delete all untracked/ignored
                files (git clean -fdx) after showing what would be deleted.
  -h, --help    Show this help.
EOF
}

FORCE=false
HARD_RESET=false

while [[ $# -gt 0 ]]; do
    case "$1" in
        --force)
            FORCE=true
            ;;
        --hard-reset)
            FORCE=true
            HARD_RESET=true
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "Unknown option: $1" >&2
            usage
            exit 1
            ;;
    esac
    shift
done

source "$(dirname "${BASH_SOURCE[0]}")/../lib/common.sh"
MODULE_DIR="$CORE_ROOT/src/modules"
REGISTRY_FILE="$CORE_ROOT/config/module-registry.yaml"

confirm_or_abort() {
    local prompt="$1"
    local reply
    read -r -p "$prompt [y/N]: " reply
    if [[ ! "$reply" =~ ^[Yy]$ ]]; then
        echo "Aborted."
        exit 1
    fi
}

require_origin_main() {
    local repo_path="$1"
    if ! git -C "$repo_path" rev-parse --verify --quiet origin/main >/dev/null; then
        echo "ERROR: '$repo_path' does not have origin/main. Cannot continue with --force/--hard-reset." >&2
        exit 1
    fi
}

if [[ "$BACKSEAT_PLATFORM" == "jetson" ]]; then
    echo "Jetson detected — GPU modules will build from their Jetson-specific Dockerfiles."
fi

echo "Updating Software and Building Images..."
mkdir -p "$MODULE_DIR"

# 1. Parse the Module Registry using Python
PAYLOADS=$(python3 -c "
import yaml, sys
try:
    with open('$REGISTRY_FILE') as f:
        data = yaml.safe_load(f)
        for p in data.get("modules", []):
            print(f\"{p['name']} {p['repo']} {p['version']}\")
except Exception as e:
    sys.exit(1)
")

if [ -z "$PAYLOADS" ]; then
    echo "No modules found in registry: $REGISTRY_FILE"
    exit 0
fi

if [ "$FORCE" = true ]; then
    echo "---------------------------------------------------"
    echo "[Preview] Tracked files that would be overwritten by reset to origin/main:"
    any_overwrite=false

    while read -r NAME REPO VERSION; do
        if [ -z "$NAME" ]; then continue; fi
        TARGET_PATH="$MODULE_DIR/$NAME"
        if [ -d "$TARGET_PATH/.git" ]; then
            changes=$(git -C "$TARGET_PATH" status --porcelain --untracked-files=no || true)
            if [ -n "$changes" ]; then
                any_overwrite=true
                while IFS= read -r line; do
                    [ -z "$line" ] && continue
                    echo "  - $NAME/${line:3}"
                done <<< "$changes"
            fi
        fi
    done <<< "$PAYLOADS"

    if [ "$any_overwrite" = false ]; then
        echo "  (No tracked changes detected.)"
    fi

    confirm_or_abort "Proceed with force reset of all module repos to origin/main?"
fi

if [ "$HARD_RESET" = true ]; then
    echo "---------------------------------------------------"
    echo "[Preview] Untracked/ignored files that would be deleted (git clean -fdx):"
    any_delete=false

    while read -r NAME REPO VERSION; do
        if [ -z "$NAME" ]; then continue; fi
        TARGET_PATH="$MODULE_DIR/$NAME"
        if [ -d "$TARGET_PATH/.git" ]; then
            deletions=$(git -C "$TARGET_PATH" clean -ndx || true)
            if [ -n "$deletions" ]; then
                any_delete=true
                while IFS= read -r line; do
                    [ -z "$line" ] && continue
                    echo "  - $NAME/${line#Would remove }"
                done <<< "$deletions"
            fi
        fi
    done <<< "$PAYLOADS"

    if [ "$any_delete" = false ]; then
        echo "  (No untracked or ignored files detected.)"
    fi

    confirm_or_abort "Proceed with hard reset cleanup (delete listed files) in all module repos?"
fi

# 2. Pull Code and Compile
while read -r NAME REPO VERSION; do
    if [ -z "$NAME" ]; then continue; fi
    TARGET_PATH="$MODULE_DIR/$NAME"

    echo "---------------------------------------------------"
    if [ "$FORCE" = true ]; then
        echo "Updating Module: $NAME (FORCED to origin/main)"
    else
        echo "Updating Module: $NAME (Target: $VERSION)"
    fi

    if [ -d "$TARGET_PATH/.git" ]; then
        git -C "$TARGET_PATH" fetch --all
    elif [ -d "$TARGET_PATH" ]; then
        echo "ERROR: $TARGET_PATH exists but is not a git repository." >&2
        exit 1
    else
        git clone "$REPO" "$TARGET_PATH"
    fi

    if [ "$FORCE" = true ]; then
        git -C "$TARGET_PATH" fetch origin main
        require_origin_main "$TARGET_PATH"
        git -C "$TARGET_PATH" checkout -f -B main origin/main
        if [ "$HARD_RESET" = true ]; then
            git -C "$TARGET_PATH" clean -fdx
        fi
    else
        # Enforce version and pull
        git -C "$TARGET_PATH" checkout "$VERSION"
        git -C "$TARGET_PATH" pull origin "$VERSION" 2>/dev/null || true
    fi

    # Pre-compile the Docker Image.
    # On Jetson, prefer the GPU/TRT-specific compose file if the module provides one.
    COMPOSE_FILE=""
    if [[ "$BACKSEAT_PLATFORM" == "jetson" && -f "$TARGET_PATH/docker/docker-compose.module.jetson.yml" ]]; then
        COMPOSE_FILE="$TARGET_PATH/docker/docker-compose.module.jetson.yml"
    elif [[ -f "$TARGET_PATH/docker/docker-compose.module.yml" ]]; then
        COMPOSE_FILE="$TARGET_PATH/docker/docker-compose.module.yml"
    fi

    if [[ -n "$COMPOSE_FILE" ]]; then
        echo "Compiling Docker Image for $NAME (compose: $(basename "$COMPOSE_FILE"))..."
        docker compose -f "$COMPOSE_FILE" -p "$NAME" build
    else
        echo "ERROR: No docker-compose found in $TARGET_PATH/docker/"
    fi
done <<< "$PAYLOADS"

echo "---------------------------------------------------"
echo "SOFTWARE UPDATE COMPLETE."
