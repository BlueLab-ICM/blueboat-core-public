#!/bin/bash

# ==============================================================================
# ICM BLUEBOAT: AUTOMATED PAYLOAD REMOVAL TOOL
# ==============================================================================

if [ -z "$1" ]; then
    echo "ERROR: Please provide the module name to remove."
    echo "Usage: scripts/modules/unregister-module.sh <module_repo_name>"
    exit 1
fi

MODULE_NAME=$1
KEY_FILE="$HOME/.ssh/blueboat_${MODULE_NAME}"
SSH_CONFIG="$HOME/.ssh/config"
source "$(dirname "${BASH_SOURCE[0]}")/../lib/common.sh"
REGISTRY_FILE="$CORE_ROOT/config/module-registry.yaml"
MODULE_DIR="$CORE_ROOT/src/modules/$MODULE_NAME"

echo "Initiating cleanup for: $MODULE_NAME"

# 1. DELETE SSH KEYS
if [ -f "$KEY_FILE" ]; then
    echo "Removing SSH keys..."
    rm -f "$KEY_FILE" "${KEY_FILE}.pub"
else
    echo "WARNING: No SSH key found for $MODULE_NAME."
fi

# 2. CLEAN UP SSH CONFIG
if grep -q "Host github-${MODULE_NAME}" "$SSH_CONFIG" 2>/dev/null; then
    echo "Scrubbing routing from ~/.ssh/config..."
    # Delete from the auto-generated comment down to the 'IdentitiesOnly' line
    sed -i "/# --- AUTO-GENERATED: $MODULE_NAME ---/,/IdentitiesOnly yes/d" "$SSH_CONFIG"
    # Remove any stray blank lines left behind
    sed -i '/^[[:space:]]*$/N;/^[[:space:]]*\n[[:space:]]*$/D' "$SSH_CONFIG"
else
    echo "WARNING: No SSH routing found in config."
fi

# 3. REMOVE FROM PAYLOAD REGISTRY
if [ -f "$REGISTRY_FILE" ] && grep -q "name: $MODULE_NAME" "$REGISTRY_FILE"; then
    echo "Removing from module-registry.yaml..."
    # An entry runs from its "- name:" line to the next entry or the end of the file.
    awk -v name="$MODULE_NAME" '/^  - name: / { skip = ($3 == name) } !skip' \
        "$REGISTRY_FILE" > "$REGISTRY_FILE.tmp" && mv "$REGISTRY_FILE.tmp" "$REGISTRY_FILE"
else
    echo "WARNING: Module not found in registry."
fi

# 4. DELETE THE DOWNLOADED CODE (If it exists)
if [ -d "$MODULE_DIR" ]; then
    echo "Deleting cloned repository from src/modules..."
    rm -rf "$MODULE_DIR"
fi

echo "==================================================="
echo "SCRUB COMPLETE: $MODULE_NAME has been completely erased from the Pi."
echo "==================================================="
echo "Note: Don't forget to also delete the Deploy Key from the GitHub repository settings if it was already added!"
