#!/bin/bash

# ==============================================================================
# ICM BLUEBOAT: AUTOMATED PAYLOAD REGISTRATION TOOL
# ==============================================================================

if [ -z "$1" ]; then
    echo "ERROR: Please provide a module name."
    echo "Usage: scripts/modules/register-module.sh <module-repo-name>"
    exit 1
fi

MODULE_NAME=$1
ORG_NAME="BlueLab-ICM"
KEY_FILE="$HOME/.ssh/blueboat-${MODULE_NAME}"
SSH_CONFIG="$HOME/.ssh/config"
source "$(dirname "${BASH_SOURCE[0]}")/../lib/common.sh"
REGISTRY_FILE="$CORE_ROOT/config/module-registry.yaml"
REGISTRY_UPDATED=false
REGISTRY_ENTRY="  - name: $MODULE_NAME
    repo: git@github-${MODULE_NAME}:${ORG_NAME}/${MODULE_NAME}.git
    version: main"

echo "Registering new module: $MODULE_NAME"

# 1. GENERATE THE SSH KEY
if [ -f "$KEY_FILE" ]; then
    echo "WARNING: Key already exists for $MODULE_NAME. Skipping generation."
else
    echo "Generating Ed25519 SSH Key..."
    ssh-keygen -t ed25519 -f "$KEY_FILE" -C "deploy-key-$MODULE_NAME" -N "" > /dev/null
fi

# 2. UPDATE SSH CONFIG
if grep -q "Host github-${MODULE_NAME}" "$SSH_CONFIG" 2>/dev/null; then
    echo "SSH alias already exists in $SSH_CONFIG"
else
    echo "Adding SSH routing alias..."
    cat <<EOF >> "$SSH_CONFIG"

# --- AUTO-GENERATED: $MODULE_NAME ---
Host github-${MODULE_NAME}
    HostName github.com
    User git
    IdentityFile $KEY_FILE
    IdentitiesOnly yes
EOF
fi

# 3. APPEND TO PAYLOAD REGISTRY
mkdir -p "$(dirname "$REGISTRY_FILE")"
if [ ! -f "$REGISTRY_FILE" ]; then
    cat > "$REGISTRY_FILE" <<'EOF'
# Module repos this vehicle can run. Entries are added by scripts/modules/register-module.sh;
# set `version` to pin a branch or tag. Which ones start: active_payloads in mission-config.yaml.
modules:
EOF
    REGISTRY_UPDATED=true
fi

if grep -q "name: $MODULE_NAME" "$REGISTRY_FILE" 2>/dev/null; then
    echo "WARNING: Module already exists in registry."
else
    echo "Appending to module-registry.yaml..."
    # Guard against a missing trailing newline on the existing file — a bare
    # `>>` would otherwise glue this entry onto the last line of the
    # previous one (e.g. "version: main  - name: ...") and corrupt the YAML.
    if [ -s "$REGISTRY_FILE" ] && [ -n "$(tail -c1 "$REGISTRY_FILE")" ]; then
        printf '\n' >> "$REGISTRY_FILE"
    fi
    printf '%s\n\n' "$REGISTRY_ENTRY" >> "$REGISTRY_FILE"
    REGISTRY_UPDATED=true
fi

# 4. REGISTRY CHANGE GOES IN FROM THE WORKSTATION
# A vehicle runs a field tag (detached HEAD) or main; a commit here has no branch or diverges.
if [ "$REGISTRY_UPDATED" = true ]; then
    echo "module-registry.yaml changed here. Add the same entry on the workstation and commit it there:"
    echo "$REGISTRY_ENTRY"
fi

# 5. INSTRUCTIONS FOR THE ADMIN
echo "==================================================="
echo "REGISTRATION COMPLETE ON THE PI"
echo "==================================================="
echo "To finish deployment, go to GitHub and add this Deploy Key"
echo "to the repository: https://github.com/$ORG_NAME/$MODULE_NAME/settings/keys"
echo ""
echo "--- COPY BELOW THIS LINE ---"
cat "${KEY_FILE}.pub"
echo "--- COPY ABOVE THIS LINE ---"
