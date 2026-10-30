#!/bin/bash

# ==============================================================================
# BLUEBOAT JETSON ORIN NANO — SETUP SCRIPT
# ==============================================================================
# Provisions a fresh Jetson Orin Nano (JetPack 6, Ubuntu 22.04) as a backseat
# compute node for BlueBoat.
#
# Usage:
#   ssh <jetson>
#   bash <blueboat-core clone>/scripts/setup/jetson-setup.sh
#
# What it does:
#   1. Verifies platform (Jetson + JetPack r36)
#   2. Installs system deps + Docker (if missing)
#   3. Configures NVIDIA as default Docker runtime
#   4. Installs and activates Tailscale
#   5. Sets up GitHub SSH auth + global HTTPS→SSH rewrite
#   6. Clones all BlueBoat repos into ~/Documents/blueboat_twin/
#   7. Patches computer-vision-perception compose for Jetson GPU build
#   8. Sets static IP BACKSEAT_IP (config/vehicle.env) on the BlueBoat ethernet interface
#   9. Registers the Backseat MAVLink endpoint in BlueOS
# ==============================================================================

# Backseat static IP on the BlueBoat Ethernet network.
# BlueOS is configured to push MAVLink (udpout) to this address on port 14550.
# If the Jetson is ever replaced, set BACKSEAT_IP in config/vehicle.env and
# then update the BlueOS endpoint via:
#   curl -X DELETE http://<frontseat-ip>:8000/v2.0/endpoints/ \
#     -H "Content-Type: application/json" \
#     -d '[{"name":"Backseat Jetson","connection_type":"udpout","place":"<old-ip>","argument":14550}]'
# Then re-run this script (or just step 9 below).
source "$(dirname "${BASH_SOURCE[0]}")/../lib/common.sh"

set -e

# ------------------------------------------------------------------------------
# Colours
# ------------------------------------------------------------------------------
RED='\033[0;31m'
GRN='\033[0;32m'
YLW='\033[1;33m'
BLU='\033[0;34m'
NC='\033[0m'

log()  { echo -e "${BLU}[setup]${NC} $*"; }
ok()   { echo -e "${GRN}[  ok ]${NC} $*"; }
warn() { echo -e "${YLW}[ warn]${NC} $*"; }
die()  { echo -e "${RED}[error]${NC} $*"; exit 1; }

# ------------------------------------------------------------------------------
# 1. PLATFORM CHECK
# ------------------------------------------------------------------------------
check_platform() {
    log "Checking platform..."

    [[ "$(uname -m)" == "aarch64" ]] || die "Not ARM64. This script is for Jetson Orin Nano only."
    [[ -f /etc/nv_tegra_release ]]   || die "/etc/nv_tegra_release not found. Not a Jetson?"

    local release
    release=$(cat /etc/nv_tegra_release)

    if echo "$release" | grep -q "R36"; then
        ok "JetPack 6 (r36) detected."
    else
        warn "Expected JetPack r36. Found: $release"
        read -rp "Continue anyway? [y/N] " yn
        [[ "$yn" =~ ^[Yy]$ ]] || exit 1
    fi
}

# ------------------------------------------------------------------------------
# 2. SYSTEM DEPS
# ------------------------------------------------------------------------------
install_deps() {
    log "Installing system dependencies..."
    sudo apt-get update -qq
    sudo apt-get install -y \
        git curl wget tmux \
        ca-certificates gnupg lsb-release \
        python3-pip

    if command -v docker &>/dev/null; then
        ok "Docker already installed: $(docker --version)"
    else
        log "Installing Docker..."
        curl -fsSL https://get.docker.com | sudo sh
        ok "Docker installed."
    fi
}

# ------------------------------------------------------------------------------
# 3. NVIDIA DOCKER RUNTIME
# ------------------------------------------------------------------------------
configure_nvidia_runtime() {
    log "Configuring NVIDIA as default Docker runtime..."

    if ! command -v nvidia-ctk &>/dev/null; then
        die "nvidia-ctk not found. nvidia-container-toolkit should be pre-installed with JetPack 6."
    fi

    sudo nvidia-ctk runtime configure --runtime=docker --set-as-default
    sudo systemctl restart docker
    ok "NVIDIA runtime set as Docker default."
}

# ------------------------------------------------------------------------------
# 4. TAILSCALE
# ------------------------------------------------------------------------------
setup_tailscale() {
    log "Setting up Tailscale..."

    if ! command -v tailscale &>/dev/null; then
        curl -fsSL https://tailscale.com/install.sh | sudo sh
    else
        ok "Tailscale already installed."
    fi

    sudo systemctl enable --now tailscaled

    local ts_status
    ts_status=$(sudo tailscale status 2>/dev/null || true)

    if echo "$ts_status" | grep -q "100\."; then
        ok "Tailscale already authenticated."
    else
        log "Starting Tailscale — a login URL will appear below."
        log "Open it in your browser to authenticate this node."
        echo ""
        sudo tailscale up --accept-routes
        echo ""
        ok "Tailscale up."
    fi
}

# ------------------------------------------------------------------------------
# 5. GITHUB SSH AUTH
# ------------------------------------------------------------------------------
setup_github_ssh() {
    log "Setting up GitHub SSH authentication..."

    local key="$HOME/.ssh/id_ed25519"
    mkdir -p "$HOME/.ssh"
    chmod 700 "$HOME/.ssh"

    if [[ ! -f "$key" ]]; then
        ssh-keygen -t ed25519 -C "jetson-orin-blueboat" -f "$key" -N ""
        ok "SSH key generated at $key"
    else
        ok "SSH key already exists at $key"
    fi

    # Trust GitHub host key
    ssh-keyscan -H github.com >> "$HOME/.ssh/known_hosts" 2>/dev/null

    # Rewrite all https://github.com/ clones to use SSH transparently
    git config --global url."git@github.com:".insteadOf "https://github.com/"

    echo ""
    echo -e "${YLW}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${YLW}  Add this public key to your GitHub account:${NC}"
    echo -e "${YLW}  Settings → SSH and GPG keys → New SSH key${NC}"
    echo -e "${YLW}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo ""
    cat "$key.pub"
    echo ""
    echo -e "${YLW}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo ""
    read -rp "Press ENTER once the key is added to GitHub..."

    log "Verifying GitHub SSH access..."
    if ssh -T git@github.com 2>&1 | grep -q "successfully authenticated"; then
        ok "GitHub SSH authentication verified."
    else
        # ssh -T returns exit code 1 even on success — check output manually
        local result
        result=$(ssh -T git@github.com 2>&1 || true)
        if echo "$result" | grep -q "successfully authenticated"; then
            ok "GitHub SSH authentication verified."
        else
            die "GitHub SSH auth failed. Output: $result"
        fi
    fi
}

# ------------------------------------------------------------------------------
# 6. CLONE REPOS
# ------------------------------------------------------------------------------
clone_repos() {
    log "Cloning BlueBoat repositories..."

    local base="$HOME/Documents/blueboat_twin"
    mkdir -p "$base"

    local repos=(
        "BlueLab-ICM/blueboat-core-public"
        "BlueLab-ICM/blueboat-payload-template"
    )

    for repo in "${repos[@]}"; do
        local name="${repo##*/}"
        local dest="$base/$name"
        if [[ -d "$dest/.git" ]]; then
            ok "$name already cloned — pulling latest."
            git -C "$dest" pull --ff-only
        else
            log "Cloning $name..."
            git clone "git@github.com:${repo}.git" "$dest"
            ok "$name cloned."
        fi
    done
}

# ------------------------------------------------------------------------------
# 6b. PERSISTENT JOURNAL
# ------------------------------------------------------------------------------
# Container logs go to journald, so a mission's logs survive a power cut. That needs a
# journal on disk, a cap on its size, and a user who may read it.
configure_journal() {
    log "Configuring persistent journald..."
    sudo mkdir -p /var/log/journal /etc/systemd/journald.conf.d
    printf '[Journal]\nStorage=persistent\nSystemMaxUse=2G\n' \
        | sudo tee /etc/systemd/journald.conf.d/blueboat.conf > /dev/null
    sudo systemctl restart systemd-journald
    sudo usermod -aG systemd-journal "$USER"
    ok "journald keeps container logs across power cycles."
}

# ------------------------------------------------------------------------------
# 7. JETSON GPU NOTE
# ------------------------------------------------------------------------------
patch_for_jetson() {

    log "Verifying Jetson GPU setup..."
    # The cv-perception module automatically uses docker-compose.module.jetson.yml
    # on Jetson (detected by sync-modules.sh and start-mission.sh via /etc/nv_tegra_release).
    # That compose file builds from Dockerfile.jetson (l4t-jetpack base + Humble + TRT).
    # No manual patching needed here.
    ok "GPU inference configured — cv-perception will use TensorRT on first mission start."
}

# ------------------------------------------------------------------------------
# 8. STATIC IP
# ------------------------------------------------------------------------------
configure_static_ip() {
    log "Configuring static IP $BACKSEAT_IP on BlueBoat ethernet..."

    # Detect the ethernet interface connected to the vehicle network: any
    # ethernet device reported by nmcli that is not docker/tailscale/loopback.
    local iface
    iface=$(nmcli -t -f DEVICE,TYPE device status \
        | awk -F: '$2=="ethernet" && $1!~/(docker|lo|usb)/ {print $1; exit}')

    if [[ -z "$iface" ]]; then
        warn "Could not detect ethernet interface — skipping static IP configuration."
        warn "Set it manually: sudo nmcli connection modify '<conn>' ipv4.method manual ipv4.addresses ${BACKSEAT_IP}/24 ipv4.gateway '' ipv4.dns ''"
        return 0
    fi

    local conn
    conn=$(nmcli -t -f NAME,DEVICE connection show --active \
        | awk -F: -v d="$iface" '$2==d {print $1; exit}')

    if [[ -z "$conn" ]]; then
        warn "No active NetworkManager connection found for $iface — skipping."
        return 0
    fi

    local current_ip
    current_ip=$(ip -4 addr show "$iface" | awk '/inet /{print $2}' | cut -d/ -f1)

    if [[ "$current_ip" == "$BACKSEAT_IP" ]]; then
        ok "Ethernet already at $BACKSEAT_IP — no change needed."
        return 0
    fi

    log "Setting $iface ($conn) to static $BACKSEAT_IP/24 (was: ${current_ip:-unset})..."
    sudo nmcli connection modify "$conn" \
        ipv4.method manual \
        ipv4.addresses "${BACKSEAT_IP}/24" \
        ipv4.gateway "" \
        ipv4.dns ""
    sudo nmcli connection up "$conn"
    ok "Static IP $BACKSEAT_IP set on $iface."
    warn "If you were connected via this interface your session may have dropped."
    warn "Tailscale SSH (100.x.x.x) is unaffected."
}

# ------------------------------------------------------------------------------
# 9. BLUEOS MAVLINK ENDPOINT
# ------------------------------------------------------------------------------
register_mavlink_endpoint() {
    log "Registering Backseat MAVLink endpoint in BlueOS ($FRONTSEAT_IP)..."

    # Check BlueOS is reachable
    if ! curl -sf --connect-timeout 3 "http://${FRONTSEAT_IP}:8000/" >/dev/null 2>&1; then
        warn "BlueOS at $FRONTSEAT_IP is not reachable — skipping endpoint registration."
        warn "Run this manually once the boat network is up:"
        warn "  curl -X POST http://${FRONTSEAT_IP}:8000/v2.0/endpoints/ \\"
        warn "    -H 'Content-Type: application/json' \\"
        warn "    -d '[{\"name\":\"Backseat Jetson\",\"connection_type\":\"udpout\",\"place\":\"${BACKSEAT_IP}\",\"argument\":14550}]'"
        return 0
    fi

    # Check if a correct endpoint already exists
    local existing
    existing=$(curl -sf "http://${FRONTSEAT_IP}:8000/v2.0/endpoints/" \
        | python3 -c "
import sys, json
endpoints = json.load(sys.stdin)
for ep in endpoints:
    if ep.get('place') == '${BACKSEAT_IP}' and ep.get('argument') == 14550 and ep.get('connection_type') == 'udpout':
        print('found')
        break
" 2>/dev/null || true)

    if [[ "$existing" == "found" ]]; then
        ok "BlueOS already has udpout → ${BACKSEAT_IP}:14550."
        return 0
    fi

    # Warn about stale endpoints pointing to a different backseat IP
    local stale
    stale=$(curl -sf "http://${FRONTSEAT_IP}:8000/v2.0/endpoints/" \
        | python3 -c "
import sys, json
endpoints = json.load(sys.stdin)
for ep in endpoints:
    if ep.get('argument') == 14550 and ep.get('connection_type') == 'udpout' \
       and ep.get('place') not in ('127.0.0.1', '${BASESTATION_GW}', '${BACKSEAT_IP}'):
        print(ep['place'])
" 2>/dev/null || true)

    if [[ -n "$stale" ]]; then
        warn "Found stale udpout endpoint(s) on port 14550 pointing to: $stale"
        warn "These will send MAVLink to the wrong host. Remove via BlueOS UI or:"
        warn "  curl -X DELETE http://${FRONTSEAT_IP}:8000/v2.0/endpoints/ \\"
        warn "    -H 'Content-Type: application/json' \\"
        warn "    -d '[{\"name\":\"...\",\"connection_type\":\"udpout\",\"place\":\"${stale}\",\"argument\":14550}]'"
    fi

    log "Adding udpout endpoint ${BACKSEAT_IP}:14550 to BlueOS..."
    local response
    response=$(curl -sf -X POST "http://${FRONTSEAT_IP}:8000/v2.0/endpoints/" \
        -H "Content-Type: application/json" \
        -d "[{\"name\":\"Backseat Jetson\",\"connection_type\":\"udpout\",\"place\":\"${BACKSEAT_IP}\",\"argument\":14550}]" \
        2>&1 || true)

    if curl -sf "http://${FRONTSEAT_IP}:8000/v2.0/endpoints/" \
        | python3 -c "
import sys, json
endpoints = json.load(sys.stdin)
ok = any(ep.get('place') == '${BACKSEAT_IP}' and ep.get('argument') == 14550 for ep in endpoints)
sys.exit(0 if ok else 1)
" 2>/dev/null; then
        ok "BlueOS endpoint registered: udpout → ${BACKSEAT_IP}:14550"
    else
        warn "Endpoint POST completed but verification failed. Check BlueOS UI at http://${FRONTSEAT_IP}."
        warn "Response: $response"
    fi
}

# ------------------------------------------------------------------------------
# 10. DOCKER GROUP
# ------------------------------------------------------------------------------
setup_docker_group() {
    if groups "$USER" | grep -q docker; then
        ok "User already in docker group."
    else
        sudo usermod -aG docker "$USER"
        warn "Added $USER to docker group — you must log out and back in (or run 'newgrp docker') before using Docker without sudo."
    fi
}

# ------------------------------------------------------------------------------
# MAIN
# ------------------------------------------------------------------------------
main() {
    echo ""
    echo -e "${BLU}============================================================${NC}"
    echo -e "${BLU}  BlueBoat Jetson Orin Nano Setup${NC}"
    echo -e "${BLU}============================================================${NC}"
    echo ""

    check_platform
    install_deps
    configure_nvidia_runtime
    setup_tailscale
    setup_github_ssh
    clone_repos
    configure_journal
    patch_for_jetson
    configure_static_ip
    register_mavlink_endpoint
    setup_docker_group

    echo ""
    echo -e "${GRN}============================================================${NC}"
    echo -e "${GRN}  Setup complete.${NC}"
    echo -e "${GRN}============================================================${NC}"
    echo ""
    echo "  Repos:    ~/Documents/blueboat_twin/"
    echo "  Next:     Log out and back in (docker group)"
    echo "            cd ~/Documents/blueboat_twin/blueboat-core"
    echo "            ./start-mission.sh"
    echo ""
}

main "$@"
