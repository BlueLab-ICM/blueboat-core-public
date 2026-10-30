#!/bin/bash
# Load a plan from experiments/ and start it, for systems without a console.
# Usage: run-plan.sh <plan.json under experiments/>
set -euo pipefail

source "$(dirname "${BASH_SOURCE[0]}")/scripts/lib/common.sh"

if [[ $# -ne 1 ]]; then
    echo "Usage: $0 <plan.json under experiments/>" >&2
    exit 2
fi

cli() {
    docker exec -i core_executive bash -c \
        "source install/setup.bash && ros2 run core_executive executive_cli $*"
}

cli load "/experiments/$(basename "$1")"
cli start
