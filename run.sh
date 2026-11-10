#!/usr/bin/env bash
# FZMotion -> ROS 2 Jazzy. YAML defaults are applied before CLI overrides.
set -eo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$HERE/source_jazzy.sh"
CONFIG="${FZMOTION_CONFIG:-$HERE/src/fzmotion_ros/config/fzmotion.yaml}"
if [[ ! -f "$CONFIG" ]]; then
    echo "Parameter file not found: $CONFIG" >&2
    exit 1
fi
exec "$HERE/env_jazzy/bin/python" -m fzmotion_ros.fzmotion_node \
    --ros-args --params-file "$CONFIG" "$@"
