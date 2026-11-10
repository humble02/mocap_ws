#!/usr/bin/env bash
# Source this file in each terminal that uses this workspace.
_mocap_source_jazzy() {
    local mocap_root
    mocap_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    if [[ "${ROS_DISTRO:-jazzy}" != jazzy ]]; then
        echo "Open a new terminal without another ROS distribution sourced." >&2
        return 1
    fi
    if [[ ! -f "$mocap_root/install_jazzy/local_setup.bash" || ! -x "$mocap_root/env_jazzy/bin/python" ]]; then
        echo "Run $mocap_root/setup_jazzy.sh first." >&2
        return 1
    fi
    source /opt/ros/jazzy/setup.bash || return
    # local_setup avoids chaining old workspace underlays recorded by colcon.
    source "$mocap_root/install_jazzy/local_setup.bash" || return
    export VIRTUAL_ENV="$mocap_root/env_jazzy"
    export PATH="$VIRTUAL_ENV/bin:$PATH"
    export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
    export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
}
_mocap_source_jazzy
