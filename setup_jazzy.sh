#!/usr/bin/env bash
# Build with the same Python ABI as the Ubuntu 24.04 Jazzy binaries.
set -eo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ "${ROS_DISTRO:-jazzy}" != jazzy ]]; then
    echo "Open a new terminal without another ROS distribution sourced." >&2
    exit 1
fi
if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
    echo "ROS 2 Jazzy is required at /opt/ros/jazzy." >&2
    exit 1
fi
/usr/bin/python3 -c 'import sys; assert sys.version_info[:2] == (3, 12), "Ubuntu 24.04 / Python 3.12 required"'
source /opt/ros/jazzy/setup.bash
cd "$HERE"
# Keep the old Humble env/, build/, install/ and log/ intact.
if [[ ! -f env_jazzy/pyvenv.cfg ]]; then
    /usr/bin/python3 -m venv --system-site-packages --without-pip env_jazzy
fi
touch env_jazzy/COLCON_IGNORE
PYTHON="$HERE/env_jazzy/bin/python"
"$PYTHON" -c 'import sys; assert sys.version_info[:2] == (3, 12); assert sys.prefix != sys.base_prefix'
# Ubuntu's python3-pip is visible through --system-site-packages. Installs go
# only into env_jazzy; no --break-system-packages or global pip installation.
"$PYTHON" -m pip install -r requirements.txt
export PATH="$HERE/env_jazzy/bin:$PATH"
export VIRTUAL_ENV="$HERE/env_jazzy"
"$PYTHON" -m colcon --log-base log_jazzy build \
    --base-paths src --packages-select fzmotion_ros --symlink-install \
    --build-base build_jazzy --install-base install_jazzy
source "$HERE/install_jazzy/local_setup.bash"
"$PYTHON" -c 'import rclpy, tf2_ros, zmq; from fzmotion_ros import LusterFrameStruct_pb2; print("Jazzy / Python 3.12 imports OK")'
echo "Ready: ./run.sh  (or source ./source_jazzy.sh)"
