#!/usr/bin/env bash
set -eo pipefail
REPO_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
SIM_ROOT="$REPO_ROOT/simulation"
source "${RM_ROS_SETUP:-/opt/ros/humble/setup.bash}"
source "$SIM_ROOT/install_interfaces/setup.bash"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-89}
export WINIT_UNIX_BACKEND=${WINIT_UNIX_BACKEND:-x11}
if [[ "$WINIT_UNIX_BACKEND" == x11 ]]; then
    # winit 0.30 selects Wayland from these variables, ignoring WINIT_UNIX_BACKEND.
    unset WAYLAND_DISPLAY WAYLAND_SOCKET
fi
export RUST_LOG="${RUST_LOG:-warn},daedalus::training=info"
export BEVY_ASSET_ROOT="$SIM_ROOT/simulator"
export LD_LIBRARY_PATH="$SIM_ROOT/simulator/bin/lib:${LD_LIBRARY_PATH:-}"
# 演示可在独立目录读取临时 config.toml，原始场景配置不变。
cd "${RM_SIM_RUN_DIR:-$SIM_ROOT/simulator}"
if [[ ! -f config.toml ]]; then
    echo "找不到仿真配置：$PWD/config.toml" >&2
    exit 1
fi
exec "$SIM_ROOT/simulator/bin/daedalus"
