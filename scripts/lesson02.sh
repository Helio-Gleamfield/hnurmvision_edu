#!/usr/bin/env bash
# 第二讲统一入口：只在本进程加载依赖，不改系统和 shell 配置。
set -eo pipefail
REPO_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd "$REPO_ROOT"
deps_root="$HOME/RM_Week1_Tools/deps/ubuntu22_humble"
if [[ -z "${RM_ROS_SETUP:-}" ]]; then
    if [[ -f /opt/ros/humble/setup.bash ]]; then
        export RM_ROS_SETUP=/opt/ros/humble/setup.bash
    else
        export RM_ROS_SETUP="$deps_root/opt/ros/humble/setup.bash"
    fi
fi
if [[ "$RM_ROS_SETUP" == "$deps_root/opt/ros/humble/setup.bash" ]]; then
    export PATH="$deps_root/usr/bin:$PATH"
    export PYTHONPATH="$deps_root/usr/lib/python3/dist-packages${PYTHONPATH:+:$PYTHONPATH}"
    export LD_LIBRARY_PATH="$deps_root/usr/lib:$deps_root/usr/lib/x86_64-linux-gnu:$deps_root/usr/lib/x86_64-linux-gnu/blas:$deps_root/usr/lib/x86_64-linux-gnu/lapack${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
    export CMAKE_PREFIX_PATH="$deps_root/usr:$deps_root/opt/ros/humble${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
    export C_INCLUDE_PATH="$deps_root/usr/include${C_INCLUDE_PATH:+:$C_INCLUDE_PATH}"
    export CPLUS_INCLUDE_PATH="$deps_root/usr/include${CPLUS_INCLUDE_PATH:+:$CPLUS_INCLUDE_PATH}"
fi
if [[ ! -f "$RM_ROS_SETUP" ]]; then
    echo "找不到 ROS2 Humble：$RM_ROS_SETUP" >&2
    exit 1
fi
source "$RM_ROS_SETUP"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-89}
export PYTHONDONTWRITEBYTECODE=1
action=${1:-help}
if [[ $# -gt 0 ]]; then shift; fi
if [[ "$action" == build ]]; then exec bash scripts/build_training.sh; fi
source simulation/install_interfaces/setup.bash
case "$action" in
    test) exec ctest --test-dir build --output-on-failure "$@" ;;
    training) exec ./build/training "${1:-configs/simulation.yaml}" ;;
    simulator)
        if [[ -e /dev/nvidia0 && -f /usr/share/vulkan/icd.d/nvidia_icd.json ]]; then
            export VK_DRIVER_FILES=/usr/share/vulkan/icd.d/nvidia_icd.json
            export VK_ICD_FILENAMES="$VK_DRIVER_FILES"
            export __NV_PRIME_RENDER_OFFLOAD=1
            export __GLX_VENDOR_LIBRARY_NAME=nvidia
            export WGPU_BACKEND=vulkan
            export WGPU_ADAPTER_NAME="RTX 5060"
        fi
        exec bash scripts/run_simulator.sh ;;
    inspect)
        ros2 node list --no-daemon
        ros2 topic list --no-daemon
        ros2 topic info /image_raw -v --no-daemon
        timeout 10s ros2 topic echo /vision_send_data --once --no-daemon
        ;;
    demo) exec /usr/bin/python3 -u scripts/lesson02_demo.py "$@" ;;
    ros) exec ros2 "$@" ;;
    *) echo "用法：bash scripts/lesson02.sh {build|test|training|simulator|inspect|demo|ros}" ;;
esac
