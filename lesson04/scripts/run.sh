#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
action=${1:-help}; if [[ $# -gt 0 ]];then shift;fi
if [[ "$action" == build || "$action" == test ]];then exec bash scripts/lesson02.sh "$action" "$@";fi
if [[ -f /opt/ros/humble/setup.bash ]];then source /opt/ros/humble/setup.bash;else source "$HOME/.config/ros2/humble.bash";fi
source simulation/install_interfaces/setup.bash
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-94}
case "$action" in
 simulator)
  scenario=${1:-static};case "$scenario" in static|translation);;*) echo 'scenario must be static or translation' >&2;exit 2;;esac
  export RM_LESSON04_SCENARIO="$scenario"
  export RM_LESSON04_FIXED_OBSERVER=1
  if [[ -f /usr/share/vulkan/icd.d/nvidia_icd.json && -e /dev/nvidia0 ]];then
   export VK_DRIVER_FILES=/usr/share/vulkan/icd.d/nvidia_icd.json VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia WGPU_BACKEND=vulkan WGPU_ADAPTER_NAME='RTX 5060'
  fi
  exec bash scripts/run_simulator.sh;;
 demo) exec ./build/lesson04_demo "$@";;
 training) exec ./build/training configs/simulation.yaml;;
 ros) exec ros2 "$@";;
 *) echo 'Usage: bash lesson04/scripts/run.sh {build|test|simulator static|simulator translation|demo [--output DIR --seconds 25 --horizon-ms 250]|training|ros ...}';;
esac
