#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
action=${1:-help}; shift || true
if [[ "$action" == build || "$action" == test ]];then exec bash scripts/lesson02.sh "$action" "$@";fi
if [[ -n "${RM_ROS_SETUP:-}" ]]; then
  source "$RM_ROS_SETUP"
elif [[ -f /opt/ros/humble/setup.bash ]]; then
  source /opt/ros/humble/setup.bash
else
  source "$HOME/.config/ros2/humble.bash"
fi
source simulation/install_interfaces/setup.bash
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-89}
case "$action" in
 camera) exec ./build/camera_record configs/simulation.yaml "$@" ;;
 video) exec ./build/armor_video "$@" ;;
 training) exec ./build/training configs/simulation.yaml ;;
 *) echo 'Usage: bash lesson03/scripts/run.sh {build|test|camera OUT.avi SECONDS|video IN OUT.avi FRAMES [show]|training}';;
esac
