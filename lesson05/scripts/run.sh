#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
action=${1:-help};if [[ $# -gt 0 ]];then shift;fi
if [[ "$action" == build || "$action" == test ]];then exec bash scripts/lesson02.sh "$action" "$@";fi
if [[ -f /opt/ros/humble/setup.bash ]];then source /opt/ros/humble/setup.bash;else source "$HOME/.config/ros2/humble.bash";fi
source simulation/install_interfaces/setup.bash
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-95}
case "$action" in
 simulator)
  export RM_LESSON05_AUTOSTART=1
  exec bash lesson04/scripts/run.sh simulator "${1:-static}";;
 demo) exec ./build/lesson05_demo "$@";;
 ros) exec ros2 "$@";;
 *) echo 'Usage: bash lesson05/scripts/run.sh {build|test|simulator static|simulator translation|demo --seconds 30 --output /fresh/directory|ros ...}';;
esac
