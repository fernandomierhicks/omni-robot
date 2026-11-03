#!/usr/bin/env bash
set -eo pipefail
export ROS_DOMAIN_ID=73
export OMNI_SIM_DIR="$HOME/omni_ws/sim"
export OMNI_FIRMWARE_DIR="$HOME/omni_ws/firmware"
export OMNI_ROS_SRC="$HOME/omni_ws/src"
source /opt/ros/jazzy/setup.bash
source "$HOME/.venvs/omni-ros/bin/activate"
source "$HOME/omni_ws/install/setup.bash"
exec python -u "/mnt/c/Dropbox/Personal Projects/Robotics/omni-robot/ros/tools/arm_load.py" "/mnt/c/Dropbox/Personal Projects/Robotics/omni-robot/tmp/pc_general_arm_wall.json" --phases NAVTIDY,ROUTE --legs 1 --laps 1
