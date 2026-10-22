#!/usr/bin/env bash
# One-time setup of the v10 ROS 2 environment inside WSL2 Ubuntu 24.04 (design.md 28).
#
#   wsl -d Ubuntu-24.04 -- bash "/mnt/c/Dropbox/Personal Projects/Robotics/omni-robot/ros/setup/install_wsl.sh"
#
# Installs ROS 2 Jazzy desktop, colcon, slam_toolbox, robot_localization, Nav2, foxglove_bridge, joy / teleop,
# then a Python venv (~/.venvs/omni-ros, with the system site packages so rclpy is visible) holding MuJoCo at the
# same version as the Windows venv. Safe to re-run. Needs sudo inside WSL only; nothing on the Windows side.
set -euo pipefail

MUJOCO_VERSION=3.16.0          # = C:\Users\ferna\.venvs\omni-robot-sim (same physics on both sides)
VENV="$HOME/.venvs/omni-ros"

echo "== locale, base tools"
sudo apt-get update -qq
sudo apt-get install -y -qq locales software-properties-common curl rsync bc git python3-venv python3-pip \
    joystick evtest mesa-utils >/dev/null
sudo locale-gen en_US en_US.UTF-8 >/dev/null
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
sudo add-apt-repository -y universe >/dev/null

echo "== ROS 2 apt source"
if ! dpkg -s ros2-apt-source >/dev/null 2>&1; then
    VER=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
          | grep -F '"tag_name"' | awk -F'"' '{print $4}')
    CODENAME=$(. /etc/os-release && echo "${UBUNTU_CODENAME:-$VERSION_CODENAME}")
    curl -sSL -o /tmp/ros2-apt-source.deb \
        "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${VER}/ros2-apt-source_${VER}.${CODENAME}_all.deb"
    sudo dpkg -i /tmp/ros2-apt-source.deb
fi
sudo apt-get update -qq
sudo DEBIAN_FRONTEND=noninteractive apt-get upgrade -y -qq

echo "== ROS 2 Jazzy desktop + the stack (several GB; 10-30 min)"
sudo DEBIAN_FRONTEND=noninteractive apt-get install -y -qq \
    ros-jazzy-desktop ros-dev-tools python3-colcon-common-extensions \
    ros-jazzy-slam-toolbox ros-jazzy-robot-localization \
    ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-nav2-simple-commander \
    ros-jazzy-foxglove-bridge \
    ros-jazzy-joy ros-jazzy-teleop-twist-joy ros-jazzy-teleop-twist-keyboard \
    ros-jazzy-rosbag2-storage-mcap >/dev/null

echo "== rosdep"
[ -f /etc/ros/rosdep/sources.list.d/20-default.list ] || sudo rosdep init >/dev/null
rosdep update >/dev/null

echo "== Python venv for MuJoCo: $VENV"
mkdir -p "$(dirname "$VENV")"
[ -d "$VENV" ] || python3 -m venv --system-site-packages "$VENV"
"$VENV/bin/pip" install -q --upgrade pip
"$VENV/bin/pip" install -q "mujoco==${MUJOCO_VERSION}" scipy pillow pytest websockets
set +u; source /opt/ros/jazzy/setup.bash; set -u
"$VENV/bin/python" -c "import mujoco, numpy, scipy, rclpy; print('mujoco', mujoco.__version__, 'numpy', numpy.__version__, 'rclpy ok')"

echo "== shell setup"
LINE='source /opt/ros/jazzy/setup.bash'
grep -qxF "$LINE" ~/.bashrc || echo "$LINE" >> ~/.bashrc
LINE='[ -f ~/omni_ws/install/setup.bash ] && source ~/omni_ws/install/setup.bash'
grep -qxF "$LINE" ~/.bashrc || echo "$LINE" >> ~/.bashrc
echo "done"
