#!/usr/bin/env bash
# ROS 2 Jazzy (headless ros-base) + build tools on a Pi running Ubuntu 24.04.
# The same apt source as ros/setup/install_wsl.sh; no desktop, Nav2 or sim here:
# this board only captures and detects. Safe to re-run.
# Usage (on the Pi): bash install_ros.sh
set -euo pipefail
APT="sudo DEBIAN_FRONTEND=noninteractive apt-get -o DPkg::Lock::Timeout=900"

echo "== swap (Ubuntu Server for Pi has none; four rclcpp compiles hung a 4 GB Pi 4)"
if ! swapon --show | grep -q /swapfile; then
    sudo fallocate -l 4G /swapfile && sudo chmod 600 /swapfile && sudo mkswap /swapfile >/dev/null
    sudo swapon /swapfile
    grep -q "^/swapfile" /etc/fstab || echo "/swapfile none swap sw 0 0" | sudo tee -a /etc/fstab >/dev/null
fi

echo "== CPU governor: performance at boot (the kernel's ondemand clocks down between 120 fps frames:"
echo "   capture alone was 45 ms sensor -> Python with ~35 drops, 16 ms and none with performance)"
sudo tee /etc/systemd/system/cpu-performance.service >/dev/null <<'EOF'
[Unit]
Description=CPU governor performance (camera head latency)

[Service]
Type=oneshot
ExecStart=/bin/sh -c 'for g in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do echo performance > $g; done'
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
EOF
sudo systemctl daemon-reload
sudo systemctl enable --now cpu-performance.service

echo "== locale, base tools"
$APT update -qq
$APT install -y -qq locales software-properties-common curl rsync git python3-venv python3-pip \
    python3-numpy python3-pytest htop >/dev/null
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
$APT update -qq

echo "== ROS 2 Jazzy ros-base + colcon"
$APT install -y -qq ros-jazzy-ros-base ros-dev-tools python3-colcon-common-extensions >/dev/null

echo "== rosdep"
[ -f /etc/ros/rosdep/sources.list.d/20-default.list ] || sudo rosdep init >/dev/null
rosdep update >/dev/null

grep -q "/opt/ros/jazzy/setup.bash" ~/.bashrc || echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
set +u   # ROS's setup scripts read unset variables
source /opt/ros/jazzy/setup.bash
echo "ROS_DISTRO=$ROS_DISTRO"
ros2 pkg list | wc -l
