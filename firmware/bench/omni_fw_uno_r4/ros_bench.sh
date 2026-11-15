#!/usr/bin/env bash
# On the robot computer (the Pi): the robot's mcu_link and mode_manager against the bench MCU on USB, then
# teleop_bench.py drives TELEOP twists through them. Stops only the processes it started.
#   bash ~/fw_bench/ros_bench.sh            (flash_pi.sh copies this folder's scripts to ~/fw_bench)
set -eo pipefail
PORT=${MCU_PORT:-$(ls /dev/serial/by-id/usb-Arduino* | head -1)}
WS=~/omni_ws
OUT=~/fw_bench/teleop_$(date +%H%M%S)
source /opt/ros/jazzy/setup.bash
source $WS/install/setup.bash
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-44} ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST   # its own domain, this machine only
PARAMS=$WS/src/omni_bringup/config/robot.yaml

# each node in its own session: the trap signals the whole group (ros2 run's wrapper does not pass a kill on)
setsid ros2 run omni_cpp mcu_link --ros-args --params-file $PARAMS -p port:=serial:$PORT > $OUT.mcu_link.log 2>&1 &
P1=$!
setsid ros2 run omni_cpp mode_manager --ros-args --params-file $PARAMS -p arm:=false > $OUT.mode_manager.log 2>&1 &
P2=$!
trap 'kill -INT -- -$P1 -$P2 2>/dev/null; sleep 2; kill -- -$P1 -$P2 2>/dev/null || true' EXIT
sleep 3
python3 ~/fw_bench/teleop_bench.py $OUT.csv
echo "== mcu_link log"; grep -vE '^\s*$' $OUT.mcu_link.log | tail -5
