#!/usr/bin/env bash
# On the Pi: the robot's mcu_link + mode_manager against the bench MCU, and the operator console (rosbridge, rosapi,
# omni_console's mission_node and console_web), so the PC's browser drives it: http://omni-cam.local:8080
# (Test > Motors: each wheel motor by itself). Starts and stops only its own processes (their process groups).
#   bash ~/fw_bench/console_bench.sh start | stop | status
set -eo pipefail
WS=~/omni_ws
RUN=~/fw_bench/console_run
mkdir -p $RUN
case "${1:-status}" in
start)
    if [ -s $RUN/pids ]; then echo "already running (stop first)"; exit 1; fi
    PORT=${MCU_PORT:-$(ls /dev/serial/by-id/usb-Arduino* | head -1)}
    source /opt/ros/jazzy/setup.bash
    source $WS/install/setup.bash
    export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-44} ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
    P=$WS/src/omni_bringup/config/robot.yaml
    go() { local name=$1; shift; setsid "$@" > $RUN/$name.log 2>&1 & echo $! >> $RUN/pids; }
    go mcu_link ros2 run omni_cpp mcu_link --ros-args --params-file $P -p port:=serial:$PORT
    go mode_manager ros2 run omni_cpp mode_manager --ros-args --params-file $P -p arm:=false
    go rosbridge ros2 run rosbridge_server rosbridge_websocket --ros-args -p port:=9090
    go rosapi ros2 run rosapi rosapi_node
    go console ros2 run omni_console mission_node --ros-args -r __node:=omni_console \
        -p state_dir:=$RUN/state -p places_dir:=$WS/maps -p log_source:=bench -p cmd_vel_topic:=/cmd_vel/manual
    go console_web ros2 run omni_console console_web
    echo "started on $PORT (ROS domain $ROS_DOMAIN_ID): http://$(hostname).local:8080   logs: $RUN"
    ;;
stop)
    [ -s $RUN/pids ] || { echo "not running"; exit 0; }
    for p in $(cat $RUN/pids); do kill -INT -- -$p 2>/dev/null || true; done
    sleep 2
    for p in $(cat $RUN/pids); do kill -- -$p 2>/dev/null || true; done
    rm -f $RUN/pids; echo stopped
    ;;
status)
    [ -s $RUN/pids ] || { echo "not running"; exit 0; }
    for p in $(cat $RUN/pids); do ps -o pid=,etime=,args= -g $p 2>/dev/null | grep -v "ros2 run" | head -1; done
    ;;
esac
