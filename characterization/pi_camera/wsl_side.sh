#!/usr/bin/env bash
# The PC's (WSL) half of arm_split.sh:  wsl_side.sh start | drive | stop | copy  TAG [args...]
# (env PC_REPO = the repo as WSL sees it, PI_PEER = the Pi's address for DDS's unicast discovery)
cmd=$1; TAG=$2; shift 2
WREPO=${PC_REPO:?PC_REPO}
export ROS_DOMAIN_ID=43 ROS_STATIC_PEERS=${PI_PEER:-192.168.0.101}
case $cmd in
  start)
    cd "$WREPO/ros"
    setsid nohup bash omni.sh arm side:=sim "$@" >"/tmp/arm_sim_$TAG.log" 2>&1 < /dev/null &
    echo $! >/tmp/arm_sim.pid
    echo "sim side pid $(cat /tmp/arm_sim.pid)" ;;
  drive)
    source /opt/ros/jazzy/setup.bash; source ~/omni_ws/install/setup.bash
    python3 "$WREPO/ros/tools/arm_load.py" "/tmp/arm_load_$TAG.json" "$@" ;;
  stop)
    P=$(cat /tmp/arm_sim.pid); kill -INT -- -"$P" 2>/dev/null; sleep 8; kill -9 -- -"$P" 2>/dev/null
    echo "sim side stopped" ;;
  copy)
    cp "/tmp/arm_load_$TAG.json" "/tmp/arm_sim_$TAG.log" "$WREPO/characterization/pi_camera/results/logs/" ;;
esac
