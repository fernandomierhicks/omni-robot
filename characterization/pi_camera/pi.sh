#!/usr/bin/env bash
# The PC -> Pi loop for the camera-head prototype (Pi 4 "omni-cam", Ubuntu 24.04 + Jazzy).
# Run from Git Bash (or WSL) anywhere in the repo:
#
#   bash characterization/pi_camera/pi.sh deploy   # copy ros/src (no omni_sim), firmware, pi_camera to ~/omni_ws
#   bash characterization/pi_camera/pi.sh build    # deploy + colcon build of the robot packages the head needs
#   bash characterization/pi_camera/pi.sh test     # deploy + omni_cpp's core tests (C++ vs Python cores) on ARM
#   bash characterization/pi_camera/pi.sh run CMD  # run CMD on the Pi with ROS sourced, in ~/omni_ws
#   bash characterization/pi_camera/pi.sh head [ARGS]  # the camera head (capture + catch_perception), bench config
#   bash characterization/pi_camera/pi.sh probe [S]    # head_probe.py against a running head: S seconds (10)
#   bash characterization/pi_camera/pi.sh ping [N]     # ROS 2 round trip WSL <-> Pi, N pings (200)
#   bash characterization/pi_camera/pi.sh robot STEP [ARGS]   # the robot's nodes on the Pi (side:=robot), the sim on the PC
#   bash characterization/pi_camera/pi.sh sim STEP [ARGS]      # the PC's half in WSL (side:=sim): bridge + firmware in the loop
#   bash characterization/pi_camera/pi.sh split STEP LABEL [SECONDS] [ARGS]  # both halves + pi_keepup.py, one scored run
#   bash characterization/pi_camera/pi.sh keepup OUT [S]       # pi_keepup.py alone, beside whatever runs on the Pi
#   bash characterization/pi_camera/pi.sh ghost [ARGS]        # the camera head publishing /ghost/track on the sim's ROS domain
#   bash characterization/pi_camera/pi.sh mcucheck             # can the Pi reach the PC's MCU port (TCP)? the firewall rule
#   bash characterization/pi_camera/pi.sh ssh      # a shell on the Pi
#
# Like omni.sh's mirror into ~/omni_ws: the Pi holds a copy, never the master. Edit on the PC, deploy, run.
set -euo pipefail

HOST=${OMNI_PI:-omni-cam}
REPO=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
WS='~/omni_ws'
# The camera head's packages and what they need (omni_sim is PC only; the rest are the robot computer's).
PKGS=${OMNI_PI_PKGS:-"omni_interfaces omni_mcu omni_catch omni_perception omni_cpp omni_camera omni_bringup"}
# the whole arm robot (design.md 4.9 / 11): OMNI_PI_PKGS="$ARM_PKGS" pi.sh build
ARM_PKGS="omni_interfaces omni_mcu omni_catch omni_perception omni_cpp omni_camera omni_slam omni_calib omni_arm omni_face omni_console omni_bringup"
# the Pi's address for DDS's unicast discovery (OMNI_PI_PEER=192.168.0.101 on the wired switch, rpi.md)
PEER=${OMNI_PI_PEER:-$HOST.local}
SSH=(ssh -o BatchMode=yes)

remote() { "${SSH[@]}" "$HOST" "source /opt/ros/jazzy/setup.bash 2>/dev/null; \
    export OMNI_FIRMWARE_DIR=$WS/firmware; cd $WS 2>/dev/null || true; $*"; }

deploy() {
    echo "== deploy -> $HOST:$WS"
    # tar the tree, unpack into a staging dir, then rsync --delete into place (files removed on the PC go away)
    tar -C "$REPO" -cf - \
        --exclude='__pycache__' --exclude='.pytest_cache' --exclude='.pio' --exclude='sil/build' \
        --exclude='ros/src/omni_sim' --exclude='pi_camera/results/raw' \
        ros/src ros/tools ros/maps firmware characterization/pi_camera \
    | "${SSH[@]}" "$HOST" "set -e; rm -rf ~/.omni_stage; mkdir -p ~/.omni_stage $WS; tar -C ~/.omni_stage -xf -; \
        rsync -a --delete ~/.omni_stage/ros/src/ $WS/src/; \
        rsync -a --delete ~/.omni_stage/firmware/ $WS/firmware/; rsync -a --delete ~/.omni_stage/ros/tools/ $WS/tools/; \
        mkdir -p $WS/maps; rsync -a --update ~/.omni_stage/ros/maps/ $WS/maps/; \
        rsync -a --delete --exclude results/ ~/.omni_stage/characterization/pi_camera/ $WS/pi_camera/; \
        rm -rf ~/.omni_stage; echo deployed: \$(find $WS/src $WS/firmware $WS/pi_camera -type f | wc -l) files"
}

# -- the robot computer in the loop (design.md 8, phase 1C step 2; stack.py side:=sim | robot) -------------------------
# The PC (WSL, mirrored networking: the PC's LAN address) runs the world and the firmware in the loop; the Pi runs the
# robot's nodes and connects to the PC's MCU port. A ROS domain of their own, unicast discovery both ways.
DOMAIN=${OMNI_DOMAIN:-43}
MCU_PORT=${OMNI_MCU_PORT:-5761}
WSL_REPO=$(echo "$REPO" | sed -E 's#^/([a-zA-Z])/#/mnt/\L\1/#')
RESULTS="$REPO/characterization/pi_camera/results"

pc_ip() {         # the PC's LAN address (WSL shares it): from Windows' ipconfig, else OMNI_PC_IP
    if [ -n "${OMNI_PC_IP:-}" ]; then echo "$OMNI_PC_IP"; return; fi
    /c/Windows/System32/ipconfig.exe 2>/dev/null | tr -d '\r' | awk '/IPv4/ && $NF ~ /^192\.168\./ {print $NF; exit}'
}

wsl_run() { MSYS_NO_PATHCONV=1 /c/Windows/System32/wsl.exe -d Ubuntu-24.04 -- bash -c "$1"; }

pi_launch_cmd() { # the Pi's half of a step, as the command line (the step's name uses _ in the launch file)
    local step=$1; shift
    echo "ros2 launch omni_bringup ${step//-/_}.launch.py side:=robot mcu_host:=${OMNI_MCU_HOST:-$(pc_ip)} mcu_port:=$MCU_PORT console:=false $*"
}

pi_env() { echo "source /opt/ros/jazzy/setup.bash; cd $WS && source install/setup.bash && export ROS_DOMAIN_ID=$DOMAIN ROS_STATIC_PEERS=$(pc_ip) OMNI_FIRMWARE_DIR=$WS/firmware"; }

pc_sim_cmd() {    # the PC's half: omni.sh in WSL on the split's ROS domain
    local step=$1; shift
    echo "export ROS_DOMAIN_ID=$DOMAIN ROS_STATIC_PEERS=$PEER; cd '$WSL_REPO/ros' && exec bash omni.sh $step side:=sim mcu_port:=$MCU_PORT console:=false viewer:=false $*"
}

split_run() {     # one scored run: STEP LABEL [SECONDS] [ARGS]
    local step=$1 label=$2 secs=${3:-120}
    shift 2; [ $# -gt 0 ] && shift
    local tag; tag="$(printf '%s' "$label" | tr -c 'A-Za-z0-9' '_' | head -c 40)_$(date +%H%M%S)"
    mkdir -p "$RESULTS/logs"
    echo "== split run '$label': $step, PC $(pc_ip), Pi $HOST, ROS domain $DOMAIN, $secs s"
    # (the [x] in each pattern keeps pkill from matching the remote shell's own command line, which contains it)
    "${SSH[@]}" "$HOST" "pkill -INT -f 'ros2 launch [o]mni_bringup' || true" >/dev/null 2>&1 || true
    wsl_run "cd '$WSL_REPO/ros' && bash omni.sh stop" >/dev/null 2>&1 || true
    # the Pi's half first (mcu_link keeps retrying until the bridge listens), then the PC's
    "${SSH[@]}" "$HOST" "$(pi_env) && (setsid nohup $(pi_launch_cmd "$step" "$@") >/tmp/pi_robot.log 2>&1 &)"
    sleep 5
    (wsl_run "$(pc_sim_cmd "$step" "$@")" >"$RESULTS/logs/${tag}_pc.log" 2>&1 &)
    sleep "${OMNI_SPLIT_WARMUP:-20}"
    "${SSH[@]}" "$HOST" "$(pi_env) && python3 tools/pi_keepup.py pi_camera/results/pi_robot_${tag}.json --seconds $secs --label '$label'; \
        cp /tmp/pi_robot.log pi_camera/results/pi_robot_${tag}.log; \
        pkill -INT -f 'ros2 launch [o]mni_bringup' || true; sleep 3; pkill -9 -f '[o]mni_cpp/|[o]mni_catch/|[o]mni_mcu/|[c]omponent_container|[s]lam_toolbox|[e]kf_node' || true" || true
    wsl_run "cd '$WSL_REPO/ros' && bash omni.sh stop" >/dev/null 2>&1 || true
    scp -q -o BatchMode=yes "$HOST:omni_ws/pi_camera/results/pi_robot_${tag}.*" "$RESULTS/" 2>/dev/null || true
    echo "== results: $RESULTS/pi_robot_${tag}.json  (logs: pi_robot_${tag}.log, logs/${tag}_pc.log)"
}

case "${1:-}" in
    deploy) deploy ;;
    build)
        deploy
        echo "== colcon build: $PKGS"
        # one package at a time, two compiles each: rclcpp units take ~1 GB apiece (4 at once hung a 4 GB Pi 4)
        remote "MAKEFLAGS=-j${OMNI_PI_JOBS:-2} colcon build --packages-up-to $PKGS --executor sequential \
            --event-handlers console_cohesion- status- 2>&1 | tail -15"
        ;;
    test)
        deploy
        echo "== omni_cpp core tests on $(remote 'uname -m')"
        remote "python3 -m pytest -q src/omni_cpp/test ${2:-}"
        ;;
    run) shift; remote "[ -f install/setup.bash ] && source install/setup.bash; $*" ;;
    head)
        # the bench's one-camera robot_config, then the camera head in the foreground (Ctrl-C stops it)
        shift
        "${SSH[@]}" -t "$HOST" "source /opt/ros/jazzy/setup.bash; cd $WS && source install/setup.bash && \
            python3 pi_camera/bench_config.py && \
            exec ros2 launch omni_camera camera_head.launch.py \
            robot_config:=\$HOME/omni_ws/pi_camera/robot_bench.json sleep:=false $*"
        ;;
    probe) remote "source install/setup.bash; python3 pi_camera/head_probe.py --seconds ${2:-10}" ;;
    ping)
        # ROS 2 between WSL and the Pi (rpi.md): an echo on the Pi, pings from WSL with unicast discovery to it
        remote "(setsid nohup python3 pi_camera/dds_ping.py echo --seconds 60 >/tmp/dds_echo.log 2>&1 &); sleep 1"
        wsl_run "source /opt/ros/jazzy/setup.bash; export ROS_STATIC_PEERS=$HOST.local; cd '$WSL_REPO/characterization/pi_camera' && \
            python3 dds_ping.py ping --count ${2:-200}"
        ;;
    ghost)
        # the camera head tracking a real ball, its track going to the PC's sim (omni_sim ghost_ball; on the PC:
        # pi.sh sim catch ghost:=true tosses:=0, which also runs the catch code against it)
        shift
        "${SSH[@]}" -t "$HOST" "$(pi_env) && python3 pi_camera/bench_config.py && exec ros2 launch omni_camera camera_head.launch.py             robot_config:=\$HOME/omni_ws/pi_camera/robot_bench.json sleep:=false track_topic:=/ghost/track $*"
        ;;
    robot) shift; step=$1; shift; "${SSH[@]}" -t "$HOST" "$(pi_env) && exec $(pi_launch_cmd "$step" "$@")" ;;
    sim) shift; step=$1; shift; wsl_run "$(pc_sim_cmd "$step" "$@")" ;;
    keepup) remote "export ROS_DOMAIN_ID=$DOMAIN; source install/setup.bash; python3 tools/pi_keepup.py ${2:?out.json} --seconds ${3:-60}" ;;
    split) shift; split_run "$@" ;;
    mcucheck)
        # a listener in WSL on the MCU port, a connection from the Pi: the Hyper-V firewall rule for TCP is the usual blocker
        (wsl_run "timeout 12 python3 -c \"import socket;s=socket.socket();s.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1);s.bind(('0.0.0.0',$MCU_PORT));s.listen(1);s.settimeout(10);c,a=s.accept();print('PC: accepted',a)\"" &) ; sleep 2
        "${SSH[@]}" "$HOST" "python3 -c \"import socket;c=socket.create_connection(('$(pc_ip)',$MCU_PORT),5);print('Pi: connected to $(pc_ip):$MCU_PORT')\"" \
            || echo "Pi: cannot connect to $(pc_ip):$MCU_PORT -> needs an inbound TCP rule for WSL (admin PowerShell): New-NetFirewallHyperVRule -Name WSL-MCU-TCP -DisplayName 'WSL MCU port from the LAN' -Direction Inbound -VMCreatorId '{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}' -Protocol TCP -LocalPorts $MCU_PORT -RemoteAddresses 192.168.1.0/24 -Action Allow"
        wait
        ;;
    ssh) exec ssh "$HOST" ;;
    *) sed -n '2,17p' "${BASH_SOURCE[0]}"; exit 1 ;;
esac
