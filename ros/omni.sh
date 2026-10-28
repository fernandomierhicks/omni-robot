#!/usr/bin/env bash
# omni-robot ROS 2 stack (design.md 28): one command per step. Run inside WSL (Ubuntu-24.04), e.g. from Windows:
#
#   wsl -d Ubuntu-24.04 -- bash "/mnt/c/Dropbox/Personal Projects/Robotics/omni-robot/ros/omni.sh" map
#
# or click it in Launcher.cmd (repo root). The repo (a monorepo in Dropbox) is the source of truth; every command
# mirrors ros/src (the ROS packages) into ~/omni_ws/src and the sim version into ~/omni_ws/sim (ext4), builds there
# when something changed, and copies maps and results back into the repo. Nothing is ever built on /mnt/c.
#
# On the robot (later): clone the repo, `colcon build --packages-skip omni_sim`, launch omni_bringup with the real
# drivers instead of omni_sim's bridge.
set -eo pipefail

REPO="/mnt/c/Dropbox/Personal Projects/Robotics/omni-robot"
SIM_VERSION="${OMNI_SIM_VERSION:-v11 sil}"                  # the MuJoCo sim the bridge runs (v10 ros bridge: v10's)
SIM_WIN="$REPO/simulation/$SIM_VERSION"
WS="$HOME/omni_ws"
VENV="$HOME/.venvs/omni-ros"
export OMNI_SIM_DIR="$WS/sim"
export OMNI_MAPS_DIR="$WS/maps"
# v11 (design.md 30): the robot's firmware (its C++ core runs in the sim: SIL, built here with g++ on first use) and
# the robot's ROS sources, which the sim's in-process harness and tests import
export OMNI_FIRMWARE_DIR="$WS/firmware"
export OMNI_ROS_SRC="$WS/src"
export OMNI_VERSION_FILE="$WS/version.json"
export PYTHONDONTWRITEBYTECODE=1
export RCUTILS_COLORIZED_OUTPUT=0
# MuJoCo's 3D view is drawn on Windows (ros/tools/mujoco_view.py, native OpenGL), started by the bridge through WSL's
# interop: in WSL the house took ~0.8 s a frame through WSLg's D3D12 translation, or the sim's CPU through llvmpipe
WIN_HOME="$(wslpath "$(/mnt/c/Windows/System32/cmd.exe /c 'echo %USERPROFILE%' 2>/dev/null | tr -d '\r')" 2>/dev/null || true)"
export OMNI_WIN_PY="$WIN_HOME/.venvs/omni-robot-sim/Scripts/python.exe"
export OMNI_WIN_VIEWER="$(wslpath -w "$REPO/ros/tools/mujoco_view.py")"
export OMNI_WIN_FACE="$(wslpath -w "$REPO/ros/tools/face_view.py")"     # the robot's face and voice (omni_face)
export OMNI_WIN_SIM="$(wslpath -w "$SIM_WIN")"
# what still draws in WSL (RViz): software OpenGL, limited to 4 threads so it leaves the sim its cores
export LP_NUM_THREADS=4

sync() {
    mkdir -p "$WS/src" "$OMNI_SIM_DIR" "$OMNI_MAPS_DIR"
    rsync -a --delete --exclude '__pycache__' "$REPO/ros/src/" "$WS/src/"
    rsync -a --delete --exclude '__pycache__' --exclude 'sil/build/' --exclude '.pio/' "$REPO/firmware/" "$OMNI_FIRMWARE_DIR/"
    rsync -a --delete --exclude '__pycache__' --exclude '.pytest_cache' --exclude 'renders/' \
        --exclude 'studies/results/' --exclude '*.mp4' "$SIM_WIN/" "$OMNI_SIM_DIR/"
    mkdir -p "$OMNI_SIM_DIR/studies/results"
    rsync -a --update "$REPO/ros/maps/" "$OMNI_MAPS_DIR/" 2>/dev/null || true
    version_file
}

version_file() {                    # what software this is, for the console's Version tab (cheap: no git status)
    local g=(git -c safe.directory='*' -C "$REPO")
    printf '{"commit": "%s", "commit_time": "%s", "synced": "%s"}\n' \
        "$("${g[@]}" rev-parse --short HEAD 2>/dev/null || echo unknown)" \
        "$("${g[@]}" log -1 --format=%cI 2>/dev/null || true)" "$(date -Iseconds)" > "$WS/version.json"
}

build() {
    sync
    source /opt/ros/jazzy/setup.bash
    source "$VENV/bin/activate"
    local stamp="$WS/.build_stamp"
    if [ ! -f "$stamp" ] || [ -n "$(find "$WS/src" -newer "$stamp" -type f -print -quit)" ]; then
        echo "building ROS packages ..."
        (cd "$WS" && python -m colcon build --symlink-install --event-handlers console_direct- >/dev/null) \
            || { echo "colcon build failed: cd ~/omni_ws && colcon build"; exit 1; }
        touch "$stamp"
    fi
    source "$WS/install/setup.bash"
}

copy_back() {                       # maps and results into the repo
    mkdir -p "$REPO/ros/maps" "$SIM_WIN/studies/results"
    rsync -a --update "$OMNI_MAPS_DIR/" "$REPO/ros/maps/" 2>/dev/null || true
    rsync -a --update "$OMNI_SIM_DIR/studies/results/" "$SIM_WIN/studies/results/" 2>/dev/null || true
}

stop_all() {                        # everything this script started, in any ROS domain
    pkill -INT -f "ros2 launch omni_bringup" 2>/dev/null || true
    sleep 3
    pkill -9 -f "omni_sim/|omni_perception/|omni_bringup/|omni_mcu/|omni_catch/|omni_console/|omni_slam/|omni_face/|omni_arm/|rosbridge_websocket|rosapi_node|rviz2|foxglove_bridge|slam_toolbox|ekf_node|teleop_node|nav2_|controller_server|planner_server|bt_navigator|behavior_server|smoother_server|velocity_smoother|collision_monitor|lifecycle_manager" 2>/dev/null || true
    echo "stopped"
}

cmd="${1:-help}"; shift || true
case "$cmd" in
    sync)   sync ;;
    build)  build; echo "built: $(ls "$WS/install" | grep omni_ | tr '\n' ' ')" ;;
    stop)   stop_all ;;
    status) if pgrep -f "python3 -m omni_console.supervisor" >/dev/null; then echo "console: running (http://localhost:8080)"; fi
            pgrep -af "ros2 launch omni_bringup" | sed 's/.*launch omni_bringup /running: /' || echo "nothing running" ;;
    # the operator console as the launcher (design.md 2): its supervisor serves http://localhost:8080 and starts and
    # stops the steps above from the page (each through this script). Options: --autostart robot, key:=value defaults
    console) build
             if pgrep -f "omni_console.supervisor" >/dev/null; then echo "the console already runs: http://localhost:8080"; exit 0; fi
             exec python3 -m omni_console.supervisor --steps sim --omni-sh "$REPO/ros/omni.sh" \
                 --web "$WS/install/omni_console/share/omni_console/web" "$@" ;;
    stop-console) pkill -INT -f "omni_console.supervisor" 2>/dev/null && echo "console stopping (and its step)" || echo "no console running" ;;
    py)     sync; source /opt/ros/jazzy/setup.bash; cd "$OMNI_SIM_DIR"; "$VENV/bin/python" "$@"; copy_back ;;
    test)   sync; source /opt/ros/jazzy/setup.bash; cd "$OMNI_SIM_DIR"; "$VENV/bin/python" -m pytest -q "$@" ;;
    soak)   source /opt/ros/jazzy/setup.bash; source "$WS/install/setup.bash"     # against a running 'robot' step
            exec python3 -u "$REPO/ros/tools/soak.py" "$@" ;;
    *)      build; trap copy_back EXIT
            case "$cmd" in
                sim)       ros2 launch omni_bringup sim.launch.py "$@" ;;
                map)       ros2 launch omni_bringup mapping.launch.py "$@" ;;
                save-map)  ros2 run omni_bringup save_map --ros-args -p name:="${1:-house}" ;;
                localize)  ros2 launch omni_bringup localization.launch.py "$@" ;;
                nav)       ros2 launch omni_bringup navigation.launch.py "$@" ;;
                robot)     ros2 launch omni_bringup robot.launch.py "$@" ;;
                arm)       ros2 launch omni_bringup arm.launch.py "$@" ;;
                auto-map)  ros2 launch omni_bringup auto_map.launch.py "$@" ;;
                tour)      ros2 launch omni_bringup tour.launch.py "$@" ;;
                goals)     ros2 launch omni_bringup goals.launch.py "$@" ;;
                score-map) ros2 run omni_sim score_map "$@" ;;
                catch)     ros2 launch omni_bringup catch.launch.py "$@" ;;
                catch-score) ros2 launch omni_bringup catch_score.launch.py "$@" ;;
                help|*)    sed -n '2,13p' "$0"; echo
                           echo "steps: sim map save-map localize nav robot arm catch | auto-map tour goals score-map catch-score | console stop-console | soak [--hours 2] (with robot running: patrol the house, watch it) | build test py stop status" ;;
            esac ;;
esac
