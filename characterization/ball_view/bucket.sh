#!/usr/bin/env bash
# Bucket lab, on its own port; never stops Fernando's other camera processes.
# bash characterization/ball_view/bucket.sh start [--profile orange] [--arducam]
# bash characterization/ball_view/bucket.sh demo | stop | status | deploy
set -euo pipefail
REPO=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
HOST=${OMNI_PI:-omni-cam}
PORT=${BUCKET_PORT:-8086}
SSH=(ssh -o BatchMode=yes -o ConnectTimeout=8)
deploy() {
    tar -C "$REPO" -cf - --exclude=__pycache__ --exclude=.pytest_cache --exclude=data --exclude=results --exclude=settings \
        ros/src/omni_catch/omni_catch characterization/ball_view characterization/throw_track characterization/realsense/rs_common.py \
    | "${SSH[@]}" "$HOST" 'set -e; mkdir -p ~/omni_ws/bucket_stage ~/omni_ws/ball_view ~/omni_ws/throw_track ~/omni_ws/realsense ~/omni_ws/src/omni_catch; tar -C ~/omni_ws/bucket_stage -xf -; cp -a ~/omni_ws/bucket_stage/ros/src/omni_catch/omni_catch/. ~/omni_ws/src/omni_catch/omni_catch/; cp -a ~/omni_ws/bucket_stage/characterization/ball_view/. ~/omni_ws/ball_view/; cp -a ~/omni_ws/bucket_stage/characterization/throw_track/. ~/omni_ws/throw_track/; cp ~/omni_ws/bucket_stage/characterization/realsense/rs_common.py ~/omni_ws/realsense/'
}
case "${1:-start}" in
    deploy) deploy ;;
    start|demo)
        mode=${1:-start}; [ $# -gt 0 ] && shift
        deploy
        source=realsense; [ "$mode" = demo ] && source=synthetic
        "${SSH[@]}" "$HOST" "cd ~/omni_ws/ball_view; if test -s ~/omni_ws/bucket_live.pid && kill -0 \$(cat ~/omni_ws/bucket_live.pid) 2>/dev/null; then echo 'Bucket lab is already running; use its page or stop its own PID first.'; exit 1; fi; nohup ~/.venvs/cam/bin/python -u bucket_live.py --source $source --port $PORT $* > ~/omni_ws/bucket_live.log 2>&1 < /dev/null & echo \$! > ~/omni_ws/bucket_live.pid; cat ~/omni_ws/bucket_live.pid"
        echo "Bucket lab: http://omni-cam.local:$PORT/ (log: ~/omni_ws/bucket_live.log)"
        ;;
    status) "${SSH[@]}" "$HOST" 'test ! -s ~/omni_ws/bucket_live.pid || ps -p $(cat ~/omni_ws/bucket_live.pid) -o pid,etime,pcpu,rss,args; tail -12 ~/omni_ws/bucket_live.log' ;;
    stop)
        "${SSH[@]}" "$HOST" 'if test -s ~/omni_ws/bucket_live.pid; then pid=$(cat ~/omni_ws/bucket_live.pid); if test -r /proc/$pid/cmdline && tr "\0" " " < /proc/$pid/cmdline | grep -q "bucket_live.py"; then kill -TERM "$pid"; fi; fi'
        ;;
    clips) "${SSH[@]}" "$HOST" 'cd ~/omni_ws/bucket_sessions 2>/dev/null && du -sh clip_* 2>/dev/null | tail -20' ;;
    fetch-clip)   # bucket.sh fetch-clip [clip_YYYYmmdd_HHMMSS]  (default: the newest) -> C:/Users/ferna/throw_data/bucket_clips/
        name=${2:-$("${SSH[@]}" "$HOST" 'ls -1d ~/omni_ws/bucket_sessions/clip_* 2>/dev/null | tail -1 | xargs -r basename')}
        [ -n "$name" ] || { echo 'No clips on the Pi'; exit 1; }
        dest=/c/Users/ferna/throw_data/bucket_clips; mkdir -p "$dest"
        "${SSH[@]}" "$HOST" "tar -C ~/omni_ws/bucket_sessions -cf - $name" | tar -C "$dest" -xf -
        echo "Fetched $dest/$name"
        ;;
    *) echo 'Use start, demo, deploy, status, stop, clips or fetch-clip'; exit 1 ;;
esac
