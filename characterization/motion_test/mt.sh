#!/usr/bin/env bash
# Camera-motion test (README.md). From Git Bash, in the repo:
#
#   bash characterization/motion_test/mt.sh record [ARGS]   # Pi: stops the bucket lab, records (ARGS -> motion_record.py)
#   bash characterization/motion_test/mt.sh fetch [NAME]    # Pi -> C:/Users/ferna/throw_data/motion_test/ (the newest, or one)
#   bash characterization/motion_test/mt.sh analyze [DIR] [ARGS]   # PC: motion_analyze.py (the newest fetched)
#   bash characterization/motion_test/mt.sh list | test
set -euo pipefail
HOST=${OMNI_PI:-omni-cam}
REPO=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
HERE="$REPO/characterization/motion_test"
DATA=${MT_DATA:-/c/Users/ferna/throw_data/motion_test}
SSH=(ssh -o BatchMode=yes -o ConnectTimeout=8)
RSPY=${OMNI_RS_PY:-/c/Users/ferna/.venvs/realsense/Scripts/python.exe}

case "${1:-}" in
    record)
        shift
        bash "$REPO/characterization/ball_view/bucket.sh" stop || true
        for i in 1 2 3 4 5 6; do
            "${SSH[@]}" "$HOST" 'pgrep -f "python -u bucket_live.py" >/dev/null' || break; sleep 2
        done
        tar -C "$HERE" -cf - motion_record.py | "${SSH[@]}" "$HOST" 'mkdir -p ~/omni_ws/motion_test && tar -C ~/omni_ws/motion_test -xf -'
        "${SSH[@]}" "$HOST" "cd ~/omni_ws/motion_test && ~/.venvs/cam/bin/python -u motion_record.py $*"
        ;;
    fetch)
        name=${2:-$("${SSH[@]}" "$HOST" 'ls -1d ~/omni_ws/motion_test/2*_* 2>/dev/null | tail -1 | xargs -r basename')}
        [ -n "$name" ] || { echo 'No recordings on the Pi'; exit 1; }
        mkdir -p "$DATA"
        "${SSH[@]}" "$HOST" "tar -C ~/omni_ws/motion_test -cf - $name" | tar -C "$DATA" -xf -
        echo "Fetched $DATA/$name"
        ;;
    analyze)
        d=${2:-$(ls -1d "$DATA"/2*_* | tail -1)}; shift 2 2>/dev/null || shift $#
        cd "$HERE" && "$RSPY" motion_analyze.py "$d" "$@"
        ;;
    list) "${SSH[@]}" "$HOST" 'cd ~/omni_ws/motion_test 2>/dev/null && du -sh 2*_* 2>/dev/null' ;;
    test) cd "$HERE" && "$RSPY" -m pytest -q ;;
    *) sed -n '2,8p' "${BASH_SOURCE[0]}"; exit 1 ;;
esac
