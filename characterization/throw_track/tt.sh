#!/usr/bin/env bash
# Throw tracking with the real cameras (design.md 8, phase 1C). From Git Bash, in the repo:
#
#   bash characterization/throw_track/tt.sh record [ARGS]   # on the Pi: record throws (asks y/n after each); ARGS -> record_throws.py
#   bash characterization/throw_track/tt.sh synthetic       # the same with rendered throws, no camera (a dry run on the Pi)
#   bash characterization/throw_track/tt.sh fetch [SESSION] # copy sessions Pi -> characterization/throw_track/data/ (the latest, or one)
#   bash characterization/throw_track/tt.sh analyze [DIR]   # analyze_throws.py on the PC (the latest fetched session)
#   bash characterization/throw_track/tt.sh tag [DIR]       # basket tag pose from the session's tag.npy (needs OpenCV: the realsense venv)
#   bash characterization/throw_track/tt.sh test            # this folder's tests (synthetic throws, no camera)
#   bash characterization/throw_track/tt.sh deploy | list   # just copy the code to the Pi | the Pi's sessions
#
# RealSense D435 on the Pi (every imager: colour, depth, IR pair; record_rs.py, README.md "RealSense"):
#   bash characterization/throw_track/tt.sh rscheck [SEC]    # no throws: each stream's rate, lost frames, clock offsets (first thing to run)
#   bash characterization/throw_track/tt.sh rsautotune       # the ball still in view: white balance, exposure, gain, colour window -> realsense_auto.json
#   bash characterization/throw_track/tt.sh rsrecord [ARGS]  # record throws, no labels (ARGS -> record_rs.py: --name --distance --count ...)
#   bash characterization/throw_track/tt.sh rssynthetic      # the same code with rendered throws, no camera
#   bash characterization/throw_track/tt.sh rsfetch [SESSION]       # Pi -> ~/throw_data (outside Dropbox: ~400 MB a throw; TT_RS_DATA changes it)
#   bash characterization/throw_track/tt.sh rsanalyze [DIR] [ARGS]  # replay on the PC: detectors on every imager, arcs, agreement (rs_analyze.py)
#   bash characterization/throw_track/tt.sh rsview N [DIR]   # contact sheet of throw N (rs_view.py; --scrub for a window)
#   bash characterization/throw_track/tt.sh rsreview [DIR]   # the review page: video + vision overlay, your in / rim / out call (rs_review.py)
#
# record deploys the detector (ros/src/omni_catch) and this folder to ~/omni_ws first and runs in the foreground (a
# terminal: you type y / n after each throw). Only one process can hold the camera: stop `bv.sh` or the head first.
# Tune the ball's colour in `bv.sh picam` first and press "save settings": record uses those.
set -euo pipefail

HOST=${OMNI_PI:-omni-cam}
REPO=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
HERE="$REPO/characterization/throw_track"
SSH=(ssh -o BatchMode=yes)
PY=${OMNI_SIM_PY:-/c/Users/ferna/.venvs/omni-robot-sim/Scripts/python.exe}
RSPY=${OMNI_RS_PY:-/c/Users/ferna/.venvs/realsense/Scripts/python.exe}

deploy() {
    # rs_common.py (the D435 helpers) goes to ~/omni_ws/realsense/ where rs_pi.sh puts it: record_rs.py imports it from there
    tar -C "$REPO" -cf - --exclude='__pycache__' --exclude='.pytest_cache' --exclude='build' --exclude='data' --exclude='results' \
        ros/src/omni_catch/omni_catch characterization/throw_track characterization/realsense/rs_common.py \
    | "${SSH[@]}" "$HOST" "set -e; rm -rf ~/.tt_stage; mkdir -p ~/.tt_stage ~/omni_ws/src/omni_catch ~/omni_ws/throw_track ~/omni_ws/realsense; \
        tar -C ~/.tt_stage -xf -; \
        rsync -a ~/.tt_stage/ros/src/omni_catch/omni_catch/ ~/omni_ws/src/omni_catch/omni_catch/; \
        rsync -a ~/.tt_stage/characterization/realsense/rs_common.py ~/omni_ws/realsense/rs_common.py; \
        rsync -a --delete --exclude data/ --exclude results/ ~/.tt_stage/characterization/throw_track/ ~/omni_ws/throw_track/; rm -rf ~/.tt_stage"
}

RSDATA=${TT_RS_DATA:-$HOME/throw_data}
latest_local() { ls -dt "$HERE"/data/*/ 2>/dev/null | head -1 | sed 's#/$##'; }
latest_rs() { ls -dt "$RSDATA"/*/ 2>/dev/null | head -1 | sed 's#/$##'; }

case "${1:-}" in
    record|synthetic)
        mode=$1; shift
        deploy
        extra=(); [ "$mode" = synthetic ] && extra=(--synthetic)
        tty=(); [ -t 0 ] && tty=(-t)
        "${SSH[@]}" "${tty[@]}" "$HOST" "cd ~/omni_ws/throw_track && exec ~/.venvs/cam/bin/python record_throws.py ${extra[*]:-} $*"
        ;;
    rsautotune)
        # the ball still in view (a table, near the centre): white balance, exposure, gain and colour window from the ball itself
        shift; deploy
        "${SSH[@]}" "$HOST" "cd ~/omni_ws/throw_track && exec ~/.venvs/rs/bin/python rs_autotune.py $*"
        ;;
    rscheck|rsrecord|rssynthetic)
        mode=$1; shift
        deploy
        case $mode in
            rscheck) extra=(--check "${1:-8}"); [ $# -gt 0 ] && shift ;;
            rssynthetic) extra=(--synthetic) ;;
            *) extra=() ;;
        esac
        tty=(); [ -t 0 ] && tty=(-t)
        # the D435 library lives in the rs venv (rs_pi.sh setup); one process holds the camera: stop ball_view / the head first
        "${SSH[@]}" "${tty[@]}" "$HOST" "cd ~/omni_ws/throw_track && exec ~/.venvs/rs/bin/python record_rs.py ${extra[*]:-} $*"
        ;;
    rsfetch)
        # RealSense sessions are ~400 MB a throw: they go outside the Dropbox folder (TT_RS_DATA, default ~/throw_data)
        mkdir -p "$RSDATA"
        name=${2:-$("${SSH[@]}" "$HOST" 'ls -t ~/omni_ws/throw_track/data | head -1')}
        echo "== $HOST:omni_ws/throw_track/data/$name -> $RSDATA/"
        scp -q -r -o BatchMode=yes "$HOST:omni_ws/throw_track/data/$name" "$RSDATA/"
        echo "$RSDATA/$name"
        ;;
    rsanalyze) d=${2:-$(latest_rs)}; shift 2>/dev/null || true; shift 2>/dev/null || true; cd "$HERE" && "$PY" rs_analyze.py "$d" "$@" ;;
    rsview) n=${2:?throw number}; d=${3:-$(latest_rs)}; shift 3 2>/dev/null || true; cd "$HERE" && "$PY" rs_view.py "$d" "$n" "$@" ;;
    rsreview) d=${2:-$(latest_rs)}; shift 2>/dev/null || true; shift 2>/dev/null || true; cd "$HERE" && "$PY" rs_review.py "$d" "$@" ;;
    fetch)
        mkdir -p "$HERE/data"
        name=${2:-$("${SSH[@]}" "$HOST" 'ls -t ~/omni_ws/throw_track/data | head -1')}
        echo "== $HOST:omni_ws/throw_track/data/$name -> $HERE/data/"
        scp -q -r -o BatchMode=yes "$HOST:omni_ws/throw_track/data/$name" "$HERE/data/"
        echo "$HERE/data/$name"
        ;;
    list) "${SSH[@]}" "$HOST" 'cd ~/omni_ws/throw_track/data 2>/dev/null && for d in *; do echo "$d  $(ls -d $d/t* 2>/dev/null | wc -l) throws  $(du -sh $d | cut -f1)"; done' ;;
    analyze) d=${2:-$(latest_local)}; shift 2>/dev/null || true; shift 2>/dev/null || true; cd "$HERE" && "$PY" analyze_throws.py "$d" "$@" ;;
    tag) d=${2:-$(latest_local)}; cd "$HERE" && "$RSPY" tag_pose.py "$d" ;;
    deploy) deploy ;;
    test) cd "$HERE" && "$PY" -m pytest -q ;;
    *) sed -n '2,25p' "${BASH_SOURCE[0]}"; exit 1 ;;
esac
