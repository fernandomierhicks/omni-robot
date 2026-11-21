#!/bin/bash
# cad/v16: regenerate only what a change reaches, no checks (Fernando, Nov 19: "ridiculous we have to spend 30 min every time we make a minor
# change"). regen_all.sh stays the full rebuild + checks; this is the everyday path.
#   ./quick.sh pod        a pod part changed (towers, hangers, beam, forks, plungers, springs, wheel ...):
#                         pod STEPs + print files, the robot STEP (base and upper from robot_step's cache), the pod's SolidWorks part STEPs
#   ./quick.sh pod sw     ... then SolidWorks: re-import only the parts whose STEP changed, rebuild the static sub-assemblies and re-mate
#                         the pods (the robot assembly picks the pods up when opened; SolidWorks open, attended)
# Needs one full run of tools/robot_step.py first (it fills cache/robot_step); rerun that after a base or upper change.
cd "$(dirname "$0")"
PY=/c/Users/ferna/.venvs/omni-robot-sim/Scripts/python.exe
SW=/c/Users/ferna/cadenv/Scripts/python.exe
mkdir -p logs
run() {   # run <log name> <interpreter> <script and args...>
  local name=$1 py=$2; shift 2
  local s=$(date +%s)
  ( cd tools && $py -B "$@" ) > logs/quick_$name.log 2>&1
  local rc=$?
  echo "$name -> exit $rc ($(( $(date +%s) - s )) s)"
  [ $rc -ne 0 ] && { tail -15 logs/quick_$name.log; echo "STOPPED"; exit $rc; }
}
case "$1" in
  pod)
    run pod_export   $PY pod_mid.py export          # step_export/pod_mid: the pod's print STEPs and its assembly STEP
    run print_files  $PY print_files.py             # print_files/ (the tracker kept)
    run robot_step   $PY robot_step.py pods         # step_export/robot_assembly.step
    run sw_parts     $PY sw_robot_parts.py pod      # solidworks/step_parts: the pod's parts (unchanged files keep their mtime)
    if [ "$2" = "sw" ]; then
      run sw_pod     $SW sw_robot.py import plunger static pod   # plunger: rebuild each plunger's sub-assembly (Nov 20: its parts change with plunger.KIND)
    fi
    ;;
  *)
    echo "usage: ./quick.sh pod [sw]"; exit 2 ;;
esac
echo "QUICK DONE"
