#!/bin/bash
# cad/v16: regenerate everything, in dependency order, one step at a time (parallel builds ran out of memory). Logs: logs/<step>.log.
#   ./regen_all.sh            all build123d steps (about 1 h 30)
#   ./regen_all.sh sw         then rebuild the SolidWorks assembly and its checks (SolidWorks open, attended)
#   FROM=upper ./regen_all.sh sw   resume: skip the steps before the named one (their outputs on disk are used)
# For an everyday change use ./quick.sh (outputs only, no checks; e.g. ./quick.sh pod sw), not this.
# A step that fails stops the chain. The arm and D435 caches (validation/d435/*.json|npz) are rebuilt only when missing: delete
# validation/d435/view_world.npz after a base change.
cd "$(dirname "$0")"
PY=/c/Users/ferna/.venvs/omni-robot-sim/Scripts/python.exe
SW=/c/Users/ferna/cadenv/Scripts/python.exe
mkdir -p logs
run() {   # run <log name> <interpreter> <script and args...>
  local name=$1 py=$2; shift 2
  if [ -n "$FROM" ]; then [ "$name" = "$FROM" ] && FROM= || { echo "$name -> skipped"; return 0; }; fi
  local s=$(date +%s)
  ( cd tools && $py -B "$@" ) > logs/$name.log 2>&1
  local rc=$?
  echo "$name -> exit $rc ($(( $(date +%s) - s )) s)"
  [ $rc -ne 0 ] && { tail -15 logs/$name.log; echo "CHAIN STOPPED"; exit $rc; }
}
run wheel_motor   $PY wheel_motor.py all       # the wheel and motor (Ø2.7 axle pilots)
run coupon_687    $PY coupon_687.py all
run coupon_627    $PY coupon_627.py all
run coupon_dt     $PY coupon_dovetail.py all   # Nov 15: the deck laps' dovetail fit (one tab, sockets 0.05 / 0)
run pod_mid       $PY pod_mid.py all           # the pod: checks, renders, STEPs, its footprints (validation/pod_mid/pod_fp.pkl)
run plunger       $PY plunger.py               # Nov 19: the M3 plunger's report and its droop / nominal / bump STEPs
run chassis_mid   $PY chassis_mid.py all       # the deck, fins, bumper, battery, stand pads (and the fins' lidar shadows)
run d435_motion   $PY d435_riser.py motion     # the arm's poses and moves (cached; upper's D435 check uses them)
run upper         $PY upper.py all             # the second floor and everything on it, its check against world.py
run d435_render   $PY d435_riser.py render
run arm_sweep     $PY arm_sweep.py             # the SO-101 swept against the robot, planned with the mid deck's ring
run arm_sweep_rr  $PY arm_sweep.py robot_ring  # ... and planned with the robot's own ring (omni_arm robot_arm.json)
run harness_route $PY harness_route.py         # the leads' routes on the mid deck
run harness       $PY harness.py all           # the leads drawn, checks, harness_review.step
run harness_table $PY harness_table.py        # Nov 18: the leads' table (from, to, through, length)
run electrical    $PY electrical_step.py      # Nov 18: the electrics + harness review STEP
# service.py (driver access to every screw, pods and battery dropping out) is no longer in the chain (Fernando, Nov 19: he checks
# screw access visually; it took ~2 h). Run it by hand only when asked.
run robot_step    $PY robot_step.py            # step_export/robot_assembly.step
run print_files   $PY print_files.py           # print_files/First floor (colour folders; the tracker is kept)
echo "BUILD123D DONE"
[ "$1" = "sw" ] || exit 0
run sw_parts      $PY sw_robot_parts.py        # the part STEPs and the manifest
n=0
for stages in "import" "roller wheel motor unsprung static plunger" "pod" "group_frame group_mounts group_bumper group_battery group_reference" \
              "group_upper" "group_electronics" "group_arm" "group_fasteners" "group_harness" "robot" "test paths" \
              "interference_base" "interference_harness" "views_close"; do
  n=$((n + 1))
  run "sw_$n" $SW sw_robot.py $stages
done
echo "SOLIDWORKS DONE"
