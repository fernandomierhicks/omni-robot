#!/bin/bash
# one process per stage group (SolidWorks leaks memory over many opens); stop at the first failure
cd "$(dirname "$0")/../tools"
PY="C:/Users/ferna/cadenv/Scripts/python.exe"
for st in "$@"; do
  echo "##### $st" >> ../solidworks/sw_build.log
  "$PY" -u sw_robot.py $st >> ../solidworks/sw_build.log 2>&1 || { echo "FAILED $st" >> ../solidworks/sw_build.log; echo "exit 1" >> ../solidworks/sw_build.log; exit 1; }
done
echo "exit 0" >> ../solidworks/sw_build.log
