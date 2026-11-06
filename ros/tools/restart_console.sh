#!/usr/bin/env bash
# The console's mission node stopped and started again in place (ai_smoke.py restart: ai_robot_architecture.md 6, "after a
# node restart, old tasks become INTERRUPTED and nothing resumes"): the same command line, environment and parameter
# file as the launch gave it, detached so it outlives this call. Run in WSL; $1 = a pattern for the workspace whose node
# to restart (omni_ws_claude: a test stack's; never Fernando's own without asking).
# The new node and its slam_toolbox belong to no launch: stopping the launch leaves them, so stop that stack with
# `omni.sh stop` or kill its domain's mission_node and slam_toolbox too (a next console reaps a leftover slam_toolbox,
# but under the sim's clock its stamps poison the new session's TF buffers first: design.md 4.6).
set -eo pipefail
WS="${1:?workspace pattern, e.g. omni_ws_claude}"
pid=$(pgrep -f "$WS/install/omni_console/lib/omni_console/mission_node" | head -1)
[ -n "$pid" ] || { echo "no mission_node from $WS"; exit 1; }
cmd=$(tr '\0' ' ' < "/proc/$pid/cmdline")
envf=$(mktemp)
cp "/proc/$pid/environ" "$envf"
kill -INT "$pid"                                       # its shutdown stops its slam_toolbox child too
for i in $(seq 1 30); do kill -0 "$pid" 2>/dev/null || break; sleep 0.5; done
kill -9 "$pid" 2>/dev/null || true
echo "stopped mission_node $pid"
setsid -f bash -c 'while IFS= read -r -d "" kv; do export "$kv"; done < "$0"; exec $1 > "$HOME/omni_console_restart.log" 2>&1' \
    "$envf" "$cmd"
echo "started it again: $cmd"
