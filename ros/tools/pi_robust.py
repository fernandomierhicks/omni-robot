"""Robustness checks of the robot computer's console (the Pi bench, rpi.md; omni_console/systemd/README.md).

  $py = C:\\Users\\ferna\\.venvs\\omni-robot-sim\\Scripts\\python.exe
  $py ros/tools/pi_robust.py cycles [-n 20] [--api]   start / stop the robot step N times: from the page (pi_console.py
                                                    serve must be running) or straight through the supervisor (--api);
                                                    after each stop: no step processes left, the MCU port free, 9090
                                                    closed; after each start: the MCU answers, rosbridge takes a socket
  $py ros/tools/pi_robust.py kill NAME [--signal 9] [--watch 40]   kill one piece and watch what follows: mcu_link,
                                                    mode_manager, rosbridge, rosapi, mission_node, launch (the step's
                                                    ros2 launch), supervisor (systemd restarts it), camera_snap
  $py ros/tools/pi_robust.py state                  the step, its processes, ports and the MCU port, once

Prints one line per check (PASS / FAIL) and a summary; results also as JSON (--out FILE). Stops nothing it didn't start
and kills only the one process it names (by pid). Run it only when the robot step is yours to stop (not someone's
bench) and the motors are disarmed.
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
import urllib.request

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import pi_console  # noqa: E402

HOST = os.environ.get("OMNI_PI", "omni-cam")
# the step's processes (their command lines on the Pi); the C++ nodes run as their binaries
PATTERNS = {
    "launch": "ros2 launch omni_bringup",
    "mcu_link": "lib/omni_cpp/mcu_link",
    "mode_manager": "lib/omni_cpp/mode_manager",
    "rosbridge": "rosbridge_websocket",
    "rosapi": "rosapi_node",
    "mission_node": "lib/omni_console/mission_node",
    "supervisor": "omni_console.supervisor",
    "camera_snap": "cam_snap/camera_snap.py",
}
STEP_PIECES = ["launch", "mcu_link", "mode_manager", "rosbridge", "rosapi", "mission_node"]

# one SSH round trip: the processes, ports, the MCU port's users, the supervisor's restarts (as JSON)
PROBE = r"""
python3 - <<'EOF'
import glob, json, os, subprocess
pats = %s
procs = {}
for d in glob.glob('/proc/[0-9]*'):
    try:
        argv = open(d + '/cmdline', 'rb').read().replace(b'\0', b' ').decode(errors='replace')
    except OSError:
        continue
    if argv.strip() == 'python3 -':          # this probe
        continue
    for k, p in pats.items():
        # not shells or the supervisor's own `pgrep -af "ros2 launch omni_bringup"` (each /api/state runs one)
        if p in argv and not argv.startswith(('bash -c', 'sh -c', '/bin/sh -c', 'pgrep', 'pkill')):
            procs.setdefault(k, []).append(int(d[6:]))
ss = subprocess.run(['ss', '-ltnH'], capture_output=True, text=True).stdout
ports = sorted({int(l.split()[3].rsplit(':', 1)[1]) for l in ss.splitlines() if l.strip()} & {8080, 8086, 8443, 9090})
mcu = sorted(glob.glob('/dev/serial/by-id/*Arduino*') + glob.glob('/dev/serial/by-id/*Teensy*'))
users = []
for m in mcu:
    real = os.path.realpath(m)
    for fd in glob.glob('/proc/[0-9]*/fd/*'):
        try:
            if os.path.realpath(fd) == real:
                users.append(int(fd.split('/')[2]))
        except OSError:
            pass
print(json.dumps(dict(procs=procs, ports=ports, mcu=mcu, mcu_users=sorted(set(users)),
                      sup_restarts=int(subprocess.run(['systemctl', 'show', '-p', 'NRestarts', '--value', 'omni-console'],
                                                      capture_output=True, text=True).stdout.strip() or 0))))
EOF
"""


def ssh(cmd: str, timeout: float = 30) -> str:
    r = subprocess.run(["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=10", HOST, cmd],
                       capture_output=True, text=True, timeout=timeout)
    return r.stdout


def probe() -> dict:
    out = ssh(PROBE % json.dumps(PATTERNS))
    try:
        return json.loads(out.strip().splitlines()[-1])
    except (ValueError, IndexError):
        return dict(error=out[-300:])


class Console:
    """The supervisor's API (and the page through pi_console, unless api_only)."""

    def __init__(self, base: str, api_only: bool):
        self.base, self.api_only = base.rstrip("/"), api_only

    def api(self, path: str, body: dict | None = None, timeout: float = 10):
        req = urllib.request.Request(self.base + path, data=None if body is None else json.dumps(body).encode(),
                                     headers={"Content-Type": "application/json"},
                                     method="GET" if body is None else "POST")
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return json.loads(r.read())

    def state(self) -> dict:
        try:
            return self.api("/api/state")
        except OSError as e:
            return dict(error=str(e))

    def step(self):
        return (self.state().get("current") or {})

    def page(self, op: str, **args):
        r = pi_console.call(op, args)
        if not r.get("ok"):
            raise RuntimeError(r.get("error"))
        return r["result"]

    def start(self):
        if self.api_only:
            return self.api("/api/start", {"step": "robot"})
        self.page("open", where="#system")
        cur = self.step()
        label = "Restart" if cur.get("state") == "running" else ("Start again" if cur else "Start")
        if label == "Start":
            return self.page("click", text="Start", exact=True, within=".steps")
        return self.page("click", text=label, exact=True)

    def stop(self):
        if self.api_only:
            return self.api("/api/stop", {})
        self.page("open", where="#system")
        return self.page("click", text="Stop", exact=True, within=".card")

    def log_has(self, text: str) -> bool:
        try:
            return any(text in ln for ln in self.api("/api/log?n=400")["lines"])
        except OSError:
            return False


def wait(pred, timeout: float, every: float = 0.5):
    t0 = time.time()
    while time.time() - t0 < timeout:
        v = pred()
        if v:
            return time.time() - t0
        time.sleep(every)
    return None


def rosbridge_up(host: str) -> bool:
    import socket
    try:
        socket.create_connection((host, 9090), timeout=2).close()
        return True
    except OSError:
        return False


def cycles(a, con: Console, host: str):
    rows, fails = [], 0
    for i in range(1, a.n + 1):
        row = dict(i=i)
        before = con.step().get("pid")
        t0 = time.time()
        con.start()
        row["started_s"] = wait(lambda: (lambda c: c.get("pid") not in (None, before) and c.get("state") == "running")(
            con.step()), 40, 0.2)                     # the new step (the old log stays readable until then)
        row["mcu_s"] = wait(lambda: con.log_has("MCU mode"), 40)
        row["rosbridge_s"] = wait(lambda: rosbridge_up(host), 40)
        if not con.api_only:          # the page follows: the status bar shows the MCU's mode
            row["page_s"] = wait(lambda: "DISARMED" in con.page("text", all=True), 30, 1.0)
        p = probe()
        row["running"] = sorted(k for k in STEP_PIECES if p["procs"].get(k))
        con.stop()
        row["stop_s"] = wait(lambda: con.step().get("state") == "exited", 60)
        row["code"] = con.step().get("code")
        time.sleep(1.0)
        p = probe()
        row["left"] = {k: v for k, v in p["procs"].items() if k in STEP_PIECES}
        row["port_9090"] = 9090 in p["ports"]
        row["mcu_users"] = p["mcu_users"]
        if not con.api_only:
            row["page_stopped"] = wait(lambda: "Robot software stopped" in con.page("text", all=True), 15, 1.0) is not None
        row["cycle_s"] = round(time.time() - t0, 1)
        ok = (row["started_s"] is not None and row["mcu_s"] is not None and row["rosbridge_s"] is not None and set(row["running"]) == set(STEP_PIECES)
              and row["stop_s"] is not None and not row["left"] and not row["port_9090"] and not row["mcu_users"]
              and row.get("page_s", 0) is not None and row.get("page_stopped", True))
        fails += not ok
        row["ok"] = ok
        rows.append(row)
        print(("PASS" if ok else "FAIL"), json.dumps(row), flush=True)
    print(f"== {a.n - fails} of {a.n} cycles clean")
    return rows


def kill(a, con: Console):
    p = probe()
    pids = p["procs"].get(a.name, [])
    if not pids:
        print(f"FAIL no {a.name} running")
        return dict(ok=False)
    before = dict(step=con.step(), sup_restarts=p["sup_restarts"], pids={k: v for k, v in p["procs"].items()})
    target = max(pids) if a.name != "launch" else min(pids)   # the binary, not its `ros2 run` wrapper
    print(f"== kill -{a.signal} {a.name} (pid {target}); step was {before['step'].get('state')} pid "
          f"{before['step'].get('pid')}")
    ssh(f"sudo kill -{a.signal} {target}" if a.name in ("supervisor",) else f"kill -{a.signal} {target}")
    t0 = time.time()
    seen = []
    while time.time() - t0 < a.watch:
        st = con.state()
        cur = st.get("current") or {}
        p = probe()
        line = dict(t=round(time.time() - t0, 1), step=cur.get("state"), pid=cur.get("pid"), code=cur.get("code"),
                    up=sorted(k for k in PATTERNS if p["procs"].get(k)), ports=p["ports"],
                    sup_restarts=p["sup_restarts"], err=st.get("error", "")[:60])
        if not seen or {k: v for k, v in line.items() if k != "t"} != {k: v for k, v in seen[-1].items() if k != "t"}:
            print(json.dumps(line), flush=True)
            seen.append(line)
        time.sleep(1.0)
    return dict(before=before, timeline=seen)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--url", default=os.environ.get("OMNI_CONSOLE_URL", ""))
    ap.add_argument("--out", default="")
    sub = ap.add_subparsers(dest="op", required=True)
    c = sub.add_parser("cycles")
    c.add_argument("-n", type=int, default=20)
    c.add_argument("--api", action="store_true", help="the supervisor's API instead of the page")
    k = sub.add_parser("kill")
    k.add_argument("name", choices=sorted(PATTERNS))
    k.add_argument("--signal", default="9")
    k.add_argument("--watch", type=float, default=40)
    sub.add_parser("state")
    a = ap.parse_args(argv)
    base = a.url or pi_console.pi_url()
    host = base.split("//")[1].split(":")[0]
    con = Console(base, getattr(a, "api", False) or a.op != "cycles")
    if a.op == "state":
        out = dict(step=con.step(), pi=probe())
        print(json.dumps(out, indent=1))
    elif a.op == "cycles":
        out = cycles(a, con, host)
    else:
        out = kill(a, con)
    if a.out:
        with open(a.out, "w") as f:
            json.dump(out, f, indent=1)


if __name__ == "__main__":
    sys.exit(main())
