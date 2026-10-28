"""The robot's front door (design.md 2): serves the operator page and starts and stops the robot's software, so the page
is the launcher. Standard library only and no ROS: it runs before the stack, survives it crashing, and is what the
page talks to while nothing else is up. On the Jetson systemd starts it at boot (systemd/omni-console.service) and it
starts the robot at once; in the sim `omni.sh console` starts it in WSL (the Launcher's "Console" button).

  python3 -m omni_console.supervisor --steps sim|robot|<steps.json> [--autostart robot] [--port 8080]
                                     [--web <dir>] [--state-dir ~/.omni] [--pin 1234] [key:=value ...]

key:=value pairs are defaults for every step it starts (the Launcher's options). One step runs at a time (they share
the MCU port and the ROS graph); starting one stops the other first.

HTTP (plain, on the robot's own address: the page's ws:// to rosbridge works in every browser that way)
  GET  /, /js/..., ...       the page (web/)
  GET  /config.json          the ports the page connects to
  GET  /api/state            the steps, which one runs (or ran last, and how it ended), the robot computer
  GET  /api/log?n=200        the running (or last) step's log, the last n lines
  POST /api/start            {"step": "robot", "args": {"map": "house"}}
  POST /api/stop             {}
  POST /api/power            {"action": "shutdown" | "reboot"}  (when the steps file allows it: the Jetson)
  GET  /logs/, /logs/<file>  the session logs (session_log.py), for download
POST bodies carry "pin" when the supervisor has one (kid-proofing, not security: it's the home network).
"""

from __future__ import annotations

import argparse
import functools
import json
import os
import re
import shutil
import signal
import socket
import subprocess
import sys
import threading
import time
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, unquote, urlparse

ARG_VALUE = re.compile(r"^[A-Za-z0-9_.:/+-]{0,80}$")      # what a step argument may be (no shell is involved anyway)
ANSI = re.compile(r"\x1b\[[0-9;]*m")
STOP_GRACE = 20.0                                         # s after SIGINT before SIGTERM, then SIGKILL

# The sim (WSL): every step is `omni.sh <step>` (it syncs the repo, builds, then launches), web:=false because this
# process serves the page. Arguments: the launch arguments a person picks (stack.py).
VIEW_ARGS = [
    {"key": "viewer", "label": "MuJoCo 3D view", "kind": "bool", "default": "true"},
    {"key": "foxglove", "label": "Foxglove bridge", "kind": "bool", "default": "true"},
    {"key": "fast", "label": "Fast (~real time)", "kind": "bool", "default": "true"},
    {"key": "firmware", "label": "Firmware in the loop", "kind": "bool", "default": "true"},
]
SIM_STEPS = {
    "sim": True,
    "detect": ["pgrep", "-af", "ros2 launch omni_bringup"],
    "stop_all": ["bash", "{omni_sh}", "stop"],
    "power": None,
    "steps": [
        {"id": "robot", "label": "Robot (all modes)", "group": "robot", "cmd": ["bash", "{omni_sh}", "robot", "web:=false"],
         "hint": "Everything at once: Manual, Go To, Explore, Patrol, Catch, Follow, Program, Thrower and SLAM switch "
                 "from their tabs. The house world; in Catch a thrower steps up in front of the robot.",
         "args": [{"key": "map", "label": "map", "kind": "text", "default": "house"},
                  {"key": "kid", "label": "Kid profile", "kind": "bool", "default": "false"},
                  {"key": "person", "label": "A person walking (Follow)", "kind": "bool", "default": "false"},
                  {"key": "clutter", "label": "clutter (off, toys, all)", "kind": "text", "default": "off"},
                  {"key": "thrower", "label": "A thrower (Catch)", "kind": "bool", "default": "true"}]
         + VIEW_ARGS},
        {"id": "arm", "label": "Arm robot: tidy up", "group": "robot", "cmd": ["bash", "{omni_sh}", "arm", "web:=false"],
         "hint": "The arm robot (an SO-101 on the nose instead of the catch module) in the house with toys on the "
                 "floor: everything 'Robot' does but Catch and Thrower, plus the Tidy tab (find them, pick them up, "
                 "drop them in the bin; move the arm by hand).",
         "args": [{"key": "map", "label": "map", "kind": "text", "default": "house"},
                  {"key": "objects", "label": "things", "kind": "text", "default": "toys"},
                  {"key": "n_objects", "label": "how many", "kind": "number", "default": "6"},
                  {"key": "kid", "label": "Kid profile", "kind": "bool", "default": "false"},
                  {"key": "person", "label": "A person walking (Follow)", "kind": "bool", "default": "false"},
                  {"key": "clutter", "label": "clutter (off, toys, all)", "kind": "text", "default": "toys"}]
         + VIEW_ARGS},
        {"id": "catch", "label": "Play catch", "group": "robot", "cmd": ["bash", "{omni_sh}", "catch", "web:=false"],
         "hint": "The open field: a thrower 6-7 m away, the stereo cameras, the robot's own catch code; return throws "
                 "and the two-thrower pass game. (Catch in the house is a mode of 'Robot'.)",
         "args": [{"key": "dist", "label": "thrower m", "kind": "number", "default": "6.5"},
                  {"key": "returns", "label": "Throw back", "kind": "bool", "default": "true"},
                  {"key": "pass_game", "label": "Two throwers (pass game)", "kind": "bool", "default": "false"},
                  {"key": "kid", "label": "Kid profile", "kind": "bool", "default": "false"}] + VIEW_ARGS},
        {"id": "nav", "label": "Navigate (Nav2)", "group": "sim", "cmd": ["bash", "{omni_sh}", "nav", "web:=false"],
         "hint": "Localization on a saved map + Nav2 (no console-run SLAM).",
         "args": [{"key": "map", "label": "map", "kind": "text", "default": "house"}] + VIEW_ARGS},
        {"id": "map", "label": "Map the house", "group": "sim", "cmd": ["bash", "{omni_sh}", "map", "web:=false"],
         "hint": "slam_toolbox mapping alone; drive it from Manual. (The Robot step's SLAM tab does this too.)",
         "args": VIEW_ARGS},
        {"id": "sim", "label": "Drive (sim only)", "group": "sim", "cmd": ["bash", "{omni_sh}", "sim", "web:=false"],
         "hint": "The bridge and its wheel odometry; no localization.", "args": VIEW_ARGS},
        {"id": "catch-score", "label": "Catch score", "group": "scored",
         "cmd": ["bash", "{omni_sh}", "catch-score", "web:=false", "console:=false"], "ends": True,
         "hint": "Throws through the whole ROS stack with the firmware in the loop; writes ros_catch.json.",
         "args": [{"key": "tosses", "label": "throws", "kind": "number", "default": "24"},
                  {"key": "dist", "label": "thrower m", "kind": "number", "default": "6.5"}]},
        {"id": "goals", "label": "Nav2 goals (scored)", "group": "scored",
         "cmd": ["bash", "{omni_sh}", "goals", "web:=false"], "ends": True,
         "hint": "8 goals through the doors and onto the rugs; writes ros_nav_goals.json.", "args": []},
        {"id": "tour", "label": "Localization tour (scored)", "group": "scored",
         "cmd": ["bash", "{omni_sh}", "tour", "web:=false"], "ends": True,
         "hint": "slam_toolbox + EKF against the truth and v9's estimators.", "args": []},
        {"id": "auto-map", "label": "Auto-map (tour)", "group": "scored",
         "cmd": ["bash", "{omni_sh}", "auto-map", "web:=false"], "ends": True,
         "hint": "Maps the house along v9's tour and saves it.",
         "args": [{"key": "map", "label": "map", "kind": "text", "default": "house"}]},
    ],
}


def load_steps(spec: str, omni_sh: str) -> dict:
    if spec == "sim":
        cfg = json.loads(json.dumps(SIM_STEPS))
    else:
        path = Path(spec)
        if spec == "robot":                      # share/omni_console/config (installed), else the source tree
            path = Path(__file__).resolve().parents[1] / "config" / "robot_steps.json"
            try:
                from ament_index_python.packages import get_package_share_directory
                path = Path(get_package_share_directory("omni_console")) / "config" / "robot_steps.json"
            except Exception:                    # noqa: BLE001  (no ROS environment)
                pass
        cfg = json.loads(path.read_text())

    def fill(v):
        if isinstance(v, str):
            return v.replace("{omni_sh}", omni_sh)
        if isinstance(v, list):
            return [fill(x) for x in v]
        if isinstance(v, dict):
            return {k: fill(x) for k, x in v.items()}
        return v
    return fill(cfg)


def tail(path: Path, n: int) -> list:
    try:
        with open(path, "rb") as f:
            f.seek(0, 2)
            size = f.tell()
            f.seek(max(0, size - 200 * n))
            text = f.read().decode(errors="replace")
    except OSError:
        return []
    return [ANSI.sub("", line) for line in text.splitlines()[-n:]]


def system_info(state_dir: Path) -> dict:
    """The robot computer, for the System tab and the status bar while ROS is down."""
    out = dict(host=socket.gethostname(), cpus=os.cpu_count())
    try:
        out["load"] = round(os.getloadavg()[0], 2)
    except (OSError, AttributeError):
        out["load"] = None
    try:
        out["uptime_s"] = round(float(Path("/proc/uptime").read_text().split()[0]))
    except (OSError, ValueError, IndexError):
        out["uptime_s"] = None
    try:
        mem = {k: int(v.split()[0]) for k, v in (ln.split(":", 1) for ln in Path("/proc/meminfo").read_text().splitlines())}
        out["mem_used_pct"] = round(100 * (1 - mem["MemAvailable"] / mem["MemTotal"]))
    except (OSError, ValueError, KeyError):
        out["mem_used_pct"] = None
    try:
        du = shutil.disk_usage(state_dir)
        out["disk_free_gb"] = round(du.free / 1e9, 1)
    except OSError:
        out["disk_free_gb"] = None
    temps = []
    for z in Path("/sys/class/thermal").glob("thermal_zone*/temp"):
        try:
            temps.append(int(z.read_text()) / 1000.0)
        except (OSError, ValueError):
            pass
    out["temp_c"] = round(max(temps), 1) if temps else None
    return out


class Supervisor:
    def __init__(self, cfg: dict, state_dir: Path, defaults: dict, log=print):
        self.cfg = cfg
        self.steps = {s["id"]: s for s in cfg["steps"]}
        self.state_dir = state_dir
        self.log_dir = state_dir / "logs" / "steps"
        self.log_dir.mkdir(parents=True, exist_ok=True)
        self.defaults = defaults
        self.log = log
        self.lock = threading.RLock()
        self.proc = None
        self.cur = None              # dict(step, args, since, log, state: starting|running|stopping|exited, code)
        self.restarts = []           # times of automatic restarts (keep_running steps)
        self.stopping = False
        threading.Thread(target=self._watch, daemon=True).start()

    # -- steps ------------------------------------------------------------------------------------------------
    def args_for(self, step: dict, args: dict) -> dict:
        """The step's declared arguments: what the page sent, else the supervisor's defaults, else the step's."""
        out = {}
        for a in step.get("args", []):
            k = a["key"]
            v = args.get(k, self.defaults.get(k, a.get("default", "")))
            v = ("true" if v else "false") if isinstance(v, bool) else str(v)
            if not ARG_VALUE.match(v):
                raise ValueError(f"{a.get('label', k)}: letters, digits and . _ - only")
            out[k] = v
        return out

    def start(self, step_id: str, args: dict | None = None) -> str:
        step = self.steps.get(step_id)
        if step is None:
            raise ValueError(f"no step '{step_id}'")
        a = self.args_for(step, args or {})
        with self.lock:
            self.stop(wait=True)
            if self.external():
                self.log("a step started outside the console is running: stopping it")
                self._run_stop_all()
            cmd = list(step["cmd"]) + [f"{k}:={v}" for k, v in a.items() if v != ""]
            log_path = self.log_dir / f"{step_id}.log"
            if log_path.exists():                         # the run before stays readable (crash after a restart)
                os.replace(log_path, log_path.with_suffix(".prev.log"))
            out = open(log_path, "wb")
            out.write(f"== {time.strftime('%H:%M:%S')} {' '.join(cmd)}\n".encode())
            out.flush()
            self.proc = subprocess.Popen(cmd, stdout=out, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
                                         start_new_session=True, cwd=str(self.state_dir))
            out.close()
            self.cur = dict(step=step_id, label=step.get("label", step_id), args=a, since=time.time(),
                            log=str(log_path), state="running", code=None, pid=self.proc.pid)
            self.stopping = False
            self.log(f"started {step_id} (pid {self.proc.pid}): {' '.join(cmd)}")
            return f"Started '{step.get('label', step_id)}'"

    def stop(self, wait: bool = True) -> str:
        with self.lock:
            p = self.proc
            if p is None or p.poll() is not None:
                return "Nothing running"
            self.stopping = True
            self.cur["state"] = "stopping"
            try:
                os.killpg(p.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
            if not wait:
                threading.Thread(target=self._finish_stop, args=(p,), daemon=True).start()
                return "Stopping"
            self._finish_stop(p)
            return "Stopped"

    def _finish_stop(self, p):
        for sig, grace in ((None, STOP_GRACE), (signal.SIGTERM, 5.0), (signal.SIGKILL, 5.0)):
            if sig is not None:
                try:
                    os.killpg(p.pid, sig)
                except ProcessLookupError:
                    break
            try:
                p.wait(timeout=grace)
                break
            except subprocess.TimeoutExpired:
                continue

    def external(self) -> list:
        """Steps running that this supervisor didn't start (the Launcher's buttons, a terminal)."""
        det = self.cfg.get("detect")
        if not det:
            return []
        try:
            r = subprocess.run(det, capture_output=True, text=True, timeout=5)
        except (OSError, subprocess.TimeoutExpired):
            return []
        mine = self.proc.pid if self.proc is not None and self.proc.poll() is None else None
        out = []
        for line in r.stdout.splitlines():
            pid, _, cmd = line.partition(" ")
            if mine is not None and _descends(int(pid), mine):
                continue
            out.append(cmd.strip())
        return out

    def _run_stop_all(self):
        cmd = self.cfg.get("stop_all")
        if cmd:
            subprocess.run(cmd, capture_output=True, timeout=60)

    def stop_external(self) -> str:
        if not self.external():
            return "Nothing running outside the console"
        self._run_stop_all()
        return "Stopped the steps started outside the console"

    def _watch(self):
        """A step that ends: record how; a keep_running one (the robot) is started again, at most 3 times in 5 min."""
        while True:
            time.sleep(1.0)
            with self.lock:
                p, cur = self.proc, self.cur
                if p is None or cur is None or cur["state"] == "exited":
                    continue
                code = p.poll()
                if code is None:
                    continue
                cur.update(state="exited", code=code, ended=time.time())
                self.log(f"{cur['step']} ended (exit {code})")
                step = self.steps.get(cur["step"], {})
                if step.get("keep_running") and not self.stopping:
                    now = time.time()
                    self.restarts = [t for t in self.restarts if now - t < 300] + [now]
                    if len(self.restarts) <= 3:
                        self.log(f"{cur['step']} is meant to keep running: starting it again")
                        try:
                            self.start(cur["step"], cur["args"])
                        except Exception as e:                 # noqa: BLE001  (logged, the page shows it)
                            self.log(f"restart failed: {e}")

    def state(self) -> dict:
        with self.lock:
            cur = dict(self.cur) if self.cur else None
        if cur and cur["state"] in ("running", "stopping"):
            cur["uptime_s"] = round(time.time() - cur["since"])
        groups = {"robot": "The robot", "sim": "Sim steps", "scored": "Scored runs (they stop by themselves)"}
        return dict(steps=[{k: s.get(k) for k in ("id", "label", "group", "hint", "args", "ends")}
                           for s in self.cfg["steps"]], groups=groups, current=cur, external=self.external(),
                    defaults=self.defaults, power=sorted((self.cfg.get("power") or {}).keys()),
                    system=system_info(self.state_dir), sim=bool(self.cfg.get("sim")))

    def power(self, action: str) -> str:
        cmd = (self.cfg.get("power") or {}).get(action)
        if not cmd:
            raise ValueError(f"'{action}' isn't allowed here")
        self.stop(wait=True)
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
        if r.returncode != 0:
            raise ValueError(f"{action} failed: {(r.stderr or r.stdout).strip()[:200]}")
        return f"{action.title()} requested"


def _descends(pid: int, ancestor: int) -> bool:
    for _ in range(32):
        if pid == ancestor:
            return True
        try:
            stat = Path(f"/proc/{pid}/stat").read_text()
            pid = int(stat.rsplit(")", 1)[1].split()[1])
        except (OSError, ValueError, IndexError):
            return False
        if pid <= 1:
            return False
    return False


def serve_logs(h, u, logs_dir) -> bool:
    """GET /logs/ (the session logs with their summaries) and /logs/<name> (one log): the Logbook tab reads them
    straight from here, so a long session never goes through ROS. True if it answered."""
    from omni_console.session_log import listing
    if u.path.rstrip("/") == "/logs":
        body = json.dumps(listing(logs_dir) if logs_dir and logs_dir.is_dir() else []).encode()
        h.send_response(200)
        h.send_header("Content-Type", "application/json")
        h.send_header("Content-Length", str(len(body)))
        h.end_headers()
        h.wfile.write(body)
        return True
    if not u.path.startswith("/logs/"):
        return False
    name = unquote(u.path[len("/logs/"):])
    f = (logs_dir / name) if logs_dir else None
    if not f or "/" in name or "\\" in name or ".." in name or not name.endswith(".jsonl") or not f.is_file():
        h.send_error(404, "no such log")
        return True
    data = f.read_bytes()
    h.send_response(200)
    h.send_header("Content-Type", "application/x-ndjson")
    h.send_header("Content-Length", str(len(data)))
    h.end_headers()
    h.wfile.write(data)
    return True


class Handler(SimpleHTTPRequestHandler):
    sup: Supervisor = None
    config = b"{}"
    pin = ""
    logs_dir: Path = None
    extensions_map = {**SimpleHTTPRequestHandler.extensions_map, ".js": "text/javascript",
                      ".mjs": "text/javascript", ".css": "text/css", ".svg": "image/svg+xml",
                      ".jsonl": "application/x-ndjson"}

    def end_headers(self):
        self.send_header("Cache-Control", "no-cache")
        super().end_headers()

    def log_message(self, fmt, *args):
        pass

    def _json(self, code: int, obj):
        body = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        u = urlparse(self.path)
        if u.path == "/config.json":
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(self.config)))
            self.end_headers()
            self.wfile.write(self.config)
            return
        if u.path == "/api/state":
            return self._json(200, self.sup.state())
        if u.path == "/api/log":
            n = int(parse_qs(u.query).get("n", ["200"])[0])
            cur = self.sup.cur
            lines = tail(Path(cur["log"]), max(1, min(n, 2000))) if cur else []
            return self._json(200, dict(step=cur and cur["step"], lines=lines))
        if serve_logs(self, u, self.logs_dir):
            return
        super().do_GET()

    def do_POST(self):
        u = urlparse(self.path)
        try:
            n = int(self.headers.get("Content-Length") or 0)
            body = json.loads(self.rfile.read(n) or b"{}") if n else {}
            if not isinstance(body, dict):
                raise ValueError("a JSON object, please")
            if self.pin and str(body.get("pin", "")) != self.pin:
                return self._json(403, dict(ok=False, message="Wrong PIN", need_pin=True))
            if u.path == "/api/start":
                msg = self.sup.start(str(body.get("step", "")), body.get("args") or {})
            elif u.path == "/api/stop":
                msg = self.sup.stop_external() if body.get("external") else self.sup.stop(wait=False)
            elif u.path == "/api/power":
                msg = self.sup.power(str(body.get("action", "")))
            else:
                return self._json(404, dict(ok=False, message="no such call"))
        except (ValueError, OSError) as e:
            return self._json(400, dict(ok=False, message=str(e)))
        self._json(200, dict(ok=True, message=msg))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--steps", default="sim", help="sim, robot or a steps JSON file")
    ap.add_argument("--autostart", default="", help="a step to start right away")
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--rosbridge-port", type=int, default=9090)
    ap.add_argument("--foxglove-port", type=int, default=8765)
    ap.add_argument("--web", default="", help="the page's folder (default: the package's share/omni_console/web)")
    ap.add_argument("--state-dir", default="~/.omni")
    ap.add_argument("--omni-sh", default=os.environ.get("OMNI_SH", "omni.sh"))
    ap.add_argument("--pin", default=os.environ.get("OMNI_CONSOLE_PIN", ""))
    a, rest = ap.parse_known_args(argv)
    defaults = {}
    for kv in rest:
        k, sep, v = kv.partition(":=")
        if not sep:
            ap.error(f"unknown argument {kv}")
        defaults[k] = v
    state_dir = Path(os.path.expanduser(a.state_dir))
    state_dir.mkdir(parents=True, exist_ok=True)
    web = a.web
    if not web:
        try:
            from ament_index_python.packages import get_package_share_directory
            web = os.path.join(get_package_share_directory("omni_console"), "web")
        except Exception:                                     # noqa: BLE001  (no ROS environment: next to us)
            web = str(Path(__file__).resolve().parents[1] / "web")

    def log(msg):
        print(f"[supervisor {time.strftime('%H:%M:%S')}] {msg}", flush=True)

    sup = Supervisor(load_steps(a.steps, a.omni_sh), state_dir, defaults, log)
    cfg = dict(rosbridge_port=a.rosbridge_port, foxglove_port=a.foxglove_port, supervisor=True, pin=bool(a.pin))
    handler = type("SupervisorHandler", (Handler,), dict(sup=sup, config=json.dumps(cfg).encode(), pin=a.pin,
                                                          logs_dir=state_dir / "logs" / "sessions"))
    httpd = ThreadingHTTPServer(("0.0.0.0", a.port), functools.partial(handler, directory=web))
    httpd.daemon_threads = True
    log(f"operator console: http://{socket.gethostname()}:{a.port} (page from {web}); steps: {a.steps}")
    if a.autostart:
        try:
            log(sup.start(a.autostart))
        except ValueError as e:
            log(f"autostart: {e}")

    def bye(*_):
        log("shutting down: stopping the running step")
        threading.Thread(target=httpd.shutdown, daemon=True).start()
        sup.stop(wait=True)
    signal.signal(signal.SIGTERM, bye)
    signal.signal(signal.SIGINT, bye)
    try:
        httpd.serve_forever()
    finally:
        sup.stop(wait=True)


if __name__ == "__main__":
    sys.exit(main())
