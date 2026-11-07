"""The supervisor (page server + launcher) with a stand-in step. Linux only (process groups, /proc): in WSL,
  python3 -m pytest -q ros/src/omni_console/test/test_supervisor.py
"""

import json
import os
import signal
import sys
import subprocess
import threading
import time
import urllib.request
from functools import partial
from http.server import ThreadingHTTPServer

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

pytestmark = pytest.mark.skipif(sys.platform == "win32", reason="process groups and signals: Linux (WSL, the robot computer)")

# a step that prints its arguments, then runs until SIGINT, which it answers like ros2 launch: a clean exit
STEP = ("import signal, sys, time\n"
        "print('args', ' '.join(sys.argv[1:]), flush=True)\n"
        "signal.signal(signal.SIGINT, lambda *a: (print('bye', flush=True), sys.exit(0)))\n"
        "while True: time.sleep(0.05)\n")


def config(tmp_path):
    return {"sim": True, "detect": None, "power": None, "steps": [
        {"id": "a", "label": "Step A", "group": "robot", "cmd": [sys.executable, "-c", STEP, "x"],
         "args": [{"key": "map", "kind": "text", "default": "house"}, {"key": "kid", "kind": "bool", "default": "false"}]},
        {"id": "b", "label": "Step B", "group": "sim", "cmd": [sys.executable, "-c", "print('done')"], "ends": True,
         "args": []},
        {"id": "keep", "label": "Keeps running", "group": "robot", "keep_running": True,
         "cmd": [sys.executable, "-c", "import time; time.sleep(0.2)"], "args": []}]}


def wait(pred, timeout=8.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        if pred():
            return True
        time.sleep(0.05)
    return False


def test_start_args_stop_and_one_step_at_a_time(tmp_path):
    from omni_console.supervisor import Supervisor, tail
    sup = Supervisor(config(tmp_path), tmp_path, {"kid": "true"}, log=lambda m: None)
    sup.start("a", {"map": "flat"})
    log = tmp_path / "logs" / "steps" / "a.log"
    assert wait(lambda: any(ln.startswith("args") for ln in tail(log, 10)))
    assert "args x map:=flat kid:=true" in tail(log, 10)                  # page's value, else the defaults
    with pytest.raises(ValueError):
        sup.start("a", {"map": "evil; rm -rf /"})
    with pytest.raises(ValueError):
        sup.start("nope")
    pid_a = sup.proc.pid
    sup.start("b")                                                         # stops A first, cleanly (SIGINT)
    assert "bye" in "\n".join(tail(log, 10))
    with pytest.raises(OSError):
        os.kill(pid_a, 0)
    assert wait(lambda: sup.state()["current"]["state"] == "exited")
    st = sup.state()
    assert st["current"]["step"] == "b" and st["current"]["code"] == 0
    assert [s["id"] for s in st["steps"]] == ["a", "b", "keep"] and st["sim"]
    assert sup.stop() == "Nothing running"


def test_a_keep_running_step_is_restarted_three_times_at_most(tmp_path):
    from omni_console.supervisor import Supervisor
    sup = Supervisor(config(tmp_path), tmp_path, {}, log=lambda m: None)
    sup.start("keep")
    assert wait(lambda: len(sup.restarts) == 3, 12.0)
    assert wait(lambda: sup.state()["current"]["state"] == "exited", 6.0)
    time.sleep(1.5)
    assert len(sup.restarts) == 4 and sup.state()["current"]["state"] == "exited"    # the 4th try isn't made


def test_switch_from_an_external_launch_preserves_the_console_and_other_processes(tmp_path, monkeypatch):
    from omni_console import supervisor as module
    monkeypatch.setattr(module, "STOP_GRACE", 0.3)
    code = ("import subprocess, signal, sys, time\n"
            "p=subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(120)'])\n"
            "print(p.pid, flush=True)\n"
            "signal.signal(signal.SIGINT, lambda *a: sys.exit(0))\n"
            "while True: time.sleep(.05)\n")
    launch = subprocess.Popen([sys.executable, "-u", "-c", code, "ros2", "launch", "omni_bringup"],
                              stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
    unrelated = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(120)"])
    child = int(launch.stdout.readline())
    cfg = config(tmp_path)
    # Mimic pgrep also matching the supervisor/test shell's command text. It must inspect argv before stopping.
    cfg["detect"] = [sys.executable, "-c", f"print('{launch.pid} ros2 launch omni_bringup arm.launch.py'); "
                     f"print('{os.getpid()} shell mentioning ros2 launch omni_bringup'); print('invalid output')"]
    cfg["stop_all"] = [sys.executable, "-c", "raise RuntimeError('broad stop must never run')"]
    sup = module.Supervisor(cfg, tmp_path, {}, log=lambda m: None)
    try:
        assert sup.external() == ["ros2 launch omni_bringup arm.launch.py"]
        sup.start("a")
        assert launch.wait(timeout=3) == 0
        assert wait(lambda: module._process_identity(child) is None)
        assert unrelated.poll() is None
        assert sup.proc.poll() is None
    finally:
        sup.stop()
        for p in (launch, unrelated):
            if p.poll() is None:
                p.terminate()
            p.wait(timeout=3)
        if module._process_identity(child) is not None:
            os.kill(child, signal.SIGTERM)


def test_stopping_an_owned_step_leaves_no_separately_grouped_child(tmp_path, monkeypatch):
    from omni_console import supervisor as module
    monkeypatch.setattr(module, "STOP_GRACE", 0.3)
    cfg = config(tmp_path)
    code = ("import subprocess, signal, sys, time\n"
            "p=subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(120)'], start_new_session=True)\n"
            "print('child', p.pid, flush=True)\n"
            "signal.signal(signal.SIGINT, lambda *a: sys.exit(0))\n"
            "while True: time.sleep(.05)\n")
    cfg["steps"][0]["cmd"] = [sys.executable, "-u", "-c", code]
    sup = module.Supervisor(cfg, tmp_path, {}, log=lambda m: None)
    child = None
    try:
        sup.start("a")
        log = tmp_path / "logs/steps/a.log"
        assert wait(lambda: "child " in log.read_text())
        child = int(next(s.split()[1] for s in log.read_text().splitlines() if s.startswith("child ")))
        sup.stop()
        assert module._process_identity(child) is None
        assert sup.proc.returncode == 0
    finally:
        sup.stop()
        if child is not None and module._process_identity(child) is not None:
            os.kill(child, signal.SIGTERM)


def test_http_api_and_the_page(tmp_path):
    from omni_console.supervisor import Handler, Supervisor
    web = tmp_path / "web"
    web.mkdir()
    (web / "index.html").write_text("<p>console</p>")
    sessions = tmp_path / "logs" / "sessions"
    sessions.mkdir(parents=True)
    (sessions / "2026-10-22_1200_catch.jsonl").write_text('{"type": "session"}\n')
    sup = Supervisor(config(tmp_path), tmp_path, {}, log=lambda m: None)
    h = type("H", (Handler,), dict(sup=sup, config=b'{"supervisor": true}', pin="42", logs_dir=sessions))
    srv = ThreadingHTTPServer(("127.0.0.1", 0), partial(h, directory=str(web)))
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    base = f"http://127.0.0.1:{srv.server_address[1]}"

    def post(path, body):
        req = urllib.request.Request(base + path, json.dumps(body).encode(), {"Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(req) as r:
                return r.status, json.loads(r.read())
        except urllib.error.HTTPError as e:
            return e.code, json.loads(e.read())

    try:
        assert b"console" in urllib.request.urlopen(base + "/").read()
        assert json.loads(urllib.request.urlopen(base + "/config.json").read())["supervisor"]
        assert post("/api/start", {"step": "a"})[0] == 403                 # the PIN
        code, r = post("/api/start", {"step": "a", "pin": "42"})
        assert code == 200 and r["ok"]
        st = json.loads(urllib.request.urlopen(base + "/api/state").read())
        assert st["current"]["step"] == "a" and st["system"]["cpus"] >= 1
        assert wait(lambda: json.loads(urllib.request.urlopen(base + "/api/log?n=5").read())["lines"])
        assert post("/api/power", {"action": "shutdown", "pin": "42"})[0] == 400      # not allowed in the sim
        assert post("/api/stop", {"pin": "42"})[1]["ok"]
        assert wait(lambda: sup.state()["current"]["state"] == "exited")
        logs = json.loads(urllib.request.urlopen(base + "/logs/").read())
        assert logs[0]["name"] == "2026-10-22_1200_catch.jsonl"
        assert urllib.request.urlopen(base + "/logs/" + logs[0]["name"]).read().startswith(b'{"type"')
        with pytest.raises(urllib.error.HTTPError):
            urllib.request.urlopen(base + "/logs/..%2Fsteps%2Fa.log")
    finally:
        srv.shutdown()
        sup.stop()


def test_the_sim_and_robot_step_files_load():
    from omni_console.supervisor import load_steps
    sim = load_steps("sim", "/repo/ros/omni.sh")
    robot = next(s for s in sim["steps"] if s["id"] == "robot")
    assert robot["cmd"][:3] == ["bash", "/repo/ros/omni.sh", "robot"] and "web:=false" in robot["cmd"]
    real = load_steps("robot", "")
    assert real["steps"][0]["keep_running"] and set(real["power"]) == {"shutdown", "reboot"}
