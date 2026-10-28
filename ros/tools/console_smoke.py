"""Smoke test of the operator console (omni_console) in a real browser against a running stack.

  & $py ros\\tools\\console_smoke.py                      (after `omni.sh robot` or `nav`: http://localhost:8080)
  & $py ros\\tools\\console_smoke.py --shots <folder>      also screenshots of every tab
  & $py ros\\tools\\console_smoke.py --slam                also a new map + Explore (robot step; ~2 min), then back
  & $py ros\\tools\\console_smoke.py --tabs --faults --catch --cpu   the round-3 tabs, fault injection, the catch world
                                                          through the supervisor (omni.sh console ... person:=true)

Drives Microsoft Edge (installed with Windows) through Playwright (pip install playwright; no browser download).
Checks: the page connects and the status updates at ~10 Hz; Take control + W drives the robot and letting go stops it;
closing the page while driving stops it; a Go To goal from a saved place arrives; a setting round-trips; Space stops
and disarms; the ROS tab lists topics. The subsystem tabs: the firmware the MCU reports matches this checkout's
build id, the RC receiver is reported, the Base tab's 200 Hz samples arrive, the Thrower bench mode spins the
flywheels up and fires once; a two-point Patrol runs. Prints the console's Wi-Fi traffic and the page's frame rate.
Leaves the robot in IDLE and deletes the place and the route it saved.
"""

from __future__ import annotations

import argparse
import math
import sys
import time

from playwright.sync_api import sync_playwright

WSL = r"C:\Windows\System32\wsl.exe"
CPU_SAMPLER = "/mnt/c/Dropbox/Personal Projects/Robotics/omni-robot/ros/tools/cpu_sample.py"
CONSOLE_CPU_BUDGET = 7.0          # % of one WSL core for the console's processes with a page open: ~5-6 measured,
                                  # ~12-13 before the page did the work (design.md 2)

TRUE_SPEED_JS = """() => new Promise(res => {
  // the sim's true speed (its /ground_truth/odom: the robot's pose estimate can drift when the MCU is silent)
  let v = null;
  const t = new ROSLIB.Topic({ ros: window.omniConsole.getRos(), name: '/ground_truth/odom',
                               messageType: 'nav_msgs/msg/Odometry' });
  t.subscribe(m => { v = [Math.hypot(m.twist.twist.linear.x, m.twist.twist.linear.y), Math.abs(m.twist.twist.angular.z)]; });
  setTimeout(() => { t.unsubscribe(); res(v); }, 600); })"""


def robot_cpu(seconds: float) -> dict:
    """% of one core per process group on the robot side (cpu_sample.py in WSL)."""
    import json
    import subprocess
    r = subprocess.run([WSL, "-d", "Ubuntu-24.04", "--", "python3", CPU_SAMPLER, str(seconds), "--json"],
                       capture_output=True, text=True)
    try:
        return json.loads(r.stdout.strip().splitlines()[-1])
    except (ValueError, IndexError):
        return {}

FREE_GOAL_JS = """() => {
  // a free cell 1-2.5 m from the robot with 0.35 m of free space around it (a goal Nav2 can reach)
  const s = window.omniConsole.store, m = s.map, p = s.status && s.status.pose;
  if (!m || !p) return null;
  const i = m.info, r = i.resolution, W = i.width, H = i.height, ox = i.origin.position.x, oy = i.origin.position.y;
  const k = Math.ceil(0.35 / r);
  const free = (cx, cy) => {
    for (let dy = -k; dy <= k; dy++) for (let dx = -k; dx <= k; dx++) {
      const x = cx + dx, y = cy + dy;
      if (x < 0 || y < 0 || x >= W || y >= H) return false;
      const v = m.data[y * W + x]; if (v !== 0) return false;
    }
    return true;
  };
  // from the side first: straight ahead of the start is where the sim's person walks (person:=true)
  for (let d = 2.5; d >= 1.0; d -= 0.25) for (let a = Math.PI / 2; a < 2.5 * Math.PI; a += Math.PI / 12) {
    const x = p.x + d * Math.cos(p.yaw + a), y = p.y + d * Math.sin(p.yaw + a);
    if (free(Math.floor((x - ox) / r), Math.floor((y - oy) / r))) return { x, y, d };
  }
  return null;
}"""


FREE_POINT_JS = """(d) => {
  // a free spot d m from the robot (any direction), as a route point [x, y, yaw]
  const s = window.omniConsole.store, m = s.map, p = s.status && s.status.pose;
  if (!m || !p) return null;
  const i = m.info, r = i.resolution, W = i.width, H = i.height, ox = i.origin.position.x, oy = i.origin.position.y;
  const k = Math.ceil(0.4 / r);
  const free = (cx, cy) => { for (let dy = -k; dy <= k; dy++) for (let dx = -k; dx <= k; dx++) {
    const x = cx + dx, y = cy + dy; if (x < 0 || y < 0 || x >= W || y >= H || m.data[y * W + x] !== 0) return false; }
    return true; };
  for (let a = 0; a < 2 * Math.PI; a += Math.PI / 12) {
    const x = p.x + d * Math.cos(p.yaw + a), y = p.y + d * Math.sin(p.yaw + a);
    if (free(Math.floor((x - ox) / r), Math.floor((y - oy) / r))) return [x, y, 0];
  }
  return null;
}"""


def events_since(page, t0: float) -> list:
    return page.evaluate("(t0) => window.omniConsole.store.events.filter(e => e.t >= t0).map(e => e.text)", t0)


def run_tabs(page, a, st, cmd, wait, check):
    """Home, Health, Program, Map edit (a no-go zone Nav2 must plan around, an edge that slows the stick), Calibrate
    (the IMU level check), Follow (a person: the step's person:=true), the Logbook."""
    page.goto(a.url + "/#home")
    time.sleep(1.0)
    check(page.get_by_text("Play catch").count() > 0 and page.get_by_text("Follow me / Tag").count() > 0,
          "Home: a card per mode")
    h = cmd("health")
    check(h["ok"] and len(h["data"]["history"]) >= 3, f"Health: {len(h['data']['history'])} s of history, counters "
          f"{h['data']['counters']}")
    # Program: there and back
    prog = [{"do": "turn", "deg": 90}, {"do": "forward", "m": 0.3}, {"do": "back", "m": 0.3}, {"do": "turn", "deg": -90}]
    p0 = st()["pose"]
    r = cmd("program_run", {"name": "smoke", "blocks": prog})
    check(r["ok"], f"Program: run ({r['message']})")
    wait(lambda s: s["mission"] == "PROGRAM", 5, "Program: running")
    s = wait(lambda s: s["mission"] != "PROGRAM", 60, "Program: done in 60 s")
    p1 = st()["pose"]
    check(bool(s) and s["task"].endswith("finished") and math.hypot(p1["x"] - p0["x"], p1["y"] - p0["y"]) < 0.08,
          f"Program: finished back where it started ('{s and s['task']}', "
          f"{math.hypot(p1['x'] - p0['x'], p1['y'] - p0['y']):.3f} m off)")
    # Map edit: a no-go zone around a free goal: Nav2 must refuse to go there; then gone, it can
    g = page.evaluate(FREE_GOAL_JS)
    if g is None:
        check(False, "Map edit: a free spot for the zone")
    else:
        z = [[g["x"] - 0.5, g["y"] - 0.5], [g["x"] + 0.5, g["y"] - 0.5], [g["x"] + 0.5, g["y"] + 0.5], [g["x"] - 0.5, g["y"] + 0.5]]
        check(cmd("zone_save", {"name": "smoke zone", "points": z})["ok"], "Map edit: a no-go zone saved")
        time.sleep(3.0)                                    # the masks reach the costmaps
        cmd("goto", {"x": g["x"], "y": g["y"], "yaw": 0.0, "name": "inside the zone"})
        wait(lambda s: s["mission"] == "GOTO", 5, "Map edit: Go To into the zone sent")
        s = wait(lambda s: s["mission"] != "GOTO", 60, "Map edit: Go To into the no-go zone ends")
        check(bool(s) and s["task"].startswith("Could not reach"), f"Map edit: Nav2 won't enter it ('{s and s['task']}')")
        cmd("zone_delete", {"name": "smoke zone"})
    # an edge across the robot: the stick is capped to its speed in its band
    p = st()["pose"]
    cmd("edge_save", {"name": "smoke edge", "points": [[p["x"] - 1.0, p["y"] - 1.0], [p["x"] + 1.0, p["y"] + 1.0]],
                      "height_mm": 12, "speed": 0.4, "kind": "threshold"})
    page.goto(a.url + "/#manual")
    time.sleep(1.0)
    if st()["mission"] != "MANUAL":
        page.get_by_role("button", name="Take control").click()
        wait(lambda s: s["mission"] == "MANUAL", 5, "Manual again")
    page.keyboard.down("w")
    time.sleep(0.5)
    s = st()
    page.keyboard.up("w")
    sp = math.hypot(*(s["manual"]["cmd"] or [9, 9])[:2]) if s["manual"]["active"] else None
    check(s.get("slow") is not None and sp is not None and sp <= 0.401,
          f"Map edit: in the edge's band the stick is capped ({s.get('slow')}, {sp} m/s)")
    cmd("edge_delete", {"name": "smoke edge"})
    time.sleep(1.0)
    # Calibrate: the IMU level check (it turns half round by itself)
    r = cmd("calib_start", {"proc": "imu_level"})
    check(r["ok"], f"Calibrate: IMU level started ({r['message']})")
    t0 = time.time()
    res = None
    while time.time() - t0 < 40:
        c = cmd("calib")["data"]
        if c and (c.get("results") or {}).get("imu_level") and c["run"] and c["run"]["phase"] == "done":
            res = c["results"]["imu_level"][-1]
            break
        time.sleep(0.5)
    check(res is not None, f"Calibrate: IMU level result: {res and res['summary']}")
    cmd("stop")
    # Follow: needs a person in the sim (person:=true), who walks in front of the robot's start (the map's origin)
    cmd("goto", {"x": 0.0, "y": 0.0, "yaw": 0.0, "name": "the start"})
    wait(lambda s: s["mission"] == "GOTO", 5, "Follow: back to the start")
    wait(lambda s: s["mission"] != "GOTO", 90, "Follow: at the start")
    r = cmd("follow", {"mode": "follow"})
    p0 = st()["pose"]
    s = wait(lambda s: (s.get("follow") or {}).get("phase") == "follow", 30, "Follow: a person found")
    if s:
        seen, n, far = 0, 0, 0.0
        for _ in range(30):                              # 15 s: how much of it the robot had them
            time.sleep(0.5)
            s = st()
            f = s.get("follow") or {}
            n += 1
            seen += f.get("phase") == "follow"
            far = max(far, math.hypot(s["pose"]["x"] - p0["x"], s["pose"]["y"] - p0["y"]))
        check(far > 0.3 and seen >= 0.7 * n, f"Follow: the robot went after them ({far:.2f} m out, tracked "
              f"{100 * seen / n:.0f}% of 15 s)")
        if a.shots:
            page.goto(a.url + "/#follow")
            time.sleep(1.5)
            page.screenshot(path=f"{a.shots}/console_follow_live.png")
    cmd("stop")
    time.sleep(1.0)
    logs = page.evaluate("() => fetch('/logs/').then(r => r.json())")
    check(len(logs) > 0, f"Logbook: {len(logs)} session logs ({logs[0]['name'] if logs else '-'})")


def run_faults(page, a, st, cmd, wait, check):
    """The sim's fault injection (/sim/fault): the firmware stops the robot, the console says what happened."""
    pub = lambda kind, on, s=None: page.evaluate(                                   # noqa: E731
        "([k, on, s]) => window.omniConsole.publishJson('/sim/fault', {kind: k, on: on, s: s})", [kind, on, s])
    faults = lambda: [f["name"] for f in (st()["mcu"] or {}).get("faults", [])]      # noqa: E731
    pub("battery", True)
    wait(lambda s: "LOW_BATT" in [f["name"] for f in s["mcu"]["faults"]], 5, "Faults: low battery -> LOW_BATT")
    pub("battery", False)
    wait(lambda s: "LOW_BATT" not in [f["name"] for f in s["mcu"]["faults"]], 5, "Faults: LOW_BATT clears")
    # the link drops while driving: the MCU stops the robot by itself; the console sees it go quiet
    page.goto(a.url + "/#manual")
    time.sleep(1.0)
    if st()["mission"] != "MANUAL":
        page.get_by_role("button", name="Take control").click()
        wait(lambda s: s["mission"] == "MANUAL", 5, "Faults: Manual")
    t0 = page.evaluate("() => Date.now() / 1000")
    page.keyboard.down("q")                      # turning in place: nothing in the way can stop it but the MCU
    time.sleep(1.0)
    v0 = page.evaluate(TRUE_SPEED_JS)
    pub("link", True, 4.0)
    time.sleep(2.0)
    s2 = st()
    v1 = page.evaluate(TRUE_SPEED_JS)
    page.keyboard.up("q")
    check(s2["mcu"]["age"] > 1.0 and v0 is not None and v0[1] > 0.5 and v1 is not None and v1[1] < 0.05,
          f"Faults: link down: the MCU went quiet ({s2['mcu']['age']} s) and the robot stopped turning (true "
          f"{v0 and round(v0[1], 2)} -> {v1 and round(v1[1], 3)} rad/s)")
    check(bool(s2["pose"] and s2["pose"].get("stale")), "Faults: the page marks the robot's position as stale")
    time.sleep(3.0)
    ev = events_since(page, t0)
    check(any("went quiet" in e for e in ev) and any("MCU is back" in e for e in ev),
          "Faults: the console said the MCU went quiet and came back")
    pub("estop", True)
    wait(lambda s: s["mcu"]["mode"] == "ESTOP" and s["mission"] == "IDLE", 5, "Faults: e-stop -> ESTOP, mission IDLE")
    pub("estop", False)
    wait(lambda s: s["mcu"]["mode"] == "DISARMED", 5, "Faults: released -> DISARMED (no re-arm by itself)")
    check(not faults(), f"Faults: none left ({faults()})")


def play_two(page, a, cmd, wait, check, where):
    """Catch with the console: two new throws on the leaderboard, the radar shows the last one."""
    cmd("player_set", {"name": "Smoke"})
    before = next((p["throws"] for p in cmd("stats")["data"]["board"] if p["player"] == "Smoke"), 0)
    cmd("mission", {"name": "CATCH"})
    wait(lambda s: s["mission"] == "CATCH" and s["robot_mode"] == "CATCH", 10, f"Catch ({where}): playing")
    t0 = time.time()
    board = None
    while time.time() - t0 < 120:
        b = cmd("stats")["data"]["board"]
        me = next((p for p in b if p["player"] == "Smoke"), None)
        if me and me["throws"] >= before + 2:
            board = me
            break
        time.sleep(2.0)
    check(board is not None, f"Catch ({where}): two new throws on the leaderboard ({board})")
    page.goto(a.url + "/#catch")
    time.sleep(2.0)
    radar = page.locator(".radar .stat .v").first.inner_text()
    check("km/h" in radar, f"Catch ({where}): the throw radar shows '{radar}'")
    if a.shots:
        page.screenshot(path=f"{a.shots}/console_catch_radar_{where.split()[-1]}.png")


def run_catch(page, a, st, cmd, wait, check):
    """Catch in the house's one world (the robot step, catch:=true): a thrower steps up where there's room, two
    throws. Then the page as the launcher: the supervisor switches to the open field's
    world ('catch'), two throws, and back."""
    s = st()
    if s["caps"]["catch"]:                               # the thrower steps up where there's room (house_catch.py)
        play_two(page, a, cmd, wait, check, "in the house")
        cmd("mission", {"name": "MANUAL"})
        wait(lambda s: s["robot_mode"] == "TELEOP", 5, "Catch (house): back to MANUAL")
    post = lambda path, body: page.evaluate(                                         # noqa: E731
        "([p, b]) => fetch(p, {method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify(b)})"
        ".then(r => r.json())", [path, body])
    r = post("/api/start", {"step": "catch", "args": {"viewer": "false", "foxglove": "false"}})
    check(r.get("ok"), f"Catch: the supervisor switches to the catch world ({r.get('message')})")
    time.sleep(10)
    t0 = time.time()
    ok = False
    while time.time() - t0 < 150:
        s = page.evaluate("() => window.omniConsole.store.status")
        if s and s["caps"]["catch"]:
            ok = True
            break
        time.sleep(1.0)
    check(ok, "Catch: the catch code is up")
    play_two(page, a, cmd, wait, check, "the open field")
    cmd("mission", {"name": "IDLE"})
    r = post("/api/start", {"step": "robot", "args": {"viewer": "false", "foxglove": "false"}})
    t0 = time.time()
    while time.time() - t0 < 240:                         # a failed SLAM / Nav2 bring-up retries itself (~1-2 min)
        s = page.evaluate("() => window.omniConsole.store.status")
        if s and s["caps"]["nav"] and s["pose"] and s["pose"]["frame"] == "map":
            break
        time.sleep(1.0)
    check(bool(s and s["caps"]["nav"]), "Catch: back to the house world (navigation up)")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--url", default="http://localhost:8080")
    ap.add_argument("--shots", help="folder for screenshots of every tab")
    ap.add_argument("--headed", action="store_true", help="show the browser")
    ap.add_argument("--slam", action="store_true", help="also New map + Explore, then reload the map (robot step)")
    ap.add_argument("--cpu", action="store_true", help="also the console's CPU on the robot side (WSL), per tab")
    ap.add_argument("--tabs", action="store_true", help="also Home, Program, Map edit (a no-go zone, an edge), "
                    "Calibrate, Health, Follow (with a person: person:=true), the Logbook")
    ap.add_argument("--faults", action="store_true", help="also fault injection (link, battery, e-stop: the sim)")
    ap.add_argument("--catch", action="store_true", help="also the catch world through the supervisor: switch to it, "
                    "play two throws, check the radar and the leaderboard, switch back")
    a = ap.parse_args()
    fails = []

    def check(ok, what):
        print(("  ok   " if ok else "  FAIL ") + what)
        if not ok:
            fails.append(what)

    with sync_playwright() as pw:
        browser = pw.chromium.launch(channel="msedge", headless=not a.headed)
        page = browser.new_page(viewport={"width": 1400, "height": 860})
        errors = []
        page.on("pageerror", lambda e: errors.append(str(e)))
        page.on("console", lambda m: errors.append(m.text) if m.type == "error" else None)
        # count what crosses the (Wi-Fi) link inside the page: every message rosbridge sends it
        page.add_init_script("""(() => { const W = window.WebSocket; window.__rx = 0;
          window.WebSocket = class extends W { constructor(...a) { super(...a); this.addEventListener('message', (e) => {
            window.__rx += typeof e.data === 'string' ? e.data.length : (e.data.byteLength || e.data.size || 0); }); } }; })()""")

        st = lambda: page.evaluate("() => window.omniConsole.store.status")              # noqa: E731
        cmd = lambda verb, args=None: page.evaluate(                                      # noqa: E731
            "([v, a]) => window.omniConsole.command(v, a)", [verb, args or {}])

        def wait(pred, timeout, what):
            t0 = time.time()
            while time.time() - t0 < timeout:
                s = st()
                if s and pred(s):
                    check(True, what)
                    return s
                time.sleep(0.1)
            check(False, what)
            return None

        print(f"console at {a.url}")
        page.goto(a.url + "/#manual")
        s = wait(lambda s: True, 30, "status arrives")
        if s is None:
            print("no status: is the stack running with console:=true?")
            sys.exit(1)
        time.sleep(2.5)
        hz = page.evaluate("() => window.omniConsole.store.statusHz")
        check(8 <= hz <= 12, f"status at {hz} Hz (10 expected)")
        b0, t0 = page.evaluate("() => window.__rx"), time.time()
        time.sleep(5)
        kbps = (page.evaluate("() => window.__rx") - b0) * 8 / 1000 / (time.time() - t0)
        check(kbps < 1000, f"console traffic {kbps:.0f} kbit/s (map, scan, status; < 1000)")
        fps = page.evaluate("""() => new Promise(res => { let n = 0; const t0 = performance.now();
            const f = () => { n++; if (performance.now() - t0 < 2000) requestAnimationFrame(f);
            else res(n * 1000 / (performance.now() - t0)); }; requestAnimationFrame(f); })""")
        check(fps >= 30, f"page frame rate {fps:.0f} fps (>= 30)")
        # the raw topics, decoded and placed by the page
        sc = page.evaluate("() => { const s = window.omniConsole.store; return s.scan && "
                           "{frame: s.scan.frame, n: s.scan.xy.length / 2}; }")
        check(bool(sc) and sc["n"] > 200, f"the lidar scan, placed by the page: {sc}")
        mp = page.evaluate("() => { const m = window.omniConsole.store.map; return m && "
                           "[m.info.width, m.info.height, m.info.resolution, m.data.length]; }")
        check(bool(mp) and mp[0] * mp[1] == mp[3], f"the map, decoded by the page: {mp}")
        if a.cpu:
            for tab in ("manual", "base"):
                page.goto(f"{a.url}/#{tab}")
                time.sleep(3)
                use = robot_cpu(10)
                tot = sum(v for k, v in use.items() if k.startswith("console"))
                print(f"  CPU with the {tab} tab open: console {tot:.1f}% of a core ("
                      + ", ".join(f"{k} {v}" for k, v in use.items() if k.startswith("console")) + ")")
                check(tot < CONSOLE_CPU_BUDGET, f"console CPU {tot:.1f}% < {CONSOLE_CPU_BUDGET}% ({tab} tab)")
            page.goto(a.url + "/#manual")
            time.sleep(1)

        # Manual: take control, hold W, let go
        if st()["mission"] != "MANUAL":                 # the nav step starts in TELEOP: already MANUAL
            page.get_by_role("button", name="Take control").click()
        s = wait(lambda s: s["mission"] == "MANUAL" and s["robot_mode"] == "TELEOP", 5, "Take control -> MANUAL")
        p0 = st()["pose"]
        page.keyboard.down("w")
        time.sleep(0.6)
        active = st()["manual"]["active"]
        time.sleep(1.4)
        page.keyboard.up("w")
        time.sleep(1.2)
        p1 = st()["pose"]
        moved = math.hypot(p1["x"] - p0["x"], p1["y"] - p0["y"])
        check(active, "the robot reports the console's stick while W is held")
        check(moved > 0.3, f"W drove the robot {moved:.2f} m in 2 s")
        time.sleep(1.0)
        p2 = st()["pose"]
        check(math.hypot(p2["x"] - p1["x"], p2["y"] - p1["y"]) < 0.05, "it stays put after W is released")
        check(not st()["manual"]["active"], "the stick is no longer active")

        # a setting round-trips (schema echo)
        r = cmd("set", {"key": "manual.max_speed", "value": 0.7})
        time.sleep(0.5)
        val = page.evaluate("() => window.omniConsole.store.schema.settings.find(s => s.key === 'manual.max_speed').value")
        check(r["ok"] and abs(val - 0.7) < 1e-9, "Top speed 0.7 m/s set and echoed")
        check(not cmd("set", {"key": "manual.max_speed", "value": 5})["ok"], "out-of-range setting refused")
        cmd("set", {"key": "manual.max_speed", "value": 1.0})

        # closing the page while driving stops the robot
        page2 = browser.new_page(viewport={"width": 1400, "height": 860})
        page2.goto(a.url + "/#manual")
        page2.wait_for_function("() => window.omniConsole.store.status !== null", timeout=20000)
        page2.keyboard.down("w")
        time.sleep(1.0)
        page2.close()
        time.sleep(0.8)
        q1 = st()["pose"]
        time.sleep(1.0)
        q2 = st()["pose"]
        check(math.hypot(q2["x"] - q1["x"], q2["y"] - q1["y"]) < 0.05,
              "closing the page while driving: stopped within 0.8 s")

        # Go To: a click on the map makes a goal; a saved free place is reached
        page.goto(a.url + "/#goto")
        page.wait_for_selector(".map canvas")
        time.sleep(1.5)
        box = page.locator(".map canvas").bounding_box()
        page.mouse.click(box["x"] + box["width"] / 2 + 60, box["y"] + box["height"] / 2)
        check(page.get_by_text("New goal").count() > 0, "a click on the map makes a pending goal")
        if a.shots:
            page.screenshot(path=f"{a.shots}/console_goto_pending.png")
        page.keyboard.press("Escape")
        g = page.evaluate(FREE_GOAL_JS)
        if g is None:
            check(False, "found a free spot for a goal (map and pose needed)")
        else:
            cmd("place_save", {"name": "smoke test", "x": g["x"], "y": g["y"], "yaw": 0.0})
            time.sleep(0.5)
            page.locator("li", has_text="smoke test").get_by_role("button", name="Go").click()
            wait(lambda s: s["mission"] == "GOTO", 5, "Go -> GOTO")
            time.sleep(1.0)
            s = st()
            if a.shots:
                page.screenshot(path=f"{a.shots}/console_goto_driving.png")
            check(s["mission"] != "GOTO" or s["progress"] is not None, f"progress reported: '{s['task']}'")
            s = wait(lambda s: s["mission"] != "GOTO", 60, f"goal {g['d']:.1f} m away reached in 60 s")
            if s:
                check(s["task"].startswith("Arrived"), f"Go To ended: '{s['task']}'")
            cmd("place_delete", {"name": "smoke test"})

        # the subsystem tabs
        v = page.evaluate("() => window.omniConsole.store.version")
        f = (v or {}).get("firmware") or {}
        check(bool(f.get("running")) and f.get("running") == f.get("expected")
              and f.get("protocol") == f.get("protocol_host"),
              f"firmware: MCU runs {f.get('running')}, this checkout builds {f.get('expected')}, "
              f"protocol {f.get('protocol')}")
        check(st()["rc"] is not None and len(st()["rc"]["ch"]) == 8, "RC receiver reported (INFO, 10 Hz)")
        rows = page.evaluate("""() => import('/js/cdr.js').then(cdr => new Promise(res => { let n = 0;
            const t = new ROSLIB.Topic({ ros: window.omniConsole.getRos(), name: '/mcu/samples',
                                         messageType: 'std_msgs/msg/Float32MultiArray', compression: 'cbor-raw' });
            t.subscribe(m => { const d = cdr.decodeFloat32MultiArray(m.bytes); n += d.dims[0].size; });
            setTimeout(() => { t.unsubscribe(); res(n); }, 2000); }))""")
        check(rows > 250, f"Base tab: {rows} MCU samples in 2 s (200 Hz in batches, raw from mcu_link)")
        page.goto(a.url + "/#thrower")
        page.get_by_role("button", name="Take control").click()
        wait(lambda s: s["mission"] == "THROWER" and s["robot_mode"] == "THROWER", 5, "Thrower: bench mode")
        cmd("thrower", {"rpm": 3000, "tilt": 0})          # a 30 deg launch (the flattest fired is 25: the face)
        wait(lambda s: s["mcu"]["fly_ready"], 10, "Thrower: flywheels at 3000 rpm")
        done = st()["mcu"]["fire_seq_done"]
        page.once("dialog", lambda d: d.accept())
        page.get_by_role("button", name="FIRE ONE BALL").click()
        wait(lambda s: s["mcu"]["fire_seq_done"] == (done + 1) & 0xFFFF, 3, "Thrower: one fire request reaches the MCU")
        if a.shots:
            page.screenshot(path=f"{a.shots}/console_thrower_live.png")
        cmd("thrower", {"rpm": 0, "tilt": 0})
        cmd("mission", {"name": "MANUAL"})
        wait(lambda s: s["robot_mode"] == "TELEOP" and s["mcu"]["fire_seq_done"] == (done + 1) & 0xFFFF, 5,
             "back to MANUAL without another shot")

        # Patrol: two free points, once
        pa, pb = page.evaluate(FREE_POINT_JS, 1.2), page.evaluate(FREE_POINT_JS, 2.2)
        if pa and pb:
            old = {k: page.evaluate(f"() => window.omniConsole.store.schema.settings.find(s => s.key === '{k}').value")
                   for k in ("patrol.mode", "patrol.dwell", "patrol.laps")}
            for k, val in (("patrol.mode", "once"), ("patrol.dwell", 1), ("patrol.laps", 1)):
                cmd("set", {"key": k, "value": val})
            cmd("route_save", {"name": "smoke test", "points": [pa, pb]})
            page.goto(a.url + "/#patrol")
            time.sleep(1.0)
            page.locator("li", has_text="smoke test").get_by_role("button", name="Run").click()
            wait(lambda s: s["mission"] == "PATROL", 5, "Patrol: Run")
            if a.shots:
                time.sleep(1.5)
                page.screenshot(path=f"{a.shots}/console_patrol_live.png")
            s = wait(lambda s: s["mission"] != "PATROL", 90, "Patrol: both points visited")
            check(bool(s) and s["task"].endswith("finished"), f"Patrol ended: '{s and s['task']}'")
            cmd("route_delete", {"name": "smoke test"})
            for k, val in old.items():
                cmd("set", {"key": k, "value": val})
        else:
            check(False, "Patrol: free points for a route (map and pose needed)")

        if a.slam:
            sl = st()["slam"]
            if not sl["managed"]:
                check(False, "--slam needs the robot step (the console runs SLAM)")
            else:
                back = sl["map"]
                cmd("explore", {"new_map": True})
                wait(lambda s: s["slam"]["mode"] == "mapping", 60, "Explore: a new map")
                s = wait(lambda s: s["explore"] and s["explore"]["info"].get("known_m2"), 60, "Explore: planning")
                a0 = s["explore"]["info"]["known_m2"] if s else 0
                time.sleep(45)
                s = st()
                a1 = (s["explore"] or {}).get("info", {}).get("known_m2", a0) if s["mission"] == "EXPLORE" else a0
                check(s["mission"] != "EXPLORE" or a1 > a0, f"Explore: mapped {a0} -> {a1} m2 in 45 s ('{s['task']}')")
                if a.shots:
                    page.goto(a.url + "/#explore")
                    time.sleep(1.5)
                    page.screenshot(path=f"{a.shots}/console_explore_live.png")
                cmd("cancel")
                if back:
                    cmd("slam_load", {"name": back})
                    wait(lambda s: s["slam"]["mode"] == "localization" and s["slam"]["map"] == back, 60,
                         f"SLAM: back on '{back}'")

        sup = page.evaluate("() => fetch('/api/state').then(r => r.ok ? r.json() : null).catch(() => null)")
        if sup:
            cur = sup.get("current") or {}
            check(cur.get("state") == "running", f"supervisor: '{cur.get('label')}' running ({len(sup['steps'])} steps)")
            lines = page.evaluate("() => fetch('/api/log?n=20').then(r => r.json()).then(d => d.lines.length)")
            check(lines > 0, f"supervisor: the step's log ({lines} lines)")
        if a.tabs:
            run_tabs(page, a, st, cmd, wait, check)
        if a.faults:
            run_faults(page, a, st, cmd, wait, check)
        if a.catch:
            if sup:
                run_catch(page, a, st, cmd, wait, check)
            else:
                check(False, "--catch needs the supervisor (omni.sh console)")

        # STOP: the space bar disarms
        page.goto(a.url + "/#manual")
        page.wait_for_function("() => window.omniConsole.store.status !== null", timeout=20000)
        page.keyboard.press("Space")
        wait(lambda s: s["mission"] == "IDLE" and s["robot_mode"] == "IDLE", 5, "Space -> IDLE")
        wait(lambda s: s["mcu"] and s["mcu"]["mode"] == "DISARMED", 5, "the MCU disarms")

        # every tab renders; the ROS tab lists topics
        for tab in ("home", "manual", "goto", "explore", "patrol", "catch", "follow", "program", "base", "thrower",
                    "face", "vision", "slam", "mapedit", "calibrate", "health", "logbook", "system", "rc", "version", "ros"):
            page.goto(f"{a.url}/#{tab}")
            time.sleep(1.0 if tab != "ros" else 2.5)
            if a.shots:
                page.screenshot(path=f"{a.shots}/console_{tab}.png")
        n = page.locator("table.t tbody tr").count()
        check(n > 10, f"ROS tab lists {n} topics")
        # rosbridge refusing while the supervisor switches steps is expected (--catch): the page reconnects
        errors = [e for e in errors if not (a.catch and "ws://" in e and "ERR_CONNECTION_REFUSED" in e)]
        check(not errors, "no errors in the page" + (f": {errors[:3]}" if errors else ""))
        browser.close()
    print(f"\n{len(fails)} failed" if fails else "\nall passed")
    sys.exit(1 if fails else 0)


if __name__ == "__main__":
    main()
