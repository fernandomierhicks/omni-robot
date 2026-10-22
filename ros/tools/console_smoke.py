"""Smoke test of the operator console (omni_console) in a real browser against a running stack.

  & $py ros\\tools\\console_smoke.py                      (after `omni.sh nav`; the page at http://localhost:8080)
  & $py ros\\tools\\console_smoke.py --shots <folder>      also screenshots of every tab

Drives Microsoft Edge (installed with Windows) through Playwright (pip install playwright; no browser download).
Checks: the page connects and the status updates at ~10 Hz; Take control + W drives the robot and letting go stops it;
closing the page while driving stops it; a Go To goal from a saved place arrives; a setting round-trips; Space stops
and disarms; the ROS tab lists topics. Prints the console's Wi-Fi traffic and the page's frame rate. Leaves the robot
in IDLE and deletes the place it saved.
"""

from __future__ import annotations

import argparse
import math
import sys
import time

from playwright.sync_api import sync_playwright

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
  for (let d = 2.5; d >= 1.0; d -= 0.25) for (let a = 0; a < 2 * Math.PI; a += Math.PI / 12) {
    const x = p.x + d * Math.cos(p.yaw + a), y = p.y + d * Math.sin(p.yaw + a);
    if (free(Math.floor((x - ox) / r), Math.floor((y - oy) / r))) return { x, y, d };
  }
  return null;
}"""


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--url", default="http://localhost:8080")
    ap.add_argument("--shots", help="folder for screenshots of every tab")
    ap.add_argument("--headed", action="store_true", help="show the browser")
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

        # STOP: the space bar disarms
        page.goto(a.url + "/#manual")
        page.wait_for_function("() => window.omniConsole.store.status !== null", timeout=20000)
        page.keyboard.press("Space")
        wait(lambda s: s["mission"] == "IDLE" and s["robot_mode"] == "IDLE", 5, "Space -> IDLE")
        wait(lambda s: s["mcu"] and s["mcu"]["mode"] == "DISARMED", 5, "the MCU disarms")

        # every tab renders; the ROS tab lists topics
        for tab in ("manual", "goto", "explore", "patrol", "catch", "base", "thrower", "vision", "slam", "rc", "version", "ros"):
            page.goto(f"{a.url}/#{tab}")
            time.sleep(1.0 if tab != "ros" else 2.5)
            if a.shots:
                page.screenshot(path=f"{a.shots}/console_{tab}.png")
        n = page.locator("table.t tbody tr").count()
        check(n > 10, f"ROS tab lists {n} topics")
        check(not errors, "no errors in the page" + (f": {errors[:3]}" if errors else ""))
        browser.close()
    print(f"\n{len(fails)} failed" if fails else "\nall passed")
    sys.exit(1 if fails else 0)


if __name__ == "__main__":
    main()
