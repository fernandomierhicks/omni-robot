"""Drive the robot's web console in a real browser, one step at a time (for an AI session or a person at a terminal).

A background server holds one Microsoft Edge (Playwright, the sim venv; no browser download) with its tabs and a log
of everything that went wrong in them; short commands talk to it, so the browser, its websockets and the page's state
live on between steps:

  $py = C:\\Users\\ferna\\.venvs\\omni-robot-sim\\Scripts\\python.exe
  $py ros/tools/pi_console.py serve [--url http://192.168.0.100:8080] [--headed] [--shots DIR]   (run in background)
  $py ros/tools/pi_console.py open '#lidar'          a page of the console (#home, #system, #cameras, #motors, ...),
                                                     a path or a full URL
  $py ros/tools/pi_console.py click 'Start' [--nth 1] [--exact] [--within CSS]   a button (else any element) by text
  $py ros/tools/pi_console.py text [CSS] [--all]     the page's text (main; --all: with the tab bar and status bar)
  $py ros/tools/pi_console.py shot [name] [--full]   a screenshot (PNG path printed: look at it)
  $py ros/tools/pi_console.py errors [--peek]        console errors / warnings, page errors, failed requests, HTTP >= 400
                                                     (404s first: a missing module blanks the whole page) since the last
                                                     call; --peek leaves them for the next one
  $py ros/tools/pi_console.py events [--n 40]        everything logged (also websocket opens / closes, dialogs)
  $py ros/tools/pi_console.py wait 'text' [--gone] [--timeout 20]   until the text shows (or goes)
  $py ros/tools/pi_console.py eval 'js expression'   (window.omniConsole.store: the page's state)
  $py ros/tools/pi_console.py key Space | fill CSS VALUE | select CSS VALUE | check CSS [--off] | reload
  $py ros/tools/pi_console.py hold w 2 [--js EXPR] [--after 1.5] [--press Space@1]   hold a key (the page's W / S
                                                     dead man), sampling EXPR every 0.2 s during and after; --press
                                                     presses another key that many seconds in (STOP while driving)
  $py ros/tools/pi_console.py tab new [--mobile] [URL] | tab N | tab close | tabs
  $py ros/tools/pi_console.py status | quit

The page's confirm() dialogs (Stop, Reboot, ...) are accepted and logged. `serve` finds the Pi's wired address over
SSH (`ssh omni-cam hostname -I`: omni-cam.local can take ~11 s to resolve from Windows) unless --url or
OMNI_CONSOLE_URL says where. Commands reach the server on 127.0.0.1:--port (8399; OMNI_PI_CONSOLE_PORT).
Shots go to --shots (default: %TEMP%/pi_console).
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import tempfile
import time
import urllib.error
import urllib.request
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path

PORT = int(os.environ.get("OMNI_PI_CONSOLE_PORT", "8399"))
BAD = ("console.error", "pageerror", "requestfailed", "http")


def pi_url() -> str:
    """The console on the Pi: its wired address if it has one (same switch as the PC), else its first address."""
    if os.environ.get("OMNI_CONSOLE_URL"):
        return os.environ["OMNI_CONSOLE_URL"].rstrip("/")
    host = os.environ.get("OMNI_PI", "omni-cam")
    try:
        r = subprocess.run(["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=10", host, "hostname -I"],
                           capture_output=True, text=True, timeout=30)
        addrs = [a for a in r.stdout.split() if a.count(".") == 3]
    except (OSError, subprocess.TimeoutExpired):
        addrs = []
    wired = [a for a in addrs if a.startswith("192.168.0.")]
    return f"http://{(wired or addrs or ['omni-cam.local'])[0]}:8080"


# -- the server: one browser, its tabs, the log ------------------------------------------------------------------------
class Browser:
    def __init__(self, url: str, headed: bool, shots: Path):
        from playwright.sync_api import sync_playwright
        self.base = url.rstrip("/")
        self.shots = shots
        shots.mkdir(parents=True, exist_ok=True)
        self.pw = sync_playwright().start()
        self.browser = self.pw.chromium.launch(channel="msedge", headless=not headed)
        self.ctx = self.browser.new_context(viewport={"width": 1400, "height": 900})
        self.pages = []
        self.cur = 0
        self.log = []                    # dict(t, tab, kind, text)
        self.read_upto = 0               # errors already reported
        self.new_tab(None)

    def note(self, page, kind: str, text: str):
        tab = self.pages.index(page) if page in self.pages else -1
        self.log.append(dict(t=time.time(), tab=tab, kind=kind, text=text[:600]))
        if len(self.log) > 5000:
            drop = len(self.log) - 4000
            self.log = self.log[drop:]
            self.read_upto = max(0, self.read_upto - drop)

    def new_tab(self, url, mobile=False):
        if mobile:
            dev = self.pw.devices["iPhone 13"]
            ctx = self.browser.new_context(**{k: v for k, v in dev.items() if k != "default_browser_type"})
        else:
            ctx = self.ctx
        p = ctx.new_page()
        p.on("console", lambda m, p=p: self.note(p, f"console.{m.type}", m.text)
             if m.type in ("error", "warning") else None)
        p.on("pageerror", lambda e, p=p: self.note(p, "pageerror", str(e)))
        p.on("requestfailed", lambda r, p=p: self.note(p, "requestfailed", f"{r.method} {r.url}: {r.failure}"))
        p.on("response", lambda r, p=p: self.note(p, "http", f"{r.status} {r.request.method} {r.url}")
             if r.status >= 400 else None)
        p.on("dialog", lambda d, p=p: (self.note(p, "dialog", f"{d.type}: {d.message} (accepted)"), d.accept()))

        def ws(w, p=p):
            self.note(p, "ws", f"open {w.url}")
            w.on("close", lambda _w=None: self.note(p, "ws", f"closed {w.url}"))
            w.on("socketerror", lambda e: self.note(p, "ws", f"error {w.url}: {e}"))
        p.on("websocket", ws)
        self.pages.append(p)
        self.cur = len(self.pages) - 1
        if url is not None or len(self.pages) == 1:
            self.goto(url or "#home")
        return self.cur

    @property
    def page(self):
        return self.pages[self.cur]

    def resolve(self, where: str) -> str:
        if where.startswith("http"):
            return where
        if where.startswith("#"):
            return f"{self.base}/{where}"
        return f"{self.base}/{where.lstrip('/')}"

    def goto(self, where: str):
        url = self.resolve(where)
        p = self.page
        if url.split("#")[0] == p.url.split("#")[0] and "#" in url:
            p.evaluate("(h) => { location.hash = h; }", "#" + url.split("#", 1)[1])
        else:
            p.goto(url, wait_until="domcontentloaded", timeout=30000)
        p.wait_for_timeout(1500)
        return dict(url=p.url, title=p.title())

    def click(self, text: str, nth: int = 0, exact: bool = False, within: str = ""):
        root = self.page.locator(within) if within else self.page
        loc = root.get_by_role("button", name=text, exact=exact)
        kind = "button"
        if loc.count() == 0:
            loc, kind = root.get_by_text(text, exact=exact), "text"
        n = loc.count()
        if n == 0:
            raise ValueError(f"nothing with the text '{text}' on {self.page.url}")
        loc.nth(nth).click(timeout=8000)
        self.page.wait_for_timeout(500)
        return dict(clicked=f"{kind} '{text}'", matches=n, nth=nth)

    def text(self, sel: str, full: bool):
        sel = sel or ("body" if full else "main.page")
        return self.page.locator(sel).first.inner_text(timeout=8000)

    def shot(self, name: str, full: bool):
        name = name or time.strftime("shot-%H%M%S")
        path = self.shots / (name if name.endswith(".png") else name + ".png")
        self.page.screenshot(path=str(path), full_page=full)
        return str(path)

    def problems(self, peek: bool):
        new = [e for e in self.log[self.read_upto:] if e["kind"] in BAD
               and not (e["kind"] == "console.error" and "Failed to load resource" in e["text"])]
        if not peek:
            self.read_upto = len(self.log)
        missing = [e for e in new if e["kind"] == "http" and e["text"].startswith("404")]
        groups = {}                        # the same problem repeated (a reconnect loop): once, with a count
        for e in new:
            key = (e["tab"], e["kind"], e["text"].split("?")[0])
            g = groups.setdefault(key, dict(n=0, first=e["t"], last=e["t"], text=e["text"]))
            g["n"] += 1
            g["last"] = e["t"]
        hm = lambda t: time.strftime("%H:%M:%S", time.localtime(t))          # noqa: E731
        return dict(n=len(new), missing_404=sorted({e["text"] for e in missing}),
                    items=[f"tab{tab} {hm(g['first'])}" + (f"-{hm(g['last'])} x{g['n']}" if g["n"] > 1 else "")
                           + f" {kind}: {g['text']}" for (tab, kind, _), g in list(groups.items())[-40:]])

    def hold(self, key: str, seconds: float, js: str, after: float, press: list):
        """Hold a key down for `seconds` (the page's W / S dead man), sampling `js` every 0.2 s during and after;
        press: ["Space@1.5", ...] presses those keys that many seconds in (STOP while driving)."""
        p = self.page
        out, t0 = [], time.time()
        sample = lambda: out.append([round(time.time() - t0, 2), p.evaluate(js) if js else None])   # noqa: E731
        todo = sorted((float(x.rsplit("@", 1)[1]), x.rsplit("@", 1)[0]) for x in press)
        p.keyboard.down(key)
        try:
            while time.time() - t0 < seconds:
                while todo and time.time() - t0 >= todo[0][0]:
                    p.keyboard.press(todo.pop(0)[1])
                    out.append([round(time.time() - t0, 2), "pressed"])
                sample()
                p.wait_for_timeout(200)
        finally:
            p.keyboard.up(key)
        t1 = time.time()
        while time.time() - t1 < after:
            sample()
            p.wait_for_timeout(200)
        return out

    def wait(self, text: str, gone: bool, timeout: float):
        loc = self.page.get_by_text(text).first
        loc.wait_for(state="hidden" if gone else "visible", timeout=timeout * 1000)
        return f"'{text}' {'gone' if gone else 'shown'}"

    def handle(self, op: str, a: dict):
        p = self.page
        if op == "open":
            return self.goto(a["where"])
        if op == "click":
            return self.click(a["text"], a.get("nth", 0), a.get("exact", False), a.get("within", ""))
        if op == "text":
            return self.text(a.get("sel", ""), a.get("all", False))
        if op == "shot":
            return self.shot(a.get("name", ""), a.get("full", False))
        if op == "errors":
            return self.problems(a.get("peek", False))
        if op == "events":
            n = a.get("n", 40)
            return [f"tab{e['tab']} {time.strftime('%H:%M:%S', time.localtime(e['t']))} {e['kind']}: {e['text']}"
                    for e in self.log[-n:]]
        if op == "wait":
            return self.wait(a["text"], a.get("gone", False), a.get("timeout", 20))
        if op == "eval":
            return p.evaluate(a["js"])
        if op == "key":
            p.keyboard.press(a["key"])
            return f"pressed {a['key']}"
        if op == "hold":
            return self.hold(a["key"], a.get("seconds", 1.0), a.get("js", ""), a.get("after", 1.5), a.get("press") or [])
        if op == "fill":
            p.locator(a["sel"]).first.fill(a["value"])
            p.locator(a["sel"]).first.dispatch_event("change")
            return "filled"
        if op == "select":
            return p.locator(a["sel"]).first.select_option(a["value"])
        if op == "check":                # a styled switch hides its input: click it from script (fires input + change)
            want = not a.get("off", False)
            was = p.locator(a["sel"]).first.evaluate("(el, want) => { const was = el.checked; "
                                                     "if (was !== want) el.click(); return was; }", want)
            p.wait_for_timeout(300)
            return f"{'on' if want else 'off'} (was {'on' if was else 'off'})"
        if op == "reload":
            p.reload(wait_until="domcontentloaded")
            p.wait_for_timeout(1500)
            return p.url
        if op == "tab":
            w = a.get("what", "")
            if w == "new":
                return dict(tab=self.new_tab(a.get("url") or "#home", a.get("mobile", False)), url=self.page.url)
            if w == "close":
                if len(self.pages) == 1:
                    raise ValueError("the last tab: use quit")
                self.page.close()
                self.pages.pop(self.cur)
                self.cur = len(self.pages) - 1
                return dict(tab=self.cur, url=self.page.url)
            self.cur = int(w)
            self.page.bring_to_front()
            return dict(tab=self.cur, url=self.page.url)
        if op in ("tabs", "status"):
            return dict(base=self.base, cur=self.cur, tabs=[x.url for x in self.pages], logged=len(self.log),
                        unread_problems=self.problems(peek=True)["n"])
        raise ValueError(f"unknown command {op}")

    def close(self):
        try:
            self.browser.close()
        finally:
            self.pw.stop()


def serve(a):
    b = Browser(a.url or pi_url(), a.headed, Path(a.shots))
    done = []

    class H(BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass

        def do_POST(self):
            req = json.loads(self.rfile.read(int(self.headers.get("Content-Length") or 0)) or b"{}")
            op = req.get("op", "")
            try:
                if op == "quit":
                    done.append(1)
                    out = dict(ok=True, result="bye")
                else:
                    out = dict(ok=True, result=b.handle(op, req.get("args") or {}))
            except Exception as e:                                   # noqa: BLE001  (reported to the caller)
                out = dict(ok=False, error=f"{type(e).__name__}: {e}"[:2000])
            body = json.dumps(out, default=str).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

    httpd = HTTPServer(("127.0.0.1", a.port), H)
    httpd.timeout = 0.25
    print(f"pi_console: Edge on {b.base}, commands on 127.0.0.1:{a.port}, shots in {a.shots}", flush=True)
    while not done:
        httpd.handle_request()
        try:
            b.page.wait_for_timeout(5)            # lets Playwright deliver the tabs' events between commands
        except Exception:                         # noqa: BLE001  (a tab mid-navigation)
            pass
    b.close()


# -- the commands -----------------------------------------------------------------------------------------------------
def call(op: str, args: dict | None = None, port: int = PORT, timeout: float = 120):
    req = urllib.request.Request(f"http://127.0.0.1:{port}/", data=json.dumps(dict(op=op, args=args or {})).encode(),
                                 headers={"Content-Type": "application/json"}, method="POST")
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.loads(r.read())


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=PORT)
    sub = ap.add_subparsers(dest="op", required=True)
    s = sub.add_parser("serve")
    s.add_argument("--url", default="")
    s.add_argument("--headed", action="store_true")
    s.add_argument("--shots", default=str(Path(tempfile.gettempdir()) / "pi_console"))
    sub.add_parser("open").add_argument("where")
    c = sub.add_parser("click")
    c.add_argument("text")
    c.add_argument("--nth", type=int, default=0)
    c.add_argument("--exact", action="store_true")
    c.add_argument("--within", default="")
    t = sub.add_parser("text")
    t.add_argument("sel", nargs="?", default="")
    t.add_argument("--all", action="store_true")
    sh = sub.add_parser("shot")
    sh.add_argument("name", nargs="?", default="")
    sh.add_argument("--full", action="store_true")
    sub.add_parser("errors").add_argument("--peek", action="store_true")
    sub.add_parser("events").add_argument("--n", type=int, default=40)
    w = sub.add_parser("wait")
    w.add_argument("text")
    w.add_argument("--gone", action="store_true")
    w.add_argument("--timeout", type=float, default=20)
    sub.add_parser("eval").add_argument("js")
    sub.add_parser("key").add_argument("key")
    hd = sub.add_parser("hold")
    hd.add_argument("key")
    hd.add_argument("seconds", type=float)
    hd.add_argument("--js", default="", help="sampled every 0.2 s, e.g. the wheel speeds")
    hd.add_argument("--after", type=float, default=1.5, help="s of sampling after the key is let go")
    hd.add_argument("--press", action="append", default=[], help="KEY@SECONDS: press it while holding (Space@1.5)")
    for name in ("fill", "select"):
        f = sub.add_parser(name)
        f.add_argument("sel")
        f.add_argument("value")
    ck = sub.add_parser("check")
    ck.add_argument("sel")
    ck.add_argument("--off", action="store_true")
    sub.add_parser("reload")
    tb = sub.add_parser("tab")
    tb.add_argument("what")
    tb.add_argument("url", nargs="?", default="")
    tb.add_argument("--mobile", action="store_true")
    for name in ("tabs", "status", "quit"):
        sub.add_parser(name)
    a = ap.parse_args(argv)
    for stream in (sys.stdout, sys.stderr):              # the page's text has arrows and symbols (Windows: cp1252)
        try:
            stream.reconfigure(encoding="utf-8", errors="replace")
        except (AttributeError, ValueError):
            pass
    if a.op == "serve":
        return serve(a)
    args = {k: v for k, v in vars(a).items() if k not in ("op", "port")}
    timeout = args.get("timeout", 20) + 30 if a.op == "wait" else 120
    try:
        out = call(a.op, args, a.port, timeout)
    except (urllib.error.URLError, ConnectionError) as e:
        print(f"no pi_console server on 127.0.0.1:{a.port} ({e}): start `pi_console.py serve` first", file=sys.stderr)
        return 2
    if not out.get("ok"):
        print("ERROR:", out.get("error"), file=sys.stderr)
        return 1
    r = out["result"]
    if isinstance(r, str):
        print(r)
    else:
        print(json.dumps(r, indent=1, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    sys.exit(main())
