"""The AI lease (ai_robot_architecture.md 7): a background thread renews it every second while the MCP server runs.
Motion verbs from this session need it; when the server dies, the desktop sleeps or the link drops, it runs out
(ttl 3 s) and the robot's console cancels the AI's task and stops the robot. A live lease says only that this process
is alive, not that the model is reasoning well: the tasks' deadlines cover that.
"""

from __future__ import annotations

import threading
import time


class LeaseKeeper:
    def __init__(self, rb, session: str, ttl_s: float = 3.0, period_s: float = 1.0, log=None):
        self.rb, self.session, self.ttl, self.period = rb, session, ttl_s, period_s
        self.log = log or (lambda m: None)
        self.ok = False
        self.last_ok = None              # time.monotonic() of the last renewal the robot accepted
        self.error = "not renewed yet"
        self.owner = None
        self._stop = threading.Event()
        self.thread = threading.Thread(target=self._run, name="ai-lease", daemon=True)

    def start(self):
        self.thread.start()
        return self

    def renew(self) -> bool:
        try:
            r = self.rb.console("ai_lease", dict(session=self.session, ttl_s=self.ttl), timeout=self.period)
        except Exception as e:                       # rosbridge down, the console not running
            self.ok, self.error = False, str(e)
            return False
        was = self.ok
        self.ok = bool(r["ok"])
        if self.ok:
            self.last_ok, self.error = time.monotonic(), ""
            self.owner = (r.get("data") or {}).get("owner")
            if not was:
                self.log(f"AI lease held (session {self.session[:8]})")
        else:
            self.error = r.get("message", "refused")
        return self.ok

    def _run(self):
        while not self._stop.is_set():
            t0 = time.monotonic()
            self.renew()
            self._stop.wait(max(0.05, self.period - (time.monotonic() - t0)))

    def stop(self):
        self._stop.set()

    def state(self) -> dict:
        age = None if self.last_ok is None else round(time.monotonic() - self.last_ok, 1)
        return dict(held=self.ok and age is not None and age < self.ttl, session=self.session[:8], last_renewed_s=age,
                    error=self.error or None)
