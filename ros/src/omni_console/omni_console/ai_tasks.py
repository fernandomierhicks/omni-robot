"""The AI's tasks (ai_robot_architecture.md 6), ROS-free: what an AI client (omni_mcp: Claude, Codex) asked the robot
to do and how it ended. missions.py starts and ends them; this keeps the records.

  ACCEPTED -> RUNNING -> SUCCEEDED | FAILED | CANCELLED | TIMED_OUT,  and INTERRUPTED after a restart

A task is the record of one console mission the AI started (PROGRAM for move, GOTO, TIDY for pick) or of one find; it
is not a second mission engine. It ends when its mission ends, with the evidence the robot has (the turn it measured,
Nav2's result and the pose, the pick's stages). A rejected request never becomes a task. A retry with the same
request_id gets the same task back, never a second motion. The node saves the book (ai_tasks.json): after a restart
the tasks that were running become INTERRUPTED, and nothing resumes by itself.
"""

from __future__ import annotations

import math
from collections import OrderedDict
from dataclasses import asdict, dataclass, field

STATES = ("ACCEPTED", "RUNNING", "SUCCEEDED", "FAILED", "CANCELLED", "TIMED_OUT", "INTERRUPTED")
TERMINAL = ("SUCCEEDED", "FAILED", "CANCELLED", "TIMED_OUT", "INTERRUPTED")
KEEP = 50                          # records kept (the newest)
MOTION = ("move", "goto", "pick", "arm_pose")      # kinds that drive the base or the arm: one at a time
# the reply's error codes (Reply.data["error"]): ai_robot_architecture.md 6
ERRORS = ("no_lease", "busy", "owner_conflict", "arm_out", "not_localized", "unknown_place", "not_found", "lost",
          "out_of_limits", "fault", "stale_object", "timeout", "no_arm", "bad_request", "unknown_task")


@dataclass
class Task:
    id: str
    request_id: str
    kind: str                       # move | goto | pick | arm_pose | find
    args: dict
    owner: str                      # the AI session that asked
    state: str = "ACCEPTED"
    mission: str = ""               # the console mission it runs as ("" for find)
    t_accepted: float = 0.0         # wall clock (s since the epoch)
    t_end: float | None = None
    deadline: float = math.inf      # monotonic (the node's clock): not saved
    t0: float = 0.0                 # monotonic, when accepted
    message: str = ""
    result: dict = field(default_factory=dict)
    evidence: dict = field(default_factory=dict)

    @property
    def done(self) -> bool:
        return self.state in TERMINAL

    def public(self, now: float | None = None) -> dict:
        d = dict(task_id=self.id, request_id=self.request_id, kind=self.kind, args=self.args, state=self.state,
                 message=self.message, result=self.result, evidence=self.evidence,
                 t_accepted=round(self.t_accepted, 2), t_end=None if self.t_end is None else round(self.t_end, 2))
        if now is not None and not self.done:
            d["running_s"] = round(now - self.t0, 1)
            if math.isfinite(self.deadline):
                d["deadline_in_s"] = round(self.deadline - now, 1)
        return d


def log_fields(rec: dict) -> dict:
    """A finished task's record for the session log (session_log "ai_task"): its kind as task_kind (the log's own
    write() takes the record's type as kind)."""
    return {("task_kind" if k == "kind" else k): v for k, v in rec.items()}


class TaskBook:
    def __init__(self, saved: dict | None = None):
        self.tasks: OrderedDict[str, Task] = OrderedDict()
        self.by_request: dict[str, str] = {}
        self.n = 0
        self.changed = False
        self.finished: list = []                       # records to log (the node writes them to the session log)
        for d in (saved or {}).get("tasks", []):
            try:
                t = Task(**{k: v for k, v in d.items() if k in Task.__dataclass_fields__ and k != "deadline"})
            except TypeError:
                continue
            if not t.done:
                t.state, t.message = "INTERRUPTED", "the robot's console restarted while it ran: it did not resume"
                self.changed = True
            self._add(t)
            try:
                self.n = max(self.n, int(t.id.split("-")[-1]))
            except ValueError:
                pass

    def _add(self, t: Task):
        self.tasks[t.id] = t
        if t.request_id:
            self.by_request[t.request_id] = t.id
        while len(self.tasks) > KEEP:
            old_id, old = self.tasks.popitem(last=False)
            if self.by_request.get(old.request_id) == old_id:
                del self.by_request[old.request_id]

    def new(self, kind: str, args: dict, owner: str, request_id: str, now: float, wall: float, deadline: float,
            mission: str = "") -> Task:
        self.n += 1
        t = Task(id=f"t-{self.n:04d}", request_id=request_id or f"auto-{self.n:04d}", kind=kind, args=args,
                 owner=owner, mission=mission, t_accepted=wall, t0=now, deadline=deadline)
        self._add(t)
        self.changed = True
        return t

    def get(self, task_id: str) -> Task | None:
        return self.tasks.get(task_id)

    def by_req(self, request_id: str) -> Task | None:
        i = self.by_request.get(request_id) if request_id else None
        return self.tasks.get(i) if i else None

    def running(self, kinds=MOTION) -> Task | None:
        """The task of these kinds that hasn't ended (one at a time for the motion kinds)."""
        for t in reversed(self.tasks.values()):
            if t.kind in kinds and not t.done:
                return t
        return None

    def start(self, t: Task, message: str = ""):
        t.state = "RUNNING"
        t.message = message or t.message
        self.changed = True

    def finish(self, t: Task, state: str, wall: float, message: str = "", result: dict | None = None,
               evidence: dict | None = None):
        if t.done:
            return
        assert state in TERMINAL, state
        t.state, t.t_end = state, wall
        t.message = message or t.message
        if result:
            t.result.update(result)
        if evidence:
            t.evidence.update(evidence)
        self.changed = True
        self.finished.append(t.public())

    def pop_finished(self) -> list:
        out, self.finished = self.finished, []
        return out

    def saved(self) -> dict:
        return dict(tasks=[{k: v for k, v in asdict(t).items() if k not in ("deadline", "t0")}
                           for t in self.tasks.values()])
