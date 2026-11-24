"""The voice brain's own tools for a Claude Code session that stays open (server.py --voice; voice_brain_prompt.md):
answer a heard sentence, run and write skills, ask before a plan, remind later. Added to the toolkit's robot_* tools
(toolkit.py) by build_server(voice=True), with two things the plain MCP server doesn't do:

- **Asking first, enforced here (not left to the model):** per heard sentence (the newest in the voice desk), one
  motion may start at once; a second motion, a robot_move of several steps, or the first run of a skill nobody has
  tried is refused until robot_propose_plan came back confirmed ("go").
- **The lease only while acting:** the session stays open all day, so it takes the AI lease when a motion starts and
  lets it go once no AI task or skill has run for IDLE_S; another AI client (a developer's Claude Code session) can
  drive in between.
"""

import json
import sys
import threading
import time
import uuid
from pathlib import Path
from typing import Literal

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import voice_desk as desk  # noqa: E402
from toolkit import FEELINGS, Tool  # noqa: E402

IDLE_S = 20.0            # the lease goes after this long with no AI task or skill running
PLAN_S = 35.0            # a plan waits this long for "go"
FACE_DIR = HERE.parents[1] / "src" / "omni_face"


def _multi(steps) -> bool:
    return isinstance(steps, list) and (len(steps) > 1 or any(isinstance(s, dict) and s.get("do") == "repeat"
                                                              for s in steps))


class VoiceState:
    """The gate per heard sentence, the lease on demand, the skills running."""

    def __init__(self, robot, library=None, log=None):
        self.robot = robot
        self.log = log or (lambda m: None)
        self.library = library
        self.gate_id, self.moves, self.confirmed = None, 0, False
        self.runs: dict = {}
        self.lock = threading.Lock()
        self.timers: list = []
        self.t_busy = time.monotonic()
        self._stop = threading.Event()
        threading.Thread(target=self._idle_watch, name="voice-lease", daemon=True).start()

    def lib(self):
        if self.library is None:
            from skillbox import Library
            self.library = Library()
        return self.library

    # -- the gate ------------------------------------------------------------------------------------------------
    def _sync(self):
        e = desk.latest("say")
        rid = e["id"] if e else None
        if rid != self.gate_id:                          # a new sentence: a fresh count
            self.gate_id, self.moves, self.confirmed = rid, 0, False

    def gate(self, multi: bool = False, needs_ok: bool = False) -> str:
        with self.lock:
            self._sync()
            if needs_ok and not self.confirmed:
                return "This skill is new and nobody has tried it: call robot_propose_plan with its plan first."
            if self.gate_id is not None and not self.confirmed and (self.moves >= 1 or multi):
                return ("Ask first: call robot_propose_plan with the whole plan in one short sentence, and carry on "
                        "only if it comes back confirmed.")
            return ""

    def moved(self):
        with self.lock:
            self.moves += 1
        self.t_busy = time.monotonic()

    def confirm(self):
        with self.lock:
            self._sync()
            self.confirmed = True

    # -- the lease -----------------------------------------------------------------------------------------------
    def ensure_lease(self) -> str:
        r = self.robot
        if r.lease is not None and r.lease.ok:
            return ""
        from lease import LeaseKeeper
        if not r.rb.wait_up(2.0):
            return "The robot isn't reachable (no rosbridge)."
        k = LeaseKeeper(r.rb, r.session, log=self.log)
        k.renew()
        if not k.ok:
            return f"The robot refused the AI lease: {k.error} (another AI client may be in control)."
        r.lease = k.start()
        self.t_busy = time.monotonic()
        return ""

    def _idle_watch(self):
        while not self._stop.wait(2.0):
            r = self.robot
            if r.lease is None:
                continue
            running = any(x["outcome"] is None for x in list(self.runs.values()))
            owner = ((r.status().get("data") or {}).get("ai") or {}).get("owner")
            if running or owner == "ai":
                self.t_busy = time.monotonic()
            elif time.monotonic() - self.t_busy > IDLE_S:
                r.lease.stop()
                r.lease = None
                self.log("voice: lease released (idle)")

    def close(self):
        self._stop.set()


def make_voice_tools(robot, st: VoiceState) -> list:
    T = []

    def tool(**kw):
        def deco(fn):
            T.append(Tool(fn.__name__, fn, **kw))
            return fn
        return deco

    @tool()
    def robot_voice_reply(request_id: str, say: str, feeling: Literal[FEELINGS] = "neutral", intensity: float = 0.5,
                          remember: str = "") -> dict:
        """Answer the person: what the robot says aloud for a heard sentence (its id from the voice event), one or
        two short sentences in character, with how it feels (intensity 0-1). Within ~9 s of the sentence it is the
        voice's own answer; later it is said through the console. remember: a short fact worth keeping about the
        people or the house ("" if none). Call it for every sentence, once you know the outcome."""
        say = str(say).strip()[:300]
        how = "chat reply" if desk.reply(request_id, say, feeling, intensity) else "said through the console"
        if how != "chat reply" and say:
            r = robot.express(feeling, intensity, say) if feeling != "neutral" else robot.say(say)
            if not r.get("ok"):
                return dict(ok=False, message=f"couldn't say it: {r.get('message')}", data=dict(error="no_robot"))
        if remember.strip():
            try:
                if str(FACE_DIR) not in sys.path:
                    sys.path.insert(0, str(FACE_DIR))
                from omni_face.chat import MAX_NOTES, Memory
                m = Memory()
                note = remember.strip()[:120]
                if note not in m.d["notes"]:
                    m.d["notes"] = (m.d["notes"] + [note])[-MAX_NOTES:]
                    m.save()
            except Exception as e:                       # noqa: BLE001
                return dict(ok=True, message=f"{how}; note not kept: {e}", data=dict(how=how))
        return dict(ok=True, message=how, data=dict(how=how))

    @tool()
    def robot_skills() -> dict:
        """The robot's skills: ready routines (come to me, take a picture, dance in a room, check a room...) and the
        ones written before (invented). Each: name, what it does, example phrases, its arguments (JSON schema), moves,
        new (untried: propose before its first run). Read-only."""
        out = []
        for s in sorted(st.lib().all().values(), key=lambda s: (s.builtin, s.name)):
            out.append(dict(name=s.name, description=s.description, says=s.meta.get("says", []),
                            args=s.schema(), moves=s.moves, new=s.needs_ok, builtin=s.builtin))
        return dict(ok=True, message=f"{len(out)} skills", data=dict(skills=out))

    def _run_view(rid: str) -> dict:
        x = st.runs.get(rid)
        if x is None:
            return dict(ok=False, message=f"no skill run {rid}", data=dict(error="unknown_task"))
        state = "RUNNING" if x["outcome"] is None else x["outcome"].upper()
        return dict(ok=True, message=f"{x['name']}: {state.lower()}", data=dict(
            run_id=rid, skill=x["name"], state=state, result=x["result"], said=x["said"],
            s=round(time.monotonic() - x["t0"], 1)))

    @tool()
    def robot_skill(name: str, args: dict | None = None) -> dict:
        """Run a skill (robot_skills) with its arguments. Waits up to 20 s; a longer one returns its run_id with
        state RUNNING: follow it with robot_skill_wait. States end DONE, FAILED, STOPPED (a person stopped the robot:
        stop too) or TIMEOUT; result is the skill's own account."""
        s = st.lib().get(name)
        if s is None:
            return dict(ok=False, message=f"no skill {name}", data=dict(error="not_found"))
        why = st.gate(needs_ok=s.needs_ok) if s.moves else ("" if not s.needs_ok else st.gate(needs_ok=True))
        if not why and s.moves:
            why = st.ensure_lease()
        if why:
            return dict(ok=False, message=why, data=dict(error="ask_first" if "propose" in why else "no_lease"))
        from skillbox import RealAPI, run_skill
        rid = uuid.uuid4().hex[:8]
        stop = threading.Event()
        rec = dict(name=name, outcome=None, result=None, said=[], t0=time.monotonic(), stop=stop)
        st.runs[rid] = rec
        if s.moves:
            st.moved()

        def go():
            out = run_skill(s.code, args or {}, RealAPI(robot, stop, log=st.log), s.max_s, stop=stop)
            rec.update(outcome=out["outcome"], result=out["result"], said=out["said"])
            if out["outcome"] == "done" and s.needs_ok:
                st.lib().mark_tried(name)
            st.t_busy = time.monotonic()
        threading.Thread(target=go, name=f"skill-{name}", daemon=True).start()
        t_end = time.monotonic() + 20.0
        while rec["outcome"] is None and time.monotonic() < t_end:
            time.sleep(0.1)
        return _run_view(rid)

    @tool()
    def robot_skill_wait(run_id: str, wait_s: float = 30.0) -> dict:
        """A skill run's state, waiting up to wait_s (<= 60) for it to end. Call again until it isn't RUNNING."""
        t_end = time.monotonic() + max(0.0, min(60.0, float(wait_s)))
        while time.monotonic() < t_end and (st.runs.get(run_id) or {}).get("outcome", "x") is None:
            time.sleep(0.2)
        return _run_view(run_id)

    @tool()
    def robot_skill_stop(run_id: str) -> dict:
        """End a running skill now (the robot stops what it started)."""
        x = st.runs.get(run_id)
        if x is None:
            return dict(ok=False, message=f"no skill run {run_id}", data=dict(error="unknown_task"))
        x["stop"].set()
        return dict(ok=True, message="stopping", data=dict(run_id=run_id))

    @tool()
    def robot_propose_plan(say: str, wait_s: float = PLAN_S) -> dict:
        """Say a plan aloud in one short sentence ending with "Say go." and wait (up to 35 s) for the person's answer:
        {confirmed: true} on go / yes, else false. Needed before a second motion for one request, a robot_move of
        several steps, or a new skill's first run; the server refuses those until it is confirmed."""
        wait_s = max(5.0, min(60.0, float(wait_s)))
        try:
            robot.rb.publish("/face/say", "std_msgs/msg/String", {"data": json.dumps(dict(
                text=str(say)[:300], feel="curious", intensity=0.4, pending=wait_s))})
        except Exception as e:                           # noqa: BLE001
            return dict(ok=False, message=f"couldn't say the plan: {e}", data=dict(error="no_robot"))
        t0 = time.time()
        desk.plan_open(wait_s)
        try:
            ans = desk.wait_answer(t0, wait_s)
        finally:
            desk.plan_close()
        if ans == "yes":
            st.confirm()
            return dict(ok=True, message="confirmed", data=dict(confirmed=True))
        return dict(ok=True, message="not confirmed", data=dict(confirmed=False,
                                                                why="they said no" if ans else "no answer in time"))

    @tool()
    def robot_save_skill(code: str, request: str) -> dict:
        """Keep a new skill you wrote for a request no skill covers. code: the whole file in the skill format
        (skillbox.py's docstring, its API dict, and skills/*.py as examples). It is checked (what a skill may
        contain) and dry-run in a made-up house first; the reply says what failed, or the new skill's name. It is
        new until it has run once: propose its plan before the first run."""
        from skillbox import FakeAPI, run_skill, sample_args, validate
        meta, bad = validate(code)
        lib = st.lib()
        if not bad and meta.get("name") in lib.all():
            bad = [f"the name {meta['name']} is taken: pick another"]
        if not bad:
            dry = run_skill(code, sample_args(meta), FakeAPI(), max_s=min(20.0, float(meta.get("max_s", 60))))
            if dry["outcome"] != "done":
                bad = [f"a dry run in a made-up house (one person 2 m ahead to the left; places kitchen, living "
                       f"room, home, front door; every motion succeeds) ended {dry['outcome']}: {dry['result']}"]
        if bad:
            return dict(ok=False, message="not saved", data=dict(error="bad_request", problems=bad))
        s = lib.save(code, request)
        return dict(ok=True, message=f"saved {s.name}", data=dict(name=s.name, description=s.description, new=True))

    @tool()
    def robot_remind(minutes: float, text: str) -> dict:
        """Say a line later (0.5-240 minutes), wherever the robot is. Lost if this session ends."""
        minutes = max(0.5, min(240.0, float(minutes)))
        line = str(text)[:180] or "Reminder."
        tm = threading.Timer(minutes * 60.0, lambda: robot.express("focused", 0.6, line))
        tm.daemon = True
        tm.start()
        st.timers = [t for t in st.timers if t.is_alive()] + [tm]
        return dict(ok=True, message=f"in {minutes:g} min", data=dict(minutes=minutes))

    return T


def guard(t: Tool, st: VoiceState) -> Tool:
    """A toolkit motion tool behind the voice gate and the on-demand lease."""
    def run(**kw):
        why = st.gate(multi=t.name == "robot_move" and _multi(kw.get("steps")))
        if not why:
            why = st.ensure_lease()
        if why:
            return dict(ok=False, message=why, data=dict(error="ask_first" if "propose" in why else "no_lease"))
        r = t.fn(**kw)
        if isinstance(r, dict) and r.get("ok"):
            st.moved()
        return r
    import inspect
    run.__signature__ = inspect.signature(t.fn)
    run.__doc__ = t.fn.__doc__
    run.__name__ = t.name
    return Tool(t.name, run, moves=True, image=t.image, mcp=t.mcp, voice=t.voice)
