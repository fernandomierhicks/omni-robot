"""The AI client's verbs and tasks (ai_robot_architecture.md), part of missions.Console. ROS-free."""

from __future__ import annotations

import math

from omni_console.console_defs import (AI_MOVE_MAX, AI_TIMEOUT, AI_TTL, ARM_POSES, FAULTS_STOP_AI, FIND_SHAPES,
                                       FIND_STALE, FW_LIMITS, HUMAN_HOLD, Reply)  # noqa: F401


class AiMixin:
    @staticmethod
    def _err(code: str, message: str, **data) -> Reply:
        return Reply(False, message, dict(error=code, **data))

    def owner(self, now: float):
        """Who has the robot: "ai" (its task runs), "rc", "page" (a person: the page or the voice), or None (free)."""
        if self.ai_book.running() is not None:
            return "ai"
        if self.rc_manual:
            return "rc"
        if self.manual_active(now) or now - self.human_t < HUMAN_HOLD or self.mission not in ("IDLE", "MANUAL"):
            return "page"
        return None

    def _human_owner(self, now: float) -> str:
        if self.rc_manual:
            return "The RC transmitter has manual control"
        if now - self.human_t < HUMAN_HOLD:
            return (f"A person took control {now - self.human_t:.0f} s ago: the AI waits {HUMAN_HOLD:g} s before "
                    f"moving the robot again")
        if self.manual_active(now):
            return "Someone is driving with the console's stick"
        if self.mission not in ("IDLE", "MANUAL"):
            return f"The robot is busy ({self.mission.lower()}, from the console: {self.task}). Stop it there first"
        return ""

    def _fault(self) -> str:
        m = self.mcu or {}
        names = sorted({f["name"] for f in m.get("faults", [])} & set(FAULTS_STOP_AI))
        if m.get("mode") == "ESTOP" and "ESTOP" not in names:
            names.append("ESTOP")
        return f"The MCU reports {', '.join(names)}: a person has to clear it" if names else ""

    def _arm_out(self) -> bool:
        return self._arm_up() and bool(self.arm.get("out"))

    def _ai_gate(self, a: dict, motion: bool = True) -> Reply | None:
        """None when the AI may do this now, else the refusal: no_lease, busy, owner_conflict, fault."""
        now, sess = self.now, str(a.get("ai") or "")
        if not sess:
            return self._err("no_lease", "AI commands carry the AI session (args 'ai')")
        if self.ai is None or self.ai["session"] != sess or now > self.ai["until"]:
            return self._err("no_lease", "No valid AI lease: omni_mcp renews it every second (is it running?)")
        if motion:
            t = self.ai_book.running()
            if t is not None:
                return self._err("busy", f"Task {t.id} ({t.kind}) is still {t.state.lower()}: wait for it "
                                         f"(robot_task) or cancel it", task_id=t.id)
            who = self._human_owner(now)
            if who:
                return self._err("owner_conflict", who)
            bad = self._fault()
            if bad:
                return self._err("fault", bad)
        return None

    def _ai_dup(self, a: dict) -> Reply | None:
        """A retry with a request_id already seen: the same task back, nothing new started."""
        t = self.ai_book.by_req(str(a.get("request_id") or ""))
        if t is None:
            return None
        return Reply(True, f"Already accepted as {t.id} (same request_id): {t.state.lower()}", t.public(self.now))

    def _ai_new(self, kind: str, a: dict, args: dict, timeout: float, mission: str = ""):
        t = self.ai_book.new(kind, args, str(a.get("ai") or "page"), str(a.get("request_id") or ""), self.now,
                             self.wall_time(), self.now + timeout, mission)
        self.ai_book.start(t)
        return t

    @staticmethod
    def _timeout(a: dict, kind: str) -> float:
        default, most = AI_TIMEOUT[kind]
        v = float(a.get("timeout_s") or default)
        if not math.isfinite(v) or v <= 0:
            raise ValueError("timeout_s: a positive number")
        return min(v, most)

    def _ai_evidence(self, t) -> dict:
        if t.kind == "move" and self.runner is not None:
            return self.runner.measured()
        return {}

    def _ai_end_running(self, state: str, why: str):
        t = self.ai_book.running()
        if t is not None:
            self.ai_book.finish(t, state, self.wall_time(), why, evidence=self._ai_evidence(t),
                                result={"error": "timeout"} if state == "TIMED_OUT" else None)
            self.event("warn" if state != "SUCCEEDED" else "info", f"AI task {t.id} ({t.kind}): {why}")

    def _ai_stop(self, state: str, why: str) -> list:
        """End the AI's running task and stop what it was doing (the robot holds still, armed: MANUAL)."""
        self._ai_end_running(state, why)
        return list(self._enter("MANUAL", f"AI stopped: {why}", "warn").effects)

    def _ai_takeover(self, why: str) -> tuple:
        """A person took the robot (the page, the voice, the RC): the AI's task ends, its mission stops."""
        self.human_t = self.now
        if self.ai_book.running() is None:
            return ()
        return tuple(self._ai_stop("CANCELLED", why))

    def on_rc(self, manual: bool):
        """The RC transmitter's manual switch (/mcu/info), from the node."""
        if manual and not self.rc_manual:
            self.fx_queue += list(self._ai_takeover("the RC transmitter took manual control"))
        self.rc_manual = manual

    def _ai_tick(self, now: float) -> list:
        """The lease, the deadlines and the mission under each AI task: what ended them, and the stop."""
        fx = []
        if self.ai is not None and now > self.ai["until"]:
            self.ai = None
            self.event("warn", "The AI client's lease ran out (its connection or its process is gone)")
            if self.ai_book.running() is not None:
                fx += self._ai_stop("CANCELLED", "the AI client's lease ran out")
        t = self.ai_book.running()
        if t is not None and now > t.deadline:
            fx += self._ai_stop("TIMED_OUT", f"no result in {t.deadline - t.t0:.0f} s")
        t = self.ai_book.running()
        if t is not None and t.mission and self.mission != t.mission:
            self._ai_end_running("CANCELLED", f"the robot left {t.mission}: {self.task}")
        f = self.ai_book.running(("find",))
        if f is not None and now > f.deadline:
            self.ai_book.finish(f, "TIMED_OUT", self.wall_time(), "no answer from omni_arm's find",
                                result={"error": "timeout"})
        return fx

    def _ai_lease(self, a: dict) -> Reply:
        sess = str(a.get("session") or a.get("ai") or "")
        if not sess or len(sess) > 64:
            return self._err("bad_request", "ai_lease: a session id (up to 64 characters)")
        ttl = float(a.get("ttl_s", 3.0))
        if not AI_TTL[0] <= ttl <= AI_TTL[1]:
            return self._err("out_of_limits", f"ai_lease: ttl_s {AI_TTL[0]:g} to {AI_TTL[1]:g}")
        now = self.now
        if self.ai is not None and self.ai["session"] != sess and now <= self.ai["until"]:
            return self._err("owner_conflict", f"Another AI session ({self.ai['session'][:8]}) holds the lease")
        if self.ai is None or self.ai["session"] != sess:
            self.event("info", f"AI client connected (session {sess[:8]})")
        self.ai = dict(session=sess, until=now + ttl, ttl=ttl)
        return Reply(True, "lease", dict(session=sess, ttl_s=ttl, owner=self.owner(now)))

    @staticmethod
    def _ceiling(value, cap: float, default: float, what: str):
        """(used, capped): "max" -> the cap; a number -> itself, at most the cap; None -> the default."""
        if value is None:
            return min(default, cap), False
        if value == "max":
            return cap, False
        v = float(value)
        if not math.isfinite(v) or v <= 0:
            raise ValueError(f"{what}: a positive number or \"max\"")
        return min(v, cap), v > cap

    def _ai_move(self, a: dict) -> Reply:
        from omni_console.program import ProgramRunner, check
        dup = self._ai_dup(a)
        if dup:
            return dup
        bad = self._ai_gate(a)
        if bad:
            return bad
        try:
            blocks = check(a.get("steps"))
            runner_check = ProgramRunner("AI move", blocks, 0.1, 0.1)
        except ValueError as e:
            return self._err("bad_request", f"move: {e}")
        kinds = {s[0] for s in runner_check.steps}
        if not runner_check.steps:
            return self._err("bad_request", "move: no steps")
        if "goto" in kinds:
            return self._err("bad_request", "move has no goto blocks: use goto (robot_goto) for places")
        if "move" in kinds and not self.caps["nav"]:
            return self._err("not_localized", "Driving (forward, back, left, right) needs navigation running: its "
                                              "collision monitor guards the moves. Turns and spins in place work "
                                              "without it")
        if self._arm_out():
            return self._err("arm_out", "The arm is out: stow it first (robot_arm_pose stow); turning with it out "
                                        "hits things")
        prof = "kid" if self.values["profile.kid"] else "adult"
        v_fw, w_fw, a_fw = FW_LIMITS[prof]
        v_cap = min(self.values["ai.max_speed"], v_fw)
        w_cap = min(self.values["ai.max_turn"], math.degrees(w_fw))
        v, v_capped = self._ceiling(a.get("speed"), v_cap, self.values["program.speed"], "speed")
        w, w_capped = self._ceiling(a.get("turn_rate"), w_cap, self.values["program.turn"], "turn_rate")
        runner = ProgramRunner("AI move", blocks, v, math.radians(w))
        est = runner.estimate(a_fw, 2.0)
        timeout = float(a.get("timeout_s") or 1.5 * est + 10.0)
        if not math.isfinite(timeout) or timeout <= 0:
            return self._err("bad_request", "timeout_s: a positive number")
        timeout = min(timeout, AI_MOVE_MAX)
        if est > timeout:
            return self._err("out_of_limits", f"move: about {est:.0f} s, longer than its time limit "
                                              f"({timeout:.0f} s, at most {AI_MOVE_MAX:g})")
        r = self._enter("PROGRAM", f"AI move: {len(runner.steps)} steps, about {est:.0f} s", "info")
        self.runner = runner
        t = self._ai_new("move", a, dict(steps=blocks, speed=a.get("speed"), turn_rate=a.get("turn_rate")), timeout,
                         "PROGRAM")
        t.result.update(estimated_s=round(est, 1), limits=dict(
            speed_m_s=round(v, 3), turn_rate_deg_s=round(w, 1), speed_capped=v_capped, turn_rate_capped=w_capped,
            ai_max_speed=self.values["ai.max_speed"], ai_max_turn=self.values["ai.max_turn"], profile=prof,
            firmware_max_speed=v_fw, firmware_max_turn_deg_s=round(math.degrees(w_fw), 1)))
        t.message = f"moving: {len(runner.steps)} steps at {v:g} m/s, {w:.0f} deg/s; about {est:.1f} s"
        return Reply(True, t.message, t.public(self.now), r.effects)

    def _ai_goto(self, a: dict) -> Reply:
        dup = self._ai_dup(a)
        if dup:
            return dup
        bad = self._ai_gate(a)
        if bad:
            return bad
        if not self.caps["nav"]:
            return self._err("not_localized", "Navigation isn't running (Nav2): goto needs it")
        place = str(a.get("place") or "")
        if place:
            if place not in self.places:
                room = next((k for k in self.rooms if k.lower() == place.lower()), None)
                if room is not None:      # a room is an outline, not a pose: the caller picks a free point in it
                    pts = self.rooms[room]
                    cx, cy = sum(p[0] for p in pts) / len(pts), sum(p[1] for p in pts) / len(pts)
                    return self._err("unknown_place", f"'{room}' is a room, not a saved place: goto x, y inside its "
                                                      f"outline (its centre is ({cx:.2f}, {cy:.2f}); check it is free "
                                                      f"on the map first)")
                return self._err("unknown_place", f"No place called '{place}' on this map (places: "
                                                  f"{', '.join(sorted(self.places)) or 'none'}; rooms: "
                                                  f"{', '.join(sorted(self.rooms)) or 'none'})")
            x, y, yaw = self.places[place]
        else:
            x, y, yaw = float(a["x"]), float(a["y"]), math.radians(float(a.get("yaw_deg", 0.0)))
        if self.pose is None or self.pose[0] != "map":
            return self._err("not_localized", "The robot isn't localized on a map")
        h = self.loc_health_now()
        if h is not None and h.get("level") == "LOST":
            return self._err("not_localized", f"Localization is LOST: {'; '.join(h.get('reasons') or [])}")
        if self._arm_out():
            return self._err("arm_out", "The arm is out: stow it first (robot_arm_pose stow)")
        timeout = self._timeout(a, "goto")
        r = self._goto(x, y, yaw, place)
        if not r.ok:
            return r
        t = self._ai_new("goto", a, dict(place=place) if place else dict(x=x, y=y, yaw_deg=math.degrees(yaw)), timeout,
                         "GOTO")
        t.result["goal"] = dict(frame="map", x=round(x, 3), y=round(y, 3), yaw_deg=round(math.degrees(yaw), 1),
                                place=place or None)
        t.message = r.message
        return Reply(True, r.message, t.public(self.now), r.effects)

    def _ai_cancel(self, a: dict) -> Reply:
        """Cancel the AI's task (no lease needed: stopping is always allowed)."""
        t = self.ai_book.get(str(a.get("task_id", "")))
        if t is None:
            return self._err("unknown_task", f"No task '{a.get('task_id')}' (the robot's console may have restarted)")
        if t.done:
            return Reply(True, f"Task {t.id} had already ended: {t.state}", t.public(self.now))
        if t.kind == "find":
            self.ai_book.finish(t, "CANCELLED", self.wall_time(), "cancelled by the AI client")
            return Reply(True, f"Task {t.id} cancelled", t.public(self.now))
        fx = self._ai_stop("CANCELLED", "cancelled by the AI client")
        return Reply(True, f"Task {t.id} cancelled: the robot stopped", t.public(self.now), tuple(fx))

    def _ai_task(self, a: dict) -> Reply:
        t = self.ai_book.get(str(a.get("task_id", "")))
        if t is None:
            return self._err("unknown_task", f"No task '{a.get('task_id')}' (the robot's console may have restarted)")
        return Reply(True, t.message, t.public(self.now))

    def _ai_arm(self, a: dict) -> Reply:
        """The AI's arm: named poses only, on the arm's checked paths (never jog, relax or the gripper alone)."""
        if str(a.get("do", "")) != "pose":
            return self._err("bad_request", "The AI moves the arm to named poses only: do = pose")
        name = str(a.get("name", ""))
        if name not in ARM_POSES:
            return self._err("bad_request", f"arm pose: one of {', '.join(ARM_POSES)}")
        dup = self._ai_dup(a)
        if dup:
            return dup
        bad = self._ai_gate(a)
        if bad:
            return bad
        if not self._arm_up():
            return self._err("no_arm", "The arm isn't connected")
        t = self._ai_new("arm_pose", a, dict(name=name), self._timeout(a, "arm_pose"))
        t.result.update(pose=name, arm_ev=self.arm_ev)
        t.message = f"arm to {name}"
        return Reply(True, f"Arm to {name}", t.public(self.now), (("arm", dict(verb="pose", name=name)),))

    def _ai_find(self, a: dict) -> Reply:
        """Candidates on the floor by colour and shape (omni_arm: floor_vision's detections, colour_vision's labels).
        Read-only: no lease, no owner. The result comes with /arm/state (on_arm_state)."""
        if self.ai_book.running(("find",)) is not None:
            return self._err("busy", "A find is already running")
        if not self._arm_up():
            return self._err("no_arm", "find needs the arm robot running (omni_arm's arm_node)")
        color = a.get("color")
        if isinstance(color, dict):
            h = color.get("h")
            if not (isinstance(h, list) and len(h) == 2 and all(0 <= float(v) <= 179 for v in h)):
                return self._err("bad_request", "color: {h: [lo, hi] (0-179, OpenCV), s_min, v_min} or a name")
            color = dict(h=[float(h[0]), float(h[1])], s_min=float(color.get("s_min", 80)),
                         v_min=float(color.get("v_min", 50)))
        elif color is not None:
            color = str(color).strip().lower()[:20] or None
        shape = str(a.get("shape") or "any").lower()
        if shape not in FIND_SHAPES:
            return self._err("bad_request", f"shape: one of {', '.join(FIND_SHAPES)}")
        rng = float(a.get("max_range_m", 1.6))
        if not 0.3 <= rng <= 2.0:
            return self._err("out_of_limits", "max_range_m: 0.3 to 2.0 m (the depth camera's useful range)")
        t = self._ai_new("find", a, dict(color=color, shape=shape, max_range_m=rng), self._timeout(a, "find"))
        t.message = "looking"
        req = dict(verb="find", id=t.id, color=color, shape=shape, max_range=rng)
        return Reply(True, "Looking", t.public(self.now), (("arm", req),))

    def _ai_pick(self, a: dict) -> Reply:
        """The tidy, for one object of the last find (its id in omni_arm's floor map): approach, wrist look, grasp,
        the bin."""
        dup = self._ai_dup(a)
        if dup:
            return dup
        bad = self._ai_gate(a)
        if bad:
            return bad
        if not self._arm_up():
            return self._err("no_arm", "pick needs the arm robot running (omni_arm's arm_node)")
        if self._arm_out():
            return self._err("arm_out", "The arm isn't stowed: robot_arm_pose stow first")
        oid = int(a["object_id"])
        f = self.found
        if f is None:
            return self._err("stale_object", "No find yet: robot_find first")
        if self.found_epoch != self.loc_epoch:
            return self._err("stale_object", "The map or the localization changed since the find: find again")
        age = self.now - self.t_found
        if age > FIND_STALE:
            return self._err("stale_object", f"The last find is {age:.0f} s old (at most {FIND_STALE:g}): find again")
        cands = {int(c["id"]): c for c in f.get("candidates", [])}
        if oid not in cands:
            return self._err("not_found", f"No object {oid} in the last find (ids: "
                                          f"{', '.join(str(i) for i in sorted(cands)) or 'none'})")
        timeout = self._timeout(a, "pick")
        r = self._enter("TIDY", f"AI pick: object {oid}", "info")
        radius = max(float(self.values["tidy.radius"]), float(cands[oid].get("range_m", 1.0)) + 0.5)
        req = dict(verb="tidy", only=oid, max_objects=1, radius=radius)
        t = self._ai_new("pick", a, dict(object_id=oid), timeout, "TIDY")
        t.result["object"] = cands[oid]
        t.message = f"picking object {oid}"
        return Reply(True, f"Picking object {oid}", t.public(self.now), r.effects + (("arm", req),))

    def _look(self, a: dict) -> Reply:
        """A fresh frame from a camera the sim renders on request (front: the depth camera's colour, wrist). On the
        robot the cameras stream and the request is ignored."""
        cam = str(a.get("camera", "front"))
        what = {"front": "color", "d435_color": "color", "wrist": "wrist", "depth": "depth"}.get(cam)
        if what is None:
            return self._err("bad_request", "look: camera front | wrist | d435_color | depth")
        return Reply(True, f"{cam} frame requested", dict(camera=cam), (("camera_request", what),))

    def _ai_on_arm(self, s: dict, new_events: list):
        """/arm/state for the AI's tasks: a find's answer, an arm pose's end."""
        f = s.get("found")
        if isinstance(f, dict) and f.get("id"):
            t = self.ai_book.get(str(f["id"]))
            if t is not None and t.kind == "find" and not t.done:
                if f.get("error"):
                    self.ai_book.finish(t, "FAILED", self.wall_time(), f["error"], result=dict(error="not_found"))
                else:
                    self.found, self.t_found, self.found_epoch = f, self.now, self.loc_epoch
                    n = len(f.get("candidates", []))
                    self.ai_book.finish(t, "SUCCEEDED", self.wall_time(), f"{n} candidate{'s' * (n != 1)}",
                                        result={k: f.get(k) for k in ("candidates", "image_only", "query", "image",
                                                                       "frame", "t", "dropped", "hint", "pose")})
        t = self.ai_book.running(("arm_pose",))
        if t is None:
            return
        name = t.result["pose"]
        for text in new_events:
            if text.startswith(f"{name}: refused") or text.startswith(f"pose {name}:") or "isn't connected" in text:
                self.ai_book.finish(t, "FAILED", self.wall_time(), text, result=dict(error="arm_out"))
                return
            if text == f"moving to {name}":
                t.result["moving"] = True
        if t.result.get("moving") and s.get("source") == "hold":
            self.ai_book.finish(t, "SUCCEEDED", self.wall_time(), f"the arm is at {name}",
                                evidence=dict(joints_deg=s.get("joints"), out=s.get("out")))

    def _ai_pick_end(self, tidy: dict):
        """The tidy ended under an AI pick: its stages are the evidence (the robot has no bin check yet)."""
        t = self.ai_book.running(("pick",))
        if t is None:
            return
        stages = tidy.get("stages") or {}
        outcome = tidy.get("outcome") or tidy.get("reason") or ""
        ok = bool(stages.get("released"))
        err = None if ok else {"not_found": "not_found", "lost": "lost", "too_big": "out_of_limits"}.get(outcome,
                                                                                                         "failed")
        ev = dict(attempted=bool(stages.get("attempted")), grasped=bool(stages.get("grasped")), released=ok,
                  in_bin="uncertain", in_bin_reason="the robot has no bin check yet (released over the bin is what "
                                                    "it knows; the sim's truth comes from omni_mcp)",
                  outcome=outcome, tries=tidy.get("tries"))
        msg = f"object {t.args['object_id']}: " + ("released over the bin" if ok else (outcome or "not picked"))
        self.ai_book.finish(t, "SUCCEEDED" if ok else "FAILED", self.wall_time(), msg,
                            result=dict(error=err, in_bin="uncertain"), evidence=ev)

    def ai_cleanup(self) -> set:
        """What a restarted console must stop, for the AI tasks it found interrupted: "nav" (Nav2 keeps driving a goal
        whose client died), "arm" (omni_arm keeps running a tidy). Nothing resumes; the robot stops."""
        kinds = {t.kind for t in self.ai_book.tasks.values() if t.state == "INTERRUPTED" and t.t_end is None}
        return ({"nav"} if "goto" in kinds else set()) | ({"arm"} if "pick" in kinds else set())

    def ai_summary(self) -> dict:
        now = self.now
        last = next(reversed(self.ai_book.tasks.values()), None)
        live = self.ai is not None and now <= self.ai["until"]
        return dict(lease=live, session=self.ai["session"][:8] if live else None, owner=self.owner(now),
                    task=None if last is None else dict(task_id=last.id, kind=last.kind, state=last.state,
                                                        message=last.message),
                    found=None if self.found is None else dict(id=self.found.get("id"),
                                                                n=len(self.found.get("candidates", [])),
                                                                age_s=round(now - self.t_found, 1)),
                    limits=dict(max_speed=self.values["ai.max_speed"], max_turn=self.values["ai.max_turn"]))
