"""The AI client's verbs in the console's core (ai_robot_architecture.md 3.2, 6, 7): the lease, tasks, move, goto,
find and pick, owners and take-over, deadlines, restarts. ROS-free. Run from the repo root:
    & $py -m pytest -q ros/src/omni_console/test
"""

import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from omni_console.ai_tasks import TaskBook                                    # noqa: E402
from omni_console.missions import HUMAN_HOLD, ZERO_TAIL, Console             # noqa: E402
from omni_console.program import ProgramRunner                               # noqa: E402

S = "sess-1"


def ready(nav=True, t=100.0) -> Console:
    c = Console()
    c.now = t
    c.on_robot_mode("TELEOP adult")
    c.set_caps(nav=nav, catch=False, collision=False)
    c.odom_pose = (0.0, 0.0, 0.0)
    c.pose = ("map", 0.0, 0.0, 0.0)
    lease(c)
    return c


def lease(c: Console, sess: str = S, ttl: float = 3.0):
    r = c.command("ai_lease", dict(session=sess, ttl_s=ttl))
    assert r.ok, r.message
    return r


def ai(c: Console, verb: str, **a):
    return c.command(verb, dict(a, ai=S))


def drive(c: Console, seconds: float, dt: float = 0.05, renew: bool = True, until=None):
    """Integrate the odometry from drive_output's twist (a perfect base), renewing the lease each second."""
    t_end = c.now + seconds
    sent = []
    while c.now < t_end:
        now = c.now + dt
        if renew and int(now) != int(now - dt):
            c.now = now
            lease(c)
        tw = c.drive_output(now)
        sent.append(tw)
        if tw is not None:
            x, y, yaw = c.odom_pose
            vx, vy, wz = tw
            x += (vx * math.cos(yaw) - vy * math.sin(yaw)) * dt
            y += (vx * math.sin(yaw) + vy * math.cos(yaw)) * dt
            c.odom_pose = (x, y, yaw + wz * dt)
        if until is not None and until(c):
            break
    return sent


def test_motion_needs_the_lease():
    c = ready()
    r = c.command("move", dict(steps=[{"do": "spin", "turns": 1}]))
    assert not r.ok and r.data["error"] == "no_lease"
    r = c.command("move", dict(steps=[{"do": "spin", "turns": 1}], ai="someone-else"))
    assert not r.ok and r.data["error"] == "no_lease"
    c.now += 3.5                                                    # the lease ran out
    r = ai(c, "move", steps=[{"do": "spin", "turns": 1}])
    assert not r.ok and r.data["error"] == "no_lease"
    lease(c)
    assert ai(c, "move", steps=[{"do": "spin", "turns": 1}]).ok


def test_another_session_cant_take_a_live_lease():
    c = ready()
    r = c.command("ai_lease", dict(session="other", ttl_s=3))
    assert not r.ok and r.data["error"] == "owner_conflict"
    c.now += 3.1
    assert c.command("ai_lease", dict(session="other", ttl_s=3)).ok


def test_spin_five_times_as_fast_as_allowed():
    c = ready()
    r = ai(c, "move", steps=[{"do": "spin", "turns": 5}], turn_rate="max")
    assert r.ok, r.message
    lim = r.data["result"]["limits"]
    assert lim["turn_rate_deg_s"] == 180.0 and lim["profile"] == "adult"
    assert c.mission == "PROGRAM" and c.owner(c.now) == "ai"
    est = r.data["result"]["estimated_s"]
    assert 10.0 < est < 13.0                                         # 1800 deg at 180 deg/s, plus the ends
    tid = r.data["task_id"]
    drive(c, 30.0, until=lambda c: c.mission != "PROGRAM")
    t = c.command("task", dict(task_id=tid)).data
    assert t["state"] == "SUCCEEDED", t
    assert abs(t["evidence"]["turned_deg"] - 1800.0) < 2.0
    assert abs(t["evidence"]["duration_s"] - est) < 0.15 * est     # the estimate is good to 15% (smoke test's bar)
    assert t["evidence"]["max_turn_rate_deg_s"] <= 180.5
    assert c.mission == "MANUAL" and c.owner(c.now) is None


def test_the_ceilings_cap_a_request_and_say_so():
    c = ready()
    c.command("set", dict(key="ai.max_turn", value=90))
    r = ai(c, "move", steps=[{"do": "turn", "deg": 90}], turn_rate=200, speed=3.0)
    lim = r.data["result"]["limits"]
    assert lim["turn_rate_deg_s"] == 90.0 and lim["turn_rate_capped"] and lim["speed_capped"]
    c2 = ready()
    c2.command("set", dict(key="profile.kid", value=True))
    r = ai(c2, "move", steps=[{"do": "spin", "turns": 1}], turn_rate="max")
    assert abs(r.data["result"]["limits"]["turn_rate_deg_s"] - math.degrees(2.0)) < 0.1   # the kid firmware's


def test_a_retry_with_the_same_request_id_is_the_same_task():
    c = ready()
    r1 = ai(c, "move", steps=[{"do": "spin", "turns": 1}], request_id="abc")
    r2 = ai(c, "move", steps=[{"do": "spin", "turns": 1}], request_id="abc")
    assert r1.ok and r2.ok and r1.data["task_id"] == r2.data["task_id"]
    assert r2.effects == ()                                          # nothing started twice
    r3 = ai(c, "move", steps=[{"do": "spin", "turns": 1}], request_id="def")
    assert not r3.ok and r3.data["error"] == "busy"


def test_bad_moves_are_refused_with_a_reason():
    c = ready(nav=False)
    assert ai(c, "move", steps=[{"do": "forward", "m": 1}]).data["error"] == "not_localized"
    assert ai(c, "move", steps=[{"do": "goto", "place": "x"}]).data["error"] == "bad_request"
    assert ai(c, "move", steps=[{"do": "spin", "turns": 50}]).data["error"] == "bad_request"
    assert ai(c, "move", steps=[{"do": "spin", "turns": 1}], turn_rate=-5).data["error"] == "bad_request"
    c.on_arm_state(dict(connected=True, out=True), c.now)
    assert ai(c, "move", steps=[{"do": "spin", "turns": 1}]).data["error"] == "arm_out"


def test_a_person_takes_over_and_the_ai_waits():
    c = ready()
    r = ai(c, "move", steps=[{"do": "spin", "turns": 3}], turn_rate="max")
    drive(c, 1.0)
    r2 = c.command("mission", dict(name="MANUAL"))                 # the page's Take control
    assert r2.ok and c.mission == "MANUAL" and c.runner is None
    t = c.command("task", dict(task_id=r.data["task_id"])).data
    assert t["state"] == "CANCELLED" and "took over" in t["message"]
    assert t["evidence"]["turned_deg"] > 10.0                        # how far it got
    r3 = ai(c, "move", steps=[{"do": "spin", "turns": 1}])
    assert not r3.ok and r3.data["error"] == "owner_conflict" and c.owner(c.now) == "page"
    c.now += HUMAN_HOLD + 0.1
    lease(c)
    assert ai(c, "move", steps=[{"do": "spin", "turns": 1}]).ok


def test_the_stick_or_a_console_mission_blocks_the_ai():
    c = ready()
    c.manual_input(0.5, 0.0, 0.0, False, c.now)
    assert ai(c, "move", steps=[{"do": "spin", "turns": 1}]).data["error"] == "owner_conflict"
    c2 = ready()
    c2.human_t = -1e9
    c2.mission = "FOLLOW"
    assert ai(c2, "move", steps=[{"do": "spin", "turns": 1}]).data["error"] == "owner_conflict"


def test_the_rc_switch_takes_over():
    c = ready()
    r = ai(c, "move", steps=[{"do": "spin", "turns": 2}])
    c.on_rc(True)
    assert c.mission == "MANUAL"
    assert c.ai_book.get(r.data["task_id"]).state == "CANCELLED"
    assert ai(c, "move", steps=[{"do": "spin", "turns": 1}]).data["error"] == "owner_conflict"


def test_a_lapsed_lease_stops_the_robot():
    c = ready()
    r = ai(c, "move", steps=[{"do": "spin", "turns": 5}], turn_rate="max")
    drive(c, 2.0)                                                    # spinning, lease renewed
    t_renewed = c.ai["until"] - c.ai["ttl"]
    t_cut = c.now
    sent = drive(c, 4.0, renew=False)                                # the MCP process died
    t = c.ai_book.get(r.data["task_id"])
    assert t.state == "CANCELLED" and "lease" in t.message
    assert c.mission == "MANUAL" and c.ai is None
    k = next(i for i, tw in enumerate(sent) if tw is not None and tw[2] == 0.0)
    t_stop = t_cut + 0.05 * (k + 1)
    assert t_stop - t_renewed <= 3.0 + 0.06                          # stopped within the TTL of the last renewal
    zeros = [tw for tw in sent[k:] if tw is not None]
    assert zeros and all(tw == (0.0, 0.0, 0.0) for tw in zeros)       # zeros for the tail, then silence
    assert len(zeros) * 0.05 <= ZERO_TAIL + 0.1


def test_a_task_past_its_deadline_times_out_and_stops():
    c = ready()
    r = ai(c, "move", steps=[{"do": "wait", "s": 20}], timeout_s=21)
    assert r.ok
    assert ai(c, "move", steps=[{"do": "wait", "s": 20}], timeout_s=5).data["error"] in ("busy", "out_of_limits")
    drive(c, 22.0)
    t = c.ai_book.get(r.data["task_id"])
    assert t.state in ("SUCCEEDED", "TIMED_OUT")
    c2 = ready()
    r = ai(c2, "move", steps=[{"do": "wait", "s": 20}], timeout_s=21)
    c2.ai_book.get(r.data["task_id"]).deadline = c2.now + 1.0      # a blocked move: past its deadline
    drive(c2, 2.0)
    t = c2.ai_book.get(r.data["task_id"])
    assert t.state == "TIMED_OUT" and t.result["error"] == "timeout" and c2.mission == "MANUAL"


def test_stop_and_cancel_need_no_lease():
    c = ready()
    r = ai(c, "move", steps=[{"do": "spin", "turns": 2}])
    c.ai = None                                                      # whatever happened to the lease
    rc = c.command("cancel", dict(task_id=r.data["task_id"], ai=S))
    assert rc.ok and rc.data["state"] == "CANCELLED" and c.mission == "MANUAL"
    lease(c)
    c.human_t = -1e9
    r = ai(c, "move", steps=[{"do": "spin", "turns": 2}])
    c.ai = None
    assert ai(c, "stop").ok and c.mission == "IDLE"
    assert c.ai_book.get(r.data["task_id"]).state == "CANCELLED"
    assert c.command("task", dict(task_id="t-9999")).data["error"] == "unknown_task"


def test_goto_a_saved_place_with_evidence():
    c = ready()
    c.places["dock"] = [2.0, 1.0, 0.0]
    assert ai(c, "goto", place="kitchen").data["error"] == "unknown_place"
    c.rooms["kitchen"] = [[3.0, -5.0], [9.0, -5.0], [9.0, -1.0], [3.0, -1.0]]
    r = ai(c, "goto", place="Kitchen")        # a room named: refused, with its centre for the caller's own goal
    assert r.data["error"] == "unknown_place" and "is a room" in r.message and "(6.00, -3.00)" in r.message
    r = ai(c, "goto", place="dock")
    assert r.ok and c.mission == "GOTO"
    assert any(fx[0] == "nav_goal" for fx in r.effects)
    c.pose = ("map", 1.98, 1.01, 0.02)
    c.on_nav_done(c.goal["seq"], "succeeded", c.now + 5)
    t = c.ai_book.get(r.data["task_id"])
    assert t.state == "SUCCEEDED" and t.evidence["nav2"] == "succeeded"
    assert t.evidence["distance_to_goal_m"] < 0.05
    r = ai(c, "goto", x=5.0, y=0.0, yaw_deg=90)
    c.on_nav_done(c.goal["seq"], "aborted", c.now + 5)
    t = c.ai_book.get(r.data["task_id"])
    assert t.state == "FAILED" and t.result["error"] == "nav2_aborted"


def test_goto_needs_navigation_and_a_map_pose():
    c = ready(nav=False)
    assert ai(c, "goto", x=1, y=1).data["error"] == "not_localized"
    c2 = ready()
    c2.pose = ("odom", 0.0, 0.0, 0.0)
    assert ai(c2, "goto", x=1, y=1).data["error"] == "not_localized"


def arm_state(**kw):
    s = dict(connected=True, out=False, source="hold", events=[], tidy=None, tidy_on=False)
    s.update(kw)
    return s


FOUND = dict(id=None, candidates=[dict(id=4, x=1.0, y=0.2, range_m=0.9, score=0.93, shape={"class": "square"},
                                       color={"name": "purple"}),
                                  dict(id=6, x=1.1, y=-0.3, range_m=1.0, score=0.2)], image_only=[])


def test_find_then_pick_one_object():
    c = ready()
    c.on_arm_state(arm_state(), c.now)
    r = ai(c, "find", color="purple", shape="square")
    assert r.ok and r.effects[0][0] == "arm" and r.effects[0][1]["verb"] == "find"
    fid = r.data["task_id"]
    assert r.effects[0][1]["id"] == fid
    c.on_arm_state(arm_state(found=dict(FOUND, id=fid, hint="6: move closer", pose=[0, 0, 0])), c.now + 0.5)
    t = c.ai_book.get(fid)
    assert t.state == "SUCCEEDED" and t.result["candidates"][0]["id"] == 4
    assert t.result["hint"] == "6: move closer" and t.result["pose"] == [0, 0, 0]     # all of it reaches the AI
    assert ai(c, "pick", object_id=9).data["error"] == "not_found"
    r = ai(c, "pick", object_id=4)
    assert r.ok and c.mission == "TIDY"
    req = [fx[1] for fx in r.effects if fx[0] == "arm"][0]
    assert req["verb"] == "tidy" and req["only"] == 4 and req["max_objects"] == 1
    pid = r.data["task_id"]
    c.on_arm_state(arm_state(tidy_on=True, tidy=dict(state="APPROACH")), c.now + 1)
    c.on_arm_state(arm_state(tidy_on=False, tidy=dict(state="DONE", outcome="picked", stages=dict(
        attempted=True, grasped=True, released=True))), c.now + 20)
    t = c.ai_book.get(pid)
    assert t.state == "SUCCEEDED" and t.evidence["released"] and t.evidence["in_bin"] == "uncertain"
    assert c.mission == "MANUAL"


def test_a_pick_that_loses_its_object_fails_with_the_reason():
    c = ready()
    c.on_arm_state(arm_state(), c.now)
    fid = ai(c, "find", color="purple").data["task_id"]
    c.on_arm_state(arm_state(found=dict(FOUND, id=fid)), c.now)
    pid = ai(c, "pick", object_id=4).data["task_id"]
    c.on_arm_state(arm_state(tidy_on=True, tidy=dict(state="LOOK")), c.now + 1)
    c.on_arm_state(arm_state(tidy_on=False, tidy=dict(state="DONE", outcome="lost", stages=dict(attempted=False))),
                   c.now + 5)
    t = c.ai_book.get(pid)
    assert t.state == "FAILED" and t.result["error"] == "lost"


def test_a_find_goes_stale():
    c = ready()
    c.on_arm_state(arm_state(), c.now)
    fid = ai(c, "find", color="purple").data["task_id"]
    c.on_arm_state(arm_state(found=dict(FOUND, id=fid)), c.now)
    c.now += 31
    c.on_arm_state(arm_state(), c.now)
    lease(c)
    assert ai(c, "pick", object_id=4).data["error"] == "stale_object"
    fid = ai(c, "find", color="purple").data["task_id"]
    c.on_arm_state(arm_state(found=dict(FOUND, id=fid)), c.now)
    c.slam["mode"] = "localization"
    c.command("initial_pose", dict(x=0, y=0, yaw=0, ai=S))           # localization reset
    assert ai(c, "pick", object_id=4).data["error"] == "stale_object"


def test_find_needs_the_arm_and_answers_or_times_out():
    c = ready()
    assert ai(c, "find", color="purple").data["error"] == "no_arm"
    c.on_arm_state(arm_state(), c.now)
    assert ai(c, "find", shape="triangle").data["error"] == "bad_request"
    r = ai(c, "find", color={"h": [126, 160], "s_min": 60})
    assert r.ok and r.effects[0][1]["color"]["h"] == [126.0, 160.0]
    assert ai(c, "find", color="red").data["error"] == "busy"
    c.on_arm_state(arm_state(), c.now + 16)
    c.drive_output(c.now + 16)
    assert c.ai_book.get(r.data["task_id"]).state == "TIMED_OUT"


def test_arm_pose_task_follows_the_arms_events():
    c = ready()
    c.on_arm_state(arm_state(), c.now)
    assert ai(c, "arm", do="relax").data["error"] == "bad_request"   # never torque off from the AI
    r = ai(c, "arm", do="pose", name="ready")
    assert r.ok and r.effects == (("arm", dict(verb="pose", name="ready")),)
    ev = [dict(n=1, text="moving to ready")]
    c.on_arm_state(arm_state(source="pose", events=ev), c.now + 0.2)
    assert c.ai_book.get(r.data["task_id"]).state == "RUNNING"
    c.on_arm_state(arm_state(source="hold", events=ev, joints=[0, -30, 30, 80, 0, 0]), c.now + 2)
    t = c.ai_book.get(r.data["task_id"])
    assert t.state == "SUCCEEDED" and t.evidence["joints_deg"][1] == -30


def test_a_restart_interrupts_running_tasks():
    c = ready()
    r = ai(c, "move", steps=[{"do": "spin", "turns": 2}])
    saved = c.ai_book.saved()
    c2 = Console(ai_tasks=saved)
    t = c2.ai_book.get(r.data["task_id"])
    assert t.state == "INTERRUPTED" and c2.mission == "IDLE"
    assert c2.ai_book.new("move", {}, S, "", 0, 0, 1).id != r.data["task_id"]       # ids aren't reused
    assert c2.ai_cleanup() == set()                                  # a move stops by itself (the MCU's watchdog)
    c3 = ready()
    c3.on_arm_state(arm_state(), c3.now)
    ai(c3, "goto", x=1.0, y=0.0)
    c4 = Console(ai_tasks=c3.ai_book.saved())
    assert c4.ai_cleanup() == {"nav"}                                # Nav2 would drive on: the restart cancels it


def test_a_finished_task_goes_into_the_session_log(tmp_path):
    from omni_console.ai_tasks import log_fields
    from omni_console.session_log import SessionLog, read
    c = ready()
    r = ai(c, "move", steps=[{"do": "spin", "turns": 1}])
    drive(c, 10.0, until=lambda c: c.mission != "PROGRAM")
    recs = c.ai_book.pop_finished()
    assert [x["task_id"] for x in recs] == [r.data["task_id"]]
    log = SessionLog(tmp_path, "PROGRAM", source="sim")
    for rec in recs:
        log.write("ai_task", **log_fields(rec))                     # as mission_node.save does
    log.close()
    _, rows = read(log.path)
    row = [x for x in rows if x["type"] == "ai_task"][0]
    assert row["task_kind"] == "move" and row["state"] == "SUCCEEDED"


def test_rooms_ride_with_the_map_and_tf_answers_read_only():
    from omni_console.tflite import TfLite
    c = ready()
    c.set_map("house", {}, {}, rooms={"kitchen": [[3, -5.7], [9, -5.7], [9, -0.8]], "bad": [[0, 0]]})
    assert list(c.rooms) == ["kitchen"] and c.schema()["rooms"]["kitchen"][0] == [3.0, -5.7]
    assert c.command("room_save", dict(name="office", points=[[2, 0.8], [5, 0.8], [5, 3.2], [2, 3.2]])).ok
    assert "office" in c.extras()["rooms"] and c.places_changed
    assert c.command("tf", dict(parent="map", child="base_link", ai=S)).data["error"] == "no_robot"
    tf = TfLite()
    tf.tf = {"odom": ("map", 0.0, 1.0, 2.0, 0.0), "base_link": ("odom", 5.0, 0.5, 0.0, math.pi / 2)}
    c.tf = tf
    r = c.command("tf", dict(parent="map", child="base_link", ai=S))
    assert r.ok and r.data["x"] == 1.5 and r.data["yaw_deg"] == 90.0
    assert c.command("frames", {}).data["frames"] == {"base_link": "odom", "odom": "map"}
    assert c.command("tf", dict(parent="base_link", child="map")).data["error"] == "not_found"


def test_status_and_schema_say_who_has_the_robot():
    c = ready()
    s = c.summary()["ai"]
    assert s["lease"] and s["owner"] is None and s["task"] is None
    ai(c, "move", steps=[{"do": "spin", "turns": 1}])
    s = c.summary()["ai"]
    assert s["owner"] == "ai" and s["task"]["state"] == "RUNNING"
    sch = c.schema()["ai"]
    assert "move" in sch["verbs"] and sch["firmware"]["adult"]["max_turn_deg_s"] == 229.2


def test_runner_measures_and_estimates():
    c = ready()
    r = ai(c, "move", steps=[{"do": "forward", "m": 1.0}, {"do": "turn", "deg": 90}], speed=0.5, turn_rate=90)
    est = r.data["result"]["estimated_s"]
    drive(c, 20.0, until=lambda c: c.mission != "PROGRAM")
    t = c.ai_book.get(r.data["task_id"])
    assert t.state == "SUCCEEDED"
    assert abs(t.evidence["distance_m"] - 1.0) < 0.03 and abs(t.evidence["turned_deg"] - 90) < 2
    assert abs(t.evidence["duration_s"] - est) < 0.15 * est
    p = ProgramRunner("x", [{"do": "wait", "s": 2.0}], 0.5, 1.0)
    assert abs(p.estimate() - 2.05) < 1e-9
    # a steady 180 deg/s seen through jittery ticks (the pose late on one tick, caught up on the next): the fastest
    # rate is measured over RATE_WINDOW, not between two ticks (which read ~240 here)
    p = ProgramRunner("x", [{"do": "wait", "s": 2.0}], 0.5, 1.0)
    t, rate = 0.0, math.radians(180)
    for i in range(60):
        t += 0.05
        seen = t - (0.03 if i % 3 == 1 else 0.0)            # every third pose is 30 ms stale
        p._measure(t, (0.0, 0.0, rate * seen))
    assert abs(p.measured()["max_turn_rate_deg_s"] - 180) < 12
    book = TaskBook()
    t = book.new("move", {}, S, "r1", 0.0, 0.0, 10.0)
    assert book.by_req("r1") is t and book.running() is t


# -- the fun missions as AI tasks (console_fun.py _ai_mission) ------------------------------------------------------
def test_ai_mission_needs_the_lease_and_becomes_a_task_that_ends_with_the_mission():
    c = ready()
    assert ai(c, "ai_mission", name="dance", args=dict(routine="robot")).ok
    t = c.ai_book.running(("mission",))
    assert t is not None and t.args["name"] == "dance" and c.mission == "DANCE"
    c.command("set", dict(key="dance.seconds", value=15))             # a person's setting: no takeover
    while c.mission == "DANCE":
        c.now += 0.05
        lease(c)
        c.drive_output(c.now)
    assert t.state == "SUCCEEDED" and t.evidence["result"].startswith("Dance over")
    other = Console()
    other.on_robot_mode("TELEOP adult")
    assert other.command("ai_mission", dict(name="dance", ai=S)).data["error"] == "no_lease"


def test_ai_mission_refused_while_a_person_drives_and_cancelled_when_one_takes_over():
    c = ready()
    c.command("mission", dict(name="MANUAL"))
    c.manual_input(0.5, 0, 0, False, c.now)
    assert ai(c, "ai_mission", name="goalie").data["error"] == "owner_conflict"
    c = ready()
    r = ai(c, "ai_mission", name="goalie", args=dict(seconds=0))
    assert r.ok and c.mission == "GOALIE"
    tid = r.data["task_id"]
    c.command("stop")                                                    # a person: the AI's task ends
    assert c.ai_book.get(tid).state == "CANCELLED" and c.mission == "IDLE"


def test_ai_goalie_for_a_set_time_ends_with_the_score():
    c = ready()
    r = ai(c, "ai_mission", name="goalie", args=dict(seconds=2))
    tid = r.data["task_id"]
    c.goalie.saves, c.goalie.shots = 3, 4
    for _ in range(6):
        c.now += 0.5
        lease(c)
        c.tick(c.now)
    t = c.ai_book.get(tid)
    assert t.state == "SUCCEEDED" and t.evidence["saves"] == 3 and c.mission == "MANUAL"


def test_ai_mission_stops_when_the_lease_lapses_and_checks_its_name():
    c = ready()
    assert ai(c, "ai_mission", name="tango").data["error"] == "bad_request"
    r = ai(c, "ai_mission", name="goalie")
    c.now += 4.0                                                          # no renewal: the 3 s lease is gone
    c.tick(c.now)
    assert c.ai_book.get(r.data["task_id"]).state == "CANCELLED" and c.mission == "MANUAL"
