"""omni_bringup's heading core (heading.py). Run from the repo root, Windows or WSL:
    & $py -m pytest -q ros/src/omni_bringup/test
"""

import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from omni_bringup.heading import HeadingConfig, HeadingShaper, wrap     # noqa: E402


def drive(mode: str, course=(1.0, 0.0), seconds=4.0, plan=10.0, shaped=True, lag=0.03, nav_w=0.0, **kw):
    """Nav2 (20 Hz) holds a world course; the shaper republishes at 50 Hz; the firmware, `lag` s later, turns each
    body command into a world velocity with its own yaw every 1 ms. Returns the track [(t, x, y, yaw)]."""
    sh = HeadingShaper(HeadingConfig(mode=mode, lead=lag, **kw))
    x = y = yaw = 0.0
    wz = 0.0
    queue, cmd, track = [], (0.0, 0.0, 0.0), []
    for i in range(int(seconds * 1000)):
        t = i * 1e-3
        if i % 10 == 0:
            sh.on_odom(t, x, y, yaw, wz)
        if i % 1000 == 0:
            sh.on_plan(t, max(0.0, plan - math.hypot(x, y)))
        out = None
        if i % 50 == 0:                                   # Nav2: the course in the body frame at the current yaw
            c, s = math.cos(yaw), math.sin(yaw)
            out = sh.on_cmd(t, c * course[0] + s * course[1], -s * course[0] + c * course[1], nav_w)
            if not shaped:
                out = (c * course[0] + s * course[1], -s * course[0] + c * course[1], out[2])
        elif i % 20 == 0 and shaped:
            out = sh.step(t)
        if out is not None:
            queue.append((t + lag, out))
        while queue and queue[0][0] <= t:
            cmd = queue.pop(0)[1]
        c, s = math.cos(yaw), math.sin(yaw)
        x += (c * cmd[0] - s * cmd[1]) * 1e-3
        y += (s * cmd[0] + c * cmd[1]) * 1e-3
        wz = cmd[2]
        yaw = wrap(yaw + wz * 1e-3)
        track.append((t, x, y, yaw))
    return track


def test_free_passes_nav2_through():
    sh = HeadingShaper(HeadingConfig(mode="agnostic"))
    sh.on_odom(0.0, 0.0, 0.0, 0.3, 0.0)
    assert sh.on_cmd(0.0, 0.5, -0.2, 0.7) == (0.5, -0.2, 0.7)
    assert sh.step(0.01) is None                         # nothing of its own to publish


def test_forward_turns_the_face_to_the_course_and_keeps_it():
    tr = drive("forward", course=(0.0, 1.0))             # travelling +y, starting with the face along +x
    t, x, y, yaw = tr[-1]
    assert abs(math.degrees(wrap(yaw - math.pi / 2))) < 3.0
    assert abs(x) < 0.05 and y > 3.5                     # it kept its course while it turned
    t1 = next(p[0] for p in tr if abs(wrap(p[3] - math.pi / 2)) < math.radians(5))
    assert t1 < 2.0                                      # 90 deg at <= 90 deg/s, ramped


def test_spinning_spins_and_holds_the_course():
    tr = drive("spinning", seconds=4.0)
    turned = sum(abs(wrap(b[3] - a[3])) for a, b in zip(tr, tr[1:]))
    assert turned > math.radians(120) * 3.0              # ~4 s at 120 deg/s, less the ramp up
    t, x, y, yaw = tr[-1]
    assert abs(y) < 0.05 and x > 3.8                     # still on the straight line
    naive = drive("spinning", shaped=False)             # Nav2's 20 Hz body command held while spinning
    assert abs(naive[-1][2]) > 3 * abs(y)


def test_hands_the_turn_back_near_the_goal():
    sh = HeadingShaper(HeadingConfig(mode="spinning"))
    sh.on_odom(0.0, 0.0, 0.0, 0.0, 0.0)
    sh.on_plan(0.0, 3.0)
    sh.on_cmd(0.0, 0.5, 0.0, 0.1)
    assert sh.target_rate(0.0) == sh.cfg.spin_rate
    sh.on_plan(0.0, 0.3)                                 # under handback[0]: Nav2's own turn rate
    assert sh.target_rate(0.0) == 0.1
    sh.plan = None                                       # no plan: unknown distance, Nav2's rate
    assert sh.target_rate(0.0) == 0.1


def test_forward_leaves_a_slow_robot_to_nav2():
    sh = HeadingShaper(HeadingConfig(mode="forward"))
    sh.on_odom(0.0, 0.0, 0.0, 0.0, 0.0)
    sh.on_plan(0.0, 5.0)
    sh.on_cmd(0.0, 0.0, 0.05, -0.2)                      # creeping sideways
    assert sh.target_rate(0.0) == -0.2


def test_goal_distance_counts_down_between_plans():
    sh = HeadingShaper()
    sh.on_odom(0.0, 0.0, 0.0, 0.0, 0.0)
    sh.on_plan(0.0, 5.0)
    sh.on_odom(0.5, 1.0, 0.0, 0.0, 0.0)
    assert abs(sh.goal_distance(0.5) - 4.0) < 1e-9
    assert sh.goal_distance(4.0) is None                 # stale plan


def test_stops_publishing_when_nav2_stops_and_starts_fresh():
    sh = HeadingShaper(HeadingConfig(mode="spinning"))
    sh.on_odom(0.0, 0.0, 0.0, 0.0, 0.0)
    sh.on_plan(0.0, 5.0)
    sh.on_cmd(0.0, 0.5, 0.0, 0.0)
    for k in range(1, 15):
        sh.step(k * 0.02)
    assert sh.step(0.3) is not None and sh.step(0.31) is None and sh.w_out > 0.9
    sh.on_cmd(5.0, 0.5, 0.0, 0.0)                        # a new drive ramps up from Nav2's rate again
    assert sh.w_out < 0.2
