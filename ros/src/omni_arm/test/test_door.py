"""omni_arm.door without the sim or ROS: the door's geometry, the stance, the level grasp, and the pull on a
kinematic door (an ideal base, the arm a spring whose hand the hinge holds on its arc). The whole task on the
detailed arm with the firmware is simulation/v11 sil/studies/door_open.py.

    python -m pytest -q ros/src/omni_arm/test/test_door.py
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from omni_arm.door import (PAD_BACK, Door, DoorCamera, DoorInputs, DoorTag, DoorTask, _R, _tag_corners,  # noqa: E402
                           door_from_tag, face_plane, upright_tag_pose)
from omni_arm.kinematics import ArmConfig, ArmKinematics, config_path, gap_at  # noqa: E402


@pytest.fixture(scope="module")
def kin():
    return ArmKinematics(ArmConfig(config_path("d435")))


def door(sense: int) -> Door:
    """A door 1 m ahead of the odom origin, facing it, hinged on the robot's left (sense -1) or right (+1)."""
    if sense < 0:
        return Door((1.0, 0.25), -math.pi / 2, -1, 0.417, 0.051, 0.38)
    return Door((1.0, -0.25), math.pi / 2, 1, 0.417, 0.051, 0.38)


@pytest.mark.parametrize("sense", [-1, 1])
def test_door_geometry(sense):
    d = door(sense)
    for th in np.radians([0, 20, 45, 90, 110]):
        u, n = d.u(th), d.n(th)
        assert abs(u @ n) < 1e-12
        assert abs(math.atan2(u[0] * n[1] - u[1] * n[0], u @ n) - sense * math.pi / 2) < 1e-9
        assert d.angle_of(d.bar(th)) == pytest.approx(th, abs=1e-9)
        assert np.linalg.norm(d.bar(th) - d.hinge) == pytest.approx(math.hypot(d.handle_r, d.handle_out))
    assert d.n(0.0) @ np.array([-1.0, 0.0]) > 0.999          # the room is toward the robot
    assert d.bar(math.radians(30))[0] < d.bar(0.0)[0]         # opening brings the bar toward the robot


@pytest.mark.parametrize("sense", [-1, 1])
def test_level_grasp_and_stance(kin, sense):
    t = DoorTask(kin, door(sense))
    assert t.q_pre is not None and t.q_probe is not None
    for q in (t.q_pre, t.q_probe):
        T = kin.tool(q)
        assert T[:3, 0] @ t.a > 0.999                      # fingers level, straight ahead
        assert T[:3, 2] @ t.w > 0.995                      # the opening across, toward the free edge
        assert abs(T[:3, 3][1]) < 0.003                    # on base_link's x axis
    d = t.door
    for th in np.radians([0, 30, 60, 90]):
        x, y, yaw = t.stance(th)
        bar_b = _R(-yaw) @ (d.bar(th) - np.array([x, y]))
        assert np.allclose(bar_b, t.bar_ref[:2], atol=1e-9)
        assert np.array([math.cos(yaw), math.sin(yaw)]) @ -d.n(th) > 0.999      # facing the door
        assert (_R(-yaw) @ d.u(th)) @ t.w[:2] > 0.999      # the jaws open toward the free edge (the fixed finger,
                                                           # the probe's, lands on the door)
    # the bar sits centred between the open jaws at the pads' middle
    T = kin.tool(t.q_pre)
    rel = t.bar_ref - (T[:3, 3] + t.a * t.p.pre_back)
    assert rel @ t.w == pytest.approx(t.g_open / 2, abs=0.003)
    assert rel @ t.a == pytest.approx(-PAD_BACK, abs=0.003)


class KinematicDoor:
    """The pull on an ideal world: the base does what it is told, the hinge holds the hand on the bar's arc (the arm
    a spring: the hand is where its goal is, pulled onto the circle), a closing torque the robot never sees but in
    the hand's lag. drop_at: the bar slips out of the jaws at this door angle."""

    def __init__(self, kin, t: DoorTask, pose, drop_at: float | None = None):
        self.kin, self.t, self.pose = kin, t, np.array(pose, float)
        self.th = 0.0
        self.q5 = t.grip_gap_q5
        self.drop_at = drop_at
        self.min_clear = math.inf

    def step(self, twist, goal, dt=0.02):
        if twist is not None:
            vx, vy, wz = twist
            self.pose[:2] += _R(self.pose[2]) @ np.array([vx, vy]) * dt
            self.pose[2] += wz * dt
        d, t = self.t.door, self.t
        # the hand (the goal's bar) put on the arc: the door's angle follows it, its stop at open_max
        Tg = self.kin.tool(goal)
        bar_goal_b = Tg[:3, 3] + Tg[:3, 2] * d.bar_d / 2 - Tg[:3, 0] * PAD_BACK
        bar_goal = self.pose[:2] + _R(self.pose[2]) @ bar_goal_b[:2]
        self.th = float(np.clip(d.angle_of(bar_goal), 0.0, d.open_max))
        if self.drop_at is not None and self.th >= self.drop_at:
            self.q5 = t.q_closed                            # the jaws close on nothing
        give_odom = d.bar(self.th) - bar_goal
        give_b = _R(-self.pose[2]) @ give_odom
        q = np.asarray(goal, float).copy()
        q[5] = self.q5
        # the measured angles: the goal's tool moved by the give (the arm's spring), found by IK
        qm = t.ik_level(Tg[:3, 3] + np.r_[give_b, 0.0], t.w, self.q5, seed=goal)
        if qm is not None:
            q = qm
        # the base's clearance from the door's leaf (the bumper's radius 0.233 m)
        h = np.asarray(d.hinge)
        u = d.u(self.th)
        rel = self.pose[:2] - h
        s = float(np.clip(rel @ u, 0.0, 0.457))
        self.min_clear = min(self.min_clear, float(np.linalg.norm(rel - s * u)) - 0.233)
        return q


def held_task(kin, sense):
    """A task seated and holding the bar, at the stance, in PULL."""
    t = DoorTask(kin, door(sense))
    # held, the tool is back on base_link's x axis (the base followed the jaws across) at the grasp
    q = t.ik_level(t.G, t.w, t.q_closed, seed=t.q_probe)
    q[5] = 0.05                                          # the jaws on the bar
    t.bar_ref = kin.tool(q)[:3, 3] + kin.tool(q)[:3, 2] * t.door.bar_d / 2 - kin.tool(q)[:3, 0] * PAD_BACK
    t.grip_gap = gap_at(kin.cfg, 0.05)
    t.grip_gap_q5 = 0.05
    t.state, t.t_state, t.t_best = "PULL", 0.0, 0.0
    return t, q


@pytest.mark.parametrize("sense", [-1, 1])
def test_pull_opens_on_a_kinematic_door(kin, sense):
    t, goal = held_task(kin, sense)
    world = KinematicDoor(kin, t, t.stance(0.0))
    q = goal.copy()
    tt = 0.0
    while t.state == "PULL" and tt < 20.0:
        out = t.step(tt, DoorInputs(tuple(world.pose), q, goal, np.zeros(3), False, False))
        q = world.step(out.twist, goal)
        tt += 0.02
    assert t.state == "RELEASE"
    assert math.degrees(world.th) >= 85.0
    assert abs(t.th_est - world.th) < math.radians(2)
    assert t.peak["give"] < 0.02
    assert world.min_clear > 0.15                        # the bumper well clear of the door's leaf


def test_pull_notices_a_lost_grip(kin):
    t, goal = held_task(kin, -1)
    world = KinematicDoor(kin, t, t.stance(0.0), drop_at=math.radians(30))
    q = goal.copy()
    tt = 0.0
    while t.state == "PULL" and tt < 20.0:
        out = t.step(tt, DoorInputs(tuple(world.pose), q, goal, np.zeros(3), False, False))
        q = world.step(out.twist, goal)
        tt += 0.02
    assert t.state == "RELEASE" and t.outcome == "lost"
    assert world.th < math.radians(40)


def test_pull_waits_at_the_force_limit(kin):
    """At f_max the reference stops: no progress for stall_s, then it lets go (STALLED)."""
    t, goal = held_task(kin, -1)
    world = KinematicDoor(kin, t, t.stance(0.0))
    q = goal.copy()
    tt = 0.0
    while t.state == "PULL" and tt < 20.0:
        f = np.array([t.p.f_max + 2.0, 0.0, 0.0]) if tt > 1.0 else np.zeros(3)
        out = t.step(tt, DoorInputs(tuple(world.pose), q, goal, f, False, False))
        q = world.step(out.twist, goal)
        tt += 0.02
    assert t.state == "RELEASE" and "stalled" in t.events[-1][2]
    assert world.th < math.radians(25)


# -- finding the door by its tag ---------------------------------------------------------------------------------------
TAG_Z = 0.142


def tag_on(d: Door, size: float = 0.048):
    """The tag right under d's pull on its face (odom: centre xy, its normal's heading) and the door map's entry."""
    n, u = d.n(0.0), d.u(0.0)
    c = np.asarray(d.hinge) + d.handle_r * u + (d.handle_out - d.bar_off) * n
    xt = np.array([-n[1], n[0]])
    rec = DoorTag(7, size, float((np.asarray(d.hinge) - c) @ xt), d.handle_z - TAG_Z,
                  door_t=d.handle_out - d.bar_off, bar_off=d.bar_off, bar_d=d.bar_d, open_max=d.open_max)
    return c, math.atan2(n[1], n[0]), rec


@pytest.mark.parametrize("sense", [-1, 1])
def test_door_from_its_tag(sense):
    d = door(sense)
    c, hd, rec = tag_on(d)
    assert (rec.hinge_x < 0) == (sense < 0)                  # hinged on the left seen from the room: clockwise
    e = door_from_tag(rec, c, TAG_Z, hd)
    assert e.sense == d.sense
    assert np.allclose(e.hinge, d.hinge, atol=1e-9)
    assert math.remainder(e.closed_heading - d.closed_heading, 2 * math.pi) == pytest.approx(0, abs=1e-9)
    assert (e.handle_r, e.handle_out, e.handle_z) == pytest.approx((d.handle_r, d.handle_out, d.handle_z))
    for th in np.radians([0, 45, 90]):
        assert np.allclose(e.bar(th), d.bar(th), atol=1e-9)
        assert e.angle_of_normal(math.atan2(*d.n(th)[::-1])) == pytest.approx(th, abs=1e-9)


def _camera(kin):
    return DoorCamera.from_config(kin.cfg.cameras[0])


def _project(cam, centre, heading, size, roll=0.0):
    c, s = math.cos(heading), math.sin(heading)
    R = np.array([[-s, 0.0, c], [c, 0.0, s], [0.0, 1.0, 0.0]])
    cr, sr = math.cos(roll), math.sin(roll)
    obj = _tag_corners(size) @ np.array([[cr, -sr, 0], [sr, cr, 0], [0, 0, 1.0]]).T
    P = (obj @ R.T + centre - cam.T_base_cam[:3, 3]) @ cam.T_base_cam[:3, :3]
    return np.column_stack([cam.fx * P[:, 0] / P[:, 2] + cam.cx, cam.fy * P[:, 1] / P[:, 2] + cam.cy])


def test_upright_tag_pose(kin):
    cam = _camera(kin)
    centre, hd = np.array([0.80, -0.12, TAG_Z]), math.pi + math.radians(8)
    uv = _project(cam, centre, hd, 0.048)
    assert np.all((uv >= 0) & (uv < [480, 270]))             # in the 480 x 270 stream's picture
    p, h, rms = upright_tag_pose(cam, uv, 0.048)
    assert np.linalg.norm(p - centre) < 1e-5 and abs(math.remainder(h - hd, 2 * math.pi)) < 1e-5 and rms < 1e-4
    # the corners 0.2 px off: the place holds to a few mm; the heading of a 20 px tag does not (face_plane's job)
    rng = np.random.default_rng(3)
    p, h, rms = upright_tag_pose(cam, uv + rng.normal(0, 0.2, uv.shape), 0.048)
    assert np.linalg.norm(p - centre) < 0.02 and rms < 1.0
    # a big tag close, stuck on 1.5 deg turned: the roll is fitted (unfitted it missed by > 1.5 px and was refused)
    centre = np.array([0.45, 0.05, TAG_Z])
    uv = _project(cam, centre, hd, 0.080, roll=math.radians(1.5))
    p, h, rms = upright_tag_pose(cam, uv, 0.080)
    assert np.linalg.norm(p - centre) < 1e-4 and abs(math.remainder(h - hd, 2 * math.pi)) < 1e-4 and rms < 0.01


def test_face_plane(kin):
    """A vertical face (heading 3 deg off facing the robot) over a floor, seen by the depth camera with 2 mm noise:
    the heading to a few tenths of a degree, the tag's offset to the face to a few mm."""
    cam = _camera(kin)
    H, W = 270, 480
    hd = math.pi + math.radians(3)
    n = np.array([math.cos(hd), math.sin(hd), 0.0])
    centre = np.array([0.80, -0.05, TAG_Z])
    v, u = np.mgrid[0:H, 0:W]
    ray_c = np.stack([(u - cam.cx) / cam.fx, (v - cam.cy) / cam.fy, np.ones_like(u, float)], axis=-1)
    ray = ray_c @ cam.T_base_cam[:3, :3].T
    o = cam.T_base_cam[:3, 3]
    s_face = ((centre - o) @ n) / (ray @ n)                    # along the ray (z-depth units: ray_c z = 1)
    s_floor = np.where(ray[..., 2] < 0, -o[2] / np.minimum(ray[..., 2], -1e-9), np.inf)
    face_hit = o + ray * s_face[..., None]
    s_face = np.where((face_hit[..., 2] > 0.105) & (s_face > 0), s_face, np.inf)
    depth = np.minimum(s_face, s_floor)
    depth = depth + np.random.default_rng(5).normal(0, 0.002, depth.shape)
    depth[~np.isfinite(depth)] = np.nan
    guess = centre + n * 0.012                                 # the tag's own place 12 mm proud, its heading 4 deg off
    f = face_plane(cam, depth, guess, hd + math.radians(4))
    assert f is not None
    h, off, k, rms = f
    assert abs(math.degrees(math.remainder(h - hd, 2 * math.pi))) < 0.3
    assert off == pytest.approx(-0.012, abs=0.003) and k > 1000 and rms < 0.004


def test_find_then_look(kin):
    """The door off to the side, out of the camera's view: the base turns till the tag is seen, stops, looks
    (5 frames), drives to the standoff, looks again, and the door it made is the door."""
    d = door(-1)
    c, hd, rec = tag_on(d)
    cam = _camera(kin)
    t = DoorTask(kin, None, tag=rec, camera=cam)
    # the door's face 0.82 m ahead, ~70 deg to the left of where the robot faces (from 1 m the tag is over the
    # top of the D435's view: the robot sees a door's tag from ~0.9 m at most)
    pose = np.array([0.2, 0.0, math.radians(-70)])
    t.start(0.0)
    tt, k = 0.0, 0
    states = []
    while tt < 30.0 and t.state not in ("REACH", "DONE"):
        tags = None
        if k % 5 == 0:                                         # 10 Hz frames
            cb = _R(-pose[2]) @ (c - pose[:2])
            uv = _project(cam, np.r_[cb, TAG_Z], hd - pose[2], rec.size)
            P = (np.r_[cb, TAG_Z] - cam.T_base_cam[:3, 3]) @ cam.T_base_cam[:3, :3]
            seen = P[2] > 0.1 and np.all((uv >= 5) & (uv < [475, 265]))
            tags = [(7, uv, 90.0)] if seen else []
        out = t.step(tt, DoorInputs(tuple(pose), np.zeros(6), np.zeros(6), tags=tags))
        if out.twist is not None:                              # an ideal base
            vx, vy, wz = out.twist
            pose[:2] += _R(pose[2]) @ np.array([vx, vy]) * 0.02
            pose[2] += wz * 0.02
        if not states or states[-1] != t.state:
            states.append(t.state)
        tt += 0.02
        k += 1
    assert states[:5] == ["FIND", "LOOK", "APPROACH", "LOOK", "APPROACH"], states
    assert t.state == "REACH" and len(t.looks) == 2
    assert np.linalg.norm(np.asarray(t.door.hinge) - d.hinge) < 1e-3
    assert abs(math.remainder(t.door.closed_heading - d.closed_heading, 2 * math.pi)) < 1e-3
