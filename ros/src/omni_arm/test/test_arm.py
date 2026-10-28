"""omni_arm's cores without the sim or ROS (design.md 11): kinematics, the STS3215 protocol, motions, the floor and
wrist vision on synthetic images, the tidy task's poses and grasps. The sim's own checks (the kinematics against
MuJoCo, the servo bus, a whole pick) are in simulation/v11 sil/test_v11.py.

    python -m pytest -q ros/src/omni_arm/test
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from omni_arm import feetech as F  # noqa: E402
from omni_arm import floor_vision as fv  # noqa: E402
from omni_arm import motion as mo  # noqa: E402
from omni_arm.kinematics import DOWN, ArmConfig, ArmKinematics, gap_at, q_for_gap  # noqa: E402
from omni_arm.pick import MJ_TO_OPTICAL, PickParams, TidyTask  # noqa: E402
from omni_arm.wrist_vision import WristFrame, box_corners, refine, self_mask  # noqa: E402


@pytest.fixture(scope="module")
def kin():
    return ArmKinematics()


# -- kinematics ------------------------------------------------------------------------------------------------------
def test_ik_puts_the_tool_on_the_floor_ahead_fingers_down_turned_to_the_yaw(kin):
    """Every heading of the opening, one way round or the other (the wrist rolls 320 deg, not 360: the grasp tries
    both, as the fingers close the same way across an object either way). Out to 0.36 m: the arm kit's reach with
    the fingers straight down (design.md 12.3; 0.42 m with them 15 deg off)."""
    n = 0
    for x in (0.27, 0.33, 0.36):
        for y in (-0.08, 0.0, 0.08):
            for yaw in (0.0, 1.0, 2.5):
                sols = [(w, kin.ik([x, y, 0.02], DOWN, yaw=w, flip_ok=False)) for w in (yaw, yaw + math.pi)]
                sols = [(w, q) for w, q in sols if q is not None]
                assert sols, (x, y, yaw)
                for w, q in sols:
                    T = kin.tool(q)
                    assert np.linalg.norm(T[:3, 3] - [x, y, 0.02]) < 0.002
                    assert T[:3, 0] @ DOWN > math.cos(math.radians(2))
                    o = T[:3, 2]
                    assert abs(math.remainder(math.atan2(o[1], o[0]) - w, 2 * math.pi)) < 0.01
                    assert np.all(q >= kin.lo - 1e-9) and np.all(q <= kin.hi + 1e-9)
                n += 1
    assert n == 27


def test_ik_says_none_out_of_reach(kin):
    assert kin.ik([0.9, 0.0, 0.02], DOWN) is None                  # 0.78 m from the mount
    assert kin.ik([0.30, 0.0, 0.40], DOWN) is None                 # fingers down that high: no


def test_clear_sees_the_floor_and_the_robot(kin):
    q = kin.ik([0.33, 0.0, 0.02], DOWN, yaw=math.pi / 2)
    assert kin.clear(q)[0]
    low = kin.ik([0.33, 0.0, 0.002], DOWN, yaw=math.pi / 2)        # fingertips through the floor
    assert low is None or not kin.clear(low)[0]
    near = kin.ik([0.21, 0.0, 0.05], DOWN, yaw=math.pi / 2)         # down onto the bumper
    assert near is None or kin.clear(near)[1] in ("bumper", "robot", "floor", "the left camera pod",
                                                    "the right camera pod")


def test_gap_and_its_inverse():
    c = ArmConfig()
    assert gap_at(c, c.pads["gripper_closed"]) < 0.005               # closed on nothing: ~3 mm
    assert 0.08 < gap_at(c, 1.0) < 0.09
    for g in (0.01, 0.03, 0.06):
        assert abs(gap_at(c, q_for_gap(c, g)) - g) < 1e-5


def test_arm_out(kin):
    t = TidyTask(kin)
    assert not kin.tool_out(t.poses["stow"])                         # folded: inside the bumper
    assert kin.tool_out(kin.ik([0.33, 0.0, 0.05], DOWN))            # reaching for the floor


# -- the bus ---------------------------------------------------------------------------------------------------------
def test_packets_round_trip_through_the_parser():
    p = F.packet(3, F.INST_WRITE, bytes([42, 0x10, 0x08]))
    assert p[:2] == b"\xff\xff" and p[3] == 5 and F.checksum(p[2:-1]) == p[-1]
    got = F.Parser().feed(b"\x00\x13" + p + F.status_packet(3, 0, b"\x01\x02") + p[:4])
    assert got[0] == (3, 5, F.INST_WRITE, bytes([42, 0x10, 0x08])) and got[1] == (3, 4, 0, b"\x01\x02")
    bad = bytearray(p)
    bad[-1] ^= 0xFF
    par = F.Parser()
    assert par.feed(bytes(bad)) == [] and par.errors == 1


def test_sign_magnitude_and_calibration():
    for v in (0, 5, -5, 1023, -1023):
        assert F.from_sm(F.to_sm(v, 10), 10) == v
    c = F.JointCal(1, 0, 0, 1024, 3072)
    assert c.to_ticks(0.0) == 2048 and c.to_rad(2048) == 0.0
    assert abs(c.to_rad(c.to_ticks(0.7)) - 0.7) < 2 * math.pi / 4096
    assert c.to_ticks(5.0) == 3072                                   # clipped to the range
    assert F.JointCal(1, 1, 0, 1024, 3072).to_rad(2148) < 0          # drive_mode 1 turns the sign


class _Servos:
    """Six servos answering SYNC_READ with fixed positions (a stand-in for the bus)."""

    def __init__(self):
        self.tx = bytearray()
        self.parser = F.Parser()
        self.goals = {}

    def write(self, data):
        for sid, _, instr, p in self.parser.feed(data):
            if instr == F.INST_SYNC_READ:
                for s in p[2:]:
                    body = F.u16(2048 + 100 * s) + F.u16(F.to_sm(-50, 15)) + F.u16(F.to_sm(-200, 10)) + bytes([120, 31])
                    self.tx += F.status_packet(s, 0, body)
            elif instr == F.INST_SYNC_WRITE:
                for i in range(2, len(p), 7):
                    self.goals[p[i]] = p[i + 1] | p[i + 2] << 8
            elif instr in (F.INST_PING, F.INST_WRITE):
                self.tx += F.status_packet(sid, 0)

    def read(self, n=256):
        out = bytes(self.tx[:n])
        del self.tx[:n]
        return out


def test_the_driver_reads_and_writes_in_radians():
    from omni_arm.driver import ArmDriver
    s = _Servos()
    d = ArmDriver(F.Bus(s, timeout=0.0, clock=lambda: 0.0), ArmConfig())
    assert d.ping() == []
    d.enable(True)
    st = d.read(0.0)
    assert st.ok and abs(st.q[0] - 100 * 2 * math.pi / 4096) < 1e-9 and abs(st.load[0] + 0.2) < 1e-9
    assert st.temp_c[0] == 31 and abs(st.volts - 12.0) < 1e-9
    d.send(np.zeros(6))
    assert set(s.goals) == {1, 2, 3, 4, 5, 6} and all(v == 2048 for v in s.goals.values())


# -- motions ---------------------------------------------------------------------------------------------------------
def test_motions_are_timed_by_the_slowest_joint(kin):
    sp = mo.speeds(kin)
    q0, q1 = np.zeros(6), np.array([1.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    m = mo.Motion([q0, q1], sp, t0=10.0)
    assert m.t_end == pytest.approx(10.0 + 1.875 / sp[0])
    assert np.allclose(m.sample(10.0), q0) and np.allclose(m.sample(m.t_end + 1), q1)
    ts = np.linspace(10.0, m.t_end, 400)
    v = np.diff([m.sample(t)[0] for t in ts]) / np.diff(ts)
    assert v.max() <= sp[0] * 1.001


# -- floor vision ----------------------------------------------------------------------------------------------------
def _depth_of(cam_T, K, wh, boxes):
    """Depth image of the floor plane plus axis-aligned boxes (x0, x1, y0, y1, h), by ray casting."""
    W, H = wh
    fx, fy, cx, cy = K
    v, u = np.mgrid[0:H, 0:W]
    rays = np.stack([(u - cx) / fx, (v - cy) / fy, np.ones_like(u, float)], axis=-1) @ cam_T[:3, :3].T
    o = cam_T[:3, 3]
    s_best = np.where(rays[..., 2] < 0, (0 - o[2]) / np.minimum(rays[..., 2], -1e-9), np.inf)
    for x0, x1, y0, y1, h in boxes:
        s = (h - o[2]) / np.minimum(rays[..., 2], -1e-9)                    # the top face
        p = o + rays * s[..., None]
        hit = (rays[..., 2] < 0) & (p[..., 0] > x0) & (p[..., 0] < x1) & (p[..., 1] > y0) & (p[..., 1] < y1)
        s_best = np.where(hit & (s < s_best), s, s_best)
    return s_best.astype(np.float32)                                         # z of the optical frame = s (ray z = 1)


K_DEPTH, WH_DEPTH = (142.8, 142.8, 160.0, 100.0), (320, 200)     # the left camera's 70 deg at 320 x 200


def test_detect_finds_a_block_on_the_floor_and_its_size(kin):
    t = TidyTask(kin)
    z = _depth_of(t.T_base_cam, K_DEPTH, WH_DEPTH, [(0.50, 0.54, -0.03, 0.03, 0.03)])   # 40 x 60 x 30 mm, 0.5 m ahead
    dets = fv.detect(fv.DepthFrame(0.0, z, K_DEPTH, t.T_base_cam))
    assert len(dets) == 1
    d = dets[0]
    assert abs(d.x - 0.52) < 0.012 and abs(d.y) < 0.012 and abs(d.top - 0.03) < 0.004
    assert 0.045 < d.length < 0.075 and 0.025 < d.width < 0.055 and not d.big


def test_detect_leaves_out_tall_things(kin):
    t = TidyTask(kin)
    K, wh = K_DEPTH, WH_DEPTH
    z = _depth_of(t.T_base_cam, K, wh, [(0.60, 0.64, -0.02, 0.02, 0.40)])     # a chair leg
    assert fv.detect(fv.DepthFrame(0.0, z, K, t.T_base_cam)) == []


def test_detect_leaves_out_a_rugs_edge(kin):
    """A 12 mm rug whose edge crosses the view, in noisy depth: no objects; a cube on the bare floor beside it: one."""
    t = TidyTask(kin)
    K, wh = K_DEPTH, WH_DEPTH
    rug = (0.45, 2.0, -2.0, -0.02, 0.012)                                  # the rug's top face (to the right)
    rng = np.random.default_rng(1)
    on_rug = (0.55, 0.58, -0.15, -0.12, 0.012 + 0.03)                    # a cube standing on the rug
    for boxes, n in (([rug], 0), ([rug, (0.50, 0.53, 0.06, 0.09, 0.03)], 1), ([rug, on_rug], 1)):
        z = _depth_of(t.T_base_cam, K, wh, boxes)
        z = z * (1 + rng.normal(0, 0.004, z.shape)).astype(np.float32)   # ~4 mm at 1 m
        dets = [d for d in fv.detect(fv.DepthFrame(0.0, z, K, t.T_base_cam)) if not d.big]
        assert len(dets) == n, dets


def test_floor_map_merges_and_keeps_picked_ones_out():
    m = fv.FloorMap()
    d = fv.Detection(1.0, 0.5, 0.03, 0.04, 0.03, 0.0, 50)
    m.update(0.0, [d])
    m.update(0.1, [fv.Detection(1.02, 0.5, 0.03, 0.04, 0.03, 0.0, 50)])
    assert len(m.tracked) == 1 and m.tracked[0].seen == 2
    m.tracked[0].status = "picked"
    m.update(0.2, [fv.Detection(1.01, 0.5, 0.03, 0.04, 0.03, 0.0, 50)])
    assert m.candidates() == []


# -- wrist vision ----------------------------------------------------------------------------------------------------
def test_refine_finds_a_red_block_on_a_wooden_floor():
    K = (207.8, 207.8, 160.0, 120.0)
    T = np.eye(4)
    T[:3, :3] = np.diag([1.0, -1.0, -1.0])                     # looking straight down from 0.2 m
    T[:3, 3] = (0.35, 0.0, 0.20)
    rng = np.random.default_rng(0)
    img = np.clip(np.array([200, 170, 120]) + rng.normal(0, 4, (240, 320, 3)), 0, 255)
    true = (0.36, 0.012)
    # the block's top (30 x 30 mm at 30 mm) seen from 0.17 m: pixels within it
    f = K[0] / 0.17
    u0, v0 = K[2] + f * (true[0] - 0.35), K[3] + f * (true[1] - 0.0)       # optical x = base x, y = -base y
    v0 = K[3] - f * (true[1])
    img[int(v0 - f * 0.015):int(v0 + f * 0.015), int(u0 - f * 0.015):int(u0 + f * 0.015)] = (200, 30, 30)
    frame = WristFrame(0.0, img.astype(np.uint8), K, T)
    guess = fv.Detection(0.35, 0.0, 0.03, 0.03, 0.03, 0.0, 50)
    r = refine(frame, guess)
    assert r is not None
    assert abs(r.x - true[0]) < 0.004 and abs(r.y - true[1]) < 0.004


def test_the_self_mask_covers_the_fingers(kin):
    t = TidyTask(kin)
    q = kin.ik([0.33, 0.0, 0.12], DOWN, yaw=math.pi / 2)
    fk = kin.fk(q)
    T = fk["wrist_cam"] @ MJ_TO_OPTICAL
    wc = kin.cfg.wrist_cam
    boxes = [box_corners(fk["arm_gripper"], lo, hi) for lo, hi in wc["self_gripper"]]
    m = self_mask((240, 320), (207.8, 207.8, 160.0, 120.0), T, boxes)
    assert 0.02 < m.mean() < 0.45                                    # the fingers at the image's edge, not all of it
    assert not m[120, 160]                                           # the middle (under the fingers) is clear
    del t


def test_a_jog_moves_one_joint_in_its_range_and_never_into_the_robot(kin):
    """The console's "by hand": one joint by a step from where the arm is sent, at most 30 deg, kept in range;
    refused when it would hit something; from the drop (in the bin's box) the arm may move."""
    P = TidyTask(kin).poses
    q = P["ready"]
    t, why = mo.jog(kin, q, 0, math.radians(5))
    assert why == "" and abs(t[0] - q[0] - math.radians(5)) < 1e-9 and np.allclose(t[1:], q[1:])
    t, _ = mo.jog(kin, q, 0, math.radians(90))                          # capped
    assert abs(t[0] - q[0] - mo.JOG_MAX) < 1e-9
    edge = q.copy()
    edge[0] = kin.hi[0]
    assert mo.jog(kin, edge, 0, 0.1)[0] is None                          # the end of its range
    t, _ = mo.jog(kin, q, 1, 0.1, base=mo.jog(kin, q, 1, 0.1)[0])        # a click while one plays adds up
    assert abs(t[1] - q[1] - 0.2) < 1e-9
    # down towards the floor from ready (16 cm over it, fingers down) a step at a time: refused before it gets there
    q2, hit = q.copy(), ""
    for _ in range(40):
        nxt, hit = mo.jog(kin, q2, 1, math.radians(15))
        if nxt is None:
            break
        assert kin.clear(nxt)[0]
        q2 = nxt
    assert hit.startswith("it would hit"), "the shoulder ran its whole range without a refusal"
    assert mo.jog(kin, P["drop"], 4, math.radians(5))[0] is not None     # the wrist turns over the bin


# -- the task --------------------------------------------------------------------------------------------------------
def test_the_tasks_poses_are_clear_and_it_grasps_at_the_sweet_spot(kin):
    t = TidyTask(kin)
    for name in ("stow", "ready"):
        assert kin.clear(t.poses[name])[0], name
    assert kin.clear(t.poses["drop"], allow_bin=True)[0]
    P = t.poses
    via = [P[n] for n in ("high", "ready", "predrop") if n in P]          # as TidyTask._go
    for a, b in (("stow", "ready"), ("ready", "stow"), ("stow", "drop"), ("drop", "stow"), ("ready", "drop")):
        assert mo.plan(kin, P[a], P[b], via=via, allow_bin=True) is not None, (a, b)
    for a, b in (("stow", "high"), ("high", "stow")):                    # the console's other poses
        assert mo.plan(kin, P[a], P[b], via=via) is not None, (a, b)
    p = PickParams()
    for yaw in (0.0, 0.8, 1.6):
        d = fv.Detection(p.sweet[0], p.sweet[1], 0.03, 0.04, 0.03, yaw, 50)
        g = t._grasp_for((0.0, 0.0, 0.0), d, t.poses["ready"])
        assert g is not None, yaw
        hover, tool, qh, gy = g
        assert kin.clear(qh)[0]
        # the fingers close across the short side
        assert abs(math.cos(gy - (yaw + math.pi / 2))) > 0.99
