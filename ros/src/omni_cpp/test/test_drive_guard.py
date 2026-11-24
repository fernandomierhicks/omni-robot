"""omni_cpp's drive guard (drive_guard.hpp: mode_manager's lidar guard) through test/core_check.cpp's "guard" core.
The guard has no Python twin; the checks here are its geometry and speed law worked by hand, and a stop simulated
with the firmware's braking limits.
    & $py -m pytest -q ros/src/omni_cpp/test/test_drive_guard.py
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from test_cores import run

R, BUF, A, TAU, TOUCH, SELF = 0.217, 0.10, 2.5, 0.10, 0.05, 0.01
CFG = f"cfg {R} {BUF} {A} {TAU} {TOUCH} {SELF}"


def law(free: float) -> float:
    d = free - BUF
    v = -A * TAU + math.sqrt((A * TAU) ** 2 + 2 * A * d) if d > 0 else 0.0
    return max(v, TOUCH * min(max(free / BUF, 0.0), 1.0))


def wall(x: float, y0: float = -2.0, y1: float = 2.0, step: float = 0.01) -> str:
    ys = np.arange(y0, y1 + 1e-9, step)
    return " ".join(f"{x} {y}" for y in ys)


def g(lines: list[str]) -> list[list[float]]:
    return [[float(t) for t in ln.split()] for ln in run("guard", [CFG] + lines)]


def test_the_speed_law_stops_buffer_short_and_creeps_to_contact():
    frees = [0.0, 0.02, 0.05, 0.1, 0.11, 0.2, 0.5, 1.0, 1.5, 3.0]
    out = g([f"speed {f}" for f in frees])
    for f, (v,) in zip(frees, out):
        assert v == pytest.approx(law(f), abs=1e-12)
    assert out[0][0] == 0.0                                   # at contact: nothing more toward it
    assert out[3][0] == pytest.approx(TOUCH)                  # at the buffer: the creep
    # the stop it plans: latency at speed, then braking at A, ends BUF short
    v = law(1.5)
    assert v * TAU + v * v / (2 * A) == pytest.approx(1.5 - BUF, abs=1e-9)


def test_a_wall_ahead_is_the_rim_distance_away_and_never_blocks_leaving_or_sliding():
    out = g(["pts", wall(1.0), "free 1 0", "free -1 0", "free 0 1", "free 0.7071067811865476 0.7071067811865476"])
    assert out[0][0] == pytest.approx(1.0 - R, abs=1e-9)      # the nearest point is straight ahead
    assert out[1][0] > 1e8 and out[2][0] > 1e8                 # away, and along it (it's beside the way)
    # at 45 deg the circle meets the wall where its edge, R off the centre line, reaches x = 1
    assert out[3][0] == pytest.approx((1.0 - R) / math.cos(math.pi / 4), abs=0.01)


def test_its_own_structure_is_ignored_and_a_touching_point_stops_only_the_way_toward_it():
    inside = R - SELF - 0.005                                  # a post inside the rim: the robot itself
    touching = R - 0.004                                       # an obstacle at the rim, within the margin
    out = g(["pts", f"{inside} 0.0", "free 1 0", "pts", f"{touching} 0.0", "free 1 0", "free -1 0",
             "limit 1.0 0 0", "limit -2.0 0 0"])
    assert out[0][0] > 1e8
    assert out[1][0] == 0.0 and out[2][0] > 1e8
    assert out[3][:3] == [0.0, 0.0, 0.0] and out[3][5] == 1.0  # toward: stopped
    assert out[4][:3] == [-2.0, 0.0, 0.0] and out[4][5] == 0.0  # away: full speed


def hwall(y: float, x0: float = -2.0, x1: float = 4.0, step: float = 0.01) -> str:
    xs = np.arange(x0, x1 + 1e-9, step)
    return " ".join(f"{x} {y}" for x in xs)


def test_full_speed_square_at_a_wall_is_capped_to_the_law_keeping_direction_and_turn():
    out = g(["pts", wall(1.3), "limit 2.0 0 0.3", "limit 0.2 0 0", "limit 2.0 0 0"])
    vx, vy, wz, lim, free, cut = out[0]
    assert cut == 1.0 and wz == 0.3 and abs(vy) < 0.02 * vx     # square on: nothing to slide along
    assert free <= 1.3 - R + 1e-9                              # the turn's fan finds the wall no farther
    assert lim == pytest.approx(law(free), abs=1e-9)
    assert run("guard", [CFG, "pts", wall(1.3), f"safe {vx} {vy} {wz}"]) == ["1"]
    assert out[1][:3] == [0.2, 0.0, 0.0] and out[1][5] == 0.0  # slow enough: untouched
    assert out[2][1] == 0.0 and out[2][0] == pytest.approx(law(1.3 - R), abs=1e-9)


@pytest.mark.parametrize("gap", [0.05, 0.15, 0.4])
def test_a_diagonal_push_at_a_wall_slides_along_it_cutting_only_the_part_into_it(gap):
    w = hwall(-(R + gap))                                      # a wall on the right, gap from the rim
    ask = "limit 1.5 -1.5 0"
    vx, vy, wz, lim, free, cut = g(["pts", w, ask])[0]
    assert cut == 1.0
    assert vx == pytest.approx(1.5, abs=0.01)                  # along the wall: kept
    assert -1.5 < vy <= 0.0                                    # into it: cut
    assert math.hypot(vx, vy) > lim                            # faster than the old cap on the asked way
    assert run("guard", [CFG, "pts", w, f"safe {vx} {vy} 0"]) == ["1"]   # and still within the law on its own way
    # the old behaviour (slide 0): the whole push scaled to the asked way's cap, the direction kept
    ox, oy, _, olim, _, _ = (float(t) for t in run("guard", [CFG + " 0", "pts", w, ask])[0].split())
    assert ox == pytest.approx(-oy, abs=1e-9) and math.hypot(ox, oy) == pytest.approx(olim, abs=1e-9)
    # along it, beside it: untouched; away: untouched
    out = g(["pts", w, "limit 2.0 0 0", "limit 1.0 1.0 0"])
    assert out[0][:3] == [2.0, 0.0, 0.0] and out[0][5] == 0.0
    assert out[1][:3] == [1.0, 1.0, 0.0] and out[1][5] == 0.0


def test_a_slide_into_a_corner_is_cut_by_both_walls_and_leaving_is_free():
    gap = 0.3
    w = wall(R + gap, -2.0, 2.0) + " " + hwall(-(R + gap), -2.0, R + gap)
    vx, vy, wz, lim, free, cut = g(["pts", w, "limit 1.5 -1.5 0"])[0]
    assert cut == 1.0 and 0.0 <= vx < 1.0 and -1.0 < vy <= 0.0  # both parts cut: the slide meets the other wall
    assert run("guard", [CFG, "pts", w, f"safe {vx} {vy} 0"]) == ["1"]
    out = g(["pts", w, "limit -1.5 1.5 0", "limit 2.0 0 0", "limit 0 -2.0 0"])
    assert out[0][:3] == [-1.5, 1.5, 0.0]                      # out of the corner: full speed
    for o, along in ((out[1], 0), (out[2], 1)):                # square at either wall: the old cap, no drift
        assert o[1 - along] == 0.0 and abs(o[along]) == pytest.approx(law(gap), abs=1e-9)


def test_a_slide_round_a_post_keeps_within_the_law():
    post = "0.6 -0.12"                                         # ahead, a little right of the centre line
    vx, vy, _, lim, _, cut = g(["pts", post, "limit 2.0 0 0"])[0]
    assert cut == 1.0 and vy > 0.0 and vx > 0.0                # eases left round it
    assert run("guard", [CFG, "pts", post, f"safe {vx} {vy} 0"]) == ["1"]


def test_a_turn_while_driving_sweeps_the_way_into_a_post_beside_it():
    post = "1.0 0.45"                                          # beside the straight way (0.45 > R)
    out = g(["pts", post, "limit 2.0 0 0", "limit 2.0 0 2.0", "limit 2.0 0 -2.0"])
    assert out[0][5] == 0.0 and out[0][4] > 1e8                # straight: passes it
    assert out[1][5] == 1.0 and out[1][4] < 1.0                # turning left into it: slowed
    assert out[2][5] == 0.0                                    # turning right, away from it: not


def test_blind_wedges_and_dropouts_are_filled_from_their_flanks_open_space_is_not():
    n = 360
    inc = 2 * math.pi / n
    rng = np.full(n, 2.0)                                      # a room 2 m round
    rng[170:191] = np.nan                                      # a 21 deg blind wedge behind the posts
    rng[90] = np.nan                                           # one dropped return
    rng[0:3] = np.nan
    rng[357:360] = np.nan                                      # a 6 deg gap across the scan's ends (wraps)
    rng[250:300] = np.inf                                      # 50 deg of open doorway: nothing comes back
    line = " ".join("nan" if not np.isfinite(r) else f"{r}" for r in rng)
    out = run("guard", [CFG, f"scan 0 {inc} 0.15 12.0 {math.radians(30)} 1", line, "free -1 0", "free 0 -1",
                        "free 1 0"])
    count, filled = (int(t) for t in out[0].split())
    assert filled == 21 + 1 + 6 and count == n - 50
    back = float(out[1])
    # the wedge behind is a wall at the flanks' range x cos(half the wedge), not open space
    assert back == pytest.approx(2.0 * math.cos(math.radians(11)) - R, abs=0.02)
    assert float(out[2]) > 1e8                                 # the doorway (-y, 270 deg) stays open
    assert float(out[3]) == pytest.approx(2.0 - R, abs=0.01)   # ahead across the wrapped gap: filled too


def guard_vs_gap(v_ask: float, gaps: np.ndarray) -> np.ndarray:
    """The guard's commanded speed toward a straight wall at each rim-to-wall gap (it depends on nothing else)."""
    lines = [CFG]
    for gp in gaps:
        lines += ["pts", wall(R + gp, -1.0, 1.0, 0.02), f"limit {v_ask} 0 0"]
    return np.array([float(ln.split()[0]) for ln in run("guard", lines)])


def simulate(v0: float, gap0: float, a_brake: float, latency: float, dt: float = 0.001, jerk: float = 40.0):
    """A base at v0, gap0 m from a wall (rim to wall), held at full stick through the guard (every 10 ms, as
    mode_manager), its controller following the guard's command after `latency` within a_brake and the firmware's
    jerk limit (base.cpp velocity_step)."""
    grid = np.linspace(0.0, gap0, 600)
    cmd_of = guard_vs_gap(v0, grid)
    x, v, a, t, cmd = 0.0, v0, 0.0, 0.0, v0
    hist = []                                                   # (t, cmd) as sent
    trace = []
    for k in range(int(10.0 / dt)):
        gap = gap0 - x
        if k % 10 == 0:
            cmd = float(np.interp(gap, grid, cmd_of))
            hist.append((t, cmd))
        late = [c for tc, c in hist if tc <= t - latency]
        target = late[-1] if late else v0
        dv = target - v
        mag = min(a_brake, math.sqrt(2 * jerk * abs(dv)))
        a_des = math.copysign(mag, dv) if abs(dv) > 1e-9 else 0.0
        a += max(-jerk * dt, min(jerk * dt, a_des - a))
        step = a * dt
        v = target if (step > 0) == (dv > 0) and abs(step) > abs(dv) else v + step
        v = max(v, 0.0)
        x += v * dt
        t += dt
        trace.append((t, gap0 - x, v))
        if gap0 - x < 0 or (v < 1e-3 and t > 1.0):
            break
    return trace


@pytest.mark.parametrize("v0", [1.0, 2.0, 3.5])
def test_a_full_speed_run_at_a_wall_stops_short_with_the_firmwares_braking(v0):
    # the firmware's worst-direction braking (fw_params_arm.h ACCEL_VALS: 2.84 m/s^2) and its 40 m/s^3 jerk limit
    # (base.cpp), behind ~30 ms of tick, link and motor: it creeps into the buffer, never at speed, from any speed
    tr = np.array(simulate(v0, 4.0, 2.84, 0.03))
    gaps, vs = tr[:, 1], tr[:, 2]
    assert gaps.min() > 0.0
    near = gaps <= BUF
    assert near.any() and vs[near].max() <= TOUCH + 0.02       # inside the buffer: no faster than the creep
    # three times that delay on top of the jerk ramp (more than the guard's latency covers): later, still no contact
    tr = np.array(simulate(v0, 4.0, 2.84, 0.10))
    assert tr[:, 1].min() > 0.0
