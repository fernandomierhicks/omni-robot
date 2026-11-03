"""ghost_ball's arithmetic (ros/src/omni_sim/omni_sim/ghost_core.py, no ROS): frame transform and clock mapping."""
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "ros" / "src" / "omni_sim" / "omni_sim"))

import ghost_core as G   # noqa: E402


def test_identity_transform_keeps_the_track():
    x = [3.0, 0.5, 1.2, -4.0, 0.1, 2.0]
    P = np.diag([0.01, 0.02, 0.03, 0.4, 0.5, 0.6]).reshape(-1)
    xs, Ps, cr = G.transform_track(x, P, [1.0, 0.0, 0.0], (0.0, 0.0), 0.0)
    assert np.allclose(xs, x) and np.allclose(Ps, P.reshape(6, 6)) and np.allclose(cr, [1, 0, 0])


def test_yaw_turns_position_velocity_and_covariance_and_shifts_the_position_only():
    x = [2.0, 0.0, 1.0, -3.0, 0.0, 0.5]
    P = np.diag([0.01, 0.04, 0.01, 0.2, 0.8, 0.2]).reshape(-1)
    xs, Ps, cr = G.transform_track(x, P, [1.0, 0.0, 0.0], (10.0, 5.0), math.pi / 2)
    assert np.allclose(xs[:3], [10.0, 7.0, 1.0]) and np.allclose(xs[3:], [0.0, -3.0, 0.5])      # position moved, velocity not
    assert np.allclose(cr, [10.0, 6.0, 0.0])
    assert np.allclose(np.diag(Ps)[:3], [0.04, 0.01, 0.01])                                       # x and y variances swap
    assert np.allclose(Ps, Ps.T)


def test_clock_map_follows_the_shortest_delay_and_stays_put_under_jitter():
    m = G.ClockMap(latency_min=0.017)
    rng = np.random.default_rng(0)
    off = 1000.0                                           # the sim clock reads 1000 s ahead of the head's
    out = []
    for k in range(400):
        head_t = 50.0 + 0.0083 * k
        delay = 0.017 + abs(rng.normal(0, 0.006))          # capture -> here: never faster than 17 ms
        out.append(m.update(head_t, head_t + off + delay) - (head_t + off))
    out = np.array(out)
    assert abs(out[-50:]).max() < 0.004                    # settled onto the true mapping within a few ms
    assert abs(np.median(out)) < 0.004
