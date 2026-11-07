"""Goalie, scored on the truth: a play ball (22 cm, 0.3 kg, rolling) rolled at the arm robot from 3 m; the robot's own
code tracks it (omni_arm's ball_track on the D435's rendered colour and depth, with the D435's depth errors) and blocks
it (omni_console's goalie.py), through the mode mux and the C++ firmware, as the console would drive it. A shot is
saved when the ball never crosses the goal line between the posts (the truth), a goal when it does.

Each shot: the ball from 3 m ahead and up to 0.4 m to a side, aimed at a point up to 0.55 m either side of the goal's
middle (the posts at 0.6 m), at 1.5-3 m/s (seeded). The tracker runs at 15 Hz (the D435 gives 30; rendering is slow).

  python studies/goalie.py                   10 shots, seed 0 (MUJOCO_GL=osmesa without a display)
  python studies/goalie.py --shots 20 --seed 3 --speed 2.5
Writes results/goalie.json.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "ros" / "src" / "omni_console"))
from common import save  # noqa: E402

RATE = 15.0                    # Hz: the tracker's frames


def run(shots: int = 10, seed: int = 0, speed: float | None = None, verbose: bool = False, parked: bool = False) -> dict:
    """parked: the same shots against the robot standing still (the baseline: what it saves without moving)."""
    import mujoco

    import arm_objects
    from sil_pick import SilPick                       # first: it puts ros/src on the path
    from omni_arm.ball_track import BallParams, BallTracker
    from omni_arm.brain_core import state_from_joints
    from omni_arm.floor_vision import DepthFrame
    from omni_console.goalie import Goalie, GoalieParams
    from omni_mcu.protocol import State
    arm_objects.SETS["goalie"] = ("ball_big",)
    g = SilPick(1, "goalie", seed, spread=0.3, depth_camera="d435")
    rng = np.random.default_rng(seed)
    m, d = g.m, g.d
    jb = m.joint("obj0")
    qa, va = jb.qposadr[0], jb.dofadr[0]
    base = m.body("base").id
    rad = 0.11
    tracker = BallTracker(BallParams(color="red", diameter=0.22))
    results = []
    t_cmd = t_frame = t_drv = 0.0
    twist = (0.0, 0.0, 0.0)
    for k in range(shots):
        # the robot where it is now: the goal's frame (its heading, its place)
        mujoco.mj_forward(m, d)
        R0, p0 = d.xmat[base].reshape(3, 3).copy(), d.xpos[base].copy()
        fwd, left = R0[:, 0].copy(), R0[:, 1].copy()
        fwd[2] = left[2] = 0.0
        fwd /= np.linalg.norm(fwd)
        left /= np.linalg.norm(left)
        goalie = Goalie(tuple(g.pose), GoalieParams())
        side = rng.uniform(-0.4, 0.4)
        aim = rng.uniform(-0.55, 0.55)
        v = speed or rng.uniform(1.5, 3.0)
        start = p0 + 3.0 * fwd + side * left
        target = p0 + 0.0 * fwd + aim * left
        dirv = (target - start)
        dirv[2] = 0.0
        dirv /= np.linalg.norm(dirv)
        d.qpos[qa:qa + 3] = [start[0], start[1], rad + 0.001]
        d.qpos[qa + 3:qa + 7] = [1, 0, 0, 0]
        d.qvel[va:va + 3] = v * dirv
        d.qvel[va + 3:va + 6] = np.cross([0, 0, 1.0], dirv) * (v / rad)        # rolling, not skidding
        crossed, t0 = None, d.time
        max_n, at_line, closest = 0, None, 9.0
        while d.time - t0 < 4.0:
            t = d.time
            for msg in g.link.poll(t):
                if isinstance(msg, State):
                    g.pose = (msg.x, msg.y, msg.yaw)
            if t >= t_drv:
                t_drv = t + 0.02
                hw = g.driver.cycle(t)                         # the arm holds where it is (stowed)
                state_from_joints(t, hw.q, hw.dq, hw.load)
            if t >= t_frame:
                t_frame = t + 1.0 / RATE
                f = DepthFrame(t, g.cams.depth(), g.cams.Kd, g.task.T_base_cam)
                st = tracker.step(t, f, g.cams.color(), g.pose)
                ball = dict(st, age=st.get("age", 0.0)) if st.get("seen") else None
                max_n = max(max_n, st.get("n", 0))
                twist = goalie.tick(t, g.pose, ball)
                if parked:
                    twist = (0.0, 0.0, 0.0)
            if t >= t_cmd:
                t_cmd = t + 0.01
                g.link.send(g.mux.command(g.link.host.sync, pick=twist), t)
            g.robot.step()
            b = d.xpos[m.body("obj0").id]
            u, w = float(np.dot(b - p0, fwd)), float(np.dot(b - p0, left))
            rb = d.xpos[base]
            closest = min(closest, float(np.hypot(*(b[:2] - rb[:2]))))
            if at_line is None and u < 0.35:                       # the ball at the robot's front: where was the robot?
                at_line = dict(robot=round(float(np.dot(rb - p0, left)), 3), ball=round(w, 3))
            if crossed is None and u < 0.0:
                crossed = w
            if crossed is not None or np.linalg.norm(d.qvel[va:va + 2]) < 0.05 and d.time - t0 > 1.0:
                break
        goal = crossed is not None and abs(crossed) < 0.6
        res = dict(shot=k, speed=round(v, 2), aim=round(aim, 3), start_side=round(side, 2),
                   result="goal" if goal else ("wide" if crossed is not None else "save"),
                   crossed_at=None if crossed is None else round(crossed, 3), goalie_said=goalie.state,
                   tracked=max_n, at_front=at_line, closest_m=round(closest, 3))
        results.append(res)
        if verbose:
            print(res)
        # back to the middle of the goal for the next shot (the goalie's own controller, no ball)
        goalie_home = Goalie(goalie.home, GoalieParams())
        t1 = d.time
        d.qpos[qa:qa + 3] = [p0[0] - 30.0, p0[1], rad]             # the ball out of the way
        d.qvel[va:va + 6] = 0.0
        while d.time - t1 < 2.0:
            t = d.time
            for msg in g.link.poll(t):
                if isinstance(msg, State):
                    g.pose = (msg.x, msg.y, msg.yaw)
            if t >= t_drv:
                t_drv = t + 0.02
                g.driver.cycle(t)
            if t >= t_cmd:
                t_cmd = t + 0.01
                twist = goalie_home.tick(t, g.pose, None)
                g.link.send(g.mux.command(g.link.host.sync, pick=twist), t)
            g.robot.step()
        tracker = BallTracker(BallParams(color="red", diameter=0.22))
    g.cams.close()
    shots_on = [r for r in results if r["result"] != "wide"]
    saves = sum(r["result"] == "save" for r in results)
    return dict(shots=len(results), on_target=len(shots_on), saves=saves,
                goals=sum(r["result"] == "goal" for r in results), wide=sum(r["result"] == "wide" for r in results),
                save_rate=round(saves / max(1, len(shots_on)), 2), runs=results, seed=seed, parked=parked)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--shots", type=int, default=10)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--speed", type=float, default=None)
    ap.add_argument("--parked", action="store_true", help="the baseline: the robot stands still")
    ap.add_argument("--out", default="", help="the results' name (default goalie / goalie_parked)")
    a = ap.parse_args()
    r = run(a.shots, a.seed, a.speed, verbose=True, parked=a.parked)
    print({k: v for k, v in r.items() if k != "runs"})
    save(a.out or ("goalie_parked" if a.parked else "goalie"), r)


if __name__ == "__main__":
    main()
