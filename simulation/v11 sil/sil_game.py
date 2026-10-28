"""v11 (design.md 30): the whole catch game in one process, deterministic: the world (catch_world.py), the robot with
its C++ firmware (sil_robot.py), and the Jetson's code as the ROS nodes run it (ros/src/omni_catch, omni_mcu):

  world physics, 0.5 ms --- CAN / sensors ---> firmware (C++, 1 kHz) --- USB bytes (sil_link) ---> Jetson
                                                                                                     |
  cameras: SparseFrames after the capture latency ---------------------> perception (detector + filter)
                                                                                                     |
                                                       CMD (100 Hz) <--- mode mux <--- catch behavior

Without ROS the timing is modeled instead of measured: the Jetson's processing per frame pair (WorldConfig.processing,
est 5 ms) and the behavior's 100 Hz cycle. The ROS run (omni.sh catch) measures them.

The cameras (design.md 31): --frames sparse (default) sends CameraSim's pixel lists, --frames rendered sends MuJoCo
renders through a sensor model (rendered_camera.py) as full images, on sim time (far slower than real time).
--scene house puts the field in a furnished house (house_scene.py; the same physics). Either way the robot's code is
the same; PerceptionScore grades what its detector reported against where the ball really was.

    python sil_game.py                      6 throws from 6.5 m at a 1 m apex, verbose
    python sil_game.py --dist 7 --apex 2 --tosses 12 --seed 3 --returns
    python sil_game.py --view --returns --apex 0.75 --apex-max 2 --tosses 20      watch it (MuJoCo's window)
    python sil_game.py --view --frames rendered --scene house                     the robot's left camera inset
"""

from __future__ import annotations

import argparse
import math
import time
from collections import Counter

import numpy as np

from catch_world import CatchWorld, WorldConfig
from sil_link import SimLink  # noqa: F401  (puts ros/src on the path)
from omni_catch.detector import DetectorConfig  # noqa: E402
from omni_catch.frames import ArrayFrame, SparseFrame  # noqa: E402
from omni_catch.game import CatchBehavior, GameParams, RobotView  # noqa: E402
from omni_catch.modes import ModeMux  # noqa: E402
from omni_catch.perception import Perception  # noqa: E402
from omni_catch.robot_config import RobotConfig  # noqa: E402
from omni_catch.tracker import TrackerConfig  # noqa: E402
from omni_mcu.protocol import Imu, State  # noqa: E402


def tracker_config(v) -> TrackerConfig:
    """What the filter is told about its cameras: the camera model's calibration figures (vision.ImageTracker)."""
    return TrackerConfig(calib_rot_deg=v.calib_rot_deg, calib_f_rel=v.calib_f_rel, ball_size_rel=v.ball_size_rel,
                         ego_sigma=v.ego_sigma, accel_sigma=v.accel_sigma, min_frames=v.min_frames)


def robot_frame(f):
    """A sim frame as the robot's code receives it."""
    if hasattr(f, "img"):
        return ArrayFrame(f.img)
    return SparseFrame(f.width, f.height, f.idx, f.clutter, f.seed)


class PerceptionScore:
    """Grades the detector's output per camera frame against the truth (never shown to the robot):
      on the ball   a blob within the ball's radius + 3 px of where the real ball projects
      false         a blob anywhere else (attributed to the nearest ball-coloured object of the house, if any)
      in flight     frames from release to the ball reaching the funnel, with the ball fully in the image and
                    nothing in front of its centre: the detection rate is on-the-ball / in-flight frames
    """

    def __init__(self, world):
        self.w = world
        self.cam_frames = 0
        self.flight = self.hits = self.false = 0
        self.err: list = []                 # px, centroid error on in-flight hits
        self.err_r: list = []               # px, the true radius of those
        self.size: list = []                # measured / true radius of those
        self.starts = self.false_starts = 0  # tracks started; ... on something that wasn't the ball in flight
        self.false_by: Counter = Counter()
        try:
            import house_scene
            self.clutter = dict(house_scene.CLUTTER) if world.p.scene == "house" else {}
        except ImportError:
            self.clutter = {}

    def _attribute(self, cam: int, u: float, v: float) -> str:
        c, Rc, f = self.w.cams._real_cam(cam)
        best, name = 40.0, "other"
        for k, xyz in self.clutter.items():
            s = Rc.T @ (np.asarray(xyz) - c)
            if s[2] >= 0:
                continue
            du = self.w.cams.cx + f * s[0] / -s[2] - u
            dv = self.w.cams.cy - f * s[1] / -s[2] - v
            if math.hypot(du, dv) < best:
                best, name = math.hypot(du, dv), k
        return name

    def add(self, pair, blobs, cur):
        by_cam = {b.cam: b for b in blobs}
        t_cap = pair[0].t_cap
        in_flight = (cur is not None and cur["t_release"] <= t_cap <= min(cur.get("t_in", math.inf),
                                                                           cur.get("t_result", math.inf)))
        if cur is not None:
            cur.setdefault("perc", dict(flight=0, hits=0, false=0))
        for f in pair:
            self.cam_frames += 1
            tr, bl = f.truth, by_cam.get(f.cam)
            on = bl is not None and tr is not None and math.hypot(bl.u - tr[0], bl.v - tr[1]) <= tr[2] + 3.0
            if bl is not None and not on:
                self.false += 1
                self.false_by[self._attribute(f.cam, bl.u, bl.v)] += 1
                if cur is not None and cur["t_release"] <= t_cap <= cur.get("t_result", math.inf):
                    cur["perc"]["false"] += 1
            if in_flight and tr is not None and tr[3]:
                self.flight += 1
                cur["perc"]["flight"] += 1
                if on:
                    self.hits += 1
                    cur["perc"]["hits"] += 1
                    self.err.append(math.hypot(bl.u - tr[0], bl.v - tr[1]))
                    self.err_r.append(tr[2])
                    if bl.b:
                        self.size.append(bl.b / tr[2])

    def started(self, pair, blobs, cur):
        """A track started on this pair: on the real ball (every blob it started from), or on something else?"""
        self.starts += 1
        truth = {f.cam: f.truth for f in pair}
        on = bool(blobs) and all(
            truth[b.cam] is not None and math.hypot(b.u - truth[b.cam][0], b.v - truth[b.cam][1]) <= truth[b.cam][2] + 3
            for b in blobs)
        self.false_starts += not on

    def summary(self) -> dict:
        e = np.array(self.err)
        return dict(cam_frames=self.cam_frames, flight_frames=self.flight, hits=self.hits, false=self.false,
                    detect_rate=self.hits / max(self.flight, 1),
                    center_err_med=float(np.median(e)) if len(e) else math.nan,
                    center_err_95=float(np.percentile(e, 95)) if len(e) else math.nan,
                    size_ratio_med=float(np.median(self.size)) if self.size else math.nan,
                    starts=self.starts, false_starts=self.false_starts,
                    false_by=dict(self.false_by))


class SilCatchGame:
    def __init__(self, cfg: WorldConfig | None = None, verbose: bool = False, game_kw: dict | None = None,
                 dcfg: DetectorConfig | None = None, rc: RobotConfig | None = None, **param_kw):
        """rc: the robot as the Jetson's code knows it (default robot_v11.json; when param_kw builds another robot,
        robot_export.robot_json of its params)."""
        self.cfg = c = cfg or WorldConfig()
        self.verbose = verbose
        self.world = w = CatchWorld(c, **param_kw)
        self.robot = w.robot
        self.link = SimLink(w.robot, seed=c.seed)
        self.rc = rc or RobotConfig()
        self.perc = Perception(self.rc, tracker_config(c.vision), dcfg)
        # what the robot is told about its surroundings, in its odom frame: home is where it starts, the thrower about
        # where the world puts them (a person detector or an AprilTag would say), the play area from the map
        hand = w.odom_from_world(w.thrower.hand[:2])
        ac = w.odom_from_world((0.0, 0.0))
        gp = GameParams(home=(0.0, 0.0), thrower=tuple(hand), arena_center=tuple(ac),
                        arena_half=(w.p.area_size[0] / 2, w.p.area_size[1] / 2), return_throws=c.return_throws,
                        return_height=c.return_height, release_height=c.release_height, **(game_kw or {}))
        self.perc.set_play_area(gp.arena_center, gp.arena_half)
        self.game = CatchBehavior(self.rc, gp, self.perc.flight, log=self._log)
        self.mux = ModeMux("CATCH")
        self.t_cmd = 0.0
        self.rv = RobotView()
        self.n_states = 0
        self.wall = 0.0
        self.score = PerceptionScore(w)
        self.last_view = None               # (left image, its blob, the truth) for the viewer's inset
        self._last_pair = None              # (the last frame pair, the blobs the robot's code reported)
        self.records = []                   # the robot's own record of each throw (game.last), as it logs them
        self.log = None                     # session_log.SessionLog (--log): the robot's format, plus the truth
        self._n_logged = 0
        self._t_pose = 0.0

    def open_log(self, folder):
        """Write this run as a session log (omni-log/1, the robot's own format: omni_console/session_log.py), with the
        world's truth beside each throw, so the sim's runs and the robot's sessions compare line by line."""
        from omni_console.session_log import SessionLog
        c = self.cfg
        self.log = SessionLog(folder, "CATCH", source="sim", map=None,          # t: sim seconds from here on
                              world=dict(dist=c.dist, apex_range=list(c.apex_range), seed=c.seed, frames=c.frames,
                                         scene=c.scene, throwers=c.throwers, returns=c.return_throws))
        return self.log

    def _log_step(self, t: float):
        g = self.game
        if g.last.get("toss") is not None and (not self.records or self.records[-1]["toss"] != g.last["toss"]):
            self.records.append(dict(g.last))
        if self.log is None:
            return
        if t - self._t_pose >= 0.5:
            self._t_pose = t
            p = self.rv.pose
            self.log.write("pose", t=round(t, 3), frame="odom", x=round(float(p[0]), 3), y=round(float(p[1]), 3),
                           yaw=round(float(p[2]), 3))
        res, cur = self.world.results, self.world.cur
        while self._n_logged < len(self.records):
            rec = self.records[self._n_logged]
            t0 = rec.get("t0") or 0.0
            if cur is not None and "t_result" not in cur and cur["t_release"] <= t0:
                break                                     # the world hasn't decided this throw yet
            # the world's throw this record is about: released before the robot's track started, the latest such
            match = [r for r in res if r["t_release"] <= t0 <= r["t_release"] + 2.5]
            truth = None
            if match:
                w = match[-1]
                truth = {k: w.get(k) for k in ("caught", "pos_error", "release", "release_v", "apex", "thrower",
                                               "return_error", "launched")}
            self.log.write("throw", t=round(rec.get("t0") or t, 3), frame="odom", player=f"thrower {rec.get('thrower', 0) + 1}",
                           **{k: v for k, v in rec.items() if k != "t0"}, truth=truth)
            self._n_logged += 1

    def close_log(self):
        if self.log is not None:
            # the robot's own record of the last throw can land a few ms after the world decided it (run() stops on
            # the world's count): let its game catch up, briefly
            t_end = self.world.d.time + 0.5
            while len(self.records) < len(self.world.results) and self.world.d.time < t_end:
                self.step()
            self._log_step(self.world.d.time)              # the throws the world decided last
            s = self.log.close(now=self.log.start + self.world.d.time)
            self.log = None
            return s
        return None

    def _log(self, msg: str):
        if self.verbose:
            print(f"[{self.world.d.time:7.2f}s] {msg}")

    def step(self):
        w, d = self.world, self.world.d
        t = d.time
        for msg in self.link.poll(t):
            if isinstance(msg, State):
                self.n_states += 1
                if not math.isnan(msg.t_host):
                    self.perc.add_pose(msg.t_host, msg.x, msg.y, msg.yaw, msg.roll, msg.pitch)
                    self.rv = RobotView.from_state(msg)
            elif isinstance(msg, Imu):
                pass
        for pair in w.pop_frames(t):
            was = self.perc.tracker.x is not None
            blobs = self.perc.process(pair[0].t_cap, [robot_frame(f) for f in pair])
            cur = w.cur
            self.score.add(pair, blobs, cur)
            if not was and self.perc.tracker.x is not None:
                self.score.started(pair, blobs, cur)
            if cur is not None and "t_track" not in cur and pair[0].t_cap >= cur["t_release"] and self.perc.ready():
                cur["t_track"] = t - cur["t_release"]            # the behaviour has a track from now on
            self._last_pair = (pair, blobs)
            if hasattr(pair[0], "img"):
                self.last_view = (pair[0].img, next((b for b in blobs if b.cam == 0), None), pair[0].truth)
        if t >= self.t_cmd - 1e-9:
            self.t_cmd = t + 0.01
            cc = self.game.step(t, self.rv, self.perc.view())
            self.link.send(self.mux.command(self.link.host.sync, catch=cc), t)
            self._log_step(t)
        w.step()

    def run(self, tosses: int, t_max: float = 400.0, view: bool = False):
        w0 = time.perf_counter()
        viewer, w_view = None, 0.0
        if view:                                   # MuJoCo's window, paced to real time
            import mujoco.viewer
            viewer = mujoco.viewer.launch_passive(self.world.m, self.world.d, show_left_ui=False, show_right_ui=False)
            with viewer.lock():
                viewer.cam.lookat[:] = (self.world.p.home_xy[0] - 2.0, 0.0, 0.5)
                viewer.cam.distance, viewer.cam.elevation, viewer.cam.azimuth = 6.0, -20.0, 160.0
        s0 = self.world.d.time
        while len(self.world.results) < tosses and self.world.d.time < t_max:
            n = len(self.world.results)
            self.step()
            if viewer is not None:
                if not viewer.is_running():
                    break
                now = time.perf_counter()
                if now - w_view > 1 / 30:
                    w_view = now
                    if self.last_view is not None:
                        viewer.set_images(self._inset(*self.last_view))
                    viewer.sync()
                ahead = (self.world.d.time - s0) - (now - w0)
                if ahead > 0.002:
                    time.sleep(ahead)
            if self.verbose and len(self.world.results) > n:
                r = self.world.results[-1]
                print(f"[{self.world.d.time:7.2f}s] world: {'CAUGHT' if r['caught'] else 'missed'}, funnel offset "
                      f"{100 * r['pos_error']:.1f} cm, arrival {r['arrival_deg']:.0f} deg, catch speed "
                      f"{r['catch_speed']:.2f} m/s")
        self.wall = time.perf_counter() - w0
        if viewer is not None:
            if hasattr(self.world.cams, "close"):
                self.world.cams.close()           # its GL context before the viewer ends GLFW
            viewer.close()
        return self.world.results

    @staticmethod
    def _inset(img, blob, truth, scale: int = 2):
        """The left camera at half size for the viewer, with the detector's blob (red) and the true ball (blue)."""
        import mujoco
        small = img[::scale, ::scale].copy()
        h, w = small.shape[:2]
        for obj, col, grow in ((truth, (40, 120, 255), 6), (blob, (255, 30, 30), 3)):
            if obj is None:
                continue
            u, v, r = (obj.u, obj.v, obj.b or 2.0) if hasattr(obj, "u") else obj[:3]
            x0, x1 = int((u - r) / scale) - grow, int((u + r) / scale) + grow
            y0, y1 = int((v - r) / scale) - grow, int((v + r) / scale) + grow
            x0, x1, y0, y1 = max(0, x0), min(w - 1, x1), max(0, y0), min(h - 1, y1)
            if x1 > x0 and y1 > y0:
                small[y0, x0:x1] = small[y1, x0:x1] = col
                small[y0:y1, x0] = small[y0:y1, x1] = col
        return (mujoco.MjrRect(10, 10, w, h), small)

    def summary(self) -> str:
        res = self.world.results
        k = sum(r["caught"] for r in res)
        off = np.array([r["pos_error"] for r in res])
        off = off[np.isfinite(off)]
        s = f"{k}/{len(res)} caught"
        if len(off):
            s += f", funnel offset median {100 * np.median(off):.1f} cm"
        ret = [r["return_error"] for r in res if r.get("launched")]
        if ret:
            s += f", returns land median {100 * np.nanmedian(ret):.0f} cm from the hands ({len(ret)} launched)"
        st = self.link.host.stats
        s += (f"; sim {self.world.d.time:.1f} s in {self.wall:.1f} s wall ({self.world.d.time / max(self.wall, 1e-9):.2f}x); "
              f"link {st.states} STATE, {st.imus} IMU, {st.errors} errors; sync offset error "
              f"{1e6 * abs(self.link.host.sync.offset_ns * 1e-9 - self.robot.boot_offset):.0f} us; "
              f"perception {self.perc.frames} frame pairs, {self.perc.detections} with a blob, coarse searches "
              f"{self.perc.det.n_coarse}")
        ps = self.score.summary()
        s += (f"\n  detector vs truth: ball found in {ps['hits']} of {ps['flight_frames']} in-flight camera frames "
              f"({100 * ps['detect_rate']:.0f}%), centre error median {ps['center_err_med']:.2f} px (95% "
              f"{ps['center_err_95']:.2f}), {ps['false']} false blobs in {ps['cam_frames']} camera frames; "
              f"{ps['starts']} tracks started, {ps['false_starts']} not on the ball")
        if ps["false_by"]:
            s += " (" + ", ".join(f"{k} {n}" for k, n in sorted(ps["false_by"].items(), key=lambda x: -x[1])) + ")"
        ms = getattr(self.world.cams, "ms_render", None)
        if ms:
            s += (f"\n  rendering: {np.median(ms):.1f} ms per camera frame (median; 95% {np.percentile(ms, 95):.1f}), "
                  f"{np.mean(self.world.cams.n_sub):.2f} sub-exposure renders per frame")
        return s


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dist", type=float, default=6.5)
    ap.add_argument("--apex", type=float, default=1.0)
    ap.add_argument("--tosses", type=int, default=6)
    ap.add_argument("--seed", type=int, default=9000)
    ap.add_argument("--returns", action="store_true")
    ap.add_argument("--apex-max", type=float, default=None, help="throws between --apex and this")
    ap.add_argument("--view", action="store_true", help="MuJoCo's window, at real time (or slower: rendered frames)")
    ap.add_argument("--frames", choices=("sparse", "rendered"), default="sparse")
    ap.add_argument("--scene", choices=("arena", "house"), default="arena")
    ap.add_argument("--acquire", type=int, default=None, help="DetectorConfig.acquire_candidates (default: its own)")
    ap.add_argument("--pass-game", action="store_true", help="two throwers taking turns; the robot returns to the other")
    ap.add_argument("--log", default=None, help="write the run as a session log (omni-log/1) into this folder")
    a = ap.parse_args()
    dcfg = DetectorConfig(acquire_candidates=a.acquire) if a.acquire else None
    g = SilCatchGame(WorldConfig(dist=a.dist, apex_range=(a.apex, a.apex_max or a.apex), seed=a.seed,
                                 return_throws=a.returns, frames=a.frames, scene=a.scene,
                                 throwers=2 if a.pass_game else 1), verbose=True, dcfg=dcfg,
                     game_kw=dict(return_to="other") if a.pass_game else None)
    if a.log:
        g.open_log(a.log)
    g.run(a.tosses, view=a.view)
    print(g.summary())
    if a.log:
        s = g.close_log()
        print(f"session log: {s['name']} ({s['throws']} throws) in {a.log}")


if __name__ == "__main__":
    main()
