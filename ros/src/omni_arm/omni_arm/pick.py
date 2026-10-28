"""Tidying up: find small things on the floor, drive to each, pick it up and drop it in the bin (design.md 11). No ROS:
arm_node runs it on the robot, simulation/v11 sil/sil_pick.py in the sim.

TidyTask.step(t, Inputs) -> Outputs every cycle (arm_node: 50 Hz):
  in   the base's pose (odom frame, from the MCU's odometry), the arm's state (driver.ArmState), the newest depth frame
       and wrist frame (each with its capture time: a look uses only frames taken after it asked)
  out  the base's body twist (None = stop: the mode manager ramps it down), the arm's goal (6 rad), which frames it
       wants now, arm_out (the base's "arm out" caps), and a status for the console

One object, start to end:
  LOOK       the base stands still; a depth frame -> floor_vision.detect -> FloorMap (odom). Nothing there: turn 60 deg
             and look again, a full turn at most; then back to where the tidy started for a full turn there (unless
             done since the last pick), then toward the nearest far sighting (beyond max_range: a hint), else done
  APPROACH   drive (holonomic, on odometry) until the object sits at the reach's sweet spot, facing it; a closer look
             there (the stereo error shrinks with range) and once more if it moved more than 2 cm
  HOVER      the arm from stow (through "ready" if the straight path isn't clear) to 8 cm over the object, fingers down,
             open to its width + 3 cm, turned to close across its short side
  REFINE     a wrist frame -> wrist_vision.refine; the hover moves to the refined spot (twice at most)
  DESCEND    the tool straight down until the fingertips are 5 mm off the floor
  CLOSE      close against the object; held if the fingers stopped apart (the gap says so), else open, up, and back to
             REFINE (two retries, then the object is marked failed)
  LIFT       straight up 12 cm, still holding?
  DROP       over the bin (through "ready"), open, back to stow
After the last object: stow, done. stop() stows the arm wherever the task is (with a held object: into the bin first).
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field

import numpy as np

from omni_arm import floor_vision as fv
from omni_arm import motion as mo
from omni_arm.kinematics import DOWN, ArmKinematics, gap_at, q_for_gap
from omni_arm.wrist_vision import WristFrame, box_corners, refine, self_mask

MJ_TO_OPTICAL = np.diag([1.0, -1.0, -1.0, 1.0])     # MuJoCo camera (looks along -z, y up) -> optical (z fwd, y down)
HELD_GAP = 0.006                                     # m: closed fingers this far apart hold something


def wrist_frame(kin: ArmKinematics, t: float, rgb, K, q) -> WristFrame:
    """A wrist image with its camera's pose from the arm's joint angles when it was taken (base_link, optical) and the
    pixels the gripper itself covers."""
    fk = kin.fk(q)
    T = fk["wrist_cam"] @ MJ_TO_OPTICAL
    wc = kin.cfg.wrist_cam
    boxes = [box_corners(fk["arm_gripper"], lo, hi) for lo, hi in wc["self_gripper"]] + \
            [box_corners(fk["arm_jaw"], lo, hi) for lo, hi in wc["self_jaw"]]
    return WristFrame(t, rgb, K, T, self_mask(rgb.shape[:2], K, T, boxes))


@dataclass
class PickParams:
    sweet: tuple = (0.33, 0.0)          # m, base_link: where the base brings an object before the reach
    tol_xy: float = 0.025
    tol_yaw: float = math.radians(4.0)
    v_max: float = 0.30                 # m/s while approaching (the stowed arm allows more; this is near clutter)
    v_slow: float = 0.10                # within 0.3 m of the goal
    w_max: float = 0.8                  # rad/s
    settle: float = 0.4                 # s still before a look
    look_turn: float = math.radians(60)
    hover_dz: float = 0.08              # m: tool over the object's top for the wrist's look
    tip_z: float = 0.005                # m: the fingertips over the floor at the grasp
    tip_z_low: float = 0.002            # m: ... for things under low_top tall (a bottle cap: every mm of pad counts)
    low_top: float = 0.02
    open_margin: float = 0.03           # m: opened to the object's width + this
    max_width: float = 0.065            # m: wider than this isn't tried
    side_clear: float = 0.008           # m: the fixed finger this far from the object's side as it comes down
    lift: float = 0.12
    retries: int = 2
    max_objects: int = 20
    area_radius: float = 2.0            # m: only things within this of where the tidy started (odom)
    max_range: float = 1.6              # m from the robot: farther detections wait for a closer look
    refine_moves: int = 1
    refine_tol: float = 0.004           # m: the refined spot this close to the hover's: go down
    arm_speed: float = mo.SPEED_FRAC


@dataclass
class Inputs:
    pose: tuple                         # (x, y, yaw) odom
    arm: object                         # driver.ArmState
    depth: fv.DepthFrame | None = None
    wrist: WristFrame | None = None


@dataclass
class Outputs:
    twist: tuple | None = None          # (vx, vy, wz) body frame
    arm_goal: np.ndarray | None = None
    want_depth: bool = False
    want_wrist: bool = False
    arm_out: bool = False
    status: dict = field(default_factory=dict)


class TidyTask:
    def __init__(self, kin: ArmKinematics, params: PickParams | None = None, log=None):
        # the reach's sweet spot is the layout's (robot_arm.json "reach"), unless the caller sets its own
        sweet = kin.cfg.raw.get("reach", {}).get("sweet")
        self.kin, self.p = kin, params or (PickParams(sweet=tuple(sweet)) if sweet else PickParams())
        self.log = log or (lambda m: None)
        self.map = fv.FloorMap()
        self.speed = mo.speeds(kin, self.p.arm_speed)
        c = kin.cfg
        self.q_closed, self.q_open_full = c.pads["gripper_closed"], c.pads["gripper_open"]
        self.cam = c.cameras[0]                          # the left camera: the depth image's frame
        self.T_base_cam = np.eye(4)
        self.T_base_cam[:3, :3] = np.array(self.cam["R"])
        self.T_base_cam[:3, 3] = self.cam["pos"]
        self.T_base_cam = self.T_base_cam @ MJ_TO_OPTICAL
        self.poses = self._poses()
        self.state, self.t_state = "IDLE", 0.0
        self.motion: mo.Motion | None = None
        self.goal = None
        self.target: fv.Tracked | None = None
        self.grasp = None                                 # (detection base_link, yaw, tool xyz, q)
        self.history = deque(maxlen=400)                  # (t, pose)
        self.t_req = math.inf
        self.looks = self.turned = 0
        self.tries = self.refines = 0
        self.picked = self.failed = 0
        self.skipped = 0                                  # too big for the gripper
        self.events: list = []
        self.done_reason = ""
        self.pose_goal = None
        self.stopping = False
        self.fresh = False
        self.hints: list = []                             # far sightings (odom xy) to go and look at
        self.recheck = False                              # after a failed grasp: a close look again once stowed
        self.close_looks = 0
        self.swept_origin = False                         # a full turn at the start since the last pick

    # -- fixed poses -----------------------------------------------------------------------------------------------
    def _poses(self) -> dict:
        k = self.kin
        if getattr(k, "_tidy_poses", None) is None:      # the same for every task on this arm: worked out once
            k._tidy_poses = self._find_poses()
        return {n: q.copy() for n, q in k._tidy_poses.items()}

    def _find_poses(self) -> dict:
        k = self.kin
        given = k.cfg.poses                              # the layout's own (robot_export.ARM_POSES), when it has them
        # stow: folded inside the bumper's edge, clear of the bin and the display, out of the cameras' view (the Oct 24
        # nose layout's: the upper arm up, the forearm folded down in front of it over the nose)
        stow = given["stow"].copy() if "stow" in given else np.array([0.0, -0.06, 1.65, 0.88, 0.0, self.q_closed])
        # high: the forearm raised; the way between stow and the work in front or the bin behind
        high = given["high"].copy() if "high" in given else np.array([0.0, -0.3, 1.3, 1.2, 0.0, self.q_closed])
        stow[5] = high[5] = self.q_closed
        # ready: fingers down 16 cm over the floor ahead (the highest a top-down tool goes here is ~0.2 m)
        ready = k.ik([0.28, 0.0, 0.16], DOWN, yaw=math.pi / 2, seed=np.array([0, -0.56, 0.59, 1.54, 0, 0]))
        ready[5] = self.q_closed
        bx, by, bz = k.cfg.bin["centre"]
        top = bz + k.cfg.bin["size"][2]
        # drop: fingers tipped back over the bin, as near its middle as the arm reaches behind itself (studies/
        # arm_mount.py: ~5 cm behind it, fingers 50-55 deg from down, the display backs the bin)
        drop = None
        if "drop" in given and k.clear(given["drop"], allow_bin=True)[0]:
            drop = given["drop"].copy()
        for dx in (() if drop is not None else (0.0, -0.025, -0.05, -0.06)):     # nearest the bin's middle first
            for dz in (0.08, 0.10, 0.12):
                for tilt in np.radians((45, 50, 55, 40, 60)):
                    a = np.array([-math.sin(tilt), 0.0, -math.cos(tilt)])
                    q = k.ik([bx + dx, by, top + dz], a, seed=np.array([0, -0.58, -1.69, -1.6, 0, 0]))
                    if q is not None and k.clear(q, allow_bin=True)[0]:
                        drop = q
                        break
                if drop is not None:
                    break
            if drop is not None:
                break
        if drop is None:
            raise RuntimeError("no clear drop pose over the bin")
        drop[5] = self.q_closed
        out = dict(stow=stow, high=high, ready=ready, drop=drop)
        if "predrop" in given:                          # folded back low, before the drop (a face over the bin)
            out["predrop"] = given["predrop"].copy()
            out["predrop"][5] = self.q_closed
        return out

    # -- helpers ---------------------------------------------------------------------------------------------------
    def _set(self, state: str, t: float, msg: str = ""):
        self.state, self.t_state = state, t
        self.fresh = True                                # the state's entry actions are due
        if msg:
            self.log(f"{state}: {msg}")
            self.events.append((round(t, 2), state, msg))

    def _move(self, t: float, q_now, waypoints, gripper=None):
        wps = [np.asarray(q_now, float)] + [np.asarray(w, float).copy() for w in waypoints]
        if gripper is not None:
            for w in wps[1:]:
                w[5] = gripper
        self.motion = mo.Motion(wps, self.speed, t)

    def _go(self, t: float, q_now, q_goal, gripper=None, allow_bin=False) -> bool:
        via = [self.poses[n] for n in ("high", "ready", "predrop") if n in self.poses]
        path = mo.plan(self.kin, q_now, q_goal, via=via, allow_bin=allow_bin)
        if path is None:
            return False
        self._move(t, q_now, path[1:], gripper)
        return True

    def _pose_at(self, t: float):
        best = None
        for tt, p in self.history:
            if tt <= t + 1e-6:
                best = p
        return best if best is not None else (self.history[-1][1] if self.history else (0.0, 0.0, 0.0))

    def _base_from_odom(self, pose, x, y):
        c, s = math.cos(pose[2]), math.sin(pose[2])
        dx, dy = x - pose[0], y - pose[1]
        return c * dx + s * dy, -s * dx + c * dy

    def arm_out(self, q) -> bool:
        return self.kin.tool_out(q)

    # -- the public side ---------------------------------------------------------------------------------------------
    def start(self, t: float, max_objects: int | None = None, area_radius: float | None = None):
        if max_objects:
            self.p.max_objects = max_objects
        if area_radius:
            self.p.area_radius = area_radius
        self.origin = self.history[-1][1] if self.history else None      # set on the first step otherwise
        self.looks = self.turned = 0
        self.stopping = False
        self.done_reason = ""
        self._set("STOW", t, "start: arm to stow")

    def stop(self, t: float):
        """Stow and end; an object already lifted goes into the bin first."""
        self.stopping = True
        if self.state not in ("IDLE", "DONE", "LIFT", "TO_BIN", "RELEASE", "STOW"):
            if self.target is not None and self.target.status == "new":
                self.target.status = "new"                # still on the floor: a later tidy finds it again
            self.target = None
            self._set("STOW", t, "stop: arm to stow")

    def status(self) -> dict:
        o = self.target
        return dict(state=self.state, picked=self.picked, failed=self.failed, skipped=self.skipped,
                    known=len(self.map.candidates()), target=None if o is None else dict(
                        id=o.id, x=round(o.det.x, 3), y=round(o.det.y, 3), size=[round(o.det.length, 3), round(o.det.width, 3)]),
                    tries=self.tries, reason=self.done_reason)

    # -- the cycle ---------------------------------------------------------------------------------------------------
    def step(self, t: float, inp: Inputs) -> Outputs:
        self.history.append((t, tuple(inp.pose)))
        if getattr(self, "origin", None) is None:
            self.origin = tuple(inp.pose)
        out = Outputs()
        q = np.asarray(inp.arm.q, float)
        if self.goal is None:
            self.goal = q.copy()
        h = getattr(self, "_s_" + self.state.lower())
        h(t, inp, out, q)
        if self.motion is not None:
            self.goal = self.motion.sample(t)
        out.arm_goal = self.goal.copy()
        out.arm_out = self.arm_out(self.goal) or self.arm_out(q)
        out.status = self.status()
        return out

    def _moving(self, t) -> bool:
        return self.motion is not None and not self.motion.done(t + 0.0)

    def _arrived(self, t, q, tol=0.03) -> bool:
        """The motion has played and the arm is there (the servos lag their goals a little)."""
        return (self.motion is None or self.motion.done(t)) and np.max(np.abs(q[:5] - self.goal[:5])) < tol

    # -- states ------------------------------------------------------------------------------------------------------
    def _s_idle(self, t, inp, out, q):
        pass

    def _s_done(self, t, inp, out, q):
        pass

    def _s_stow(self, t, inp, out, q):
        if self.fresh:
            self.fresh = False
            self.motion = None
            if np.max(np.abs(q[:5] - self.poses["stow"][:5])) > 0.03:
                if not self._go(t, q, self.poses["stow"], gripper=self.q_closed):
                    self._move(t, q, [self.poses["high"], self.poses["stow"]], gripper=self.q_closed)
            else:
                self.goal = self.poses["stow"].copy()
        if self._arrived(t, q, 0.05):
            self.motion = None
            if self.recheck and self.target is not None and not self.stopping:
                self.recheck = False
                self.t_req = t + self.p.settle
                self._set("CHECK", t)
                return
            self.recheck = False
            if self.stopping:
                self._set("DONE", t, "stopped")
                self.done_reason = "stopped"
            elif self.picked + self.failed >= self.p.max_objects:
                self._set("DONE", t, f"{self.picked} picked")
                self.done_reason = "max objects"
            else:
                self._begin_look(t)

    def _begin_look(self, t):
        self.t_req = t + self.p.settle
        self._set("LOOK", t)

    def _s_look(self, t, inp, out, q):
        out.want_depth = t >= self.t_req
        f = inp.depth
        if f is None or f.t < self.t_req:
            return
        pose = self._pose_at(f.t)
        all_dets = fv.detect(f, rng=np.random.default_rng(int(t * 1000) % 2 ** 31))
        dets = [d for d in all_dets if math.hypot(d.x, d.y) < self.p.max_range]
        ox, oy = self.origin[:2]
        for d in fv.to_frame([d for d in all_dets if math.hypot(d.x, d.y) >= self.p.max_range and not d.big], pose):
            if math.hypot(d.x - ox, d.y - oy) < self.p.area_radius and \
                    all(math.hypot(d.x - h[0], d.y - h[1]) > 0.3 for h in self.hints):
                self.hints.append((d.x, d.y))
        self.map.update(t, fv.to_frame(dets, pose), in_view=lambda x, y: self._in_view(pose, x, y))
        self.t_req = math.inf
        cands = [o for o in self.map.candidates() if o.det.width <= self.p.max_width
                 and math.hypot(o.det.x - ox, o.det.y - oy) < self.p.area_radius]
        if not cands:
            if self.turned * self.p.look_turn < 2 * math.pi - 1e-6:
                self.turned += 1
                self.pose_goal = (inp.pose[0], inp.pose[1], math.remainder(inp.pose[2] + self.p.look_turn, 2 * math.pi))
                self._set("TURN", t, f"nothing in view: turn {math.degrees(self.p.look_turn):.0f} deg")
                return
            self.turned = 0
            at_origin = math.hypot(inp.pose[0] - ox, inp.pose[1] - oy) < 0.3
            if at_origin:
                self.swept_origin = True
            if not self.swept_origin:
                self.swept_origin = True                  # once (until the next pick), whatever that look finds
                self.pose_goal = (ox, oy, inp.pose[2])
                self._set("TURN", t, "nothing here: back to the start for a look round")
                return
            # the nearest far sighting: drive to 0.9 m of it, facing it
            hints = [h for h in self.hints if math.hypot(h[0] - inp.pose[0], h[1] - inp.pose[1]) > 0.9]
            if hints:
                h = min(hints, key=lambda h: math.hypot(h[0] - inp.pose[0], h[1] - inp.pose[1]))
                self.hints.remove(h)
                a = math.atan2(h[1] - inp.pose[1], h[0] - inp.pose[0])
                self.turned = 6                       # look there once, no turn
                self.pose_goal = (h[0] - 0.9 * math.cos(a), h[1] - 0.9 * math.sin(a), a)
                self._set("TURN", t, f"something far off at ({h[0]:.2f}, {h[1]:.2f}): going closer")
                return
            self._set("DONE", t, f"nothing more in sight ({self.picked} picked, {self.failed} failed)")
            self.done_reason = "nothing in sight"
            return
        self.turned = 0
        px, py = inp.pose[0], inp.pose[1]
        self.target = min(cands, key=lambda o: math.hypot(o.det.x - px, o.det.y - py))
        self.tries = 0
        self._approach_goal(inp.pose)
        self._set("APPROACH", t, f"object {self.target.id} at ({self.target.det.x:.2f}, {self.target.det.y:.2f}), "
                                 f"{1000 * self.target.det.length:.0f} x {1000 * self.target.det.width:.0f} mm")
        self.close_looks = 0

    def _in_view(self, pose, x, y) -> bool:
        bx, by = self._base_from_odom(pose, x, y)
        ang = math.atan2(by - self.cam["pos"][1], bx - self.cam["pos"][0])
        r = math.hypot(bx, by)
        return 0.32 < r < 1.4 and abs(ang) < math.radians(38)

    def _approach_goal(self, pose):
        o = self.target.det
        heading = math.atan2(o.y - pose[1], o.x - pose[0])
        if math.hypot(o.x - pose[0], o.y - pose[1]) < 0.45:      # close already: keep the heading, slide
            heading = pose[2]
        sx, sy = self.p.sweet
        c, s = math.cos(heading), math.sin(heading)
        self.pose_goal = (o.x - (c * sx - s * sy), o.y - (s * sx + c * sy), heading)

    def _drive(self, pose, goal, out) -> bool:
        """A holonomic P step toward goal (odom); True when there."""
        bx, by = self._base_from_odom(pose, goal[0], goal[1])
        e_yaw = math.remainder(goal[2] - pose[2], 2 * math.pi)
        d = math.hypot(bx, by)
        if d < self.p.tol_xy and abs(e_yaw) < self.p.tol_yaw:
            out.twist = None
            return True
        v = min(self.p.v_max, max(self.p.v_slow if d < 0.3 else self.p.v_max, 0.0), 1.2 * d + 0.02)
        vx, vy = (bx / d * v, by / d * v) if d > 1e-4 else (0.0, 0.0)
        if d < self.p.tol_xy:
            vx = vy = 0.0
        w = max(-self.p.w_max, min(self.p.w_max, 2.0 * e_yaw))
        out.twist = (vx, vy, w)
        return False

    def _s_turn(self, t, inp, out, q):
        if self._drive(inp.pose, self.pose_goal, out):
            self._begin_look(t)

    def _s_approach(self, t, inp, out, q):
        if not self._drive(inp.pose, self.pose_goal, out):
            return
        self.t_req = t + self.p.settle
        self._set("CHECK", t)

    def _s_check(self, t, inp, out, q):
        """The close look from the sweet spot: is the object where the far look said?"""
        out.want_depth = t >= self.t_req
        f = inp.depth
        if f is None or f.t < self.t_req:
            return
        self.t_req = math.inf
        pose = self._pose_at(f.t)
        dets = fv.to_frame(fv.detect(f, rng=np.random.default_rng(7)), pose)
        o = self.target
        near = [d for d in dets if math.hypot(d.x - o.det.x, d.y - o.det.y) < 0.08]
        if not near:
            o.status = "gone"
            self._set("STOW", t, f"object {o.id} not there on the close look")
            self.target = None
            return
        d = min(near, key=lambda d: math.hypot(d.x - o.det.x, d.y - o.det.y))
        moved = math.hypot(d.x - o.det.x, d.y - o.det.y)
        self.map.replace(o, d)
        if d.width > self.p.max_width or d.big:
            o.status = "skipped"
            self.skipped += 1
            self._set("STOW", t, f"object {o.id}: {1000 * d.length:.0f} x {1000 * d.width:.0f} mm, too big for the gripper")
            return
        bx, by = self._base_from_odom(inp.pose, d.x, d.y)
        if (moved > 0.02 or math.hypot(bx - self.p.sweet[0], by - self.p.sweet[1]) > 0.04) and self.close_looks < 2:
            self.close_looks += 1
            self._approach_goal(inp.pose)
            self._set("APPROACH", t, f"re-centre on object {o.id} ({100 * moved:.1f} cm off)")
            return
        self._plan_grasp(t, inp, q, d)

    def _grasp_for(self, pose, d: fv.Detection, q_seed):
        """(tool point at the hover, at the grasp, q at the hover, gripper opening) for detection d (odom)."""
        bx, by = self._base_from_odom(pose, d.x, d.y)
        yaw_b = math.remainder(d.yaw - pose[2], math.pi)
        close_dir = yaw_b + math.pi / 2                       # close across the short side
        if d.length - d.width < 0.006:                        # round or square: whichever turns the wrist least
            close_dir = math.atan2(by - self.kin.mount[1], bx - self.kin.mount[0]) + math.pi / 2
        best = None
        for yaw in (close_dir, close_dir + math.pi):
            o = np.array([math.cos(yaw), math.sin(yaw), 0.0])
            ctr = np.array([bx, by, 0.0])
            tool = ctr - o * (d.width / 2 + self.p.side_clear)
            tool[2] = (self.p.tip_z_low if d.top < self.p.low_top else self.p.tip_z) + \
                self.kin.cfg.pads["fingertip_below_tool"]
            hover = tool + np.array([0, 0, d.top + self.p.hover_dz])
            qh = self.kin.ik(hover, DOWN, yaw=yaw, seed=q_seed, flip_ok=False)
            if qh is None or not self.kin.clear(qh)[0]:
                continue
            if best is None or abs(qh[4]) < abs(best[2][4]):
                best = (hover, tool, qh, yaw)
        return best

    def _plan_grasp(self, t, inp, q, d):
        g = self._grasp_for(inp.pose, d, q)
        if g is None:
            self.target.status = "failed"
            self.failed += 1
            bx, by = self._base_from_odom(inp.pose, d.x, d.y)
            self._set("STOW", t, f"object {self.target.id}: no grasp within reach (at {bx:.2f}, {by:.2f} from the "
                                 f"robot, {1000 * d.length:.0f} x {1000 * d.width:.0f} mm)")
            return
        hover, tool, qh, yaw = g
        gap = min(gap_at(self.kin.cfg, 1.5), d.width + self.p.open_margin)
        qh[5] = q_for_gap(self.kin.cfg, gap)
        if not self._go(t, q, qh, gripper=qh[5]):
            self.target.status = "failed"
            self.failed += 1
            self._set("STOW", t, f"object {self.target.id}: no clear path to the hover")
            return
        self.grasp = dict(det=d, hover=hover, tool=tool, yaw=yaw, q=qh, opening=qh[5])
        self.refines = 0
        self._set("HOVER", t, f"over object {self.target.id}")

    def _s_hover(self, t, inp, out, q):
        if self._arrived(t, q):
            self.t_req = t + 0.25
            self._set("REFINE", t)

    def _s_refine(self, t, inp, out, q):
        out.want_wrist = t >= self.t_req
        f = inp.wrist
        if f is None or f.t < self.t_req:
            return
        self.t_req = math.inf
        g = self.grasp
        pose = self._pose_at(f.t)
        bx, by = self._base_from_odom(pose, g["det"].x, g["det"].y)
        guess = fv.Detection(bx, by, g["det"].top, g["det"].length, g["det"].width,
                             math.remainder(g["det"].yaw - pose[2], math.pi), g["det"].n)
        r = refine(f, guess)
        if r is not None:
            c, s = math.cos(pose[2]), math.sin(pose[2])
            d = fv.Detection(pose[0] + c * r.x - s * r.y, pose[1] + s * r.x + c * r.y, guess.top, r.length,
                             min(r.width, g["det"].width + 0.01), math.remainder(r.yaw + pose[2], math.pi), r.n)
            shift = math.hypot(r.x - bx, r.y - by)
            if shift > self.p.refine_tol and self.refines < self.p.refine_moves:
                self.refines += 1
                gg = self._grasp_for(inp.pose, d, q)
                if gg is not None:
                    hover, tool, qh, yaw = gg
                    qh[5] = g["opening"]
                    self._move(t, q, [qh])
                    self.grasp.update(det=d, hover=hover, tool=tool, yaw=yaw, q=qh)
                    self._set("HOVER", t, f"wrist camera: {1000 * shift:.0f} mm off, moving over it")
                    return
            self.grasp["det"] = d
        else:
            self.log("REFINE: the wrist camera didn't find it: grasping on the stereo estimate")
        wps = mo.line(self.kin, q if self.refines == 0 else self.goal, self.grasp["tool"], DOWN, self.grasp["yaw"])
        if wps is None:
            self._fail(t, q, "no straight way down")
            return
        self._move(t, self.goal, wps, gripper=self.grasp["opening"])
        self._set("DESCEND", t)

    def _s_descend(self, t, inp, out, q):
        if self._arrived(t, q, 0.04):
            g = self.goal.copy()
            g[5] = self.q_closed
            self._move(t, self.goal, [g])
            self.motion.t[-1] = t + 0.6                      # close over 0.6 s
            self._set("CLOSE", t)

    def _s_close(self, t, inp, out, q):
        if t - self.t_state < 1.0:
            return
        # held: the fingers stopped apart (the narrowest thing it picks is ~8 mm; closed on nothing they meet at 3 mm)
        held = gap_at(self.kin.cfg, q[5]) > HELD_GAP
        up = self.grasp["tool"] + np.array([0, 0, self.p.lift])
        wps = mo.line(self.kin, self.goal, up, DOWN, self.grasp["yaw"], step=0.01)
        if not held:
            self._fail(t, q, f"missed (gripper at {math.degrees(q[5]):.0f} deg)", wps)
            return
        if wps is None:
            wps = [self.poses["ready"]]
        self._move(t, self.goal, wps, gripper=self.q_closed)
        self._set("LIFT", t, f"holding object {self.target.id} ({1000 * gap_at(self.kin.cfg, q[5]):.0f} mm)")

    def _fail(self, t, q, why, wps=None):
        """A grasp that missed (or a dropped object): up, the arm out of the cameras' way (stow), a fresh close look
        (the fingers may have nudged it), then the grasp again from there."""
        self.tries += 1
        o = self.target
        if self.tries > self.p.retries:
            o.status = "failed"
            self.failed += 1
            self._set("STOW", t, f"object {o.id}: {why}; giving up after {self.tries} tries")
            self.target = None
            return
        self.recheck = True
        self.close_looks = 0
        self._set("STOW", t, f"object {o.id}: {why}; try {self.tries + 1} after a fresh look")

    def _s_lift(self, t, inp, out, q):
        if not self._arrived(t, q, 0.05):
            return
        if gap_at(self.kin.cfg, q[5]) <= HELD_GAP:
            self._fail(t, q, "dropped it on the way up")
            return
        drop = self.poses["drop"].copy()
        drop[5] = self.q_closed
        if not self._go(t, q, drop, gripper=self.q_closed, allow_bin=True):
            self._move(t, q, [self.poses[n] for n in ("high", "predrop") if n in self.poses] + [drop],
                       gripper=self.q_closed)
        self._set("TO_BIN", t)

    def _s_to_bin(self, t, inp, out, q):
        if self._arrived(t, q, 0.05):
            g = self.goal.copy()
            g[5] = q_for_gap(self.kin.cfg, 0.07)
            self._move(t, self.goal, [g])
            self._set("RELEASE", t)

    def _s_release(self, t, inp, out, q):
        if t - self.t_state < 0.8:
            return
        o = self.target
        o.status = "picked"
        self.picked += 1
        self.swept_origin = False
        self._set("STOW", t, f"object {o.id} in the bin ({self.picked} so far)")
        self.target = None
        self.motion = None
