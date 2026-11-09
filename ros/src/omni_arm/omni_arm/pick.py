"""Tidying up: find small things on the floor, drive to each, pick it up and drop it in the bin (design.md 11). No ROS:
arm_node runs it on the robot, simulation/v11 sil/sil_pick.py in the sim.

TidyTask.step(t, Inputs) -> Outputs every cycle (arm_node: 50 Hz):
  in   the base's pose (odom frame, from the MCU's odometry), the arm's state (brain_core.ArmState), the newest depth frame
       and wrist frame (each with its capture time: a look uses only frames taken after it asked)
  out  the base's body twist (None = stop: the mode manager ramps it down), the arm's goal (6 rad), which frames it
       wants now, arm_out (the base's "arm out" caps), and a status for the console

One object, start to end:
  LOOK       the base stands still; a depth frame -> floor_vision.detect -> FloorMap (odom), and the floor it saw ->
             coverage.Coverage (the region's floor checked, what blocks the base). Nothing there: turn 60 deg and look
             again, a full turn at most while the ring round the base has floor still to check; then
  GO         to the reachable spot whose look round checks the most of the region's unchecked floor per second, through
             floor seen free (coverage.Coverage.plan); a look round there. Done when no reachable look checks more: the
             whole region seen, or what is left hidden or out of reach (status: coverage)
  APPROACH   drive (holonomic, on odometry) until the object sits at the reach's sweet spot, facing it; a closer look
             there (the stereo error shrinks with range) and once more if it moved more than 2 cm. A camera that can't
             see the sweet spot (the nose D435 tilted for the room, design.md 4.11) looks from look_back m short of it,
             then CREEP: the base slides that far in on odometry
  HOVER      the arm from stow (through "ready" if the straight path isn't clear) to 8 cm over the object, fingers down,
             open to its width + 3 cm, turned to close across its short side
  REFINE     a wrist frame -> wrist_vision.refine; the hover moves to the refined spot (twice at most)
  DESCEND    the tool straight down until the fingertips are 5 mm off the floor
  CLOSE      close against the object; held if the fingers stopped apart (the gap says so), else open, up, and back to
             REFINE (two retries, then the object is marked failed)
  LIFT       straight up 12 cm, still holding?
  DROP       over the bin (through "ready"), open, back to stow
After the last object: stow, done. stop() stows the arm wherever the task is (with a held object: into the bin first).

start(only=id, fmap=...) picks one object an AI's find chose (ai_robot_architecture.md 5.2): the find's floor map,
so the ids agree; only that object is a candidate, and the task ends when it is in the bin or can't be: outcome
picked | failed | lost (not there on the close look, or dropped from the map) | too_big | not_found (not in the map,
or out of the area), with no turning round to look for others. stages say how far it got: attempted (it went down
for it), grasped (it lifted it held), released (opened over the bin).

Carry and put down (the console's Fetch and Block tower): start(keep=True) keeps the object instead of binning it:
  HELD_LOOK  after LIFT, a wrist frame of the block in the fingers (wrist_vision.held): where it sits across the pads
             and how askew, so it goes down where it should and lined up (none found: centred, square, as before)
  CARRY      then the arm to "ready" still holding (the base is capped as arm-out); the task ends with outcome
             held, the arm holding there
start_place(...) puts the held object down: at a floor point (odom), or "ahead" (the sweet spot where the base is):
  P_APPROACH drive so the spot sits at the sweet spot (for a tower: the close look's spot, then P_CHECK and P_CREEP)
  P_CHECK    a depth look finds the tower again (its place and its top's height); not there: put down on the floor
  P_HOVER    over the spot, the held object's bottom place_dz above where it goes, turned to line up with the top block
             (the block's own faces: the grip's yaw less its measured skew; its centre where HELD_LOOK saw it)
  P_DESCEND  straight down until the object's bottom is place_gap over its place (the grasp's own fingertip offset)
  P_OPEN     open; P_RISE straight up; then stow (outcome placed)
start(tower=True) builds a tower of blocks (cube-like things, block_size): the first block it finds is the tower's
base and stays where it is; each other block is picked (kept) and put on top, until max_blocks or none is left in the
region. stop() while holding puts the object in the bin first, as a tidy does.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field

import numpy as np

from omni_arm import coverage as cv
from omni_arm import floor_vision as fv
from omni_arm import motion as mo
from omni_arm.kinematics import DOWN, ArmKinematics, gap_at, q_for_gap
from omni_arm.wrist_vision import WristFrame, box_corners, held as held_block, refine, self_mask

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


def camera_view(kin: ArmKinematics, q) -> dict:
    """Where the wrist camera points now (base_link): elevation and azimuth of its axis (deg, + up / + left), its
    height, and whether its image is upside down (its up axis below the horizon): robot_look turns that one round."""
    T = kin.fk(np.asarray(q, float))["wrist_cam"]                  # a MuJoCo camera: looks along -z, y up
    look, up = -T[:3, 2], T[:3, 1]
    return dict(elevation_deg=round(math.degrees(math.asin(max(-1.0, min(1.0, look[2])))), 1),
                azimuth_deg=round(math.degrees(math.atan2(look[1], look[0])), 1), height_m=round(float(T[2, 3]), 3),
                upside_down=bool(up[2] < -0.3))


@dataclass
class PickParams:
    sweet: tuple = (0.33, 0.0)          # m, base_link: where the base brings an object before the reach
    look_back: float = 0.0              # m: the close look this far short of the sweet spot (then CREEP in)
    view_near: float = 0.32             # m from the centre: nearer than this the depth camera doesn't see the floor
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
    # carrying and putting down (Fetch, Block tower)
    place_dz: float = 0.04              # m: the held object's bottom this far over its place before going down
    place_gap: float = 0.004            # m: ... and this far over it when the gripper opens (it drops the rest)
    rise: float = 0.07                  # m: straight up after letting go
    release_slide: float = 0.012        # m: the fixed finger slid off the object's side before going up
    block_size: tuple = (0.025, 0.06)   # m: a block for the tower: its top and its short side within this
    block_long: float = 0.10            # m: its long side at most (the depth smears a 40 mm cube's footprint along
                                        #   the view to 1.1-1.85 x its width: color_vision's notes)
    max_blocks: int = 4                 # the tallest tower tried (the fingers-down reach tops out near 0.2 m)
    tower_find: float = 0.08            # m: the tower found again within this of where it was
    tower_keep: float = 0.32            # m: the base's centre keeps this far from the tower (its bumper ~9 cm off)
    regrip_gap: float = 0.009           # m: unseen, a block held this much wider than it is was gripped askew: down,
                                        #   again (square it reads ~5.7 mm wide in the sim: the pads; 10 deg adds ~6)
    regrip_skew: float = math.radians(10)  # rad: seen, a block turned this much in the fingers: down, again
    held_look: bool = True              # carrying (Fetch, the tower): a wrist frame of where it sits in the fingers


@dataclass
class Inputs:
    pose: tuple                         # (x, y, yaw) odom
    arm: object                         # brain_core.ArmState
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
        # the reach's sweet spot, the close look's step back and the camera's nearest floor are the layout's
        # (robot_arm.json "reach"), unless the caller sets its own
        r = kin.cfg.raw.get("reach", {})
        kw = {k: (tuple(r[k]) if k == "sweet" else float(r[k])) for k in ("sweet", "look_back", "view_near") if k in r}
        # the camera's horizontal half field: the look round's turns overlap by >= 10 deg (60 deg for the landscape D435)
        c0 = kin.cfg.cameras[0]
        wide = math.atan(math.tan(math.radians(c0["fovy_deg"]) / 2) * c0["width"] / c0["height"])
        kw["look_turn"] = min(math.radians(60), 2 * wide - math.radians(10))
        self.kin, self.p = kin, params or PickParams(**kw)
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
        # the bearing a floor thing must be within to count as in view: 38 deg (the landscape D435's 44.6 deg half
        # field less a margin), less for a narrower camera (a D435 on its side: 29 deg half field, 22 deg)
        self.view_half = min(math.radians(38), wide - math.radians(6.6))
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
        self.no_path = ""                                 # why the last _go found no checked path
        self.pose_goal = None
        self.stopping = False
        self.fresh = False
        self.recheck = False                              # after a failed grasp: a close look again once stowed
        self.close_looks = 0
        self.checked = None                               # the close look's detection, for the grasp after CREEP
        self.cover: cv.Coverage | None = None             # the region's floor checked (made at the first look)
        self.cover_p = cv.CoverParams()
        self.path: list = []                              # GO's waypoints (odom xy)
        self.at_spot = False                              # GO arrived: a whole look round here
        self.only = None                                  # one object's id (an AI's pick), or None: everything
        self.outcome = ""                                 # how it ended for that object
        self.stages = dict(attempted=False, grasped=False, released=False)
        self.keep = False                                 # keep what it lifts (CARRY) instead of the bin
        self.held = None                                  # dict(id, width, height, gripper yaw) while it holds a thing
        self.place = None                                 # dict(x, y, z (its bottom's height), yaw, tower) to put it
        self.tower = None                                 # dict(id, x, y, top, n, yaw) while building a tower
        self.mode = "tidy"                                # tidy | keep | place | tower
        self.regrips: dict = {}                           # object id -> times put down to grip it square

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
        # look: the wrist camera straight ahead over the nose, 24 deg up from 0.57 m, its horizon level (the floor
        # camera sees only the floor): an AI's eye for walls, signs and faces (ai_robot_architecture.md). The pan and
        # the roll undo the camera's offset beside the jaw. Found by search over clear poses with checked paths from and
        # to stow (est geometry: the kinematic model's camera mount). The image comes upside down (an upright one needs
        # the wrist roll past its range): camera_view says so and robot_look turns it round.
        look = np.array([-0.3, -0.609, -0.668, 1.102, 0.16, self.q_closed])
        if k.clear(look)[0]:
            out["look"] = look
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
        q_now = np.asarray(q_now, float)
        path, why = mo.plan_checked(self.kin, q_now, q_goal, via=via, allow_bin=allow_bin)
        if path is None and self.goal is not None and np.max(np.abs(q_now[:5] - self.goal[:5])) < 0.05:
            # from where the arm is commanded (the servos lag it a little): the way out of the bin passes the face
            # closely, and a few mrad of lag can close it
            path, why = mo.plan_checked(self.kin, self.goal, q_goal, via=via, allow_bin=allow_bin)
            q_now = self.goal.copy()
        if path is None:
            self.no_path = why
            return False
        if why:
            self.log(f"{self.state}: {why}")
        self._move(t, q_now, path[1:], gripper)
        return True

    def _halt(self, t: float, q, why: str):
        """No checked way on: the task ends with the arm held where it is commanded (nothing unchecked plays)."""
        self.motion = None
        self.goal = (self.goal if self.goal is not None else np.asarray(q, float)).copy()
        self.done_reason = why
        self._set("DONE", t, f"{why}: holding here")

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
    def start(self, t: float, max_objects: int | None = None, area_radius: float | None = None,
              only: int | None = None, fmap: fv.FloorMap | None = None, keep: bool = False, tower: bool = False,
              max_blocks: int | None = None):
        if self.held is not None:
            raise ValueError("the arm holds something: put it down (start_place) or stop first")
        self.keep, self.tower = keep or tower, None
        self.mode = "tower" if tower else "keep" if keep else "tidy"
        self.place = None
        if max_blocks:
            self.p.max_blocks = max_blocks
        if max_objects:
            self.p.max_objects = max_objects
        if area_radius:
            self.p.area_radius = area_radius
        if fmap is not None:
            self.map = fmap
        self.only = only
        for o in self.map.tracked:
            if o.id == only and o.status == "failed":
                o.status = "new"                          # asked again: a fresh try
        self.outcome = ""
        self.stages = dict(attempted=False, grasped=False, released=False)
        self.origin = self.history[-1][1] if self.history else None      # set on the first step otherwise
        self.cover = None                                 # a fresh region round where this tidy starts
        self.looks = self.turned = 0
        self.stopping = False
        self.done_reason = ""
        self._set("STOW", t, "start: arm to stow")

    def start_place(self, t: float, x: float | None = None, y: float | None = None, z: float = 0.0,
                    ahead: bool = False):
        """Put the held object down: at (x, y) odom with its bottom at height z, or ahead (the sweet spot where the base
        stands now, on the floor)."""
        if self.held is None:
            raise ValueError("nothing held")
        pose = self.history[-1][1] if self.history else (0.0, 0.0, 0.0)
        if ahead or x is None or y is None:
            sx, sy = self.p.sweet
            c, s_ = math.cos(pose[2]), math.sin(pose[2])
            x, y, z = pose[0] + c * sx - s_ * sy, pose[1] + s_ * sx + c * sy, 0.0
        self.mode, self.stopping, self.done_reason, self.outcome = "place", False, "", ""
        self.place = dict(x=float(x), y=float(y), z=float(z), yaw=None, tower=False, ahead=bool(ahead))
        self._begin_place(t, pose)

    def stop(self, t: float):
        """Stow and end; an object already lifted goes into the bin first."""
        self.stopping = True
        self.outcome = self.outcome or "stopped"
        if self.held is not None and self.state in ("DONE", "IDLE", "CARRY", "HELD_LOOK") or self.state.startswith("P_") \
                and self.state not in ("P_OPEN", "P_RISE"):
            q = self.goal if self.goal is not None else np.zeros(6)
            self.place = None
            self._to_bin(t, q)                           # what it holds goes in the bin, as a tidy's would
            return
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
                    tries=self.tries, reason=self.done_reason, only=self.only, outcome=self.outcome,
                    stages=dict(self.stages), coverage=None if self.cover is None else self.cover.coverage(),
                    mode=self.mode, held=None if self.held is None else dict(self.held),
                    tower=None if self.tower is None else dict(n=self.tower["n"], top=round(self.tower["top"], 3),
                                                               x=round(self.tower["x"], 3), y=round(self.tower["y"], 3)))

    def _only_outcome(self) -> str:
        """For an only-pick: how it ended for its object ('' while it is still to do)."""
        o = next((x for x in self.map.tracked if x.id == self.only), None)
        if o is None:
            return "not_found"
        return {"picked": "picked", "failed": "failed", "gone": "lost", "skipped": "too_big"}.get(o.status, "")

    def _end_only(self, t: float, outcome: str):
        self.outcome = outcome
        self.done_reason = outcome
        self._set("DONE", t, f"object {self.only}: {outcome.replace('_', ' ')}")

    # -- the cycle ---------------------------------------------------------------------------------------------------
    def step(self, t: float, inp: Inputs) -> Outputs:
        self.history.append((t, tuple(inp.pose)))
        if getattr(self, "origin", None) is None:
            self.origin = tuple(inp.pose)
        if self.cover is not None:
            self.cover.occupy(inp.pose[0], inp.pose[1])
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
                    self._halt(t, q, f"no checked way to stow ({self.no_path})")
                    return
            else:
                self.goal = self.poses["stow"].copy()
        if self._arrived(t, q, 0.05):
            self.motion = None
            if self.recheck and self.target is not None and not self.stopping:
                self.recheck = False
                if self.p.look_back > 0:                  # the sweet spot is out of the camera's view: back off
                    self._approach_goal(inp.pose)
                    self._set("APPROACH", t, f"back {100 * self.p.look_back:.0f} cm for a fresh look")
                    return
                self.t_req = t + self.p.settle
                self._set("CHECK", t)
                return
            self.recheck = False
            if self.stopping:
                self._set("DONE", t, self.done_reason or "stopped")
                self.done_reason = self.done_reason or "stopped"
            elif self.only is not None and self._only_outcome():
                self._end_only(t, self._only_outcome())
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
        P = fv.points(f)
        P = P[np.hypot(P[:, 0], P[:, 1]) > 0.23]                  # as fv.detect: the robot's own bumper out
        cam_xy = f.T_base_cam[:2, 3]
        all_dets, h, _ = fv.detect_points(P, cam_xy, np.random.default_rng(int(t * 1000) % 2 ** 31))
        dets = [d for d in all_dets if math.hypot(d.x, d.y) < self.p.max_range]
        ox, oy = self.origin[:2]
        if self.cover is None:
            self.cover = cv.Coverage((ox, oy), self.p.area_radius, self.cover_p)
            self.cover.occupy(ox, oy, self.p.view_near + 0.05)     # where it starts: it got there, so it is clear
        self.cover.update(pose, P, h, cam_xy, all_dets)
        self.map.update(t, fv.to_frame(dets, pose), in_view=lambda x, y: self._in_view(pose, x, y))
        self.t_req = math.inf
        cands = [o for o in self.map.candidates() if o.det.width <= self.p.max_width
                 and math.hypot(o.det.x - ox, o.det.y - oy) < self.p.area_radius]
        if self.mode == "tower":
            cands = [o for o in cands if self.is_block(o.det)]
            if self.tower is None and cands:             # the first block is the tower's base: it stays where it is
                px, py = inp.pose[0], inp.pose[1]
                base = min(cands, key=lambda o: math.hypot(o.det.x - px, o.det.y - py))
                base.status = "tower"
                self.tower = dict(id=base.id, x=base.det.x, y=base.det.y, top=base.det.top, n=1, yaw=base.det.yaw,
                                  size=base.det.top)
                self._set("LOOK", t, f"tower: object {base.id} at ({base.det.x:.2f}, {base.det.y:.2f}) is its base")
                cands = [o for o in cands if o is not base]
            if self.tower is not None and self.tower["n"] >= self.p.max_blocks:
                self._set("DONE", t, f"tower: {self.tower['n']} blocks high")
                self.done_reason = self.outcome = "tower done"
                return
        if self.only is not None:
            cands = [o for o in cands if o.id == self.only]
            if not cands:                                 # gone, too big, out of the area: no looking round for it
                o = next((x for x in self.map.tracked if x.id == self.only), None)
                why = self._only_outcome() or ("too_big" if o is not None and (o.det.big or o.det.width >
                                                                              self.p.max_width) else "not_found")
                self._end_only(t, why)
                return
        if not cands:
            x, y = inp.pose[0], inp.pose[1]
            near, far = self._look_ring()
            # a whole look round at a spot GO came to (it was planned for it), else while the ring has floor to check
            if self.turned * self.p.look_turn < 2 * math.pi - 1e-6 and (self.at_spot or self.cover.todo_near(x, y, near, far)):
                self.turned += 1
                self.pose_goal = (x, y, math.remainder(inp.pose[2] + self.p.look_turn, 2 * math.pi))
                self._set("TURN", t, f"nothing in view: turn {math.degrees(self.p.look_turn):.0f} deg")
                return
            if self.turned:
                self.cover.end_look(x, y, near, far)
            self.turned = 0
            self.at_spot = False
            plan = self.cover.plan(inp.pose, near, far)
            c = self.cover.coverage()
            if plan is not None:
                self.path = list(plan.path)
                self._set("GO", t, f"to look from ({plan.goal[0]:.2f}, {plan.goal[1]:.2f}), {plan.dist:.1f} m: "
                                   f"{plan.gain:.2f} m^2 to check there ({100 * c['share']:.0f}% checked)")
                return
            self._set("DONE", t, f"region covered: {100 * c['share']:.0f}% of its floor checked, {c['unchecked']:.2f} m^2 "
                                 f"hidden or out of reach ({self.picked} picked, {self.failed} failed)")
            self.done_reason = "region covered"
            if self.mode == "tower":
                self.outcome = "tower done" if self.tower is not None else "no blocks"
            return
        self.turned = 0
        self.at_spot = False
        px, py = inp.pose[0], inp.pose[1]
        self.target = min(cands, key=lambda o: math.hypot(o.det.x - px, o.det.y - py))
        self.tries = 0
        self._approach_goal(inp.pose)
        self._set("APPROACH", t, f"object {self.target.id} at ({self.target.det.x:.2f}, {self.target.det.y:.2f}), "
                                 f"{1000 * self.target.det.length:.0f} x {1000 * self.target.det.width:.0f} mm")
        self.close_looks = 0

    def _look_ring(self) -> tuple:
        """The ring round the base (m from its centre) a look round checks: from the camera's nearest floor out to the
        coverage's check range from the camera."""
        return self.p.view_near, self.cover_p.check_range + math.hypot(*self.T_base_cam[:2, 3])

    def _s_go(self, t, inp, out, q):
        """To the next look's spot, leg by leg, facing the way (the camera sees the floor ahead), through floor the
        coverage saw free when it planned (looks are taken stopped: nothing new is seen on the way)."""
        if not self.path:
            self.at_spot = True
            self._begin_look(t)
            return
        wx, wy = self.path[0]
        d = math.hypot(wx - inp.pose[0], wy - inp.pose[1])
        heading = math.atan2(wy - inp.pose[1], wx - inp.pose[0]) if d > 0.05 else inp.pose[2]
        if self._drive(inp.pose, (wx, wy, heading), out):
            self.path.pop(0)

    def _in_view(self, pose, x, y) -> bool:
        bx, by = self._base_from_odom(pose, x, y)
        ang = math.atan2(by - self.cam["pos"][1], bx - self.cam["pos"][0])
        r = math.hypot(bx, by)
        # (the image's lower edge meets the floor along a line across the view: x, not the radius)
        return bx > self.p.view_near and r < 1.4 and abs(ang) < self.view_half

    def _look_spot(self) -> tuple:
        """Where the close look puts the object (base_link): the sweet spot, or look_back short of it."""
        return self.p.sweet[0] + self.p.look_back, self.p.sweet[1]

    def _approach_goal(self, pose):
        o = self.target.det
        heading = math.atan2(o.y - pose[1], o.x - pose[0])
        if math.hypot(o.x - pose[0], o.y - pose[1]) < 0.45 + self.p.look_back:   # close already: keep the heading, slide
            heading = pose[2]
        sx, sy = self._look_spot()
        c, s = math.cos(heading), math.sin(heading)
        self.pose_goal = (o.x - (c * sx - s * sy), o.y - (s * sx + c * sy), heading)

    def _tower_detour(self, pose, goal):
        """While building, the base never drives through the tower (it would knock it over). When the straight way to
        goal passes within tower_keep of it: straight away from it out to a ring tower_keep + 0.12 round it, then round
        the ring in 40 deg chords (a chord never cuts inside the keep), toward the goal's side, until the way is clear.
        The approach to the tower to stack on it is the one way in."""
        tw = self.tower
        if self.mode != "tower" or tw is None or self.state in ("P_APPROACH", "P_CREEP"):
            return goal
        R = self.p.tower_keep
        px, py, gx, gy, cx, cy = pose[0], pose[1], goal[0], goal[1], tw["x"], tw["y"]
        if math.hypot(gx - cx, gy - cy) < R or not self._seg_near(px, py, gx, gy, cx, cy, R):
            return goal
        ring = R + 0.12
        rr = math.hypot(px - cx, py - cy)
        ar = math.atan2(py - cy, px - cx)
        if rr < ring - 0.03:                              # out first, straight away from it
            return cx + ring * math.cos(ar), cy + ring * math.sin(ar), goal[2]
        side = math.remainder(math.atan2(gy - cy, gx - cx) - ar, 2 * math.pi)
        a = ar + math.copysign(math.radians(40), side)
        return cx + ring * math.cos(a), cy + ring * math.sin(a), goal[2]

    @staticmethod
    def _seg_near(px, py, gx, gy, cx, cy, R) -> bool:
        dx, dy = gx - px, gy - py
        L2 = dx * dx + dy * dy
        k = 0.0 if L2 < 1e-9 else max(0.0, min(1.0, ((cx - px) * dx + (cy - py) * dy) / L2))
        return math.hypot(px + k * dx - cx, py + k * dy - cy) < R

    def _drive(self, pose, goal, out) -> bool:
        """A holonomic P step toward goal (odom); True when there."""
        real = goal
        goal = self._tower_detour(pose, goal)
        if goal is not real:                              # round the tower first: never "there" on the way
            bx, by = self._base_from_odom(pose, goal[0], goal[1])
            d = math.hypot(bx, by)
            v = min(self.p.v_max, 1.2 * d + 0.05)
            out.twist = (bx / d * v, by / d * v, 0.0) if d > 1e-4 else (0.0, 0.0, 0.0)
            return False
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
        """The close look from the sweet spot (or look_back short of it): is the object where the far look said?"""
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
        lx, ly = self._look_spot()
        if (moved > 0.02 or math.hypot(bx - lx, by - ly) > 0.04) and self.close_looks < 2:
            self.close_looks += 1
            self._approach_goal(inp.pose)
            self._set("APPROACH", t, f"re-centre on object {o.id} ({100 * moved:.1f} cm off)")
            return
        if self.p.look_back > 0:                          # seen: slide in to the sweet spot, the heading kept
            sx, sy = self.p.sweet
            c, s = math.cos(inp.pose[2]), math.sin(inp.pose[2])
            self.pose_goal = (d.x - (c * sx - s * sy), d.y - (s * sx + c * sy), inp.pose[2])
            self.checked = d
            self._set("CREEP", t, f"in {100 * self.p.look_back:.0f} cm to object {o.id}")
            return
        self._plan_grasp(t, inp, q, d)

    def _s_creep(self, t, inp, out, q):
        if self._drive(inp.pose, self.pose_goal, out):
            self._plan_grasp(t, inp, q, self.checked)

    def _grasp_for(self, pose, d: fv.Detection, q_seed):
        """(tool point at the hover, at the grasp, q at the hover, gripper opening) for detection d (odom)."""
        bx, by = self._base_from_odom(pose, d.x, d.y)
        yaw_b = math.remainder(d.yaw - pose[2], math.pi)
        close_dir = yaw_b + math.pi / 2                       # close across the short side
        dirs = (close_dir, close_dir + math.pi)
        if self.mode == "tower" and self.is_block(d):
            # a tower's block is closed across its faces, whichever pair turns the wrist least: held askew it
            # leans on the fixed finger and topples off the tower when let go (the sim, Nov 7)
            dirs = tuple(yaw_b + k * math.pi / 2 for k in range(4))
        elif d.length - d.width < 0.006:                      # round or square: whichever turns the wrist least
            close_dir = math.atan2(by - self.kin.mount[1], bx - self.kin.mount[0]) + math.pi / 2
            dirs = (close_dir, close_dir + math.pi)
        best = None
        for yaw in dirs:
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
            turn = abs(math.remainder(r.yaw - guess.yaw, math.pi / 2))
            squaring = self.mode == "tower" and self.is_block(d) and turn > math.radians(4)
            if (shift > self.p.refine_tol or squaring) and self.refines < self.p.refine_moves:
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
        self.stages["attempted"] = True
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
        self.stages["grasped"] = True
        gap = float(gap_at(self.kin.cfg, q[5]))
        o = self.target
        # zc, yc, skew: where it sits in the fingers (tool frame), until the wrist camera says otherwise: pressed
        # against the fixed finger, centred across the pads, square. Its centre half its own width from that finger:
        # the gap reads wider than what it holds (a 40 mm cube read 45.7 mm in the sim), half of it put a tower's
        # block 3 mm off; a block's side is its height (the depth reads that well)
        width = min(gap, self.grasp["det"].width, o.det.top if self.mode == "tower" else math.inf)
        self.held = dict(id=o.id, width=round(gap, 4), height=round(o.det.top, 4), yaw=self.grasp["yaw"],
                         tip=float(self.grasp["tool"][2] - self.kin.cfg.pads["fingertip_below_tool"]),
                         zc=width / 2, yc=0.0, skew=0.0, seen=False)
        if self.keep and self.p.held_look:
            self.t_req = t + 0.25
            self._set("HELD_LOOK", t)
            return
        self._after_lift(t, inp, q)

    def _s_held_look(self, t, inp, out, q):
        """The block in the fingers through the wrist camera (the arm still, lifted clear of the floor)."""
        out.want_wrist = t >= self.t_req
        f = inp.wrist
        if f is None or f.t < self.t_req:
            if t - self.t_req > 2.0:                      # no wrist frame: carry on as if square
                self.log("HELD_LOOK: no wrist frame")
                self._after_lift(t, inp, q)
            return
        self.t_req = math.inf
        h, g = self.held, self.grasp
        side = min(g["det"].width, h["height"])
        top_x = -(h["height"] - float(g["tool"][2]))      # the top face's height over the tool point, along the fingers
        r = held_block(f, self.kin.tool(q), side, h["width"], top_x)
        if r is None:
            self.log("HELD_LOOK: the wrist camera didn't find the block's edge: placing it as if centred")
        else:
            # its bottom along the fingers from the tool point: where it goes down to (it can ride up as it's gripped)
            h.update(zc=r.zc, yc=r.yc, skew=r.skew, bottom_x=r.top_x + h["height"], seen=True)
            self.log(f"HELD_LOOK: {1000 * r.yc:+.0f} mm across the pads, {math.degrees(r.skew):+.0f} deg in the fingers, "
                     f"{1000 * (top_x - r.top_x):+.0f} mm up")
        self._after_lift(t, inp, q)

    def _after_lift(self, t, inp, q):
        """Lifted and held: a tower's block gripped askew goes down again to be gripped square, a kept one is carried,
        anything else goes to the bin."""
        gap = self.held["width"]
        # a cube's side: its height (the depth reads it well; a footprint seen askew reads wide)
        side = min(self.grasp["det"].width, self.target.det.top)
        # askew: turned in the fingers, as the wrist camera saw it; unseen, the jaws closed wider than the block (a
        # rougher sign: the gap reads a few mm wide even square, 5.7 mm for a 40 mm cube in the sim)
        askew = (abs(self.held["skew"]) > self.p.regrip_skew if self.held["seen"] else
                 gap > side + self.p.regrip_gap)
        if self.mode == "tower" and askew and self.regrips.get(self.target.id, 0) < 1:
            # held askew (the jaws wider than the block): straight back down where it was, then picked again
            o = self.target
            self.regrips[o.id] = self.regrips.get(o.id, 0) + 1
            pose = inp.pose
            c, s_ = math.cos(pose[2]), math.sin(pose[2])
            gy = self.grasp["yaw"]                        # where it was: the tool's spot, the object beside it
            bx = self.grasp["tool"][0] + math.cos(gy) * gap / 2
            by = self.grasp["tool"][1] + math.sin(gy) * gap / 2
            self.place = dict(x=pose[0] + c * bx - s_ * by, y=pose[1] + s_ * bx + c * by, z=0.0, yaw=None,
                              tower=False, ahead=False, regrip=True)
            why = (f"{1000 * gap:.0f} mm for {1000 * side:.0f}" if gap > side + self.p.regrip_gap else
                   f"turned {math.degrees(self.held['skew']):.0f} deg in the fingers")
            self.held.update(zc=gap / 2, yc=0.0, skew=0.0)  # back where it was: the grip as it closed
            self.held.pop("bottom_x", None)
            self._set("P_HOVER", t, f"object {o.id} held askew ({why}): down again to grip it square")
            return
        if self.keep:                                    # Fetch, the tower: carry it instead of the bin
            o = self.target
            o.status = "picked"
            # building, the block is carried up high: at "ready" it hung in the D435's view and hid the tower's top
            # from the look before stacking (it read 43 mm for an 80 mm tower in the sim, and the next block went
            # down into it)
            carry = self.poses["high" if self.mode == "tower" else "ready"].copy()
            carry[5] = self.q_closed
            if not self._go(t, q, carry, gripper=self.q_closed):
                self._halt(t, q, f"no checked way to carry object {o.id} ({self.no_path})")
                return
            self._set("CARRY", t, f"carrying object {o.id}")
            return
        self.held = None                                 # a tidy's: straight to the bin
        drop = self.poses["drop"].copy()
        drop[5] = self.q_closed
        if not self._go(t, q, drop, gripper=self.q_closed, allow_bin=True):
            self._halt(t, q, f"no checked way to the bin with object {self.target.id} ({self.no_path})")
            return
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
        if o is not None:
            o.status = "picked"
        self.picked += 1
        self.stages["released"] = True
        oid = o.id if o is not None else (self.held or {}).get("id", "?")
        self.held = None
        self._set("STOW", t, f"object {oid} in the bin ({self.picked} so far)")
        self.target = None
        self.motion = None

    # -- carrying and putting down (Fetch, Block tower) ---------------------------------------------------------------
    def is_block(self, d: fv.Detection) -> bool:
        lo, hi = self.p.block_size
        return lo <= d.top <= hi and lo <= d.width <= hi and d.length <= self.p.block_long and not d.big

    def _to_bin(self, t: float, q):
        drop = self.poses["drop"].copy()
        drop[5] = self.q_closed
        if not self._go(t, q, drop, gripper=self.q_closed, allow_bin=True):
            self._halt(t, q, f"no checked way to the bin ({self.no_path})")
            return
        self._set("TO_BIN", t, "what it holds goes in the bin")

    def _s_carry(self, t, inp, out, q):
        if not self._arrived(t, q, 0.05):
            return
        self.motion = None
        if self.mode == "tower" and self.tower is not None:
            tw = self.tower
            self.place = dict(x=tw["x"], y=tw["y"], z=tw["top"], yaw=tw["yaw"], tower=True, ahead=False)
            self._begin_place(t, inp.pose)
            return
        self.outcome = self.done_reason = "held"
        self._set("DONE", t, f"holding object {self.held['id']}")

    def _begin_place(self, t, pose):
        pl = self.place
        if pl["ahead"]:                                   # where it stands: no driving
            self.pose_goal = None
            self._set("P_HOVER", t, "putting it down ahead")
            self.fresh = True
            return
        heading = math.atan2(pl["y"] - pose[1], pl["x"] - pose[0])
        if math.hypot(pl["x"] - pose[0], pl["y"] - pose[1]) < 0.45 + self.p.look_back:
            heading = pose[2]
        sx, sy = (self._look_spot() if pl["tower"] else self.p.sweet)
        c, s_ = math.cos(heading), math.sin(heading)
        self.pose_goal = (pl["x"] - (c * sx - s_ * sy), pl["y"] - (s_ * sx + c * sy), heading)
        self._set("P_APPROACH", t, f"to put object {self.held['id']} down at ({pl['x']:.2f}, {pl['y']:.2f})"
                                    + (f" on the tower ({self.tower['n']} high)" if pl["tower"] else ""))

    def _s_p_approach(self, t, inp, out, q):
        if not self._drive(inp.pose, self.pose_goal, out):
            return
        if self.place["tower"]:
            self.t_req = t + self.p.settle
            self._set("P_CHECK", t)
            return
        self._set("P_HOVER", t)

    def _s_p_check(self, t, inp, out, q):
        """The close look at the tower: where it is and how high (a depth frame from after the base settled)."""
        out.want_depth = t >= self.t_req
        f = inp.depth
        if f is None or f.t < self.t_req:
            return
        self.t_req = math.inf
        pose = self._pose_at(f.t)
        dets = fv.to_frame(fv.detect(f, rng=np.random.default_rng(11)), pose)
        tw, pl = self.tower, self.place
        near = [d for d in dets if math.hypot(d.x - tw["x"], d.y - tw["y"]) < self.p.tower_find]
        if not near:                                      # knocked over, or out of sight: a new base where it was
            self.log(f"P_CHECK: the tower isn't there: object {self.held['id']} starts a new one")
            pl["z"], pl["yaw"] = 0.0, None
            tw.update(n=0, top=0.0)
        else:
            d = min(near, key=lambda d: math.hypot(d.x - tw["x"], d.y - tw["y"]))
            if d.top < tw["top"] - 0.02:                  # lower than it was built: fallen, or the view is blocked
                self.log(f"P_CHECK: the tower reads {1000 * d.top:.0f} mm, built to {1000 * tw['top']:.0f}: "
                         "going by what is seen")
            # the top seen (the highest points) rather than the count: a block not quite flat says so
            tw.update(x=d.x, y=d.y, top=d.top, yaw=d.yaw)
            pl.update(x=d.x, y=d.y, z=tw["top"], yaw=d.yaw)
        if self.p.look_back > 0:                          # seen: slide in to the sweet spot, the heading kept
            sx, sy = self.p.sweet
            c, s_ = math.cos(inp.pose[2]), math.sin(inp.pose[2])
            self.pose_goal = (pl["x"] - (c * sx - s_ * sy), pl["y"] - (s_ * sx + c * sy), inp.pose[2])
            self._set("P_CREEP", t, f"in {100 * self.p.look_back:.0f} cm to the tower (top {1000 * tw['top']:.0f} mm)")
            return
        self._set("P_HOVER", t)

    def _s_p_creep(self, t, inp, out, q):
        if self._drive(inp.pose, self.pose_goal, out):
            self._set("P_HOVER", t)

    def _place_tool(self, pose, z_bottom: float, yaw: float):
        """The tool point that puts the held object's bottom at z_bottom over the place (odom), the fingers turned to
        yaw (base frame). The tool point is the fixed finger's face (_grasp_for brings it side_clear from the object's
        side); closing, the moving jaw pushes the object against it: its centre is half its width from the tool point,
        on the closing line (zc), and yc across the pads (HELD_LOOK; 0 unseen). Fingers down, the tool's z axis is the
        closing line (yaw) and its y axis a quarter turn left of it."""
        pl, h = self.place, self.held
        bx, by = self._base_from_odom(pose, pl["x"], pl["y"])
        o = np.array([math.cos(yaw), math.sin(yaw), 0.0])
        across = np.array([-math.sin(yaw), math.cos(yaw), 0.0])
        tool = np.array([bx, by, 0.0]) - o * h.get("zc", h["width"] / 2) - across * h.get("yc", 0.0)
        # the fingertips were h["tip"] over the floor at the grasp, the object's bottom on it (or where HELD_LOOK saw it)
        tool[2] = z_bottom + h.get("bottom_x", h["tip"] + self.kin.cfg.pads["fingertip_below_tool"])
        return tool

    def _s_p_hover(self, t, inp, out, q):
        if self.fresh:
            self.fresh = False
            pl, h = self.place, self.held
            pose = inp.pose
            # line the held block up with the one below (a square block: any quarter turn), the least wrist roll; its
            # faces are turned skew from the grip's (HELD_LOOK), so the grip turns that much less
            base_yaw = h["yaw"] if pl["yaw"] is None else math.remainder(pl["yaw"] - pose[2] - h.get("skew", 0.0),
                                                                        2 * math.pi)
            best = None
            for k in range(4):
                yaw = math.remainder(base_yaw + k * math.pi / 2 if pl["yaw"] is not None else h["yaw"] + k * math.pi,
                                     2 * math.pi)                # the fingers' yaw is kept unwrapped (it was 6.2 rad, Nov 9)
                if pl["yaw"] is None and k > 1:
                    break
                tool = self._place_tool(pose, pl["z"] + self.p.place_dz, yaw)
                qh = self.kin.ik(tool, DOWN, yaw=yaw, seed=q, flip_ok=False)
                if qh is None or not self.kin.clear(qh)[0]:
                    continue
                if best is None or abs(qh[4]) < abs(best[1][4]):
                    best = (yaw, qh)
            if best is None:
                tool = self._place_tool(pose, pl["z"] + self.p.place_dz, base_yaw)
                self.events.append((round(t, 2), "P_HOVER", f"no reach: tool {np.round(tool, 3).tolist()}, grip yaw "
                                    f"{math.degrees(base_yaw):.0f} deg, held "
                                    f"{ {k: round(float(h[k]), 3) for k in ('yaw', 'width', 'zc', 'yc', 'tip', 'bottom_x') if k in h} }, "
                                    f"arm {np.round(q, 2).tolist()}"))
                self._place_failed(t, q, "no reach over the spot" + (" (the tower is too high)" if pl["tower"] else ""))
                return
            yaw, qh = best
            qh[5] = self.q_closed
            if not self._go(t, q, qh, gripper=self.q_closed):
                self._place_failed(t, q, f"no checked way over the spot ({self.no_path})")
                return
            self.place["grip_yaw"] = yaw
            return
        if self._arrived(t, q):
            pl = self.place
            tool = self._place_tool(inp.pose, pl["z"] + self.p.place_gap, pl["grip_yaw"])
            wps = mo.line(self.kin, self.goal, tool, DOWN, pl["grip_yaw"])
            if wps is None:
                self._place_failed(t, q, "no straight way down")
                return
            self._move(t, self.goal, wps, gripper=self.q_closed)
            self._set("P_DESCEND", t)

    def _s_p_descend(self, t, inp, out, q):
        if self._arrived(t, q, 0.04):
            g = self.goal.copy()
            g[5] = q_for_gap(self.kin.cfg, min(gap_at(self.kin.cfg, 1.5), self.held["width"] + self.p.open_margin))
            self._move(t, self.goal, [g])
            self.motion.t[-1] = t + 0.6
            self._set("P_OPEN", t)

    def _s_p_open(self, t, inp, out, q):
        """Opened: only the moving jaw let go; the object still leans on the fixed finger (the tool point), and straight
        up dragged a block off the tower in the sim. So first slide the fixed finger off its side (away from the
        object, release_slide), then up."""
        if t - self.t_state < 0.9:
            return
        yaw = self.place["grip_yaw"]
        p0 = self.kin.tool(self.goal)[:3, 3]
        side = p0 - self.p.release_slide * np.array([math.cos(yaw), math.sin(yaw), 0.0])
        wps = mo.line(self.kin, self.goal, side, DOWN, yaw, step=0.004)
        up = None
        if wps:
            up = mo.line(self.kin, wps[-1], side + np.array([0.0, 0.0, self.p.rise]), DOWN, yaw, step=0.01)
        if wps and up:
            wps = wps + up
        else:
            wps = mo.line(self.kin, self.goal, p0 + np.array([0.0, 0.0, self.p.rise]), DOWN, yaw, step=0.01)
        self._move(t, self.goal, wps if wps else [self.poses["ready"]])
        self._set("P_RISE", t)

    def _s_p_rise(self, t, inp, out, q):
        if not self._arrived(t, q, 0.05):
            return
        held, pl = self.held, self.place
        self.held, self.place = None, None
        if pl.get("regrip"):                              # put back to grip it square: a fresh look, then again
            o = self.target
            if o is not None:
                o.status = "new"
            self.target = None
            self._set("STOW", t, f"object {held['id']} down again: a fresh look, then a square grip")
            return
        self.stages["released"] = True
        self.picked += 1
        if self.mode == "tower" and self.tower is not None:
            tw = self.tower
            tw["n"] += 1
            tw["top"] = pl["z"] + held["height"]
            if tw["n"] == 1:                              # it started a new tower: it is the base now
                tw.update(x=pl["x"], y=pl["y"])
            self._set("STOW", t, f"tower: {tw['n']} high (top {1000 * tw['top']:.0f} mm)")
            return
        self.outcome = "placed"
        self.done_reason = "placed"
        self.stopping = True                              # STOW ends the task
        self._set("STOW", t, f"object {held['id']} put down")

    def _place_failed(self, t, q, why: str):
        """It can't put it down there: into the bin instead (it never lets go somewhere it hasn't checked)."""
        self.outcome = self.done_reason = f"place failed: {why}"
        if self.mode == "tower":
            self.mode = "keep"                            # the tower is over
        self.stopping = True
        self.place = None
        self.log(f"P: {why}: into the bin")
        self._to_bin(t, q)
