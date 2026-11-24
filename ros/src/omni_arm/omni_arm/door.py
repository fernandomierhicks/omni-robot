"""Opening a hinged door by its pull: the arm holds the pull, the base pulls (design.md 4.5; Nov 21, proposal). No ROS:
simulation/v11 sil/studies/door_open.py runs it in the sim; an arm_node primitive later.

The SO-101 is weak at reach (2.9 N m stall), so it is used as a compliant hand, not as the puller: it grips the pull
and holds one pose, and the omni base drives the gripper round the hinge, turning with the door so the arm's pose
stays the same all the way. The arm's give (its goal against where it really is) and the driver's force estimate
(design.md 4.5: the load model's tool force) steer and limit the base.

DoorTask.step(t, DoorInputs) -> DoorOutputs every arm cycle (50 Hz):
  in   the base's pose (odom, the MCU's odometry), the arm's measured and goal angles (the C++ driver's), its tool
       force estimate (base_link, the force the world puts on the tool), whether the reflex fired, whether a motion
       is playing
  out  the base's body twist (None = stop: the mode manager ramps it down), arm motions to send (as arm_brain sends
       them: MOVE with times and waypoints), a status

The door (Door, odom frame: the hinge's axis, the closed door's heading, which way it opens, the pull's place) is
given, or found by its AprilTag (Nov 21): a DoorTag is the door map's entry for it (the tag's id and size, the hinge's
and the pull's place from the tag, measured once with a ruler) and the D435 finds it. The D435 on the arm riser
(0.19 m up, 30 deg down) sees nothing on a door above ~0.18 m, so the tag (48 mm) sits at the door's bottom edge
under the pull, and is seen with the robot ~0.9 m from the door at most (there at the image's top edge; a forward
camera, design.md 4.12, sees a bigger tag beside the pull from 2 m). At 20 px a tag's place is good to ~1 cm but its heading only to
3-5 deg, so the heading (and the face's distance) come from the depth of the strip of the door's face the D435 sees
beside it (face_plane), the tag's pose fitted upright (the door is vertical: 5 unknowns with its roll on the door, no mirror ambiguity).
Stages, finding:
  FIND       still a moment (the tag may be in view), then the base turns on the spot until the tag is seen: once
             round at most, else NOT_FOUND (also when the looks lose the tag a third time)
  LOOK       the base still: look_n sightings averaged (the tag's centre, the face's heading) make the door; done
             far (where FIND stopped), and again from the standoff (close_look: ~0.3 m off) before the reach
Stages, opening:
  APPROACH   the base to standoff short of the stance (facing the door, the pull straight ahead), so the arm's
             sweep out stays clear of the door when the door's estimate is off
  REACH      the arm, open wide, level, to pre_back short of the grasp (planned)
  CREEP      the base straight in to the stance: the bar comes in between the open jaws, centred
  PROBE      straight in, slowly and guarded, past where the door's face should be, until the fingertips touch it:
             the reflex (the driver's calibrated load model; design.md 4.5), or the arm's give along the fingers
             once the line has played. Where the fingertips stopped is the door's face: the door's estimate moves
             along its normal to it (the grasp's depth, which the jaws can't measure). No touch: MISSED
  SEAT       back to the depth that puts the bar at the pads' middle
  CLOSE      the gripper closes on the pull; held if it stopped clear of its closed stop and no wider than the bar.
             The jaws pull the compliant arm across until the fixed pad is on the bar: where it really is says where
             the bar is across the fingers, and the door's estimate moves along the door to it (touch localisation)
  PULL       the door's angle reference runs ahead (ramped, at most lead past the angle the gripper says the door is
             at, slowed as the pull force rises from f_soft to f_max, and at f_max back to where the door is so the
             pull slackens; the force from the friction band's middle, see pull_force); the base tracks the stance
             (turning with the door), and moves along the radius to take up the arm's radial give (the hinge's
             estimate is off). Stops at open_to; at f_max held for stall_s, or no progress for stall_s: STALLED;
             the jaws closing lost_gap past where they gripped (the bar slid out), or the arm's give over
             give_max: LOST (the gripper's place alone would say the door kept opening)
  RELEASE    open; the base slides release_side across (the fixed pad off the bar: backing out along the fingers
             would drag it), backs off release_back (the arm, folding, would sweep the open door), the arm to stow
  (pulling, the tag and the face give the door's angle too: th_tag, logged against the gripper's, not used to steer)
Outcome: opened | stalled | lost | blocked | unreachable | missed | not_found.

Nothing about the door's forces is known or commanded: the base drives the grip along the arc at a set rate; the
pull force is only read (the arm's servos and the drive currents) and limits the rate. The arc: the hinge's place
and the pull's radius from the door (the tag + its map entry), corrected as it goes by the arm's radial give.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from . import motion as mo
from .kinematics import ArmKinematics, gap_at, q_for_gap

MOVE, HOLD = 0, 1                          # omni_interfaces/ArmMotion
PAD_BACK = 0.010                           # m: the fixed pad's middle this far back up the finger from the tool point


@dataclass
class Door:
    """A hinged door as the robot knows it, odom frame."""
    hinge: tuple                           # (x, y): the hinge's axis
    closed_heading: float                  # rad: from the hinge toward the free edge, the door shut
    sense: int                             # +1 it opens counter-clockwise seen from above, -1 clockwise
    handle_r: float                        # m: the pull's bar from the hinge's axis, along the door
    handle_out: float                      # m: ... and out from it, into the room (door thickness + the pull's standoff)
    handle_z: float                        # m: the grip's height over the floor (the bar is vertical)
    bar_d: float = 0.012                   # m: the bar's diameter
    open_max: float = math.radians(110)    # rad: the hinge's stop
    bar_off: float = 0.032                 # m: the bar's axis off the door's face (the pull's standoff)

    def u(self, th: float) -> np.ndarray:
        a = self.closed_heading + self.sense * th
        return np.array([math.cos(a), math.sin(a)])

    def n(self, th: float) -> np.ndarray:
        """The door's outward normal (into the room) at angle th."""
        a = self.closed_heading + self.sense * (th + math.pi / 2)
        return np.array([math.cos(a), math.sin(a)])

    def bar(self, th: float) -> np.ndarray:
        return np.asarray(self.hinge, float) + self.handle_r * self.u(th) + self.handle_out * self.n(th)

    def angle_of(self, p) -> float:
        """The door's angle for the bar at p (xy)."""
        h = np.asarray(self.hinge, float)
        a = math.atan2(p[1] - h[1], p[0] - h[0])
        b0 = self.bar(0.0) - h
        return self.sense * math.remainder(a - math.atan2(b0[1], b0[0]), 2 * math.pi)

    def shifted(self, d) -> "Door":
        return Door((self.hinge[0] + d[0], self.hinge[1] + d[1]), self.closed_heading, self.sense, self.handle_r,
                    self.handle_out, self.handle_z, self.bar_d, self.open_max, self.bar_off)

    def angle_of_normal(self, heading: float) -> float:
        """The door's angle for its face's outward normal at heading (rad, odom)."""
        return self.sense * math.remainder(heading - self.closed_heading - self.sense * math.pi / 2, 2 * math.pi)


@dataclass
class DoorTag:
    """A door's AprilTag and where the door is from it: the robot's door map, measured once with a ruler. The tag's
    frame: x right seen from the room, y up, z out of the door's face; its origin the black square's centre."""
    tag_id: int
    size: float                            # m: the black square
    hinge_x: float                         # m: the hinge's axis along the tag's x (< 0: the hinge on the left, seen
                                           #   from the room; the door opens clockwise seen from above)
    pull_dy: float                         # m: the grip's height over the tag's centre
    pull_x: float = 0.0                    # m: the pull's bar along the tag's x
    door_t: float = 0.019                  # m: the door's thickness (the hinge's axis behind the face, as Door has it)
    bar_off: float = 0.032                 # m: the bar's axis out from the face
    bar_d: float = 0.012
    open_max: float = math.radians(110)


def door_from_tag(rec: DoorTag, xy, z: float, heading: float) -> Door:
    """The door (odom) from its tag's centre (xy, z) and the heading of its face's normal (rad), the tag upright."""
    zt = np.array([math.cos(heading), math.sin(heading)])
    xt = np.array([-zt[1], zt[0]])                       # y up: x = y x z
    hinge = np.asarray(xy, float) + rec.hinge_x * xt - rec.door_t * zt
    sense = 1 if rec.hinge_x > 0 else -1
    u0 = -sense * xt                                     # from the hinge toward the free edge
    return Door((float(hinge[0]), float(hinge[1])), math.atan2(u0[1], u0[0]), sense, abs(rec.pull_x - rec.hinge_x),
                rec.door_t + rec.bar_off, z + rec.pull_dy, rec.bar_d, rec.open_max, rec.bar_off)


@dataclass
class DoorCamera:
    """The camera the tag is seen with: a pinhole on undistorted pixels and its optical frame in base_link (x right,
    y down, z along the view)."""
    fx: float
    fy: float
    cx: float
    cy: float
    T_base_cam: np.ndarray

    @classmethod
    def from_config(cls, c: dict, width: int | None = None, height: int | None = None) -> "DoorCamera":
        """From an arm config's camera (kin.cfg.cameras[0]: MuJoCo's camera axes, x right, y up, looking along -z)
        at its stream size or another (the same field)."""
        w, h = width or c["width"], height or c["height"]
        f = (h / 2) / math.tan(math.radians(c["fovy_deg"]) / 2)
        T = np.eye(4)
        T[:3, :3] = np.asarray(c["R"], float) @ np.diag([1.0, -1.0, -1.0])
        T[:3, 3] = c["pos"]
        return cls(f, f, (w - 1) / 2, (h - 1) / 2, T)


def _tag_corners(size: float) -> np.ndarray:
    """The black square's corners in the tag's frame, counter-clockwise from its bottom left (apriltag's order)."""
    s = size / 2
    return np.array([(-s, -s, 0.0), (s, -s, 0.0), (s, s, 0.0), (-s, s, 0.0)])


def upright_tag_pose(cam: DoorCamera, corners, size: float):
    """An upright tag's pose in base_link from its corners: (centre (3), its normal's heading (rad), rms px).

    The door is vertical, so the tag's normal is level (the floor level, the robot still): its centre, its normal's
    heading and its roll about the normal (a sticker is never put on quite level: at 1 deg a tag 176 px wide misses by
    1.5 px if the roll is not fitted, Nov 21), five unknowns, not six. A small tag far off is ambiguous in six (its
    mirror pose, tipped the other way about the view ray, fits nearly as well: omni_slam.tags) but not in five."""
    from scipy.optimize import least_squares
    uv = np.asarray(corners, float).reshape(4, 2)
    obj0 = _tag_corners(size)
    Rbc, pbc = cam.T_base_cam[:3, :3], cam.T_base_cam[:3, 3]

    def project(x):
        c, s = math.cos(x[3]), math.sin(x[3])
        R = np.array([[-s, 0.0, c], [c, 0.0, s], [0.0, 1.0, 0.0]])     # columns: the tag's x, y (up), z (out)
        cr, sr = math.cos(x[4]), math.sin(x[4])
        obj = obj0 @ np.array([[cr, -sr, 0.0], [sr, cr, 0.0], [0.0, 0.0, 1.0]]).T   # rolled in its plane
        P = (obj @ R.T + x[:3] - pbc) @ Rbc                          # in the camera
        return np.column_stack([cam.fx * P[:, 0] / P[:, 2] + cam.cx, cam.fy * P[:, 1] / P[:, 2] + cam.cy])

    # the start: the centre's ray at the range the tag's size gives, facing the camera
    m = uv.mean(axis=0)
    side = float(np.mean([np.linalg.norm(uv[k] - uv[(k + 1) % 4]) for k in range(4)]))
    ray = Rbc @ np.array([(m[0] - cam.cx) / cam.fx, (m[1] - cam.cy) / cam.fy, 1.0])
    p0 = pbc + ray / np.linalg.norm(ray) * cam.fy * size / max(side, 1.0)
    to_cam = pbc - p0
    x0 = np.r_[p0, math.atan2(to_cam[1], to_cam[0]), 0.0]
    sol = least_squares(lambda x: (project(x) - uv).ravel(), x0, method="lm")
    rms = float(np.sqrt(np.mean(sol.fun ** 2) * 2))
    return sol.x[:3].copy(), float(math.remainder(sol.x[3], 2 * math.pi)), rms


def face_plane(cam: DoorCamera, depth, c, heading: float, half_w: float = 0.30, z_lo: float = 0.11,
               z_hi: float = 0.20, band: float = 0.04, min_pts: int = 200):
    """The door's face from the depth camera (its image in cam's pixels, m along the view, NaN none): the vertical
    plane through the depth points beside the tag (within half_w along the face, z_lo..z_hi high, band off the tag's
    plane; a full-overlay door's neighbours share the face), fitted in plan (total least squares, twice, the band
    narrowed to 3 sigma). Returns (its normal's heading toward the camera, the tag's offset along that normal to the
    face (m, + toward the camera), points, rms m), or None."""
    D = np.asarray(depth, np.float64)
    H, W = D.shape
    v, u = np.mgrid[0:H, 0:W]
    ok = np.isfinite(D) & (D > 0.05)
    z = D[ok]
    P = np.column_stack([(u[ok] - cam.cx) / cam.fx * z, (v[ok] - cam.cy) / cam.fy * z, z])
    P = P @ cam.T_base_cam[:3, :3].T + cam.T_base_cam[:3, 3]
    c = np.asarray(c, float)
    n = np.array([math.cos(heading), math.sin(heading)])
    x = np.array([-n[1], n[0]])
    rel = P[:, :2] - c[:2]
    sel = (P[:, 2] > z_lo) & (P[:, 2] < z_hi) & (np.abs(rel @ x) < half_w)
    off, rms, k = 0.0, band, 0
    for _ in range(3):
        m = sel & (np.abs(rel @ n - off) < band)
        k = int(m.sum())
        if k < min_pts:
            return None
        Q = rel[m]
        mu = Q.mean(axis=0)
        _, _, Vt = np.linalg.svd(Q - mu, full_matrices=False)
        nn = Vt[1] if Vt[1] @ n > 0 else -Vt[1]          # the normal: the least spread, toward the camera
        n, x = nn, np.array([-nn[1], nn[0]])
        off = float(mu @ n)
        rms = float(np.sqrt(np.mean(((Q - mu) @ n) ** 2)))
        band = max(0.006, 3 * rms)
    return float(math.atan2(n[1], n[0])), off, k, rms


@dataclass
class DoorParams:
    grasp_x: float = 0.51                  # m: the tool point this far ahead of the base's centre at the grasp (the
                                           #   level grasp with the jaws' opening horizontal reaches 0.45-0.57)
    pre_back: float = 0.05                 # m: the reach stops this far short of the grasp, then the probe
    standoff: float = 0.07                 # m: the base reaches out this far short of the stance, then creeps in
    v_creep: float = 0.05                  # m/s
    probe_past: float = 0.03               # m: the probe's line ends this far past the expected face
    probe_speed: float = 0.02              # m/s
    tip_gap: float = 0.016                 # m: seated, the fingertips this far off the door's face
    contact_give: float = 0.0012           # m: probing, the tool this far behind its goal along the fingers: touched
                                           #   (moving free it trails 0.2-0.45 mm in the sim)
    probe_comp: float = 0.004              # m: the probe reads the face this much deep (the door and the arm give
                                           #   under the touch: 2-6 mm in the sim, est); the seat aims short by it
    open_gap: float = 0.040                # m: the jaws open to the bar's width plus this, the bar centred
    min_gap: float = 0.002                 # m: held if the jaws stopped this far clear of their closed stop ...
    grip_tol: float = 0.003                # m: ... and no wider than the bar plus this
    lost_gap: float = 0.003                # m: pulling, the jaws this much closer than when they gripped: LOST
    close_s: float = 1.2                   # s: the jaws close this slowly (the base follows the arm across)
    k_follow: float = 4.0                  # 1/s: closing, the base follows the arm's give
    v_follow: float = 0.06                 # m/s
    open_to: float = math.radians(90)      # rad: the door's angle to stop at (kept 5 deg short of the hinge's stop)
    w_max: float = 0.25                    # rad/s: the door's opening rate at most
    ramp_s: float = 1.0                    # s: ... reached after this
    lead: float = math.radians(5)          # rad: the reference never this far past the angle the gripper says
    f_soft: float = 15.0                   # N: the pull slows from here
    f_max: float = 25.0                    # N: ... and stops here: slack, then STALLED unless the door moves on
    wheel_kt: float = 0.48                 # N m/A at the wheel: the AK40-10's torque per amp (bench-inferred, VERIFY)
    base_tau: float = 0.2                  # s: the base's force reading's low-pass
    give_max: float = 0.03                 # m: the arm's give (where the tool is against its goal) past this: LOST
    stall_s: float = 3.0                   # s: no progress, or at f_max, this long: STALLED
    k_pos: float = 1.5                     # 1/s: the base onto the stance
    k_yaw: float = 2.0                     # 1/s
    k_radial: float = 2.0                  # 1/s: the radial give taken up by moving the stance
    v_max: float = 0.25                    # m/s: the base, driving to the stance
    v_pull: float = 0.20                   # m/s: ... pulling
    w_base: float = 0.8                    # rad/s
    tol_xy: float = 0.01                   # m: at the stance
    tol_yaw: float = math.radians(1.5)
    arm_speed: float = 0.3                 # of the servos' no-load speed (mo.speeds)
    guard_reach: bool = True               # the reflex armed on the reach in
    settle_s: float = 0.6                  # s: the gripper closing, opening
    release_side: float = 0.012            # m: letting go, the base slides the fixed pad this far off the bar
    release_back: float = 0.18             # m: ... then backs off this far before the arm folds
    # finding the door by its tag (DoorTask with a DoorTag and a DoorCamera)
    w_search: float = 0.4                  # rad/s: the base turns on the spot looking for the tag
    look_n: int = 5                        # sightings averaged per look, the base still
    look_settle: float = 0.4               # s: still this long before a look counts sightings
    look_timeout: float = 3.0              # s: a look gets fewer: it uses what it has (2 at least), else searches
    close_look: bool = True                # look again from the standoff (the tag ~0.3 m off, not ~0.7)
    min_margin: float = 30.0               # the detector's decision margin at least (tag_slam's gate)
    max_rms: float = 1.5                   # px: the upright fit's reprojection at most
    face_depth: bool = True                # the door's heading and face from the depth beside the tag (face_plane):
                                           #   a 48 mm tag 20 px wide reads its heading 3-5 deg off (sim, est);
                                           #   the face is fitted where the D435 sees it (z_lo..z_hi), wherever
                                           #   on the door the tag is
    face_max_rms: float = 0.006            # m: the face's fit at most, else the tag's own heading


@dataclass
class DoorInputs:
    pose: tuple                            # (x, y, yaw) odom
    q: np.ndarray                          # the arm's measured angles (6), the driver's
    goal: np.ndarray                       # the angles it is holding or playing toward (6)
    tool_force: np.ndarray | None = None   # N, base_link: the world on the tool (the driver's load model)
    reflex: bool = False                   # the driver's reflex fired and is active
    moving: bool = False                   # a motion is playing
    motor: np.ndarray | None = None        # N m: the driver's motor torque per joint (ArmServoState)
    gravity: np.ndarray | None = None      # N m: ... and its gravity model's
    wheel_cur: np.ndarray | None = None    # A: the four drive motors' currents (the MCU's STATE, protocol 9)
    tags: list | None = None               # a new camera frame's tags [(id, corners (4, 2) undistorted px, margin)]
                                           #   (tag_detector's /tags/detections); None: no new frame
    depth: np.ndarray | None = None        # m, the depth camera's newest frame (the D435's depth is in its left
                                           #   infrared imager's frame: the tags' camera too unless a forward one
                                           #   finds them)


@dataclass
class DoorOutputs:
    twist: tuple | None = None             # (vx, vy, wz) body frame
    arm: list = field(default_factory=list)      # [(cmd, times, waypoints, linear, guard)] to send now
    status: dict = field(default_factory=dict)


def _R(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s], [s, c]])


class DoorTask:
    def __init__(self, kin: ArmKinematics, door: Door | None, params: DoorParams | None = None, log=None,
                 tag: DoorTag | None = None, camera: DoorCamera | None = None,
                 depth_camera: DoorCamera | None = None):
        """door: the door given (odom); or None and tag + camera: the task finds it (FIND, LOOK). camera: the one
        the tags are found in; depth_camera: the depth's (default camera: the D435's infrared and depth share it)."""
        self.kin, self.p = kin, params or DoorParams()
        self.log = log or (lambda m: None)
        if door is None and (tag is None or camera is None):
            raise ValueError("DoorTask: a door, or its tag and the camera to find it with")
        self.tag, self.camera = tag, camera
        self.depth_camera = depth_camera or camera
        p, c = self.p, kin.cfg
        self.speed = mo.speeds(kin, p.arm_speed)
        self.q_closed = c.pads["gripper_closed"]
        self.closed_gap = gap_at(c, self.q_closed)
        self.tip = float(c.pads.get("fingertip_below_tool", 0.006))   # the fingertips past the tool point
        self.a = np.array([1.0, 0.0, 0.0])                       # the fingers: straight ahead, level
        self.q_seat = None
        self.grip_gap = None
        self.J_base = np.array(c.base["J"], float)               # wheel rad/s per body (vx, vy, wz)
        self.f_arm = self.f_base = 0.0
        self.q_stow = np.array(c.poses["stow"], float) if "stow" in c.poses else None
        self.via = [c.poses[n] for n in ("high",) if n in c.poses]
        self.door = self.door0 = None
        self.q_pre = self.q_probe = None
        if door is not None:
            self._setup(door)
        # finding the door: the tag's sightings (odom), the looks done
        self.sightings: list = []           # (t, x, y, z, normal heading) of the current look
        self.seen_t = -1e9                  # the last sighting's time
        self.seen_last = None
        self.face = None                    # the last sighting's face_plane fit, or None (the tag's heading)
        self.looks: list = []               # per look: dict(n, xy spread mm, heading spread deg, door)
        self.turned = 0.0
        self.yaw_last = None
        self.n_lost = 0                     # looks that lost the tag (a third: NOT_FOUND)
        self.th_tag = None                  # pulling: the door's angle its tag says (logged; not used to steer)
        self.state = "IDLE"
        self.outcome = None
        self.events: list = []
        self.t0 = self.t_state = 0.0
        self.t_arm = 0.0                    # the motion sent last ends then
        self.th_ref = 0.0
        self.th_est = 0.0
        self.th_best, self.t_best = 0.0, 0.0
        self.t_fmax = None
        self.corr = np.zeros(2)             # the stance moved along the radius (odom, m)
        self.f_pull = 0.0
        self.give = np.zeros(3)
        self.peak = dict(f_pull=0.0, give=0.0)
        self.way_out: list = []             # releasing: the base's poses to drive through (odom)
        self.reach_tries = 0
        self.trace: list = []               # per cycle while pulling: t, th_est, th_ref, f_pull, give (status)
        self.probe_trace: list = []         # per cycle while probing: t, the give along the fingers (mm)

    def _setup(self, door: Door):
        """What the grasp needs from the door (it changes when a look moves the door)."""
        p, c, kin = self.p, self.kin.cfg, self.kin
        self.door = self.door0 = door
        self.q_open = q_for_gap(c, door.bar_d + p.open_gap)
        self.g_open = gap_at(c, self.q_open)
        # the opening direction (fixed jaw -> moving jaw) points along the door toward its free edge, so the fixed
        # finger (the probe's, on base_link's x axis) lands on the door, not past its edge: in base_link at the
        # stance that is (0, sense)
        self.w = np.array([0.0, float(door.sense), 0.0])
        # The arm has five joints: with the fingers level and straight ahead its tool stays on base_link's x axis
        # (y = 0), so everything across the fingers is the base's. The tool point at the grasp, base_link:
        self.G = np.array([p.grasp_x, 0.0, door.handle_z])
        # the bar where the stance puts it before the grasp: centred between the open jaws at the pads' middle
        self.bar_ref = self.G + self.w * self.g_open / 2 - self.a * PAD_BACK
        self.q_pre = self.ik_level(self.G - self.a * p.pre_back, self.w, self.q_open)
        self.q_probe = self.ik_level(self._probe_end(self.G, self.bar_ref), self.w, self.q_open)
        Tt = kin._tool                                            # the tool's frame in the gripper's
        self.bar_gripper = Tt[:3, 3] + Tt[:3, 2] * door.bar_d / 2 - Tt[:3, 0] * PAD_BACK

    @property
    def wants_camera(self) -> bool:
        """The tag is looked for now (the camera's frames are useful): finding, looking, pulling (logged)."""
        return self.tag is not None and self.state in ("FIND", "LOOK", "PULL")

    def _sight(self, t, inp):
        """The door's tag in this frame (odom: centre and normal heading), if seen well."""
        if inp.tags is None or self.tag is None:
            return
        for tid, corners, margin in inp.tags:
            if int(tid) != self.tag.tag_id or (margin is not None and margin < self.p.min_margin):
                continue
            c, hd, rms = upright_tag_pose(self.camera, corners, self.tag.size)
            if rms > self.p.max_rms:
                continue
            self.face = None
            if inp.depth is not None and self.p.face_depth:
                f = face_plane(self.depth_camera, inp.depth, c, hd)
                if f is not None and f[3] <= self.p.face_max_rms:
                    self.face = f
                    hd = f[0]                       # the face's heading; the tag onto the face along its normal
                    n = np.array([math.cos(hd), math.sin(hd), 0.0])
                    c = c + n * f[1]
            x, y, yaw = inp.pose
            xy = np.array([x, y]) + _R(yaw) @ c[:2]
            self.seen_t = t
            self.seen_last = (t, float(xy[0]), float(xy[1]), float(c[2]), float(math.remainder(hd + yaw, 2 * math.pi)),
                              self.face is not None)
            return

    @property
    def wants_depth(self) -> bool:
        """The depth is used now: looking, pulling (the door's face beside the tag)."""
        return self.wants_camera and self.state in ("LOOK", "PULL") and self.p.face_depth

    # -- geometry ---------------------------------------------------------------------------------------------
    def ik_level(self, p, w, grip, seed=None):
        """Joint angles with the tool point at p (base_link), the fingers level along self.a and the opening
        along w (horizontal): the wrist roll found by turning it until the tool's z lies on w. None if out of reach."""
        kin = self.kin
        best = None
        for r0 in (0.0, math.pi / 2, -math.pi / 2):
            s = np.zeros(6) if seed is None else np.asarray(seed, float).copy()
            s[4] = r0
            for _ in range(6):
                q = kin.ik(np.asarray(p, float), self.a, None, seed=s)
                if q is None:
                    break
                z = kin.tool(q)[:3, 2]
                ang = math.atan2(np.cross(z, w) @ self.a, z @ w)
                if abs(ang) < 1e-3:
                    break
                # which way the roll turns z about the fingers
                q2 = q.copy()
                q2[4] += 0.01
                z2 = kin.tool(q2)[:3, 2]
                sgn = 1.0 if (np.cross(z, z2) @ self.a) > 0 else -1.0
                s = q.copy()
                s[4] = float(np.clip(q[4] + sgn * ang, kin.lo[4], kin.hi[4]))
            else:
                q = None
            if q is None:
                continue
            q = q.copy()
            q[5] = grip
            T = kin.tool(q)
            if (np.linalg.norm(T[:3, 3] - p) < 0.003 and T[:3, 2] @ w > 0.995 and kin.clear(q)[0]):
                if best is None or abs(q[4]) < abs(best[4]):
                    best = q
        return best

    def _probe_end(self, tool, bar) -> np.ndarray:
        """The probe's last point (base_link), straight ahead of tool: the fingertips probe_past beyond where the
        door's face should be for the bar at bar."""
        x = bar[0] + self.door.bar_off - self.tip + self.p.probe_past
        return np.array([x, tool[1], tool[2]])

    def _bar_base_now(self, pose, door: Door) -> np.ndarray:
        """The door's bar (shut) in base_link from where the base is."""
        b = _R(-pose[2]) @ (door.bar(0.0) - np.array(pose[:2]))
        return np.array([b[0], b[1], door.handle_z])

    def stance(self, th: float, door: Door | None = None) -> tuple:
        """The base's pose (x, y, yaw, odom) with the bar at bar_ref (base_link) and the door at th."""
        d = door or self.door
        yaw = math.atan2(-d.n(th)[1], -d.n(th)[0])
        xy = d.bar(th) - _R(yaw) @ self.bar_ref[:2]
        return float(xy[0]), float(xy[1]), yaw

    def stance_rate(self, th: float) -> tuple:
        """d stance / d th (x, y, yaw)."""
        e = 1e-4
        a, b = self.stance(th - e), self.stance(th + e)
        return ((b[0] - a[0]) / (2 * e), (b[1] - a[1]) / (2 * e), math.remainder(b[2] - a[2], 2 * math.pi) / (2 * e))

    def pull_force(self, inp: DoorInputs) -> float:
        """The door's pull on the hand along the fingers (N), the larger of what the arm and the base read.

        The arm: held still, the driver's tool force is the outside torque past the gearboxes' friction band (0
        inside it), so it reads a steady pull low or not at all; the band's middle (-(gravity + motor), as the arm
        load calibration scores it) through the Jacobian at the bar is unbiased, but a level pull runs ~0.1 m over
        the lift's axis and the gearbox's friction carries what the motor doesn't: a locked door's 40 N read 13 (sim).
        The base: the drive motors' currents through the wheels' Jacobian (W = J^T tau) see the pull the base makes,
        steady or not (the locked door's 16-30 N read 19-34; ~4 N of rolling resistance and acceleration besides)."""
        f = 0.0
        if inp.motor is not None and inp.gravity is not None:
            tau = -(np.asarray(inp.gravity, float)[:5] + np.asarray(inp.motor, float)[:5])
            J = self.kin.jacobian(inp.q, self.bar_gripper)
            f = float(np.linalg.solve(J @ J.T + 1e-4 * np.eye(3), J @ tau)[0])
        elif inp.tool_force is not None:
            f = float(inp.tool_force[0])
        self.f_arm = f
        if inp.wheel_cur is not None:
            W = self.J_base.T @ (self.p.wheel_kt * np.asarray(inp.wheel_cur, float))
            a = min(1.0, 0.02 / self.p.base_tau)
            self.f_base += a * (-float(W[0]) - self.f_base)      # the wheels hold the base back from the door
            f = max(f, self.f_base)
        return f

    def bar_odom(self, pose, q) -> np.ndarray:
        """Where the bar is (odom xy) for the arm at q: the fixed pad against it."""
        T = self.kin.tool(q)
        pb = T[:3, 3] + T[:3, 2] * self.door.bar_d / 2 - T[:3, 0] * PAD_BACK
        return np.array(pose[:2]) + _R(pose[2]) @ pb[:2]

    # -- the task ----------------------------------------------------------------------------------------------
    def start(self, t: float):
        self.t0 = t
        if self.door is None:
            self._go(t, "FIND")
            return
        if self.q_pre is None or self.q_probe is None:
            self._end(t, "unreachable", f"no level grasp at {self.G.round(3).tolist()}")
            return
        self._go(t, "APPROACH")

    def _s_find(self, t, inp, out):
        # still first (the tag may be in view already), then turn on the spot one way, once round at most
        if self.seen_t > self.t_state:
            out.twist = None
            self._go(t, "LOOK", "the tag seen")
            return
        if t < self.t_state + self.p.look_settle + 0.3:
            return
        if self.yaw_last is not None:
            self.turned += abs(math.remainder(inp.pose[2] - self.yaw_last, 2 * math.pi))
        self.yaw_last = inp.pose[2]
        if self.turned > 2 * math.pi + 0.3:
            self._end(t, "not_found", f"no tag {self.tag.tag_id} once round")
            return
        out.twist = (0.0, 0.0, self.p.w_search)

    def _s_look(self, t, inp, out):
        # the base still; the sightings of look_n frames averaged (centre, the normal's heading)
        out.twist = None
        if t < self.t_state + self.p.look_settle:
            self.sightings = []
            return
        if self.seen_last is not None and self.seen_t > self.t_state + self.p.look_settle and (
                not self.sightings or self.seen_last[0] > self.sightings[-1][0]):
            self.sightings.append(self.seen_last)
        n = len(self.sightings)
        if n < self.p.look_n and t < self.t_state + self.p.look_settle + self.p.look_timeout:
            return
        if n < 2:
            self.n_lost += 1
            if self.n_lost > 2:
                self._end(t, "not_found", f"the looks lost the tag {self.n_lost} times")
                return
            self.yaw_last, self.turned = None, 0.0
            self._go(t, "FIND", f"lost the tag ({n} sightings)")
            return
        S = np.array([s[1:5] for s in self.sightings])
        n_face = sum(bool(s[5]) for s in self.sightings)
        hd = math.atan2(np.sin(S[:, 3]).mean(), np.cos(S[:, 3]).mean())
        xy, z = S[:, :2].mean(axis=0), float(S[:, 2].mean())
        door = door_from_tag(self.tag, xy, z, hd)
        spread_xy = float(np.max(np.linalg.norm(S[:, :2] - xy, axis=1)))
        spread_hd = float(np.max(np.abs(np.remainder(S[:, 3] - hd + math.pi, 2 * math.pi) - math.pi)))
        close = bool(self.looks)
        self.looks.append(dict(n=n, n_face=n_face, close=close, spread_mm=round(spread_xy * 1000, 1),
                               spread_deg=round(math.degrees(spread_hd), 2), tag_xy=xy.round(4).tolist(),
                               tag_z=round(z, 4), heading=round(hd, 5)))
        self._setup(door)
        self.events.append((round(t, 2), "door", f"{'close' if close else 'far'} look, {n} sightings ({n_face} with "
                                                 f"the face's depth), spread "
                                                 f"{spread_xy * 1000:.1f} mm, {math.degrees(spread_hd):.2f} deg"))
        if self.q_pre is None or self.q_probe is None:
            self._end(t, "unreachable", f"no level grasp at {self.G.round(3).tolist()}")
            return
        self._go(t, "APPROACH")

    def _go(self, t, state, note=""):
        self.state, self.t_state = state, t
        self.events.append((round(t, 2), state, note))
        self.log(f"door: {state}{' (' + note + ')' if note else ''}")

    def _end(self, t, outcome, note=""):
        self.outcome = outcome
        self._go(t, "DONE", f"{outcome}{': ' + note if note else ''}")

    def _move(self, t, out, q0, q1, guard=True, planned=True) -> bool:
        if planned:
            path, why = mo.plan_checked(self.kin, q0, q1, via=self.via)
            if path is None:
                self.log(f"door: no path ({why})")
                return False
        else:
            path = [np.asarray(q0, float), np.asarray(q1, float)]
        m = mo.Motion(path, self.speed, t)
        out.arm.append((MOVE, list(m.t), [list(map(float, w)) for w in m.q], False, guard))
        self.t_arm = m.t_end
        return True

    def _line(self, t, out, qs, duration, guard=True):
        ts = list(np.linspace(t, t + duration, len(qs)))
        out.arm.append((MOVE, ts, [list(map(float, w)) for w in qs], True, guard))
        self.t_arm = ts[-1]

    def _arm_done(self, t, inp) -> bool:
        return t >= self.t_arm + 0.1 and not inp.moving

    def _drive(self, pose, goal, out, v_max, tol: float | None = None) -> bool:
        p = self.p
        tol = p.tol_xy if tol is None else tol
        dx, dy = goal[0] - pose[0], goal[1] - pose[1]
        e_yaw = math.remainder(goal[2] - pose[2], 2 * math.pi)
        d = math.hypot(dx, dy)
        if d < tol and abs(e_yaw) < p.tol_yaw:
            out.twist = None
            return True
        v = min(v_max, 1.2 * d + 0.02)
        vw = np.array([dx, dy]) / d * v if d > tol * 0.5 else np.zeros(2)
        vb = _R(-pose[2]) @ vw
        w = max(-p.w_base, min(p.w_base, 2.0 * e_yaw))
        out.twist = (float(vb[0]), float(vb[1]), w)
        return False

    def step(self, t: float, inp: DoorInputs) -> DoorOutputs:
        out = DoorOutputs()
        self._sight(t, inp)
        getattr(self, f"_s_{self.state.lower()}", lambda *a: None)(t, inp, out)
        out.status = dict(state=self.state, outcome=self.outcome, angle_deg=round(math.degrees(self.th_est), 1),
                          ref_deg=round(math.degrees(self.th_ref), 1), pull_n=round(self.f_pull, 2),
                          give_mm=round(float(np.linalg.norm(self.give[:2])) * 1000, 1))
        return out

    def _standoff(self) -> tuple:
        x, y, yaw = self.stance(0.0)
        return x - self.p.standoff * math.cos(yaw), y - self.p.standoff * math.sin(yaw), yaw

    def _s_approach(self, t, inp, out):
        if self._drive(inp.pose, self._standoff(), out, self.p.v_max):
            if self.tag is not None and self.p.close_look and len(self.looks) == 1:
                self._go(t, "LOOK", "again from the standoff")
                return
            q = np.asarray(inp.goal, float).copy()
            pre = self.q_pre.copy()
            if not self._move(t, out, q, pre, guard=self.p.guard_reach):
                self._end(t, "unreachable", "no clear path to the pull")
                return
            self._go(t, "REACH")

    def _s_reach(self, t, inp, out):
        if inp.reflex:
            # the reach ends pre_back short of the door: a reflex here is something in the way, or a false one (a
            # long level reach loads the arm most); again from where it stopped, twice
            self.reach_tries += 1
            if self.reach_tries > 2 or not self._move(t, out, inp.goal, self.q_pre, guard=self.p.guard_reach):
                self._back_out(t, inp, out, "blocked", "the reflex stopped the reach three times")
            else:
                self.events.append((round(t, 2), "reflex", f"reaching out; again ({self.reach_tries})"))
            return
        if not self._arm_done(t, inp):
            return
        self._go(t, "CREEP")

    def _s_creep(self, t, inp, out):
        if inp.reflex:
            self._back_out(t, inp, out, "blocked", "touched something creeping in")
            return
        if not self._drive(inp.pose, self.stance(0.0), out, self.p.v_creep, tol=0.005):
            return
        # the probe: straight in from where the base really is
        bar = self._bar_base_now(inp.pose, self.door)
        q0 = np.asarray(inp.goal, float)
        end = self._probe_end(self.kin.tool(q0)[:3, 3], bar)
        path = mo.line(self.kin, q0, end, self.a, None, step=0.005)
        if not path:
            self._back_out(t, inp, out, "unreachable", "no straight line in")
            return
        self.probe_from = self.kin.tool(q0)[:3, 3]
        dist = float(np.linalg.norm(end - self.probe_from))
        self._line(t, out, [q0] + list(path), dist / self.p.probe_speed, guard=True)
        self._go(t, "PROBE")

    def _tip_odom(self, pose, q) -> np.ndarray:
        T = self.kin.tool(q)
        tip = T[:3, 3] + T[:3, 0] * self.tip
        return np.array(pose[:2]) + _R(pose[2]) @ tip[:2]

    def _s_probe(self, t, inp, out):
        # touched: the tool held back along the fingers (the soft servos' give, ~0.5 mm/N here: it stops at a few N,
        # before the unseen flex of a hard push), or the reflex. A level push hardly loads the wrist and elbow, so the
        # reflex alone fires late
        Tg = self.kin.tool(inp.goal)
        give = float((self.kin.tool(inp.q)[:3, 3] - Tg[:3, 3]) @ Tg[:3, 0])
        self.probe_trace.append((round(t, 3), round(give * 1000, 2)))
        touched = inp.reflex or give < -self.p.contact_give
        if not touched:
            if not self._arm_done(t, inp) or t < self.t_arm + 0.5:
                return
            self._back_out(t, inp, out, "missed", f"no door face within {self.p.probe_past * 1000:.0f} mm past "
                                                  "where it should be")
            return
        d = self.door
        n = d.n(0.0)
        face = d.bar(0.0) - n * d.bar_off
        delta = float((self._tip_odom(inp.pose, inp.q) - face) @ n) + self.p.probe_comp
        self.door = d.shifted(n * delta)
        self.events.append((round(t, 2), "face", f"{'reflex' if inp.reflex else 'give'} {give * 1000:.1f} mm: door "
                                                 f"{delta * 1000:+.1f} mm along its normal"))
        bar = self._bar_base_now(inp.pose, self.door)
        q0 = np.asarray(inp.goal, float)           # (after a reflex, its back-off: the seat keeps the probe's line)
        tool = self.probe_from
        self.q_seat = self.ik_level(np.array([bar[0] + PAD_BACK, tool[1], tool[2]]), self.w, self.q_open,
                                    seed=self.q_pre)
        if self.q_seat is None:
            self._back_out(t, inp, out, "unreachable", "no seat for the bar")
            return
        self._line(t, out, [q0, self.q_seat], 0.8, guard=False)
        self._go(t, "SEAT")

    def _s_seat(self, t, inp, out):
        if not self._arm_done(t, inp):
            return
        q = self.q_seat.copy()
        q[5] = self.q_closed
        self._line(t, out, [self.q_seat, q], self.p.close_s, guard=False)
        self._go(t, "CLOSE")

    def _s_close(self, t, inp, out):
        p = self.p
        # the one moving jaw drags the compliant arm across onto the bar: the base follows the arm's give so the
        # arm never strains (its goal comes to where the jaws put it)
        give = self.kin.tool(inp.q)[:3, 3] - self.kin.tool(inp.goal)[:3, 3]
        v = p.k_follow * give[:2]
        sp = float(np.linalg.norm(v))
        if sp > p.v_follow:
            v *= p.v_follow / sp
        out.twist = (float(v[0]), float(v[1]), 0.0)
        if t < self.t_arm + self.p.settle_s:
            return
        out.twist = None
        gap = gap_at(self.kin.cfg, float(inp.q[5]))
        bar = self.door.bar_d
        if gap < self.closed_gap + p.min_gap or gap > bar + p.grip_tol:
            self._back_out(t, inp, out, "missed", f"the jaws stopped at {gap * 1000:.1f} mm, the bar is {bar * 1000:.0f}")
            return
        # across the fingers (along the door), the fixed pad is on the bar: the door moves along itself to it
        u = self.door.u(0.0)
        lat = float((self.bar_odom(inp.pose, inp.q) - self.door.bar(0.0)) @ u)
        self.door = self.door.shifted(u * lat)
        # the stance from now on puts the bar where the held pose (the arm's goal) has it
        Tg = self.kin.tool(inp.goal)
        self.bar_ref = Tg[:3, 3] + Tg[:3, 2] * bar / 2 - Tg[:3, 0] * PAD_BACK
        self.grip_gap = gap
        self.events.append((round(t, 2), "touch", f"door {lat * 1000:+.1f} mm along itself"))
        self.th_ref = self.th_est = self.th_best = 0.0
        self.t_best = t
        self._go(t, "PULL", f"held, gap {gap * 1000:.1f} mm")

    def _s_pull(self, t, inp, out):
        p, d = self.p, self.door
        q, goal = np.asarray(inp.q, float), np.asarray(inp.goal, float)
        tool_m, tool_g = self.kin.tool(q)[:3, 3], self.kin.tool(goal)[:3, 3]
        self.give = tool_m - tool_g                                    # base_link
        give_odom = _R(inp.pose[2]) @ self.give[:2]
        bar = self.bar_odom(inp.pose, q)
        self.th_est = d.angle_of(bar)
        if self.tag is not None:                                       # what the door's tag says (logged)
            fresh = self.seen_last is not None and t - self.seen_t < 0.15
            self.th_tag = d.angle_of_normal(self.seen_last[4]) if fresh else None
        self.f_pull = max(0.0, self.pull_force(inp))                   # the door pulling the hand toward it
        self.peak["f_pull"] = max(self.peak["f_pull"], self.f_pull)
        g = float(np.linalg.norm(self.give[:2]))
        self.peak["give"] = max(self.peak["give"], g)
        self.trace.append((round(t, 3), round(self.th_est, 4), round(self.th_ref, 4), round(self.f_pull, 2),
                           round(g * 1000, 1)))
        # lost: the jaws closed past where they gripped (the bar slid out), or the arm pulled far off its goal
        if gap_at(self.kin.cfg, float(q[5])) < self.grip_gap - p.lost_gap or g > p.give_max:
            out.twist = None
            self._back_out(t, inp, out, "lost", f"at {math.degrees(self.th_est):.0f} deg, give {g * 1000:.0f} mm")
            return
        if self.th_est >= p.open_to:
            out.twist = None
            q1 = goal.copy()
            q1[5] = self.q_open
            self._line(t, out, [goal, q1], 0.4, guard=False)
            self._go(t, "RELEASE", f"open at {math.degrees(self.th_est):.0f} deg")
            return
        if self.th_est > self.th_best + math.radians(1):
            self.th_best, self.t_best = self.th_est, t
        if t - self.t_best > p.stall_s:
            out.twist = None
            self._release_here(t, goal, out, f"no progress past {math.degrees(self.th_best):.0f} deg")
            return
        # the reference: ramped, slowed by the pull force, never far past where the door is
        ramp = min(1.0, (t - self.t_state) / p.ramp_s)
        fs = float(np.clip((p.f_max - self.f_pull) / (p.f_max - p.f_soft), 0.0, 1.0))
        if fs <= 0.0 and self.t_fmax is None:
            self.t_fmax, self.th_fmax, self.f_hit = t, self.th_est, self.f_pull
            self.events.append((round(t, 2), "limit", f"{self.f_pull:.0f} N at {math.degrees(self.th_est):.0f} deg"))
        if self.t_fmax is not None:
            # at the limit the base stops pulling and stays slack: the stance falls back to where the door is (the
            # arm unloads). The door moving on (a catch let go) resumes; still there after stall_s: let go
            if self.th_est > self.th_fmax + math.radians(3) and fs > 0.0:
                self.t_fmax = None
            elif t - self.t_fmax > p.stall_s:
                out.twist = None
                self._release_here(t, goal, out, f"{self.f_hit:.0f} N at {math.degrees(self.th_fmax):.0f} deg")
                return
            else:
                fs = 0.0
                self.th_ref = min(self.th_ref, self.th_est)
        w = p.w_max * ramp * fs
        self.th_ref = min(self.th_ref + w * 0.02, self.th_est + p.lead, p.open_to + math.radians(2), d.open_max)
        # the stance for the reference, moved along the radius to take up the arm's radial give
        r = bar - np.asarray(d.hinge, float)
        r /= np.linalg.norm(r)
        self.corr += p.k_radial * float(give_odom @ r) * r * 0.02
        sx, sy, syaw = self.stance(self.th_ref)
        rx, ry, ryaw = self.stance_rate(self.th_ref)
        ex, ey = sx + self.corr[0] - inp.pose[0], sy + self.corr[1] - inp.pose[1]
        eyaw = math.remainder(syaw - inp.pose[2], 2 * math.pi)
        vw = np.array([rx, ry]) * w + p.k_pos * np.array([ex, ey])
        sp = float(np.linalg.norm(vw))
        if sp > p.v_pull:
            vw *= p.v_pull / sp
        wz = float(np.clip(ryaw * w + p.k_yaw * eyaw, -p.w_base, p.w_base))
        vb = _R(-inp.pose[2]) @ vw
        out.twist = (float(vb[0]), float(vb[1]), wz)

    def _release_here(self, t, goal, out, note):
        q1 = np.asarray(goal, float).copy()
        q1[5] = self.q_open
        self._line(t, out, [goal, q1], 0.4, guard=False)
        self.stalled = note
        self._go(t, "RELEASE", f"stalled: {note}")

    def _back_out(self, t, inp, out, outcome, note):
        self.outcome = outcome
        q0 = np.asarray(inp.q, float).copy()
        q1 = q0.copy()
        q1[5] = self.q_open
        self._line(t, out, [q0, q1], 0.4, guard=False)
        self._go(t, "RELEASE", f"{outcome}: {note}")

    def _s_release(self, t, inp, out):
        if t < self.t_arm + self.p.settle_s:
            return
        # the base's way out (odom): across, the fixed pad off the bar (the bar is on its +w side), then back
        x, y, yaw = inp.pose
        side = _R(yaw) @ (-self.w[:2] * self.p.release_side)
        back = _R(yaw) @ np.array([-self.p.release_back, 0.0])
        self.way_out = [(x + side[0], y + side[1], yaw), (x + side[0] + back[0], y + side[1] + back[1], yaw)]
        self._go(t, "RETREAT")

    def _s_retreat(self, t, inp, out):
        if self.way_out:
            if self._drive(inp.pose, self.way_out[0], out, 0.10, tol=0.003):
                self.way_out.pop(0)
            return
        if self.q_stow is not None:
            q1 = self.q_stow.copy()
            q1[5] = float(inp.goal[5])
            if not self._move(t, out, inp.goal, q1, guard=False):
                self.log("door: no path to stow, holding")
        self._go(t, "STOW")

    def _s_stow(self, t, inp, out):
        if not self._arm_done(t, inp):
            return
        if self.outcome is None:
            self.outcome = "stalled" if getattr(self, "stalled", None) else "opened"
        self._end(t, self.outcome, getattr(self, "stalled", "") or "")
