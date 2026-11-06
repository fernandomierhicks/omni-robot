"""Nav2's view of the floor from the arm robot's depth camera (design.md 4.5, 4.11). No ROS.

The lidar scans 163 mm up, so the toys the arm robot tidies (12-40 mm) are under it; Nav2 drove through them and the
bumper pushed the taller ones away. A raw depth cloud in Nav2's own ObstacleLayer saw none of them: its marking floor
(4 cm over z = 0 in odom) had to clear the base's pitch on its pods (odom -> base_link has none) and the depth noise.
Here each frame is measured against the floor the camera sees in that frame (floor_vision: a plane fitted to the
floor, rugs and mats as raised floor levels), so the base's pitch and the camera's calibration error drop out, and a
thing on the floor is told from a rug's edge or a threshold by floor_vision's own tests (its ring of floor).

floor_obstacles(), per frame, in base_link:
  things  floor_vision's objects 8 mm - 10 cm tall (a big one from 25 mm) within THING_RANGE, as points over each
          footprint
  tall    anything TALL over the floor level it stands on (a shoe, a chair leg, a person's feet), up to TOP
  floor   the floor the camera plainly sees: the ends of its rays to the floor (module _clear_cells) cover a cell and
          the cells round it (not the view's edge, not a thing's shadow), and nothing raised is within KEEP

ObstacleGrid keeps them in the map frame over the frames, as Nav2 takes them (a StaticLayer on its grid,
stack.FLOOR_LAYER): a cell is marked when something tall is seen in it, or a thing in THING_HITS frames without a
clear sight of its floor in between, and stays marked until the camera
plainly sees floor there, so a toy stays marked when it passes under the camera's view (the D435 at 7.5 deg sees no
floor nearer than ~0.34 m from the robot's centre), out of the field to the side or behind the robot. Tall marks also
lapse TALL_TTL after they were last seen (people walk; the lidar covers most tall things). forget() clears a disc: the
thing the arm just picked, and every frame the robot's own outline (the camera can't see under it, and a mark there
puts every plan's start in collision). Nav2's own costmap clearing (its recoveries) leaves a StaticLayer alone.

Tried first and dropped (Nov 5): the marks in a VoxelLayer, cleared by Nav2's ray tracing. A ray to the floor beside a
toy crosses the toy's 5 cm cell near the floor and erased it once the toy was out of view; rays aimed to clear only
their last cell don't reach it (the VoxelLayer pulls every ray's end back by two cells), and the rolling local
costmap lost its voxel marks as the window moved.
"""

from __future__ import annotations

import math

import numpy as np

from omni_arm import floor_vision as fv

TALL = 0.025            # m over the floor level: anything taller is an obstacle (a rug or a threshold is lower)
LOW = 0.008             # m: an object lower than this the robot rolls over (it crosses 18 mm transitions): not
                        # marked, which also keeps the odd strip of a rug's edge that floor_vision lets by (7 mm) out.
                        # A big one (longer than floor_vision.MAX_SIZE) is marked only from TALL: a long low strip is
                        # a carpet's or rug's edge, or a threshold (Nov 5: the bedroom's 12 mm carpet, seen through its
                        # door from the hallway, came out as a 'big' thing and closed the door)
TOP = 0.60              # m: higher than the robot, never in its way
MARK_STEP = 0.025       # m between the points over an object's footprint (the costmap has 5 cm cells)
TALL_MIN_POINTS = 2     # points of a tall thing per MARK_STEP cell: a lone stray pixel marks nothing
CLEAR_CELL = 0.025      # m: the seen floor's cells
SEEN_R = 2              # cells round a floor cell that must be seen too: a 5 cm map cell reaches up to 3.75 cm past
                        # the point, which mustn't hold a toy just outside the view
KEEP = 0.075            # m: no clearing this near anything raised (H_MIN: 6 mm) over the floor
ROBOT_RADIUS = 0.24     # m: nothing inside the robot's outline (Nav2's robot_radius) is seen as an obstacle
TALL_TTL = 20.0         # s: a tall mark not seen again lapses (an object's mark lasts until its floor is seen)
THING_RANGE = 1.5       # m from the camera: floor_vision's things nearer than this only (its ring test thins out
                        # farther; a toy farther away is marked as the robot comes nearer). Tall things: RANGE
THING_HITS = 2          # frames a cell must be seen holding a thing, with no clear sight of its floor in between,
                        # before it is marked: a toy is there in every frame, the odd patch of a thick rug's square
                        # edge isn't (Nov 5: a 16 mm rug's edge closed the way onto it)
EMPTY, THING, TALLMARK = 0, 1, 2


def _footprint(d: fv.Detection) -> np.ndarray:
    """(n, 2) points over a detection's footprint, MARK_STEP apart, its outline included."""
    nl = max(2, math.ceil(d.length / MARK_STEP) + 1)
    nw = max(2, math.ceil(d.width / MARK_STEP) + 1)
    a, b = np.meshgrid(np.linspace(-d.length / 2, d.length / 2, nl), np.linspace(-d.width / 2, d.width / 2, nw))
    c, s = math.cos(d.yaw), math.sin(d.yaw)
    return np.stack([d.x + c * a.ravel() - s * b.ravel(), d.y + s * a.ravel() + c * b.ravel()], axis=1)


def _cells(xy: np.ndarray, size: float, min_points: int = 1) -> np.ndarray:
    """The centres of the size x size cells holding at least min_points of xy."""
    if not len(xy):
        return np.zeros((0, 2))
    k = np.floor(xy / size).astype(np.int64)
    u, n = np.unique(k, axis=0, return_counts=True)
    return (u[n >= min_points] + 0.5) * size


def _clear_cells(P: np.ndarray, floor: np.ndarray, raised: np.ndarray, cam: np.ndarray) -> np.ndarray:
    """The centres of the CLEAR_CELL cells plainly seen as floor (module notes). A floor point the camera sees vouches
    for the end of its ray where the ray runs lower than LOW over the floor: nothing that tall stands there, or the
    ray would have stopped on it. Those stretches (~7% of a ray: 15 cm at 2 m, where the floor's rows are as far
    apart) make up the seen floor; a cell is cleared when it and SEEN_R cells round it are seen and nothing raised is
    within KEEP. Unseen: under and beside the view, and a thing's shadow."""
    fxy = _cells(P[floor, :2], CLEAR_CELL / 2)                         # the floor, thinned
    if not len(fxy):
        return np.zeros((0, 2))
    tail = 1.0 - LOW / max(cam[2], LOW)                                # where a ray to the floor is LOW over it
    span = np.hypot(fxy[:, 0] - cam[0], fxy[:, 1] - cam[1]) * (1.0 - tail)
    n = int(np.ceil(span.max() / (CLEAR_CELL / 2))) + 1
    f = tail + (1.0 - tail) * np.linspace(0.0, 1.0, n)
    seen_xy = (cam[:2] + f[None, :, None] * (fxy[:, None, :] - cam[:2])).reshape(-1, 2)
    k_seen = np.floor(seen_xy / CLEAR_CELL).astype(np.int64)
    k_up = np.floor(P[raised, :2] / CLEAR_CELL).astype(np.int64)
    k0 = k_seen.min(axis=0) - SEEN_R - 1
    if len(k_up):
        k0 = np.minimum(k0, k_up.min(axis=0))
    k_seen, k_up = k_seen - k0, k_up - k0
    shape = tuple(np.maximum(k_seen.max(axis=0), k_up.max(axis=0) if len(k_up) else 0) + SEEN_R + 2)
    seen, up = np.zeros(shape, bool), np.zeros(shape, bool)
    seen[k_seen[:, 0], k_seen[:, 1]] = True
    up[k_up[:, 0], k_up[:, 1]] = True
    ok = seen & ~fv._dilate(~seen, SEEN_R) & ~fv._dilate(up, int(math.ceil(KEEP / CLEAR_CELL)))
    i, j = np.nonzero(ok)
    return (np.stack([i, j], axis=1) + k0 + 0.5) * CLEAR_CELL


def floor_obstacles(frame: fv.DepthFrame, stride: int = 1, rng: np.random.Generator | None = None,
                    robot_radius: float = ROBOT_RADIUS):
    """(things, tall, floor, detections): (n, 2) points in base_link (module notes) and floor_vision's detections in
    base_link. Nothing at all when the frame shows too little floor to measure against."""
    none = np.zeros((0, 2))
    P = fv.points(frame, stride)
    P = P[np.hypot(P[:, 0], P[:, 1]) > robot_radius]
    dets, h, on_floor = fv.detect_points(P, frame.T_base_cam[:2, 3], rng)
    if h is None or on_floor.sum() < 50:
        return none, none, none, []
    tall = _cells(P[(h > TALL) & (h < TOP), :2], MARK_STEP, TALL_MIN_POINTS)
    cam = frame.T_base_cam[:2, 3]
    objs = [_footprint(d) for d in dets if d.top >= (TALL if d.big else LOW)
            and math.hypot(d.x - cam[0], d.y - cam[1]) < THING_RANGE]
    things = np.concatenate(objs) if objs else none
    floor = _clear_cells(P, on_floor, (h > fv.H_MIN) & ~on_floor, frame.T_base_cam[:3, 3])
    return things, tall, floor, dets


def to_frame(xy: np.ndarray, pose) -> np.ndarray:
    """Points from base_link into a frame where the base is at pose (x, y, yaw)."""
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return np.c_[pose[0] + c * xy[:, 0] - s * xy[:, 1], pose[1] + s * xy[:, 0] + c * xy[:, 1]] if len(xy) else xy


class ObstacleGrid:
    """The floor's things in the map frame over many frames (module notes), on the map's own cells (the geometry of
    the map Nav2's global costmap is built on, so a StaticLayer takes it as it is)."""

    def __init__(self):
        self.res = None
        self.origin = (0.0, 0.0)
        self.kind = np.zeros((0, 0), np.int8)              # [row (y), col (x)]: EMPTY | THING | TALLMARK
        self.t = np.zeros((0, 0))                           # when a tall mark was last seen
        self.hits = np.zeros((0, 0), np.int16)              # frames a thing was seen in a cell since its floor last was

    def geometry(self, res: float, ox: float, oy: float, width: int, height: int) -> bool:
        """Take the map's geometry, keeping the marks where the old and new cells overlap. True if it changed."""
        if (self.res, self.origin, self.kind.shape) == (res, (ox, oy), (height, width)):
            return False
        new = [np.zeros((height, width), a.dtype) for a in (self.kind, self.t, self.hits)]
        if self.res == res and self.kind.size:
            di, dj = round((self.origin[1] - oy) / res), round((self.origin[0] - ox) / res)
            h0, w0 = self.kind.shape
            i0, i1, j0, j1 = max(0, di), min(height, di + h0), max(0, dj), min(width, dj + w0)
            if i0 < i1 and j0 < j1:
                for a, b in zip(new, (self.kind, self.t, self.hits)):
                    a[i0:i1, j0:j1] = b[i0 - di:i1 - di, j0 - dj:j1 - dj]
        self.res, self.origin = res, (ox, oy)
        self.kind, self.t, self.hits = new
        return True

    def _ij(self, xy: np.ndarray):
        if not len(xy) or self.res is None:
            return np.zeros(0, int), np.zeros(0, int)
        j = np.floor((xy[:, 0] - self.origin[0]) / self.res).astype(int)
        i = np.floor((xy[:, 1] - self.origin[1]) / self.res).astype(int)
        ok = (i >= 0) & (i < self.kind.shape[0]) & (j >= 0) & (j < self.kind.shape[1])
        return i[ok], j[ok]

    def update(self, t: float, things: np.ndarray, tall: np.ndarray, floor: np.ndarray) -> bool:
        """One frame's points in the map frame: the seen floor clears (and resets its thing count), then tall things
        mark, and things count up, marking at THING_HITS (a thing's mark is never turned into a lapsing tall one); then
        old tall marks lapse. True if anything changed."""
        before = self.kind.copy()
        ti, tj = self._ij(things)
        thing_now = np.zeros(self.kind.shape, bool)
        thing_now[ti, tj] = True
        i, j = self._ij(floor)
        clear = ~thing_now[i, j]                         # a cell holding a thing this frame isn't its floor
        self.kind[i[clear], j[clear]] = EMPTY
        self.hits[i[clear], j[clear]] = 0
        i, j = self._ij(tall)
        keep = self.kind[i, j] != THING
        self.kind[i[keep], j[keep]] = TALLMARK
        self.t[i, j] = t
        i, j = np.nonzero(thing_now)
        self.hits[i, j] += 1
        sure = self.hits[i, j] >= THING_HITS
        self.kind[i[sure], j[sure]] = THING
        self.kind[(self.kind == TALLMARK) & (self.t < t - TALL_TTL)] = EMPTY
        return not np.array_equal(before, self.kind)

    def forget(self, x: float, y: float, r: float) -> bool:
        """Clear every mark within r of (x, y) (map frame). True if anything changed."""
        if self.res is None:
            return False
        H, W = self.kind.shape
        n = int(math.ceil(r / self.res)) + 1
        ci, cj = int((y - self.origin[1]) / self.res), int((x - self.origin[0]) / self.res)
        i0, i1, j0, j1 = max(0, ci - n), min(H, ci + n + 1), max(0, cj - n), min(W, cj + n + 1)
        if i0 >= i1 or j0 >= j1:
            return False
        yy, xx = np.mgrid[i0:i1, j0:j1]
        near = np.hypot(self.origin[0] + (xx + 0.5) * self.res - x, self.origin[1] + (yy + 0.5) * self.res - y) < r
        win = self.kind[i0:i1, j0:j1]
        changed = bool(np.any(win[near] != EMPTY))
        win[near] = EMPTY
        self.hits[i0:i1, j0:j1][near] = 0
        return changed

    def occupancy(self) -> np.ndarray:
        """The grid as nav_msgs/OccupancyGrid data (row-major from the origin): 100 marked, -1 (no information) else."""
        return np.where(self.kind != EMPTY, 100, -1).astype(np.int8).ravel()

    def marked(self) -> int:
        return int(np.count_nonzero(self.kind))
