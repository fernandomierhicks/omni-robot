"""People from the forward colour camera and the lidar, ROS-free (design.md 4.12; people_node.py runs it on the robot).

The lidar finds people by their legs (omni_console.people.detect: two short arcs at the A2M12's ~15 cm plane), but
chair and table legs look the same, and a person whose legs hide behind furniture is missed. The forward IMX662 (on
the nose under the lidar, 20 deg up, 90 x 58 deg) sees a standing adult whole from ~1.5 m. This module joins them:

  detect     a person detector on the camera's frame: YOLOX (Megvii, Apache-2.0; COCO-trained, class 0 = person) as
             ONNX through onnxruntime on the CPU. PersonDetector letterboxes the frame to the model's input, decodes
             the model's grid output and keeps person boxes after non-maximum suppression. The models are fetched by
             ros/tools/fetch_models.py into ~/.omni_robot/models (not in git)
  geometry   Camera: the camera's intrinsics and its pose on the base (from TF). A box gives a bearing interval in
             the base frame and a rough range from its foot point and its head top (an assumed height), each where
             it is inside the image. The camera is only ~0.1 m up, so both are rough (sigma ~0.3 m at 2 m, ~0.8 m at
             5 m): the lidar gives the range when it has the person
  fuse       each box claims the lidar candidate inside its bearing interval (and near its foot range when it has
             one): a lidar person the camera confirms. A box with no lidar candidate is a person at its foot point (or
             the head-top range). A lidar candidate the camera could see (in view, not too close) that no box claims
             is evidence it is not a person
  track      PeopleTracker keeps people in a fixed frame (odom or map) across frames: each track's chance of being a
             person as log-odds (a lidar leg pair is weak evidence, a box strong, "in view and no box" against), its
             velocity (an alpha-beta filter) and its id, so another person walking past does not take over a follow
             and a chair leg the camera has looked at stops being a person

Status per track: "person" (the camera has confirmed it), "candidate" (lidar only: not looked at, or not yet sure),
"not_person" (in the camera's view repeatedly with no box: furniture). Consumers (the console's Follow, omni_mcp's
robot_people) use "person" first, then "candidate", never "not_person".

Who the person is (identity) is not here yet: a track is the place an identity (face, clothing) would attach.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

# -- the detector --------------------------------------------------------------------------------------------------------

PERSON = 0                                  # COCO's class index for "person"
MODELS = {                                  # name: input size (the YOLOX release's ONNX files, 0.1.1rc0)
    "yolox_nano": 416,
    "yolox_tiny": 416,
    "yolox_s": 640,
}


@dataclass
class Box:
    """A person box in the image (pixels, x0 < x1 left to right, y0 < y1 top to bottom) and its score 0..1."""
    x0: float
    y0: float
    x1: float
    y1: float
    score: float

    @property
    def w(self) -> float:
        return self.x1 - self.x0

    @property
    def h(self) -> float:
        return self.y1 - self.y0


def letterbox(rgb: np.ndarray, size: int):
    """rgb H x W x 3 uint8 -> (size x size x 3 BGR float32 for YOLOX, scale): resized to fit, kept to the top left,
    padded with 114 (YOLOX's preproc)."""
    import cv2
    h, w = rgb.shape[:2]
    r = min(size / h, size / w)
    nh, nw = int(round(h * r)), int(round(w * r))
    img = cv2.resize(rgb, (nw, nh), interpolation=cv2.INTER_LINEAR)
    out = np.full((size, size, 3), 114.0, np.float32)
    out[:nh, :nw] = img[:, :, ::-1]                                     # RGB -> BGR (the model was trained on BGR)
    return out, r


def grids(size: int, strides=(8, 16, 32)):
    """YOLOX's anchor points: (N x 2 cell xy, N strides) for an input size x size."""
    g, s = [], []
    for st in strides:
        n = size // st
        yy, xx = np.meshgrid(np.arange(n), np.arange(n), indexing="ij")
        g.append(np.stack([xx.ravel(), yy.ravel()], 1))
        s.append(np.full(n * n, st))
    return np.concatenate(g).astype(np.float32), np.concatenate(s).astype(np.float32)


def decode(out: np.ndarray, size: int, scale: float, min_score: float = 0.35, nms_iou: float = 0.45,
           cls: int = PERSON) -> list:
    """YOLOX's raw output (N x (4 + 1 + classes): cx, cy, log w, log h on the grid, objectness, class scores) ->
    the class's boxes in the original image's pixels, best first, after NMS."""
    out = np.asarray(out, np.float32).reshape(-1, out.shape[-1])
    g, st = grids(size)
    if len(g) != len(out):
        raise ValueError(f"the model gave {len(out)} rows; a {size} px YOLOX gives {len(g)}")
    score = out[:, 4] * out[:, 5 + cls]
    keep = score >= min_score
    if not keep.any():
        return []
    o, g, st, score = out[keep], g[keep], st[keep], score[keep]
    cxy = (o[:, :2] + g) * st[:, None]
    wh = np.exp(np.clip(o[:, 2:4], -10, 10)) * st[:, None]
    b = np.concatenate([cxy - wh / 2, cxy + wh / 2], 1) / scale
    return [Box(*map(float, b[i]), float(score[i])) for i in nms(b, score, nms_iou)]


def nms(b: np.ndarray, score: np.ndarray, iou: float) -> list:
    """Indices of the boxes kept by greedy non-maximum suppression, best first."""
    order = np.argsort(-score)
    area = (b[:, 2] - b[:, 0]) * (b[:, 3] - b[:, 1])
    kept = []
    while len(order):
        i = order[0]
        kept.append(int(i))
        rest = order[1:]
        xx0 = np.maximum(b[i, 0], b[rest, 0])
        yy0 = np.maximum(b[i, 1], b[rest, 1])
        xx1 = np.minimum(b[i, 2], b[rest, 2])
        yy1 = np.minimum(b[i, 3], b[rest, 3])
        inter = np.clip(xx1 - xx0, 0, None) * np.clip(yy1 - yy0, 0, None)
        order = rest[inter / (area[i] + area[rest] - inter + 1e-9) <= iou]
    return kept


class PersonDetector:
    """detect(rgb) -> [Box]: YOLOX on onnxruntime's CPU provider. model: an .onnx path; size: its input (MODELS)."""

    def __init__(self, model: str, size: int | None = None, threads: int = 2, min_score: float = 0.35,
                 nms_iou: float = 0.45, min_height_px: float = 24.0):
        import onnxruntime as ort
        so = ort.SessionOptions()
        so.intra_op_num_threads = int(threads)
        so.inter_op_num_threads = 1
        self.sess = ort.InferenceSession(str(model), so, providers=["CPUExecutionProvider"])
        i = self.sess.get_inputs()[0]
        self.input = i.name
        self.size = int(size or i.shape[-1])
        self.min_score, self.nms_iou, self.min_h = min_score, nms_iou, min_height_px
        self.ms = 0.0                                                     # the last frame's inference time

    def detect(self, rgb: np.ndarray) -> list:
        import time
        img, r = letterbox(rgb, self.size)
        t = time.perf_counter()
        out = self.sess.run(None, {self.input: img.transpose(2, 0, 1)[None]})[0]
        self.ms = 1000.0 * (time.perf_counter() - t)
        return [b for b in decode(out[0], self.size, r, self.min_score, self.nms_iou) if b.h >= self.min_h]


# -- the camera's geometry -----------------------------------------------------------------------------------------------

@dataclass
class Camera:
    """A pinhole camera: K = (fx, fy, cx, cy) px, width x height px, T = its optical frame (x right, y down, z along
    the view) in the base frame (4 x 4; base_link: the floor under the robot's centre, x forward, z up)."""
    K: tuple
    width: int
    height: int
    T: np.ndarray

    def ray(self, u: float, v: float) -> np.ndarray:
        """the unit direction in the base frame of pixel (u, v)"""
        fx, fy, cx, cy = self.K
        d = self.T[:3, :3] @ np.array([(u - cx) / fx, (v - cy) / fy, 1.0])
        return d / np.linalg.norm(d)

    @property
    def origin(self) -> np.ndarray:
        return self.T[:3, 3]

    def azimuth(self, u: float, v: float) -> float:
        d = self.ray(u, v)
        return math.atan2(d[1], d[0])

    def bearings(self, b: Box) -> tuple:
        """(left-most, right-most, middle) azimuth (rad, base frame, + to the left) of the box at its middle row"""
        vm = 0.5 * (b.y0 + b.y1)
        a_l, a_r, a_m = self.azimuth(b.x0, vm), self.azimuth(b.x1, vm), self.azimuth(0.5 * (b.x0 + b.x1), vm)
        return max(a_l, a_r), min(a_l, a_r), a_m

    def floor_point(self, u: float, v: float):
        """where pixel (u, v)'s ray meets the floor (z = 0) in the base frame, or None (at or above the horizon)"""
        d, o = self.ray(u, v), self.origin
        if d[2] > -1e-3:
            return None
        s = -o[2] / d[2]
        return o[:2] + s * d[:2]

    def project(self, p) -> tuple | None:
        """(u, v) of base-frame point p, or None behind the camera"""
        q = np.linalg.inv(self.T) @ np.array([p[0], p[1], p[2], 1.0])
        if q[2] <= 1e-6:
            return None
        fx, fy, cx, cy = self.K
        return fx * q[0] / q[2] + cx, fy * q[1] / q[2] + cy

    def sees(self, x: float, y: float, z0: float = 0.3, z1: float = 1.2, margin: float = 0.05) -> bool:
        """a person-sized upright segment at (x, y) (z0..z1 m) is inside the image (both ends, `margin` of the size
        inside the edges)"""
        mu, mv = margin * self.width, margin * self.height
        for z in (z0, z1):
            uv = self.project((x, y, z))
            if uv is None or not (mu <= uv[0] <= self.width - mu and mv <= uv[1] <= self.height - mv):
                return False
        return True


def plausible(cam: Camera, b: Box, floating_min_score: float = 0.5, edge_px: float = 4.0) -> bool:
    """A box whose bottom is inside the image but above the horizon has no feet on the floor in view: a person behind
    a sofa, or a picture or sign on a wall. Those need floating_min_score (on the sim house's renders YOLOX nano scored
    wall tags 0.36-0.45, people standing in the open 0.45-0.9); boxes with their feet on the floor, or cut off by the
    frame's bottom, keep the detector's own threshold."""
    if b.y1 >= cam.height - edge_px:
        return True
    d = cam.ray(0.5 * (b.x0 + b.x1), b.y1)
    return d[2] < 0.0 or b.score >= floating_min_score


@dataclass
class Sighting:
    """One person the camera saw this frame: its box, bearing interval, and a range if the image gives one, with its
    one-sigma error (m): from the foot point ("foot"), the head top with an assumed height ("height"), both combined
    ("both"), or none (cut off at the top and the bottom: bearing only)."""
    box: Box
    az: tuple                   # (left, right, middle) rad, base frame
    xy: tuple | None            # base-frame floor point, or None
    how: str | None             # "foot" | "height" | "both" | None
    sigma: float = math.inf     # m, the range's one-sigma error


def sight(cam: Camera, b: Box, edge_px: float = 4.0, person_h: float = 1.65, person_h_sigma: float = 0.25,
          edge_sigma: float = 0.03) -> Sighting:
    """A box as a Sighting. Two ranges, each where its edge is clear of the image's edge (`edge_px`):
      foot   the box's bottom on the floor. The camera is ~0.1 m up, so the feet are a few degrees under the horizon
             past ~2 m and a pixel moves them a lot: sigma = (r^2 + z^2) / z * the edge's angle sigma, the edge
             `edge_sigma` of the box's height (a detector's box is loose by a few %: on a rendered person at 3 m its
             bottom sat 13 px of 540 under the feet, which read 2.4 m)
      head   the box's top assumed person_h up (sigma person_h_sigma: seated people, children and tall people):
             sigma = r * person_h_sigma / (person_h - camera height); anyone shorter than that reads too far
    combined by their variances along the box's middle bearing. The lidar's range, when it has the person, is better
    than either (fuse)."""
    az = cam.bearings(b)
    um = 0.5 * (b.x0 + b.x1)
    o = cam.origin
    s_ang = max(2.0, edge_sigma * b.h) / cam.K[1]
    est = []                                                          # (horizontal range from the camera, sigma)
    if b.y1 < cam.height - edge_px:
        d = cam.ray(um, b.y1)
        if d[2] < -1e-3:
            r = o[2] / -d[2] * math.hypot(d[0], d[1])                 # r = z / tan(depression); dr = (r^2 + z^2) / z dA
            est.append((r, (r * r + o[2] ** 2) / max(o[2], 0.02) * s_ang + 0.02 * r))
    if b.y0 > edge_px:
        d = cam.ray(um, b.y0)
        rise = person_h - o[2]
        if d[2] > 1e-3 and rise > 0:
            r = rise / d[2] * math.hypot(d[0], d[1])
            est.append((r, r * person_h_sigma / rise + 0.02 * r))
    if not est:
        return Sighting(b, az, None, None)
    w = [1.0 / (sg * sg) for _, sg in est]
    r = sum(wi * ri for wi, (ri, _) in zip(w, est)) / sum(w)
    sigma = math.sqrt(1.0 / sum(w))
    how = "both" if len(est) == 2 else ("foot" if b.y1 < cam.height - edge_px else "height")
    a = az[2]
    return Sighting(b, az, (float(o[0] + r * math.cos(a)), float(o[1] + r * math.sin(a))), how, sigma)


# -- the camera and the lidar together -----------------------------------------------------------------------------------

@dataclass
class Observation:
    """A person this frame in the base frame: from the lidar, the camera or both."""
    x: float
    y: float
    lidar: bool
    camera: bool
    score: float = 0.0          # the box's score (camera)
    how: str = "lidar"          # lidar | foot | height | both (the camera's range: Sighting.how)
    box: Box | None = None
    looked: bool = False        # lidar only: the camera had it in view and saw no person there
    sigma: float = 0.05         # m: the position's one-sigma error (the lidar's ~5 cm; a camera range's Sighting.sigma)


def fuse(sightings: list, legs: list, cam: Camera | None, bearing_margin: float = math.radians(3.0),
         range_k: float = 2.5, range_tol_min: float = 0.5, look_min: float = 0.8, look_max: float = 6.0,
         look_z: tuple = (0.1, 1.6)) -> list:
    """sightings (camera, this frame) and legs (lidar people (x, y), base frame) -> [Observation].

    Each sighting takes the lidar candidate inside its bearing interval (+ margin) nearest the camera's range (within
    range_k of its sigma, at least range_tol_min m; any range when the camera has none), best-scored boxes first, one
    candidate each; the lidar's position is kept. A sighting no candidate takes is a camera-only observation.
    "Looked" needs a whole person (look_z: feet to head) inside the frame: the detector misses people cut off at the
    frame's top (nearer than ~1.6 m from the robot's centre: on sim renders nano found 9/18 at 1.2 m, 15/18 at 1.5 m,
    all 18 at 2-4 m), so a near miss is no evidence against. Unclaimed lidar candidates the camera had in view between look_min and look_max m are marked looked."""
    out, used = [], set()
    for s in sorted(sightings, key=lambda s: -s.box.score):
        lo, hi = s.az[1] - bearing_margin, s.az[0] + bearing_margin
        r_cam = math.hypot(*s.xy) if s.xy is not None else None
        best, bd = None, math.inf
        for j, (x, y) in enumerate(legs):
            if j in used:
                continue
            a = math.atan2(y, x)
            if not lo <= a <= hi:
                continue
            r = math.hypot(x, y)
            if r_cam is not None and abs(r - r_cam) > max(range_tol_min, range_k * s.sigma):
                continue
            cost = abs(r - r_cam) if r_cam is not None else abs(a - s.az[2]) * r
            if cost < bd:
                best, bd = j, cost
        if best is not None:
            used.add(best)
            x, y = legs[best]
            out.append(Observation(x, y, True, True, s.box.score, "lidar", s.box))
        elif s.xy is not None:
            out.append(Observation(s.xy[0], s.xy[1], False, True, s.box.score, s.how, s.box, sigma=s.sigma))
    for j, (x, y) in enumerate(legs):
        if j in used:
            continue
        r = math.hypot(x, y)
        looked = cam is not None and look_min <= r <= look_max and cam.sees(x, y, look_z[0], look_z[1])
        out.append(Observation(x, y, True, False, how="lidar", looked=looked))
    return out


# -- tracks over time ----------------------------------------------------------------------------------------------------

def _logit(p: float) -> float:
    return math.log(p / (1 - p))


@dataclass
class Track:
    id: int
    x: float
    y: float
    t: float                            # last update
    t0: float                           # first seen
    vx: float = 0.0
    vy: float = 0.0
    lo: float = 0.0                     # log-odds it is a person
    t_cam: float = -math.inf            # last camera sighting
    t_lidar: float = -math.inf
    n_cam: int = 0
    how: str = "lidar"
    score: float = 0.0
    box: Box | None = None
    extra: dict = field(default_factory=dict)   # for identity later (face / clothing), set by the caller

    def p(self) -> float:
        return 1.0 / (1.0 + math.exp(-self.lo))


class PeopleTracker:
    """update(t, observations in the base frame, base pose (x, y, yaw) in the fixed frame) -> tracks.

    Association: greedy nearest pairs within gate + speed allowance. Evidence per frame (log-odds): a lidar leg pair
    `lidar_lo` (weak: chair legs too; the lidar alone never lifts a track past lidar_cap), a camera box `camera_lo` x
    its score, looked-at-and-no-box `looked_lo`. A track is a "person" above p_person, "not_person" below p_not. Tracks end lost_s after the last update ("person" tracks
    coast keep_s on their velocity first). A camera-only observation starts a track only if its range sigma is under
    new_sigma, and moves a track by less the rougher it is."""

    def __init__(self, gate: float = 0.7, prior: float = 0.4, lidar_lo: float = 0.15, camera_lo: float = 2.5,
                 looked_lo: float = -0.6, lidar_cap: float = 0.6, lo_max: float = 6.0, p_person: float = 0.8,
                 p_not: float = 0.12, new_sigma: float = 0.5, lost_s: float = 2.0, keep_s: float = 4.0, alpha: float = 0.5, beta: float = 0.2, v_max: float = 2.5):
        self.gate, self.prior_lo = gate, _logit(prior)
        self.lidar_lo, self.camera_lo, self.looked_lo, self.lo_max = lidar_lo, camera_lo, looked_lo, lo_max
        self.lidar_cap_lo = _logit(lidar_cap)
        self.new_sigma = new_sigma
        self.p_person, self.p_not = p_person, p_not
        self.lost_s, self.keep_s = lost_s, keep_s
        self.alpha, self.beta, self.v_max = alpha, beta, v_max
        self.tracks: list[Track] = []
        self._next = 1

    def status(self, tr: Track) -> str:
        p = tr.p()
        return "person" if p >= self.p_person else ("not_person" if p <= self.p_not else "candidate")

    def update(self, t: float, obs: list, pose) -> list:
        x0, y0, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)
        world = [(x0 + c * o.x - s * o.y, y0 + s * o.x + c * o.y) for o in obs]
        pred = []
        for tr in self.tracks:
            dt = max(0.0, t - tr.t)
            pred.append((tr.x + tr.vx * dt, tr.y + tr.vy * dt, dt))
        pairs = []
        for i, (px, py, dt) in enumerate(pred):
            g = self.gate + min(self.v_max, math.hypot(self.tracks[i].vx, self.tracks[i].vy)) * dt
            for j, (wx, wy) in enumerate(world):
                d = math.hypot(wx - px, wy - py)
                if d < g:
                    pairs.append((d, i, j))
        pairs.sort()
        ti, oj = set(), set()
        for d, i, j in pairs:
            if i in ti or j in oj:
                continue
            ti.add(i)
            oj.add(j)
            self._correct(self.tracks[i], pred[i], world[j], obs[j], t)
        for j, o in enumerate(obs):
            if j not in oj and (o.lidar or o.camera):
                if not o.lidar and o.sigma > self.new_sigma:
                    continue                                   # a rough camera-only range: no new track from it
                tr = Track(self._next, world[j][0], world[j][1], t, t, lo=self.prior_lo)
                self._next += 1
                self._evidence(tr, o, t)
                self.tracks.append(tr)
        keep = []
        for tr in self.tracks:
            age = t - tr.t
            limit = self.keep_s if self.status(tr) == "person" else self.lost_s
            if age <= limit:
                keep.append(tr)
        self.tracks = keep
        return self.tracks

    def _correct(self, tr: Track, pred, w, o: Observation, t: float):
        px, py, dt = pred
        rx, ry = w[0] - px, w[1] - py
        a = self.alpha if o.lidar else min(self.alpha, 0.1 / max(o.sigma, 1e-3))   # a rough camera range: less
        tr.x, tr.y = px + a * rx, py + a * ry
        if dt > 1e-3:
            tr.vx += self.beta * rx / dt
            tr.vy += self.beta * ry / dt
            v = math.hypot(tr.vx, tr.vy)
            if v > self.v_max:
                tr.vx, tr.vy = tr.vx * self.v_max / v, tr.vy * self.v_max / v
        tr.t = t
        self._evidence(tr, o, t)

    def _evidence(self, tr: Track, o: Observation, t: float):
        if o.camera:
            tr.lo += self.camera_lo * max(0.3, o.score)
            tr.t_cam, tr.n_cam, tr.score, tr.box = t, tr.n_cam + 1, o.score, o.box
        if o.lidar:
            tr.t_lidar = t
            if not o.camera:
                tr.lo = min(tr.lo + self.lidar_lo, max(tr.lo, self.lidar_cap_lo))
            if o.looked:
                tr.lo += self.looked_lo
        tr.how = o.how if not (o.lidar and o.camera) else "lidar+camera"
        tr.lo = max(-self.lo_max, min(self.lo_max, tr.lo))

    def report(self, t: float, pose) -> list:
        """the tracks as dicts for /people: fixed-frame and base-frame position, velocity, status, evidence"""
        x0, y0, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)
        out = []
        for tr in self.tracks:
            dx, dy = tr.x - x0, tr.y - y0
            bx, by = c * dx + s * dy, -s * dx + c * dy
            out.append(dict(id=tr.id, status=self.status(tr), p=round(tr.p(), 3), x=round(tr.x, 3), y=round(tr.y, 3),
                            vx=round(tr.vx, 2), vy=round(tr.vy, 2), base_x=round(bx, 3), base_y=round(by, 3),
                            range=round(math.hypot(bx, by), 2), bearing_deg=round(math.degrees(math.atan2(by, bx)), 1),
                            camera=t - tr.t_cam < 1.0, lidar=t - tr.t_lidar < 1.0, how=tr.how,
                            seen_s=round(t - tr.t, 2), age_s=round(t - tr.t0, 1), camera_hits=tr.n_cam,
                            score=round(tr.score, 2)))
        rank = {"person": 0, "candidate": 1, "not_person": 2}
        out.sort(key=lambda d: (rank[d["status"]], d["range"]))
        return out



def move_points(points, src, dst) -> list:
    """base-frame points seen with the base at fixed-frame pose src (x, y, yaw), as seen from pose dst: the camera's
    frame and the lidar's scan come at different times while the robot moves (0.2 s at 1 rad/s is 11 deg)."""
    cs, ss = math.cos(src[2]), math.sin(src[2])
    cd, sd = math.cos(dst[2]), math.sin(dst[2])
    out = []
    for x, y in points:
        wx, wy = src[0] + cs * x - ss * y - dst[0], src[1] + ss * x + cs * y - dst[1]
        out.append((cd * wx + sd * wy, -sd * wx + cd * wy))
    return out
