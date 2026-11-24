"""A kitchen base cabinet with a hinged door on self-closing concealed hinges, for the door-opening study (Nov 21;
studies/door_open.py). Added to a built scene's XML (with_cabinet), as imx662_cam adds its camera.

Geometry (a US base cabinet; nominal, est): carcass 0.61 m deep, 0.876 m to the worktop's underside, a toe kick 0.10 m
high recessed 0.075 m; a full-overlay door 457 x 762 x 19 mm (18 x 30 in) from 0.105 m up, a 2 mm gap to the face
frame; a 38 mm worktop to 0.914 m. One neighbour on each side (no doors that move). The pull is Fernando's choice of
a low bar pull (Nov 21): a vertical bar, 128 mm centres, d 12 mm, 32 mm from the door's face to the bar's axis, 40 mm in
from the free edge, its middle at handle_z (0.38 m: the SO-101's level reach from the robot is 0.30-0.45 m high).

Hinge: the door turns about a vertical axis at its back face's hinge-side edge (a concealed hinge's four-bar moves the
door out as it opens; not modelled, so the neighbour on the hinge side touches nothing, see contacts). Self-closing
(est, Blum CLIP top-like): a closing torque tau0 at shut falling linearly to 0 at close_deg (30 deg), free beyond to the
110 deg stop; two hinges: tau0 = 2 x 1.0 N m (est: ~5 N at the pull; VERIFY with a luggage scale on the real door),
friction 0.05 N m and damping 0.05 N m s (est). The door 4.5 kg (est: 19 mm MDF/plywood).

Contacts: the door and the pull contype 4. The door's conaffinity 3: the robot's structure (bit 1), the arm's links
and fingertips (bit 2); the pull's 11: also the finger pads (bit 8). The cabinet (static) contype 4, conaffinity 3:
the robot and the arm touch it, the door doesn't (4 & 3 = 0 both ways). The pull's friction 0.7 (est: TPU on a
metal pull) with priority over the pads' 1.1.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass
class DoorSpec:
    hinge: tuple = (0.10, -0.25)           # (x, y) world: the hinge's axis (the door's back face, hinge edge)
    closed_heading: float = math.pi / 2    # rad: from the hinge along the shut door toward its free edge
    sense: int = -1                        # +1 opens counter-clockwise seen from above
    width: float = 0.457
    height: float = 0.762
    thick: float = 0.019
    z0: float = 0.105                      # the door's bottom edge
    gap: float = 0.002                     # the door's back face off the cabinet's face
    mass: float = 4.5
    handle_in: float = 0.040               # the bar's axis this far in from the free edge
    handle_off: float = 0.032              # ... and this far out from the door's face
    handle_z: float = 0.38                 # the bar's middle
    bar_d: float = 0.012
    bar_centres: float = 0.128
    post_d: float = 0.010
    bar_mu: float = 0.7
    tau0: float = 2.0                      # N m at shut, both hinges
    close_deg: float = 30.0
    friction: float = 0.05                 # N m
    damping: float = 0.05                  # N m s
    open_max_deg: float = 110.0
    depth: float = 0.61
    top: float = 0.876
    kick_h: float = 0.10
    kick_in: float = 0.075
    neighbours: bool = True
    locked: bool = False                   # a child lock: the door opens 1 deg and stops hard
    # the door's AprilTag (Nov 21): a 36h11 code printed with a one-cell white margin, stuck on the door's face at its
    # bottom edge right under the pull. The arm robot's D435 (on the riser, 0.19 m up, 30 deg down) sees nothing on a
    # door above ~0.18 m (its view's top is 1 deg under the horizontal), so this is the only place on the door it sees
    tag_id: int = 7                        # -1: no tag
    tag_size: float = 0.048                # m: the black square (8 cells of 6 mm; 60 mm printed with the margin)
    tag_z: float = 0.142                   # m: its centre over the floor (the print's bottom 7 mm above the door's)
    tag_roll_deg: float = 0.0              # stuck on turned this much about its normal (by hand: ~1 deg, est)
    tag_u: float | None = None             # m: its centre from the hinge along the door (None: under the pull)

    @property
    def u0(self) -> np.ndarray:
        return np.array([math.cos(self.closed_heading), math.sin(self.closed_heading)])

    @property
    def n0(self) -> np.ndarray:
        a = self.closed_heading + self.sense * math.pi / 2
        return np.array([math.cos(a), math.sin(a)])

    @property
    def handle_r(self) -> float:
        return self.width - self.handle_in

    @property
    def tag_along(self) -> float:
        return self.handle_r if self.tag_u is None else self.tag_u

    @property
    def handle_out(self) -> float:
        return self.thick + self.handle_off

    def bar(self, th: float) -> np.ndarray:
        """The bar's axis (world xy) with the door at th."""
        c, s = math.cos(self.sense * th), math.sin(self.sense * th)
        R = np.array([[c, -s], [s, c]])
        return np.asarray(self.hinge) + R @ (self.handle_r * self.u0 + self.handle_out * self.n0)

    def closing_torque(self, th: float) -> float:
        """The hinges' torque toward shut (N m, >= 0) at angle th (rad)."""
        c = math.radians(self.close_deg)
        return self.tau0 * max(0.0, 1.0 - max(th, 0.0) / c)


def _f(*v) -> str:
    return " ".join(f"{x:.6g}" for x in v)


def _box(name, centre, half, yaw, rgba, extra=""):
    q = (math.cos(yaw / 2), 0, 0, math.sin(yaw / 2))
    return (f'<geom name="{name}" type="box" size="{_f(*half)}" pos="{_f(*centre)}" quat="{_f(*q)}" '
            f'rgba="{_f(*rgba)}" {extra}/>')


def xml(s: DoorSpec) -> str:
    u, n = s.u0, s.n0
    yaw_u = math.atan2(u[1], u[0])
    h = np.asarray(s.hinge, float)
    face = h - n * s.gap                       # the cabinet's face, at the hinge
    W, D = s.width + 0.003, s.depth
    wood, white, top_c, kick_c = (0.93, 0.92, 0.88, 1), (0.97, 0.97, 0.95, 1), (0.35, 0.33, 0.31, 1), (0.2, 0.2, 0.2, 1)
    static = 'contype="4" conaffinity="3" group="0"'
    out = []
    units = [0.0] + ([-W, W] if s.neighbours else [])
    for k, off in enumerate(units):
        c0 = face + u * (W / 2 - 0.0015 + off) - n * (D / 2)
        out.append(_box(f"cab{k}_body", (*c0, (s.kick_h + s.top) / 2), (W / 2, D / 2, (s.top - s.kick_h) / 2), yaw_u,
                        white, static))
        ck = face + u * (W / 2 - 0.0015 + off) - n * (s.kick_in + (D - s.kick_in) / 2)
        out.append(_box(f"cab{k}_kick", (*ck, s.kick_h / 2), (W / 2, (D - s.kick_in) / 2, s.kick_h / 2), yaw_u,
                        kick_c, static))
        if off:                                # the neighbours' doors: fixed, part of the face
            cd = face + u * (W / 2 - 0.0015 + off) + n * (s.gap + s.thick / 2)
            out.append(_box(f"cab{k}_door", (*cd, s.z0 + s.height / 2), (s.width / 2 - 0.0015, s.thick / 2,
                                                                          s.height / 2), yaw_u, wood, static))
    ct = face + u * (W / 2 - 0.0015) - n * (D / 2 - 0.025)
    out.append(_box("worktop", (*ct, s.top + 0.019), (W * len(units) / 2 + 0.01, D / 2 + 0.025, 0.019), yaw_u, top_c,
                    static))
    # the door: its body at the hinge's axis, frame turned with the shut door (x along u); geoms in that frame
    dn = 1.0 if s.sense > 0 else -1.0          # the outward normal is the body's +y (sense +1) or -y
    door = (f'<geom name="door" type="box" size="{_f(s.width / 2, s.thick / 2, s.height / 2)}" '
            f'pos="{_f(s.width / 2, dn * s.thick / 2, s.z0 + s.height / 2)}" mass="{s.mass}" rgba="{_f(*wood)}" '
            f'contype="4" conaffinity="3" friction="0.5 0.005 0.0001"/>')
    bx, by = s.handle_r, dn * s.handle_out
    pull = 'contype="4" conaffinity="11" condim="4" priority="1" ' \
           f'friction="{s.bar_mu} 0.01 0.0002" rgba="0.62 0.63 0.65 1" solref="0.004 1" solimp="0.95 0.99 0.001"'
    door += (f'<geom name="pull_bar" type="capsule" size="{_f(s.bar_d / 2, s.bar_centres / 2 + 0.012)}" '
             f'pos="{_f(bx, by, s.handle_z)}" mass="0.08" {pull}/>')
    for k, dz in enumerate((-s.bar_centres / 2, s.bar_centres / 2)):
        py = dn * (s.thick + s.handle_off / 2)
        door += (f'<geom name="pull_post{k}" type="cylinder" size="{_f(s.post_d / 2, s.handle_off / 2)}" '
                 f'pos="{_f(bx, py, s.handle_z + dz)}" quat="0.7071068 0.7071068 0 0" mass="0.01" {pull}/>')
    if s.tag_id >= 0:                          # the tag's print on the door's face, its x right seen from the room
        w = s.tag_size / 0.8
        r = math.radians(s.tag_roll_deg)       # x right seen from the room (-dn, 0, 0), y up, z out (0, dn, 0)
        x_tag = (-dn * math.cos(r), 0.0, math.sin(r))
        y_tag = (dn * math.sin(r), 0.0, math.cos(r))
        door += (f'<geom name="door_tag" type="box" class="decor" material="door_tag" '
                 f'size="{_f(w / 2, w / 2, 0.0003)}" pos="{_f(s.tag_along, dn * (s.thick + 0.0003), s.tag_z)}" '
                 f'xyaxes="{_f(*x_tag, *y_tag)}" contype="0" conaffinity="0" mass="0"/>')
    jaxis = _f(0, 0, s.sense)
    body = (f'<body name="door" pos="{_f(*h, 0)}" quat="{_f(math.cos(yaw_u / 2), 0, 0, math.sin(yaw_u / 2))}">'
            f'<joint name="door_hinge" type="hinge" axis="{jaxis}" '
            f'range="0 {math.radians(1.0 if s.locked else s.open_max_deg):.6g}" '
            f'damping="{s.damping}" frictionloss="{s.friction}" armature="0.001" '
            f'solreflimit="0.004 1" solimplimit="0.95 0.99 0.001"/>{door}</body>')
    out.append(body)
    out.append(f'<camera name="door_view" pos="{_f(*(h + n * 1.6 + u * 1.3), 1.45)}" mode="targetbody" target="door" '
               f'fovy="50"/>')
    out.append(f'<camera name="door_top" pos="{_f(*(h + n * 0.55 - u * 0.15), 2.6)}" xyaxes="{_top_axes(u, n)}" fovy="55"/>')
    return "".join(out)


def _top_axes(u, n) -> str:
    """A camera looking straight down, the cabinet at the image's top."""
    x = -np.array([u[0], u[1], 0.0])
    if n[0] * u[1] - n[1] * u[0] < 0:
        x = -x
    y = np.array([n[0], n[1], 0.0]) * -1
    y = -y if np.cross(x, y)[2] < 0 else y
    return _f(*x, *y)


TAG_PX = 10                                    # texture pixels per tag cell (10 x 10 cells with the margin)


def with_cabinet(scene_xml: str, s: DoorSpec) -> str:
    """scene_xml with the cabinet. With a tag, call paint_tag(model, spec) before rendering."""
    out = scene_xml.replace("</worldbody>", xml(s) + "</worldbody>", 1)
    if s.tag_id >= 0:
        n = 10 * TAG_PX
        out = out.replace("</asset>", f'<texture name="door_tag" type="2d" builtin="flat" width="{n}" height="{n}" '
                          f'rgb1="1 1 1" rgb2="1 1 1"/><material name="door_tag" texture="door_tag" '
                          f'specular="0.1"/></asset>', 1)
    return out


def paint_tag(m, s: DoorSpec):
    """Paint the door's tag texture with the real 36h11 code of s.tag_id (cv2.aruco's table, studies/data), as
    imx662_cam.real_tags does the house's. Before a Renderer is made: it uploads the textures."""
    if s.tag_id < 0:
        return
    from pathlib import Path
    codes = np.load(Path(__file__).resolve().parent / "studies" / "data" / "tag36h11_codes.npz")["codes"]
    i = m.texture("door_tag").id
    w, h, ch = m.tex_width[i], m.tex_height[i], m.tex_nchannel[i]
    img = m.tex_data[m.tex_adr[i]:m.tex_adr[i] + w * h * ch].reshape(h, w, ch)
    c = TAG_PX
    img[:] = 255
    # turned 180 deg: the texture's rows run up the box's face (checked against the detector's pose, Nov 21)
    code = np.rot90(codes[s.tag_id], 2)
    cell = np.kron(np.where(code, 255, 0).astype(np.uint8), np.ones((c, c), np.uint8))
    img[c:9 * c, c:9 * c, :] = cell[:, :, None]


def tag_record(s: DoorSpec) -> dict:
    """The door's map entry as measured on the door from its tag (omni_arm.door.DoorTag's fields, exact). The tag's
    x (right seen from the room) is -dn along the door from the hinge."""
    dn = 1.0 if s.sense > 0 else -1.0
    return dict(tag_id=s.tag_id, size=s.tag_size, hinge_x=dn * s.tag_along, pull_dy=s.handle_z - s.tag_z,
                pull_x=-dn * (s.handle_r - s.tag_along),
                door_t=s.thick, bar_off=s.handle_off, bar_d=s.bar_d, open_max=math.radians(s.open_max_deg))


def tag_truth(m, d) -> np.ndarray:
    """The tag's pose in the world (4 x 4: x right seen from the room, y up, z out of the door's face)."""
    g = m.geom("door_tag").id
    T = np.eye(4)
    T[:3, :3] = d.geom_xmat[g].reshape(3, 3)
    T[:3, 3] = d.geom_xpos[g]
    return T


class Hinge:
    """The hinges' self-closing torque, applied each physics step (qfrc_applied on the door's joint)."""

    def __init__(self, m, d, s: DoorSpec):
        self.d, self.s = d, s
        j = m.joint("door_hinge")
        self.qadr, self.dof = j.qposadr[0], j.dofadr[0]

    @property
    def angle(self) -> float:
        return float(self.d.qpos[self.qadr])

    def apply(self):
        self.d.qfrc_applied[self.dof] = -self.s.closing_torque(self.angle)
