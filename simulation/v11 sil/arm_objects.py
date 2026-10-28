"""Loose things on the floor for the arm to pick up (design.md 11, Oct 24): params.floor_objects.

Each object is a free body: kind "box" (size = full x, y, z), "cylinder" (lying along its x: size = (length, d, d)),
"puck" (a cylinder standing on its end: size = (d, d, h)), "sphere" (size = (d, d, d)), at (x, y) on the floor in the
world frame, turned yaw deg. Contacts: the floor, walls,
the robot's structure (bumper, bin) and each other (bit 1), and the arm's finger pads (bit 8: conaffinity 9). The
wheels' rollers only touch the floor, so the robot could drive through an object below the bumper (bottom ~50 mm):
the pick task stops short of them. Stiff contacts (solref 4 ms) so a grasped toy doesn't sink into the pads.

SETS are the object sets the pick studies and the ROS world use: small toys and household clutter, 12-60 mm across
(the gripper opens to ~84 mm, its fingers are 15 mm wide), 6-60 g (est, printed or moulded plastic).
"""

from __future__ import annotations

import math

import numpy as np

KINDS = ("box", "cylinder", "puck", "sphere")
CONTYPE, CONAFFINITY = 1, 9
PARKED_X = 30.0                   # m: an object this far out is parked (sim_world.World): no contacts, no weight

# name: (kind, size xyz m, rgba, kg)
CATALOG = {
    "cube30": ("box", (0.03, 0.03, 0.03), (0.85, 0.15, 0.12, 1), 0.020),
    "cube40": ("box", (0.04, 0.04, 0.04), (0.12, 0.35, 0.85, 1), 0.035),
    "brick": ("box", (0.048, 0.024, 0.015), (0.95, 0.75, 0.1, 1), 0.012),       # a large building brick
    "crayon": ("cylinder", (0.09, 0.012, 0.012), (0.2, 0.7, 0.25, 1), 0.008),
    "marker": ("cylinder", (0.13, 0.018, 0.018), (0.1, 0.1, 0.1, 1), 0.015),
    "ball40": ("sphere", (0.04, 0.04, 0.04), (0.95, 0.45, 0.1, 1), 0.025),
    "cap": ("puck", (0.03, 0.03, 0.012), (0.9, 0.9, 0.95, 1), 0.004),          # a bottle cap
    "block": ("box", (0.06, 0.03, 0.03), (0.55, 0.2, 0.7, 1), 0.040),
    # larger things left on a floor (the Launcher's clutter, sim_world.py; est sizes and masses): the boxes, bag,
    # basket and stool reach the lidar's 15 cm plane, the shoes don't (the bumper finds them), the play ball rolls
    "box_big": ("box", (0.55, 0.40, 0.40), (0.72, 0.56, 0.38, 1), 3.0),       # a moving box
    "box_small": ("box", (0.40, 0.30, 0.30), (0.78, 0.62, 0.42, 1), 1.5),
    "bag": ("box", (0.42, 0.22, 0.34), (0.15, 0.22, 0.32, 1), 2.0),            # a backpack
    "shoe": ("box", (0.28, 0.10, 0.11), (0.3, 0.2, 0.15, 1), 0.4),
    "basket": ("puck", (0.45, 0.45, 0.35), (0.85, 0.85, 0.8, 1), 1.5),         # a laundry basket
    "stool": ("puck", (0.32, 0.32, 0.45), (0.45, 0.3, 0.2, 1), 3.0),
    "ball_big": ("sphere", (0.22, 0.22, 0.22), (0.9, 0.2, 0.25, 1), 0.3),      # a play ball
}
SETS = {
    "toys": ("cube30", "cube40", "brick", "crayon", "ball40", "block"),
    "clutter": ("marker", "cap", "brick", "cube30", "crayon", "cube40"),
    "obstacles": ("box_big", "box_small", "bag", "shoe", "shoe", "basket", "stool", "ball_big"),
}


def objects_for(names, xy, yaw_deg) -> tuple:
    """params.floor_objects entries for catalog names at positions (world) and yaws."""
    out = []
    for n, (x, y), a in zip(names, xy, yaw_deg):
        kind, size, rgba, kg = CATALOG[n]
        out.append((kind, size, float(x), float(y), float(a), rgba, kg))
    return tuple(out)


def scatter(names, centre, radius: float, seed: int = 0, min_gap: float = 0.25, keep_out=None) -> tuple:
    """Catalog objects scattered in a disc around centre (world), at least min_gap apart (and outside keep_out =
    ((x, y), r))."""
    rng = np.random.default_rng(seed)
    pts = []
    while len(pts) < len(names):
        r, a = radius * math.sqrt(rng.random()), rng.uniform(0, 2 * math.pi)
        p = (centre[0] + r * math.cos(a), centre[1] + r * math.sin(a))
        if any(math.hypot(p[0] - q[0], p[1] - q[1]) < min_gap for q in pts):
            continue
        if keep_out is not None and math.hypot(p[0] - keep_out[0][0], p[1] - keep_out[0][1]) < keep_out[1]:
            continue
        pts.append(p)
    return objects_for(names, pts, rng.uniform(0, 180, len(names)))


def height(o) -> float:
    kind, size = o[0], o[1]
    return size[2] if kind in ("box", "puck") else size[1]


def world_xml(p) -> str:
    out = []
    for k, (kind, size, x, y, yaw, rgba, kg) in enumerate(p.floor_objects):
        if kind not in KINDS:
            raise ValueError(f"floor object {k}: kind {kind!r}")
        h = height((kind, size))
        a = math.radians(yaw) / 2
        if kind == "box":
            geom = f'type="box" size="{size[0] / 2:.5g} {size[1] / 2:.5g} {size[2] / 2:.5g}"'
        elif kind == "cylinder":                        # lying: its axis along the body's x
            geom = f'type="cylinder" size="{size[1] / 2:.5g} {size[0] / 2:.5g}" quat="0.7071068 0 0.7071068 0"'
        elif kind == "puck":
            geom = f'type="cylinder" size="{size[0] / 2:.5g} {size[2] / 2:.5g}"'
        else:
            geom = f'type="sphere" size="{size[0] / 2:.5g}"'
        inertia = kg * (max(size) ** 2) / 10
        parked = x >= PARKED_X           # floats where it is, touching nothing, until sim_world puts it in the house
        ct, ca = (0, 0) if parked else (CONTYPE, CONAFFINITY)
        gc = ' gravcomp="1"' if parked else ""
        out.append(f'<body name="obj{k}" pos="{x:.5g} {y:.5g} {h / 2 + 0.0005:.5g}" quat="{math.cos(a):.7g} 0 0 {math.sin(a):.7g}"{gc}>'
                   f'<freejoint name="obj{k}"/><inertial pos="0 0 0" mass="{kg:.5g}" diaginertia="{inertia:.3g} {inertia:.3g} {inertia:.3g}"/>'
                   f'<geom name="obj{k}" {geom} contype="{ct}" conaffinity="{ca}" condim="4" friction="0.8 0.01 0.0002" '
                   f'solref="0.004 1" solimp="0.95 0.99 0.001" density="0" rgba="{" ".join(f"{c:.3g}" for c in rgba)}"/></body>')
    return "".join(out)


def positions(m, d) -> np.ndarray:
    """(n, 3) world positions of the floor objects (the sim's truth: scoring only)."""
    out = []
    k = 0
    while True:
        try:
            out.append(d.xpos[m.body(f"obj{k}").id].copy())
        except KeyError:
            break
        k += 1
    return np.array(out).reshape(-1, 3)
