"""The course world (Nov 26): a compact made-up floor that stresses Nav2 and Explore, so a change can be checked in
minutes instead of a 40 min mapping of the home. world:=course in the ROS steps (the arm step: Explore, Go To).

What it holds (16 x 9 m inside its outer walls, ~135 m2 of floor; walls 1.0 m tall, 0.1 m thick, est):
  hall          a straight hall 16 m long, 1.2 m wide, along the middle: top speed and braking (2 m/s needs ~1 m to
                reach and ~1 m to stop at 2 m/s^2)
  start room    3.5 x 3.2 m, a 1.6 m opening to the hall; the robot starts in it facing east (the map frame)
  dead ends     a 1.0 m corridor 3.2 m deep, and a 0.66 m one 2.0 m deep (the robot is 0.48 m across: in, then out
                backwards)
  bathroom      behind a 0.61 m (24 in) door, as the home's bathroom 3; vanity, toilet, tub
  ramp room     raised 68 mm like the home's family room: a 12.7 mm square step, then a 20 deg ramp over 6 in, in a
                0.86 m doorway off the hall (the home's hall entry, where runs above 0.5 m/s stalled, Nov 26)
  back room     two doors to the hall (a back way) round a bed
  spiral        a square spiral of 1.0-1.1 m corridors, ~1.75 turns, ending in a 0.8 m pocket (a dead end that has
                to be backed out of)
  loop room     an island (a full-height block) with corridors round it and two doors to the hall: two loops
  open room     a 2 m opening; a table with chairs, a sofa, a cabinet, two stools 0.55 m apart, a thin lamp pole

Everything is est (made up for the test, not a real place). The course is drawn in local coordinates (0..16, 0..9)
and placed with its middle at the world's origin (L()).

  python course.py               MuJoCo's viewer on the course (no robot code)
  python course.py --top out.png a top view (renders/)
  python course.py --map         ros/maps/course.{pgm,yaml}, course_origin.json, course.places.json (map frame = START)
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path

HERE = Path(__file__).resolve().parent

SIZE = (16.0, 9.0)                       # the course's outer walls, local x 0..16, y 0..9
AREA = (17.0, 10.0)                      # SimParams.area_size: the sim's own boundary walls clear of the course
WALL_H = 1.0                             # m (est): above the lidar and the D435's view of a wall nearby
T = 0.1                                  # wall thickness
RAMP_H = 0.0127 + 0.1524 * math.tan(math.radians(20.0))    # the home's family room: 68 mm


def L(x: float, y: float) -> tuple:
    """Local course coordinates -> world (the course's middle at the origin)."""
    return x - SIZE[0] / 2, y - SIZE[1] / 2


# walls (local x0, y0, x1, y1): outer, the hall's two sides with their openings, the rooms' partitions
WALLS = (
    (0.0, 0.0, 16.0, T), (0.0, 9.0 - T, 16.0, 9.0), (0.0, 0.0, T, 9.0), (16.0 - T, 0.0, 16.0, 9.0),
    # the hall's south side (y 3.3..3.4): openings to the start room 1.0..2.6, dead end 3.7..4.7, narrow dead end
    # 4.8..5.46, bathroom door 6.0..6.61, ramp room door 9.0..9.86, back room doors 12.0..12.9 and 14.8..15.7
    *((x0, 3.3, x1, 3.4) for x0, x1 in ((T, 1.0), (2.6, 3.7), (4.7, 4.8), (5.46, 6.0), (6.61, 9.0), (9.86, 12.0),
                                         (12.9, 14.8), (15.7, 16.0 - T))),
    # the hall's north side (y 4.6..4.7): openings to the spiral 0.1..1.1, the loop room 5.3..6.2 and 9.4..10.3, the
    # open room 11.0..13.0
    *((x0, 4.6, x1, 4.7) for x0, x1 in ((1.1, 5.3), (6.2, 9.4), (10.3, 11.0), (13.0, 16.0 - T))),
    # south rooms' partitions
    (3.6, T, 3.7, 3.3), (4.7, T, 4.8, 3.3), (5.46, T, 5.56, 3.3), (8.4, T, 8.5, 3.3), (11.6, T, 11.7, 3.3),
    (4.8, 1.2, 5.46, 1.3),                                # the narrow dead end's end
    # north rooms' partitions
    (4.9, 4.7, 5.0, 9.0 - T), (10.5, 4.7, 10.6, 9.0 - T),
    # the spiral (x 0.1..4.9, y 4.7..8.9): in at the hall, north, east along the top, south, west along the bottom,
    # north again, east into the pocket
    (1.1, 4.7, 1.2, 7.9), (1.2, 7.8, 3.9, 7.9), (3.8, 5.8, 3.9, 7.8), (2.3, 5.7, 3.8, 5.8), (2.3, 5.8, 2.4, 7.0),
)
ISLAND = (6.3, 5.9, 9.2, 7.8)            # the loop room's island (full height)

ROOMS = {                                # local, inside the walls: the console's room names
    "hall": (0.1, 3.4, 15.9, 4.6), "start room": (0.1, 0.1, 3.6, 3.3), "dead end": (3.7, 0.1, 4.7, 3.3),
    "narrow dead end": (4.8, 1.3, 5.46, 3.3), "bathroom": (5.56, 0.1, 8.4, 3.3), "ramp room": (8.5, 0.1, 11.6, 3.3),
    "back room": (11.7, 0.1, 15.9, 3.3), "spiral": (0.1, 4.7, 4.9, 8.9), "loop room": (5.0, 4.7, 10.5, 8.9),
    "open room": (10.6, 4.7, 15.9, 8.9),
}
START_L = (1.8, 1.6, 0.0)
START = (*L(*START_L[:2]), 0.0)          # the start room, facing east (x, y, yaw deg): the map frame's origin
ARM_AREA = (L(0.8, 2.5), 0.35)           # the arm world's things: the start room's north-west corner
THROWER_PARK = (30.0, -30.0)             # out of every sensor's sight
PLACES_L = {                             # the console's named places (local x, y, yaw deg)
    "start": START_L, "hall west": (0.8, 4.0, 0.0), "hall east": (15.2, 4.0, 180.0),
    "dead end": (4.2, 0.8, -90.0), "narrow end": (5.13, 1.9, -90.0), "bathroom": (7.0, 1.5, -90.0),
    "ramp room": (10.0, 1.6, -90.0), "back room": (13.3, 2.7, 0.0), "spiral end": (3.2, 7.4, 0.0),
    "loop north": (7.7, 8.35, 0.0), "loop south": (7.7, 5.3, 0.0), "table": (11.5, 6.85, 0.0),
    "sofa": (14.7, 7.4, 90.0),
}
PLACES = {k: (*L(x, y), yaw) for k, (x, y, yaw) in PLACES_L.items()}
SPOTS = PLACES


def room_at(xy) -> str | None:
    for name, (x0, y0, x1, y1) in ROOMS.items():
        a, b = L(x0, y0)
        c, d = L(x1, y1)
        if a <= xy[0] <= c and b <= xy[1] <= d:
            return name
    return None


def floor_features() -> tuple:
    """SimParams.floor_features: the ramp room's raised floor and the ramp in its doorway (the foot in the hall, rising
    south to the room's side of the wall, as the home's family room entries)."""
    from floor import Platform, Ramp
    wood = (0.72, 0.55, 0.36, 1)
    x0, y0 = L(8.5, T)
    x1, y1 = L(11.6, 3.3)
    fx, fy = L((9.0 + 9.86) / 2, 3.3 + 0.1524)
    return (Platform(x0, y0, x1, y1, thickness=RAMP_H, mu=0.9, crr=0.02, softness=0.003, rgba=wood),
            Ramp(fx, fy, -90.0, length=0.86, step=0.0127, run=0.1524, height=RAMP_H, mu=0.9, rgba=wood))


def floor_z(xy) -> float:
    x0, y0 = L(8.5, T)
    x1, y1 = L(11.6, 3.3)
    return RAMP_H if x0 <= xy[0] <= x1 and y0 <= xy[1] <= y1 else 0.0


def door_centres() -> list:
    """The middle of every opening off the hall (for keeping clutter out of them)."""
    south = ((1.0, 2.6), (3.7, 4.7), (4.8, 5.46), (6.0, 6.61), (9.0, 9.86), (12.0, 12.9), (14.8, 15.7))
    north = ((0.1, 1.1), (5.3, 6.2), (9.4, 10.3), (11.0, 13.0))
    return [L((a + b) / 2, 3.35) for a, b in south] + [L((a + b) / 2, 4.65) for a, b in north]


CLUTTER_ROOMS = ("start room", "back room", "open room", "loop room")


def worldbody_xml(p=None) -> str:
    from home import _G
    G = _G()

    def box(x0, y0, x1, y1, z0, z1, mat, kind="prop_course"):
        a, b = L(x0, y0)
        c, d = L(x1, y1)
        G.box((a, b, z0), (c, d, z1), mat, kind=kind)

    for x0, y0, x1, y1 in WALLS:
        box(x0, y0, x1, y1, 0.0, WALL_H, "hm_wall", kind="course_wall")
    box(*ISLAND, 0.0, WALL_H, "hm_cabinet", kind="course_wall")
    # start room: a coffee table in the far corner
    a, b = L(0.4, 0.4)
    c, d = L(1.1, 0.9)
    G.table(a, b, c, d, 0.45)
    # bathroom: vanity, toilet, tub (the door is at x 6.0..6.61 on the hall side)
    box(5.6, 0.1, 6.6, 0.65, 0.0, 0.85, "hm_cabinet")
    G.toilet(*L(7.9, 0.45), 90.0)
    box(6.9, 2.6, 8.4, 3.3, 0.0, 0.55, "hm_porcelain")
    # ramp room (raised): an armchair and a side table, set on its floor
    G.dz = RAMP_H
    G.sofa(*L(10.4, 0.2), *L(11.5, 1.1), "e", mat="hm_fabric2")
    a, b = L(8.7, 0.4)
    c, d = L(9.2, 0.9)
    G.table(a, b, c, d, 0.55, leg=0.02)
    G.dz = 0.0
    # back room: a bed against the south wall between the two doors
    G.bed(*L(12.6, 0.1), *L(14.4, 2.0), "s")
    # open room: a table and four chairs, a sofa on the north wall, a cabinet, two stools 0.55 m apart (between their
    # legs), a floor lamp's thin pole
    a, b = L(12.2, 6.4)
    c, d = L(13.8, 7.3)
    G.table(a, b, c, d, 0.75)
    for x, y, f in ((12.6, 6.0, 90.0), (13.4, 6.0, 90.0), (12.6, 7.7, -90.0), (13.4, 7.7, -90.0)):
        G.chair(*L(x, y), f)
    G.sofa(*L(13.6, 8.0), *L(15.85, 8.85), "n")
    box(15.0, 4.75, 15.85, 5.6, 0.0, 0.85, "hm_wood")
    for x in (11.0, 11.81):
        G.stool(*L(x, 5.6))
    G.cyl((*L(14.6, 6.1), 0.0), (*L(14.6, 6.1), 1.5), 0.015, "hm_steel")
    return G.body()


def assets_xml(p=None) -> str:
    import home
    return home.assets_xml(p)


def write_map(out_dir: Path, name: str = "course", base: str = "drawn", res: float = 0.05):
    """<name>.pgm / .yaml, <name>_origin.json, <name>.places.json in the map frame (the robot's START), as
    home.write_map: the course's own map for localizer:=amcl and its named places (a saved Explore session is what
    slam_toolbox's localization needs)."""
    import json
    import mujoco
    import numpy as np
    from scipy import ndimage as ndi
    from bridge_core import CoreConfig, model_params
    from home import occupancy
    from robot_model import build_model
    m = build_model(model_params(CoreConfig(base=base, world="course", payload="arm", arm_camera="d435")))
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    z = float(d.site_xpos[m.site("lidar").id][2])
    occ, x0, y0 = occupancy(m, d, res, z, area=AREA)
    sx, sy, _ = START
    lab, _ = ndi.label(~occ)
    inside = lab == lab[int((sy - y0) / res), int((sx - x0) / res)]
    img = np.full(occ.shape, 205, dtype=np.uint8)
    img[inside] = 254
    img[occ] = 0
    out_dir.mkdir(parents=True, exist_ok=True)
    h, w = img.shape
    with open(out_dir / f"{name}.pgm", "wb") as f:
        f.write(f"P5\n{w} {h}\n255\n".encode())
        f.write(img[::-1].tobytes())
    (out_dir / f"{name}.yaml").write_text(
        f"image: {name}.pgm\nmode: trinary\nresolution: {res:.3f}\norigin: [{x0 - sx:.3f}, {y0 - sy:.3f}, 0]\n"
        f"negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n")
    (out_dir / f"{name}_origin.json").write_text(json.dumps(dict(world_from_map=[sx, sy, 0.0])))

    def to_map(x, y):
        return [round(x - sx, 3), round(y - sy, 3)]

    rooms = {}
    for r, (a, b, c, e) in ROOMS.items():
        (a, b), (c, e) = L(a, b), L(c, e)
        rooms[r] = [to_map(a, b), to_map(c, b), to_map(c, e), to_map(a, e)]
    places = {k: [*to_map(x, y), round(math.radians(yaw), 4)] for k, (x, y, yaw) in PLACES.items()}
    from floor import Ramp
    edges = {}
    for f in floor_features():
        if isinstance(f, Ramp):
            c_, s_ = math.cos(math.radians(f.yaw_deg)), math.sin(math.radians(f.yaw_deg))
            hl = f.length / 2
            edges["ramp room ramp"] = dict(points=[to_map(f.x + s_ * hl, f.y - c_ * hl), to_map(f.x - s_ * hl, f.y + c_ * hl)],
                                           height_mm=round(f.step * 1000, 1), speed=1.0, kind="step")
    (out_dir / f"{name}.places.json").write_text(json.dumps(dict(rooms=rooms, places=places, zones={}, edges=edges,
                                                                 routes={}), indent=1))
    print(f"{name}: {w} x {h} cells at {res} m (lidar plane {z:.3f} m), {inside.sum() * res * res:.0f} m2 free; "
          f"{len(rooms)} rooms, {len(places)} places -> {out_dir}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--top", metavar="PNG", help="render a top view to renders/PNG")
    ap.add_argument("--map", action="store_true", help="write ros/maps/course.* (map, origin, places)")
    a = ap.parse_args()
    if a.map:
        write_map(HERE.parents[1] / "ros" / "maps")
        return
    import mujoco
    from bridge_core import CoreConfig, model_params
    from robot_model import build_model
    m = build_model(model_params(CoreConfig(base="drawn", world="course", payload="arm", arm_camera="d435")))
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    print(f"course: {m.ngeom} geoms")
    if a.top:
        m.vis.global_.offwidth, m.vis.global_.offheight = 1800, 1100
        m.vis.global_.orthographic = 1
        m.vis.global_.fovy = 10.0
        r = mujoco.Renderer(m, 1100, 1800)
        cam = mujoco.MjvCamera()
        cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        cam.lookat[:] = (0.0, 0.0, 0.0)
        cam.distance, cam.azimuth, cam.elevation = 30.0, 90.0, -90.0
        r.update_scene(d, cam, mujoco.MjvOption())
        out = HERE / "renders" / a.top
        out.parent.mkdir(exist_ok=True)
        from PIL import Image
        Image.fromarray(r.render()).save(out)
        print(f"-> {out}")
        return
    import mujoco.viewer
    mujoco.viewer.launch(m, d)


if __name__ == "__main__":
    main()
