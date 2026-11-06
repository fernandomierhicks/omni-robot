"""Floor toys on the arm robot's route (design.md 4.5): does Nav2 drive over them, push them, and leave stale marks?

Against a running arm step (the house, objects:=toys, n_objects 6) on the PC; the bridge's control port places the toys
(OMNI_CONTROL_PORT, sim_world.World.place_toys). One run:

  1  the route's path: --path (a JSON of world x, y the robot drove), or one lap of the eight spots with the toys put
     away, recorded from /ground_truth/odom and written there
  2  the toys on that path (seeded): --on of them on it (within 5 cm of the line), the rest 0.30-0.40 m beside it; each
     >= 0.8 m from a goal spot and 0.7 m from the hallway's doorways, >= 1.0 m apart, with the map's walls >= 0.55 m
     away (on the path: room to go round) or >= 0.25 m (beside it)
  3  ROUTE: --laps laps of the eight spots (arm_load.py's Go Tos): per goal reached, time, true error, localization
     error; per toy the robot's closest approach (centre to toy), how often its outline went over it (< OVERRUN m: on
     the robot a wheel or the bumper meets it; in the sim only the bumper, 50 mm up, can touch a toy, and only a 40 mm
     one when the base pitches), how far it moved, and whether the global costmap marks it (cost >= MARKED) at the end
  4  TIDY: drive 0.8 m short of the nearest toy still on the floor (truth), the console's tidy, until none is left
     (a toy given up on after two legs): picks, toys in the bin (truth), and marks left in the global costmap where a
     binned toy lay (stale)

  python3 toy_route.py OUT.json --seed 0 --path PATH.json [--laps 2] [--on 4] [--map ~/omni_ws_claude/maps/house]

Writes OUT.json: the placements, every goal and tidy leg, per toy the scores, and a summary.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import socket
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
import yaml
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

sys.path.insert(0, str(Path(__file__).resolve().parent))
from arm_load import ArmDriver   # noqa: E402
from soak import SPOTS           # noqa: E402

OVERRUN = 0.225          # m: the bumper's outer radius (robot_export: deck 217 + 8)
MOVED = 0.03             # m: a toy displaced
MARKED = 99              # the global costmap's OccupancyGrid: 99 inscribed, 100 lethal
FALSE_R = 0.3            # m: a floor-grid mark this far from every toy and wall is a false one (ToyDriver.false_marks)
SLOW = 0.8               # m/s: a speed limit under this counts as slowing the robot (depth_layer:=height)
DOORS = ((-3.0, 2.0), (0.45, 2.0), (3.45, 2.0))      # sim_world.DOORS (the hallway's)
STAND_OFF = 0.8


class ToyDriver(ArmDriver):
    def __init__(self):
        super().__init__()
        self.track = []            # per toy: min distance, overrun passes, inside now
        self.path = []             # recorded truth while recording
        self.recording = False
        self.costmap = self.floor_grid = None
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, "/global_costmap/costmap", lambda m: setattr(self, "costmap", m), qos)
        # omni_arm's floor_obstacles grid (depth_layer:=floor): which marks are the camera's, not the lidar's
        self.create_subscription(OccupancyGrid, "/floor_obstacles/grid", lambda m: setattr(self, "floor_grid", m), qos)
        self.map = None
        self.create_subscription(OccupancyGrid, "/map", lambda m: setattr(self, "map", m), qos)
        self.false_max = 0
        # depth_layer:=height: its speed limit over the route (samples of /floor_obstacles/state, 2 Hz)
        self.limits = None                 # None: not counting; else {"n": samples, "limited": n, "why": {reason: n}}
        self.create_subscription(String, "/floor_obstacles/state", self.on_state, 5)

    def on_state(self, m):
        if self.limits is None:
            return
        try:
            st = json.loads(m.data)
        except ValueError:
            return
        self.limits["n"] += 1
        lim = st.get("speed_limit", 0)
        if 0 < lim < SLOW:
            self.limits["slow"] = self.limits.get("slow", 0) + 1
        if lim > 0:
            self.limits["limited"] += 1
            why = st.get("why", "").split(" ")[0]
            self.limits["why"][why] = self.limits["why"].get(why, 0) + 1

    def false_marks(self) -> int | None:
        """The floor grid's marked cells more than FALSE_R from every toy and from the map's walls: in the bare sim
        house (no props) nothing else is on the floor, so each is a false mark (None: no grid / map yet)."""
        g, m = self.floor_grid, self.map
        if g is None or m is None or (g.info.width, g.info.height) != (m.info.width, m.info.height):
            return None
        from scipy.ndimage import distance_transform_edt
        res, ox, oy = g.info.resolution, g.info.origin.position.x, g.info.origin.position.y
        marked = np.asarray(g.data, np.int16).reshape(g.info.height, g.info.width) >= 100
        walls = np.asarray(m.data, np.int16).reshape(m.info.height, m.info.width) >= 65
        ok = marked & (distance_transform_edt(~walls) * res > FALSE_R)
        i, j = np.nonzero(ok)
        if not len(i):
            return 0
        x, y = ox + (j + 0.5) * res, oy + (i + 0.5) * res
        for p in (self.objects or {}).get("positions") or []:
            mx, my, _ = self.to_map(p[0], p[1], 0.0)
            keep = np.hypot(x - mx, y - my) > FALSE_R
            x, y = x[keep], y[keep]
        return int(len(x))

    def on_truth(self, m):
        super().on_truth(m)
        x, y = self.truth[:2]
        if self.recording and (not self.path or math.hypot(x - self.path[-1][0], y - self.path[-1][1]) > 0.02):
            self.path.append((round(x, 3), round(y, 3)))
        P = (self.objects or {}).get("positions") or []
        for k, t in enumerate(self.track):
            if k >= len(P) or t is None:
                continue
            d = math.hypot(P[k][0] - x, P[k][1] - y)
            t["min_d"] = min(t["min_d"], d)
            inside = d < OVERRUN
            if inside and not t["inside"]:
                t["overruns"] += 1
            t["inside"] = inside

    def cost_at(self, wx, wy, r=0.05, grid=None):
        """The global costmap's (or grid's) highest value within r of a world point (-1: none yet / off the map)."""
        m = self.costmap if grid is None else grid
        if m is None or self.origin is None:
            return -1
        mx, my, _ = self.to_map(wx, wy, 0.0)
        res, ox, oy = m.info.resolution, m.info.origin.position.x, m.info.origin.position.y
        W, H = m.info.width, m.info.height
        g = np.asarray(m.data, np.int16).reshape(H, W)
        n = int(math.ceil(r / res))
        i, j = int((my - oy) / res), int((mx - ox) / res)
        win = g[max(0, i - n):min(H, i + n + 1), max(0, j - n):min(W, j + n + 1)]
        return int(win.max()) if win.size else -1


def control(req: dict, port: int) -> dict:
    with socket.create_connection(("127.0.0.1", port), timeout=10) as s:
        s.sendall((json.dumps(req) + "\n").encode())
        buf = b""
        while b"\n" not in buf:
            chunk = s.recv(4096)
            if not chunk:
                break
            buf += chunk
    return json.loads(buf.split(b"\n")[0] or b"{}")


class Clearance:
    """Distance (m) from a world point to the nearest occupied cell of the saved map (map frame = n.to_map)."""

    def __init__(self, base: str, to_map):
        from scipy.ndimage import distance_transform_edt
        info = yaml.safe_load(open(base + ".yaml"))
        raw = Path(base + ".pgm").read_bytes()
        parts, k = [], 0
        while len(parts) < 4:                          # P5, width, height, maxval (no comments in the saver's)
            while raw[k:k + 1].isspace():
                k += 1
            e = k
            while not raw[e:e + 1].isspace():
                e += 1
            parts.append(raw[k:e])
            k = e
        w, h = int(parts[1]), int(parts[2])
        img = np.frombuffer(raw[k + 1:k + 1 + w * h], np.uint8).reshape(h, w)[::-1]       # row 0 = the map's bottom
        occ = (255 - img.astype(float)) / 255.0 > info["occupied_thresh"]
        self.d = distance_transform_edt(~occ) * info["resolution"]
        self.res, self.org = info["resolution"], info["origin"]
        self.to_map = to_map

    def __call__(self, wx, wy) -> float:
        mx, my, _ = self.to_map(wx, wy, 0.0)
        j, i = int((mx - self.org[0]) / self.res), int((my - self.org[1]) / self.res)
        if not (0 <= i < self.d.shape[0] and 0 <= j < self.d.shape[1]):
            return 0.0
        return float(self.d[i, j])


def placements(path, n: int, n_on: int, seed: int, clear, avoid=(), on_clear: float = 0.55) -> list:
    """(x, y, yaw deg, kind) for n toys along the path (module notes); avoid: (x, y) already taken (>= 1 m away)."""
    P = np.asarray(path, float)
    seg = np.hypot(*np.diff(P, axis=0).T)
    s = np.r_[0.0, np.cumsum(seg)]
    rng = np.random.default_rng(seed + 4200)
    out = []
    for _ in range(20000):
        if len(out) == n:
            break
        on = len(out) < n_on
        a = rng.uniform(0.0, s[-1])
        k = min(int(np.searchsorted(s, a)) - 1, len(seg) - 1)
        k = max(k, 0)
        u = (P[k + 1] - P[k]) / max(seg[k], 1e-9)
        f = (a - s[k]) / max(seg[k], 1e-9)
        c = P[k] + f * (P[k + 1] - P[k])
        off = rng.uniform(-0.05, 0.05) if on else rng.choice([-1, 1]) * rng.uniform(0.30, 0.40)
        x, y = c + off * np.array([-u[1], u[0]])
        if any(math.hypot(x - sx, y - sy) < 0.8 for sx, sy, _ in SPOTS):
            continue
        if any(math.hypot(x - dx, y - dy) < 0.7 for dx, dy in DOORS):
            continue
        if any(math.hypot(x - q[0], y - q[1]) < 1.0 for q in list(out) + list(avoid)):
            continue
        if clear(x, y) < (on_clear if on else 0.25):
            continue
        out.append((round(float(x), 3), round(float(y), 3), round(float(rng.uniform(0, 180)), 1), "on" if on else "beside"))
    return out


def record_path(n: ToyDriver, port: int) -> list:
    control({"toys": []}, port)                       # every toy put away
    n.spin_for(2.0)
    n.cmd("mission", {"name": "MANUAL"})
    n.spin_for(2.0)
    n.path, n.recording = [], True
    for i, (x, y, yaw) in enumerate(SPOTS):
        g = n.goto(x, y, yaw, f"path {i + 1}")
        print(f"  path spot {i + 1}: {'ok' if g['reached'] else 'NO'} {g['time_s']} s", flush=True)
    n.recording = False
    return n.path


def route(n: ToyDriver, laps: int) -> list:
    goals = []
    for lap in range(laps):
        for i, (x, y, yaw) in enumerate(SPOTS):
            g = n.goto(x, y, yaw, f"spot {i + 1}")
            g["lap"] = lap + 1
            g["false_marks"] = n.false_marks()
            n.false_max = max(n.false_max, g["false_marks"] or 0)
            goals.append(g)
            print(f"  lap {lap + 1} spot {i + 1}: {'ok ' if g['reached'] else 'NO '} {g['time_s']:5.1f} s, "
                  f"err {g['err_m']:.2f} m, loc {g['loc_err_m']}, vmax {g['vmax']:.2f}  ({g['task']})", flush=True)
    return goals


def tidy(n: ToyDriver, timeout: float, budget: float) -> list:
    legs, tries = [], {}
    t_end = time.monotonic() + budget
    while time.monotonic() < t_end:
        n.cmd("mission", {"name": "MANUAL"})
        n.spin_for(2.0)
        objs = n.objects or {}
        left = [(k, p) for k, (p, inb) in enumerate(zip(objs.get("positions", []), objs.get("in_bin", [])))
                if not inb and tries.get(k, 0) < 2 and math.hypot(p[0], p[1]) < 20]
        if not left or n.truth is None:
            break
        rx, ry = n.truth[:2]
        k, (ox, oy, _) = min(left, key=lambda kp: math.hypot(kp[1][0] - rx, kp[1][1] - ry))
        tries[k] = tries.get(k, 0) + 1
        d = math.hypot(ox - rx, oy - ry)
        ux, uy = (ox - rx) / max(d, 1e-6), (oy - ry) / max(d, 1e-6)
        before = [list(p) for p in objs.get("positions", [])]
        bin0 = objs.get("n_in_bin", 0)
        g = n.goto(ox - STAND_OFF * ux, oy - STAND_OFF * uy, math.degrees(math.atan2(uy, ux)), f"toy {k}")
        rec = {"toy": k, "object": [ox, oy], "drive": g, "bin_before": bin0}
        if g["reached"]:
            t0 = time.monotonic()
            ok, msg, _ = n.cmd("tidy")
            until = time.monotonic() + 5.0
            while ok and time.monotonic() < until and not (n.arm or {}).get("tidy_on"):
                n.spin_for(0.1)
            while time.monotonic() - t0 < timeout and (n.arm or {}).get("tidy_on"):
                n.spin_for(0.5)
            if (n.arm or {}).get("tidy_on"):
                n.cmd("mission", {"name": "MANUAL"})
                until = time.monotonic() + 30.0
                while (n.arm or {}).get("tidy_on") and time.monotonic() < until:
                    n.spin_for(0.2)
                rec["timed_out"] = True
            n.spin_for(5.0)                              # the bin's truth settles; the costmap's 1 Hz catches up
            t = (n.arm or {}).get("tidy") or {}
            rec.update(tidy_s=round(time.monotonic() - t0, 1), picked=t.get("picked"), failed=t.get("failed"),
                       reason=t.get("reason"))
        after = n.objects or {}
        rec["bin_after"] = after.get("n_in_bin")
        binned = [j for j, inb in enumerate(after.get("in_bin", [])) if inb and j < len(before)
                  and not (objs.get("in_bin") or [False] * 99)[j]]
        rec["binned"] = binned
        rec["stale"] = {j: n.cost_at(before[j][0], before[j][1]) for j in binned}
        if n.floor_grid is not None:                     # 100: the camera's own mark (not the lidar's)
            rec["stale_grid"] = {j: n.cost_at(before[j][0], before[j][1], grid=n.floor_grid) for j in binned}
        legs.append(rec)
        print(f"  toy {k}: drive {'ok' if g['reached'] else 'NO'} {g['time_s']} s; tidy {rec.get('picked')} picked "
              f"{rec.get('failed')} failed ({rec.get('reason')}) in {rec.get('tidy_s')} s; bin {bin0} -> "
              f"{rec['bin_after']}; costs where binned ones lay {rec['stale']}", flush=True)
        if (n.arm or {}).get("tidy_on"):
            break
    return legs


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("out")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--path", required=True, help="the route's path JSON: read, or recorded and written")
    ap.add_argument("--laps", type=int, default=2)
    ap.add_argument("--on", type=int, default=4, help="toys on the path; the rest beside it")
    ap.add_argument("--shoes", type=int, default=0, help="shoes (11 cm: under the lidar's plane) on the path, 0-2")
    ap.add_argument("--map", default=os.path.expanduser("~/omni_ws_claude/maps/house"))
    ap.add_argument("--port", type=int, default=int(os.environ.get("OMNI_CONTROL_PORT", 9873)))
    ap.add_argument("--tidy", type=int, default=1, help="1: the tidy phase, 0: route only")
    ap.add_argument("--tidy-timeout", type=float, default=600.0)
    ap.add_argument("--tidy-budget", type=float, default=2400.0)
    ap.add_argument("--wait", type=float, default=300.0)
    a = ap.parse_args()
    rclpy.init()
    n = ToyDriver()
    t0 = time.monotonic()
    ready = lambda: (n.status and n.origin and n.cli.service_is_ready() and (n.status.get("caps") or {}).get("nav")  # noqa: E731
                     and (n.status.get("pose") or {}).get("frame") == "map" and (n.arm or {}).get("connected")
                     and n.objects)
    while time.monotonic() - t0 < a.wait and not ready():
        n.spin_for(1.0)
    res = {"started": time.strftime("%Y-%m-%d %H:%M:%S"), "seed": a.seed, "ready": bool(ready()),
           "ready_after_s": round(time.monotonic() - t0, 1)}
    if not res["ready"]:
        print("not ready", flush=True)
    else:
        if Path(a.path).exists():
            path = json.loads(Path(a.path).read_text())["path"]
        else:
            print("recording the route's path ...", flush=True)
            path = record_path(n, a.port)
            Path(a.path).write_text(json.dumps({"path": path, "recorded": time.strftime("%Y-%m-%d %H:%M:%S")}))
        clear = Clearance(a.map, n.to_map)
        n_toys = len((n.objects or {}).get("names") or []) or 6
        for attempt in range(5):           # a toy that can't rest there is parked, and /sim/objects leaves it out
            spots = placements(path, n_toys, a.on, a.seed + 1000 * attempt, clear)
            shoes = placements(path, a.shoes, a.shoes, a.seed + 7000 + 1000 * attempt, clear,
                               avoid=[s[:2] for s in spots], on_clear=0.7) if a.shoes else []
            rep = control({"toys": [s[:3] for s in spots],
                           "obstacles": [[3 + k % 2, *s[:3]] for k, s in enumerate(shoes)]}, a.port)
            if all(rep.get("placed", [])) and len(rep.get("placed", [])) == n_toys                     and all(rep.get("placed_obstacles", [])):
                break
        res["placement_attempts"] = attempt + 1
        n.spin_for(3.0)
        P0 = [list(p) for p in (n.objects or {}).get("positions", [])]
        names = (n.objects or {}).get("names") or []
        print(f"toys: {rep.get('message')}: " + ", ".join(f"{nm} {k} ({s[0]}, {s[1]})"
                                                          for nm, s, k in zip(names, spots, (s[3] for s in spots))),
              flush=True)
        n.track = [{"min_d": math.inf, "overruns": 0, "inside": False} if ok else None
                   for ok in list(rep.get("placed", [])) + list(rep.get("placed_obstacles", []))]
        if shoes:
            print("shoes: " + ", ".join(f"({s[0]}, {s[1]})" for s in shoes), flush=True)
        n.cmd("mission", {"name": "MANUAL"})
        n.spin_for(5.0)
        marked0 = [n.cost_at(p[0], p[1]) for p in P0]
        print("ROUTE ...", flush=True)
        t_r = time.monotonic()
        n.limits = {"n": 0, "limited": 0, "why": {}}
        goals = route(n, a.laps)
        limits, n.limits = n.limits, None
        route_s = round(time.monotonic() - t_r, 1)
        n.spin_for(3.0)
        P1 = [list(p) for p in (n.objects or {}).get("positions", [])]
        toys = []
        for k, (nm, s) in enumerate(zip(names, spots)):
            if k >= len(n.track) or n.track[k] is None:
                toys.append({"name": nm, "placed": False})
                continue
            moved = math.hypot(P1[k][0] - P0[k][0], P1[k][1] - P0[k][1])
            toys.append({"name": nm, "kind": s[3], "at": P0[k][:2], "min_d": round(n.track[k]["min_d"], 3),
                         "overruns": n.track[k]["overruns"], "moved_m": round(moved, 3),
                         "cost_before": marked0[k], "cost_after_route": n.cost_at(P1[k][0], P1[k][1])})
            print(f"  {nm:8s} {s[3]:6s} closest {toys[-1]['min_d']:.2f} m, over it {toys[-1]['overruns']}x, "
                  f"moved {moved:.3f} m, cost {marked0[k]} -> {toys[-1]['cost_after_route']}", flush=True)
        obst = []
        for k, s in enumerate(shoes):
            j = len(spots) + k
            if j >= len(n.track) or n.track[j] is None or j >= len(P1):
                continue
            moved = math.hypot(P1[j][0] - P0[j][0], P1[j][1] - P0[j][1])
            obst.append({"name": names[j] if j < len(names) else "shoe", "at": P0[j][:2],
                         "min_d": round(n.track[j]["min_d"], 3), "moved_m": round(moved, 3),
                         "cost_after_route": n.cost_at(P1[j][0], P1[j][1])})
            print(f"  shoe     on     closest {obst[-1]['min_d']:.2f} m, moved {moved:.3f} m, cost after "
                  f"{obst[-1]['cost_after_route']}", flush=True)
        res.update(path_len_m=round(float(np.sum(np.hypot(*np.diff(np.asarray(path), axis=0).T))), 1),
                   placements=spots, shoe_placements=shoes, toys=toys, obstacles=obst, goals=goals,
                   route_s=route_s, speed_limits=limits)
        legs = []
        if a.tidy:
            print("TIDY ...", flush=True)
            legs = tidy(n, a.tidy_timeout, a.tidy_budget)
        res["tidy"] = legs
        ok = [g for g in goals if g["reached"]]
        placed = [t for t in toys if t.get("placed", True)]
        stale = [c for leg in legs for c in leg.get("stale", {}).values()]
        res["summary"] = {
            "goals": f"{len(ok)}/{len(goals)}",
            "route_s": route_s,
            "loc_err_max_m": max((g["loc_err_m"] or 0) for g in goals) if goals else None,
            "err_mean_m": round(sum(g["err_m"] for g in ok) / len(ok), 3) if ok else None,
            "toys": len(placed),
            "overrun": sum(t["overruns"] > 0 for t in placed),
            "overrun_on": sum(t["overruns"] > 0 for t in placed if t["kind"] == "on"),
            "moved": sum(t["moved_m"] > MOVED for t in placed),
            "marked_after_route": sum(t["cost_after_route"] >= MARKED for t in placed),
            "false_marks_max": n.false_max if n.floor_grid is not None else None,
            "shoes": len(obst), "shoes_moved": sum(o["moved_m"] > MOVED for o in obst),
            "shoes_closest_m": min((o["min_d"] for o in obst), default=None),
            "limited_share": round(limits["limited"] / limits["n"], 2) if limits and limits["n"] else None,
            "slow_share": round(limits.get("slow", 0) / limits["n"], 2) if limits and limits["n"] else None,
            "in_bin": (n.objects or {}).get("n_in_bin"),
            "tidy_legs": len(legs),
            "stale_marks": sum(c >= MARKED for c in stale), "binned_checked": len(stale),
            "stale_grid_marks": sum(c >= 100 for leg in legs for c in leg.get("stale_grid", {}).values()),
        }
        print("summary", json.dumps(res["summary"]), flush=True)
    n.cmd("mission", {"name": "IDLE"})
    Path(a.out).parent.mkdir(parents=True, exist_ok=True)
    Path(a.out).write_text(json.dumps(res, indent=1, default=str))
    print("wrote", a.out, flush=True)
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
