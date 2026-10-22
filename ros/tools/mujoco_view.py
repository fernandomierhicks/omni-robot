"""MuJoCo's 3D view of the ROS sim, drawn on Windows (design.md 28.6).

The sim runs in WSL. Drawing it there is slow: WSLg translates MuJoCo's many small OpenGL calls to Direct3D 12 across
the VM (~0.8 s a frame for the house), or renders on the CPU the sim needs. So this window runs on Windows with native
OpenGL. It builds the same model from the same sim folder, and the bridge (omni_sim bridge_node) streams the state
(qpos) and the last lidar scan's hit points over localhost TCP at ~30 Hz.

  & $py ros\\tools\\mujoco_view.py                          (the bridge also starts it by itself when viewer:=true)
  & $py ros\\tools\\mujoco_view.py --sim "simulation\\v10 ros bridge" --port 9871

The camera follows the robot; drag to orbit, scroll to zoom, double-click a body to track it. Red dots = the lidar.
"""

from __future__ import annotations

import argparse
import json
import socket
import struct
import sys
import time
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
HDR = struct.Struct("<4sIII")          # magic b"OMNI", seq, nq, n points


def recv_exact(s, n):
    buf = bytearray()
    while len(buf) < n:
        chunk = s.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("bridge closed")
        buf += chunk
    return bytes(buf)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sim", default="simulation/v11 sil", help="the sim folder (relative to the repo or absolute)")
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--port", type=int, default=9871)
    a = ap.parse_args()
    sim = Path(a.sim) if Path(a.sim).is_absolute() else REPO / a.sim
    sys.path.insert(0, str(sim))
    sys.path.insert(0, str(sim / "studies"))
    import mujoco
    import mujoco.viewer

    while True:                                           # wait for the bridge
        try:
            s = socket.create_connection((a.host, a.port), timeout=2.0)
            break
        except OSError:
            print(f"waiting for the bridge on {a.host}:{a.port} ...", flush=True)
            time.sleep(1.0)
    s.settimeout(None)
    f = s.makefile("rb")
    hello = json.loads(f.readline())
    print("bridge:", hello, flush=True)
    from robot_model import build_model
    if hello.get("scene") == "catch":                     # v11: the catch game's field (catch_world.py)
        from catch_world import AREA, HOME
        from home_floor import GAME_FLOORS
        from robot_export import v11_params
        m = build_model(v11_params(home_xy=HOME, thrower_xy=(HOME[0] - hello["dist"] - 0.3, 0.0), area_size=AREA,
                                   **GAME_FLOORS[hello.get("floor", "flat")]))
    else:
        import house
        m = build_model(house.house_params(hello["base"], wheel_model=hello["wheel_model"]))
    if m.nq != hello["nq"]:
        sys.exit(f"model mismatch: this sim folder has nq {m.nq}, the bridge's {hello['nq']} (sync the sim folders)")
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    v = mujoco.viewer.launch_passive(m, d, show_left_ui=False, show_right_ui=False)
    with v.lock():
        v.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
        v.cam.trackbodyid = m.body("base").id
        v.cam.distance, v.cam.elevation, v.cam.azimuth = 3.5, -35.0, 200.0
        v.opt.flags[mujoco.mjtVisFlag.mjVIS_RANGEFINDER] = False
    eye = np.eye(3).flatten()
    size = np.array([0.012, 0.0, 0.0])
    red = np.array([1.0, 0.1, 0.1, 0.9])
    pts = np.zeros((0, 3))
    try:
        while v.is_running():
            magic, seq, nq, n = HDR.unpack(f.read(HDR.size))
            if magic != b"OMNI":
                raise ConnectionError("bad frame")
            q = np.frombuffer(f.read(8 * nq), dtype=np.float64)
            if n:
                pts = np.frombuffer(f.read(12 * n), dtype=np.float32).reshape(n, 3)
            with v.lock():
                d.qpos[:] = q
                mujoco.mj_kinematics(m, d)
                scn = v.user_scn
                scn.ngeom = 0
                for p in pts[::2]:
                    if scn.ngeom >= scn.maxgeom:
                        break
                    mujoco.mjv_initGeom(scn.geoms[scn.ngeom], mujoco.mjtGeom.mjGEOM_SPHERE, size, p, eye, red)
                    scn.ngeom += 1
            v.sync()
    except (ConnectionError, OSError, struct.error) as e:
        print("bridge gone:", e, flush=True)
    finally:
        s.close()
        if v.is_running():
            v.close()


if __name__ == "__main__":
    main()
