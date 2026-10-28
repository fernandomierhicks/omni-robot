"""MuJoCo's 3D view of the ROS sim, drawn on Windows (design.md 28.6).

The sim runs in WSL. Drawing it there is slow: WSLg translates MuJoCo's many small OpenGL calls to Direct3D 12 across
the VM (~0.8 s a frame for the house), or renders on the CPU the sim needs. So this window runs on Windows with native
OpenGL. It builds the same model from the same sim folder, and the bridge (omni_sim bridge_node) streams the state
(qpos) and the last lidar scan's hit points over localhost TCP at ~30 Hz.

  & $py ros\\tools\\mujoco_view.py                          (the bridge also starts it by itself when viewer:=true)
  & $py ros\\tools\\mujoco_view.py --sim "simulation\\v10 ros bridge" --port 9871

The camera follows the robot; drag to orbit, scroll to zoom, double-click a body to track it. Red dots = the lidar.
The Launcher's camera buttons switch the view (UDP 9872, localhost): follow, chase, free, top, robot (set_view).
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
SETTINGS = REPO / "logs" / "controller.json"     # the Launcher's choices (the view this window opens with)
VIEWS = ("follow", "chase", "free", "top", "robot")


def saved_view() -> str:
    try:
        v = json.loads(SETTINGS.read_text()).get("view", "follow")
    except (OSError, ValueError, AttributeError):
        return "follow"
    return v if v in VIEWS else "follow"


def control_socket(port: int):
    """The Launcher's camera buttons: UDP datagrams {"view": name} on localhost (None if the port is taken)."""
    if not port:
        return None
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.bind(("127.0.0.1", port))
    except OSError:
        print(f"camera buttons: UDP {port} is taken (another 3D view?)", flush=True)
        return None
    s.setblocking(False)
    return s


def read_control(s) -> str | None:
    """The last view asked for since the last frame, or None."""
    want = None
    while s is not None:
        try:
            msg = json.loads(s.recv(512))
        except (BlockingIOError, OSError):
            break
        except ValueError:
            continue
        if isinstance(msg, dict) and msg.get("view") in VIEWS:
            want = msg["view"]
    return want


def set_view(v, m, d, base: int, name: str):
    """follow: orbiting behind the robot; chase: low behind it, turning with it; free: the mouse moves it; top: the
    whole house from above; robot: the robot's own (left stereo) camera."""
    import mujoco
    cam = v.cam
    if name == "robot":
        ids = [k for k in range(m.ncam) if m.camera(k).name.startswith("stereo_")]
        left = [k for k in ids if "left" in m.camera(k).name] or ids
        if left:
            cam.type, cam.fixedcamid = mujoco.mjtCamera.mjCAMERA_FIXED, left[0]
            return
        name = "follow"
    if name == "top":
        cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        cam.lookat[:] = (0.0, 0.0, 0.0)
        cam.distance, cam.elevation, cam.azimuth = 13.0, -89.0, 90.0
    elif name == "free":
        cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        cam.lookat[:] = d.xpos[base]
    elif name == "chase":
        cam.type, cam.trackbodyid = mujoco.mjtCamera.mjCAMERA_TRACKING, base
        cam.distance, cam.elevation = 2.2, -18.0
    else:
        cam.type, cam.trackbodyid = mujoco.mjtCamera.mjCAMERA_TRACKING, base
        cam.distance, cam.elevation, cam.azimuth = 3.5, -35.0, 200.0


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
    ap.add_argument("--control", type=int, default=9872, help="UDP port for the Launcher's camera buttons (0 = none)")
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
    elif "core" in hello:                                 # v11: the bridge's model settings (drawn robot, arm, person)
        from bridge_core import CoreConfig, model_params
        m = build_model(model_params(CoreConfig(**hello["core"])))
    else:
        import house
        m = build_model(house.house_params(hello["base"], wheel_model=hello["wheel_model"]))
    if m.nq != hello["nq"]:
        sys.exit(f"model mismatch: this sim folder has nq {m.nq}, the bridge's {hello['nq']} (sync the sim folders)")
    nmocap = int(hello.get("nmocap", 0))
    if nmocap and nmocap != m.nmocap:
        sys.exit(f"model mismatch: {m.nmocap} mocap bodies here, the bridge's {nmocap}")
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    v = mujoco.viewer.launch_passive(m, d, show_left_ui=False, show_right_ui=False)
    base = m.body("base").id
    view = saved_view()
    with v.lock():
        set_view(v, m, d, base, view)
        v.opt.flags[mujoco.mjtVisFlag.mjVIS_RANGEFINDER] = False
    ctl = control_socket(a.control)
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
            if nmocap:
                mp = np.frombuffer(f.read(24 * nmocap), dtype=np.float64).reshape(nmocap, 3)
                mq = np.frombuffer(f.read(32 * nmocap), dtype=np.float64).reshape(nmocap, 4)
            want = read_control(ctl)
            with v.lock():
                d.qpos[:] = q
                if nmocap:
                    d.mocap_pos[:], d.mocap_quat[:] = mp, mq
                mujoco.mj_kinematics(m, d)
                mujoco.mj_comPos(m, d)                    # subtree_com: what the tracking (follow, chase) camera follows
                if want and want != view:
                    view = want
                    set_view(v, m, d, base, view)
                if view == "chase":                       # behind the robot, looking where it heads
                    R = d.xmat[base].reshape(3, 3)
                    v.cam.azimuth = float(np.degrees(np.arctan2(R[1, 0], R[0, 0])))
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
        if ctl is not None:
            ctl.close()
        if v.is_running():
            v.close()


if __name__ == "__main__":
    main()
