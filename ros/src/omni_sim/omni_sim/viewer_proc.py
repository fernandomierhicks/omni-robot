"""MuJoCo's 3D view of the bridge's sim (design.md 28.6).

  WinViewer   (default) streams the state (qpos) and the last scan's hit points over TCP to ros/tools/mujoco_view.py,
              which the bridge starts on Windows through WSL's interop. Windows draws it with native OpenGL on the
              GPU. In WSL, the house took ~0.8 s a frame through WSLg's Direct3D 12 translation (MuJoCo makes many small
              legacy GL calls) and ~0.09 s on the CPU (llvmpipe), which also took the cores the sim needs.
  ViewerLink  (viewer_mode:=wsl) the viewer in its own WSL process, fed through shared memory. In the bridge's own
              process the viewer's render thread competed with the physics for Python's lock (~25% of real time).
"""

from __future__ import annotations

import json
import math
import multiprocessing as mp
import os
import socket
import struct
import subprocess
import sys
import time

import numpy as np

MAX_PTS = 1600
HDR = struct.Struct("<4sIII")          # magic b"OMNI", seq, nq, n points (then qpos float64, points float32 xyz)


class WinViewer:
    """A TCP server for ros/tools/mujoco_view.py on Windows, and (with OMNI_WIN_PY etc. set by ros/omni.sh) the
    viewer process itself, started through WSL's Windows interop."""

    def __init__(self, hello: dict, port: int = 9871, log=None):
        self.hello, self.log = hello, log
        self.srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.srv.bind(("0.0.0.0", port))
        self.srv.listen(1)
        self.srv.setblocking(False)
        self.conn, self.seq, self.proc = None, 0, None
        py, script, sim = (os.environ.get(k) for k in ("OMNI_WIN_PY", "OMNI_WIN_VIEWER", "OMNI_WIN_SIM"))
        if py and script and sim and os.path.exists(py):
            out = open("/tmp/mujoco_view.log", "w")
            self.proc = subprocess.Popen([py, "-u", script, "--sim", sim, "--port", str(port)], stdout=out,
                                         stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL)
            self._log(f"MuJoCo view starting on Windows (log /tmp/mujoco_view.log)")
        else:
            self._log(f"MuJoCo view: run ros/tools/mujoco_view.py on Windows (it connects to port {port})")

    def _log(self, msg):
        if self.log:
            self.log(msg)

    def running(self) -> bool:
        return True

    def push(self, qpos, pts=None):
        if self.conn is None:
            try:
                self.conn, _ = self.srv.accept()
                self.conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                self.conn.settimeout(0.5)          # a stalled viewer is dropped, never the sim
                self.conn.sendall((json.dumps(self.hello) + "\n").encode())
                self._log("MuJoCo view connected")
                self._pts = None
            except BlockingIOError:
                return
        if pts is None and self.seq == 0:
            pts = np.zeros((0, 3))
        p = np.zeros((0, 3), np.float32) if pts is None else np.asarray(pts[:MAX_PTS], np.float32)
        self.seq += 1
        msg = HDR.pack(b"OMNI", self.seq, len(qpos), len(p)) + np.asarray(qpos, np.float64).tobytes() + p.tobytes()
        try:
            self.conn.sendall(msg)
        except OSError:
            self._log("MuJoCo view closed; the sim keeps running")
            self.conn.close()
            self.conn = None

    def close(self):
        if self.conn is not None:
            self.conn.close()
        self.srv.close()
        if self.proc is not None and self.proc.poll() is None:
            self.proc.terminate()


class ViewerLink:
    """The bridge's end: start the viewer process, push the state now and then."""

    def __init__(self, sim_dir: str, base: str, wheel_model: str, nq: int, base_body: int):
        ctx = mp.get_context("spawn")
        self.qpos = ctx.Array("d", nq, lock=False)
        self.pts = ctx.Array("d", 3 * MAX_PTS, lock=False)
        self.n_pts = ctx.Value("i", 0, lock=False)
        self.seq = ctx.Value("i", 0, lock=False)
        self.alive = ctx.Value("i", 1, lock=False)
        self.proc = ctx.Process(target=_run, args=(sim_dir, base, wheel_model, base_body, self.qpos, self.pts,
                                                   self.n_pts, self.seq, self.alive), daemon=True)
        self.proc.start()
        self._q = np.frombuffer(self.qpos, dtype=np.float64)
        self._p = np.frombuffer(self.pts, dtype=np.float64)

    def running(self) -> bool:
        return self.proc.is_alive() and bool(self.alive.value)

    def push(self, qpos, pts=None):
        self._q[:] = qpos
        if pts is not None:
            n = min(len(pts), MAX_PTS)
            self._p[:3 * n] = np.asarray(pts[:n], dtype=np.float64).ravel()
            self.n_pts.value = n
        self.seq.value += 1

    def close(self):
        self.alive.value = 0
        self.proc.join(timeout=2.0)
        if self.proc.is_alive():
            self.proc.terminate()


def _run(sim_dir, base, wheel_model, base_body, qpos, pts, n_pts, seq, alive):
    sys.path.insert(0, sim_dir)
    sys.path.insert(0, sim_dir + "/studies")
    import mujoco
    import mujoco.viewer

    import house
    from robot_model import build_model
    m = build_model(house.house_params(base, wheel_model=wheel_model))
    d = mujoco.MjData(m)
    q = np.frombuffer(qpos, dtype=np.float64)
    P = np.frombuffer(pts, dtype=np.float64)
    d.qpos[:] = q
    mujoco.mj_forward(m, d)
    v = mujoco.viewer.launch_passive(m, d, show_left_ui=False, show_right_ui=False)
    with v.lock():
        v.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
        v.cam.trackbodyid = base_body
        v.cam.distance, v.cam.elevation, v.cam.azimuth = 3.5, -35.0, 200.0
        v.opt.flags[mujoco.mjtVisFlag.mjVIS_RANGEFINDER] = False
    eye = np.eye(3).flatten()
    size = np.array([0.012, 0.0, 0.0])
    red = np.array([1.0, 0.1, 0.1, 0.9])
    last = -1
    while v.is_running() and alive.value:
        if seq.value == last:
            time.sleep(0.005)
            continue
        last = seq.value
        with v.lock():
            d.qpos[:] = q
            mujoco.mj_kinematics(m, d)
            scn = v.user_scn
            scn.ngeom = 0
            n = n_pts.value
            for k in range(0, n, 2):
                if scn.ngeom >= scn.maxgeom:
                    break
                mujoco.mjv_initGeom(scn.geoms[scn.ngeom], mujoco.mjtGeom.mjGEOM_SPHERE, size,
                                    P[3 * k:3 * k + 3], eye, red)
                scn.ngeom += 1
        v.sync()
    alive.value = 0
    v.close()
