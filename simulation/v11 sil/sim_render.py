"""The ROS bridge's cameras rendered on Windows (Nov 26). In WSL MuJoCo renders through Mesa's llvmpipe, on the CPU: a
D435 depth frame took 23 ms, its segmentation 16 ms (78 ms before sil_pick kept its segment table), a lit colour or
wrist frame ~150 ms (shadows and the floor's reflection), together about a third of the bridge's time in the full
stack. Windows renders them on the GPU in a few ms (WSLg's D3D12 path was slower still: MuJoCo's legacy GL).

Only the raw renders move (sil_pick.PickCameras.raw: depth, albedo, wrist, colour); the noise and its random stream
stay in the bridge, so a frame is the same draw for draw wherever it was rendered, and the bridge falls back to
rendering itself whenever no server is connected (or one fails).

  bridge (WSL)    RenderLink(hello, port): listens; starts this file on Windows through WSL's interop (OMNI_WIN_PY,
                  OMNI_WIN_SIM from ros/omni.sh); once the server has built the same model and cameras (checked: nq and
                  the depth camera's calibration error) it serves PickCameras.remote
  server (Win)    python sim_render.py --port P [--host localhost]: connects, builds the bridge's model (bridge_core
                  model_params of the hello's CoreConfig) and PickCameras with the same seed, answers raw() requests

Port: OMNI_RENDER_PORT (default 9881), a second stack beside Fernando's uses its own (AGENTS.md).
"""

from __future__ import annotations

import argparse
import json
import os
import socket
import struct
import subprocess
import sys
import threading
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
REQ = struct.Struct("<4sBI")            # b"RNDQ", kind index, nq; then qpos float64
RESP = struct.Struct("<4sBIII")         # b"RNDA", dtype index, h, w, channels; then the array
KINDS = ("depth", "albedo", "wrist", "color")
DTYPES = (np.float32, np.float64, np.uint8)
PORT = int(os.environ.get("OMNI_RENDER_PORT", 9881))


def _recv(s, n: int) -> bytes:
    buf = bytearray()
    while len(buf) < n:
        chunk = s.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("render link closed")
        buf += chunk
    return bytes(buf)


def _line(s) -> dict:
    buf = bytearray()
    while not buf.endswith(b"\n"):
        chunk = s.recv(1)
        if not chunk:
            raise ConnectionError("render link closed")
        buf += chunk
    return json.loads(buf)


class RenderLink:
    """The bridge's end. ready once a server has connected and matched; raw(kind, qpos) -> the server's render."""

    def __init__(self, hello: dict, check: dict, port: int = PORT, log=None, start: bool = True):
        self.hello, self.check, self.log = hello, check, log or (lambda m: None)
        self.srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.srv.bind(("0.0.0.0", port))
        self.srv.listen(1)
        self.conn, self.ready, self.proc, self.failed = None, False, None, None
        self.n, self.ms = 0, 0.0
        py, sim = os.environ.get("OMNI_WIN_PY"), os.environ.get("OMNI_WIN_SIM")
        if start and py and sim and os.path.exists(py):
            out = open(f"/tmp/sim_render_{port}.log", "w")
            script = sim.rstrip("\\") + "\\sim_render.py"
            self.proc = subprocess.Popen([py, "-u", script, "--port", str(port)], stdout=out, stderr=subprocess.STDOUT,
                                         stdin=subprocess.DEVNULL)
            self.log(f"cameras: Windows render server starting (port {port}, log /tmp/sim_render_{port}.log); "
                     f"rendering here until it is up")
        elif start:
            self.log(f"cameras: rendered here (no Windows interop; run sim_render.py --port {port} on Windows)")
        threading.Thread(target=self._accept, daemon=True).start()

    def _accept(self):
        try:
            conn, _ = self.srv.accept()
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            conn.settimeout(180.0)                      # the model build on Windows
            conn.sendall((json.dumps(self.hello) + "\n").encode())
            reply = _line(conn)
            bad = [k for k, v in self.check.items() if not np.allclose(np.asarray(reply.get(k), float), v, atol=1e-9)]
            if not reply.get("ok") or bad:
                self.failed = f"server's model differs ({', '.join(bad) or reply.get('error')})"
                self.log(f"cameras: Windows render server refused: {self.failed}; rendering here")
                conn.close()
                return
            conn.settimeout(10.0)
            self.conn, self.ready = conn, True
            self.log(f"cameras: rendered on Windows ({reply.get('gl', '?')})")
        except Exception as e:                          # noqa: BLE001 (a missing server just means local renders)
            self.failed = str(e)
            self.log(f"cameras: Windows render server not available ({e}); rendering here")

    def raw(self, kind: str, qpos) -> np.ndarray:
        if not self.ready:
            raise ValueError("render link not ready")
        w = time.perf_counter()
        q = np.ascontiguousarray(qpos, np.float64)
        try:
            self.conn.sendall(REQ.pack(b"RNDQ", KINDS.index(kind), len(q)) + q.tobytes())
            magic, dt, h, wd, ch = RESP.unpack(_recv(self.conn, RESP.size))
            if magic != b"RNDA":
                raise ConnectionError("bad render reply")
            dtype = np.dtype(DTYPES[dt])
            shape = (h, wd) if ch == 0 else (h, wd, ch)
            arr = np.frombuffer(_recv(self.conn, int(np.prod(shape)) * dtype.itemsize), dtype).reshape(shape).copy()
        except (OSError, ConnectionError) as e:
            self.ready = False
            raise OSError(f"render link: {e}") from e
        self.n += 1
        self.ms += 1000 * (time.perf_counter() - w)
        return arr

    def close(self):
        self.ready = False
        for s in (self.conn, self.srv):
            try:
                if s is not None:
                    s.close()
            except OSError:
                pass
        if self.proc is not None and self.proc.poll() is None:
            self.proc.terminate()


def camera_check(cams) -> dict:
    """What the server must match: the model's size and the depth camera's calibration error (drawn from the seed)."""
    return dict(nq=float(cams.m.nq), cam_quat=[float(x) for x in cams.mr.cam_quat[cams.cl]])


def serve(host: str, port: int):
    sys.path.insert(0, str(HERE))
    import mujoco
    from bridge_core import CoreConfig, model_params
    from robot_model import build_model
    from sil_pick import PickCameras
    while True:
        try:
            s = socket.create_connection((host, port), timeout=2.0)
            break
        except OSError:
            print(f"waiting for the bridge on {host}:{port} ...", flush=True)
            time.sleep(1.0)
    s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    s.settimeout(None)
    hello = _line(s)
    print("bridge:", hello, flush=True)
    try:
        c = hello["core"]
        if c.get("home_doors"):
            c["home_doors"] = tuple(tuple(x) for x in c["home_doors"])
        p = model_params(CoreConfig(**c))
        m = build_model(p)
        d = mujoco.MjData(m)
        mujoco.mj_forward(m, d)
        cams = PickCameras(m, d, p, seed=hello["seed"], noise=hello["noise"])
        cams.raw("depth")                                # the renderers' first frame (context, textures)
        try:
            from OpenGL import GL
            gl = GL.glGetString(GL.GL_RENDERER).decode()
        except Exception:                               # noqa: BLE001
            gl = "native GL"
        s.sendall((json.dumps(dict(ok=True, gl=gl, **camera_check(cams))) + "\n").encode())
    except Exception as e:                              # noqa: BLE001
        s.sendall((json.dumps(dict(ok=False, error=repr(e))) + "\n").encode())
        raise
    print(f"serving {gl}", flush=True)
    n, t0 = 0, time.perf_counter()
    while True:
        magic, k, nq = REQ.unpack(_recv(s, REQ.size))
        if magic != b"RNDQ":
            raise ConnectionError("bad request")
        q = np.frombuffer(_recv(s, 8 * nq), np.float64)
        a = np.ascontiguousarray(cams.raw(KINDS[k], q))
        dt = next(i for i, t in enumerate(DTYPES) if a.dtype == t)
        s.sendall(RESP.pack(b"RNDA", dt, a.shape[0], a.shape[1], a.shape[2] if a.ndim == 3 else 0) + a.tobytes())
        n += 1
        if n % 500 == 0:
            print(f"{n} frames, {n / (time.perf_counter() - t0):.1f} a second", flush=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--port", type=int, default=PORT)
    a = ap.parse_args()
    serve(a.host, a.port)


if __name__ == "__main__":
    main()
