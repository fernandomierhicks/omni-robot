"""Shared pieces of the RealSense D435 characterization (README.md).

Two halves: the camera helpers (need pyrealsense2 and a camera) and the pure maths the scripts share
(plane fits, interval and latency statistics, the stereo error model, flash edges, ballistic fits),
which `test_realsense.py` checks without a camera.

What the design asks of a camera (design.md 4.1, 4.9 and section 8 phase 1C;
`simulation/v11 sil/studies/compute_choice.py` CAMERA_REQ) is in REQ below; every script reports
against it.
"""
from __future__ import annotations

import datetime
import json
import os
import pathlib
import platform
import time

import numpy as np

try:
    import pyrealsense2 as rs
except ImportError:          # the pure maths and the tests run without it
    rs = None

HERE = pathlib.Path(__file__).resolve().parent
RESULTS = pathlib.Path(os.environ["RS_RESULTS"]) if os.environ.get("RS_RESULTS") else HERE / "results"
RECORDINGS = pathlib.Path.home() / "realsense_recordings"     # ~110 MB/s at 90 fps: kept out of Dropbox

# design.md: CAMERA_REQ in compute_choice.py, plus phase 1C's pass bands (drift <~ 0.1 deg, latency <~ 60 ms)
REQ = dict(global_shutter=True, color=True, fps_min=60, res_min=(640, 400), hw_sync=True,
           latency_max_ms=60.0, calib_drift_deg=0.1, calib_drift_good_deg=0.04)
BALL_D_M = 0.074                 # the design's foam ball (design.md 1)
THROW_RANGE_M = (6.0, 7.0)       # thrower distance (design.md 1)
SIM_BASELINE_M = 0.190           # the design's stereo bar (design.md 2)
SIM_F_PX = 400 / np.tan(np.radians(35.0))   # 1280x800 with the sim's 70 deg vertical FOV: ~571 px
G = 9.81

# D435 facts the camera can't report about itself (Intel RealSense D400 series datasheet; VERIFY on the unit)
D435_DATASHEET = dict(
    depth_sensor="OmniVision OV9282 (x2), global shutter, 3 um pixels, 1280x800, mono (IR + visible)",
    rgb_sensor="OmniVision OV2740, rolling shutter, 1.4 um pixels, 1920x1080",
    baseline_mm=50.0,
    depth_fov_deg=(87, 58),
    rgb_fov_deg=(69, 42),
    min_z_m_848x480=0.195,       # datasheet min-Z at max resolution 0.28 m; 848x480 ~0.195 m (VERIFY)
    ground_zero_mm=4.2,          # depth origin behind the front cover glass: Intel's "Depth Cameras Depth Start Point" table, D435 / D435i -4.2 mm (was 3.2 est)
    depth_origin_from_tripod_mm=17.5,   # the left imager, from the centreline of the 1/4-20 hole, which is the middle of the lens group (Intel, Figure 4-6 / Table 4-15)
    lens_layout_mm="from the left imager (depth origin), toward the camera's right: projector +29, right imager +50; the RGB camera -15 (Intel Figure 4-6; the camera's own calibration agrees: RGB 15 mm, IR baseline 49.85 mm)",
    sync_connector="9-pin JST SH on the back; sync pin 1.8 V (inter_cam_sync_mode)",
    depth_rms_spec="<= 2% at 2 m, 80% ROI, 848x480 (datasheet)",
    source="Intel RealSense D400 Series Product Family Datasheet (rev 2023+); VERIFY",
)


# ---------------------------------------------------------------- camera helpers

def need_rs():
    if rs is None:
        raise SystemExit("pyrealsense2 is missing: use C:\\Users\\ferna\\.venvs\\realsense\\Scripts\\python.exe "
                         "(README.md, Setup)")
    return rs


USB2_HINT = ("The D435 needs USB 3 for its fast modes. PID 0AD6 ('RealSense USB2') is firmware 5.9.x on USB 2: "
             "update it (rs_fw_update.py). If it shows up as USB 2.x: a USB 3 cable, a USB 3 port straight on "
             "the PC (not a hub, not a front-panel USB 2 port), and flip the USB-C plug at the camera end; if "
             "nothing helps, try another computer (README.md, Status).")


def find_device(serial: str | None = None):
    """The first RealSense device (or the one with `serial`), with a useful error if there is none."""
    need_rs()
    devs = list(rs.context().query_devices())
    if serial:
        devs = [d for d in devs if d.get_info(rs.camera_info.serial_number) == serial]
    if not devs:
        raise SystemExit("No RealSense camera found. " + USB2_HINT)
    return devs[0]


def info(dev, key) -> str | None:
    k = getattr(rs.camera_info, key)
    return dev.get_info(k) if dev.supports(k) else None


def device_summary(dev) -> dict:
    keys = ["name", "serial_number", "firmware_version", "recommended_firmware_version", "product_id",
            "usb_type_descriptor", "physical_port", "product_line", "asic_serial_number"]
    out = {k: info(dev, k) for k in keys}
    out["librealsense"] = getattr(rs, "__version__", None)
    return out


def usb3(dev) -> bool:
    u = info(dev, "usb_type_descriptor") or ""
    return u.startswith("3")


def depth_sensor(dev):
    return dev.first_depth_sensor()


def color_sensor(dev):
    for s in dev.query_sensors():
        if "RGB" in (s.get_info(rs.camera_info.name) if s.supports(rs.camera_info.name) else ""):
            return s
    return None


def option_name(opt) -> str:
    """'Inter Cam Sync Mode' (pyrealsense2 2.5x) or 'option.inter_cam_sync_mode' -> 'inter_cam_sync_mode'."""
    return str(opt).split(".")[-1].strip().lower().replace(" ", "_")


def set_opt(sensor, name: str, value) -> bool:
    """Set an option if the sensor has it; False (and no exception) if it doesn't."""
    opt = getattr(rs.option, name, None)
    if sensor is None or opt is None or not sensor.supports(opt):
        return False
    try:
        sensor.set_option(opt, float(value))
        return True
    except RuntimeError as e:
        print(f"  could not set {name}={value}: {e}")
        return False


def get_opt(sensor, name: str):
    opt = getattr(rs.option, name, None)
    if sensor is None or opt is None or not sensor.supports(opt):
        return None
    try:
        return sensor.get_option(opt)
    except RuntimeError:
        return None


def temperatures(dev) -> dict:
    s = depth_sensor(dev)
    return {k: get_opt(s, k) for k in ("asic_temperature", "projector_temperature")}


def meta(frame, key: str):
    m = getattr(rs.frame_metadata_value, key, None)
    if m is None or not frame.supports_frame_metadata(m):
        return None
    return frame.get_frame_metadata(m)


def frame_times(frame, t_host_ms: float | None = None) -> dict:
    """Every clock a frame carries, in ms. t_mid_ms is mid-exposure on the host clock when the camera's
    metadata allows it (global time domain + sensor_timestamp), else None."""
    ts = frame.get_timestamp()
    dom = str(frame.get_frame_timestamp_domain()).split(".")[-1]
    sensor_us, frame_us = meta(frame, "sensor_timestamp"), meta(frame, "frame_timestamp")
    t_mid = None
    if dom == "global_time":
        # get_timestamp() follows frame_timestamp (start of transfer); sensor_timestamp is mid-exposure
        t_mid = ts - (frame_us - sensor_us) / 1000.0 if (sensor_us is not None and frame_us is not None) else ts
    return dict(t_ms=ts, domain=dom, frame_number=frame.get_frame_number(), t_mid_ms=t_mid,
                arrival_ms=meta(frame, "time_of_arrival"), backend_ms=meta(frame, "backend_timestamp"),
                counter=meta(frame, "frame_counter"), exposure_us=meta(frame, "actual_exposure"),
                gain=meta(frame, "gain_level"), laser=meta(frame, "frame_laser_power"),
                t_host_ms=t_host_ms)


def host_ms() -> float:
    """The host clock librealsense's global time domain maps onto (system clock, ms since the epoch)."""
    return time.time() * 1000.0


def parse_mode(text: str):
    """'ir:848x480@90', 'depth:848x480@90', 'stereo:848x480@90' (both IR), 'color:640x480@60',
    joined with '+' for several streams. Returns [(kind, w, h, fps)]."""
    out = []
    for part in text.split("+"):
        kind, rest = part.split(":")
        wh, fps = rest.split("@")
        w, h = wh.split("x")
        if kind not in ("ir", "ir2", "stereo", "depth", "color"):
            raise ValueError(f"unknown stream kind {kind!r}")
        out.append((kind, int(w), int(h), int(fps)))
    return out


def enable_streams(cfg, mode):
    for kind, w, h, fps in mode:
        if kind == "depth":
            cfg.enable_stream(rs.stream.depth, w, h, rs.format.z16, fps)
        elif kind == "ir":
            cfg.enable_stream(rs.stream.infrared, 1, w, h, rs.format.y8, fps)
        elif kind == "ir2":
            cfg.enable_stream(rs.stream.infrared, 2, w, h, rs.format.y8, fps)
        elif kind == "stereo":
            cfg.enable_stream(rs.stream.infrared, 1, w, h, rs.format.y8, fps)
            cfg.enable_stream(rs.stream.infrared, 2, w, h, rs.format.y8, fps)
        elif kind == "color":
            cfg.enable_stream(rs.stream.color, w, h, rs.format.bgr8, fps)


def mode_supported(dev, mode) -> bool:
    have = set()
    for s in dev.query_sensors():
        for p in s.get_stream_profiles():
            if p.is_video_stream_profile():
                v = p.as_video_stream_profile()
                have.add((str(p.stream_type()).split(".")[-1], v.width(), v.height(), p.fps()))
    names = {"depth": "depth", "ir": "infrared", "ir2": "infrared", "stereo": "infrared", "color": "color"}
    return all((names[k], w, h, f) in have for k, w, h, f in mode)


def configure_stereo(dev, emitter: bool | None = None, exposure_us: float | None = None,
                     laser_power: float | None = None, preset: str | None = None):
    """Depth / IR sensor settings shared by the scripts. None leaves a setting as it is."""
    s = depth_sensor(dev)
    if preset:
        presets = {"custom": 0, "default": 1, "hand": 2, "high_accuracy": 3, "high_density": 4, "medium_density": 5}
        set_opt(s, "visual_preset", presets[preset])
    if emitter is not None:
        set_opt(s, "emitter_enabled", 1 if emitter else 0)
    if laser_power is not None:
        set_opt(s, "laser_power", laser_power)
    if exposure_us is not None:
        set_opt(s, "enable_auto_exposure", 0)
        set_opt(s, "exposure", exposure_us)
    set_opt(s, "global_time_enabled", 1)
    return s


def ir_baseline_m(dev) -> float:
    """The IR pair's baseline from the camera's own calibration (the depth stream's baseline)."""
    profs = {}
    for p in depth_sensor(dev).get_stream_profiles():
        if p.stream_type() == rs.stream.infrared and p.is_video_stream_profile():
            profs.setdefault(p.stream_index(), p)
    if 1 in profs and 2 in profs:
        return abs(profs[1].get_extrinsics_to(profs[2]).translation[0])
    return D435_DATASHEET["baseline_mm"] / 1000


def intrinsics_dict(intr) -> dict:
    return dict(width=intr.width, height=intr.height, fx=intr.fx, fy=intr.fy, ppx=intr.ppx, ppy=intr.ppy,
                model=str(intr.model).split(".")[-1], coeffs=list(intr.coeffs))


def save_result(test: str, data: dict, dev=None) -> pathlib.Path:
    """results/<test>_<serial>_<time>.json with the device and host attached."""
    RESULTS.mkdir(exist_ok=True)
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    serial = info(dev, "serial_number") if (dev is not None and rs is not None) else "nodev"
    doc = dict(test=test, date=datetime.datetime.now().isoformat(timespec="seconds"),
               host=dict(machine=platform.node(), os=platform.platform(), cpu=platform.processor()))
    doc.update(data)
    if "device" not in doc:
        doc["device"] = device_summary(dev) if (dev is not None and rs is not None) else None
    path = RESULTS / f"{test}_{serial}_{stamp}.json"
    path.write_text(json.dumps(doc, indent=1, default=_json_default))
    print(f"wrote {path}")
    return path


def _json_default(o):
    if isinstance(o, (np.integer,)):
        return int(o)
    if isinstance(o, (np.floating,)):
        return None if not np.isfinite(o) else float(o)
    if isinstance(o, np.ndarray):
        return o.tolist()
    return str(o)


def latest(test: str) -> dict | None:
    files = sorted(RESULTS.glob(f"{test}_*.json"), key=lambda p: p.stat().st_mtime)
    return json.loads(files[-1].read_text()) if files else None


def all_results(test: str) -> list[dict]:
    files = sorted(RESULTS.glob(f"{test}_*.json"), key=lambda p: p.stat().st_mtime)
    return [json.loads(p.read_text()) for p in files]


# ---------------------------------------------------------------- pure maths (tested)

def stats(x) -> dict:
    """Median, mean, std, p5 / p95 / p99, min, max, n of the finite values."""
    a = np.asarray([v for v in x if v is not None], dtype=float)
    a = a[np.isfinite(a)]
    if a.size == 0:
        return dict(n=0)
    p = np.percentile(a, [5, 50, 95, 99])
    return dict(n=int(a.size), mean=float(a.mean()), std=float(a.std()), min=float(a.min()), max=float(a.max()),
                p5=float(p[0]), median=float(p[1]), p95=float(p[2]), p99=float(p[3]))


def interval_stats(t_ms, nominal_fps: float) -> dict:
    """Frame-interval statistics of timestamps (ms): achieved fps, jitter, and frames lost by interval."""
    t = np.sort(np.asarray(t_ms, dtype=float))
    if t.size < 3:
        return dict(n=int(t.size))
    dt = np.diff(t)
    nominal = 1000.0 / nominal_fps
    lost = int(np.sum(np.maximum(np.round(dt / nominal) - 1, 0)))
    span = t[-1] - t[0]
    return dict(n=int(t.size), fps=float((t.size - 1) / span * 1000.0) if span > 0 else None,
                dt_median_ms=float(np.median(dt)), dt_std_ms=float(dt.std()), dt_max_ms=float(dt.max()),
                dt_p99_ms=float(np.percentile(dt, 99)), lost_by_interval=lost)


def counter_gaps(counters) -> int:
    """Frames missing from a frame-counter sequence."""
    c = np.asarray([v for v in counters if v is not None], dtype=np.int64)
    if c.size < 2:
        return 0
    d = np.diff(np.sort(c))
    return int(np.sum(np.maximum(d - 1, 0)))


def fit_plane(points: np.ndarray) -> tuple[np.ndarray, float, np.ndarray]:
    """Least-squares plane through Nx3 points: (unit normal pointing at the camera, offset d with
    n.p + d = 0, signed residuals)."""
    p = np.asarray(points, dtype=float)
    c = p.mean(axis=0)
    _, _, vt = np.linalg.svd(p - c, full_matrices=False)
    n = vt[2]
    if n @ c > 0:            # point it from the plane toward the camera at the origin
        n = -n
    d = -n @ c
    return n, float(d), (p - c) @ n


def plane_distance_on_axis(n: np.ndarray, d: float) -> float:
    """Where the optical axis (0,0,z) meets the plane: z = -d / n_z."""
    return float(-d / n[2])


def plane_pose(n: np.ndarray, d: float) -> dict:
    """The camera's height above a floor plane and its pitch / roll relative to it, in the camera's optical
    frame (x right, y down, z forward). Pitch is the optical axis' angle below the floor's horizon
    (positive = looking down); roll is the rotation about the optical axis (the up vector's angle in the
    image plane; positive = up leans toward image +x)."""
    up = np.asarray(n, dtype=float)          # floor normal, toward the camera = "up"
    height = float(d)                         # n.0 + d = d: the camera's distance from the plane
    pitch = float(np.degrees(np.arcsin(np.clip(-up[2], -1, 1))))
    roll = float(np.degrees(np.arctan2(up[0], -up[1])))
    return dict(height_m=height, pitch_deg=pitch, roll_deg=roll)


def angle_between_deg(a, b) -> float:
    a, b = np.asarray(a, float), np.asarray(b, float)
    c = abs(a @ b) / (np.linalg.norm(a) * np.linalg.norm(b))
    return float(np.degrees(np.arccos(np.clip(c, -1.0, 1.0))))


def deproject(depth_m: np.ndarray, fx, fy, ppx, ppy, mask=None, step: int = 1) -> np.ndarray:
    """Pinhole back-projection of a depth image (m) to Nx3 points (the D435's depth stream is rectified,
    no distortion). Zero / NaN depths are dropped."""
    h, w = depth_m.shape
    v, u = np.mgrid[0:h:step, 0:w:step]
    z = depth_m[::step, ::step]
    ok = np.isfinite(z) & (z > 0)
    if mask is not None:
        ok &= mask[::step, ::step]
    z, u, v = z[ok], u[ok], v[ok]
    return np.column_stack(((u - ppx) / fx * z, (v - ppy) / fy * z, z))


def stereo_sigma_z(z, f_px, baseline_m, sigma_disp_px):
    """Depth noise of a stereo pair at range z: z^2 sigma_d / (f B)."""
    return np.asarray(z, float) ** 2 * sigma_disp_px / (f_px * baseline_m)


def disparity_sigma(z, sigma_z, f_px, baseline_m):
    """The disparity noise (px) that a measured depth noise implies: the number that carries over to
    another baseline and lens."""
    return np.asarray(sigma_z, float) * f_px * baseline_m / np.asarray(z, float) ** 2


def depth_change_to_angle_deg(z, dz, baseline_m):
    """A depth shift dz at range z, read as a change in the pair's relative yaw (the design's 'stereo
    calibration' angle, design.md 4.1): d(disparity)/f = B dz / z^2 rad."""
    return np.degrees(baseline_m * np.asarray(dz, float) / np.asarray(z, float) ** 2)


def ball_radius_px(z, f_px, d_ball=BALL_D_M):
    return f_px * (d_ball / 2) / np.asarray(z, float)


def flash_edges(t_ms, level, min_jump_frac: float = 0.5) -> list[tuple[float, int]]:
    """Transitions in a brightness series: [(time of the first frame past the midpoint, +1 rising / -1
    falling)]. The midpoint is halfway between the dark and bright levels (5th and 95th percentiles);
    a step needs `min_jump_frac` of that range to count."""
    t = np.asarray(t_ms, float)
    x = np.asarray(level, float)
    if x.size < 3:
        return []
    lo, hi = np.percentile(x, 5), np.percentile(x, 95)
    if hi - lo <= 0:
        return []
    mid = (lo + hi) / 2
    state = x[0] > mid
    out = []
    last_extreme = x[0]
    for ti, xi in zip(t[1:], x[1:]):
        s = xi > mid
        if s != state and abs(xi - last_extreme) >= min_jump_frac * (hi - lo) / 2:
            out.append((float(ti), 1 if s else -1))
            state = s
            last_extreme = xi
        elif s == state:
            last_extreme = max(last_extreme, xi) if state else min(last_extreme, xi)
    return out


def match_flashes(commands: list[tuple[float, int]], edges: list[tuple[float, int]],
                  max_lag_ms: float = 300.0) -> list[float | None]:
    """For each commanded transition (time, sign), the lag to the first seen edge of the same sign after
    it (within max_lag_ms and before the next command), else None."""
    out = []
    for i, (tc, sc) in enumerate(commands):
        t_next = commands[i + 1][0] if i + 1 < len(commands) else np.inf
        lag = None
        for te, se in edges:
            if te < tc:
                continue
            if te > min(tc + max_lag_ms, t_next + max_lag_ms) or te > t_next + 0.5 * (t_next - tc):
                break
            if se == sc:
                lag = te - tc
                break
        out.append(lag)
    return out


def fit_ballistic(t_s: np.ndarray, p: np.ndarray) -> dict:
    """Quadratic (constant acceleration) fit per axis of an Nx3 track: the acceleration vector, residual
    RMS, speed at the first sample. Over a short flight a ball's drag makes |a| a little off g and tilts it
    against the motion; a gross miss means bad timestamps, depth scale or association."""
    t = np.asarray(t_s, float) - float(t_s[0])
    P = np.asarray(p, float)
    A = np.column_stack((np.ones_like(t), t, 0.5 * t ** 2))
    coef, *_ = np.linalg.lstsq(A, P, rcond=None)
    res = P - A @ coef
    return dict(p0=coef[0].tolist(), v0=coef[1].tolist(), a=coef[2].tolist(),
                a_norm=float(np.linalg.norm(coef[2])), speed0=float(np.linalg.norm(coef[1])),
                rms_m=float(np.sqrt(np.mean(np.sum(res ** 2, axis=1)))),
                rms_axis_m=np.sqrt(np.mean(res ** 2, axis=0)).tolist(), n=int(t.size),
                duration_s=float(t[-1]))


def subpixel_peak(c_left: float, c_mid: float, c_right: float) -> float:
    """Parabola vertex offset (-0.5..0.5) through three correlation samples around a peak."""
    den = c_left - 2 * c_mid + c_right
    return 0.0 if den == 0 else float(np.clip(0.5 * (c_left - c_right) / den, -0.5, 0.5))
