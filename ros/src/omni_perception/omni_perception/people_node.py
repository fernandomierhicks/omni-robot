"""people: who is around the robot, from the lidar and the forward colour camera (people_vision.py; design.md 4.12).

Subscribes scan_topic        sensor_msgs/LaserScan: people by their legs (omni_console.people.detect), furniture on
                             /map left out when the robot is localized on it
           image_topic       the forward camera: sensor_msgs/Image (rgb8 / bgr8) or, if the name ends in
                             /compressed, sensor_msgs/CompressedImage (the IMX662 sends MJPEG over USB 2)
           info_topic        its sensor_msgs/CameraInfo (intrinsics; the image is taken as rectified)
           /map              nav_msgs/OccupancyGrid (latched): the furniture test
           /tf               base_link -> the image's frame (static), fixed_frame -> base_link at each stamp
Publishes  /people           std_msgs/String JSON at the scan's rate: {"t", "frame", "people": [tracks: id, status
                             (person | candidate | not_person), p, x, y (fixed frame), vx, vy, base_x, base_y, range,
                             bearing_deg, camera, lidar, how, seen_s, age_s, camera_hits, score], "camera": {"ok",
                             "age_s", "ms", "boxes", "model"}}; people first, then candidates, nearest first
           /people/image     sensor_msgs/Image rgb8: the last camera frame shrunk to `debug_width` with its boxes and
                             track ids, only while something subscribes (the console's Vision tab, omni_mcp)

The camera is run at `rate` on the newest frame (the detector takes ~10-25 ms on a PC CPU, est); each frame's people
are fused once, with the next scan, after moving the scan's legs to where the robot stood when the frame was taken.
No camera (no frames for 1 s, no model, no camera_info or no TF): the lidar alone, as the console's Follow did.

Parameters (omni_params; values in omni_bringup/config/robot.yaml): model, model_dir, threads, rate, min_score,
floating_score,
image_topic, info_topic, scan_topic, fixed_frame, debug_width.
"""

from __future__ import annotations

import json
import math
import os
import time
from pathlib import Path

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from omni_params import Params
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from omni_perception import people_vision as pv

CAMERA_STALE_S = 1.0
MAX_SKEW_S = 0.35               # a camera frame fused with a scan at most this far apart


def _yaw(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def _matrix(tr) -> np.ndarray:
    q, p = tr.rotation, tr.translation
    x, y, z, w = q.x, q.y, q.z, q.w
    R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                  [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                  [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, (p.x, p.y, p.z)
    return T


def model_path(model: str, model_dir: str) -> Path:
    """a model name (people_vision.MODELS) in model_dir ("" = $OMNI_MODEL_DIR or ~/.omni_robot/models), or a path"""
    if model.endswith(".onnx"):
        return Path(model).expanduser()
    d = model_dir or os.environ.get("OMNI_MODEL_DIR") or os.path.expanduser("~/.omni_robot/models")
    return Path(d) / f"{model}.onnx"


class PeopleNode(Node):
    def __init__(self):
        super().__init__("people")
        p = Params(self)
        self.model_name = p.text("model", "yolox_nano", "the person detector: yolox_nano | yolox_tiny | yolox_s | an .onnx path")
        model_dir = p.text("model_dir", "", "where the models are (\"\" = $OMNI_MODEL_DIR or ~/.omni_robot/models)")
        threads = p.number("threads", 2, "onnxruntime threads for the detector", (1, 16))
        hz_to_dt = lambda hz: 1.0 / max(float(hz), 0.1)   # noqa: E731
        self.dt = p.number("rate", 5.0, "Hz: camera frames searched for people at most", (0.1, 30.0), live=self,
                           attr="dt", convert=hz_to_dt)
        self.min_score = p.number("min_score", 0.35, "a person box's least score", (0.05, 0.95), live=self)
        self.floating_score = p.number("floating_score", 0.5, "least score of a box whose bottom floats above the "
                                       "horizon (no feet on the floor: behind furniture, or a sign)", (0.05, 0.99),
                                       live=self, attr="floating_score")
        image_topic = p.text("image_topic", "/front_cam/image_raw", "the forward camera (/compressed: MJPEG)")
        info_topic = p.text("info_topic", "", "its camera_info (\"\" = beside image_topic)") or \
            image_topic.removesuffix("/compressed").rsplit("/", 1)[0] + "/camera_info"
        scan_topic = p.text("scan_topic", "/scan", "the lidar scan")
        self.fixed = p.text("fixed_frame", "odom", "the frame people are tracked in")
        self.debug_w = p.number("debug_width", 640, "px: /people/image's width", (160, 1920), live=self, attr="debug_w")

        self.det, self.det_err = None, ""
        path = model_path(self.model_name, model_dir)
        try:
            self.det = pv.PersonDetector(str(path), threads=threads, min_score=self.min_score)
        except Exception as e:                                       # no model or no onnxruntime: the lidar alone
            self.det_err = f"{type(e).__name__}: {e}"
            self.get_logger().warn(f"no person detector ({path}): {self.det_err}; run ros/tools/fetch_models.py. "
                                   "People from the lidar only.")
        self.tracker = pv.PeopleTracker()
        self.K, self.wh, self.T_cam, self.cam_frame = None, None, None, ""
        self.img, self.t_img_rx, self.t_run = None, -1e9, -1e9       # the newest frame (msg), when it came, last run
        self.sightings = None                                        # (stamp s, base pose then, Camera, [Sighting]): not fused yet
        self.last_boxes, self.last_rgb, self.ms = [], None, 0.0
        self.laser = None
        self.grid = None
        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, self)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.p_people = self.create_publisher(String, "/people", 5)
        self.p_image = self.create_publisher(Image, "/people/image", 1)
        self.create_subscription(LaserScan, scan_topic, self.on_scan, qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid, "/map", self.on_map, latched)
        self.create_subscription(CameraInfo, info_topic, self.on_info, 5)
        if image_topic.endswith("/compressed"):
            self.create_subscription(CompressedImage, image_topic, self.on_image, qos_profile_sensor_data)
        else:
            self.create_subscription(Image, image_topic, self.on_image, qos_profile_sensor_data)
        self.create_timer(0.02, self.camera_tick)
        self.get_logger().info(f"people: lidar {scan_topic}" + (f" + camera {image_topic} ({self.model_name})"
                                                                 if self.det else " only"))

    # -- inputs ----------------------------------------------------------------------------------------------------------
    def on_info(self, m: CameraInfo):
        self.K, self.wh = (m.k[0], m.k[4], m.k[2], m.k[5]), (m.width, m.height)

    def on_image(self, m):
        self.img, self.t_img_rx = m, time.monotonic()

    def on_map(self, m: OccupancyGrid):
        i = m.info
        self.grid = (i, np.frombuffer(bytes(m.data), dtype=np.int8).reshape(i.height, i.width))

    def _lookup(self, target: str, source: str, stamp=None):
        try:
            return self.tf.lookup_transform(target, source, stamp or Time()).transform
        except Exception:
            if stamp is None:
                return None
            try:                                                     # not buffered at that stamp: the latest
                return self.tf.lookup_transform(target, source, Time()).transform
            except Exception:
                return None

    def _pose(self, stamp):
        tr = self._lookup(self.fixed, "base_link", stamp)
        return None if tr is None else (tr.translation.x, tr.translation.y, _yaw(tr.rotation))

    # -- the camera ------------------------------------------------------------------------------------------------------
    def _rgb(self, m):
        import cv2
        if isinstance(m, CompressedImage):
            bgr = cv2.imdecode(np.frombuffer(bytes(m.data), np.uint8), cv2.IMREAD_COLOR)
            return None if bgr is None else bgr[:, :, ::-1]
        ch = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4}.get(m.encoding)
        if ch is None:
            return None
        a = np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.step)[:, :m.width * ch].reshape(m.height, m.width, ch)
        a = a[:, :, :3]
        return a[:, :, ::-1] if m.encoding.startswith("bgr") else a

    def camera_tick(self):
        now = time.monotonic()
        if self.det is None or self.img is None or now - self.t_run < self.dt or now - self.t_img_rx > CAMERA_STALE_S:
            return
        m, self.img = self.img, None
        self.t_run = now
        frame = m.header.frame_id
        if self.T_cam is None or frame != self.cam_frame:
            tr = self._lookup("base_link", frame)
            if tr is None:
                self.get_logger().warn(f"no TF base_link -> {frame}: camera unused", throttle_duration_sec=10.0)
                return
            self.T_cam, self.cam_frame = _matrix(tr), frame
        rgb = self._rgb(m)
        if rgb is None:
            return
        h, w = rgb.shape[:2]
        if self.K is None:
            return
        K = self.K
        if self.wh is not None and tuple(self.wh) != (w, h):              # camera_info for another resolution
            sx, sy = w / self.wh[0], h / self.wh[1]
            K = (K[0] * sx, K[1] * sy, K[2] * sx, K[3] * sy)
        self.det.min_score = self.min_score
        cam = pv.Camera(K, w, h, self.T_cam)
        boxes = [b for b in self.det.detect(rgb) if pv.plausible(cam, b, self.floating_score)]
        self.ms = self.det.ms
        stamp = Time.from_msg(m.header.stamp)
        pose = self._pose(stamp)
        self.sightings = (stamp.nanoseconds * 1e-9, pose, cam, [pv.sight(cam, b) for b in boxes])
        self.last_boxes, self.last_rgb = boxes, rgb if self.p_image.get_subscription_count() > 0 else None

    # -- the lidar, then both --------------------------------------------------------------------------------------------
    def on_scan(self, m: LaserScan):
        from omni_console.people import detect
        if self.laser is None:
            tr = self._lookup("base_link", m.header.frame_id or "laser")
            self.laser = (tr.translation.x, tr.translation.y, _yaw(tr.rotation)) if tr is not None else (0.0, 0.0, 0.0)
        stamp = Time.from_msg(m.header.stamp)
        pose = self._pose(stamp)
        if pose is None:
            return
        r = np.asarray(m.ranges, dtype=float)
        r = np.where((r > m.range_min) & (r < m.range_max), r, np.inf)
        legs = detect(r, m.angle_min, m.angle_increment, self.laser, self._occupied())
        t = stamp.nanoseconds * 1e-9
        obs, cam_state = None, dict(ok=False, model=self.model_name if self.det else "", error=self.det_err)
        s = self.sightings
        if s is not None and s[1] is not None and abs(t - s[0]) <= MAX_SKEW_S:
            self.sightings = None
            t_img, pose_img, cam, sightings = s
            legs_then = pv.move_points(legs, pose, pose_img)           # the legs as seen from where the frame was taken
            obs = pv.fuse(sightings, legs_then, cam)
            back = pv.move_points([(o.x, o.y) for o in obs], pose_img, pose)
            for o, (x, y) in zip(obs, back):
                o.x, o.y = x, y
            cam_state.update(ok=True, age_s=round(t - t_img, 3), ms=round(self.ms, 1), boxes=len(sightings))
        elif self.det is not None:
            cam_state.update(ok=time.monotonic() - self.t_img_rx < CAMERA_STALE_S, ms=round(self.ms, 1))
        if obs is None:
            obs = [pv.Observation(x, y, True, False) for x, y in legs]
        self.tracker.update(t, obs, pose)
        people = self.tracker.report(t, pose)
        self.p_people.publish(String(data=json.dumps(dict(t=round(t, 3), frame=self.fixed, people=people,
                                                          camera=cam_state))))
        if self.last_rgb is not None:
            self._debug(people)
            self.last_rgb = None

    def _occupied(self):
        """f(x_base, y_base) -> True on the map's occupied cells (5 cm round), if the robot is localized on /map"""
        if self.grid is None:
            return None
        tr = self._lookup("map", "base_link")
        if tr is None:
            return None
        i, data = self.grid
        x0, y0, yaw = tr.translation.x, tr.translation.y, _yaw(tr.rotation)
        c, s = math.cos(yaw), math.sin(yaw)

        def occ(xb, yb):
            x, y = x0 + c * xb - s * yb, y0 + s * xb + c * yb
            col, row = int((x - i.origin.position.x) / i.resolution), int((y - i.origin.position.y) / i.resolution)
            win = data[max(0, row - 1):row + 2, max(0, col - 1):col + 2]
            return win.size > 0 and int(win.max()) >= 65
        return occ

    def _debug(self, people):
        import cv2
        rgb = self.last_rgb
        h, w = rgb.shape[:2]
        k = self.debug_w / w
        img = cv2.resize(rgb, (int(w * k), int(h * k)), interpolation=cv2.INTER_AREA).copy()
        for b in self.last_boxes:
            cv2.rectangle(img, (int(b.x0 * k), int(b.y0 * k)), (int(b.x1 * k), int(b.y1 * k)), (255, 140, 0), 2)
            cv2.putText(img, f"{b.score:.2f}", (int(b.x0 * k) + 2, int(b.y0 * k) + 14), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                        (255, 140, 0), 1)
        cv2.putText(img, f"{sum(p['status'] == 'person' for p in people)} people, {self.ms:.0f} ms", (6, 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
        msg = Image()
        msg.header.stamp, msg.header.frame_id = self.get_clock().now().to_msg(), self.cam_frame
        msg.height, msg.width = img.shape[:2]
        msg.encoding, msg.step = "rgb8", img.shape[1] * 3
        msg.data = np.ascontiguousarray(img).tobytes()
        self.p_image.publish(msg)


def main():
    rclpy.init()
    n = PeopleNode()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
