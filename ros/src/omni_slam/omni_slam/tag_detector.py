"""tag_detector: AprilTags (36h11) in the left camera's images -> /tags/detections (design.md 10). On the robot.

Subscribes image (/cam/left/image_raw, sensor_msgs/Image mono8 / rgb8 / bgr8), stamped with the exposure's middle by
the camera node (omni_perception/camera_time.py), so the detections carry the time the tag was actually seen.
Detects with the C apriltag library (pupil_apriltags, through omni_calib.detect; since Nov 12, design.md 4.6), quads
found at `decimate` (2: ~26-31 ms a 720p-class picture on the Pi 4 with 2 threads, against 80-88 ms for OpenCV's aruco),
and undistorts the corners with the left camera's calibration (intrinsics: a JSON of omni_calib's Camera.as_dict, or
none: the image is already rectified). Each detection carries the decoder's margin, which tag_slam's min_margin gate
uses. Without pupil_apriltags it falls back to OpenCV's aruco, which gives no margin: those detections go out with
margin 100, so the gate passes them, and the node warns. Runs at `rate` Hz at most, skipping frames in between.
"""

from __future__ import annotations

import json
import time
from pathlib import Path

import numpy as np
import rclpy
from omni_interfaces.msg import TagDetection, TagDetections
from omni_params import Params
from rclpy.node import Node
from sensor_msgs.msg import Image


class TagDetector(Node):
    def __init__(self):
        super().__init__("tag_detector")
        p = Params(self)                                   # described parameters (omni_params; robot.yaml)
        self.rate = p.number("rate", 10.0, "Hz: frames searched for tags at most", (0.1, 60.0), live=self)
        self.size = p.number("tag_size", 0.16, "m: the tags' black square", (0.01, 1.0))
        self.decimate = p.number("decimate", 2.0, "quad search on a picture this many times smaller: 2 finds tags "
                                 "from ~14 px, 1 from ~11 px at ~2.5x the cost", (1.0, 4.0), live=self)
        self.threads = p.number("threads", 2, "the C detector's threads", (1, 8))
        self.sizes = {int(k): float(v) for k, v in json.loads(p.text("tag_sizes", "{}", "per-id sizes, JSON {id: m}")).items()}
        from omni_calib.detect import apriltag_backend
        self.backend = apriltag_backend()
        if self.backend == "apriltag":
            self.get_logger().info(f"C apriltag library, decimate {self.decimate:g}, {self.threads} threads")
        elif self.backend == "opencv":
            self.get_logger().warn("pupil_apriltags is not installed (ros/requirements-robot.txt): detecting with "
                                   "OpenCV's aruco, slower and without a decoder margin, so tag_slam's margin gate "
                                   "passes every detection")
        else:
            self.get_logger().error("no AprilTag detector: pip install pupil-apriltags")
        intr = p.text("intrinsics", "", "the camera's intrinsics json (omni_calib Camera; \"\" = already rectified)")
        self.cam = None
        if intr and Path(intr).exists():
            from omni_calib.geometry import Camera
            d = json.loads(Path(intr).read_text())
            self.cam = Camera(d["fx"], d["fy"], d["cx"], d["cy"], d["dist"], d["width"], d["height"])
        self.t_last = 0.0
        self.n = self.n_tags = 0
        self.ms = []
        self.pub = self.create_publisher(TagDetections, "/tags/detections", 10)
        self.create_subscription(Image, p.text("image", "/cam/left/image_raw", "the camera image"), self.on_image, 2)
        self.create_timer(10.0, self.report)

    def on_image(self, m: Image):
        now = time.monotonic()
        if now - self.t_last < 1.0 / self.rate:
            return
        self.t_last = now
        from omni_calib.detect import apriltags
        img = np.frombuffer(bytes(m.data), dtype=np.uint8).reshape(m.height, m.width, -1)
        gray = img[..., 0] if img.shape[2] == 1 else (img[..., :3] @ np.array([0.299, 0.587, 0.114])).astype(np.uint8)
        w0 = time.perf_counter()
        dets = apriltags(gray, decimate=self.decimate, threads=self.threads, backend=self.backend)
        self.ms.append(1000 * (time.perf_counter() - w0))
        out = TagDetections()
        out.header = m.header
        for tid, c, margin in dets:
            if self.cam is not None:                         # undistorted pixels: the graph's pinhole
                xy = self.cam.undistort_norm(c)
                c = np.column_stack([self.cam.fx * xy[:, 0] + self.cam.cx, self.cam.fy * xy[:, 1] + self.cam.cy])
            d = TagDetection()
            d.id, d.size = int(tid), float(self.sizes.get(int(tid), self.size))
            d.corners = [float(v) for v in np.asarray(c).ravel()]
            d.margin = float(margin if margin is not None else 100.0)
            out.detections.append(d)
        self.n += 1
        self.n_tags += len(out.detections)
        self.pub.publish(out)

    def report(self):
        if self.ms:
            self.get_logger().info(f"{self.n} frames, {self.n_tags} tags, detection {np.median(self.ms):.1f} ms median "
                                   f"({self.backend}, decimate {self.decimate:g})")
            self.ms = []


def main():
    rclpy.init()
    node = TagDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
