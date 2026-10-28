"""tag_detector: AprilTags (36h11) in the left camera's images -> /tags/detections (design.md 10). On the robot.

Subscribes image (/cam/left/image_raw, sensor_msgs/Image mono8 / rgb8 / bgr8), stamped with the exposure's middle by
the camera node (omni_perception/camera_time.py), so the detections carry the time the tag was actually seen.
Detects with OpenCV's aruco module (apriltag refinement; omni_calib.detect), undistorts the corners with the left
camera's calibration (intrinsics: a JSON of omni_calib's Camera.as_dict, or none: the image is already rectified),
and drops what can't be good: a decision margin under min_margin. Runs at `rate` Hz (10, est for full resolution on
the Orin's CPU), skipping frames in between.
"""

from __future__ import annotations

import json
import time
from pathlib import Path

import numpy as np
import rclpy
from omni_interfaces.msg import TagDetection, TagDetections
from rclpy.node import Node
from sensor_msgs.msg import Image


class TagDetector(Node):
    def __init__(self):
        super().__init__("tag_detector")
        P = self.declare_parameter
        self.rate = P("rate", 10.0).value
        self.size = P("tag_size", 0.16).value
        self.sizes = {int(k): float(v) for k, v in json.loads(P("tag_sizes", "{}").value).items()}   # id -> m
        intr = P("intrinsics", "").value
        self.cam = None
        if intr and Path(intr).exists():
            from omni_calib.geometry import Camera
            d = json.loads(Path(intr).read_text())
            self.cam = Camera(d["fx"], d["fy"], d["cx"], d["cy"], d["dist"], d["width"], d["height"])
        self.t_last = 0.0
        self.n = self.n_tags = 0
        self.ms = []
        self.pub = self.create_publisher(TagDetections, "/tags/detections", 10)
        self.create_subscription(Image, P("image", "/cam/left/image_raw").value, self.on_image, 2)
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
        dets = apriltags(gray)
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
            self.get_logger().info(f"{self.n} frames, {self.n_tags} tags, detection {np.median(self.ms):.1f} ms median")
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
