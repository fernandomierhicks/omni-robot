"""height_map: what the D435 adds to the lidar for Nav2, a fused height map (height_map.py; design.md 4.5, 4.11).

Subscribes depth_topic                   sensor_msgs/Image, the depth camera's frames (+ its camera_info): the D435's
                                         /d435/depth/image_rect_raw 16UC1 mm with config:=d435
           /map                          nav_msgs/OccupancyGrid: the map's geometry (the height map uses its cells)
           /odometry/filtered            nav_msgs/Odometry: which way the robot is going (the speed limit's path)
           /speed_limit_zones            nav2_msgs/SpeedLimit: the console's floor-edge zones (Nav2's SpeedFilter,
                                         remapped here by stack.with_depth_layer so the two limits combine)
           /arm/out                      std_msgs/Bool: the arm is out (arm_node, or omni_cpp's arm_driver)
           /floor_obstacles/label        std_msgs/String JSON {"id": a catalog entry | "x", "y", "r" (m, map frame),
                                         "label": floor | rug | step | low | toy | obstacle | drop, "note"}: an AI's or a
                                         person's word on a thing or a spot (height_map.set_label: a word that relaxes
                                         never clears an obstacle or a drop the camera measures)
           color_topic                   sensor_msgs/Image rgb8, the depth camera's colour registered to its depth (the
                                         sim's; on the robot the driver's align_depth, VERIFY): the catalog's crops
           /tf                           map -> base_link at each frame's stamp
Publishes  /floor_obstacles/grid         nav_msgs/OccupancyGrid (latched): 100 where an obstacle or a drop is, -1
                                         elsewhere, on the map's cells; Nav2's StaticLayer (stack.FLOOR_LAYER)
           /floor_obstacles/classes      nav_msgs/OccupancyGrid: every cell's class x 20 (0 unknown, 20 floor, 40 step,
                                         60 low, 80 obstacle, 100 drop), for the console's map and an AI
           /speed_limit                  nav2_msgs/SpeedLimit (absolute m/s; 0 = none) at 5 Hz: the lower of the height
                                         map's (height_map.speed_limit) and the zones'
           /floor_obstacles/state        std_msgs/String JSON at 2 Hz: the speed limit and why, the classes' counts
           /floor_obstacles/catalog      std_msgs/String JSON (latched): the things found (height_map.Catalog: id, x, y,
                                         size, top, class, label, note, times) and "tiles", the ids in the picture below
           /floor_obstacles/candidates   sensor_msgs/Image rgb8 (again each second): the newest unlabelled things'
                                         crops, 3 x 2 tiles of 160 px, row by row (an AI looks at them through web_video_server: omni_mcp
                                         robot_floor_unknowns; it names them with robot_floor_label)

The catalog: at 2 Hz the small LOW and OBSTACLE regions within 2 m (height_map.things) not yet in it are added, with a
crop of the newest colour frame round each (if under 0.5 s old and the thing in it). It is kept in
$OMNI_STATE_DIR/floor_catalog.json (state_dir) across restarts; the crops are not.

While the arm is out its links are in the camera's view: frames are skipped. When it comes back in, the map forgets
forget_radius round the robot (the thing it just picked: seen again when the camera looks). If no depth frame came
for 1 s the speed limit drops to height_map.V_BLIND (the lidar still sees the tall things).

Parameters config ("" | d435 | a json path: the camera's pose in base_link), depth_topic, depth_info_topic, rate (5 Hz),
stride (1), forget_radius (0.5 m), publish_rate (2 Hz), map_frame (map), vx_max (m/s: the controller's top speed, the
base of the zones' percentages), color_topic, state_dir ("" = $OMNI_STATE_DIR, else ~/.omni_robot).
"""

from __future__ import annotations

import json
import math
import os
import time
from pathlib import Path

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener

from omni_arm import height_map as hm
from omni_arm.floor_obstacles import to_frame
from omni_arm.floor_vision import DepthFrame, decode_depth
from omni_arm.kinematics import ArmConfig, config_path
from omni_arm.pick import MJ_TO_OPTICAL


def _yaw(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class HeightMapNode(Node):
    def __init__(self):
        super().__init__("height_map")
        P = self.declare_parameter
        config = P("config", "d435").value
        depth_topic = P("depth_topic", "/d435/depth/image_rect_raw" if config == "d435" else "/stereo/depth").value
        info_topic = P("depth_info_topic", "").value or depth_topic.rsplit("/", 1)[0] + "/camera_info"
        self.dt = 1.0 / max(float(P("rate", 5.0).value), 0.1)
        self.stride = int(P("stride", 1).value)
        self.forget_r = float(P("forget_radius", 0.5).value)
        self.pub_dt = 1.0 / max(float(P("publish_rate", 2.0).value), 0.1)
        self.map_frame = P("map_frame", "map").value
        self.vx_max = float(P("vx_max", 2.0).value)
        color_topic = P("color_topic", "/d435/color/image_raw" if config == "d435" else "").value
        sd = P("state_dir", "").value or os.environ.get("OMNI_STATE_DIR") or os.path.expanduser("~/.omni_robot")
        self.cat_path = Path(sd) / "floor_catalog.json"
        try:
            self.catalog = hm.Catalog(json.loads(self.cat_path.read_text()).get("items"))
        except (OSError, ValueError):
            self.catalog = hm.Catalog()
        self.crops = {}                                    # id -> rgb crop (this session)
        self.color = None                                  # (t, rgb)
        self.cat_dirty = True
        self.walls = None                                  # the map's occupied cells (the catalog leaves walls out)
        self.tiles_msg, self.t_tiles = None, -1e9          # the last tile picture, sent again each second (a snapshot
                                                           # subscribes on request and waits for the next frame)
        c = ArmConfig(config_path(config)).cameras[0]
        T = np.eye(4)
        T[:3, :3], T[:3, 3] = np.array(c["R"]), c["pos"]
        self.T = T @ MJ_TO_OPTICAL
        self.K = None
        self.out = False
        self.t_last = -1e9
        self.rng = np.random.default_rng(0)
        self.map = hm.HeightMap()
        self.pending = None
        self.vel = (0.0, 0.0)                              # base frame m/s
        self.zone = 0.0                                    # the zones' limit, m/s (0 = none)
        self.cls, self.t_cls = None, -1e9
        self.limit, self.why = 0.0, ""
        self.ms, self.skip = [], {"no map": 0, "no pose": 0}
        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, self)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.p_grid = self.create_publisher(OccupancyGrid, "/floor_obstacles/grid", latched)
        self.p_cls = self.create_publisher(OccupancyGrid, "/floor_obstacles/classes", 1)
        self.p_state = self.create_publisher(String, "/floor_obstacles/state", 5)
        self.p_cat = self.create_publisher(String, "/floor_obstacles/catalog", latched)
        self.p_tiles = self.create_publisher(Image, "/floor_obstacles/candidates", 1)
        self.p_speed = None
        try:
            from nav2_msgs.msg import SpeedLimit
            self.SpeedLimit = SpeedLimit
            self.p_speed = self.create_publisher(SpeedLimit, "/speed_limit", 5)
            self.create_subscription(SpeedLimit, "/speed_limit_zones", self.on_zone, 5)
        except ImportError:
            self.get_logger().warn("nav2_msgs not installed: no speed limits")
        self.create_subscription(OccupancyGrid, "/map", self.on_map, latched)
        self.create_subscription(CameraInfo, info_topic, lambda m: setattr(self, "K", (m.k[0], m.k[4], m.k[2], m.k[5])),
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(Image, depth_topic, self.on_depth, 2)
        self.create_subscription(Odometry, "/odometry/filtered",
                                 lambda m: setattr(self, "vel", (m.twist.twist.linear.x, m.twist.twist.linear.y)), 5)
        self.create_subscription(Bool, "/arm/out", self.on_out, 10)
        self.create_subscription(String, "/floor_obstacles/label", self.on_label, 10)
        if color_topic:
            self.create_subscription(Image, color_topic, self.on_color, 1)
        self.create_timer(0.05, self.process)
        self.create_timer(0.2, self.tick)
        self.create_timer(30.0, self.report)
        self.get_logger().info(f"height map from {depth_topic} ({c['name']}) at up to {1 / self.dt:g} Hz -> "
                               f"/floor_obstacles/grid, /speed_limit")

    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def on_map(self, m: OccupancyGrid):
        o = m.info.origin.position
        self.map.geometry(m.info.resolution, o.x, o.y, m.info.width, m.info.height)
        self.walls = np.asarray(m.data, np.int16).reshape(m.info.height, m.info.width) >= 65

    def on_zone(self, m):
        if m.speed_limit <= 0:
            self.zone = 0.0
        else:
            self.zone = m.speed_limit / 100.0 * self.vx_max if m.percentage else m.speed_limit

    def pose(self, stamp) -> tuple | None:
        try:
            tr = self.tf.lookup_transform(self.map_frame, "base_link", Time.from_msg(stamp))
        except Exception:
            return None
        p = tr.transform.translation
        return p.x, p.y, _yaw(tr.transform.rotation)

    def on_out(self, m: Bool):
        if self.out and not m.data:
            pose = self.pose(Time().to_msg())
            if pose is not None:
                self.map.forget(pose[0], pose[1], self.forget_r)
                self.get_logger().info(f"arm in: the map within {self.forget_r:g} m of the robot forgotten")
        self.out = bool(m.data)

    def on_label(self, m: String):
        try:
            r = json.loads(m.data)
            label = str(r["label"])
            if "id" in r:
                it = self.catalog.label(int(r["id"]), label, r.get("note", ""))
                if it is None:
                    raise ValueError(f"no catalog entry {r['id']}")
                x, y, rad = it["x"], it["y"], it["size"] / 2 + 0.05
                self.cat_dirty = True
            else:
                x, y, rad = float(r["x"]), float(r["y"]), float(r.get("r", 0.1))
            n = self.map.set_label(x, y, rad, label)
            self.get_logger().info(f"label {label} at ({x:.2f}, {y:.2f}): {n} cells")
        except (ValueError, KeyError, TypeError) as e:
            self.get_logger().warn(f"label {m.data!r}: {e}")

    def on_color(self, m: Image):
        if m.encoding != "rgb8":
            return
        rgb = np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.width, 3)
        self.color = (m.header.stamp, rgb)

    def _crop(self, it: dict):
        """A 160 px crop of the newest colour frame round a thing (None: no fresh frame, or not in it)."""
        if self.color is None or self.K is None:
            return None
        stamp, rgb = self.color
        if self.now() - (stamp.sec + stamp.nanosec * 1e-9) > 0.5:
            return None
        pose = self.pose(stamp)
        if pose is None:
            return None
        c, s_ = math.cos(pose[2]), math.sin(pose[2])
        bx = c * (it["x"] - pose[0]) + s_ * (it["y"] - pose[1])          # the thing in base_link
        by = -s_ * (it["x"] - pose[0]) + c * (it["y"] - pose[1])
        pc = self.T[:3, :3].T @ (np.array([bx, by, it["top"] / 2]) - self.T[:3, 3])
        if pc[2] < 0.1:
            return None
        fx, fy, cx, cy = self.K
        u, v = fx * pc[0] / pc[2] + cx, fy * pc[1] / pc[2] + cy
        half = int(max(24, min(120, fx * (it["size"] / 2 + 0.08) / pc[2])))
        H, W = rgb.shape[:2]
        if not (0 <= u < W and 0 <= v < H):
            return None
        u0, v0 = int(u) - half, int(v) - half
        tile = np.zeros((2 * half, 2 * half, 3), np.uint8)
        a0, b0 = max(0, v0), max(0, u0)
        a1, b1 = min(H, v0 + 2 * half), min(W, u0 + 2 * half)
        tile[a0 - v0:a1 - v0, b0 - u0:b1 - u0] = rgb[a0:a1, b0:b1]
        k = np.linspace(0, 2 * half - 1, 160).astype(int)              # nearest-neighbour to 160 x 160
        return tile[k][:, k]

    def catalog_step(self, t: float, pose):
        """New things into the catalog (with a crop when possible); the catalog and the tiles out when changed."""
        for th in self.map.things(self.cls, pose[0], pose[1], walls=self.walls):
            it = self.catalog.add(th, t)
            if it is not None:
                self.cat_dirty = True
                crop = self._crop(it)
                if crop is not None:
                    self.crops[it["id"]] = crop
                self.get_logger().info(f"catalog: {it['cls']} {it['id']} at ({it['x']:.2f}, {it['y']:.2f}), "
                                       f"{it['size'] * 100:.0f} cm across, {it['top'] * 100:.0f} cm up")
        if not self.cat_dirty:
            return
        self.cat_dirty = False
        tiles = [it["id"] for it in reversed(self.catalog.unlabelled()) if it["id"] in self.crops][:6]
        img = np.zeros((320, 480, 3), np.uint8)
        for k, i in enumerate(tiles):
            r, c = divmod(k, 3)
            img[r * 160:(r + 1) * 160, c * 160:(c + 1) * 160] = self.crops[i]
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.height, msg.width, msg.encoding, msg.step = 320, 480, "rgb8", 480 * 3
        msg.data = img.tobytes()
        self.tiles_msg = msg
        self.p_tiles.publish(msg)
        doc = {"items": self.catalog.items, "tiles": tiles, "map_frame": self.map_frame}
        self.p_cat.publish(String(data=json.dumps(doc)))
        try:
            self.cat_path.parent.mkdir(parents=True, exist_ok=True)
            self.cat_path.write_text(json.dumps({"items": self.catalog.items}, indent=1))
        except OSError as e:
            self.get_logger().warn(f"catalog not saved: {e}")

    def on_depth(self, m: Image):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if self.out or self.K is None or t - self.t_last < self.dt * 0.95:
            return
        if self.map.res is None:
            self.skip["no map"] += 1
            return
        z = decode_depth(m.encoding, bytes(m.data), m.height, m.width)
        if z is not None:
            self.t_last = t
            if self.pending is not None:
                self.skip["no pose"] += 1
            self.pending = (m.header.stamp, t, z)
            self.process()

    def process(self):
        """The pending frame, once TF has the base at its stamp (dropped after 1 s)."""
        if self.pending is None:
            return
        stamp, t, z = self.pending
        pose = self.pose(stamp)
        if pose is None:
            if self.now() - t > 1.0:
                self.pending = None
                self.skip["no pose"] += 1
            return
        self.pending = None
        if self.out:
            return
        w0 = time.perf_counter()
        pts = hm.frame_points(DepthFrame(t, z, self.K, self.T), self.stride, self.rng)
        if pts is not None:
            pts = np.c_[to_frame(pts[:, :2], pose), pts[:, 2]]
        self.map.update(t, pts, to_frame(self.T[None, :2, 3], pose)[0])
        self.ms.append(1000 * (time.perf_counter() - w0))

    def tick(self):
        """5 Hz: the speed limit; at publish_rate the grids and the state."""
        t = self.now()
        if self.map.res is None:
            return
        if t - self.t_cls >= self.pub_dt:
            self.cls, self.t_cls = self.map.classes(t), t
            self._grid(self.p_grid, self.map.occupancy(self.cls))
            self._grid(self.p_cls, (self.cls.astype(np.int16) * 20).astype(np.int8).ravel())
        pose = self.pose(Time().to_msg())
        if pose is None or self.cls is None:
            return
        if self.t_cls == t:
            self.catalog_step(t, pose)
        if self.tiles_msg is not None and time.monotonic() - self.t_tiles >= 1.0:
            self.t_tiles = time.monotonic()
            self.tiles_msg.header.stamp = self.get_clock().now().to_msg()
            self.p_tiles.publish(self.tiles_msg)
        vx, vy = self.vel
        heading = pose[2] + (math.atan2(vy, vx) if math.hypot(vx, vy) > 0.1 else 0.0)
        v, why = self.map.speed_limit(self.cls, t, pose[0], pose[1], heading, self.vx_max)
        if self.zone > 0 and self.zone < v:
            v, why = self.zone, "a floor-edge zone"
        limited = v < self.vx_max - 1e-6
        if self.p_speed is not None:
            msg = self.SpeedLimit()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.map_frame
            msg.percentage = False
            msg.speed_limit = float(v) if limited else 0.0
            self.p_speed.publish(msg)
        self.limit, self.why = (v if limited else 0.0), why
        if int(t * 2) != int((t - 0.2) * 2):
            counts = {hm.NAMES[k]: int(np.sum(self.cls == k)) for k in range(len(hm.NAMES))}
            self.p_state.publish(String(data=json.dumps({"speed_limit": round(self.limit, 2), "why": why,
                                                         "cells": counts, "t": round(t, 1)})))

    def _grid(self, pub, data):
        g = self.map
        m = OccupancyGrid()
        m.header.frame_id = self.map_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.info.resolution = float(g.res)
        m.info.height, m.info.width = g.shape
        m.info.origin.position.x, m.info.origin.position.y = g.origin
        m.info.origin.orientation.w = 1.0
        m.data = data.tolist()
        pub.publish(m)

    def report(self):
        skipped = ", ".join(f"{v} {k}" for k, v in self.skip.items() if v)
        if self.ms or skipped:
            ms = f", {np.median(self.ms):.0f} ms median, {max(self.ms):.0f} max" if self.ms else ""
            self.get_logger().info(f"{len(self.ms)} frames in 30 s{ms}{f' (skipped: {skipped})' if skipped else ''}; "
                                   f"speed limit {self.limit:.2f} ({self.why})")
        self.ms = []
        self.skip = dict.fromkeys(self.skip, 0)


def main():
    rclpy.init()
    n = HeightMapNode()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
