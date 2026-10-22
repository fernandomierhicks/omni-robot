"""catch_scorer: the ROS catch game's score (design.md 30). PC only.

Collects the sim's truth for every throw (/sim/catch_result) next to what the robot thought (/catch/status), the
perception's processing times (/catch/perception) and the link's statistics (/mcu/link), and rewrites
studies/results/<out>.json after every throw (the run can be stopped at any time). sil_catch.py report reads it.
"""

from __future__ import annotations

import json

import rclpy
from omni_interfaces.msg import CatchStatus
from rclpy.node import Node
from std_msgs.msg import String

from omni_sim import results_dir


class CatchScorer(Node):
    def __init__(self):
        super().__init__("catch_scorer")
        self.out = self.declare_parameter("out", "ros_catch").value
        self.label = self.declare_parameter("label", "").value
        self.throws, self.robot, self.perc, self.link = [], [], None, None
        self.create_subscription(String, "/sim/catch_result", self.on_result, 50)
        self.create_subscription(CatchStatus, "/catch/status", self.on_status, 20)
        self.create_subscription(String, "/catch/perception", lambda m: setattr(self, "perc", json.loads(m.data)), 5)
        self.create_subscription(String, "/mcu/link", lambda m: setattr(self, "link", json.loads(m.data)), 5)
        self.last_n = 0

    def on_status(self, m: CatchStatus):
        if m.tosses > self.last_n and m.last:
            last = json.loads(m.last)
            if last.get("toss") == m.tosses and m.state != "settle":
                self.robot.append(last)
                self.last_n = m.tosses

    def on_result(self, m: String):
        self.throws.append(json.loads(m.data))
        k = sum(t["caught"] for t in self.throws)
        self.get_logger().info(f"score: {k}/{len(self.throws)} caught")
        path = results_dir() / f"{self.out}.json"
        path.write_text(json.dumps(dict(label=self.label, throws=self.throws, robot=self.robot, perception=self.perc,
                                        link=self.link), indent=1))


def main():
    rclpy.init()
    node = CatchScorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
