"""The arm load calibration end to end in the sim (design.md 4.5): a simulated person does what Fernando would at the
console. They press Start on Calibrate's card (the /console/command "arm" verb, as the page does), read each prompt off
/console/status, do the step with their hands (the bridge's sim-only /sim/human: hang a weight on the finger, pull
with the luggage scale) and their tools (sim_human.ArmCalHuman: the kitchen scale, calipers, ruler and luggage scale
with their errors; the ruler reads /sim/arm_truth's fingertip height), and type the reading back in. It ends when
arm_brain reports the result (/console/status arm.cal_last), and writes it with what the person did.

Needs the arm step on the detailed arm (WSL):
    ros/omni.sh arm arm_model:=detailed viewer:=false             (beside Fernando's: another ROS_DOMAIN_ID, ports)
    ros/omni.sh py ../ros/tools/sim_arm_cal_human.py --out ~/arm_cal_ros.json [--quick] [--seed 0]
"""

from __future__ import annotations

import argparse
import json
import sys
import threading
import time
from pathlib import Path

import numpy as np
import rclpy
import rclpy.executors
from rclpy.node import Node
from std_msgs.msg import String

from omni_interfaces.srv import ConsoleCommand

HERE = Path(__file__).resolve()
for sim in (HERE.parents[2] / "simulation" / "v11 sil", Path.home() / "omni_ws" / "sim"):
    if (sim / "sim_human.py").exists():
        sys.path.insert(0, str(sim))
        break
from sim_human import ArmCalHuman  # noqa: E402


class RosArm:
    """sim_human's arm, through the bridge: what the person's hands do, what their ruler sees."""

    def __init__(self, node: Node):
        self.node = node
        self.pub = node.create_publisher(String, "/sim/human", 10)
        self.truth = None
        node.create_subscription(String, "/sim/arm_truth", self._truth, 5)

    def _truth(self, m: String):
        self.truth = json.loads(m.data)

    def _send(self, **a):
        self.pub.publish(String(data=json.dumps(a, default=float)))

    def hang(self, kg, point):
        self._send(op="hang", kg=float(kg), point=[float(v) for v in point])

    def unhang(self):
        self._send(op="unhang")

    def push(self, force, point):
        if force is None:
            self._send(op="release")
        else:                                     # ArmCalHuman gives it in the robot's frame (base_R: identity)
            self._send(op="push", force=[float(v) for v in force], point=[float(v) for v in point])

    def tip_height(self) -> float:
        return float(self.truth["tip_height_m"])


class Person(Node):
    def __init__(self, seed: int):
        super().__init__("sim_arm_cal_human")
        self.arm = RosArm(self)
        self.human = ArmCalHuman(self.arm, base_R=lambda: np.eye(3), seed=seed)
        self.cli = self.create_client(ConsoleCommand, "/console/command")
        self.status = None
        self.create_subscription(String, "/console/status", self._status, 5)

    def _status(self, m: String):
        self.status = json.loads(m.data)

    def command(self, verb: str, **args) -> dict:
        if not self.cli.wait_for_service(timeout_sec=10.0):
            raise RuntimeError("no /console/command: is the console running?")
        req = ConsoleCommand.Request(verb=verb, args=json.dumps(args))
        fut = self.cli.call_async(req)
        t = time.monotonic()
        while not fut.done() and time.monotonic() - t < 10.0:
            time.sleep(0.02)
        r = fut.result()
        return dict(ok=r.ok, message=r.message) if r else dict(ok=False, message="no reply")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=str(Path.home() / "arm_cal_ros.json"))
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--quick", action="store_true")
    ap.add_argument("--timeout", type=float, default=2400.0, help="s of wall time")
    a = ap.parse_args()
    rclpy.init()
    p = Person(a.seed)
    ex = rclpy.executors.SingleThreadedExecutor()
    ex.add_node(p)
    th = threading.Thread(target=ex.spin, daemon=True)
    th.start()
    t0 = time.monotonic()
    while (p.status is None or not (p.status.get("arm") or {}).get("names") or p.arm.truth is None) \
            and time.monotonic() - t0 < 120:
        time.sleep(0.2)
    arm = (p.status or {}).get("arm")
    if not arm:
        sys.exit("no arm in /console/status (is the arm step running?)")
    if p.arm.truth is None:
        sys.exit("no /sim/arm_truth: start the arm step with arm_model:=detailed")
    before = arm.get("cal_last")
    r = p.command("arm", do="cal_load", quick=a.quick)
    print(f"Start: {r}", flush=True)
    if not r["ok"]:
        sys.exit(1)
    seen, log = None, []
    while time.monotonic() - t0 < a.timeout:
        arm = (p.status or {}).get("arm") or {}
        last = arm.get("cal_last")
        if last is not None and last != before and arm.get("cal") is None:
            break
        pr = (arm.get("cal") or {}).get("prompt")
        key = None if pr is None else (pr.get("kind"), pr.get("key"), (arm.get("cal") or {}).get("step"))
        if pr is not None and key != seen:
            seen = key
            time.sleep(min(p.human.delay(pr), 5.0))         # (wall time: the sim runs about in real time)
            v = p.human.answer(pr)
            time.sleep(0.3)
            r = p.command("arm", do="cal_input", value=v)
            log.append(dict(prompt=pr.get("kind"), key=pr.get("key"), value=v, reply=r["message"]))
            print(f"{pr.get('kind')} {pr.get('key', '')}: {v} -> {r['message']}", flush=True)
        time.sleep(0.2)
    arm = (p.status or {}).get("arm") or {}
    out = dict(result=arm.get("cal_last"), load_model=arm.get("load_model"), person=log, truth=p.human.truth(),
               sim_truth=p.arm.truth, wall_s=round(time.monotonic() - t0, 1))
    Path(a.out).expanduser().write_text(json.dumps(out, indent=1, default=float))
    print(json.dumps(out.get("result"), indent=1, default=float))
    ex.shutdown()
    th.join(timeout=2.0)
    p.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
