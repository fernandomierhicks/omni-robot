"""face_node: the robot's status -> the face and the voice (moments.Brain) -> the display (display.py) over TCP; what
the display's microphones heard -> a voice command (commands.py) -> the console (/console/command).

Subscribes /console/status      std_msgs/String JSON (the operator console, 10 Hz): mode, faults, battery, mission,
                                the catch game, the followed person, the localization's health
           /console/schema      std_msgs/String JSON: the map's places (for "go to the kitchen") and the Face tab's
                                settings (face.antics, face.voice)
           /console/events      std_msgs/String JSON: warnings become a short note on the face
           /mcu/state_10hz      omni_interfaces/McuState: the odometry pose (where the ball's track is, for the eyes)
                                and the body twist (the driving antics)
           /face/say            std_msgs/String: text for the robot to say (any node), or JSON {text, mood, act}
                                (the console's Say box: missions.py "say")
           /face/hear           std_msgs/String: text taken as heard by the microphones (the console's "hear": test a
                                voice command without a microphone)
Publishes  /face/state          std_msgs/String JSON, 2 Hz: what the face shows (for the console and logs)
           /console/events      std_msgs/String JSON: what it heard and what it did about it (source "voice")
Calls      /console/command     omni_interfaces/ConsoleCommand: a voice command's verb, as if clicked on the page
Serves     TCP port (9881): one JSON line per tick, {"face": {...}, "out": [{"kind": "say"|"sound", ...}], "listen":
           bool}, plus {"grammar": [...]} when the phrases change, to every display connected (the panel's on the
           robot; the window on Windows in the sim); a display sends back {"heard": text, "conf": 0-1, "echo": bool}
Parameters port, bind (127.0.0.1; the sim: 0.0.0.0 so Windows reaches it through WSL), facing_deg (0: the screen
           faces forward, design.md 12.2), start_display ("auto": in the sim, start the Windows window through WSL's
           interop when ros/omni.sh says where it is; "none" on the robot, where systemd runs the display)
Wall time only (like the console's nodes: no 1 kHz /clock in Python).

Voice commands (the console's Face tab, face.voice): "off" (the microphones stay off), "chat only" (it answers and
stops, nothing else drives it), "on" (follow, catch, go to a place, explore too). A command needs the robot's name
first ("omni, follow me"), or comes within 6 s of the name alone. While it is speaking or playing a sound, only
"stop" is taken (what the microphones hear then is mostly itself; the mic array's echo cancelling does the rest).
"""

from __future__ import annotations

import json
import os
import socket
import subprocess
import time
from dataclasses import replace

import rclpy
from omni_interfaces.msg import McuState
from omni_interfaces.srv import ConsoleCommand
from rclpy.node import Node
from std_msgs.msg import String

from omni_face.commands import decide, phrases
from omni_face.moments import Brain, BrainConfig

VOICE_LEVELS = ("off", "chat only", "on")


class FaceNode(Node):
    def __init__(self):
        super().__init__("omni_face")
        P = self.declare_parameter
        self.port = P("port", 9881).value
        bind = P("bind", "127.0.0.1").value
        self.brain = Brain(BrainConfig(facing_deg=P("facing_deg", 0.0).value))
        self.status, self.t_status = None, 0.0
        self.pose, self.vel = None, None
        self.events, self.say = [], []
        self.places: dict = {}
        self.voice = "on"
        self.grammar: list = phrases()
        self.n_tick = 0
        self.srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            self.srv.bind((bind, self.port))
            self.srv.listen(4)
            self.srv.setblocking(False)
        except OSError as e:                        # a second stack alongside (another ROS domain): no window of its own
            self.get_logger().warn(f"face: port {self.port} taken ({e}); this stack's face shows nowhere")
            self.srv.close()
            self.srv = None
        self.displays: list = []                    # [socket, bytes received so far]
        self.p_state = self.create_publisher(String, "/face/state", 5)
        self.p_events = self.create_publisher(String, "/console/events", 20)
        self.console_cli = self.create_client(ConsoleCommand, "/console/command")
        self.create_subscription(String, "/console/status", self.on_status, 5)
        self.create_subscription(String, "/console/schema", self.on_schema, 2)
        self.create_subscription(String, "/console/events", self.on_event, 20)
        self.create_subscription(McuState, "/mcu/state_10hz", self.on_mcu, 5)
        self.create_subscription(String, "/face/say", self.on_say, 10)
        self.create_subscription(String, "/face/hear", lambda m: self.on_heard(m.data, 1.0, False, "typed"), 10)
        self.create_timer(0.1, self.tick)
        self.proc = None
        if P("start_display", "auto").value == "auto" and self.srv is not None:
            self.start_windows_display()
        self.get_logger().info(f"face on tcp {bind}:{self.port}")

    def start_windows_display(self):
        """The sim: the face's window on Windows (native drawing and sound), like the MuJoCo view."""
        py, script = os.environ.get("OMNI_WIN_PY"), os.environ.get("OMNI_WIN_FACE")
        if not (py and script and os.path.exists(py)):
            self.get_logger().info("face: run ros/tools/face_view.py on Windows to see and hear it")
            return
        out = open("/tmp/omni_face_view.log", "w")
        extra = os.environ.get("OMNI_FACE_ARGS", "").split()          # e.g. "--scale 1.25 --mute --listen none"
        self.proc = subprocess.Popen([py, "-u", script, "--port", str(self.port), *extra], stdout=out,
                                     stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL)
        self.get_logger().info("face window starting on Windows (log /tmp/omni_face_view.log)")

    # -- inputs ---------------------------------------------------------------------------------------------------
    def on_status(self, m: String):
        try:
            self.status, self.t_status = json.loads(m.data), time.monotonic()
        except ValueError:
            pass

    def on_schema(self, m: String):
        """The map's places and the Face tab's settings."""
        try:
            sc = json.loads(m.data)
        except ValueError:
            return
        self.places = dict(sc.get("places") or {})
        vals = {s["key"]: s.get("value") for s in sc.get("settings") or [] if isinstance(s, dict) and "key" in s}
        if vals.get("face.antics") and vals["face.antics"] != self.brain.c.antics:
            self.brain.c = replace(self.brain.c, antics=str(vals["face.antics"]))
        if vals.get("face.voice") in VOICE_LEVELS:
            self.voice = vals["face.voice"]
        g = phrases(self.places)
        if g != self.grammar:
            self.grammar = g
            self.send_all(dict(grammar=g))

    def on_event(self, m: String):
        try:
            ev = json.loads(m.data)
        except ValueError:
            return
        if ev.get("source") != "voice":             # our own
            self.events.append(ev)

    def on_mcu(self, m: McuState):
        self.pose = (m.x, m.y, m.yaw)
        self.vel = (m.vx, m.vy, m.wz)

    def on_say(self, m: String):
        d = m.data.strip()
        if d.startswith("{"):
            try:
                self.say.append(dict(json.loads(d)))
                return
            except (ValueError, TypeError):
                pass
        self.say.append(d)

    # -- what it heard ------------------------------------------------------------------------------------------
    def on_heard(self, text: str, conf: float, echo: bool, source: str = "mic"):
        now = time.monotonic()
        action, rest, intent = decide(text, conf, echo, self.brain.listening(now), self.places, self.voice)
        if action == "ignore":
            return                                  # not meant for it, or its own voice
        if action == "wake":
            self.brain.on_wake(now)
            return
        if action == "unknown":
            self.brain.on_unknown(now, rest)
            self.emit("info", f"Heard “{rest}” ({source}): not a command I know")
            return
        self.brain.on_heard(now, rest)
        if action == "chat":
            self.brain.answer(now, intent)
            self.emit("info", f"Heard “{rest}” ({source}): {intent.name}")
            return
        if action == "refused":
            self.brain.answer(now, intent, False, "Voice driving is off in the console.")
            self.emit("info", f"Heard “{rest}” ({source}): {intent.name} not allowed (voice commands: chat only)")
            return
        args = dict(intent.args)
        if intent.verb == "goto":
            name = args.pop("place")
            x, y, yaw = (list(self.places.get(name) or [0.0, 0.0, 0.0]) + [0.0])[:3]
            args = dict(x=x, y=y, yaw=yaw, name=name)
        if not self.console_cli.service_is_ready():
            self.brain.answer(now, intent, False, "My console isn't running.")
            return
        fut = self.console_cli.call_async(ConsoleCommand.Request(verb=intent.verb, args=json.dumps(args)))

        def done(f):
            r = f.result()
            ok, msg = (r.ok, r.message) if r is not None else (False, "no reply")
            self.brain.answer(time.monotonic(), intent, ok, "" if ok else msg)
            self.emit("info" if ok else "warn", f"Heard “{rest}” ({source}): {intent.verb}"
                      + (f" {json.dumps(intent.args)}" if intent.args else "") + f": {msg}")
        fut.add_done_callback(done)

    def emit(self, level: str, text: str):
        self.p_events.publish(String(data=json.dumps(dict(t=time.time(), level=level, text=text, source="voice"))))

    # -- the displays -------------------------------------------------------------------------------------------
    def tick(self):
        now = time.monotonic()
        status = self.status if now - self.t_status < 2.0 else None
        st, outs = self.brain.update(now, status, self.pose, self.events, self.say, self.vel)
        self.events, self.say = [], []
        d = st.as_dict()
        for o in outs:
            if o.kind == "say":
                self.get_logger().info(f"says: {o.value}")
        self.accept()
        self.read_displays()
        self.send_all(dict(face=d, out=[o.as_dict() for o in outs],
                           listen=self.voice != "off" and status is not None))
        self.n_tick += 1
        if self.n_tick % 5 == 0:
            self.p_state.publish(String(data=json.dumps(d)))

    def send_all(self, obj):
        line = (json.dumps(obj) + "\n").encode()
        for c in list(self.displays):
            try:
                c[0].sendall(line)
            except OSError:
                self.drop(c)

    def drop(self, c):
        if c in self.displays:
            self.displays.remove(c)
        c[0].close()

    def read_displays(self):
        """What the displays heard: {"heard": text, "conf": 0-1, "echo": bool}, one JSON line each."""
        for c in list(self.displays):
            try:
                c[0].setblocking(False)
                data = c[0].recv(65536)
                c[0].settimeout(0.2)
            except (BlockingIOError, socket.timeout):
                c[0].settimeout(0.2)
                continue
            except OSError:
                self.drop(c)
                continue
            if not data:
                self.drop(c)
                continue
            c[1] += data
            while b"\n" in c[1]:
                line, c[1] = c[1].split(b"\n", 1)
                try:
                    m = json.loads(line)
                    if "heard" in m:
                        self.get_logger().info(f"heard: {m['heard']} ({float(m.get('conf', 0)):.2f})")
                        self.on_heard(str(m["heard"]), float(m.get("conf", 0.0)), bool(m.get("echo")))
                except (ValueError, TypeError):
                    pass

    def accept(self):
        while self.srv is not None:
            try:
                c, addr = self.srv.accept()
            except BlockingIOError:
                return
            c.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            c.settimeout(0.2)                       # a stalled display is dropped, never the node
            self.displays.append([c, b""])
            c.sendall((json.dumps(dict(grammar=self.grammar)) + "\n").encode())
            self.get_logger().info(f"display connected from {addr[0]}")

    def destroy_node(self):
        for c in self.displays:
            c[0].close()
        if self.srv is not None:
            self.srv.close()                        # the window stays: it says the software stopped, then reconnects
        super().destroy_node()


def main():
    rclpy.init()
    node = FaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
