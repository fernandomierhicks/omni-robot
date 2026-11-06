"""omni_mcp against a fake rosbridge (a local websocket server speaking the rosbridge v2 JSON protocol): status caching,
reconnection, event paging, image downscaling, a clean error when rosbridge is down, the lease thread, and the tools end
to end through the MCP layer with the console's real core (omni_console/missions.py) behind the fake.

    & C:\\Users\\ferna\\.venvs\\omni-mcp\\Scripts\\python.exe -m pytest -q ros/tools/omni_mcp/test
"""

from __future__ import annotations

import asyncio
import io
import json
import math
import socket
import sys
import threading
import time
from pathlib import Path

import pytest
from PIL import Image as PILImage
from websockets.sync.server import serve

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
sys.path.insert(0, str(HERE.parents[2] / "src" / "omni_console"))     # the console's core, for the end-to-end fake

import images  # noqa: E402
from rosbridge import Rosbridge  # noqa: E402
from server import Robot, build_server  # noqa: E402


def free_port() -> int:
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


class FakeRosbridge:
    """Subscriptions, publishes and /console/command calls; reply(verb, args) -> (ok, message, data)."""

    def __init__(self, port: int, reply=None):
        self.port, self.reply = port, reply or (lambda v, a: (True, "", {}))
        self.clients, self.subs, self.calls = [], {}, []
        self.lock = threading.Lock()
        self.server = serve(self.handle, "127.0.0.1", port)
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()

    def handle(self, ws):
        with self.lock:
            self.clients.append(ws)
        try:
            for raw in ws:
                m = json.loads(raw)
                if m["op"] == "subscribe":
                    self.subs[m["topic"]] = m
                elif m["op"] == "call_service":
                    a = m["args"]
                    args = json.loads(a.get("args") or "{}")
                    self.calls.append((a["verb"], args))
                    ok, msg, data = self.reply(a["verb"], args)
                    ws.send(json.dumps(dict(op="service_response", id=m["id"], result=True, service=m["service"],
                                            values=dict(ok=ok, message=msg, data=json.dumps(data)))))
        except Exception:
            pass
        finally:
            with self.lock:
                self.clients.remove(ws)

    def publish(self, topic: str, data):
        msg = json.dumps(dict(op="publish", topic=topic, msg=dict(data=data if isinstance(data, str)
                                                                  else json.dumps(data))))
        with self.lock:
            for ws in list(self.clients):
                try:
                    ws.send(msg)
                except Exception:
                    pass

    def close(self):
        self.server.shutdown()
        with self.lock:
            for ws in list(self.clients):
                ws.close()


def wait(cond, timeout=5.0):
    t = time.monotonic() + timeout
    while time.monotonic() < t:
        if cond():
            return True
        time.sleep(0.02)
    return False


STATUS = dict(source="sim", mission="MANUAL", task="Ready to drive", progress=None, robot_mode="TELEOP", kid=False,
              pose=dict(frame="map", x=1.0, y=2.0, yaw=0.5, t=0.0), localization=None, map="house",
              slam=dict(mode="localization"), caps=dict(nav=True), mcu=None, arm=None,
              vision=dict(images=["/arm_cam/image_raw", "/stereo/left/image_color"]),
              ai=dict(lease=True, owner=None, task=None))


@pytest.fixture
def fake():
    f = FakeRosbridge(free_port())
    yield f
    f.close()


def test_status_is_cached_from_the_topic(fake):
    rb = Rosbridge(f"ws://127.0.0.1:{fake.port}")
    assert rb.wait_up(3.0)
    rb.subscribe("/console/status", "std_msgs/msg/String", throttle_ms=200)
    assert wait(lambda: "/console/status" in fake.subs)
    assert fake.subs["/console/status"]["throttle_rate"] == 200
    fake.publish("/console/status", STATUS)
    assert wait(lambda: rb.newest_json("/console/status")[0] is not None)
    s, age = rb.newest_json("/console/status")
    assert s["mission"] == "MANUAL" and 0 <= age < 1.0
    rb.close()


def test_it_reconnects_and_subscribes_again():
    port = free_port()
    f1 = FakeRosbridge(port)
    rb = Rosbridge(f"ws://127.0.0.1:{port}")
    assert rb.wait_up(3.0)
    rb.subscribe("/console/status", "std_msgs/msg/String")
    assert wait(lambda: "/console/status" in f1.subs)
    f1.close()
    assert wait(lambda: not rb.up.is_set(), 3.0)
    f2 = FakeRosbridge(port)
    assert rb.wait_up(8.0)                                           # backoff, then back
    assert wait(lambda: "/console/status" in f2.subs)                # subscribed again by itself
    f2.publish("/console/status", STATUS)
    assert wait(lambda: rb.newest_json("/console/status")[0] is not None)
    assert rb.n_connects == 2
    rb.close()
    f2.close()


def test_events_page_by_number(fake):
    rb = Rosbridge(f"ws://127.0.0.1:{fake.port}")
    rb.wait_up(3.0)
    rb.subscribe("/console/events", "std_msgs/msg/String", queue_length=50)
    assert wait(lambda: "/console/events" in fake.subs)
    for k in range(5):
        fake.publish("/console/events", dict(t=k, level="info", text=f"event {k}", source="console"))
    assert wait(lambda: rb.n_events == 5)
    ev = rb.events_since(2)
    assert [e["text"] for e in ev] == ["event 2", "event 3", "event 4"] and ev[-1]["n"] == 5
    rb.close()


def test_images_are_downscaled_to_640():
    buf = io.BytesIO()
    PILImage.new("RGB", (1280, 800), (140, 51, 191)).save(buf, format="PNG")
    jpeg, w, h = images.shrink(buf.getvalue())
    assert (w, h) == (640, 400) and jpeg[:2] == b"\xff\xd8"
    small = io.BytesIO()
    PILImage.new("RGB", (320, 240)).save(small, format="PNG")
    assert images.shrink(small.getvalue())[1:] == (320, 240)


def test_a_clean_error_when_rosbridge_is_down():
    port = free_port()                                               # nothing listens there
    robot = Robot("127.0.0.1", port, free_port(), free_port(), lease=False)
    t0 = time.monotonic()
    r = robot.call("ping")
    assert not r["ok"] and r["data"]["error"] == "no_robot" and "rosbridge" in r["message"]
    assert time.monotonic() - t0 < 5.0
    s = robot.status()
    assert not s["ok"] and s["data"]["error"] == "no_robot"
    r, jpeg = robot.look("wrist")
    assert not r["ok"] and jpeg is None
    robot.close()


def test_the_lease_is_renewed_every_second(fake):
    robot = Robot("127.0.0.1", fake.port, free_port(), free_port(), session="abc123")
    assert wait(lambda: sum(v == "ai_lease" for v, _ in fake.calls) >= 3, 4.0)
    a = [a for v, a in fake.calls if v == "ai_lease"][0]
    assert a == dict(session="abc123", ttl_s=3.0)
    assert robot.lease.state()["held"]
    robot.close()


def test_motion_carries_the_session_and_a_request_id(fake):
    robot = Robot("127.0.0.1", fake.port, free_port(), free_port(), session="s1", lease=False)
    robot.rb.wait_up(3.0)
    robot.move([{"do": "spin", "turns": 1}], turn_rate="max")
    robot.stop()
    (v1, a1), (v2, a2) = fake.calls[-2:]
    assert v1 == "move" and a1["ai"] == "s1" and a1["request_id"].startswith("s1-") and a1["turn_rate"] == "max"
    other = Robot("127.0.0.1", fake.port, free_port(), free_port(), session="s1", lease=False)
    assert other.boot != robot.boot                                  # a second run's ids don't collide with these
    other.close()
    assert v2 == "stop" and a2 == dict(ai="s1")
    robot.close()


def test_the_map_picture_and_its_pixel_formula():
    import views
    w, h, res = 120, 80, 0.05                                        # a 6 x 4 m room, walls round it
    cells = [0] * (w * h)
    for r in range(h):
        for c in range(w):
            if r in (0, h - 1) or c in (0, w - 1):
                cells[r * w + c] = 100
    msg = dict(header=dict(frame_id="map"), info=dict(width=w, height=h, resolution=res,
                                                     origin=dict(position=dict(x=-1.0, y=-2.0, z=0.0))), data=cells)
    g = views.grid_of(msg)
    png, info = views.map_picture(g, pose=(1.0, 0.0, 0.5), rooms={"room": [[-1, -2], [5, -2], [5, 2], [-1, 2]]},
                                  places={"dock": (0.0, 0.0, 0.0)}, marks=[{"x": 2.3, "y": 1.3, "label": "goal"}])
    im = PILImage.open(io.BytesIO(png)).convert("RGB")
    assert im.size == (info["width_px"], info["height_px"]) and max(im.size) <= 900
    x0, y1, m = info["crop_m"][0], info["crop_m"][3], info["m_per_px"]
    u, v = round((2.3 - x0) / m - 0.5), round((y1 - 1.3) / m - 0.5)  # the mark, by the reply's formula
    near = [im.getpixel((u + du, v + dv)) for du in (-1, 0, 1) for dv in (-1, 0, 1)]
    assert any(p[0] > 180 and p[1] < 80 for p in near)               # the red x is there


def test_scan_sectors_are_in_the_base_frame():
    import views
    n = 360
    scan = dict(angle_min=-math.pi, angle_increment=2 * math.pi / n, range_min=0.1, range_max=10.0,
                ranges=[2.0] * n)
    scan["ranges"][n // 2] = 0.5                                    # straight ahead of the laser, close
    s = views.scan_summary(scan, laser=(0.2, 0.0, 0.0))
    ahead = [x for x in s["sectors"] if x["bearing_deg"] == 5.0 or x["bearing_deg"] == -5.0]
    assert min(x["nearest_m"] for x in ahead) == 0.7                  # 0.2 m laser offset + 0.5 m
    assert views.scan_picture(s["points"])[:4] == b"\x89PNG"


def test_once_reads_one_message_and_scripts_attach(fake, tmp_path, monkeypatch):
    robot = Robot("127.0.0.1", fake.port, free_port(), free_port(), session="sess-x", lease=False)
    robot.rb.wait_up(3.0)

    def publish_soon():
        time.sleep(0.3)
        fake.publish("/localization/health", dict(level="GOOD"))
    threading.Thread(target=publish_soon, daemon=True).start()
    m = robot.rb.once("/localization/health", "std_msgs/msg/String", 3.0)
    assert json.loads(m["data"])["level"] == "GOOD" and "/localization/health" not in robot.rb.subs
    monkeypatch.setattr(Robot, "SESSION_FILE", tmp_path / "session.json")
    robot.publish_session()
    script = Robot.attach()
    assert script.session == "sess-x" and script.lease is None       # rides on the server's lease
    script.rb.wait_up(3.0)
    script.stop()
    assert fake.calls[-1] == ("stop", dict(ai="sess-x"))
    script.close()
    robot.close()


# -- end to end: the MCP tools on the console's real core --------------------------------------------------------
class ConsoleFake(FakeRosbridge):
    """The console's core behind the fake rosbridge, a perfect base under it: status at 10 Hz, drive_output at 20 Hz
    integrated into the odometry."""

    def __init__(self, port):
        from omni_console.missions import Console
        self.c = Console()
        self.c.now = time.monotonic()
        self.c.on_robot_mode("TELEOP adult")
        self.c.set_caps(nav=True, catch=False, collision=False)
        self.c.odom_pose = (0.0, 0.0, 0.0)
        self.c.pose = ("map", 0.0, 0.0, 0.0)
        self.yaw = 0.0
        self.mu = threading.Lock()
        super().__init__(port, self.on_call)
        self.run = True
        threading.Thread(target=self.loop, daemon=True).start()

    def on_call(self, verb, args):
        with self.mu:
            self.c.now = time.monotonic()
            r = self.c.command(verb, args)
        return r.ok, r.message, r.data if r.data is not None else {}

    def loop(self):
        k = 0
        while self.run:
            time.sleep(0.05)
            with self.mu:
                now = time.monotonic()
                tw = self.c.drive_output(now)
                if tw is not None:
                    x, y, yaw = self.c.odom_pose
                    yaw += tw[2] * 0.05
                    self.c.odom_pose = (x + tw[0] * 0.05 * math.cos(yaw), y + tw[0] * 0.05 * math.sin(yaw), yaw)
                k += 1
                s = dict(self.c.summary(), source="sim", pose=dict(frame="map", x=0.0, y=0.0,
                                                                   yaw=self.c.odom_pose[2], t=now),
                         vision=dict(images=[]))
            if k % 2 == 0:
                self.publish("/console/status", s)
            for level, text in self.c.pop_events():
                self.publish("/console/events", dict(t=time.time(), level=level, text=text, source="console"))

    def close(self):
        self.run = False
        super().close()


def call(server, name, args=None):
    from mcp import Client

    async def go():
        async with Client(server) as c:
            r = await c.call_tool(name, args or {})
            return r
    return asyncio.run(go())


def text_of(r) -> dict:
    return json.loads(r.content[0].text)


def test_spin_five_times_through_the_mcp_tools():
    f = ConsoleFake(free_port())
    robot = Robot("127.0.0.1", f.port, free_port(), free_port(), session="mcp-test")
    try:
        assert wait(lambda: robot.lease.state()["held"], 5.0)
        server = build_server(robot)

        async def tools():
            from mcp import Client
            async with Client(server) as c:
                return [t.name for t in (await c.list_tools()).tools]
        names = asyncio.run(tools())
        assert {"robot_status", "robot_move", "robot_task", "robot_stop", "robot_find", "robot_pick"} <= set(names)
        assert wait(lambda: robot.status()["ok"], 3.0)
        st = text_of(call(server, "robot_status"))
        assert st["ok"] and st["data"]["source"] == "sim" and st["data"]["lease"]["held"]
        r = text_of(call(server, "robot_move", dict(steps=[{"do": "spin", "turns": 5}], turn_rate="max")))
        assert r["ok"], r
        tid = r["data"]["task_id"]
        assert r["data"]["result"]["limits"]["turn_rate_deg_s"] == 180.0
        t = None
        for _ in range(4):
            t = text_of(call(server, "robot_task", dict(task_id=tid, wait_s=10)))["data"]
            if t["state"] in ("SUCCEEDED", "FAILED"):
                break
        assert t["state"] == "SUCCEEDED", t
        assert abs(t["evidence"]["turned_deg"] - 1800) < 3
        busy = text_of(call(server, "robot_move", dict(steps=[{"do": "spin", "turns": 1}], request_id="again-1")))
        dup = text_of(call(server, "robot_move", dict(steps=[{"do": "spin", "turns": 1}], request_id="again-1")))
        assert busy["data"]["task_id"] == dup["data"]["task_id"]            # the retry is the same task
        stop = text_of(call(server, "robot_stop"))
        assert stop["ok"]
        t2 = text_of(call(server, "robot_task", dict(task_id=busy["data"]["task_id"], wait_s=0)))["data"]
        assert t2["state"] == "CANCELLED"
        ev = text_of(call(server, "robot_events", dict(since=0)))["data"]["events"]
        assert any("AI" in e["text"] for e in ev)
    finally:
        robot.close()
        f.close()


def test_a_dead_server_lets_the_lease_lapse_and_the_robot_stops():
    f = ConsoleFake(free_port())
    robot = Robot("127.0.0.1", f.port, free_port(), free_port(), session="dies")
    try:
        assert wait(lambda: robot.lease.state()["held"], 5.0)
        r = robot.move([{"do": "spin", "turns": 5}], turn_rate="max")
        assert r["ok"]
        time.sleep(1.0)
        robot.lease.stop()                                          # the MCP process "dies"
        t_cut = time.monotonic()
        assert wait(lambda: f.c.mission != "PROGRAM", 6.0)
        assert time.monotonic() - t_cut < 3.0 + 0.5 + 0.2
        assert f.c.ai_book.get(r["data"]["task_id"]).state == "CANCELLED"
    finally:
        robot.close()
        f.close()
