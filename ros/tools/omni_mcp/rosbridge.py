"""One websocket to the robot's rosbridge (ai_robot_architecture.md 3.1): the rosbridge v2 JSON protocol, no ROS on
this side. A reader thread keeps the newest message of each subscribed topic and the last 200 console events, and
hands service replies to their callers; it reconnects with backoff (0.5 s doubling to 5 s) and subscribes again.

    rb = Rosbridge("ws://localhost:9090")
    rb.subscribe("/console/status", "std_msgs/msg/String")
    rb.console("ping")                              # /console/command: {"ok", "message", "data"}
"""

from __future__ import annotations

import itertools
import json
import threading
import time
from collections import deque

from websockets.exceptions import WebSocketException
from websockets.sync.client import connect

CONSOLE_SRV, CONSOLE_TYPE = "/console/command", "omni_interfaces/srv/ConsoleCommand"
EVENTS_TOPIC = "/console/events"
EVENTS_KEEP = 200


class RosbridgeError(RuntimeError):
    pass


class Rosbridge:
    def __init__(self, url: str, connect_timeout: float = 3.0, log=None):
        self.url = url
        self.connect_timeout = connect_timeout
        self.log = log or (lambda m: None)
        self.ws = None
        self.send_lock = threading.Lock()
        self.ids = itertools.count(1)
        self.pending: dict[str, list] = {}           # id -> [Event, reply]
        self.subs: dict[str, dict] = {}              # topic -> the subscribe op (sent again on reconnect)
        self.advertised: set = set()                 # topics this client publishes (publish)
        self.latest: dict[str, tuple] = {}           # topic -> (msg, time.monotonic())
        self.events: deque = deque(maxlen=EVENTS_KEEP)
        self.n_events = 0
        self.up = threading.Event()
        self.error = "not connected yet"
        self.n_connects = 0
        self._stop = threading.Event()
        self.thread = threading.Thread(target=self._run, name="rosbridge", daemon=True)
        self.thread.start()

    # -- the connection --------------------------------------------------------------------------------------------
    def _run(self):
        delay = 0.5
        while not self._stop.is_set():
            try:
                with connect(self.url, open_timeout=self.connect_timeout, max_size=2 ** 24, ping_interval=10) as ws:
                    self.ws, delay = ws, 0.5
                    self.n_connects += 1
                    self.error = ""
                    try:
                        for op in list(self.subs.values()):
                            self._send(op)
                        self.up.set()
                        self.log(f"rosbridge connected: {self.url}")
                        for raw in ws:
                            self._dispatch(raw)
                    finally:
                        self.up.clear()
                        self.ws = None
                        for ev, box in list(self.pending.values()):    # nobody will answer these now
                            box.append(None)
                            ev.set()
                self.error = self.error or "rosbridge closed the connection"
            except (OSError, WebSocketException, TimeoutError) as e:
                self.error = (f"rosbridge connection lost: {e}" if self.n_connects else
                              f"can't reach rosbridge at {self.url}: {e}")
                delay = min(5.0, delay * 2)
            self._stop.wait(delay)

    def _send(self, op: dict):
        ws = self.ws
        if ws is None:
            raise RosbridgeError(self.error or "not connected")
        with self.send_lock:
            ws.send(json.dumps(op))

    def _dispatch(self, raw):
        try:
            m = json.loads(raw)
        except ValueError:
            return
        op = m.get("op")
        if op == "service_response":
            p = self.pending.get(m.get("id"))
            if p is not None:
                p[1].append(m)
                p[0].set()
        elif op == "publish":
            topic, msg = m.get("topic"), m.get("msg")
            self.latest[topic] = (msg, time.monotonic())
            if topic == EVENTS_TOPIC:
                try:
                    ev = json.loads(msg.get("data", "{}"))
                except ValueError:
                    ev = {"text": msg.get("data")}
                self.n_events += 1
                self.events.append(dict(ev, n=self.n_events))

    def close(self):
        self._stop.set()
        ws = self.ws
        if ws is not None:
            try:
                ws.close()
            except Exception:
                pass

    def wait_up(self, timeout: float) -> bool:
        return self.up.wait(timeout)

    # -- the protocol ------------------------------------------------------------------------------------------------
    def subscribe(self, topic: str, msg_type: str, throttle_ms: int = 0, queue_length: int = 1):
        op = dict(op="subscribe", id=f"sub:{topic}", topic=topic, type=msg_type, throttle_rate=int(throttle_ms),
                  queue_length=int(queue_length))
        self.subs[topic] = op
        if self.ws is not None:
            try:
                self._send(op)
            except (RosbridgeError, OSError, WebSocketException):
                pass                                   # the reader subscribes again when it reconnects

    def unsubscribe(self, topic: str):
        if self.subs.pop(topic, None) is not None and self.ws is not None:
            try:
                self._send(dict(op="unsubscribe", id=f"sub:{topic}", topic=topic))
            except (RosbridgeError, OSError, WebSocketException):
                pass
        self.latest.pop(topic, None)

    def publish(self, topic: str, msg_type: str, msg: dict):
        """One message on a topic (advertised the first time). RosbridgeError when not connected."""
        if not self.up.is_set() and not self.up.wait(self.connect_timeout):
            raise RosbridgeError(self.error or "not connected")
        if topic not in self.advertised:
            self._send(dict(op="advertise", id=f"adv:{topic}", topic=topic, type=msg_type))
            self.advertised.add(topic)
            time.sleep(0.2)                            # rosbridge creates the publisher; subscribers match it
        self._send(dict(op="publish", id=f"pub:{topic}", topic=topic, msg=msg))

    def call_service(self, service: str, srv_type: str, args: dict, timeout: float = 5.0) -> dict:
        """The reply's values, or RosbridgeError (not connected, no reply in time, the call failed)."""
        if not self.up.is_set() and not self.up.wait(min(timeout, self.connect_timeout)):
            raise RosbridgeError(self.error or "not connected")
        rid = f"call:{next(self.ids)}"
        ev, box = threading.Event(), []
        self.pending[rid] = [ev, box]
        try:
            self._send(dict(op="call_service", id=rid, service=service, type=srv_type, args=args))
            if not ev.wait(timeout):
                raise RosbridgeError(f"{service}: no reply in {timeout:g} s (is the console's mission node running?)")
        except (OSError, WebSocketException) as e:
            raise RosbridgeError(f"{service}: {e}") from e
        finally:
            self.pending.pop(rid, None)
        r = box[0] if box else None
        if r is None:
            raise RosbridgeError(f"{service}: the connection dropped before the reply")
        if not r.get("result", False):
            raise RosbridgeError(f"{service}: {r.get('values')}")
        return r.get("values") or {}

    def console(self, verb: str, args: dict | None = None, timeout: float = 5.0) -> dict:
        """/console/command: {"ok", "message", "data"} with data decoded."""
        v = self.call_service(CONSOLE_SRV, CONSOLE_TYPE, dict(verb=verb, args=json.dumps(args or {})), timeout)
        try:
            data = json.loads(v.get("data") or "{}")
        except ValueError:
            data = {"raw": v.get("data")}
        return dict(ok=bool(v.get("ok")), message=v.get("message", ""), data=data)

    def once(self, topic: str, msg_type: str | None = None, timeout: float = 5.0, fresh: bool = True):
        """The next message on a topic (or the newest, fresh=False, when this client already follows it), or
        RosbridgeError. A topic this client didn't follow is unsubscribed again after."""
        kept = topic in self.subs
        t0 = time.monotonic()
        if kept and not fresh and topic in self.latest:
            return self.latest[topic][0]
        if msg_type is None:
            msg_type = self.topic_type(topic)
        if not kept:
            self.subscribe(topic, msg_type, queue_length=1)
        try:
            while time.monotonic() - t0 < timeout:
                m = self.latest.get(topic)
                if m is not None and m[1] >= t0:
                    return m[0]
                time.sleep(0.02)
            raise RosbridgeError(f"{topic}: no message in {timeout:g} s")
        finally:
            if not kept:
                self.unsubscribe(topic)

    def topic_type(self, topic: str) -> str:
        v = self.call_service("/rosapi/topic_type", "rosapi_msgs/srv/TopicType", dict(topic=topic))
        if not v.get("type"):
            raise RosbridgeError(f"{topic}: no such topic (or no publisher)")
        return v["type"]

    def newest(self, topic: str):
        """(message, age s) of the newest on a subscribed topic, or (None, None)."""
        m = self.latest.get(topic)
        return (None, None) if m is None else (m[0], time.monotonic() - m[1])

    def newest_json(self, topic: str):
        """(decoded JSON of a std_msgs/String, age s), or (None, None)."""
        m, age = self.newest(topic)
        if m is None:
            return None, None
        try:
            return json.loads(m.get("data", "null")), age
        except ValueError:
            return None, age

    def events_since(self, n: int) -> list:
        return [e for e in self.events if e["n"] > n]
