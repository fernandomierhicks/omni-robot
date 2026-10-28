// @ts-check
/* The page's link to the robot: one rosbridge connection (roslib), a small store the tabs read, and the robot's one
   command service. The robot only sends data; the page does the work. Small JSON from the mission node (/console/*),
   and three topics as the publisher's own bytes (rosbridge cbor-raw, decoded in cdr.js): the map, the lidar scan
   (placed here, with the robot's pose at the scan's time) and the MCU's 200 Hz samples (Base tab). No other raw
   high-rate topic crosses Wi-Fi, and no image: the camera is a video stream. */
import { useEffect, useRef, useState } from "../vendor/htm-preact-standalone.mjs";
import { decodeLaserScan, decodeOccupancyGrid, rawBytes } from "./cdr.js";

export const PAGE_VERSION = "0.3.0";       // omni_console/missions.py VERSION
const ROSLIB = /** @type {any} */ (window).ROSLIB;
const SCAN_PERIOD_MS = 200;                // the lidar on the map at 5 Hz (it turns at 10)

/** @type {Record<string, any>} */
export const store = {
  conn: "connecting",      // connecting | open | closed
  url: "",
  foxglovePort: 8765,
  status: null, tStatus: 0, statusHz: 0,
  schema: null,
  version: null,             // /console/version: software, firmware (expected, running), packages
  events: [],              // {t, level, text, source}, newest last
  map: null, mapSeq: 0,    // nav_msgs/OccupancyGrid (cdr.js decodeOccupancyGrid)
  scan: null, plan: null,  // {frame, xy: [x0, y0, x1, y1 ...]} (the scan: placed by this page)
  rtt: null,
  supervisor: null,        // the page server's step state (system.js), when the page comes from the supervisor
};

/** @type {{t: number, frame: string, x: number, y: number, yaw: number}[]} the last few poses, for placing scans */
const poses = [];

/** @type {Set<(keys: string[]) => void>} */
const listeners = new Set();

/** @param {Record<string, any>} patch */
export function update(patch) {
  Object.assign(store, patch);
  const keys = Object.keys(patch);
  listeners.forEach((f) => f(keys));
}

/** Re-render when any of the keys changes (all keys when none are given).
 *  @param {...string} keys */
export function useStore(...keys) {
  const [, bump] = useState(0);
  useEffect(() => {
    /** @param {string[]} changed */
    const f = (changed) => { if (!keys.length || changed.some((k) => keys.includes(k))) bump((n) => n + 1); };
    listeners.add(f);
    return () => { listeners.delete(f); };
  }, []);
  return store;
}

/** @param {string} level @param {string} text @param {string} [source] */
export function note(level, text, source = "page") {
  const events = store.events.concat([{ t: Date.now() / 1000, level, text, source }]).slice(-300);
  update({ events });
}

let ros = /** @type {any} */ (null);
let topics = /** @type {Record<string, any>} */ ({});
let commandSrv = /** @type {any} */ (null);
let hzCount = 0;
const pageId = (() => {
  try {
    let id = sessionStorage.getItem("console.id");
    if (!id) { id = Math.random().toString(36).slice(2, 8); sessionStorage.setItem("console.id", id); }
    return id;
  } catch { return Math.random().toString(36).slice(2, 8); }
})();

export function getRos() { return ros; }

/** rosbridge's URL: ?ros=ws://host:port, else the saved override, else this page's host on the robot's port. */
async function rosbridgeUrl() {
  const q = new URLSearchParams(location.search).get("ros");
  if (q) return q;
  try { const saved = localStorage.getItem("console.ros"); if (saved) return saved; } catch { /* private mode */ }
  let port = 9090;
  try {
    const cfg = await (await fetch("config.json", { cache: "no-store" })).json();
    port = cfg.rosbridge_port || port;
    update({ foxglovePort: cfg.foxglove_port || 8765 });
  } catch { /* opened without the robot's server: defaults */ }
  return `ws://${location.hostname || "localhost"}:${port}`;
}

/** Point the page at another rosbridge ("" = back to the default) and reconnect. @param {string} url */
export function setRosUrl(url) {
  try { url ? localStorage.setItem("console.ros", url) : localStorage.removeItem("console.ros"); } catch { /* */ }
  if (ros) { ros.__dead = true; ros.close(); }
  connect();
}

export async function connect() {
  const url = await rosbridgeUrl();
  update({ conn: "connecting", url });
  const r = new ROSLIB.Ros({ url });
  ros = r;
  r.on("connection", () => { if (!r.__dead) { update({ conn: "open" }); subscribe(r); } });
  r.on("error", () => {});
  r.on("close", () => {
    if (r.__dead) return;
    r.__dead = true;
    update({ conn: "closed", status: null });
    setTimeout(connect, 2000);
  });
}

/** @param {any} r */
function subscribe(r) {
  /** @param {string} name @param {string} type @param {(m: any) => void} cb @param {Record<string, any>} [opts] */
  const sub = (name, type, cb, opts = {}) => {
    const t = new ROSLIB.Topic({ ros: r, name, messageType: type, queue_length: 1, ...opts });
    t.subscribe(cb);
    return t;
  };
  /** @param {string} s */
  const parse = (s) => { try { return JSON.parse(s); } catch { return null; } };
  topics = {
    status: sub("/console/status", "std_msgs/msg/String", (m) => {
      hzCount++;
      const s = parse(m.data);
      if (s?.pose) { poses.push(s.pose); if (poses.length > 40) poses.shift(); }
      update({ status: s, tStatus: performance.now() });
    }),
    schema: sub("/console/schema", "std_msgs/msg/String", (m) => update({ schema: parse(m.data) })),
    version: sub("/console/version", "std_msgs/msg/String", (m) => {
      const v = parse(m.data);
      update({ version: v });
      const scanTopic = v?.topics?.scan;
      if (scanTopic && topics.scan?.name !== scanTopic) {        // the scan topic is the robot's to say
        topics.scan?.unsubscribe();
        topics.scan = sub(scanTopic, "sensor_msgs/msg/LaserScan", (sm) => onScan(sm),
          { compression: "cbor-raw", throttle_rate: SCAN_PERIOD_MS });
      }
    }),
    events: sub("/console/events", "std_msgs/msg/String", (m) => {
      const e = parse(m.data);
      if (e) update({ events: store.events.concat([e]).slice(-300) });
    }, { queue_length: 20 }),
    plan: sub("/console/plan", "std_msgs/msg/String", (m) => update({ plan: parse(m.data) })),
    // the map: the publisher's bytes, at most once a second (slam_toolbox republishes it every few seconds while mapping)
    map: sub("/map", "nav_msgs/msg/OccupancyGrid", (m) => {
      try { update({ map: decodeOccupancyGrid(rawBytes(m)), mapSeq: store.mapSeq + 1 }); } catch { /* a bad frame */ }
    }, { compression: "cbor-raw", throttle_rate: 1000 }),
    scan: null,
    twist: new ROSLIB.Topic({ ros: r, name: "/console/cmd_vel", messageType: "geometry_msgs/msg/Twist", queue_size: 1 }),
    heartbeat: new ROSLIB.Topic({ ros: r, name: "/console/heartbeat", messageType: "std_msgs/msg/String" }),
  };
  commandSrv = new ROSLIB.Service({ ros: r, name: "/console/command", serviceType: "omni_interfaces/srv/ConsoleCommand" });
  command("schema").then((res) => { if (res.ok) update({ schema: res.data }); });
}

/** The robot's pose (map or odom) when the scan started: the status poses either side of its stamp, interpolated.
 *  @param {number} t */
function poseAt(t) {
  if (!poses.length) return null;
  let a = poses[poses.length - 1];
  for (let i = poses.length - 1; i > 0; i--) {
    const p = poses[i], q = poses[i - 1];
    if (q.t <= t && t <= p.t && p.frame === q.frame && p.t > q.t) {
      const k = (t - q.t) / (p.t - q.t);
      const dyaw = Math.atan2(Math.sin(p.yaw - q.yaw), Math.cos(p.yaw - q.yaw));
      return { frame: p.frame, x: q.x + k * (p.x - q.x), y: q.y + k * (p.y - q.y), yaw: q.yaw + k * dyaw };
    }
    if (Math.abs(p.t - t) < Math.abs(a.t - t)) a = p;
  }
  return a;
}

/** A raw LaserScan: its returns as points in the robot's fixed frame. @param {any} m */
function onScan(m) {
  let s;
  try { s = decodeLaserScan(rawBytes(m)); } catch { return; }
  const p = poseAt(s.header.t);
  if (!p) return;
  const lz = store.version?.frames?.laser || [0, 0, 0];
  const c = Math.cos(p.yaw), sn = Math.sin(p.yaw);
  const x0 = p.x + c * lz[0] - sn * lz[1], y0 = p.y + sn * lz[0] + c * lz[1], th = p.yaw + lz[2];
  const xy = [];
  const r = s.ranges;
  for (let i = 0; i < r.length; i++) {
    const d = r[i];
    if (!(d > s.range_min && d < s.range_max)) continue;
    const a = th + s.angle_min + i * s.angle_increment;
    xy.push(x0 + d * Math.cos(a), y0 + d * Math.sin(a));
  }
  update({ scan: { frame: p.frame, xy, t: s.header.t } });
}

/** Listen to one of the robot's JSON topics while a tab is open (and again after a reconnect): the heavier ones
 *  (/console/camera) are only produced while someone listens.
 *  @param {string} name @param {(d: any) => void} cb */
export function useTopic(name, cb) {
  const ref = useRef(cb);
  ref.current = cb;
  const s = useStore("conn");
  useEffect(() => {
    if (!ros || s.conn !== "open") return undefined;
    const t = new ROSLIB.Topic({ ros, name, messageType: "std_msgs/msg/String", queue_length: 1 });
    t.subscribe((/** @type {any} */ m) => { try { ref.current(JSON.parse(m.data)); } catch { /* a bad frame */ } });
    return () => t.unsubscribe();
  }, [s.conn, name]);
}

/** Listen to a topic as the publisher's bytes (cbor-raw) while a tab is open; cb gets a Uint8Array.
 *  @param {string} name @param {string} type @param {(b: Uint8Array) => void} cb @param {number} [throttleMs] */
export function useRawTopic(name, type, cb, throttleMs = 0) {
  const ref = useRef(cb);
  ref.current = cb;
  const s = useStore("conn");
  useEffect(() => {
    if (!ros || s.conn !== "open" || !name) return undefined;
    const t = new ROSLIB.Topic({ ros, name, messageType: type, queue_length: 1, compression: "cbor-raw",
      throttle_rate: throttleMs });
    t.subscribe((/** @type {any} */ m) => { try { ref.current(rawBytes(m)); } catch { /* a bad frame */ } });
    return () => t.unsubscribe();
  }, [s.conn, name]);
}

/** The robot's one command. Resolves to {ok, message, data}; never rejects.
 *  @param {string} verb @param {Record<string, any>} [args] @returns {Promise<{ok: boolean, message: string, data: any}>} */
export function command(verb, args = {}) {
  return new Promise((resolve) => {
    if (!commandSrv || store.conn !== "open") {
      resolve({ ok: false, message: "Not connected to the robot", data: null });
      return;
    }
    let done = false;
    const timer = setTimeout(() => {
      if (!done) { done = true; resolve({ ok: false, message: "The robot didn't answer (is omni_console running?)", data: null }); }
    }, 3000);
    commandSrv.callService(new ROSLIB.ServiceRequest({ verb, args: JSON.stringify(args) }), (/** @type {any} */ res) => {
      if (done) return;
      done = true; clearTimeout(timer);
      let data = null;
      try { data = res.data ? JSON.parse(res.data) : null; } catch { /* */ }
      resolve({ ok: !!res.ok, message: res.message || "", data });
    }, (/** @type {any} */ err) => {
      if (done) return;
      done = true; clearTimeout(timer);
      resolve({ ok: false, message: String(err), data: null });
    });
  });
}

/** Run a command and put its answer in the event list (errors always, successes when asked).
 *  @param {string} verb @param {Record<string, any>} [args] @param {boolean} [quiet] */
export async function act(verb, args = {}, quiet = false) {
  const r = await command(verb, args);
  if (!r.ok) note("error", r.message || `${verb} failed`);
  else if (!quiet && r.message) note("info", r.message);
  return r;
}

/** Publish one JSON message on a std_msgs/String topic (the sim's tools: /sim/fault).
 *  @param {string} name @param {any} obj */
export function publishJson(name, obj) {
  if (!ros || store.conn !== "open") return false;
  new ROSLIB.Topic({ ros, name, messageType: "std_msgs/msg/String" }).publish(new ROSLIB.Message({ data: JSON.stringify(obj) }));
  return true;
}

/** The console's stick: -1..1 per axis (forward, left, turn left), boost. Sent while the dead man is held.
 *  @param {number} vx @param {number} vy @param {number} wz @param {boolean} boost */
export function sendTwist(vx, vy, wz, boost) {
  if (!topics.twist || store.conn !== "open") return;
  topics.twist.publish(new ROSLIB.Message({ linear: { x: vx, y: vy, z: boost ? 1 : 0 }, angular: { x: 0, y: 0, z: wz } }));
}

// for the browser's dev tools and ros/tools/console_smoke.py
/** @type {any} */ (window).omniConsole = { store, command, getRos, publishJson: (/** @type {string} */ n, /** @type {any} */ o) => publishJson(n, o) };

// once a second: the heartbeat (the robot counts open consoles), the status rate and the round trip (ping)
setInterval(() => {
  update({ statusHz: hzCount });
  hzCount = 0;
  if (store.conn !== "open") return;
  topics.heartbeat?.publish(new ROSLIB.Message({ data: JSON.stringify({ id: pageId }) }));
}, 1000);
setInterval(async () => {
  if (store.conn !== "open") return;
  const t0 = performance.now();
  const r = await command("ping");
  update({ rtt: r.ok ? Math.round(performance.now() - t0) : null });
}, 2000);
