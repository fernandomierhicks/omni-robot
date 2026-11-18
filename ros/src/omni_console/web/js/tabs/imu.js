// @ts-check
/* Test > IMU: the BNO085 as the robot sees it. mcu_link's /imu (the MCU forwards every sample, 100 Hz, each with its own
   time), read here as the publisher's bytes (cbor-raw, cdr.js decodeImu) at most every THROTTLE_MS while the page is
   open. A box drawn from the game rotation vector turns with the IMU (drag to look from another side; "Zero yaw" turns
   the view, not the robot's heading); charts of attitude, angular rate and acceleration (with gravity), the last 10 s.
   The IMU's health is the MCU's IMU_SILENT fault (no sample for 50 ms), as on the preflight's IMU row. */
import { html, useEffect, useRef, useState } from "../../vendor/htm-preact-standalone.mjs";
import { decodeImu } from "../cdr.js";
import { Chart, makeBuffer, push } from "../chart.js";
import { useRawTopic, useStore } from "../ros.js";
import { Banner, HelpBox } from "../ui.js";

const THROTTLE_MS = 20;                  // the page takes /imu at up to 50 Hz (the robot publishes 100)
const DEG = 180 / Math.PI;
const G = 9.80665;

const css = (/** @type {string} */ v) => getComputedStyle(document.documentElement).getPropertyValue(v).trim();

/** w x y z -> roll, pitch, yaw (rad; intrinsic Z-Y-X, REP-103: x forward, y left, z up). @param {number[]} q */
export function euler(q) {
  const [w, x, y, z] = q;
  const roll = Math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
  const pitch = Math.asin(Math.max(-1, Math.min(1, 2 * (w * y - z * x))));
  const yaw = Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
  return [roll, pitch, yaw];
}

/** Rotate v by the unit quaternion q (w x y z). @param {number[]} q @param {number[]} v */
function rotate(q, v) {
  const [w, x, y, z] = q;
  const tx = 2 * (y * v[2] - z * v[1]), ty = 2 * (z * v[0] - x * v[2]), tz = 2 * (x * v[1] - y * v[0]);
  return [v[0] + w * tx + (y * tz - z * ty), v[1] + w * ty + (z * tx - x * tz), v[2] + w * tz + (x * ty - y * tx)];
}

/** '#rrggbb' * k (0..) -> rgb(). @param {string} hex @param {number} k */
function shade(hex, k) {
  const n = parseInt((hex || "#888888").replace("#", ""), 16);
  const c = (/** @type {number} */ s) => Math.round(Math.min(255, ((n >> s) & 255) * k));
  return `rgb(${c(16)}, ${c(8)}, ${c(0)})`;
}

/** The last sample, 10 s of charts and the page's receive rate. */
function useImu() {
  const last = useRef(/** @type {any} */ (null));
  const bufs = useRef({ att: makeBuffer(3), gyro: makeBuffer(3), acc: makeBuffer(4) }).current;
  const count = useRef(0);
  const [hz, setHz] = useState(0);
  useRawTopic("/imu", "sensor_msgs/msg/Imu", (bytes) => {
    const m = decodeImu(bytes);
    const t = Number.isFinite(m.header.t) && m.header.t > 0 ? m.header.t : performance.now() / 1000;
    const [r, p, y] = euler(m.quat);
    push(bufs.att, t, [r * DEG, p * DEG, y * DEG]);
    push(bufs.gyro, t, m.gyro.map((g) => g * DEG));
    push(bufs.acc, t, [...m.accel, Math.hypot(...m.accel)]);
    last.current = { ...m, t, rpy: [r, p, y], at: performance.now() };
    count.current++;
  }, THROTTLE_MS);
  useEffect(() => {
    const id = setInterval(() => { setHz(count.current); count.current = 0; }, 1000);
    return () => clearInterval(id);
  }, []);
  return { last, bufs, hz };
}

// the box: the robot's base, x forward (long), y left, z up; drawn about its centre
const BOX = [0.8, 0.5, 0.16];
const VERTS = [-1, 1].flatMap((sx) => [-1, 1].flatMap((sy) => [-1, 1].map((sz) => [sx * BOX[0], sy * BOX[1], sz * BOX[2]])));
const vi = (/** @type {number} */ sx, /** @type {number} */ sy, /** @type {number} */ sz) => (sx > 0 ? 4 : 0) + (sy > 0 ? 2 : 0) + (sz > 0 ? 1 : 0);
const FACES = [                          // corners counter-clockwise seen from outside, outward normal, which face
  { v: [vi(1, -1, -1), vi(1, 1, -1), vi(1, 1, 1), vi(1, -1, 1)], n: [1, 0, 0], kind: "front" },
  { v: [vi(-1, -1, -1), vi(-1, -1, 1), vi(-1, 1, 1), vi(-1, 1, -1)], n: [-1, 0, 0], kind: "side" },
  { v: [vi(-1, 1, -1), vi(-1, 1, 1), vi(1, 1, 1), vi(1, 1, -1)], n: [0, 1, 0], kind: "side" },
  { v: [vi(-1, -1, -1), vi(1, -1, -1), vi(1, -1, 1), vi(-1, -1, 1)], n: [0, -1, 0], kind: "side" },
  { v: [vi(-1, -1, 1), vi(1, -1, 1), vi(1, 1, 1), vi(-1, 1, 1)], n: [0, 0, 1], kind: "top" },
  { v: [vi(-1, -1, -1), vi(-1, 1, -1), vi(1, 1, -1), vi(1, -1, -1)], n: [0, 0, -1], kind: "bottom" },
];

/** The box turned by the IMU's orientation, in perspective, redrawn every frame from the last sample.
 *  @param {{last: {current: any}, yaw0: number}} p */
function Box3D({ last, yaw0 }) {
  const cv = useRef(/** @type {HTMLCanvasElement | null} */ (null));
  const view = useRef({ az: -2.4, el: 0.42, drag: /** @type {null | {x: number, y: number, az: number, el: number}} */ (null) });
  const yawRef = useRef(yaw0);
  yawRef.current = yaw0;
  useEffect(() => {
    const c = /** @type {HTMLCanvasElement} */ (cv.current);
    let raf = 0;
    const draw = () => {
      raf = requestAnimationFrame(draw);
      const dpr = window.devicePixelRatio || 1, w = c.clientWidth, h = c.clientHeight;
      if (!w || !h) return;
      if (c.width !== Math.round(w * dpr) || c.height !== Math.round(h * dpr)) { c.width = Math.round(w * dpr); c.height = Math.round(h * dpr); }
      const ctx = /** @type {CanvasRenderingContext2D} */ (c.getContext("2d"));
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      ctx.clearRect(0, 0, w, h);
      // the camera: on a sphere round the origin, looking at it, z up
      const { az, el } = view.current, R = 3.4;
      const cam = [R * Math.cos(el) * Math.cos(az), R * Math.cos(el) * Math.sin(az), R * Math.sin(el)];
      const f = cam.map((v) => -v / R);
      const rt = [f[1], -f[0], 0].map((v, _, a) => v / Math.hypot(a[0], a[1]));
      const up = [rt[1] * f[2] - rt[2] * f[1], rt[2] * f[0] - rt[0] * f[2], rt[0] * f[1] - rt[1] * f[0]];
      const fpx = Math.min(w, h) * 1.25;
      /** @param {number[]} p */
      const proj = (p) => {
        const d = [p[0] - cam[0], p[1] - cam[1], p[2] - cam[2]];
        const zc = d[0] * f[0] + d[1] * f[1] + d[2] * f[2];
        return [w / 2 + fpx * (d[0] * rt[0] + d[1] * rt[1] + d[2] * rt[2]) / zc,
          h / 2 - fpx * (d[0] * up[0] + d[1] * up[1] + d[2] * up[2]) / zc, zc];
      };
      // the floor: a grid under the box, and the world's x (forward at zero yaw) as a faint arrow
      ctx.lineWidth = 1;
      ctx.strokeStyle = css("--map-grid") || "#222240";
      const zf = -0.75;
      for (let k = -1.5; k <= 1.5001; k += 0.5) {
        let a = proj([k, -1.5, zf]), b = proj([k, 1.5, zf]);
        ctx.beginPath(); ctx.moveTo(a[0], a[1]); ctx.lineTo(b[0], b[1]); ctx.stroke();
        a = proj([-1.5, k, zf]); b = proj([1.5, k, zf]);
        ctx.beginPath(); ctx.moveTo(a[0], a[1]); ctx.lineTo(b[0], b[1]); ctx.stroke();
      }
      const s = last.current;
      if (!s) {
        ctx.fillStyle = css("--dim");
        ctx.font = "14px " + css("--sans");
        ctx.textAlign = "center";
        ctx.fillText("waiting for /imu", w / 2, h / 2);
        return;
      }
      // the orientation, the view's yaw zero taken out (a turn about the world's z, first)
      const c0 = Math.cos(-yawRef.current / 2), s0 = Math.sin(-yawRef.current / 2);
      const [qw, qx, qy, qz] = s.quat;
      const q = [c0 * qw - s0 * qz, c0 * qx - s0 * qy, c0 * qy + s0 * qx, c0 * qz + s0 * qw];
      const P = VERTS.map((v) => rotate(q, v));
      const S = P.map(proj);
      // the shadow on the floor (straight down)
      ctx.fillStyle = "rgba(0, 0, 0, 0.35)";
      for (const fc of FACES) {
        ctx.beginPath();
        fc.v.forEach((k, i) => { const p = proj([P[k][0], P[k][1], zf]); i ? ctx.lineTo(p[0], p[1]) : ctx.moveTo(p[0], p[1]); });
        ctx.fill();
      }
      // faces facing the camera, far to near, lit from above
      const light = [0.35, -0.45, 0.82];
      const vis = FACES.map((fc) => {
        const n = rotate(q, fc.n);
        const ctr = [0, 1, 2].map((a) => fc.v.reduce((acc, k) => acc + P[k][a], 0) / 4);
        const toCam = ctr.map((v, a) => cam[a] - v);
        return { fc, n, depth: proj(ctr)[2], facing: n[0] * toCam[0] + n[1] * toCam[1] + n[2] * toCam[2] > 0 };
      }).filter((x) => x.facing).sort((a, b) => b.depth - a.depth);
      const base = { top: css("--blue"), front: css("--orange"), side: css("--surface-2") || "#222238", bottom: css("--border") };
      for (const { fc, n } of vis) {
        const lam = 0.45 + 0.75 * Math.max(0, n[0] * light[0] + n[1] * light[1] + n[2] * light[2]);
        ctx.fillStyle = shade(/** @type {any} */ (base)[fc.kind], fc.kind === "side" ? lam * 1.6 : lam);
        ctx.strokeStyle = css("--text");
        ctx.lineWidth = 1;
        ctx.beginPath();
        fc.v.forEach((k, i) => (i ? ctx.lineTo(S[k][0], S[k][1]) : ctx.moveTo(S[k][0], S[k][1])));
        ctx.closePath();
        ctx.fill();
        ctx.stroke();
        if (fc.kind === "top") {               // an arrow on the top: forward
          const a = [[-0.35, 0.22], [0.5, 0], [-0.35, -0.22], [-0.15, 0]].map(([x, y]) => proj(rotate(q, [x, y, BOX[2] + 0.001])));
          ctx.fillStyle = css("--white");
          ctx.beginPath();
          a.forEach((p, i) => (i ? ctx.lineTo(p[0], p[1]) : ctx.moveTo(p[0], p[1])));
          ctx.closePath();
          ctx.fill();
        }
      }
      // the body's axes from its centre: x red, y green, z blue
      const o = proj([0, 0, 0]);
      for (const [v, col, lbl] of /** @type {[number[], string, string][]} */ ([[[1.25, 0, 0], "--red", "x"], [[0, 0.95, 0], "--green", "y"], [[0, 0, 0.75], "--blue", "z"]])) {
        const e = proj(rotate(q, v));
        ctx.strokeStyle = ctx.fillStyle = css(col);
        ctx.lineWidth = 2.5;
        ctx.beginPath(); ctx.moveTo(o[0], o[1]); ctx.lineTo(e[0], e[1]); ctx.stroke();
        ctx.font = "bold 13px " + css("--sans");
        ctx.fillText(lbl, e[0] + 4, e[1] - 4);
      }
    };
    raf = requestAnimationFrame(draw);
    return () => cancelAnimationFrame(raf);
  }, []);
  /** @param {any} e */
  const onDown = (e) => {
    e.currentTarget.setPointerCapture(e.pointerId);
    view.current.drag = { x: e.clientX, y: e.clientY, az: view.current.az, el: view.current.el };
  };
  /** @param {any} e */
  const onMove = (e) => {
    const d = view.current.drag;
    if (!d) return;
    view.current.az = d.az - (e.clientX - d.x) * 0.01;
    view.current.el = Math.max(-1.2, Math.min(1.45, d.el + (e.clientY - d.y) * 0.01));
  };
  const onUp = () => { view.current.drag = null; };
  return html`<canvas ref=${cv} style="width: 100%; height: 360px; display: block; border-radius: 6px; background: var(--map-bg); cursor: grab; touch-action: none"
    onPointerDown=${onDown} onPointerMove=${onMove} onPointerUp=${onUp} onPointerCancel=${onUp}></canvas>`;
}

/** @param {number[] | undefined} v @param {number} k @param {number} dp */
const fmt3 = (v, k, dp) => (v ? v.map((x) => (x * k).toFixed(dp).padStart(7)).join(" ") : "—");

export function ImuTab() {
  const s = useStore("status", "conn");
  const { last, bufs, hz } = useImu();
  const [, tick] = useState(0);
  const [yaw0, setYaw0] = useState(0);
  useEffect(() => {                      // the readouts at 5 Hz (the box and charts draw every frame)
    const id = setInterval(() => tick((n) => n + 1), 200);
    return () => clearInterval(id);
  }, []);
  const m = s.status?.mcu;
  const silent = (m?.faults || []).some((/** @type {any} */ f) => (f.name ?? f) === "IMU_SILENT");
  const x = last.current;
  const age = x ? (performance.now() - x.at) / 1000 : Infinity;
  const live = age < 1;
  const rpy = x?.rpy;
  const acc = x ? Math.hypot(...x.accel) : NaN;
  return html`<div class="cols">
    <div class="side">
      <${HelpBox} tab="imu">
        The BNO085 as the robot sees it: <b>mcu_link</b>'s <span class="mono">/imu</span> (the MCU forwards every
        sample at 100 Hz, each stamped with its own time). This page takes it at up to 50 Hz while it is open.
        <ul>
          <li>The <b>box</b> turns with the IMU's orientation (the game rotation vector: gyro and accelerometer, no
            magnetometer, so its yaw starts at 0 when the MCU starts and drifts slowly). Orange is the front, the arrow
            points forward; axes x red, y green, z blue (REP-103: x forward, y left, z up). Drag it to look from
            another side.</li>
          <li><b>Zero yaw</b> turns the view so the box points forward now; it doesn't change the robot's heading.</li>
          <li>Charts: roll, pitch and yaw; angular rate; acceleration with gravity (at rest |a| ≈ 9.81 m/s²), the last
            10 s.</li>
        </ul>
        Tilt the board by hand: roll and pitch should follow, and the box with them.
      <//>
      ${!m ? html`<${Banner} title="No MCU state.">Start the robot's software (Robot › System) so mcu_link reads the MCU.<//>`
        : silent ? html`<${Banner} red title="IMU silent.">The MCU has had no BNO085 sample for 50 ms (IMU_SILENT): check
            its wiring and power. The firmware keeps trying to bring it back by itself.<//>` : null}
      <div class="card">
        <b class=${live && !silent ? "ok" : "warn"}>${live ? (silent ? "Samples, but the MCU says silent" : "IMU reporting")
          : "No /imu samples"}</b>
        <div class="dim" style="font-size: 13px; margin-top: 4px">
          ${hz} samples/s on this page (at most ${Math.round(1000 / THROTTLE_MS)}) · last ${live ? `${(age * 1000).toFixed(0)} ms` : "—"} ago
          ${x ? html`<br />frame <span class="mono">${x.header.frame}</span>` : null}
          ${m ? html`<br />MCU ${m.mode}${m.faults?.length ? ` · ${m.faults.map((/** @type {any} */ f) => f.name ?? f).join(", ")}` : ""}` : null}
        </div>
      </div>
      <div class="card mono" style="font-size: 13px; white-space: pre; line-height: 1.6">${
`              x       y       z
gyro °/s ${fmt3(x?.gyro, DEG, 2)}
acc m/s² ${fmt3(x?.accel, 1, 2)}
|a|      ${Number.isFinite(acc) ? `${acc.toFixed(3)} m/s² (${(acc / G).toFixed(3)} g)` : "—"}

          roll   pitch     yaw
deg  ${fmt3(rpy, DEG, 1)}
quat w ${x ? x.quat[0].toFixed(4) : "—"}  x ${x ? x.quat[1].toFixed(4) : "—"}
     y ${x ? x.quat[2].toFixed(4) : "—"}  z ${x ? x.quat[3].toFixed(4) : "—"}`}</div>
      <div class="seg">
        <button class="btn" disabled=${!rpy} onClick=${() => rpy && setYaw0(rpy[2])}
          title="Turn the view so the box points forward now (the robot's heading is unchanged)">Zero yaw</button>
        <button class="btn" disabled=${!yaw0} onClick=${() => setYaw0(0)}>Yaw as reported</button>
      </div>
    </div>
    <div class="main full">
      <div class="card">
        <h2>Orientation <span class="dim" style="font-weight: 400; font-size: 13px">drag to turn the view${yaw0
          ? ` · view yaw zeroed at ${(yaw0 * DEG).toFixed(1)}°` : ""}</span></h2>
        <${Box3D} last=${last} yaw0=${yaw0} />
      </div>
      <div class="charts">
        <div class="card"><${Chart} title="Attitude" unit="deg" span=${10} buf=${bufs.att} series=${[
          { label: "roll", color: "red" }, { label: "pitch", color: "green" }, { label: "yaw", color: "blue" }]} /></div>
        <div class="card"><${Chart} title="Angular rate" unit="°/s" span=${10} buf=${bufs.gyro} series=${[
          { label: "x", color: "red" }, { label: "y", color: "green" }, { label: "z", color: "blue" }]} /></div>
        <div class="card"><${Chart} title="Acceleration (with gravity)" unit="m/s²" span=${4} buf=${bufs.acc} series=${[
          { label: "x", color: "red" }, { label: "y", color: "green" }, { label: "z", color: "blue" },
          { label: "|a|", color: "orange", dash: true }]} /></div>
      </div>
    </div>
  </div>`;
}
