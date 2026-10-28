// @ts-check
/* One throw, drawn in the browser from its record (omni_catch game.py: the path the robot tracked at 20 Hz, where it
   was released, where it would cross the funnel, the robot): a side view (height over distance along the throw) and
   a top view, replayed over its flight time. */
import { html, useEffect, useRef } from "../vendor/htm-preact-standalone.mjs";

/** @param {{rec: any, height?: number}} p */
export function ThrowView({ rec, height = 220 }) {
  const cv = useRef(/** @type {HTMLCanvasElement | null} */ (null));
  useEffect(() => {
    const c = cv.current;
    if (!c || !rec) return undefined;
    const css = getComputedStyle(document.documentElement);
    const tok = (/** @type {string} */ k) => css.getPropertyValue(`--${k}`).trim();
    const path = (rec.path || []).filter((/** @type {number[]} */ q) => q.every((v) => v != null));
    const rel = rec.release, pred = rec.predicted, rob = rec.robot;
    const pts = path.map((/** @type {number[]} */ q) => q.slice(1));
    if (rel) pts.unshift(rel);
    if (!pts.length) return undefined;
    const end = pred || pts[pts.length - 1];
    const a = pts[0];
    const ux0 = end[0] - a[0], uy0 = end[1] - a[1], L = Math.hypot(ux0, uy0) || 1;
    const ux = ux0 / L, uy = uy0 / L;
    const along = (/** @type {number[]} */ q) => (q[0] - a[0]) * ux + (q[1] - a[1]) * uy;
    const W = c.parentElement?.clientWidth || 500;
    const dpr = window.devicePixelRatio || 1;
    c.width = W * dpr; c.height = height * dpr; c.style.width = `${W}px`; c.style.height = `${height}px`;
    const ctx = /** @type {CanvasRenderingContext2D} */ (c.getContext("2d"));
    const sideW = W * 0.62, topX = sideW + 10, topW = W - topX;
    const maxD = Math.max(1, ...pts.map(along), along(end)) * 1.05;
    const maxZ = Math.max(1.5, ...pts.map((q) => q[2])) * 1.1;
    const k = Math.min((sideW - 40) / maxD, (height - 40) / maxZ);
    const SX = (/** @type {number} */ d) => 30 + d * k, SY = (/** @type {number} */ z) => height - 20 - z * k;
    let raf = 0, t0 = performance.now();
    const dur = Math.max(0.3, path.length ? path[path.length - 1][0] : 1);
    const draw = () => {
      const now = ((performance.now() - t0) / 1000) % (dur + 1.2);
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      ctx.fillStyle = tok("map-bg"); ctx.fillRect(0, 0, W, height);
      ctx.strokeStyle = tok("border"); ctx.lineWidth = 1;
      ctx.beginPath(); ctx.moveTo(20, SY(0)); ctx.lineTo(sideW, SY(0)); ctx.stroke();
      ctx.fillStyle = tok("dim"); ctx.font = "11px Segoe UI, system-ui, sans-serif";
      for (let d = 0; d <= maxD; d += 1) { ctx.fillText(`${d} m`, SX(d) - 8, height - 4); }
      ctx.fillText("side view", 30, 14);
      // the path, drawn up to the replay time; the ball at that time
      ctx.strokeStyle = tok("map-ball"); ctx.lineWidth = 2;
      ctx.beginPath();
      let ball = null;
      path.forEach((/** @type {number[]} */ q, /** @type {number} */ i) => {
        if (q[0] > now) return;
        const x = SX(along(q.slice(1))), y = SY(q[3]);
        i ? ctx.lineTo(x, y) : ctx.moveTo(x, y);
        ball = [x, y];
      });
      ctx.stroke();
      if (rel) { ctx.fillStyle = tok("blue"); ctx.beginPath(); ctx.arc(SX(0), SY(rel[2]), 4, 0, 2 * Math.PI); ctx.fill(); }
      if (pred) { ctx.strokeStyle = tok("green"); ctx.beginPath(); ctx.arc(SX(along(pred)), SY(pred[2] ?? 0.5), 6, 0, 2 * Math.PI); ctx.stroke(); }
      if (ball) { ctx.fillStyle = tok("yellow"); ctx.beginPath(); ctx.arc(ball[0], ball[1], 5, 0, 2 * Math.PI); ctx.fill(); }
      // the top view: the field around the throw
      const xs = pts.map((q) => q[0]).concat(rob ? [rob[0]] : [], [end[0]]);
      const ys = pts.map((q) => q[1]).concat(rob ? [rob[1]] : [], [end[1]]);
      const cx = (Math.min(...xs) + Math.max(...xs)) / 2, cy = (Math.min(...ys) + Math.max(...ys)) / 2;
      const span = Math.max(2, Math.max(...xs) - Math.min(...xs), Math.max(...ys) - Math.min(...ys)) * 1.2;
      const kt = Math.min(topW, height - 20) / span;
      const TX = (/** @type {number} */ x, /** @type {number} */ y) => [topX + topW / 2 + (x - cx) * kt, height / 2 + 6 - (y - cy) * kt];
      ctx.strokeStyle = tok("border"); ctx.strokeRect(topX, 18, topW - 2, height - 28);
      ctx.fillStyle = tok("dim"); ctx.fillText("top view", topX + 4, 14);
      ctx.strokeStyle = tok("map-ball"); ctx.lineWidth = 1.5; ctx.beginPath();
      pts.forEach((q, i) => { const [x, y] = TX(q[0], q[1]); i ? ctx.lineTo(x, y) : ctx.moveTo(x, y); });
      ctx.stroke();
      if (rob) {
        const [x, y] = TX(rob[0], rob[1]);
        ctx.strokeStyle = tok("map-robot"); ctx.lineWidth = 2;
        ctx.beginPath(); ctx.arc(x, y, Math.max(4, 0.217 * kt), 0, 2 * Math.PI); ctx.stroke();
      }
      if (pred) { const [x, y] = TX(pred[0], pred[1]); ctx.strokeStyle = tok("green"); ctx.beginPath(); ctx.arc(x, y, 5, 0, 2 * Math.PI); ctx.stroke(); }
      raf = requestAnimationFrame(draw);
    };
    raf = requestAnimationFrame(draw);
    return () => cancelAnimationFrame(raf);
  }, [rec]);
  if (!rec) return html`<p class="dim">No throw yet.</p>`;
  return html`<div><canvas ref=${cv} style="display: block; border-radius: 6px"></canvas>
    <div class="dim" style="font-size: 12px; margin-top: 4px">Blue: released. Yellow: the ball as the robot tracked it.
      Green: where it would cross the funnel. Green ring: the robot.</div></div>`;
}

/** @param {number | null | undefined} v m/s */
export const kmh = (v) => (v == null ? "–" : `${(v * 3.6).toFixed(0)} km/h`);
