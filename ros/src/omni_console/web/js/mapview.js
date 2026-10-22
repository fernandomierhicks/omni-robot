// @ts-check
/* The map: slam_toolbox's occupancy grid, the lidar, Nav2's path, places, goals and the robot, drawn on a canvas only
   when something changed (requestAnimationFrame). World frame is the robot's fixed frame (map, or odom without one).

   Mouse: wheel zooms; right-drag (or Shift-drag, or any drag when the map isn't for goals) pans.
   Goal mode: press where the robot should go, drag to set its heading, release. Right-click clears.
   Touch: one finger as the left button, two fingers pan and pinch-zoom. */
import { html, useEffect, useRef, useState } from "../vendor/htm-preact-standalone.mjs";
import { store, useStore } from "./ros.js";

const ROBOT_R = 0.217;              // m: the bumper ring (cad/v7)

/** @typedef {{x: number, y: number, yaw: number}} Pose */

/**
 * @param {{
 *   goalMode?: boolean,
 *   pending?: Pose | null,
 *   onGoal?: (p: Pose) => void,
 *   onClear?: () => void,
 *   places?: Record<string, number[]>,
 *   hint?: string,
 *   empty?: string,
 *   catchView?: boolean,
 *   children?: any,
 * }} p
 */
export function MapView(p) {
  const wrap = useRef(/** @type {HTMLDivElement | null} */ (null));
  const canvas = useRef(/** @type {HTMLCanvasElement | null} */ (null));
  const props = useRef(p);
  props.current = p;
  const [follow, setFollow] = useState(true);
  const view = useRef({ cx: 0, cy: 0, scale: 50, fitted: false, follow: true, dirty: true, mapSeq: -1,
    img: /** @type {HTMLCanvasElement | null} */ (null), w: 0, h: 0, drag: /** @type {any} */ (null),
    pointers: new Map(), colors: /** @type {Record<string, string>} */ ({}) });
  view.current.follow = follow;
  const s = useStore("map", "status");

  useEffect(() => {
    const v = view.current;
    const cv = /** @type {HTMLCanvasElement} */ (canvas.current);
    const css = getComputedStyle(document.documentElement);
    for (const k of ["map-bg", "map-unknown", "map-free", "map-wall", "map-grid", "map-scan", "map-robot", "map-plan",
      "map-goal", "map-pending", "map-place", "map-ball", "text", "dim", "blue", "surface"]) {
      v.colors[k] = css.getPropertyValue(`--${k}`).trim();
    }
    const ro = new ResizeObserver(() => {
      const r = cv.getBoundingClientRect();
      const dpr = window.devicePixelRatio || 1;
      cv.width = Math.max(1, Math.round(r.width * dpr));
      cv.height = Math.max(1, Math.round(r.height * dpr));
      v.w = r.width; v.h = r.height; v.dirty = true;
    });
    ro.observe(cv);
    // redraw when the robot sent something new (a reference check per frame) or the view moved
    let raf = 0;
    let last = { map: null, scan: null, plan: null, status: null };
    const loop = () => {
      if (last.map !== store.map || last.scan !== store.scan || last.plan !== store.plan || last.status !== store.status) {
        last = { map: store.map, scan: store.scan, plan: store.plan, status: store.status };
        v.dirty = true;
      }
      if (v.dirty) { v.dirty = false; draw(cv, v, props.current); }
      raf = requestAnimationFrame(loop);
    };
    raf = requestAnimationFrame(loop);
    return () => { cancelAnimationFrame(raf); ro.disconnect(); };
  }, []);

  useEffect(() => { view.current.dirty = true; });

  // -- input ---------------------------------------------------------------------------------------------------
  /** @param {number} sx @param {number} sy */
  const toWorld = (sx, sy) => {
    const v = view.current;
    return { x: v.cx + (sx - v.w / 2) / v.scale, y: v.cy - (sy - v.h / 2) / v.scale };
  };
  /** @param {PointerEvent} e */
  const local = (e) => {
    const r = /** @type {HTMLCanvasElement} */ (canvas.current).getBoundingClientRect();
    return { sx: e.clientX - r.left, sy: e.clientY - r.top };
  };
  /** @param {any} e */
  const down = (e) => {
    const v = view.current;
    const cv = /** @type {HTMLCanvasElement} */ (canvas.current);
    cv.setPointerCapture(e.pointerId);
    const { sx, sy } = local(e);
    v.pointers.set(e.pointerId, { sx, sy });
    if (v.pointers.size === 2) {            // second finger: pinch / pan, drop any goal in progress
      v.drag = { kind: "pinch", ...pinchState(v) };
      return;
    }
    const goal = props.current.goalMode && e.button === 0 && !e.shiftKey;
    if (goal) {
      const w = toWorld(sx, sy);
      v.drag = { kind: "goal", x: w.x, y: w.y, sx, sy, yaw: null };
    } else if (e.button === 2 && props.current.goalMode) {
      v.drag = { kind: "pan", sx, sy, moved: false, clear: true };
    } else {
      v.drag = { kind: "pan", sx, sy, moved: false };
    }
  };
  /** @param {any} e */
  const move = (e) => {
    const v = view.current;
    if (!v.pointers.has(e.pointerId)) return;
    const { sx, sy } = local(e);
    v.pointers.set(e.pointerId, { sx, sy });
    const d = v.drag;
    if (!d) return;
    if (d.kind === "pinch" && v.pointers.size >= 2) {
      const now = pinchState(v);
      v.scale = clampScale(d.scale * now.dist / Math.max(1, d.dist));
      const w = d.world;
      v.cx = w.x - (now.mx - v.w / 2) / v.scale;
      v.cy = w.y + (now.my - v.h / 2) / v.scale;
      stopFollow();
    } else if (d.kind === "pan") {
      if (Math.hypot(sx - d.sx, sy - d.sy) > 3) d.moved = true;
      v.cx -= (sx - d.sx) / v.scale; v.cy += (sy - d.sy) / v.scale;
      d.sx = sx; d.sy = sy;
      if (d.moved) stopFollow();
    } else if (d.kind === "goal") {
      if (Math.hypot(sx - d.sx, sy - d.sy) > 12) {
        const w = toWorld(sx, sy);
        d.yaw = Math.atan2(w.y - d.y, w.x - d.x);
      }
    }
    v.dirty = true;
  };
  /** @param {any} e */
  const up = (e) => {
    const v = view.current;
    v.pointers.delete(e.pointerId);
    const d = v.drag;
    if (d && d.kind === "goal" && v.pointers.size === 0) {
      let yaw = d.yaw;
      if (yaw === null) {                  // a plain click: face the way it will drive
        const r = store.status?.pose;
        yaw = r ? Math.atan2(d.y - r.y, d.x - r.x) : 0;
      }
      props.current.onGoal?.({ x: d.x, y: d.y, yaw });
    } else if (d && d.kind === "pan" && d.clear && !d.moved) {
      props.current.onClear?.();
    }
    v.drag = v.pointers.size >= 2 ? { kind: "pinch", ...pinchState(v) } : null;
    v.dirty = true;
  };
  /** @param {any} e */
  const wheel = (e) => {
    e.preventDefault();
    const v = view.current;
    const { sx, sy } = local(e);
    const w = toWorld(sx, sy);
    v.scale = clampScale(v.scale * Math.exp(-e.deltaY * 0.0015));
    v.cx = w.x - (sx - v.w / 2) / v.scale;
    v.cy = w.y + (sy - v.h / 2) / v.scale;
    stopFollow();
    v.dirty = true;
  };
  const stopFollow = () => { if (view.current.follow) { view.current.follow = false; setFollow(false); } };
  const fit = () => { view.current.fitted = false; view.current.dirty = true; setFollow(false); view.current.follow = false; fitMap(view.current); };
  /** @param {number} k */
  const zoom = (k) => { const v = view.current; v.scale = clampScale(v.scale * k); v.dirty = true; };

  const hasPose = !!s.status?.pose;
  return html`<div class="map" ref=${wrap}>
    <canvas ref=${canvas} onPointerDown=${down} onPointerMove=${move} onPointerUp=${up} onPointerCancel=${up}
      onWheel=${wheel} onContextMenu=${(/** @type {any} */ e) => e.preventDefault()}></canvas>
    ${p.hint && html`<div class="hintline">${p.hint}</div>`}
    ${!hasPose && html`<div class="empty">${p.empty || "Waiting for the robot's position..."}</div>`}
    <div class="tools">
      <button class=${follow ? "btn on" : "btn"} title="Keep the robot in the middle"
        onClick=${() => { const n = !follow; setFollow(n); view.current.follow = n; view.current.dirty = true; }}>Follow</button>
      <button class="btn" title="Show the whole map" onClick=${fit} disabled=${!s.map}>Fit</button>
      <button class="btn" title="Zoom in" onClick=${() => zoom(1.4)}>+</button>
      <button class="btn" title="Zoom out" onClick=${() => zoom(1 / 1.4)}>−</button>
    </div>
    ${p.children}
  </div>`;
}

/** @param {number} s */
const clampScale = (s) => Math.min(600, Math.max(5, s));

/** @param {any} v */
function pinchState(v) {
  const [a, b] = [...v.pointers.values()];
  const mx = (a.sx + b.sx) / 2, my = (a.sy + b.sy) / 2;
  return { dist: Math.hypot(a.sx - b.sx, a.sy - b.sy), scale: v.scale, mx, my,
    world: { x: v.cx + (mx - v.w / 2) / v.scale, y: v.cy - (my - v.h / 2) / v.scale } };
}

/** @param {any} v */
function fitMap(v) {
  const m = store.map;
  if (!m || !v.w) return false;
  const i = m.info;
  const W = i.width * i.resolution, H = i.height * i.resolution;
  v.cx = i.origin.position.x + W / 2;
  v.cy = i.origin.position.y + H / 2;
  v.scale = clampScale(0.92 * Math.min(v.w / W, v.h / H));
  v.fitted = true;
  return true;
}

/** The grid as an image, built once per map message, in the theme's unknown / free / wall colours.
 *  @param {any} m @param {Record<string, string>} c */
function mapImage(m, c) {
  const { width: w, height: h } = m.info;
  const cv = document.createElement("canvas");
  cv.width = w; cv.height = h;
  const ctx = /** @type {CanvasRenderingContext2D} */ (cv.getContext("2d"));
  const img = ctx.createImageData(w, h);
  const rgb = (/** @type {string} */ hex) => [1, 3, 5].map((k) => parseInt(hex.slice(k, k + 2), 16));
  const free = rgb(c["map-free"]), wall = rgb(c["map-wall"]), unk = rgb(c["map-unknown"]);
  const data = m.data;
  for (let j = 0; j < h; j++) {
    const row = (h - 1 - j) * w;           // grid row 0 is at the origin (bottom): flip
    for (let i = 0; i < w; i++) {
      const v = data[j * w + i];
      const o = (row + i) * 4;
      const col = v < 0 ? unk : v >= 65 ? wall : free;
      img.data[o] = col[0]; img.data[o + 1] = col[1]; img.data[o + 2] = col[2]; img.data[o + 3] = 255;
    }
  }
  ctx.putImageData(img, 0, 0);
  return cv;
}

/** @param {HTMLCanvasElement} cv @param {any} v @param {any} p */
function draw(cv, v, p) {
  const ctx = /** @type {CanvasRenderingContext2D} */ (cv.getContext("2d"));
  const dpr = window.devicePixelRatio || 1;
  const c = v.colors;
  const st = store.status;
  const pose = st?.pose;
  const frame = pose?.frame || "map";
  if (store.map && v.mapSeq !== store.mapSeq) { v.img = mapImage(store.map, c); v.mapSeq = store.mapSeq; }
  if (!v.fitted && store.map && frame === "map") fitMap(v);
  if (v.follow && pose) { v.cx = pose.x; v.cy = pose.y; }
  else if (!v.fitted && pose && !store.map) { v.cx = pose.x; v.cy = pose.y; v.fitted = true; }

  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.fillStyle = c["map-bg"];
  ctx.fillRect(0, 0, v.w, v.h);
  /** @param {number} x @param {number} y */
  const S = (x, y) => [v.w / 2 + (x - v.cx) * v.scale, v.h / 2 - (y - v.cy) * v.scale];

  // 1 m grid (always: the catch field has no map)
  const step = v.scale < 12 ? 5 : 1;
  ctx.strokeStyle = c["map-grid"]; ctx.lineWidth = 1;
  ctx.beginPath();
  const x0 = Math.floor((v.cx - v.w / 2 / v.scale) / step) * step, x1 = v.cx + v.w / 2 / v.scale;
  const y0 = Math.floor((v.cy - v.h / 2 / v.scale) / step) * step, y1 = v.cy + v.h / 2 / v.scale;
  for (let x = x0; x <= x1; x += step) { const [sx] = S(x, 0); ctx.moveTo(sx, 0); ctx.lineTo(sx, v.h); }
  for (let y = y0; y <= y1; y += step) { const [, sy] = S(0, y); ctx.moveTo(0, sy); ctx.lineTo(v.w, sy); }
  ctx.stroke();

  if (v.img && frame === "map") {
    const i = store.map.info;
    const [sx, sy] = S(i.origin.position.x, i.origin.position.y + i.height * i.resolution);
    ctx.imageSmoothingEnabled = false;
    ctx.drawImage(v.img, sx, sy, i.width * i.resolution * v.scale, i.height * i.resolution * v.scale);
  }

  const plan = store.plan;
  if (plan && plan.frame === frame && plan.xy.length >= 4 && st?.mission === "GOTO") {
    ctx.strokeStyle = c["map-plan"]; ctx.lineWidth = 2.5; ctx.setLineDash([]);
    ctx.beginPath();
    for (let k = 0; k < plan.xy.length; k += 2) { const [sx, sy] = S(plan.xy[k], plan.xy[k + 1]); k ? ctx.lineTo(sx, sy) : ctx.moveTo(sx, sy); }
    ctx.stroke();
  }

  const scan = store.scan;
  if (scan && scan.frame === frame) {
    ctx.fillStyle = c["map-scan"];
    const r = Math.max(1.2, Math.min(3, v.scale * 0.025));
    for (let k = 0; k < scan.xy.length; k += 2) { const [sx, sy] = S(scan.xy[k], scan.xy[k + 1]); ctx.fillRect(sx - r / 2, sy - r / 2, r, r); }
  }

  if (p.places && frame === "map") {
    ctx.font = "12px Segoe UI, system-ui, sans-serif";
    for (const [name, q] of Object.entries(p.places)) {
      const [sx, sy] = S(q[0], q[1]);
      ctx.fillStyle = c["map-place"];
      ctx.beginPath(); ctx.arc(sx, sy, 4, 0, 2 * Math.PI); ctx.fill();
      ctx.fillText(name, sx + 7, sy - 6);
    }
  }

  if (st?.goal && frame === "map") arrow(ctx, S, st.goal, c["map-goal"], v.scale, false);
  if (p.pending && frame === "map") arrow(ctx, S, p.pending, c["map-pending"], v.scale, true);
  const d = v.drag;
  if (d && d.kind === "goal") arrow(ctx, S, { x: d.x, y: d.y, yaw: d.yaw ?? 0 }, c["map-pending"], v.scale, true, d.yaw === null);

  if (p.catchView && st?.catch) {
    const t = st.catch.target;
    if (t) arrow(ctx, S, { x: t[0], y: t[1], yaw: t[2] }, c["blue"], v.scale, true);
    const b = st.catch.ball;
    if (b) {
      const [sx, sy] = S(b[0], b[1]);
      ctx.fillStyle = c["map-ball"];
      ctx.beginPath(); ctx.arc(sx, sy, Math.max(4, 0.037 * v.scale), 0, 2 * Math.PI); ctx.fill();
      ctx.fillStyle = c["text"]; ctx.font = "11px Segoe UI, system-ui, sans-serif";
      ctx.fillText(`${b[2].toFixed(1)} m up`, sx + 8, sy + 4);
    }
  }

  if (pose) {
    const [sx, sy] = S(pose.x, pose.y);
    const r = Math.max(6, ROBOT_R * v.scale);
    ctx.lineWidth = 2; ctx.strokeStyle = c["map-robot"];
    ctx.fillStyle = "rgba(0, 230, 118, 0.18)";
    ctx.beginPath(); ctx.arc(sx, sy, r, 0, 2 * Math.PI); ctx.fill(); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(sx, sy); ctx.lineTo(sx + Math.cos(pose.yaw) * r * 1.5, sy - Math.sin(pose.yaw) * r * 1.5); ctx.stroke();
  }

  // scale bar
  const m = niceLength(80 / v.scale);
  ctx.strokeStyle = c["dim"]; ctx.fillStyle = c["dim"]; ctx.lineWidth = 2;
  ctx.beginPath(); ctx.moveTo(12, v.h - 14); ctx.lineTo(12 + m * v.scale, v.h - 14); ctx.stroke();
  ctx.font = "11px Segoe UI, system-ui, sans-serif";
  ctx.fillText(`${m < 1 ? m * 100 + " cm" : m + " m"}  (${frame})`, 12, v.h - 20);
}

/** @param {number} x */
function niceLength(x) {
  for (const n of [0.1, 0.2, 0.5, 1, 2, 5, 10, 20, 50]) if (n >= x) return n;
  return 100;
}

/** @param {CanvasRenderingContext2D} ctx @param {any} S @param {Pose} g @param {string} col @param {number} scale
 *  @param {boolean} dashed @param {boolean} [noHead] */
function arrow(ctx, S, g, col, scale, dashed, noHead = false) {
  const [sx, sy] = S(g.x, g.y);
  const L = Math.max(22, 0.45 * scale);
  ctx.strokeStyle = col; ctx.fillStyle = col; ctx.lineWidth = 2.5;
  ctx.setLineDash(dashed ? [5, 4] : []);
  ctx.beginPath(); ctx.arc(sx, sy, Math.max(5, ROBOT_R * scale), 0, 2 * Math.PI); ctx.stroke();
  ctx.setLineDash([]);
  if (noHead) return;
  const ex = sx + Math.cos(g.yaw) * L, ey = sy - Math.sin(g.yaw) * L;
  ctx.beginPath(); ctx.moveTo(sx, sy); ctx.lineTo(ex, ey); ctx.stroke();
  const a = Math.atan2(ey - sy, ex - sx);
  ctx.beginPath(); ctx.moveTo(ex, ey);
  ctx.lineTo(ex - 10 * Math.cos(a - 0.45), ey - 10 * Math.sin(a - 0.45));
  ctx.lineTo(ex - 10 * Math.cos(a + 0.45), ey - 10 * Math.sin(a + 0.45));
  ctx.closePath(); ctx.fill();
}
