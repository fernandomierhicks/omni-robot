// @ts-check
/* A rolling time chart (uPlot): the last WINDOW seconds of a few series, redrawn at most once per animation frame. The
   data comes in batches (the robot sends its 200 Hz samples 10 at a time), so the lines keep their full detail. */
import uPlot from "../vendor/uPlot.esm.js";
import { html, useEffect, useRef } from "../vendor/htm-preact-standalone.mjs";

export const WINDOW = 10;           // s shown

/** Columns of samples kept for the window: t plus one array per series.
 *  @param {number} n series @param {number} [window] s kept and shown */
export function makeBuffer(n, window = WINDOW) {
  return { t: /** @type {number[]} */ ([]), y: Array.from({ length: n }, () => /** @type {(number | null)[]} */ ([])),
    seq: 0, window };
}

/** @param {ReturnType<typeof makeBuffer>} b @param {number} t @param {(number | null)[]} ys */
export function push(b, t, ys) {
  b.t.push(t);
  ys.forEach((v, k) => b.y[k].push(v));
  const cut = b.t.findIndex((x) => x >= t - b.window);
  if (cut > 0) { b.t.splice(0, cut); b.y.forEach((a) => a.splice(0, cut)); }
  b.seq++;
}

/** span: the smallest y range shown (a robot standing still would otherwise fill the chart with noise)
 *  @param {{title: string, unit: string, span: number, series: {label: string, color: string, dash?: boolean}[],
 *            buf: ReturnType<typeof makeBuffer>}} p */
export function Chart({ title, unit, span, series, buf }) {
  const el = useRef(/** @type {HTMLDivElement | null} */ (null));
  useEffect(() => {
    const box = /** @type {HTMLDivElement} */ (el.current);
    const css = getComputedStyle(document.documentElement);
    const tok = (/** @type {string} */ k) => css.getPropertyValue(`--${k}`).trim() || k;
    const axis = { stroke: tok("dim"), grid: { stroke: tok("border"), width: 1 }, ticks: { stroke: tok("border") } };
    const opts = {
      width: box.clientWidth, height: 170, title, legend: { live: false },
      scales: { x: { time: false }, y: { range: (/** @type {any} */ u, /** @type {number} */ lo, /** @type {number} */ hi) => {
        const mid = (lo + hi) / 2, half = Math.max((hi - lo) / 2 * 1.1, span / 2);
        return [mid - half, mid + half];
      } } },
      axes: [{ ...axis, values: (/** @type {any} */ u, /** @type {number[]} */ v) =>
        v.map((x) => (buf.window > 120 ? `${(x / 60).toFixed(0)} min` : `${x.toFixed(0)} s`)) },
        { ...axis, label: unit, size: 50 }],
      series: [{ label: "t" }, ...series.map((s) => ({ label: s.label, stroke: tok(s.color), width: 1.5,
        dash: s.dash ? [6, 4] : undefined, points: { show: false } }))],
    };
    const u = new uPlot(/** @type {any} */ (opts), [[], ...series.map(() => [])], box);
    const ro = new ResizeObserver(() => u.setSize({ width: box.clientWidth, height: 170 }));
    ro.observe(box);
    let seen = -1, raf = 0;
    const loop = () => {
      if (buf.seq !== seen && buf.t.length) {
        seen = buf.seq;
        const t1 = buf.t[buf.t.length - 1];
        u.setData([buf.t.map((t) => t - t1), ...buf.y], false);
        u.setScale("x", { min: -buf.window, max: 0 });
      }
      raf = requestAnimationFrame(loop);
    };
    raf = requestAnimationFrame(loop);
    return () => { cancelAnimationFrame(raf); ro.disconnect(); u.destroy(); };
  }, []);
  return html`<div class="chart" ref=${el}></div>`;
}
