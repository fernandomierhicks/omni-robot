// @ts-check
import { html, useEffect, useRef, useState } from "../../vendor/htm-preact-standalone.mjs";
import { Chart, makeBuffer, push } from "../chart.js";
import { useStore } from "../ros.js";
import { HelpBox } from "../ui.js";
import { defaultUrl, serverSignal, ServerDown, usePref } from "./cameras.js";

/* Test > Lidar: the RPLIDAR straight from its serial port, through camera_snap on the robot computer
   (omni_console/rplidar.py), not ROS: what the device says it is (model, firmware, hardware, serial, health), its scan
   modes, the motor's PWM, and every turn drawn as it comes, with the turn rate, samples, returns and framing errors.
   The ROS driver's /scan, when the robot's stack runs, is summarized from the status for comparison. */

const ZOOMS = [[1, "1 m"], [2, "2 m"], [4, "4 m"], [8, "8 m"], [12, "12 m"]];
const TRAILS = [[1, "this turn"], [5, "last 5 turns"], [20, "last 20 turns"]];
const CHART_S = 60;

const css = (/** @type {string} */ v) => getComputedStyle(document.documentElement).getPropertyValue(v).trim();
const sleep = (/** @type {number} */ ms) => new Promise((r) => setTimeout(r, ms));

/** @param {string} url @param {AbortSignal} [signal] */
async function getJson(url, signal = serverSignal()) {
  const r = await fetch(url, { cache: "no-store", signal });
  const j = await r.json().catch(() => ({}));
  if (!r.ok) throw new Error(j.error || `HTTP ${r.status}`);
  return j;
}

/** A canvas sized to its box in device pixels; draw(ctx, w, h) in CSS pixels. Redrawn when `seq` changes.
 * @param {{seq: any, draw: (ctx: CanvasRenderingContext2D, w: number, h: number) => void, aspect: number,
 *   onMove?: (x: number, y: number, w: number, h: number) => void, onLeave?: () => void}} p */
function Canvas({ seq, draw, aspect, onMove, onLeave }) {
  const cv = useRef(/** @type {HTMLCanvasElement | null} */ (null));
  const [size, setSize] = useState(0);
  useEffect(() => {
    const c = /** @type {HTMLCanvasElement} */ (cv.current);
    const ro = new ResizeObserver(() => setSize(c.clientWidth));
    ro.observe(c);
    return () => ro.disconnect();
  }, []);
  useEffect(() => {
    const c = cv.current;
    if (!c || !c.clientWidth) return;
    const dpr = window.devicePixelRatio || 1, w = c.clientWidth, h = Math.round(w * aspect);
    if (c.width !== Math.round(w * dpr) || c.height !== Math.round(h * dpr)) { c.width = Math.round(w * dpr); c.height = Math.round(h * dpr); }
    const ctx = /** @type {CanvasRenderingContext2D} */ (c.getContext("2d"));
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    draw(ctx, w, h);
  });
  return html`<canvas ref=${cv} data-size=${size} style=${`width: 100%; aspect-ratio: ${1 / aspect}; display: block; border-radius: 6px; background: var(--map-bg); cursor: crosshair`}
    onMouseMove=${onMove && ((/** @type {any} */ e) => { const r = e.currentTarget.getBoundingClientRect(); onMove(e.clientX - r.left, e.clientY - r.top, r.width, r.height); })}
    onMouseLeave=${onLeave}></canvas>`;
}

/** The turn seen from above, the lidar's 0 deg up and angles clockwise (its own frame): range rings every metre (or
 * 0.25 m when zoomed in), the newest turn bright, earlier ones (trail) faded, the hovered sample ringed.
 * @param {CanvasRenderingContext2D} ctx @param {number} w @param {number} h @param {any[]} turns @param {number} zoom @param {any} hover */
function drawPolar(ctx, w, h, turns, zoom, hover) {
  const cx = w / 2, cy = h / 2, R = Math.min(w, h) / 2 - 18, k = R / zoom;
  ctx.clearRect(0, 0, w, h);
  ctx.strokeStyle = css("--map-grid") || css("--border");
  ctx.fillStyle = css("--dim");
  ctx.lineWidth = 1;
  ctx.font = "11px system-ui, sans-serif";
  const step = zoom <= 1 ? 0.25 : zoom <= 4 ? 1 : 2;
  for (let r = step; r <= zoom + 1e-6; r += step) {
    ctx.beginPath();
    ctx.arc(cx, cy, r * k, 0, 2 * Math.PI);
    ctx.stroke();
    ctx.fillText(`${+r.toFixed(2)} m`, cx + r * k * 0.5 + 3, cy - r * k * 0.866 - 3);      // along 30 deg, clear of 0 deg's label
  }
  for (let a = 0; a < 360; a += 30) {
    const s = Math.sin((a * Math.PI) / 180), c = Math.cos((a * Math.PI) / 180);
    ctx.beginPath();
    ctx.moveTo(cx, cy);
    ctx.lineTo(cx + s * R, cy - c * R);
    ctx.stroke();
    ctx.textAlign = "center";
    ctx.fillText(a === 0 ? "0° front" : `${a}°`, cx + s * (R + 9), cy - c * (R + 9) + 4);
  }
  ctx.textAlign = "left";
  const col = css("--map-scan") || css("--green");
  turns.forEach((t, i) => {
    const newest = i === turns.length - 1;
    ctx.globalAlpha = newest ? 1 : 0.15 + 0.35 * (i / Math.max(turns.length - 1, 1));
    ctx.fillStyle = col;
    const sz = newest ? 3 : 2;
    for (let j = 0; j < t.a.length; j++) {
      const r = t.r[j] / 1000;
      if (!r || r > zoom) continue;
      const th = (t.a[j] / 100) * Math.PI / 180;
      ctx.fillRect(cx + Math.sin(th) * r * k - sz / 2, cy - Math.cos(th) * r * k - sz / 2, sz, sz);
    }
  });
  ctx.globalAlpha = 1;
  ctx.fillStyle = css("--orange");                  // the lidar
  ctx.beginPath();
  ctx.arc(cx, cy, 5, 0, 2 * Math.PI);
  ctx.fill();
  ctx.fillRect(cx - 1, cy - 14, 2, 9);
  if (hover) {
    const th = (hover.a * Math.PI) / 180;
    ctx.strokeStyle = css("--white") || "#fff";
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.arc(cx + Math.sin(th) * hover.r * k, cy - Math.cos(th) * hover.r * k, 7, 0, 2 * Math.PI);
    ctx.stroke();
  }
}

/** Range against angle, the turn unrolled: gaps in the line are angles with no return.
 * @param {CanvasRenderingContext2D} ctx @param {number} w @param {number} h @param {any} t @param {number} zoom */
function drawStrip(ctx, w, h, t, zoom) {
  const L = 40, B = 16, pw = w - L - 6, ph = h - B - 6;
  ctx.clearRect(0, 0, w, h);
  ctx.strokeStyle = css("--map-grid") || css("--border");
  ctx.fillStyle = css("--dim");
  ctx.font = "11px system-ui, sans-serif";
  ctx.lineWidth = 1;
  for (let a = 0; a <= 360; a += 45) {
    const x = L + (a / 360) * pw;
    ctx.beginPath(); ctx.moveTo(x, 6); ctx.lineTo(x, 6 + ph); ctx.stroke();
    ctx.textAlign = "center";
    ctx.fillText(`${a}°`, x, h - 3);
  }
  ctx.textAlign = "right";
  for (const f of [0, 0.5, 1]) {
    const y = 6 + ph - f * ph;
    ctx.beginPath(); ctx.moveTo(L, y); ctx.lineTo(L + pw, y); ctx.stroke();
    ctx.fillText(`${+(f * zoom).toFixed(1)} m`, L - 4, y + 4);
  }
  ctx.textAlign = "left";
  if (!t) return;
  ctx.fillStyle = css("--map-scan") || css("--green");
  for (let j = 0; j < t.a.length; j++) {
    const r = t.r[j] / 1000;
    if (!r) continue;
    ctx.fillRect(L + (t.a[j] / 36000) * pw - 1, 6 + ph - Math.min(r / zoom, 1) * ph - 1, 2, 2);
  }
  ctx.fillStyle = css("--red");
  for (let j = 0; j < t.a.length; j++) {
    if (!t.r[j]) ctx.fillRect(L + (t.a[j] / 36000) * pw - 0.5, 6 + ph + 1, 1, 3);    // no return: a tick under the axis
  }
}

export function LidarTab() {
  const [url, setUrl] = usePref("url", defaultUrl());              // the Cameras tab's server: one address for both
  const [draft, setDraft] = useState(url);
  const [zoom, setZoom] = usePref("zoom", 4, "lidar");
  const [trail, setTrail] = usePref("trail", 1, "lidar");
  const [mode, setMode] = usePref("mode", "", "lidar");
  const [pwm, setPwm] = usePref("pwm", 660, "lidar");
  const [dev, setDev] = useState(/** @type {any} */ (null));
  const [reach, setReach] = useState("");
  const [err, setErr] = useState("");
  const [busy, setBusy] = useState("");
  const [hold, setHold] = useState(false);
  const [hover, setHover] = useState(/** @type {any} */ (null));
  const [visible, setVisible] = useState(!document.hidden);
  const S = useRef({ turns: /** @type {any[]} */ ([]), seq: 0, bytes: /** @type {[number, number][]} */ ([]), scanErr: "",
    hz: makeBuffer(1, CHART_S), valid: makeBuffer(1, CHART_S), n: makeBuffer(1, CHART_S) }).current;
  const [, tick] = useState(0);
  const st = useStore("status").status;

  const inFlight = useRef(false);                // one status request at a time (a Pi that is off takes the timeout)
  const poll = () => {
    if (inFlight.current) return;
    inFlight.current = true;
    getJson(`${url}/lidar`).then((j) => { setDev(j); setReach(""); })
      .catch((e) => { setReach(e.message || "no answer"); setDev(null); })
      .finally(() => { inFlight.current = false; });
  };

  useEffect(() => {                              // the device's state every 2 s (it also keeps the port open)
    let stop = false;
    const go = () => { if (!stop && !document.hidden) poll(); };
    go();
    const h = setInterval(go, 2000);
    return () => { stop = true; clearInterval(h); };
  }, [url]);

  useEffect(() => {
    const f = () => setVisible(!document.hidden);
    document.addEventListener("visibilitychange", f);
    return () => document.removeEventListener("visibilitychange", f);
  }, []);

  const running = !!dev?.running;
  useEffect(() => {                              // turns, one request after another, each waiting for a newer one
    if (!running || !visible || hold) return;
    const ac = new AbortController();
    (async () => {
      while (!ac.signal.aborted) {
        try {
          const r = await fetch(`${url}/lidar/scan?after=${S.seq}`, { cache: "no-store", signal: ac.signal });
          const text = await r.text();
          const j = JSON.parse(text || "{}");
          if (!r.ok) throw new Error(j.error || `HTTP ${r.status}`);
          S.seq = j.seq;
          S.turns = [...S.turns, j].slice(-20);
          S.bytes.push([Date.now(), text.length]);
          S.scanErr = "";
          const t = performance.now() / 1000;
          if (j.rate?.hz != null) push(S.hz, t, [j.rate.hz]);
          push(S.valid, t, [100 * j.stats.valid]);
          push(S.n, t, [j.stats.samples]);
          tick((x) => x + 1);
        } catch (e) {
          if (ac.signal.aborted) return;
          S.scanErr = /** @type {Error} */ (e).message;
          tick((x) => x + 1);
          await sleep(1000);
        }
      }
    })();
    return () => ac.abort();
  }, [running, visible, hold, url]);

  /** @param {Record<string, string | number>} q @param {string} what */
  const set = async (q, what) => {
    setBusy(what);
    try {
      setDev(await getJson(`${url}/lidar/set?${new URLSearchParams(/** @type {any} */ (q))}`));
      setErr("");
    } catch (e) { setErr(/** @type {Error} */ (e).message); poll(); }       // poll: the events say what happened
    setBusy("");
  };

  const now = Date.now();
  S.bytes = S.bytes.filter(([t]) => now - t < 5000);
  const kbps = S.bytes.reduce((a, [, b]) => a + b, 0) / 5 / 1024;
  const last = S.turns[S.turns.length - 1];
  const shown = running || hold ? S.turns.slice(-trail) : [];
  const modes = dev?.modes || [];
  const curMode = dev?.running || mode || modes.find((/** @type {any} */ m) => m.typical)?.name || "";
  const info = dev?.info;
  const rate = last?.rate || dev?.rate || {};
  const stats = last?.stats || {};
  const ros = st?.lidar;
  const spinning = !!dev?.spinning;
  const events = [...(dev?.events || [])].reverse();
  const f = (/** @type {number | null | undefined} */ v, d = 1) => (v == null ? "–" : v.toFixed(d));

  /** The sample nearest the pointer (in the newest turn). @param {number} x @param {number} y @param {number} w @param {number} h */
  const onMove = (x, y, w, h) => {
    if (!last) return;
    const R = Math.min(w, h) / 2 - 18, k = R / zoom, px = (x - w / 2) / k, py = (h / 2 - y) / k;
    let best = null, bd = 0.2 * zoom;
    for (let j = 0; j < last.a.length; j++) {
      const r = last.r[j] / 1000;
      if (!r) continue;
      const th = (last.a[j] / 100) * Math.PI / 180, d = Math.hypot(Math.sin(th) * r - px, Math.cos(th) * r - py);
      if (d < bd) { bd = d; best = { a: last.a[j] / 100, r, q: last.q[j] }; }
    }
    setHover(best);
  };

  return html`<div class="full">
    <${HelpBox} tab="lidar">
      The lidar straight from its serial port on the robot computer (today the Pi's RPLIDAR A2M8), not through ROS: what
      the device says it is, its health and scan modes, and each turn as it comes. <b>Start</b> spins the motor and
      scans in the chosen mode; <b>Stop</b> stops both. <b>Spin motor</b> runs the motor alone, without a scan. The
      motor starts gently and ramps up to its <b>PWM</b> (0–1023, 660 the default; fewer turns a second gives more
      samples a turn). The motor also stops, and the port is let go, about 20 s after this page closes. The plot is the
      lidar's own frame seen from above: its 0° (front) up, angles clockwise, the turn bright and earlier turns faded.
      Hover over a point to read it. The strip below unrolls the turn by angle: red ticks are angles with no return
      (too far, too dark, too close: under ~0.15 m, or blocked). <b>Events</b> lists what happened, newest first.<br />
      Nothing has to be started first: the server (camera_snap) starts by itself when the Pi boots, and the lidar opens
      when this page asks. The page needs neither the robot's ROS stack nor any other step.<br />
      <b>Troubleshooting, in order:</b> (1) "no answer from the Pi": the Pi is off or booting, the Server address is
      wrong, or the server was stopped. The box under the Server field says what to try for each. (2) "no lidar serial port" means the USB adapter isn't seen (its USB cable).
      (3) "only 0x00 bytes" or "nothing came back" means the adapter is fine but the lidar behind it isn't talking:
      reseat the cable between the lidar and the adapter, and check its power. (4) "is open in ..." means another
      program (the ROS driver) has the port, so stop it. (5) The device answers but won't turn: press <b>Spin
      motor</b>. If Events then says the lidar was lost a moment after the motor started, the 5 V supply dips at
      spin-up and the adapter resets. Plug it into a Pi port or a powered hub, or connect the adapter's second (power)
      micro-USB. If it stays connected but doesn't turn, the motor or its cable is at fault. (6) A health warning or
      error with a code means the device found a fault: <b>Reset device</b> reboots it. (7) <b>Reconnect</b> lets the
      port go and opens it again; the page also reconnects by itself after a USB reset. Framing errors that keep
      rising point to a bad cable or the wrong baud rate. A turn rate well under the PWM's usual rate (~12 Hz at 660)
      points to a dragging or underpowered motor. The ROS row at the bottom is the robot stack's own <span
      class="mono">/scan</span>; only one program can read the lidar at a time.
    <//>
    <div class="card">
      <div class="row">
        <span class="dim">Server</span>
        <input type="text" style="width: 230px" value=${draft} onInput=${(/** @type {any} */ e) => setDraft(e.currentTarget.value)}
          onKeyDown=${(/** @type {any} */ e) => e.key === "Enter" && setUrl(draft.replace(/\/+$/, ""))} />
        ${draft !== url && html`<button class="btn small" onClick=${() => setUrl(draft.replace(/\/+$/, ""))}>Use</button>`}
        ${reach ? html`<span class="warn">no answer from the Pi (see below)</span>`
          : !dev ? html`<span class="dim">asking...</span>`
          : dev.error && !dev.open ? html`<span class="warn">server up, lidar not connected (see below)</span>`
          : html`<span class=${running || spinning ? "ok" : "dim"}>${info?.name || "lidar"} ${running ? `scanning (${dev.running})`
              : spinning ? `motor spinning at PWM ${dev.pwm}, no scan` : "connected, motor off"}</span>`}
        <span class="grow"></span>
        <span class="dim mono">${kbps.toFixed(0)} kB/s</span>
      </div>
      ${reach && html`<${ServerDown} url=${url} reach=${reach} onUse=${(/** @type {string} */ u) => { setDraft(u); setUrl(u); }} />`}
      ${!reach && dev && !dev.open && dev.error && html`<p class="warn" style="margin: 8px 0 0">
        The server answers but can't use the lidar yet: ${dev.error}. The page tries again every few seconds; fix the
        cause above and it connects by itself (or press <b>Reconnect</b>).</p>`}
      <div class="row" style="margin-top: 8px">
        ${running
          ? html`<button class="btn" disabled=${!!busy} onClick=${() => set({ run: 0 }, "stop")}>${busy === "stop" ? "Stopping..." : "Stop"}</button>`
          : html`<button class="btn primary" disabled=${!dev?.open || !!busy} title="Spin the motor and scan in the chosen mode"
              onClick=${() => set({ run: 1, mode: curMode, pwm }, "start")}>${busy === "start" ? "Starting..." : "Start"}</button>`}
        <button class=${`btn${spinning ? " primary" : ""}`} disabled=${!dev?.open || running || !!busy}
          title="The motor alone, no scan: does it turn, and does the USB survive its start?"
          onClick=${() => set({ spin: spinning ? 0 : 1, pwm }, "spin")}>${busy === "spin" ? "..." : spinning ? "Stop motor" : "Spin motor"}</button>
        <span class="dim">mode</span>
        <select value=${curMode} disabled=${!modes.length || !!busy} onChange=${(/** @type {any} */ e) => {
          setMode(e.currentTarget.value);
          if (running) set({ mode: e.currentTarget.value }, "mode");
        }}>${modes.map((/** @type {any} */ m) => html`<option value=${m.name}>${m.name}: ${(m.rate / 1000).toFixed(0)} k samples/s${m.typical ? " (its default)" : ""}</option>`)}</select>
        <span class="dim">motor PWM</span>
        <input type="range" min="0" max="1023" step="1" style="width: 140px" value=${pwm}
          onInput=${(/** @type {any} */ e) => setPwm(Number(e.currentTarget.value))}
          onChange=${(/** @type {any} */ e) => (running || spinning) && set({ pwm: e.currentTarget.value }, "pwm")} />
        <input type="number" min="0" max="1023" style="width: 70px" value=${pwm}
          onChange=${(/** @type {any} */ e) => { const v = Math.max(0, Math.min(1023, Number(e.currentTarget.value))); setPwm(v); if (running || spinning) set({ pwm: v }, "pwm"); }} />
        <button class="btn small" disabled=${pwm === 660} onClick=${() => { setPwm(660); if (running || spinning) set({ pwm: 660 }, "pwm"); }}>660</button>
        <span class="grow"></span>
        <button class="btn small" disabled=${!!reach || !!busy} title="Let the port go and open it again (also done by itself after a USB reset)"
          onClick=${() => set({ reopen: 1 }, "reopen")}>${busy === "reopen" ? "Reconnecting..." : "Reconnect"}</button>
        <button class="btn small" disabled=${!dev?.open || running || !!busy} title="Ask the device for its health again (not while scanning)"
          onClick=${() => set({ health: 1 }, "health")}>Re-read health</button>
        <button class="btn small" disabled=${!dev?.open || !!busy} title="The device reboots itself (it stops scanning)"
          onClick=${() => confirm("Reset (reboot) the lidar?") && set({ reset: 1 }, "reset")}>${busy === "reset" ? "Resetting..." : "Reset device"}</button>
      </div>
      ${(err || (running && S.scanErr)) && html`<p class="warn" style="margin: 6px 0 0">${err || S.scanErr}</p>`}
    </div>
    <div class="row" style="align-items: stretch; gap: 10px">
      <div class="card" style="min-width: 260px">
        <h2>Device</h2>
        ${info ? html`<div class="kv">
          <span class="k">model</span><span>${info.name} <span class="dim mono">(0x${info.model.toString(16).toUpperCase()})</span></span>
          <span class="k">firmware</span><span class="mono">${info.firmware}</span>
          <span class="k">hardware</span><span class="mono">${info.hardware}</span>
          <span class="k">serial</span><span class="mono" style="font-size: 11px">${info.serial}</span>
          <span class="k">health</span><span class=${dev.health.status === "good" ? "ok" : dev.health.status === "warning" ? "warn" : "err"}>
            ${dev.health.status}${dev.health.code ? ` (code 0x${dev.health.code.toString(16)})` : ""}</span>
          <span class="k">port</span><span class="mono" style="font-size: 11px; word-break: break-all">${dev.port}${dev.path && dev.path !== dev.port ? ` (${dev.path.replace("/dev/", "")})` : ""}</span>
          <span class="k">baud</span><span class="mono">${dev.baud}</span>
        </div>` : html`<p class="dim">${dev?.error ? "Not connected." : "Asking the device..."}</p>`}
      </div>
      <div class="card grow">
        <h2>Now</h2>
        ${running ? html`<div class="kv">
          <span class="k">turn rate</span><span class="mono">${f(rate.hz, 2)} Hz at PWM ${dev.pwm}</span>
          <span class="k">samples</span><span class="mono">${rate.samples_per_turn ?? "–"} a turn, ${rate.samples_per_s ?? "–"} a second
            ${(() => { const m = modes.find((/** @type {any} */ x) => x.name === dev.running); return m ? html` <span class="dim">(${m.name} is ${m.rate}/s)</span>` : ""; })()}</span>
          <span class="k">returns</span><span class="mono">${f(100 * (stats.valid ?? 0), 0)}% of samples this turn (${f(100 * (rate.valid ?? 0), 0)}% over 5 s)</span>
          <span class="k">range</span><span class="mono">${f(stats.min_m, 2)} – ${f(stats.median_m, 2)} – ${f(stats.max_m, 2)} m (min, median, max)</span>
          <span class="k">nearest</span><span class="mono">${stats.nearest_deg != null ? `${f(stats.min_m, 3)} m at ${f(stats.nearest_deg)}°` : "–"}</span>
          <span class="k">framing errors</span><span class=${`mono${dev.bad_bytes ? " warn" : ""}`}>${dev.bad_bytes} bytes skipped of ${(dev.bytes_in / 1024).toFixed(0)} kB</span>
          <span class="k">turn age</span><span class="mono">${last ? `${f(last.age_ms, 0)} ms when sent` : "–"}</span>
        </div>` : html`<p class="dim">${!dev?.open ? "Not connected." : spinning
          ? "Motor spinning, no scan: the lidar can't report its speed without scanning, so look or listen. Start to scan."
          : "Motor off. Start to scan."}</p>`}
        ${modes.length > 0 && html`<table class="mono" style="font-size: 12px; margin-top: 8px; border-collapse: collapse">
          <thead class="dim"><tr><th style="text-align: left; padding-right: 12px">scan mode</th><th style="text-align: right; padding-right: 12px">samples/s</th>
            <th style="text-align: right; padding-right: 12px">max range</th><th style="text-align: left">format</th></tr></thead>
          <tbody>${modes.map((/** @type {any} */ m) => html`<tr class=${m.name === dev.running ? "ok" : ""}>
            <td style="padding-right: 12px">${m.name}${m.typical ? " *" : ""}</td><td style="text-align: right; padding-right: 12px">${m.rate}</td>
            <td style="text-align: right; padding-right: 12px">${m.max_m ?? "–"} m</td>
            <td>${{ 129: "standard", 130: "express capsules", 132: "ultra capsules" }[/** @type {129} */ (m.answer)] || `0x${m.answer.toString(16)} (unsupported)`}</td></tr>`)}</tbody></table>
          <div class="dim" style="font-size: 12px">* the device's default mode</div>`}
      </div>
    </div>
    <div class="card">
      <h2>Events</h2>
      ${events.length ? html`<div class="mono" style="font-size: 12px; max-height: 160px; overflow-y: auto">
        ${events.map((/** @type {any} */ ev) => html`<div class=${ev.level === "error" ? "err" : ev.level === "warn" ? "warn" : ""}>
          <span class="dim">${new Date(ev.t * 1000).toLocaleTimeString()}</span> ${ev.text}</div>`)}</div>`
        : html`<p class="dim">${reach ? "The server isn't answering." : "Nothing yet."}</p>`}
    </div>
    <div class="card">
      <div class="row">
        <b class="grow">The scan, from above</b>
        <span class="dim">show</span>
        <select value=${zoom} onChange=${(/** @type {any} */ e) => setZoom(Number(e.currentTarget.value))}>
          ${ZOOMS.map(([v, l]) => html`<option value=${v}>${l}</option>`)}</select>
        <select value=${trail} onChange=${(/** @type {any} */ e) => setTrail(Number(e.currentTarget.value))}>
          ${TRAILS.map(([v, l]) => html`<option value=${v}>${l}</option>`)}</select>
        <button class=${`btn small${hold ? " primary" : ""}`} disabled=${!running && !hold} onClick=${() => setHold(!hold)}>${hold ? "Held: resume" : "Hold"}</button>
      </div>
      <div style="max-width: 720px; margin: 8px auto 0">
        <${Canvas} seq=${`${last?.seq}-${zoom}-${trail}-${hover?.a}`} aspect=${1}
          draw=${(/** @type {CanvasRenderingContext2D} */ ctx, /** @type {number} */ w, /** @type {number} */ h) => drawPolar(ctx, w, h, shown, zoom, hover)}
          onMove=${onMove} onLeave=${() => setHover(null)} />
        <div class="mono" style="font-size: 12px; margin-top: 4px; min-height: 16px">
          ${hover ? `${hover.a.toFixed(2)}° · ${hover.r.toFixed(3)} m · quality ${hover.q}`
            : html`<span class="dim">${running ? "hover over a point to read it" : "no scan: Start"}</span>`}</div>
        <div style="margin-top: 6px">
          <${Canvas} seq=${`${last?.seq}-${zoom}`} aspect=${0.28}
            draw=${(/** @type {CanvasRenderingContext2D} */ ctx, /** @type {number} */ w, /** @type {number} */ h) => drawStrip(ctx, w, h, shown[shown.length - 1], zoom)} />
        </div>
      </div>
    </div>
    <div class="charts">
      <div class="card"><${Chart} title="Turn rate" unit="Hz" span=${2} buf=${S.hz} series=${[{ label: "turns/s", color: "blue" }]} /></div>
      <div class="card"><${Chart} title="Returns" unit="%" span=${10} buf=${S.valid} series=${[{ label: "samples with a range", color: "green" }]} /></div>
      <div class="card"><${Chart} title="Samples a turn" unit="" span=${20} buf=${S.n} series=${[{ label: "samples", color: "orange" }]} /></div>
    </div>
    <div class="card" style="margin-top: 10px">
      <div class="row"><b>ROS /scan</b>
        ${ros ? html`<span class=${ros.age > 2 ? "warn" : ros.hz < 5 ? "warn" : "ok"}>${f(ros.hz)} Hz, newest ${f(ros.age, 2)} s old</span>`
          : html`<span class="dim">no /scan in the robot's status (its lidar driver isn't running, or the console isn't connected to the robot)</span>`}</div>
    </div>
  </div>`;
}
