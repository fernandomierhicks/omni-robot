// @ts-check
import { html, useEffect, useRef, useState } from "../../vendor/htm-preact-standalone.mjs";
import { HelpBox, clock } from "../ui.js";

/* Single frames from the cameras, fetched one JPEG at a time from camera_snap (omni_console/camera_snap.py) on the
   computer they're plugged into: on demand, or one every few seconds. Never through rosbridge. */

const STREAM = /** @type {Record<string, string>} */ ({ color: "colour", ir: "IR (left imager)", depth: "depth" });
const RATES = [[0, "on demand"], [1, "every 1 s"], [2, "every 2 s"], [5, "every 5 s"], [10, "every 10 s"]];
const WIDTHS = [[320, "320 px"], [640, "640 px"], [0, "full size"]];
const QUALITIES = [[50, "low"], [70, "medium"], [85, "high"]];
// depth_colormap's palette (camera_snap.py): near red to far blue
const DEPTH_RAMP = "linear-gradient(to right, rgb(255,40,40), rgb(255,230,40), rgb(60,220,80), rgb(40,200,230), rgb(60,60,255))";

/** @param {string} key @param {any} dflt */
function usePref(key, dflt) {
  const k = `console.cameras.${key}`;
  const [v, setV] = useState(() => { try { const s = localStorage.getItem(k); return s === null ? dflt : JSON.parse(s); } catch { return dflt; } });
  /** @param {any} x */
  const set = (x) => { try { localStorage.setItem(k, JSON.stringify(x)); } catch { /* */ } setV(x); };
  return /** @type {[any, (x: any) => void]} */ ([v, set]);
}

/** The page served by the robot computer itself: its cameras are on the same host; from the PC (localhost): the Pi. */
function defaultUrl() {
  const h = location.hostname;
  return `http://${!h || h === "localhost" || h === "127.0.0.1" ? "omni-cam.local" : h}:8086`;
}

export function CamerasTab() {
  const [url, setUrl] = usePref("url", defaultUrl());
  const [draft, setDraft] = useState(url);
  const [rate, setRate] = usePref("rate", 0);
  const [width, setWidth] = usePref("width", 640);
  const [quality, setQuality] = usePref("quality", 70);
  const [near, setNear] = usePref("near", 0.2);
  const [far, setFar] = usePref("far", 4.0);
  const [off, setOff] = usePref("off", []);
  const [cams, setCams] = useState(/** @type {any[] | null} */ (null));
  const [reach, setReach] = useState("");
  const [shots, setShots] = useState(/** @type {Record<string, any>} */ ({}));
  const [probe, setProbe] = useState(/** @type {any} */ (null));
  const opts = useRef({ url, width, quality, near, far });
  opts.current = { url, width, quality, near, far };
  const live = useRef({ busy: /** @type {Record<string, boolean>} */ ({}), last: /** @type {Record<string, number>} */ ({}),
    bytes: /** @type {[number, number][]} */ ([]), srcs: /** @type {Set<string>} */ (new Set()) });

  const panels = (cams || []).flatMap((c) => c.streams.map((/** @type {string} */ s) =>
    ({ id: `${c.name}/${s}`, cam: c.name, stream: s, label: `${c.label}: ${STREAM[s] || s}` })));
  const shown = panels.filter((p) => !off.includes(p.id));

  /** @param {{id: string, cam: string, stream: string}} p */
  const snap = async (p) => {
    const L = live.current;
    if (L.busy[p.id]) return;                    // one request per picture at a time: a slow link never piles up
    L.busy[p.id] = true;
    L.last[p.id] = Date.now();
    setShots((s) => ({ ...s, [p.id]: { ...s[p.id], busy: true } }));
    const o = opts.current, t0 = performance.now();
    /** @type {any} */ let next;
    try {
      const r = await fetch(`${o.url}/snap?cam=${p.cam}&stream=${p.stream}&w=${o.width}&q=${o.quality}&near=${o.near}&far=${o.far}`,
        { cache: "no-store" });
      if (!r.ok) {
        const j = await r.json().catch(() => ({}));
        next = { error: j.error || `HTTP ${r.status}` };
      } else {
        const blob = await r.blob();
        let info = {};
        try { info = JSON.parse(r.headers.get("X-Frame-Info") || "{}"); } catch { /* */ }
        L.bytes.push([Date.now(), blob.size]);
        const src = URL.createObjectURL(blob);
        L.srcs.add(src);
        next = { src, info, bytes: blob.size, ms: performance.now() - t0, at: Date.now() / 1000 };
      }
    } catch (e) {
      next = { error: `Can't reach ${o.url} (${/** @type {Error} */ (e).message})` };
    }
    L.busy[p.id] = false;
    setShots((s) => {
      const prev = s[p.id];
      if (next.src && prev?.src) { URL.revokeObjectURL(prev.src); L.srcs.delete(prev.src); }
      return { ...s, [p.id]: next.src ? next : { ...prev, busy: false, error: next.error } };
    });
  };

  useEffect(() => {                              // which cameras the server has, and whether they're open
    let stop = false;
    const poll = () => fetch(`${url}/cameras`, { cache: "no-store" }).then((r) => r.json())
      .then((j) => { if (!stop) { setCams(j.cameras); setReach(""); } })
      .catch((e) => { if (!stop) setReach(e.message || "no answer"); });
    poll();
    const h = setInterval(poll, 4000);
    return () => { stop = true; clearInterval(h); };
  }, [url]);

  useEffect(() => {                              // the timed mode: each picture once per period, while this tab shows
    if (!rate) return;
    const h = setInterval(() => {
      if (document.hidden) return;
      for (const p of shown) if (Date.now() - (live.current.last[p.id] || 0) >= rate * 1000) snap(p);
    }, 200);
    return () => clearInterval(h);
  }, [rate, cams, off.join()]);

  useEffect(() => () => { live.current.srcs.forEach((s) => URL.revokeObjectURL(s)); }, []);

  /** @param {any} e @param {string} id */
  const onDepthClick = async (e, id) => {
    const r = e.currentTarget.getBoundingClientRect();
    const u = (e.clientX - r.left) / r.width, v = (e.clientY - r.top) / r.height;
    try {
      const j = await (await fetch(`${url}/depth?u=${u.toFixed(4)}&v=${v.toFixed(4)}`, { cache: "no-store" })).json();
      setProbe({ id, u, v, m: j.range_m, error: j.error });
    } catch (err) { setProbe({ id, u, v, m: null, error: /** @type {Error} */ (err).message }); }
  };

  const now = Date.now();
  const L = live.current;
  L.bytes = L.bytes.filter(([t]) => now - t < 10000);
  const kbps = L.bytes.reduce((a, [, b]) => a + b, 0) / 10 / 1024;

  return html`<div class="full">
    <${HelpBox} tab="cameras">
      Single pictures from the cameras on the robot computer (today the Pi: the Arducam and the D435's colour, left
      IR and depth images). <b>Snap</b> asks for one picture; a rate other than <i>on demand</i> asks for one per
      picture every few seconds while this tab is on screen, and the next only after the last one arrived, so a slow
      Wi-Fi link never fills up. Smaller pictures and lower quality cost less. Click the depth image to read the range
      at a point. A camera opens at its first picture and closes 20 s after the last, so ball_view and the camera
      head can still have it. Start the picture server on the Pi from Git Bash:
      <span class="mono">bash ros/tools/cam_snap.sh start</span> (<span class="mono">stop</span> to end it).
    <//>
    <div class="card">
      <div class="row">
        <span class="dim">Server</span>
        <input type="text" style="width: 230px" value=${draft} onInput=${(/** @type {any} */ e) => setDraft(e.currentTarget.value)}
          onKeyDown=${(/** @type {any} */ e) => e.key === "Enter" && setUrl(draft.replace(/\/+$/, ""))} />
        ${draft !== url && html`<button class="btn small" onClick=${() => setUrl(draft.replace(/\/+$/, ""))}>Use</button>`}
        ${reach ? html`<span class="warn">no answer: is camera_snap running there?</span>`
          : cams ? cams.map((c) => html`<span class=${c.error ? "warn" : c.open ? "ok" : "dim"} title=${c.error || ""}>
              ${c.label} ${c.error ? "unavailable" : c.open ? "open" : "closed"}</span>`)
          : html`<span class="dim">asking...</span>`}
      </div>
      <div class="row" style="margin-top: 8px">
        <select value=${rate} onChange=${(/** @type {any} */ e) => setRate(Number(e.currentTarget.value))}>
          ${RATES.map(([v, l]) => html`<option value=${v}>${l}</option>`)}</select>
        <select value=${width} onChange=${(/** @type {any} */ e) => setWidth(Number(e.currentTarget.value))}>
          ${WIDTHS.map(([v, l]) => html`<option value=${v}>${l}</option>`)}</select>
        <select value=${quality} onChange=${(/** @type {any} */ e) => setQuality(Number(e.currentTarget.value))}>
          ${QUALITIES.map(([v, l]) => html`<option value=${v}>${l} quality</option>`)}</select>
        <span class="dim">depth</span>
        <input type="number" step="0.1" min="0" value=${near} onChange=${(/** @type {any} */ e) => setNear(Number(e.currentTarget.value))} />
        <span class="dim">to</span>
        <input type="number" step="0.5" min="0.5" value=${far} onChange=${(/** @type {any} */ e) => setFar(Number(e.currentTarget.value))} />
        <span class="dim">m</span>
        <button class="btn primary" disabled=${!shown.length} onClick=${() => shown.forEach(snap)}>Snap all</button>
        <span class="dim mono">${kbps.toFixed(0)} kB/s</span>
      </div>
      ${off.length > 0 && html`<div class="row" style="margin-top: 8px"><span class="dim">Hidden:</span>
        ${off.map((/** @type {string} */ id) => html`<button class="btn small" onClick=${() => setOff(off.filter((/** @type {string} */ x) => x !== id))}>
          show ${panels.find((p) => p.id === id)?.label || id}</button>`)}</div>`}
    </div>
    <div style="display: grid; grid-template-columns: repeat(auto-fill, minmax(min(380px, 100%), 1fr)); gap: 10px">
      ${shown.map((p) => html`<${Panel} key=${p.id} p=${p} shot=${shots[p.id]} probe=${probe?.id === p.id ? probe : null}
        near=${near} far=${far} onSnap=${() => snap(p)} onHide=${() => setOff([...off, p.id])}
        onClick=${p.stream === "depth" ? (/** @type {any} */ e) => onDepthClick(e, p.id) : null} />`)}
    </div>
  </div>`;
}

/** @param {{p: any, shot: any, probe: any, near: number, far: number, onSnap: () => void, onHide: () => void, onClick: any}} a */
function Panel({ p, shot, probe, near, far, onSnap, onHide, onClick }) {
  const i = shot?.info || {};
  const depth = p.stream === "depth";
  return html`<div class="card" style="margin: 0">
    <div class="row"><b class="grow">${p.label}</b>
      <button class="btn small" disabled=${shot?.busy} onClick=${onSnap}>${shot?.busy ? "..." : "Snap"}</button>
      <button class="btn small" onClick=${onHide}>Hide</button></div>
    <div style="position: relative; margin-top: 6px">
      ${shot?.src
        ? html`<img src=${shot.src} alt=${p.label} onClick=${onClick}
            style=${`width: 100%; display: block; border-radius: 6px; background: var(--map-bg)${depth ? "; cursor: crosshair" : ""}`} />`
        : html`<div class="dim" style="aspect-ratio: 4 / 3; display: grid; place-items: center; border-radius: 6px; background: var(--map-bg)">
            No picture yet: Snap</div>`}
      ${probe && html`<div style=${`position: absolute; left: ${probe.u * 100}%; top: ${probe.v * 100}%; width: 14px; height: 14px;
          margin: -8px 0 0 -8px; border: 2px solid var(--white); border-radius: 50%; pointer-events: none`}></div>`}
    </div>
    ${depth && html`<div class="row" style="margin-top: 4px; font-size: 12px"><span class="dim mono">${near} m</span>
      <span class="grow" style=${`height: 8px; border-radius: 4px; background: ${DEPTH_RAMP}`}></span>
      <span class="dim mono">${far} m</span><span class="dim">black: no depth</span></div>`}
    ${shot?.at && html`<div class="dim mono" style="font-size: 12px; margin-top: 4px">
      ${clock(shot.at)} · ${i.width}×${i.height} · ${(shot.bytes / 1024).toFixed(0)} kB · ${shot.ms.toFixed(0)} ms${i.age_ms != null ? ` · frame ${i.age_ms.toFixed(0)} ms old` : ""}
      ${i.exposure_us ? ` · exposure ${(i.exposure_us / 1000).toFixed(1)} ms, gain ${i.gain}` : ""}</div>`}
    ${depth && shot?.at && html`<div class="mono" style="font-size: 12px">
      centre ${i.centre_m != null ? `${i.centre_m.toFixed(2)} m` : "–"} · ${((i.valid || 0) * 100).toFixed(0)}% has depth${i.median_m != null ? ` · 5–95%: ${i.p5_m.toFixed(2)}–${i.p95_m.toFixed(2)} m` : ""}
      ${probe ? ` · clicked: ${probe.m != null ? `${probe.m.toFixed(2)} m` : probe.error || "no depth there"}` : html`<span class="dim"> · click for a range</span>`}</div>`}
    ${shot?.error && html`<p class="warn" style="margin: 4px 0 0">${shot.error}</p>`}
  </div>`;
}
