// @ts-check
import { html, useEffect, useRef, useState } from "../../vendor/htm-preact-standalone.mjs";
import { HelpBox, clock } from "../ui.js";

/* The cameras on the robot computer, from camera_snap (omni_console/camera_snap.py) there, never through rosbridge:
   single pictures (one JPEG per request: on demand or every few seconds) or live video (one multipart stream per
   picture, read with fetch so each frame's info arrives with it). AprilTags are found there (apriltag_view.py: the
   pose and gates omni_slam uses) and drawn here, on the frame, with how steady the lock is over the last seconds. */

const STREAM = /** @type {Record<string, string>} */ ({ color: "colour", ir: "IR (left imager)", depth: "depth" });
const RATES = [[0, "on demand"], [1, "every 1 s"], [2, "every 2 s"], [5, "every 5 s"], [10, "every 10 s"]];
const WIDTHS = [[320, "320 px"], [640, "640 px"], [960, "960 px"], [0, "full size"]];
const QUALITIES = [[50, "low"], [70, "medium"], [85, "high"]];
const FPS_CAPS = [[0, "no cap"], [30, "≤ 30 fps"], [15, "≤ 15 fps"], [5, "≤ 5 fps"]];
const DECIMATE = [[1, "full resolution (slow, farthest)"], [2, "decimate 2 (default)"], [3, "decimate 3 (fastest)"]];
const FOCUS = [["", "focus as it is"], ["auto", "autofocus"], ["0.5", "focus 0.5 m"], ["1", "focus 1 m"], ["1.5", "focus 1.5 m"],
  ["2", "focus 2 m"], ["3", "focus 3 m"], ["1000", "focus far"]];
const LOCK_S = 5;                                // the lock statistics' window, s
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

const css = (/** @type {string} */ v) => getComputedStyle(document.documentElement).getPropertyValue(v).trim();
const sleep = (/** @type {number} */ ms) => new Promise((r) => setTimeout(r, ms));

// ---------------------------------------------------------------- the multipart stream

const CRLF2 = new Uint8Array([13, 10, 13, 10]);

/** @param {Uint8Array} a @param {Uint8Array} pat */
function indexOf(a, pat) {
  outer: for (let i = 0; i <= a.length - pat.length; i++) {
    for (let j = 0; j < pat.length; j++) if (a[i + j] !== pat[j]) continue outer;
    return i;
  }
  return -1;
}

/** camera_snap's /stream: calls onPart(jpeg blob or null, info, bytes) for each part until aborted or it ends.
 * @param {string} url @param {AbortSignal} signal @param {(b: Blob | null, info: any, n: number) => void} onPart */
async function readStream(url, signal, onPart) {
  const r = await fetch(url, { signal, cache: "no-store" });
  if (!r.ok || !r.body) {
    const j = await r.json().catch(() => ({}));
    throw new Error(j.error || `HTTP ${r.status}`);
  }
  const reader = r.body.getReader();
  const dec = new TextDecoder();
  let buf = new Uint8Array(0);
  for (;;) {
    const { value, done } = await reader.read();
    if (done) throw new Error("the stream ended");
    const nb = new Uint8Array(buf.length + value.length);
    nb.set(buf);
    nb.set(value, buf.length);
    buf = nb;
    for (;;) {                                   // every whole part in what arrived
      const h = indexOf(buf, CRLF2);
      if (h < 0) break;
      const head = dec.decode(buf.subarray(0, h));
      const len = Number((/Content-Length:\s*(\d+)/i.exec(head) || [])[1] || 0);
      const end = h + 4 + len;
      if (buf.length < end) break;
      let info = {};
      const m = /X-Frame-Info:\s*(.*)/i.exec(head);
      try { info = m ? JSON.parse(m[1]) : {}; } catch { /* */ }
      onPart(len && /image\/jpeg/i.test(head) ? new Blob([buf.slice(h + 4, end)], { type: "image/jpeg" }) : null, info, len);
      buf = buf.subarray(end);                   // the CRLF and the next boundary start the next header block
    }
  }
}

// ---------------------------------------------------------------- tags: drawing and lock statistics

/** The tags over a frame drawn at w x h: outline (green usable, orange ambiguous, red refused by the SLAM's gates,
 * blue no pose), corner 0 (the tag's bottom left), the tag's axes (x red, y green, z out of the face blue), id and
 * range. Dashed when the detection is from a frame more than 0.15 s older than the picture.
 * @param {CanvasRenderingContext2D} ctx @param {any[]} tags @param {number} w @param {number} h @param {boolean} stale */
function drawTags(ctx, tags, w, h, stale) {
  const lw = Math.max(1.5, w / 400), fs = Math.max(11, Math.round(w / 50));
  ctx.lineWidth = lw;
  ctx.font = `600 ${fs}px system-ui, sans-serif`;
  for (const t of tags || []) {
    const col = t.ok === undefined ? css("--blue") || "#40c4ff" : !t.ok ? css("--red") : t.ambiguous ? css("--orange") : css("--green");
    const P = t.corners.map((/** @type {number[]} */ [u, v]) => [u * w, v * h]);
    ctx.strokeStyle = col;
    ctx.setLineDash(stale ? [6, 4] : []);
    ctx.beginPath();
    P.forEach((/** @type {number[]} */ [x, y], /** @type {number} */ i) => (i ? ctx.lineTo(x, y) : ctx.moveTo(x, y)));
    ctx.closePath();
    ctx.stroke();
    ctx.setLineDash([]);
    ctx.fillStyle = col;
    ctx.beginPath();
    ctx.arc(P[0][0], P[0][1], lw * 2, 0, 2 * Math.PI);
    ctx.fill();
    if (t.axes) {
      const [o, ...ends] = t.axes.map((/** @type {number[]} */ [u, v]) => [u * w, v * h]);
      ["#ff5252", "#69f0ae", "#448aff"].forEach((c, i) => {
        ctx.strokeStyle = c;
        ctx.beginPath();
        ctx.moveTo(o[0], o[1]);
        ctx.lineTo(ends[i][0], ends[i][1]);
        ctx.stroke();
      });
    }
    const label = `${t.id}${t.range != null ? ` · ${t.range.toFixed(3)} m` : ""}`;
    const x = Math.min(...P.map((/** @type {number[]} */ p) => p[0])), y = Math.min(...P.map((/** @type {number[]} */ p) => p[1]));
    const tw = ctx.measureText(label).width;
    const ty = y - fs - 8 < 0 ? Math.max(...P.map((/** @type {number[]} */ p) => p[1])) + fs + 8 : y;
    ctx.fillStyle = "rgba(0, 0, 0, 0.65)";
    ctx.fillRect(x, ty - fs - 6, tw + 8, fs + 6);
    ctx.fillStyle = col;
    ctx.fillText(label, x + 4, ty - 5);
  }
}

/** @param {number[]} a */
function sd(a) {
  if (a.length < 2) return null;
  const m = a.reduce((s, x) => s + x, 0) / a.length;
  return Math.sqrt(a.reduce((s, x) => s + (x - m) ** 2, 0) / a.length);
}

/** A new detection result (once per detected frame, not per picture) into the window. @param {any} L @param {any} info */
function addSample(L, info) {
  if (!info.tags || info.tags_seq == null || info.tags_seq === L.lastSeq) return;
  L.lastSeq = info.tags_seq;
  const t = performance.now() / 1000, cut = t - LOCK_S;
  L.results.push(t);
  for (const g of info.tags) (L.byId[g.id] ||= []).push({ t, g });
  L.results = L.results.filter((/** @type {number} */ x) => x >= cut);
  for (const k of Object.keys(L.byId)) {
    L.byId[k] = L.byId[k].filter((/** @type {any} */ s) => s.t >= cut);
    if (!L.byId[k].length) delete L.byId[k];
  }
}

/** Per tag over the window: how often found, and the scatter of range, position, corners and the face's direction. @param {any} L */
function lockStats(L) {
  return Object.entries(L.byId).map(([id, S]) => {
    const g = S.map((/** @type {any} */ s) => s.g), last = g[g.length - 1];
    const C = g.filter((/** @type {any} */ x) => x.corners_px).map((/** @type {any} */ x) => x.corners_px.flat());
    const csd = C.length > 1 ? Math.sqrt(C[0].map((/** @type {any} */ _, /** @type {number} */ k) => sd(C.map((/** @type {number[]} */ c) => c[k])) ** 2)
      .reduce((/** @type {number} */ a, /** @type {number} */ b) => a + b, 0) / C[0].length) : null;
    const pick = (/** @type {string} */ k) => g.filter((/** @type {any} */ x) => x[k] != null).map((/** @type {any} */ x) => x[k]);
    const xyz = [0, 1, 2].map((k) => sd(g.filter((/** @type {any} */ x) => x.xyz).map((/** @type {any} */ x) => x.xyz[k])));
    return { id, last, n: g.length, rate: g.length / Math.max(L.results.length, 1), rangeSd: sd(pick("range")), xyz, csd,
      yawSd: sd(pick("normal_yaw_deg")), pitchSd: sd(pick("normal_pitch_deg")),
      okShare: g.filter((/** @type {any} */ x) => x.ok && !x.ambiguous).length / g.length };
  }).sort((a, b) => Number(a.id) - Number(b.id));
}

// ---------------------------------------------------------------- the tab

export function CamerasTab() {
  const [url, setUrl] = usePref("url", defaultUrl());
  const [draft, setDraft] = useState(url);
  const [mode, setMode] = usePref("mode", "snap");
  const [rate, setRate] = usePref("rate", 0);
  const [width, setWidth] = usePref("width", 640);
  const [quality, setQuality] = usePref("quality", 70);
  const [fps, setFps] = usePref("fps", 0);
  const [near, setNear] = usePref("near", 0.2);
  const [far, setFar] = usePref("far", 4.0);
  const [off, setOff] = usePref("off", []);
  const [liveOn, setLiveOn] = usePref("live", ["arducam/color"]);
  const [tagsOn, setTagsOn] = usePref("tags", true);
  const [family, setFamily] = usePref("family", "tag25h9");
  const [sizeMm, setSizeMm] = usePref("tagSizeMm", 140);
  const [dec, setDec] = usePref("decimate", 2);
  const [cams, setCams] = useState(/** @type {any[] | null} */ (null));
  const [detector, setDetector] = useState(/** @type {any} */ (null));
  const [reach, setReach] = useState("");
  const [shots, setShots] = useState(/** @type {Record<string, any>} */ ({}));
  const [probe, setProbe] = useState(/** @type {any} */ (null));
  const [visible, setVisible] = useState(!document.hidden);
  const [setErr, setSetErr] = useState("");
  const live = useRef({ busy: /** @type {Record<string, boolean>} */ ({}), last: /** @type {Record<string, number>} */ ({}),
    bytes: /** @type {[number, number][]} */ ([]) });
  const [, tick] = useState(0);

  const panels = (cams || []).flatMap((c) => c.streams.map((/** @type {string} */ s) =>
    ({ id: `${c.name}/${s}`, cam: c.name, stream: s, label: `${c.label}: ${STREAM[s] || s}` })));
  const shown = panels.filter((p) => !off.includes(p.id));
  const profileOf = (/** @type {string} */ cam) => (cams || []).find((c) => c.name === cam)?.profile || "";
  const tagQ = (/** @type {string} */ stream) => tagsOn && stream !== "depth" && detector?.detector
    ? `&tags=${family}&size=${(sizeMm / 1000).toFixed(4)}&dec=${dec}` : "";
  const query = (/** @type {any} */ p) => `cam=${p.cam}&stream=${p.stream}&w=${width}&q=${quality}&near=${near}&far=${far}${tagQ(p.stream)}`;

  /** @param {{id: string, cam: string, stream: string}} p */
  const snap = async (p) => {
    const L = live.current;
    if (L.busy[p.id]) return;                    // one request per picture at a time: a slow link never piles up
    L.busy[p.id] = true;
    L.last[p.id] = Date.now();
    setShots((s) => ({ ...s, [p.id]: { ...s[p.id], busy: true } }));
    const t0 = performance.now();
    /** @type {any} */ let next;
    try {
      const r = await fetch(`${url}/snap?${query(p)}`, { cache: "no-store" });
      if (!r.ok) {
        const j = await r.json().catch(() => ({}));
        next = { error: j.error || `HTTP ${r.status}` };
      } else {
        const blob = await r.blob();
        let info = {};
        try { info = JSON.parse(r.headers.get("X-Frame-Info") || "{}"); } catch { /* */ }
        L.bytes.push([Date.now(), blob.size]);
        next = { blob, info, bytes: blob.size, ms: performance.now() - t0, at: Date.now() / 1000 };
      }
    } catch (e) {
      next = { error: `Can't reach ${url} (${/** @type {Error} */ (e).message})` };
    }
    L.busy[p.id] = false;
    setShots((s) => ({ ...s, [p.id]: next.blob ? next : { ...s[p.id], busy: false, error: next.error } }));
  };

  const poll = () => fetch(`${url}/cameras`, { cache: "no-store" }).then((r) => r.json())
    .then((j) => { setCams(j.cameras); setDetector(j.tags || null); setReach(""); })
    .catch((e) => setReach(e.message || "no answer"));

  useEffect(() => {                              // which cameras the server has, and whether they're open
    let stop = false;
    const go = () => { if (!stop) poll(); };
    go();
    const h = setInterval(go, 4000);
    return () => { stop = true; clearInterval(h); };
  }, [url]);

  useEffect(() => {                              // live video only while the page is on screen
    const f = () => setVisible(!document.hidden);
    document.addEventListener("visibilitychange", f);
    return () => document.removeEventListener("visibilitychange", f);
  }, []);

  useEffect(() => {                              // the timed snapshots: each picture once per period, while this tab shows
    if (!rate || mode !== "snap") return;
    const h = setInterval(() => {
      if (document.hidden) return;
      for (const p of shown) if (Date.now() - (live.current.last[p.id] || 0) >= rate * 1000) snap(p);
    }, 200);
    return () => clearInterval(h);
  }, [rate, mode, cams, off.join(), tagsOn, family, sizeMm, dec, width, quality]);

  useEffect(() => {                              // the bandwidth readout
    const h = setInterval(() => tick((x) => x + 1), 1000);
    return () => clearInterval(h);
  }, []);

  /** @param {string} cam @param {Record<string, string>} q */
  const setCam = async (cam, q) => {
    try {
      const r = await fetch(`${url}/set?cam=${cam}&${new URLSearchParams(q)}`, { cache: "no-store" });
      const j = await r.json();
      setSetErr(r.ok ? "" : j.error || `HTTP ${r.status}`);
      poll();
    } catch (e) { setSetErr(/** @type {Error} */ (e).message); }
  };

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
  const rs = (cams || []).find((c) => c.name === "realsense");
  const ac = (cams || []).find((c) => c.name === "arducam");
  const sel = (/** @type {any} */ v, /** @type {(x: any) => void} */ set, /** @type {any[][]} */ opts, num = true) => html`<select value=${v}
    onChange=${(/** @type {any} */ e) => set(num ? Number(e.currentTarget.value) : e.currentTarget.value)}>
    ${opts.map(([x, l]) => html`<option value=${x}>${l}</option>`)}</select>`;

  return html`<div class="full">
    <${HelpBox} tab="cameras">
      The cameras on the robot computer (today the Pi: the Arducam and the D435's colour, left IR and depth
      images). <b>Snapshots</b> asks for one picture at a time, on demand or every few seconds. <b>Live video</b> streams
      each picture whose <i>Live</i> is on; stream just one for its fastest rate. A camera's mode is its capture
      resolution and frame rate (the Arducam's <i>full field</i> modes see its whole view, the <i>2/3 crop</i> runs at
      100 fps). <b>AprilTags</b> are found on the robot computer and drawn here. Green: usable by the tag SLAM; orange:
      ambiguous (a mirror pose fits nearly as well); red: refused by its gates (small, oblique, poor fit, low margin,
      which is how misreads look); dashed: found in a frame more than 0.15 s older than the picture. The size is the
      black square's side. On the D435 the range is also checked against its depth image. Its IR projector's dots hide
      tags from the IR image: switch the projector off to find tags there (depth gets worse). Click the depth image to
      read the range at a point. Start the picture server on the Pi from Git Bash with <span class="mono">bash
      ros/tools/cam_snap.sh start</span> and stop it with <span class="mono">stop</span>. Over the
      cable: put <span class="mono">OMNI_PI=omni-cam-eth</span> in front, and use Server <span class="mono">http://192.168.0.101:8086</span>.
    <//>
    <div class="card">
      <div class="row">
        <span class="dim">Server</span>
        <input type="text" style="width: 230px" value=${draft} onInput=${(/** @type {any} */ e) => setDraft(e.currentTarget.value)}
          onKeyDown=${(/** @type {any} */ e) => e.key === "Enter" && setUrl(draft.replace(/\/+$/, ""))} />
        ${draft !== url && html`<button class="btn small" onClick=${() => setUrl(draft.replace(/\/+$/, ""))}>Use</button>`}
        ${reach ? html`<span class="warn">no answer: is camera_snap running there?</span>`
          : cams ? cams.map((c) => html`<span class=${c.error ? "warn" : c.open ? "ok" : "dim"} title=${c.error || ""}>
              ${c.label} ${c.error ? "unavailable" : c.streaming ? `streaming ×${c.streaming}` : c.open ? "open" : "closed"}</span>`)
          : html`<span class="dim">asking...</span>`}
        <span class="grow"></span>
        <span class="dim mono">${kbps.toFixed(0)} kB/s</span>
      </div>
      <div class="seg" role="group" aria-label="Pictures" style="max-width: 360px">
        <button class=${`btn${mode === "snap" ? " on" : ""}`} onClick=${() => setMode("snap")}>Snapshots</button>
        <button class=${`btn${mode === "live" ? " on" : ""}`} onClick=${() => setMode("live")}>Live video</button>
      </div>
      <div class="row" style="margin-top: 4px">
        ${mode === "snap" ? sel(rate, setRate, RATES) : sel(fps, setFps, FPS_CAPS)}
        ${sel(width, setWidth, WIDTHS)}
        <select value=${quality} onChange=${(/** @type {any} */ e) => setQuality(Number(e.currentTarget.value))}>
          ${QUALITIES.map(([v, l]) => html`<option value=${v}>${l} quality</option>`)}</select>
        <span class="dim">depth</span>
        <input type="number" step="0.1" min="0" style="width: 60px" value=${near} onChange=${(/** @type {any} */ e) => setNear(Number(e.currentTarget.value))} />
        <span class="dim">to</span>
        <input type="number" step="0.5" min="0.5" style="width: 60px" value=${far} onChange=${(/** @type {any} */ e) => setFar(Number(e.currentTarget.value))} />
        <span class="dim">m</span>
        ${mode === "snap" && html`<button class="btn primary" disabled=${!shown.length} onClick=${() => shown.forEach(snap)}>Snap all</button>`}
      </div>
      ${cams && html`<div class="row" style="margin-top: 8px">
        ${ac && html`<span class="dim">Arducam</span>
          <select value=${ac.profile} onChange=${(/** @type {any} */ e) => setCam("arducam", { profile: e.currentTarget.value })}>
            ${ac.profiles.map((/** @type {any} */ p) => html`<option value=${p.name}>${p.label}</option>`)}</select>
          <select value=${ac.settings.focus == null ? "" : String(ac.settings.focus)}
            onChange=${(/** @type {any} */ e) => e.currentTarget.value && setCam("arducam", { focus: e.currentTarget.value })}>
            ${FOCUS.map(([v, l]) => html`<option value=${v}>${l}</option>`)}</select>`}
        ${rs && html`<span class="dim" style="margin-left: 8px">D435</span>
          <select value=${rs.profile} onChange=${(/** @type {any} */ e) => setCam("realsense", { profile: e.currentTarget.value })}>
            ${rs.profiles.map((/** @type {any} */ p) => html`<option value=${p.name}>${p.label}</option>`)}</select>
          <label class="row" style="gap: 6px">
            <span class="switch"><input type="checkbox" checked=${!!rs.settings.emitter}
              onChange=${(/** @type {any} */ e) => setCam("realsense", { emitter: e.currentTarget.checked ? "1" : "0" })} /><span></span></span>
            IR projector</label>`}
        ${setErr && html`<span class="warn">${setErr}</span>`}
      </div>`}
      <div class="row" style="margin-top: 8px">
        <label class="row" style="gap: 6px">
          <span class="switch"><input type="checkbox" checked=${tagsOn} onChange=${(/** @type {any} */ e) => setTagsOn(e.currentTarget.checked)} /><span></span></span>
          <b>AprilTags</b></label>
        ${sel(family, setFamily, (detector?.families?.length ? detector.families : ["tag25h9", "tag36h11", "tag16h5"]).map((/** @type {string} */ f) => [f, f]), false)}
        <span class="dim">black square</span>
        <input type="number" step="1" min="10" style="width: 70px" value=${sizeMm} onChange=${(/** @type {any} */ e) => setSizeMm(Number(e.currentTarget.value))} />
        <span class="dim">mm</span>
        ${sel(dec, setDec, DECIMATE)}
        ${detector && !detector.detector ? html`<span class="warn">no detector on the server${detector.error ? `: ${detector.error}` : ""}</span>`
          : detector && html`<span class="dim">detector: ${detector.detector === "apriltag" ? "apriltag (C)" : "OpenCV aruco"}</span>`}
      </div>
      ${off.length > 0 && html`<div class="row" style="margin-top: 8px"><span class="dim">Hidden:</span>
        ${off.map((/** @type {string} */ id) => html`<button class="btn small" onClick=${() => setOff(off.filter((/** @type {string} */ x) => x !== id))}>
          show ${panels.find((p) => p.id === id)?.label || id}</button>`)}</div>`}
    </div>
    <div style="display: grid; grid-template-columns: repeat(auto-fill, minmax(min(420px, 100%), 1fr)); gap: 10px">
      ${shown.map((p) => {
        const on = mode === "live" && liveOn.includes(p.id);
        const src = on && visible ? `${url}/stream?${query(p)}&fps=${fps}&profile=${profileOf(p.cam)}` : "";
        return html`<${Panel} key=${p.id} p=${p} mode=${mode} on=${on} src=${src} shot=${shots[p.id]}
          probe=${probe?.id === p.id ? probe : null} near=${near} far=${far} onSnap=${() => snap(p)}
          onLive=${() => setLiveOn(on ? liveOn.filter((/** @type {string} */ x) => x !== p.id) : [...liveOn, p.id])}
          onHide=${() => setOff([...off, p.id])} onBytes=${(/** @type {number} */ n) => L.bytes.push([Date.now(), n])}
          onClick=${p.stream === "depth" ? (/** @type {any} */ e) => onDepthClick(e, p.id) : null} />`;
      })}
    </div>
  </div>`;
}

/** One picture: its canvas (the frame and its tags), the live stream when on, and the numbers under it.
 * @param {{p: any, mode: string, on: boolean, src: string, shot: any, probe: any, near: number, far: number,
 *   onSnap: () => void, onLive: () => void, onHide: () => void, onBytes: (n: number) => void, onClick: any}} a */
function Panel({ p, mode, on, src, shot, probe, near, far, onSnap, onLive, onHide, onBytes, onClick }) {
  const cv = useRef(/** @type {HTMLCanvasElement | null} */ (null));
  const S = useRef(/** @type {any} */ ({ frames: [], info: null, error: "", lastSeq: null, results: [], byId: {}, drawn: false }));
  const [, tick] = useState(0);
  const depth = p.stream === "depth";

  /** @param {Blob} blob @param {any} info */
  const paint = async (blob, info) => {
    const bmp = await createImageBitmap(blob);
    const c = cv.current;
    if (c) {
      if (c.width !== bmp.width || c.height !== bmp.height) { c.width = bmp.width; c.height = bmp.height; }
      const ctx = /** @type {CanvasRenderingContext2D} */ (c.getContext("2d"));
      ctx.drawImage(bmp, 0, 0);
      if (info.tags) drawTags(ctx, info.tags, c.width, c.height, (info.tags_lag_ms || 0) > 150);
      S.current.drawn = true;
    }
    bmp.close();
  };

  useEffect(() => {                              // a snapshot arrived: draw it
    if (mode !== "snap" || !shot?.blob) return;
    S.current.info = shot.info;
    addSample(S.current, { ...shot.info, tags_seq: shot.info.tags_seq ?? shot.at });
    paint(shot.blob, shot.info).catch(() => {});
  }, [shot?.blob, mode]);

  useEffect(() => {                              // the live stream: newest frame drawn, older ones dropped
    const L = S.current;
    L.lastSeq = null; L.results = []; L.byId = {}; L.frames = []; L.error = "";
    if (!src) return;
    const ac = new AbortController();
    /** @type {any} */ let next = null;
    let busy = false;
    const draw = async () => {
      if (busy || !next) return;
      busy = true;
      const { blob, info } = next;
      next = null;
      try { await paint(blob, info); } catch { /* a broken frame: the next one */ }
      busy = false;
      if (next) draw();
    };
    (async () => {
      while (!ac.signal.aborted) {               // reconnects after an error (the server restarted, a camera hiccup)
        try {
          await readStream(src, ac.signal, (blob, info, n) => {
            L.frames.push([performance.now(), n]);
            L.info = info;
            L.error = info.error || info.tag_error || "";
            onBytes(n);
            addSample(L, info);
            if (blob) { next = { blob, info }; draw(); }
          });
        } catch (e) {
          if (ac.signal.aborted) return;
          L.error = /** @type {Error} */ (e).message;
          await sleep(1500);
        }
      }
    })();
    const h = setInterval(() => tick((x) => x + 1), 250);
    return () => { ac.abort(); clearInterval(h); };
  }, [src]);

  const L = S.current;
  const t = performance.now();
  L.frames = L.frames.filter((/** @type {number[]} */ [ts]) => t - ts < 2000);
  const fps = L.frames.length > 1 ? (L.frames.length - 1) / ((L.frames[L.frames.length - 1][0] - L.frames[0][0]) / 1000) : 0;
  const kB = L.frames.reduce((/** @type {number} */ a, /** @type {number[]} */ f) => a + f[1], 0) / 2 / 1024;
  const i = (mode === "live" ? L.info : shot?.info) || {};
  const err = mode === "live" ? on && L.error : shot?.error;
  const lock = i.tags ? lockStats(L) : [];
  const f = (/** @type {number | null | undefined} */ v, d = 1) => (v == null ? "–" : v.toFixed(d));

  return html`<div class="card" style="margin: 0">
    <div class="row"><b class="grow">${p.label}</b>
      ${mode === "live"
        ? html`<button class=${`btn small${on ? " primary" : ""}`} onClick=${onLive}>${on ? "Live: on" : "Live: off"}</button>`
        : html`<button class="btn small" disabled=${shot?.busy} onClick=${onSnap}>${shot?.busy ? "..." : "Snap"}</button>`}
      <button class="btn small" onClick=${onHide}>Hide</button></div>
    <div style="position: relative; margin-top: 6px">
      <canvas ref=${cv} onClick=${onClick} width="640" height="480"
        style=${`width: 100%; display: ${L.drawn && (mode === "snap" ? shot?.blob : on) ? "block" : "none"}; border-radius: 6px; background: var(--map-bg)${depth ? "; cursor: crosshair" : ""}`}></canvas>
      ${!(L.drawn && (mode === "snap" ? shot?.blob : on)) && html`<div class="dim" style="aspect-ratio: 4 / 3; display: grid; place-items: center; border-radius: 6px; background: var(--map-bg)">
        ${mode === "snap" ? "No picture yet: Snap" : on ? "Connecting..." : "Live: off"}</div>`}
      ${probe && html`<div style=${`position: absolute; left: ${probe.u * 100}%; top: ${probe.v * 100}%; width: 14px; height: 14px;
          margin: -8px 0 0 -8px; border: 2px solid var(--white); border-radius: 50%; pointer-events: none`}></div>`}
    </div>
    ${depth && html`<div class="row" style="margin-top: 4px; font-size: 12px"><span class="dim mono">${near} m</span>
      <span class="grow" style=${`height: 8px; border-radius: 4px; background: ${DEPTH_RAMP}`}></span>
      <span class="dim mono">${far} m</span><span class="dim">black: no depth</span></div>`}
    ${i.width && html`<div class="dim mono" style="font-size: 12px; margin-top: 4px">
      ${[mode === "live" ? `${f(fps)} fps · ${kB.toFixed(0)} kB/s` : `${clock(shot.at)} · ${(shot.bytes / 1024).toFixed(0)} kB · ${shot.ms.toFixed(0)} ms`,
        `${i.width}×${i.height}${i.profile ? ` (${i.profile})` : ""}`, i.age_ms != null && `frame ${i.age_ms.toFixed(0)} ms old`,
        p.stream === "color" && i.exposure_us && `exposure ${(i.exposure_us / 1000).toFixed(1)} ms${i.gain ? `, gain ${i.gain}` : ""}`,
        p.stream === "ir" && i.ir_exposure_us && `IR exposure ${(i.ir_exposure_us / 1000).toFixed(1)} ms`,
        i.detect_ms != null && `tags ${i.detect_ms.toFixed(0)} ms${i.tags_lag_ms ? `, ${i.tags_lag_ms.toFixed(0)} ms behind` : ""}`,
      ].filter(Boolean).join(" · ")}</div>`}
    ${depth && i.width && html`<div class="mono" style="font-size: 12px">
      centre ${i.centre_m != null ? `${i.centre_m.toFixed(2)} m` : "–"} · ${((i.valid || 0) * 100).toFixed(0)}% has depth${i.median_m != null ? ` · 5–95%: ${i.p5_m.toFixed(2)}–${i.p95_m.toFixed(2)} m` : ""}
      ${probe ? ` · clicked: ${probe.m != null ? `${probe.m.toFixed(2)} m` : probe.error || "no depth there"}` : html`<span class="dim"> · click for a range</span>`}</div>`}
    ${i.tags && html`<${TagTable} tags=${i.tags} lock=${lock} focal=${i.focal_px} />`}
    ${err && html`<p class="warn" style="margin: 4px 0 0">${err}</p>`}
  </div>`;
}

/** The tags in the picture: where (camera frame: x right, y down, z ahead), how well seen, the SLAM gates' verdict,
 * and over the last LOCK_S seconds how often found and how much each number scatters.
 * @param {{tags: any[], lock: any[], focal: number}} a */
function TagTable({ tags, lock, focal }) {
  const f = (/** @type {number | null | undefined} */ v, d = 1) => (v == null ? "–" : v.toFixed(d));
  const mm = (/** @type {number | null | undefined} */ v) => (v == null ? "–" : (v * 1000).toFixed(1));
  if (!tags.length && !lock.length) return html`<div class="dim" style="font-size: 12px; margin-top: 4px">No tags in view${focal ? ` (f ${focal.toFixed(0)} px)` : ""}</div>`;
  const th = "padding: 0 0 2px 8px; white-space: nowrap; font-weight: 500";
  const td = "padding: 1px 0 1px 8px; white-space: nowrap";
  return html`<div style="overflow-x: auto; margin-top: 6px">
    <table class="mono" style="width: 100%; font-size: 12px; border-collapse: collapse; text-align: right">
    <thead class="dim"><tr>
      <th style=${th + "; text-align: left; padding-left: 0"}>id</th><th style=${th} title="range to the tag's centre, m">range</th>
      <th style=${th} title="camera frame, m: x right, y down, z ahead">x y z</th>
      <th style=${th} title="bearing right, elevation up, degrees">brg elev</th>
      <th style=${th} title="angle between the view ray and the tag's normal, degrees">off-axis</th>
      <th style=${th} title="the black square's mean side, px">side</th><th style=${th} title="the decoder's decision margin (the SLAM's gate: 30)">margin</th>
      <th style=${th} title="reprojection RMS of the fit, px / of the mirror pose (none: no second solution)">fit / mirror</th>
      <th style=${th} title="depth over the tag / the pose's distance (D435)">depth</th>
      <th style=${th + "; text-align: left"}>SLAM</th></tr></thead>
    <tbody>${tags.map((t) => html`<tr>
      <td style=${td + "; text-align: left; padding-left: 0"}>${t.id}</td><td style=${td}>${f(t.range, 3)}</td>
      <td style=${td}>${t.xyz ? t.xyz.map((/** @type {number} */ v) => v.toFixed(3)).join(" ") : "–"}</td>
      <td style=${td}>${f(t.bearing_deg)} ${f(t.elevation_deg)}</td><td style=${td}>${f(t.angle_deg)}°</td><td style=${td}>${f(t.side_px)}</td>
      <td style=${td}>${f(t.margin, 0)}</td><td style=${td}>${f(t.rms_px, 2)} / ${t.alt_rms_px == null ? "none" : f(t.alt_rms_px, 2)}</td>
      <td style=${td} title=${t.depth ? `depth ${f(t.depth.depth_z, 3)} m, pose ${f(t.depth.pose_z, 3)} m, ${f(100 * t.depth.valid, 0)}% of the tag has depth` : ""}>
        ${t.depth?.ratio ? `×${t.depth.ratio.toFixed(3)}` : "–"}</td>
      <td style=${td + "; text-align: left"} class=${t.ok === undefined ? "dim" : !t.ok ? "err" : t.ambiguous ? "warn" : "ok"}>
        ${t.ok === undefined ? "no pose" : !t.ok ? t.reasons.join(", ") : t.ambiguous ? "ambiguous" : "usable"}</td></tr>`)}
    </tbody></table>
    ${lock.length > 0 && html`<table class="mono" style="width: 100%; font-size: 12px; margin-top: 6px; border-collapse: collapse; text-align: right">
      <thead class="dim"><tr><th style=${th + "; text-align: left; padding-left: 0"}>last ${LOCK_S} s</th><th style=${th}>found</th>
        <th style=${th} title="scatter (standard deviation) of the range, mm">range ±mm</th>
        <th style=${th} title="scatter of x, y, z, mm">x y z ±mm</th>
        <th style=${th} title="scatter of the corners, px RMS per coordinate">corners ±px</th>
        <th style=${th} title="scatter of the face's direction: yaw, pitch, degrees">face ±°</th><th style=${th}>usable</th></tr></thead>
      <tbody>${lock.map((s) => html`<tr>
        <td style=${td + "; text-align: left; padding-left: 0"}>id ${s.id} (${s.n})</td><td style=${td}>${f(100 * s.rate, 0)}%</td>
        <td style=${td}>${mm(s.rangeSd)}</td><td style=${td}>${s.xyz.map((/** @type {number | null} */ v) => mm(v)).join(" ")}</td>
        <td style=${td}>${f(s.csd, 3)}</td><td style=${td}>${f(s.yawSd, 2)} ${f(s.pitchSd, 2)}</td><td style=${td}>${f(100 * s.okShare, 0)}%</td></tr>`)}</tbody></table>`}
  </div>`;
}

