// @ts-check
import { html, useEffect, useRef, useState } from "../../vendor/htm-preact-standalone.mjs";
import { useStore, useTopic } from "../ros.js";
import { HelpBox } from "../ui.js";

/** The camera picture's URL. A preview topic (omni_perception image_preview: small, a few frames a second, already
 *  JPEG) goes through web_video_server as it is (ros_compressed: nothing is encoded per viewer); any other image
 *  topic is encoded by web_video_server for this viewer, so it's asked small.
 *  @param {string} host @param {number} port @param {string} topic */
export function streamUrl(host, port, topic) {
  // web_video_server takes the topic as it is (an encoded "/" is an invalid name to it)
  if (topic.endsWith("/compressed")) {
    const base = topic.slice(0, -"/compressed".length);
    return `http://${host}:${port}/stream?topic=${encodeURI(base)}&type=ros_compressed`;
  }
  return `http://${host}:${port}/stream?topic=${encodeURI(topic)}&type=mjpeg&quality=50&width=320&height=200`;
}

export function VisionTab() {
  const s = useStore("status");
  const v = s.status?.vision;
  // previews first: they cost the robot nothing per viewer
  const images = (v?.images || []).slice().sort((/** @type {string} */ a, /** @type {string} */ b) =>
    Number(b.endsWith("/preview/compressed")) - Number(a.endsWith("/preview/compressed")));
  const [topic, setTopic] = useState("");
  const sel = topic || images[0] || "";
  const tr = v?.track;
  const p = v?.perception;
  const host = location.hostname || "localhost";
  return html`<div class="full">
    <${HelpBox} tab="vision">
      The stereo cameras find the ball by colour, the filter turns two views into a 3D track (with air drag), and the
      catch code predicts where it will cross the catch height. <b>Track</b> shows that filter. The camera picture
      comes as a video stream from the robot (web_video_server), never through the data link: a small preview, a few
      frames a second (the robot encodes it once, whoever watches). In the sim the cameras
      send only the ball-coloured pixels, drawn here as blobs: the ball (yellow) and clutter that has the ball's colour
      (grey).
    <//>
    <div class="row" style="align-items: flex-start; gap: 10px">
      <div class="card grow" style="min-width: 320px">
        <h2>Camera</h2>
        ${images.length && v?.video_port ? html`<div class="row" style="margin-bottom: 6px">
            <select value=${sel} onChange=${(/** @type {any} */ e) => setTopic(e.currentTarget.value)}>
              ${images.map((/** @type {string} */ t) => html`<option value=${t}>${t}</option>`)}</select></div>
            <img style="width: 100%; border-radius: 6px; background: var(--map-bg)" alt="camera"
              src=${streamUrl(host, v.video_port, sel)} />`
          : html`<${SparseView} />${images.length && !v?.video_port ? html`<p class="warn">Image topics exist but web_video_server isn't running (sudo apt install ros-jazzy-web-video-server).</p>` : ""}`}
      </div>
      <div style="width: 340px">
        <div class="card"><h2>Track</h2>
          ${tr ? html`<div class="kv">
            <span class="k">tracking</span><span class=${tr.tracking ? "ok" : "dim"}>${tr.tracking ? (tr.ready ? "yes, ready to catch" : "yes, settling") : "no ball"}</span>
            <span class="k">frames</span><span class="mono">${tr.frames}</span>
            <span class="k">position</span><span class="mono">${tr.tracking ? tr.x.slice(0, 3).map((/** @type {number} */ q) => q.toFixed(2)).join(", ") + " m" : "–"}</span>
            <span class="k">speed</span><span class="mono">${tr.tracking ? Math.hypot(tr.x[3], tr.x[4], tr.x[5]).toFixed(1) + " m/s" : "–"}</span>
            <span class="k">lands at</span><span class="mono">${tr.tracking ? tr.crossing.slice(0, 2).map((/** @type {number} */ q) => q.toFixed(2)).join(", ") + ` m ± ${tr.crossing_sigma < 1 ? `${(tr.crossing_sigma * 100).toFixed(0)} cm` : `${tr.crossing_sigma.toFixed(1)} m`}` : "–"}</span>
            <span class="k">in</span><span class="mono">${tr.tracking ? `${tr.t_go.toFixed(2)} s` : "–"}</span>
          </div>` : html`<p class="dim">No ball track (the catch code isn't running: 'Play catch').</p>`}</div>
        <div class="card"><h2>Detector</h2>
          ${p ? html`<div class="kv">${Object.entries(p).slice(0, 14).map(([k, val]) => html`<span class="k">${k}</span>
            <span class="mono">${typeof val === "number" ? Math.round(val * 1000) / 1000 : JSON.stringify(val)}</span>`)}</div>`
            : html`<p class="dim">No detector report yet.</p>`}</div>
      </div>
    </div>
  </div>`;
}

/** The sim's sparse frames, drawn as blobs on the camera's frame. */
function SparseView() {
  const cv = useRef(/** @type {HTMLCanvasElement | null} */ (null));
  const [info, setInfo] = useState("Waiting for camera frames (the catch step sends them)...");
  const last = useRef(/** @type {any} */ (null));
  useTopic("/console/camera", (d) => { last.current = d; draw(); });
  const draw = () => {
    const c = cv.current, d = last.current;
    if (!c || !d) return;
    const css = getComputedStyle(document.documentElement);
    const tok = (/** @type {string} */ k) => css.getPropertyValue(`--${k}`).trim();
    const W = Math.min(c.parentElement?.clientWidth || 640, Math.round(460 * d.w / d.h)), H = Math.round(W * d.h / d.w);
    c.width = W; c.height = H;
    const ctx = /** @type {CanvasRenderingContext2D} */ (c.getContext("2d"));
    const k = W / d.w;
    ctx.fillStyle = tok("map-bg"); ctx.fillRect(0, 0, W, H);
    ctx.strokeStyle = tok("border"); ctx.strokeRect(0.5, 0.5, W - 1, H - 1);
    ctx.fillStyle = tok("dim");
    for (let i = 0; i < d.clutter.length; i += 2) ctx.fillRect(d.clutter[i] * k - 1, d.clutter[i + 1] * k - 1, 2, 2);
    if (d.ball) {
      ctx.fillStyle = tok("yellow");
      ctx.beginPath(); ctx.arc(d.ball[0] * k, d.ball[1] * k, Math.max(3, d.ball[2] * k), 0, 2 * Math.PI); ctx.fill();
      ctx.strokeStyle = tok("yellow"); ctx.lineWidth = 1.5;          // a ring: a far ball is a few pixels
      ctx.beginPath(); ctx.arc(d.ball[0] * k, d.ball[1] * k, 14, 0, 2 * Math.PI); ctx.stroke();
    }
    setInfo(`${d.cam || "left"} camera, ${d.w}×${d.h}: ${d.ball ? `ball at (${d.ball[0]}, ${d.ball[1]}), ${d.ball[3]} px` : "no ball"}, ${d.clutter.length / 2} clutter px shown`);
  };
  useEffect(() => { draw(); }, []);
  return html`<canvas ref=${cv} style="display: block; border-radius: 6px"></canvas>
    <div class="dim" style="font-size: 12px; margin-top: 4px">${info}</div>`;
}
