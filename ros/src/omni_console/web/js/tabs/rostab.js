// @ts-check
/* The ROS tab: what's running underneath, for debugging. Nodes, topics and services come from rosapi on demand; a
   topic's rate is measured for 3 s when asked (the raw bytes, so even a camera topic won't flood the page for long). */
import { html, useEffect, useState } from "../../vendor/htm-preact-standalone.mjs";
import { getRos, setRosUrl, store, useStore } from "../ros.js";
import { HelpBox } from "../ui.js";

const ROSLIB = /** @type {any} */ (window).ROSLIB;

export function RosTab() {
  const s = useStore("conn", "rtt", "statusHz", "url");
  const [nodes, setNodes] = useState(/** @type {string[]} */ ([]));
  const [topics, setTopics] = useState(/** @type {{name: string, type: string}[]} */ ([]));
  const [services, setServices] = useState(/** @type {string[]} */ ([]));
  const [rates, setRates] = useState(/** @type {Record<string, string>} */ ({}));
  const [filter, setFilter] = useState("");
  const [url, setUrl] = useState("");
  const [fps, setFps] = useState(0);
  const refresh = () => {
    const ros = getRos();
    if (!ros || store.conn !== "open") return;
    ros.getNodes((/** @type {string[]} */ n) => setNodes(n.slice().sort()), () => {});
    ros.getTopics((/** @type {any} */ r) => setTopics(r.topics.map((/** @type {string} */ n, /** @type {number} */ i) => ({ name: n, type: r.types[i] }))
      .sort((/** @type {any} */ a, /** @type {any} */ b) => a.name.localeCompare(b.name))), () => {});
    ros.getServices((/** @type {string[]} */ sv) => setServices(sv.slice().sort()), () => {});
  };
  useEffect(() => { refresh(); }, [s.conn]);
  useEffect(() => {                        // the page's own frame rate (it should sit at the display's)
    let n = 0, raf = 0, t0 = performance.now();
    const f = () => { n++; const t = performance.now(); if (t - t0 > 1000) { setFps(Math.round(n * 1000 / (t - t0))); n = 0; t0 = t; } raf = requestAnimationFrame(f); };
    raf = requestAnimationFrame(f);
    return () => cancelAnimationFrame(raf);
  }, []);
  /** @param {{name: string, type: string}} t */
  const measure = (t) => {
    const ros = getRos();
    if (!ros) return;
    setRates((r) => ({ ...r, [t.name]: "measuring..." }));
    let n = 0, bytes = 0;
    // the publisher's bytes (cbor-raw): nothing to convert on the robot, the exact size, and the same mode as the
    // page's raw topics (rosbridge shares one subscription per topic, raw or not, between its clients)
    const sub = new ROSLIB.Topic({ ros, name: t.name, messageType: t.type, compression: "cbor-raw", queue_length: 10 });
    sub.subscribe((/** @type {any} */ m) => { n++; bytes += m.bytes?.length || 0; });
    setTimeout(() => {
      sub.unsubscribe();
      setRates((r) => ({ ...r, [t.name]: `${(n / 3).toFixed(1)} Hz${n ? `, ~${Math.round(bytes / n / 100) / 10} kB` : ""}` }));
    }, 3000);
  };
  const fox = `https://app.foxglove.dev/?ds=foxglove-websocket&ds.url=${encodeURIComponent(`ws://${location.hostname || "localhost"}:${store.foxglovePort}`)}`;
  const f = filter.toLowerCase();
  return html`<div class="full">
    <${HelpBox} tab="ros">
      This tab is for debugging; driving and playing never need it. For plots, 3D, camera images and recordings,
      open <b>Foxglove</b> (the desktop app connects to <span class="mono">ws://${location.hostname || "localhost"}:${store.foxglovePort}</span>;
      import <span class="mono">ros/foxglove/omni_layout.json</span> once). <b>Rate</b> listens to a topic for 3 s.
    <//>
    <div class="row" style="align-items: stretch; gap: 10px">
      <div class="card grow">
        <h2>Connection</h2>
        <div class="kv">
          <span class="k">rosbridge</span><span class="mono">${s.url} (${s.conn})</span>
          <span class="k">round trip</span><span>${s.rtt != null ? `${s.rtt} ms` : "–"}</span>
          <span class="k">status updates</span><span>${s.statusHz} / s (10 expected)</span>
          <span class="k">page frame rate</span><span>${fps} fps</span>
        </div>
        <div class="row" style="margin-top: 8px">
          <input type="text" class="grow" placeholder="ws://192.168.1.40:9090 (empty = this page's robot)" value=${url}
            onInput=${(/** @type {any} */ e) => setUrl(e.currentTarget.value)} />
          <button class="btn small" onClick=${() => setRosUrl(url.trim())}>Connect</button>
        </div>
      </div>
      <div class="card">
        <h2>Foxglove</h2>
        <p class="dim" style="margin: 0 0 8px; max-width: 320px">Deep debugging: every topic, 3D, plots, images.</p>
        <a class="btn primary" href=${fox} target="_blank" rel="noopener">Open in Foxglove</a>
      </div>
    </div>
    <div class="row" style="margin: 6px 0"><input type="text" placeholder="Filter" value=${filter}
      onInput=${(/** @type {any} */ e) => setFilter(e.currentTarget.value)} /><button class="btn small" onClick=${refresh}>Refresh</button></div>
    <div class="row" style="align-items: flex-start; gap: 10px">
      <div class="card grow" style="max-height: 60vh; overflow: auto">
        <h2>Topics (${topics.length})</h2>
        <table class="t"><thead><tr><th>Topic</th><th>Type</th><th>Rate</th></tr></thead><tbody>
          ${topics.filter((t) => !f || t.name.toLowerCase().includes(f)).map((t) => html`<tr>
            <td class="mono">${t.name}</td><td class="dim">${t.type}</td>
            <td>${rates[t.name] || html`<button class="btn small" onClick=${() => measure(t)}>Rate</button>`}</td></tr>`)}
        </tbody></table>
      </div>
      <div style="width: 320px">
        <div class="card" style="max-height: 30vh; overflow: auto"><h2>Nodes (${nodes.length})</h2>
          <ul class="list">${nodes.filter((n) => !f || n.toLowerCase().includes(f)).map((n) => html`<li class="mono">${n}</li>`)}</ul></div>
        <div class="card" style="max-height: 28vh; overflow: auto"><h2>Services (${services.length})</h2>
          <ul class="list">${services.filter((n) => !f || n.toLowerCase().includes(f)).map((n) => html`<li class="mono" style="font-size: 12px">${n}</li>`)}</ul></div>
      </div>
    </div>
    <details class="card"><summary>Raw console status</summary><${RawStatus} /></details>
  </div>`;
}

function RawStatus() {
  const s = useStore("status");
  return html`<pre class="raw">${JSON.stringify(s.status, null, 1)}</pre>`;
}
