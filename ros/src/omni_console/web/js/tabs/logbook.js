// @ts-check
/* Logbook: the robot's session logs (omni-log/1, omni_console/session_log.py), read straight from the page server
   over HTTP (never through ROS) and replayed here: where the robot drove on a timeline, its throws, its messages. The
   same files are what simulation/v11 sil/studies/replay_log.py replays in the sim. */
import { html, useEffect, useState } from "../../vendor/htm-preact-standalone.mjs";
import { MapView } from "../mapview.js";
import { ThrowView, kmh } from "../throwview.js";
import { HelpBox, clock } from "../ui.js";

export function LogbookTab() {
  const [list, setList] = useState(/** @type {any[] | null} */ (null));
  const [sel, setSel] = useState("");
  const [log, setLog] = useState(/** @type {any} */ (null));
  const [t, setT] = useState(0);
  const [th, setTh] = useState(/** @type {any} */ (null));
  const [err, setErr] = useState("");
  const load = () => fetch("logs/", { cache: "no-store" }).then((r) => r.json()).then(setList).catch(() => setList([]));
  useEffect(() => { load(); }, []);
  const open = async (/** @type {string} */ name) => {
    setSel(name); setLog(null); setTh(null); setErr("");
    try {
      const text = await (await fetch(`logs/${encodeURIComponent(name)}`, { cache: "no-store" })).text();
      const recs = text.split("\n").filter(Boolean).map((l) => { try { return JSON.parse(l); } catch { return null; } }).filter(Boolean);
      const head = recs.find((r) => r.type === "session");
      const by = (/** @type {string} */ k) => recs.filter((r) => r.type === k);
      const end = recs.length ? recs[recs.length - 1].t : 0;
      setLog({ head, poses: by("pose"), status: by("status"), throws: by("throw"), events: by("event"), end });
      setT(end);
    } catch (e) { setErr(String(e)); }
  };
  const poses = log?.poses || [];
  const upto = poses.filter((/** @type {any} */ p) => p.t <= t);
  const here = upto[upto.length - 1];
  const stat = (log?.status || []).filter((/** @type {any} */ p) => p.t <= t).slice(-1)[0];
  return html`<div class="cols">
    <div class="side">
      <${HelpBox} tab="logbook">
        The robot writes a log of every session (from the first mode until it's been idle 30 s): where it drove, its
        status each second, every throw and every message. Pick one to replay it on the map with the slider, look at
        its throws, or download it. The sim writes the same format, so a session from the real robot can be replayed
        in the sim to compare (studies/replay_log.py).
      <//>
      <div class="row"><h3 class="grow" style="margin: 0">Sessions</h3><button class="btn small" onClick=${load}>Refresh</button></div>
      ${list === null ? html`<p class="dim">Loading...</p>` : list.length ? html`<ul class="list">${list.map((f) => html`<li>
        <a href="#logbook" class=${`grow${f.name === sel ? " sel" : ""}`} onClick=${(/** @type {any} */ e) => { e.preventDefault(); open(f.name); }}>
          ${f.name.replace(".jsonl", "")}</a>
        <span class="dim" style="font-size: 12px">${f.summary ? `${Math.round(f.summary.duration_s / 60)} min, ${f.summary.throws} throws` : "open"}</span>
        <a class="btn small" href=${`logs/${encodeURIComponent(f.name)}`} download=${f.name} title="Download">⇩</a></li>`)}</ul>`
        : html`<p class="dim">No sessions yet (or this page's server has no logs: the robot's console writes them).</p>`}
      ${err && html`<p class="err">${err}</p>`}
      ${log && html`<div class="card" style="margin-top: 10px">
        <div class="kv">
          <span class="k">started</span><span>${new Date(log.head.start * 1000).toLocaleString()}</span>
          <span class="k">source</span><span>${log.head.source}${log.head.map ? `, map '${log.head.map}'` : ""}</span>
          <span class="k">firmware</span><span class="mono">${log.head.firmware?.build || "–"}</span>
          <span class="k">length</span><span>${Math.round(log.end / 60)} min, ${poses.length} poses, ${log.throws.length} throws</span>
        </div></div>
        <h3>Throws</h3>
        ${log.throws.length ? html`<table class="t"><tbody>${log.throws.map((/** @type {any} */ r) => html`<tr class=${th === r ? "me" : ""}
          onClick=${() => { setTh(r); setT(r.t); }} style="cursor: pointer">
          <td>${clock(log.head.start + r.t)}</td><td>${r.player || ""}</td><td>${kmh(r.speed)}</td><td class=${r.caught ? "ok" : "dim"}>${r.caught ? "caught" : "missed"}</td></tr>`)}</tbody></table>`
          : html`<p class="dim">None.</p>`}
        <h3>Messages</h3>
        <div style="max-height: 200px; overflow: auto">${log.events.map((/** @type {any} */ e) => html`<div class="ev" style="grid-template-columns: 60px 1fr">
          <span class="mono dim">${clock(log.head.start + e.t)}</span><span class=${e.level === "error" ? "err" : e.level === "warn" ? "warn" : ""}>${e.text}</span></div>`)}</div>`}
    </div>
    <div class="main" style="display: flex; flex-direction: column">
      ${log && html`<div class="card" style="margin: 8px; display: flex; gap: 10px; align-items: center">
        <input type="range" class="grow" min="0" max=${log.end} step="0.5" value=${t}
          onInput=${(/** @type {any} */ e) => setT(+e.currentTarget.value)} />
        <span class="mono">${clock(log.head.start + t)}</span>
        <span class="dim">${stat ? `${stat.mission}: ${stat.task}` : ""}</span></div>`}
      ${th && html`<div class="card" style="margin: 0 8px 8px"><${ThrowView} rec=${th} height=${180} /></div>`}
      <div style="position: relative; flex: 1; min-height: 300px">
        <${MapView} trail=${upto.map((/** @type {any} */ p) => [p.x, p.y])} ghost=${here ? { x: here.x, y: here.y, yaw: here.yaw } : null}
          hint=${log ? "The green line: where it drove; the dashed ring: where it was at the slider's time" : "Pick a session"} />
      </div>
    </div>
  </div>`;
}
