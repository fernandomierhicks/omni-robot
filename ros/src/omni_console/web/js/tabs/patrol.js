// @ts-check
import { html, useState } from "../../vendor/htm-preact-standalone.mjs";
import { MapView } from "../mapview.js";
import { act, useStore } from "../ros.js";
import { Banner, DrivingPicker, HelpBox, SettingsForm, deg } from "../ui.js";

export function PatrolTab() {
  const s = useStore("status", "schema");
  const st = s.status;
  const routes = s.schema?.routes || {};
  const nav = !!st?.caps?.nav;
  const pat = st?.mission === "PATROL" ? st.patrol : null;
  const [draft, setDraft] = useState(/** @type {number[][] | null} */ (null));
  const [name, setName] = useState("");
  const [shown, setShown] = useState(/** @type {string | null} */ (null));
  const editing = draft !== null;
  const route = editing ? draft : pat ? routes[pat.route] : shown ? routes[shown] : null;
  const marks = pat && route ? [{ x: route[pat.i][0], y: route[pat.i][1], color: "blue", label: "next" }] : [];
  const save = () => act("route_save", { name: name.trim(), points: draft }).then((r) => {
    if (r.ok) { setShown(name.trim()); setDraft(null); setName(""); }
  });
  return html`<div class="cols">
    <div class="side">
      <${HelpBox} tab="patrol">
        <ol>
          <li><b>Make a route</b>: press <b>New route</b>, then click the map at each point in order. Drag before
            letting go to set which way the robot faces there. Right-click removes the last point.</li>
          <li>Name it and press <b>Save</b>. Routes belong to the map they were made on.</li>
          <li>Press <b>Run</b> next to a route. The robot drives point to point, waits at each (setting below), and
            goes round as set in <b>Order</b> and <b>Laps</b>.</li>
        </ol>
        <b>Stop</b> ends the patrol where the robot is; <kbd>Space</kbd> also disarms. A point it can't reach is
        skipped; three in a row stop the patrol.
      <//>
      ${!nav && html`<${Banner} title="Navigation isn't running.">Start 'Robot (all modes)' (or 'Navigate') in the Launcher.<//>`}
      <${DrivingPicker} />
      ${pat && html`<div class="card">
        <div class="row"><b class="grow">${st.task}</b><button class="btn danger small" onClick=${() => act("cancel")}>Stop</button></div>
        <div class="bar" style="margin-top: 6px"><div style=${`width: ${Math.round((st.progress || 0) * 100)}%`}></div></div></div>`}
      ${editing ? html`<div class="card">
          <div class="row"><b class="grow">New route: ${draft.length} points</b>
            <button class="btn small" onClick=${() => setDraft(draft.slice(0, -1))} disabled=${!draft.length}>Undo</button>
            <button class="btn small" onClick=${() => setDraft(null)}>Cancel</button></div>
          <ol style="margin: 6px 0; padding-left: 22px; font-size: 13px">${draft.map((p) => html`<li class="mono">
            ${p[0].toFixed(1)}, ${p[1].toFixed(1)} m, facing ${deg(p[2])}°</li>`)}</ol>
          <div class="row"><input type="text" class="grow" placeholder="Route name, e.g. evening" value=${name}
            onInput=${(/** @type {any} */ e) => setName(e.currentTarget.value)} />
            <button class="btn primary small" disabled=${!name.trim() || draft.length < 2} onClick=${save}>Save</button></div></div>`
        : html`<button class="btn big" style="margin-bottom: 10px" onClick=${() => { setDraft([]); setShown(null); }}>New route</button>`}
      <h3>Routes${s.schema?.map ? ` on '${s.schema.map}'` : ""}</h3>
      ${Object.keys(routes).length ? html`<ul class="list">${Object.entries(routes).map(([n, pts]) => html`<li>
          <span class="grow">${n} <span class="dim">(${pts.length})</span></span>
          <button class="btn small go" disabled=${!nav} onClick=${() => { setShown(n); act("patrol", { route: n }); }}>Run</button>
          <button class="btn small" onClick=${() => setShown(shown === n ? null : n)}>${shown === n ? "Hide" : "Show"}</button>
          <button class="btn small" title="Copy into a new route to change it" onClick=${() => { setDraft(pts.map((p) => p.slice())); setName(n); }}>Edit</button>
          <button class="btn small danger" onClick=${() => { if (confirm(`Delete route '${n}'?`)) act("route_delete", { name: n }); }}>✕</button>
        </li>`)}</ul>` : html`<p class="dim">No routes on this map yet.</p>`}
      <${SettingsForm} tab="patrol" />
    </div>
    <div class="main">
      <${MapView} goalMode=${editing} route=${route} marks=${marks}
        onGoal=${(/** @type {any} */ g) => setDraft((draft || []).concat([[g.x, g.y, g.yaw]]))}
        onClear=${() => setDraft((draft || []).slice(0, -1))}
        hint=${editing ? `Click to add point ${draft.length + 1} (drag sets its heading); right-click removes the last` :
          pat ? `Patrolling '${pat.route}'` : "Press 'New route' to draw one, or Show a saved one"} />
    </div>
  </div>`;
}
