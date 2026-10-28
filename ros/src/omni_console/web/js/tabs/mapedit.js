// @ts-check
/* Map edit: no-go zones (Nav2 plans around them: its keepout filter) and floor edges (rug edges, thresholds: a slow
   band either side, for Nav2 and for the stick), drawn on the map and kept with it (omni_console/zones.py). */
import { html, useState } from "../../vendor/htm-preact-standalone.mjs";
import { MapView } from "../mapview.js";
import { act, useStore } from "../ros.js";
import { Banner, HelpBox } from "../ui.js";

const KINDS = ["rug", "threshold", "step", "other"];

export function MapEditTab() {
  const s = useStore("status", "schema", "map");
  const st = s.status;
  const sc = s.schema;
  const zones = sc?.zones || {}, edges = sc?.edges || {};
  const [tool, setTool] = useState(/** @type {"zone" | "edge" | null} */ (null));
  const [draft, setDraft] = useState(/** @type {number[][]} */ ([]));
  const [name, setName] = useState("");
  const [kind, setKind] = useState("rug");
  const [height, setHeight] = useState(12);
  const [speed, setSpeed] = useState(1.0);
  const onMap = st?.pose?.frame === "map" || !!s.map;
  const need = tool === "zone" ? 3 : 2;
  const begin = (/** @type {"zone" | "edge"} */ t) => { setTool(t); setDraft([]); setName(""); };
  const save = async () => {
    const r = tool === "zone" ? await act("zone_save", { name: name.trim(), points: draft })
      : await act("edge_save", { name: name.trim(), points: draft, kind, height_mm: height, speed });
    if (r.ok) { setTool(null); setDraft([]); setName(""); }
  };
  return html`<div class="cols">
    <div class="side">
      <${HelpBox} tab="mapedit">
        <ul>
          <li><b>No-go zone</b>: a place the robot must never drive into (the stairs, the kitchen while cooking, the
            dog's bowl). Click its corners on the map, name it, Save. Go To, Explore, Patrol and programs plan around
            it.</li>
          <li><b>Floor edge</b>: a rug edge or a door threshold. Click along it, give its height and the speed to cross
            it at (1 m/s or less for a threshold), Save. Within 35 cm of it the robot slows down, in every mode,
            Manual included. The heights are kept for the robot's terrain assist later.</li>
        </ul>
        They belong to the map they're drawn on. Right-click removes the last point.
      <//>
      ${!onMap && html`<${Banner} title="No map.">Zones and edges are drawn on a map: load one in the SLAM tab (or start mapping).<//>`}
      ${tool ? html`<div class="card">
          <div class="row"><b class="grow">New ${tool === "zone" ? "no-go zone" : "floor edge"}: ${draft.length} points</b>
            <button class="btn small" disabled=${!draft.length} onClick=${() => setDraft(draft.slice(0, -1))}>Undo</button>
            <button class="btn small" onClick=${() => { setTool(null); setDraft([]); }}>Cancel</button></div>
          ${tool === "edge" && html`<div class="kv" style="margin: 8px 0">
            <span class="k">kind</span><select value=${kind} onChange=${(/** @type {any} */ e) => setKind(e.currentTarget.value)}>
              ${KINDS.map((k) => html`<option value=${k}>${k}</option>`)}</select>
            <span class="k">height</span><span><input type="number" min="0" max="60" step="1" value=${height}
              onChange=${(/** @type {any} */ e) => setHeight(+e.currentTarget.value)} /> mm</span>
            <span class="k">cross at</span><span><input type="number" min="0.1" max="2" step="0.1" value=${speed}
              onChange=${(/** @type {any} */ e) => setSpeed(+e.currentTarget.value)} /> m/s</span></div>`}
          <div class="row"><input type="text" class="grow" placeholder=${tool === "zone" ? "Name, e.g. stairs" : "Name, e.g. hall rug"}
            value=${name} onInput=${(/** @type {any} */ e) => setName(e.currentTarget.value)} />
            <button class="btn primary small" disabled=${!name.trim() || draft.length < need} onClick=${save}>Save</button></div></div>`
        : html`<div class="row" style="gap: 8px; margin-bottom: 10px">
          <button class="btn grow" disabled=${!onMap} onClick=${() => begin("zone")}>New no-go zone</button>
          <button class="btn grow" disabled=${!onMap} onClick=${() => begin("edge")}>New floor edge</button></div>`}
      <h3>No-go zones${sc?.map ? ` on '${sc.map}'` : ""}</h3>
      ${Object.keys(zones).length ? html`<ul class="list">${Object.entries(zones).map(([n, z]) => html`<li>
        <span class="grow">${n} <span class="dim">(${z.length} corners)</span></span>
        <button class="btn small danger" onClick=${() => { if (confirm(`Delete no-go zone '${n}'?`)) act("zone_delete", { name: n }); }}>✕</button></li>`)}</ul>`
        : html`<p class="dim">None.</p>`}
      <h3>Floor edges</h3>
      ${Object.keys(edges).length ? html`<ul class="list">${Object.entries(edges).map(([n, e]) => html`<li>
        <span class="grow">${n} <span class="dim">${e.kind}, ${e.height_mm} mm, ${e.speed} m/s</span></span>
        <button class="btn small danger" onClick=${() => { if (confirm(`Delete edge '${n}'?`)) act("edge_delete", { name: n }); }}>✕</button></li>`)}</ul>`
        : html`<p class="dim">None.</p>`}
      ${st?.slow && html`<div class="card warn">Slow zone now: ${st.slow.edge}, ${st.slow.speed} m/s</div>`}
    </div>
    <div class="main">
      <${MapView} zones=${zones} edges=${edges} draft=${tool ? draft : null} draftClosed=${tool === "zone"}
        pointMode=${!!tool} onPoint=${(/** @type {any} */ q) => setDraft(draft.concat([[q.x, q.y]]))}
        onClear=${() => setDraft(draft.slice(0, -1))}
        hint=${tool ? `Click to add point ${draft.length + 1}${draft.length >= need ? ", then name it and Save" : ""}; right-click removes the last`
          : "Pick 'New no-go zone' or 'New floor edge'"} />
    </div>
  </div>`;
}
