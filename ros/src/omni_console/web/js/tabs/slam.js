// @ts-check
import { html, useState } from "../../vendor/htm-preact-standalone.mjs";
import { MapView } from "../mapview.js";
import { act, useStore } from "../ros.js";
import { Banner, HelpBox } from "../ui.js";

export function SlamTab() {
  const s = useStore("status", "map");
  const st = s.status;
  const slam = st?.slam || {};
  const maps = slam.maps || [];
  const [name, setName] = useState("");
  const [pick, setPick] = useState("");
  const [placing, setPlacing] = useState(false);
  const sel = pick || maps[0] || "";
  const state = slam.busy ? "Starting slam_toolbox..." : slam.mode === "mapping" ? "Making a new map"
    : slam.mode === "localization" ? `Localizing on '${slam.map || "?"}'` : "Not running";
  const area = s.map ? (() => {
    let n = 0;
    for (const v of s.map.data) if (v >= 0) n++;
    return Math.round(n * s.map.info.resolution ** 2);
  })() : null;
  return html`<div class="cols">
    <div class="side">
      <${HelpBox} tab="slam">
        SLAM builds the map from the lidar and keeps track of where the robot is on it.
        <ul>
          <li><b>New map</b>: start from nothing. Drive around (Manual) or let it <b>Explore</b>; the map grows as the
            lidar sees more.</li>
          <li><b>Save as</b> (while a map is being made): keep it under a name (it goes to <span class="mono">ros/maps</span>, with its places and
            routes).</li>
          <li><b>Load</b>: localize on a saved map. The robot assumes it stands where the map started; if it doesn't,
            press <b>Set position</b> and click (and drag the heading) where it really is. <b>Load here</b> keeps the current
            position, for the map you just saved.</li>
        </ul>
        Places (Go To) and routes (Patrol) belong to a map and switch with it.
      <//>
      ${!slam.managed && html`<${Banner} title="This step runs SLAM itself.">Saving works here; new maps and loading need 'Robot (all modes)' in the Launcher.<//>`}
      <div class="card">
        <div style="font-size: 16px"><b>${state}</b></div>
        <div class="kv" style="margin-top: 6px">
          <span class="k">map frame</span><span>${st?.pose?.frame === "map" ? "yes: positions are on the map" : "not yet"}</span>
          <span class="k">known area</span><span>${area != null ? `${area} m²` : "–"}</span>
          ${s.map && html`<span class="k">size</span><span>${(s.map.info.width * s.map.info.resolution).toFixed(1)} × ${(s.map.info.height * s.map.info.resolution).toFixed(1)} m, ${Math.round(s.map.info.resolution * 100)} cm cells</span>`}
        </div>
      </div>
      <h3>New map</h3>
      <button class="btn big" disabled=${!slam.managed || slam.busy}
        onClick=${() => { if (confirm("Start a new map? The current one is lost unless you saved it.")) act("slam_new"); }}>New map</button>
      <h3>Save as</h3>
      <div class="row"><input type="text" class="grow" placeholder="Map name, e.g. house" value=${name}
        onInput=${(/** @type {any} */ e) => setName(e.currentTarget.value)} />
        <button class="btn primary" disabled=${!name.trim() || slam.mode !== "mapping"}
          onClick=${() => act("slam_save", { name: name.trim() }).then((r) => { if (r.ok) setName(""); })}>Save</button></div>
      ${slam.mode === "localization" && html`<div class="hint">Saving is for a map being made: '${slam.map}' is already saved.</div>`}
      <h3>Load a saved map</h3>
      ${maps.length ? html`<div class="row">
          <select class="grow" value=${sel} onChange=${(/** @type {any} */ e) => setPick(e.currentTarget.value)}>
            ${maps.map((/** @type {string} */ m) => html`<option value=${m}>${m}</option>`)}</select>
          <button class="btn" disabled=${!slam.managed || slam.busy} onClick=${() => act("slam_load", { name: sel })}>Load</button>
          <button class="btn" disabled=${!slam.managed || slam.busy || st?.pose?.frame !== "map"} title="Keep the robot's current position (the map you are on now, just saved)"
            onClick=${() => act("slam_load", { name: sel, here: true })}>Load here</button></div>`
        : html`<p class="dim">No saved maps yet.</p>`}
      <h3>Robot position</h3>
      <button class=${placing ? "btn primary big" : "btn big"} disabled=${slam.mode !== "localization"}
        onClick=${() => setPlacing(!placing)}>${placing ? "Click where the robot is..." : "Set position"}</button>
    </div>
    <div class="main">
      <${MapView} goalMode=${placing} pendingColor="blue"
        onGoal=${(/** @type {any} */ g) => { setPlacing(false); act("initial_pose", g); }}
        onClear=${() => setPlacing(false)}
        hint=${placing ? "Click where the robot really is; drag to show which way it faces" : state} />
    </div>
  </div>`;
}
