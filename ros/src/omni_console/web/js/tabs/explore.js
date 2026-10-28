// @ts-check
import { html, useState } from "../../vendor/htm-preact-standalone.mjs";
import { MapView } from "../mapview.js";
import { act, useStore } from "../ros.js";
import { Banner, DrivingPicker, HelpBox, SettingsForm } from "../ui.js";

export function ExploreTab() {
  const s = useStore("status");
  const st = s.status;
  const ex = st?.explore;
  const slam = st?.slam || {};
  const [name, setName] = useState("");
  const nav = !!st?.caps?.nav;
  const exploring = st?.mission === "EXPLORE";
  const marks = [];
  if (ex?.start) marks.push({ x: ex.start[0], y: ex.start[1], color: "green", label: "start" });
  for (const f of ex?.failed || []) marks.push({ x: f[0], y: f[1], color: "red", label: "skipped" });
  const phase = { wait: "Waiting for the new map to start", explore: "Driving to the next opening",
    home: "Done: driving back to the start" }[ex?.phase] || "";
  return html`<div class="cols">
    <div class="side">
      <${HelpBox} tab="explore">
        <ol>
          <li>Put the robot somewhere open, then press <b>New map + explore</b>. It starts a fresh map and drives
            by itself to the edges of what it has seen ("openings") until none are left, the time limit passes or
            it gets farther than <b>How far</b> from the start.</li>
          <li>Watch the map grow. Grey is unknown, dark blue is free floor, white is walls. Openings it couldn't
            reach are marked "skipped".</li>
          <li>When it's done (it drives back to the start), give it a name and press <b>Save map</b> (below; it can be saved while exploring too) so Go To and Patrol
            can use it later.</li>
        </ol>
        <b>Explore this map</b> continues a map that is already being made. <kbd>Space</kbd> stops everything.
        Close doors to rooms it shouldn't enter; it can't see glass or very low obstacles.
      <//>
      ${!nav && html`<${Banner} title="Navigation isn't running.">Start 'Robot (all modes)' in the Launcher.<//>`}
      ${nav && !slam.managed && slam.mode !== "mapping" && html`<${Banner} title="No map is being made.">'Robot (all modes)' lets the console start new maps; the 'Map the house' step maps too.<//>`}
      ${exploring
        ? html`<div class="card">
            <div class="row"><b class="grow">${st.task}</b>
              <button class="btn danger small" onClick=${() => act("cancel")}>Stop</button></div>
            <div class="dim" style="margin-top: 4px">${phase}</div></div>`
        : html`<div class="row" style="gap: 8px; margin-bottom: 10px">
            <button class="btn go big grow" style="width: auto" disabled=${!nav || !slam.managed || slam.busy}
              onClick=${() => { if (confirm("Start a new map and explore? The current map is lost unless you saved it.")) act("explore", { new_map: true }); }}>
              New map + explore</button>
            <button class="btn big" style="width: auto" disabled=${!nav || slam.mode !== "mapping"}
              onClick=${() => act("explore")} title="Keep adding to the map being made now">Explore this map</button></div>`}
      <${DrivingPicker} noSpin=${true} />
      <h3>Save map</h3>
      <div class="row"><input type="text" class="grow" placeholder="Map name, e.g. house" value=${name}
        onInput=${(/** @type {any} */ e) => setName(e.currentTarget.value)} />
        <button class="btn primary" disabled=${!name.trim() || slam.mode !== "mapping"}
          title="Keep the map made so far (also while it is still exploring), with its places and routes"
          onClick=${() => act("slam_save", { name: name.trim() }).then((r) => { if (r.ok) setName(""); })}>Save map</button></div>
      ${slam.mode !== "mapping" && html`<div class="hint">Saving is for a map being made (New map + explore).</div>`}
      <h3>Progress</h3>
      <div class="stats">
        <div class="stat"><div class="v">${ex?.info?.known_m2 ?? "–"}</div><div class="k">m² mapped</div></div>
        <div class="stat"><div class="v">${ex?.info?.frontiers ?? "–"}</div><div class="k">openings left</div></div>
        <div class="stat"><div class="v">${ex ? ex.failed.length : "–"}</div><div class="k">skipped</div></div>
      </div>
      <p class="dim" style="font-size: 12px">SLAM: ${slam.mode ? `${slam.mode}${slam.map ? ` on '${slam.map}'` : ""}` : slam.busy ? "starting..." : "not running"}</p>
      <${SettingsForm} tab="explore" />
    </div>
    <div class="main">
      <${MapView} marks=${marks}
        hint=${exploring ? "Exploring: the map follows the robot (Fit shows all of it)" : "The map being made appears here"}
        empty=${nav ? "Waiting for the robot's position..." : "Start 'Robot (all modes)' to explore."} />
    </div>
  </div>`;
}
