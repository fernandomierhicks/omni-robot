// @ts-check
import { html, useEffect, useState } from "../../vendor/htm-preact-standalone.mjs";
import { MapView } from "../mapview.js";
import { act, useStore } from "../ros.js";
import { Banner, DrivingPicker, HelpBox, SettingsForm, deg } from "../ui.js";

export function GotoTab() {
  const s = useStore("status", "schema");
  const st = s.status;
  const places = s.schema?.places || {};
  const [pending, setPending] = useState(/** @type {{x: number, y: number, yaw: number, name?: string} | null} */ (null));
  const [name, setName] = useState("");
  const nav = !!st?.caps?.nav;
  const going = st?.mission === "GOTO";
  const go = () => { if (pending) act("goto", pending).then((r) => { if (r.ok) setPending(null); }); };
  useEffect(() => {
    /** @param {KeyboardEvent} e */
    const k = (e) => {
      const t = /** @type {HTMLElement} */ (e.target);
      if (t && t.tagName === "INPUT") return;
      if (e.key === "Enter") go();
      if (e.key === "Escape") setPending(null);
    };
    window.addEventListener("keydown", k);
    return () => window.removeEventListener("keydown", k);
  });
  return html`<div class="cols">
    <div class="side">
      <${HelpBox} tab="goto">
        <ol>
          <li><b>Click the map</b> where the robot should go. To choose which way it faces when it gets there, <b>drag</b> before
            letting go (the arrow follows). On a tablet: touch and drag.</li>
          <li>Press <b>Go</b> (or <kbd>Enter</kbd>). The robot plans around obstacles and drives there (Nav2).</li>
          <li><b>Cancel</b> stops it where it is; <kbd>Space</kbd> stops and disarms.</li>
        </ol>
        Right-click or <kbd>Esc</kbd> clears the orange goal. Saved places: give a goal a name and press Save; then
        one click on a place sends the robot there. Pan: right-drag or two fingers; zoom: wheel, pinch or + / −.
        <b>Driving</b> picks which way the robot faces on the way (forward, agnostic or spinning); a change applies
        on the way too.
      <//>
      ${!nav && html`<${Banner} title="Navigation isn't running.">Start 'Navigate (Nav2)' in the Launcher (in the sim); on the robot it starts with it.<//>`}
      <${DrivingPicker} />
      ${going && st.goal && html`<div class="card">
        <div class="row"><b class="grow">${st.task}</b><button class="btn danger small" onClick=${() => act("cancel")}>Cancel</button></div>
        <div class="bar" style="margin-top: 6px"><div style=${`width: ${Math.round((st.progress || 0) * 100)}%`}></div></div></div>`}
      ${pending ? html`<div class="card">
          <div class="row"><b class="grow">New goal</b><button class="btn small" onClick=${() => setPending(null)}>Clear</button></div>
          <div class="mono dim" style="margin: 4px 0 8px">x ${pending.x.toFixed(2)} m, y ${pending.y.toFixed(2)} m, facing ${deg(pending.yaw)}°</div>
          <button class="btn go big" onClick=${go} disabled=${!nav}>Go</button>
          <div class="row" style="margin-top: 8px">
            <input type="text" class="grow" placeholder="Name, e.g. Kitchen" value=${name}
              onInput=${(/** @type {any} */ e) => setName(e.currentTarget.value)} />
            <button class="btn small" disabled=${!name.trim()}
              onClick=${() => act("place_save", { name: name.trim(), ...pending }).then((r) => { if (r.ok) setName(""); })}>Save place</button>
          </div></div>`
        : !going && html`<div class="card dim">Click the map to choose a goal.</div>`}
      <h3>Places</h3>
      ${Object.keys(places).length ? html`<ul class="list">${Object.entries(places).map(([n, p]) => html`<li>
          <span class="grow">${n}</span>
          <button class="btn small" disabled=${!nav} title="Go there now"
            onClick=${() => act("goto", { x: p[0], y: p[1], yaw: p[2], name: n })}>Go</button>
          <button class="btn small" title="Show it as the new goal" onClick=${() => setPending({ x: p[0], y: p[1], yaw: p[2], name: n })}>Show</button>
          <button class="btn small danger" title="Delete this place" onClick=${() => { if (confirm(`Delete '${n}'?`)) act("place_delete", { name: n }); }}>✕</button>
        </li>`)}</ul>`
        : html`<p class="dim">No saved places on this map yet.</p>`}
      <${SettingsForm} tab="goto" />
    </div>
    <div class="main">
      <${MapView} goalMode=${true} pending=${pending} places=${places} zones=${s.schema?.zones} edges=${s.schema?.edges}
        onGoal=${(/** @type {any} */ g) => setPending(g)} onClear=${() => setPending(null)}
        hint=${pending ? "Go (Enter) sends it. Click again to move it; right-click clears." : "Click where to go; drag to set the heading."}
        empty=${nav ? "Waiting for the robot's position on the map..." : "No map: start 'Navigate (Nav2)' or 'Localize on map'."} />
    </div>
  </div>`;
}
