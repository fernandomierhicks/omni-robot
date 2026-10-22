// @ts-check
import { html } from "../../vendor/htm-preact-standalone.mjs";
import { MapView } from "../mapview.js";
import { act, useStore } from "../ros.js";
import { Banner, HelpBox, SettingsForm } from "../ui.js";

/** A value of the last throw's record: numbers to 3 decimals, lists of numbers too. @param {any} v @returns {string} */
const num = (v) => (typeof v === "number" ? String(Math.round(v * 1000) / 1000)
  : Array.isArray(v) ? `[${v.map(num).join(", ")}]` : JSON.stringify(v));

export function CatchTab() {
  const s = useStore("status");
  const st = s.status;
  const c = st?.catch;
  const can = !!st?.caps?.catch;
  const playing = st?.mission === "CATCH";
  return html`<div class="cols">
    <div class="side">
      <${HelpBox} tab="catch">
        <ol>
          <li>Stand facing the robot about 6 to 7 m away (in the sim the thrower distance is the Launcher's
            'thrower' box; the robot refines where you are from each throw).</li>
          <li>Press <b>Start</b>. The robot arms and waits ("Waiting for a throw").</li>
          <li>Throw the foam ball toward it. It moves under the ball, catches it in the funnel and, with <b>Throw
            back</b> on, throws it back to you.</li>
        </ol>
        <b>Pause</b> disarms the motors (mode IDLE). <b>Kid profile</b> slows everything and lobs the returns.
        The map shows the robot, where it's heading (blue arrow) and the ball while it's tracked (yellow).
      <//>
      ${!can && html`<${Banner} title="The catch code isn't running.">Start 'Play catch' in the Launcher (in the sim); on the robot it starts with it.<//>`}
      ${playing
        ? html`<button class="btn big danger" onClick=${() => act("mission", { name: "IDLE" })}>Pause</button>`
        : html`<button class="btn go big" disabled=${!can} onClick=${() => act("mission", { name: "CATCH" })}>Start</button>`}
      <h3>Game</h3>
      <div class="card">
        <div style="font-size: 16px; margin-bottom: 8px"><b>${playing ? (c ? c.text : "Starting...") : "Not playing"}</b></div>
        <div class="stats">
          <div class="stat"><div class="v">${c ? c.tosses : "–"}</div><div class="k">throws</div></div>
          <div class="stat"><div class="v">${c ? c.catches : "–"}</div><div class="k">catches</div></div>
          <div class="stat"><div class="v">${c && c.rate != null ? `${c.rate}%` : "–"}</div><div class="k">rate</div></div>
        </div>
        ${c && Object.keys(c.last || {}).length > 0 && html`<h3>Last throw</h3><div class="kv">
          ${Object.entries(c.last).slice(0, 10).map(([k, v]) => html`<span class="k">${k}</span>
            <span class="mono">${num(v)}</span>`)}</div>`}
      </div>
      <${SettingsForm} tab="catch" />
    </div>
    <div class="main">
      <${MapView} catchView=${true} hint="The catch field (the robot's odometry frame: it starts at 0, 0 facing you)"
        empty=${can ? "Waiting for the robot's position..." : "Start 'Play catch' to see the field."} />
    </div>
  </div>`;
}
