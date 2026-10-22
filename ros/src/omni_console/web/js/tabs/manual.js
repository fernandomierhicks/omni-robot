// @ts-check
import { html, useState } from "../../vendor/htm-preact-standalone.mjs";
import { Pads, saveStickMap, stickMap, useDriveLoop } from "../drive.js";
import { MapView } from "../mapview.js";
import { act, useStore } from "../ros.js";
import { Banner, HelpBox, SettingsForm } from "../ui.js";

export function ManualTab() {
  const s = useStore("status");
  const st = s.status;
  const driving = st?.mission === "MANUAL";
  const inp = useDriveLoop(driving);
  const rc = st?.mcu?.mode === "MANUAL";
  const cmd = inp.last;
  return html`<div class="cols">
    <div class="side">
      <${HelpBox} tab="manual">
        <ol>
          <li>Press <b>Take control</b> (the mode becomes MANUAL).</li>
          <li><b>Gamepad</b>: hold <kbd>LB</kbd> or <kbd>RB</kbd> (dead man), left stick moves, right stick turns, <kbd>RT</kbd>
            boosts. Press any gamepad button once so the browser sees it.</li>
          <li><b>Keyboard</b>: hold <kbd>W</kbd><kbd>A</kbd><kbd>S</kbd><kbd>D</kbd> to move (A / D slide sideways), <kbd>Q</kbd>
            / <kbd>E</kbd> or <kbd>←</kbd> / <kbd>→</kbd> to turn, <kbd>Shift</kbd> boosts.</li>
          <li><b>Touch or mouse</b>: drag the round pad on the map to move, the long one to turn.</li>
        </ol>
        Let go and the robot stops within 0.3 s. <kbd>Space</kbd> or STOP disarms it. The RC transmitter's switch
        overrides the console. The speeds below are this tab's; the firmware caps them anyway (2 m/s, kid 0.8 m/s).
      <//>
      ${rc && html`<${Banner} title="RC has control.">The transmitter's switch is on MANUAL: the console's commands are ignored until it's switched back.<//>`}
      ${driving
        ? html`<div class="card"><b class="ok">You're driving.</b> <span class="dim">${cmd ? `Input: ${cmd.source}` : "Hold a dead man to move."}</span></div>`
        : html`<button class="btn go big" onClick=${() => act("mission", { name: "MANUAL" })} disabled=${!st}>Take control</button>`}
      <h3>Input</h3>
      <div class="card">
        ${axis("fwd", cmd?.vx)}${axis("left", cmd?.vy)}${axis("turn", cmd?.wz)}
        <div class="dim" style="font-size: 12px; margin-top: 4px">
          ${inp.gamepad ? `Gamepad: ${inp.gamepad.id}` : "No gamepad seen (press a button on it)."}
          ${st?.manual?.active && st.manual.cmd && html`<br />Robot sends ${st.manual.cmd[0]} m/s, ${st.manual.cmd[1]} m/s, ${st.manual.cmd[2]} rad/s`}
        </div>
        ${inp.gamepad && !inp.gamepad.standard && html`<${StickMapper} g=${inp.gamepad} />`}
      </div>
      <${SettingsForm} tab="manual" />
    </div>
    <div class="main">
      <${MapView} hint=${driving ? "Driving: the map follows the robot" : "Press 'Take control' to drive"}>
        ${driving && html`<${Pads} />`}
      <//>
    </div>
  </div>`;
}

/** @param {string} k @param {number | undefined} v */
function axis(k, v) {
  const x = v ?? 0;
  const left = x < 0 ? 50 + x * 50 : 50, width = Math.abs(x) * 50;
  return html`<div class="axis"><span class="dim">${k}</span>
    <div class="track"><div style=${`left: ${left}%; width: ${width}%`}></div></div>
    <span class="mono">${x.toFixed(2)}</span></div>`;
}

/** A controller without the standard mapping (a flight stick): pick its axes and buttons, saved in this browser.
 *  @param {{g: {id: string, axes: number[], buttons: number[]}}} p */
function StickMapper({ g }) {
  const [m, setM] = useState(stickMap(g.id));
  /** @param {string} k @param {string} v */
  const set = (k, v) => { const n = { ...m, [k]: parseInt(v, 10) }; setM(n); saveStickMap(g.id, n); };
  const sel = (/** @type {string} */ k, /** @type {number} */ n) => html`<select value=${m[k]}
    onChange=${(/** @type {any} */ e) => set(k, e.currentTarget.value)}>${[...Array(n).keys()].map((i) => html`<option value=${i}>${i}</option>`)}</select>`;
  return html`<details style="margin-top: 8px"><summary class="dim">Flight stick mapping</summary>
    <p class="dim" style="font-size: 12px">Move each control and watch which number changes, then pick it.</p>
    <div class="kv">
      <span class="k">Sideways axis</span>${sel("x", g.axes.length)}
      <span class="k">Fore-aft axis</span>${sel("y", g.axes.length)}
      <span class="k">Twist axis</span>${sel("twist", g.axes.length)}
      <span class="k">Dead-man button</span>${sel("deadman", g.buttons.length)}
      <span class="k">Boost button</span>${sel("boost", g.buttons.length)}
    </div>
    <div class="mono dim" style="font-size: 11px; margin-top: 6px">axes ${g.axes.map((a, i) => `${i}:${a.toFixed(2)}`).join(" ")}<br />
      buttons ${g.buttons.map((b, i) => (b ? i : null)).filter((x) => x !== null).join(" ") || "none"} pressed</div>
  </details>`;
}
