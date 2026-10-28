// @ts-check
import { html, useState } from "../../vendor/htm-preact-standalone.mjs";
import { Pads, input, saveStickMap, setPadUsed, stickMap, useInputTick } from "../drive.js";
import { MapView } from "../mapview.js";
import { SayBox } from "../say.js";
import { act, useStore } from "../ros.js";
import { Banner, HelpBox, SettingsForm } from "../ui.js";

export function ManualTab() {
  const s = useStore("status", "schema");
  const st = s.status;
  const driving = st?.mission === "MANUAL";
  useInputTick();                      // the drive loop runs in the App; this redraws the input bars
  const inp = input;
  const rc = st?.mcu?.mode === "MANUAL";
  const cmd = inp.last;
  return html`<div class="cols">
    <div class="side">
      <${HelpBox} tab="manual">
        <ol>
          <li>Press <b>Take control</b> (the mode becomes MANUAL).</li>
          <li><b>Gamepad</b>: hold <kbd>LB</kbd> or <kbd>RB</kbd> (dead man), left stick moves, right stick turns, <kbd>RT</kbd> boosts.
            <b>Flight stick</b>: hold the thumb button, push to move, twist to turn, the trigger boosts.
            Press any button once so the browser sees the controller. Every connected one is listed under Input;
            whichever holds its dead man drives. The gamepad or flight stick works on <b>every tab</b>:
            pressing its dead man takes control from whatever the robot was doing.</li>
          <li><b>Keyboard</b>: hold <kbd>W</kbd><kbd>A</kbd><kbd>S</kbd><kbd>D</kbd> to move (A / D slide sideways), <kbd>Q</kbd>
            / <kbd>E</kbd> or <kbd>←</kbd> / <kbd>→</kbd> to turn, <kbd>Shift</kbd> boosts.</li>
          <li><b>Touch or mouse</b>: drag the round pad on the map to move, the long one to turn.</li>
        </ol>
        <b>Say</b>: press <kbd>Enter</kbd> to type a line for the robot to say (the gamepad still drives), Enter
        again sends it, <kbd>Esc</kbd> goes back to the keyboard's driving. More in the Face tab.
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
          ${!inp.gamepads.length && "No gamepad or flight stick seen (press a button on it with this page in front)."}
          ${st?.manual?.active && st.manual.cmd && html`<br />Robot sends ${st.manual.cmd[0]} m/s, ${st.manual.cmd[1]} m/s, ${st.manual.cmd[2]} rad/s`}
        </div>
        ${inp.gamepads.map((g) => html`<${PadRow} key=${`${g.index}:${g.id}`} g=${g} />`)}
      </div>
      <h3>Say</h3>
      <${SayBox} hotkey compact />
      <${SettingsForm} tab="manual" />
    </div>
    <div class="main">
      <${MapView} zones=${s.schema?.zones} edges=${s.schema?.edges}
        hint=${driving ? (st.slow ? `Slow zone: ${st.slow.edge}, ${st.slow.speed} m/s` : "Driving: the map follows the robot") : "Press 'Take control' to drive"}>
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

/** One connected controller: what it is, whether it drives (virtual ones such as the SpaceMouse's are off by default),
 *  and a flight stick's mapping.
 *  @param {{g: {id: string, standard: boolean, use: boolean, driving: boolean, axes: number[], buttons: number[]}}} p */
function PadRow({ g }) {
  const [, bump] = useState(0);
  const kind = g.standard ? "gamepad (LB / RB)" : "flight stick";
  return html`<div style="font-size: 12px; margin-top: 6px">
    <label title="Off: this controller never drives the robot">
      <input type="checkbox" checked=${g.use} onChange=${(/** @type {any} */ e) => { setPadUsed(g.id, e.currentTarget.checked); bump((n) => n + 1); }} />
      ${" "}<b>${g.id}</b></label>
    <span class="dim"> ${g.use ? kind : "not used"}</span>${g.driving && html` <b class="ok">driving</b>`}
    ${g.use && !g.standard && html`<${StickMapper} g=${g} />`}
  </div>`;
}

/** A controller without the standard mapping (a flight stick): pick its axes and buttons, saved in this browser.
 *  @param {{g: {id: string, axes: number[], buttons: number[]}}} p */
function StickMapper({ g }) {
  const [m, setM] = useState(stickMap(g.id, g.axes.length));
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
