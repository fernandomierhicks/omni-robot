// @ts-check
/* Fault injection, the sim only (simulation/v11 sil/faults.py through the bridge's /sim/fault): break the robot on
   purpose and watch the console and the firmware cope (the status bar's badges, the messages, the robot stopping). */
import { html, useState } from "../../vendor/htm-preact-standalone.mjs";
import { note, publishJson, useStore, useTopic } from "../ros.js";

const TIMED = ["link", "motor", "imu", "flywheel"];        // cut-offs that end by themselves after a few seconds

export function FaultsCard() {
  const s = useStore("conn");
  const [st, setSt] = useState(/** @type {any} */ (null));
  const [motor, setMotor] = useState(1);
  useTopic("/sim/faults", setSt);
  if (s.conn !== "open") return null;
  if (!st) {
    return html`<div class="card"><h2>Fault injection (sim)</h2><p class="dim">Needs a sim step with the firmware in the loop.</p></div>`;
  }
  const active = Object.fromEntries((st.active || []).map((/** @type {any} */ a) => [a.kind, a]));
  /** @param {string} kind @param {boolean} on @param {number} [secs] */
  const send = (kind, on, secs) => {
    publishJson("/sim/fault", { kind, on, s: secs ?? null, motor: motor - 1 });
    note("warn", `Fault injection: ${kind} ${on ? (secs ? `for ${secs} s` : "on") : "off"}`);
  };
  return html`<div class="card">
    <h2>Fault injection (sim)</h2>
    <p class="dim" style="margin: 0 0 8px">Breaks the simulated robot on purpose: the firmware should stop the robot
      safely and the console should say what happened (status bar, messages). Drive it first (Manual) to see the stop.</p>
    <table class="t"><tbody>${Object.entries(st.kinds || {}).map(([k, text]) => {
      const a = active[k];
      return html`<tr><td><b>${k}</b></td><td class="dim">${text}${k === "motor" ? html` <select value=${motor}
          onChange=${(/** @type {any} */ e) => setMotor(+e.currentTarget.value)}>${[1, 2, 3, 4].map((m) => html`<option value=${m}>wheel ${m}</option>`)}</select>` : ""}</td>
        <td>${a ? html`<span class="badge red">${a.until != null ? `${a.until} s` : "on"}</span>` : ""}</td>
        <td style="white-space: nowrap">${TIMED.includes(k) && html`<button class="btn small" onClick=${() => send(k, true, 3)}>3 s</button>`}
          <button class="btn small danger" onClick=${() => send(k, true)}>On</button>
          <button class="btn small" disabled=${!a} onClick=${() => send(k, false)}>Off</button></td></tr>`;
    })}</tbody></table>
  </div>`;
}
