// @ts-check
import { html } from "../../vendor/htm-preact-standalone.mjs";
import { useStore } from "../ros.js";
import { HelpBox } from "../ui.js";

// what the firmware does with each channel (firmware.cpp: MANUAL mode drives from rc[0], rc[1], rc[3]; rc[4] > 0.5
// takes over)
const CH = ["sideways (right +)", "forward", "ch 3", "turn", "MANUAL switch", "ch 6", "ch 7", "ch 8"];

export function RcTab() {
  const s = useStore("status");
  const st = s.status;
  const rc = st?.rc;
  return html`<div class="full">
    <${HelpBox} tab="rc">
      The radio transmitter talks to an ELRS receiver on the MCU (CRSF). It is the safety override: with
      its <b>MANUAL switch</b> (channel 5) up, the MCU drives from the sticks and ignores the Jetson and this console,
      whatever mode they are in. Switch it back down to hand control back. If the receiver loses the transmitter,
      the MCU ignores the sticks (failsafe). Below: what the MCU receives, 10 times a second.
    <//>
    <div class="card" style="max-width: 640px">
      <h2>Receiver
        ${rc && html`<span class=${rc.ok ? "badge green" : "badge orange"} style="margin-left: 8px">${rc.ok ? "link up" : "no link"}</span>`}
        ${rc?.manual && html`<span class="badge red" style="margin-left: 6px">MANUAL: RC has control</span>`}</h2>
      ${!rc ? html`<p class="dim">No INFO from the MCU: its link is down, or its firmware predates the console (protocol 2).</p>`
        : html`${!rc.ok && html`<p class="dim">The MCU hears no receiver. In the sim there is none; on the robot, check the
            transmitter is on and bound.</p>`}
          ${rc.ch.map((/** @type {number} */ v, /** @type {number} */ i) => {
            const left = v < 0 ? 50 + v * 50 : 50, width = Math.abs(v) * 50;
            return html`<div class="axis" style="grid-template-columns: 150px 1fr 56px">
              <span class=${i === 4 ? "" : "dim"}>${i + 1}: ${CH[i]}</span>
              <div class="track"><div style=${`left: ${left}%; width: ${width}%`}></div></div>
              <span class="mono">${v.toFixed(2)}</span></div>`;
          })}`}
    </div>
  </div>`;
}
