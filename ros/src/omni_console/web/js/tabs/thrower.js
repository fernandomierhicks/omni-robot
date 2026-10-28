// @ts-check
import { html, useEffect, useState } from "../../vendor/htm-preact-standalone.mjs";
import { act, useStore } from "../ros.js";
import { Banner, HelpBox, SettingsForm, deg } from "../ui.js";

const PRESETS = [[0, "off"], [2600, "kid lob"], [4500, "indoor return"]];

export function ThrowerTab() {
  const s = useStore("status");
  const st = s.status;
  const m = st?.mcu;
  const on = st?.mission === "THROWER";
  const t = st?.thrower;
  const [rpm, setRpm] = useState(0);
  const [tilt, setTilt] = useState(0);
  useEffect(() => { if (t) { setRpm(t.rpm); setTilt(t.tilt); } }, [on, t?.rpm, t?.tilt]);   // the robot's values
  /** @param {number} v */
  const sendRpm = (v) => { setRpm(v); act("thrower", { rpm: v }, true); };
  /** @param {number} v */
  const sendTilt = (v) => { setTilt(v); act("thrower", { tilt: v }, true); };
  const target = m?.fly_target_rpm ?? 0;
  const tiltMax = t?.fire_tilt_max ?? 5;            // the flattest launch fired: 25° (the face, design.md 12.2)
  const tooFlat = tilt > tiltMax;
  const fire = () => {
    if (confirm("Fire one ball? Keep the path in front of the launcher clear.")) act("fire");
  };
  return html`<div class="cols">
    <div class="side">
      <${HelpBox} tab="thrower">
        Bench test of the catch module with the base parked.
        <ol>
          <li>Press <b>Take control</b> (mode THROWER: the base holds still).</li>
          <li>Set the <b>flywheel speed</b>. The two flywheels spin up; <b>ready</b> lights when both are within 3%.</li>
          <li>Set the <b>tilt</b> (−30° back to +40° forward). The ball leaves at 30° − tilt.</li>
          <li><b>Fire</b> feeds one ball, if one is held and the flywheels are ready (the firmware checks; refusals are counted).
            It fires only at a launch of 25° or steeper (tilt +${tiltMax}° or less): flatter shots hit the face between the cameras.</li>
        </ol>
        Keep hands clear: up to 6,000 rpm behind the guard. Leaving the tab doesn't stop the flywheels; STOP or another mode does.
        The kid profile caps the flywheel speed in the firmware.
      <//>
      ${m?.mode === "MANUAL" && html`<${Banner} title="RC has control.">The transmitter's switch is on MANUAL.<//>`}
      ${on ? html`<div class="card"><b class="ok">Bench mode: the base is parked.</b></div>`
        : html`<button class="btn go big" disabled=${!st} onClick=${() => act("mission", { name: "THROWER" })}>Take control</button>`}
      <h3>Flywheels</h3>
      <div class="setting">
        <div class="top"><label>Speed</label>
          <input type="number" min="0" max="6000" step="100" value=${rpm} disabled=${!on}
            onChange=${(/** @type {any} */ e) => sendRpm(Math.max(0, Math.min(6000, +e.currentTarget.value || 0)))} /><span class="unit">rpm</span></div>
        <input type="range" min="0" max="6000" step="100" value=${rpm} disabled=${!on}
          onInput=${(/** @type {any} */ e) => setRpm(+e.currentTarget.value)} onChange=${(/** @type {any} */ e) => sendRpm(+e.currentTarget.value)} />
        <div class="row" style="margin-top: 4px">${PRESETS.map(([v, l]) => html`<button class="btn small" disabled=${!on}
          onClick=${() => sendRpm(+v)}>${l} (${v})</button>`)}</div>
      </div>
      <h3>Tilt</h3>
      <div class="setting">
        <div class="top"><label>Angle</label>
          <input type="number" min="-30" max="40" step="1" value=${tilt} disabled=${!on}
            onChange=${(/** @type {any} */ e) => sendTilt(Math.max(-30, Math.min(40, +e.currentTarget.value || 0)))} /><span class="unit">deg</span></div>
        <input type="range" min="-30" max="40" step="1" value=${tilt} disabled=${!on}
          onInput=${(/** @type {any} */ e) => setTilt(+e.currentTarget.value)} onChange=${(/** @type {any} */ e) => sendTilt(+e.currentTarget.value)} />
        <div class="hint">+ tips the funnel forward (toward the thrower), − back. Launch
          <b class=${tooFlat ? "warn" : ""}>${30 - tilt}°</b>${tooFlat ? html` <span class="warn">: too flat to fire (25° at least, tilt +${tiltMax}° or less)</span>` : ""}.</div>
      </div>
      <button class="btn big" style="margin-top: 12px; background: var(--red); border-color: var(--red); color: var(--white); font-weight: 800"
        disabled=${!on || tooFlat} onClick=${fire}
        title=${tooFlat ? "Launch flatter than 25°: the ball would hit the face" : m?.fly_ready ? "" : "The flywheels aren't at speed: the firmware will refuse"}>FIRE ONE BALL</button>
      <${SettingsForm} tab="thrower" />
    </div>
    <div class="main full">
      ${m ? html`<div class="row" style="align-items: stretch; gap: 10px">
        <div class="card grow">
          <h2>Flywheels <span class=${m.fly_ready ? "badge green" : "badge"} style="margin-left: 6px">${m.fly_ready ? "ready" : "not ready"}</span></h2>
          ${[0, 1].map((k) => html`<div class="axis" style="grid-template-columns: 60px 1fr 120px">
            <span class="dim">${k ? "right" : "left"}</span>
            <div class="track"><div style=${`left: 0; width: ${Math.min(100, (m.fly_rpm[k] / 6000) * 100)}%`}></div></div>
            <span class="mono">${m.fly_rpm[k]} / ${Math.round(target)} rpm</span></div>`)}
        </div>
        <div class="card grow">
          <h2>Module</h2>
          <div class="kv">
            <span class="k">tilt</span><span class="mono">${deg(m.tilt)}° (asked ${deg(m.tilt_cmd ?? 0)}°)</span>
            <span class="k">ball held</span><span class=${m.ball_held ? "ok" : "dim"}>${m.ball_held ? "yes (catch beam)" : "no"}</span>
            <span class="k">feed</span><span>${m.feed_state}${m.feed_exit_beam ? ", ball at the exit" : ""}</span>
            <span class="k">fires done</span><span class="mono">${m.fire_seq_done}</span>
            <span class="k">refused</span><span class=${m.fire_rejected ? "warn mono" : "mono"} title="Fire requests the interlocks turned down (flywheels not ready, bumper, a launch flatter than 25°)">${m.fire_rejected}</span>
          </div>
        </div></div>` : html`<p class="dim">No MCU state yet.</p>`}
    </div>
  </div>`;
}
