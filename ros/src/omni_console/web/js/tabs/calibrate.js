// @ts-check
/* Calibrate: the bench list (design.md 7) as guided procedures with a tape measure. Results are saved on the robot
   with their time and say what to change; nothing is applied by itself (omni_console/calibration.py). */
import { html, useEffect, useState } from "../../vendor/htm-preact-standalone.mjs";
import { act, command, useStore } from "../ros.js";
import { HelpBox, SettingsForm, clock } from "../ui.js";

export function CalibrateTab() {
  const s = useStore("status");
  const st = s.status;
  const [cal, setCal] = useState(/** @type {any} */ (null));
  const refresh = () => command("calib").then((r) => { if (r.ok) setCal(r.data); });
  useEffect(() => { refresh(); const t = setInterval(refresh, 1500); return () => clearInterval(t); }, []);
  /** @param {string} verb @param {Record<string, any>} [a] */
  const run = async (verb, a = {}) => { const r = await act(verb, a); if (r.ok && r.data) setCal(r.data); return r; };
  const live = st?.mission === "CALIBRATE" ? st.calib : null;
  const res = cal?.results || {};
  return html`<div class="full">
    <${HelpBox} tab="calibrate">
      Each check takes a minute and needs a tape measure (and for the launcher, balls and 6 m of room). Do each a few
      times: when the results agree, put them into the robot's parameters (the firmware's fw_params via
      robot_export.py, the launcher table). The robot drives slowly here (0.25 m/s, 45°/s); <kbd>Space</kbd> stops it.
    <//>
    <div class="calib">
      <${Proc} title="Odometry: straight line" proc="odom_line" res=${res.odom_line} live=${live} run=${run}
        text="Put a tape mark at the front of the robot, with the set distance of clear floor ahead. Start: it drives straight. Then measure how far the front really moved and enter it."
        unit="m" ask="It really moved" />
      <${Proc} title="Heading: spin" proc="odom_spin" res=${res.odom_spin} live=${live} run=${run}
        text="Line a mark on the robot up with a mark on the floor. Start: it spins the set number of turns on its IMU heading. Then enter how many degrees past the floor mark it stopped (+ past, − short)."
        unit="deg" ask="Past the mark by" />
      <${Proc} title="IMU level" proc="imu_level" res=${res.imu_level} live=${live} run=${run}
        text="On the flattest floor you have. It measures roll and pitch at rest, turns half round and measures again: the mean is the IMU's mounting offset, the difference the floor's slope." />
      <${Launcher} cal=${cal} st=${st} run=${run} />
      <${Latency} res=${res.camera_latency} run=${run} />
    </div>
    <div class="card"><${SettingsForm} tab="calibrate" /></div>
  </div>`;
}

/** @param {{title: string, proc: string, res: any[], live: any, run: any, text: string, unit?: string, ask?: string}} p */
function Proc({ title, proc, res, live, run, text, unit, ask }) {
  const [v, setV] = useState("");
  const mine = live && live.proc === proc;
  const last = (res || [])[res?.length - 1];
  return html`<div class="card">
    <h2>${title}</h2>
    <p class="dim" style="margin: 0 0 8px">${text}</p>
    ${mine ? html`<div class="row"><b class="grow">${live.text}</b>
        <button class="btn small danger" onClick=${() => run("stop")}>Stop</button></div>
      ${live.phase === "measure" && unit && html`<div class="row" style="margin-top: 6px">${ask}
        <input type="number" step="0.001" value=${v} onInput=${(/** @type {any} */ e) => setV(e.currentTarget.value)} /> ${unit}
        <button class="btn primary small" disabled=${v === ""} onClick=${() => run("calib_measure", { value: parseFloat(v) }).then(() => setV(""))}>Done</button></div>`}`
      : html`<button class="btn go" onClick=${() => run("calib_start", { proc })}>Start</button>`}
    ${last && html`<div class="result"><span class="dim">${clock(last.t)}:</span> ${last.summary}</div>`}
    ${res && res.length > 1 && html`<details><summary class="dim">${res.length} results</summary>${res.slice().reverse().map((r) => html`<div class="dim" style="font-size: 12px">${clock(r.t)} ${r.summary}</div>`)}</details>`}
  </div>`;
}

/** @param {{cal: any, st: any, run: any}} p */
function Launcher({ cal, st, run }) {
  const [tilt, setTilt] = useState(10);
  const [dist, setDist] = useState(/** @type {Record<string, string>} */ ({}));
  const bench = st?.mission === "THROWER";
  const m = st?.mcu;
  const shots = cal?.shots || [];
  const last = (cal?.results?.launcher || []).slice(-1)[0];
  return html`<div class="card">
    <h2>Launcher: rpm against distance</h2>
    <p class="dim" style="margin: 0 0 8px">Clear floor ahead (6 m or more). Load a ball, set the rpm, fire, and enter
      where it first touched the floor, measured from the robot's centre. Five shots fit how far off the launcher's
      table is.</p>
    ${!bench ? html`<button class="btn go" onClick=${() => run("mission", { name: "THROWER" })}>Take control (Thrower bench mode)</button>`
      : html`<div class="row" style="margin-bottom: 6px">Tilt <input type="number" min="-30" max="40" value=${tilt}
          onChange=${(/** @type {any} */ e) => setTilt(+e.currentTarget.value)} />°
          <span class="dim">flywheels ${m ? `${Math.round(m.fly_rpm[0])} / ${Math.round(m.fly_target_rpm)} rpm${m.fly_ready ? ", ready" : ""}` : "–"}${m?.ball_held ? ", ball loaded" : ""}</span></div>
        <table class="t"><thead><tr><th>rpm</th><th></th><th>landed at (m)</th><th></th></tr></thead><tbody>
          ${(cal?.rpms || []).map((/** @type {number} */ rpm) => html`<tr><td class="mono">${rpm}</td>
            <td><button class="btn small" onClick=${() => run("thrower", { rpm, tilt })}>Spin up</button>
              <button class="btn small danger" disabled=${!m?.fly_ready || Math.abs((m?.fly_target_rpm || 0) - rpm) > 1}
                onClick=${() => { if (confirm("Fire one ball? Keep the path clear.")) run("fire"); }}>Fire</button></td>
            <td><input type="number" step="0.01" style="width: 70px" value=${dist[rpm] || ""}
              onInput=${(/** @type {any} */ e) => setDist({ ...dist, [rpm]: e.currentTarget.value })} /></td>
            <td><button class="btn small primary" disabled=${!dist[rpm]}
              onClick=${() => run("calib_shot", { rpm, tilt_deg: tilt, distance_m: parseFloat(dist[rpm]) })}>Record</button></td></tr>`)}
        </tbody></table>
        <div class="row" style="margin-top: 6px"><span class="grow dim">${shots.length} shots recorded</span>
          <button class="btn small" disabled=${!shots.length} onClick=${() => run("calib_clear", { proc: "shots" })}>Clear</button>
          <button class="btn small primary" disabled=${shots.length < 2} onClick=${() => run("calib_fit")}>Fit</button>
          <button class="btn small" onClick=${() => { run("thrower", { rpm: 0, tilt: 0 }); run("mission", { name: "IDLE" }); }}>Done</button></div>`}
    ${last && html`<div class="result"><span class="dim">${clock(last.t)}:</span> ${last.summary}
      <table class="t" style="margin-top: 4px"><tbody>${last.rows.map((/** @type {any} */ r) => html`<tr class="mono">
        <td>${r.rpm} rpm</td><td>${r.distance_m} m</td><td>${r.exit_speed} m/s</td><td>table ${r.table_rpm}</td><td>×${r.ratio}</td></tr>`)}</tbody></table></div>`}
  </div>`;
}

/** @param {{res: any[], run: any}} p */
function Latency({ res, run }) {
  const [v, setV] = useState("");
  const last = (res || []).slice(-1)[0];
  return html`<div class="card">
    <h2>Camera latency</h2>
    <p class="dim" style="margin: 0 0 8px">The LED and photodiode test (design.md 7): an LED the MCU flashes, in front of
      a camera, and a photodiode on the LED; the delay from the flash to the frame that shows it. Enter the measured
      median.</p>
    <div class="row"><input type="number" step="1" value=${v} onInput=${(/** @type {any} */ e) => setV(e.currentTarget.value)} /> ms
      <button class="btn small primary" disabled=${v === ""} onClick=${() => run("calib_record", { proc: "camera_latency", value: parseFloat(v) }).then(() => setV(""))}>Record</button></div>
    ${last && html`<div class="result"><span class="dim">${clock(last.t)}:</span> ${last.summary} <span class="dim">(the sim assumes 50 ms)</span></div>`}
  </div>`;
}
