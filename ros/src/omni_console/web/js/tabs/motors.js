// @ts-check
/* The motor test (Test > Motors): drive each wheel motor by itself, wheels off the ground. Mission MOTORS (mode_manager
   WHEELS, the firmware's DRIVE_WHEELS): the stick's forward axis turns the chosen wheels, each at the same speed, up to
   the motor's own limit (the firmware caps the test there and ramps every wheel). The stick is the page's usual one
   (drive.js): a gamepad or flight stick with its dead man held (on any tab while the test runs), W / S on this tab, or
   the bar below. The charts: each chosen wheel's asked and measured speed and its motor's current at the MCU's 200 Hz
   (mcu_link's /mcu/samples, columns c1..c4, w1..w4 and, protocol 9, i1..i4). Profiles (motors_run: steps, a speed
   sweep) play timed speeds on the chosen wheels at the Ramp setting; Save CSV writes the last 60 s of samples. */
import { html, useEffect, useRef, useState } from "../../vendor/htm-preact-standalone.mjs";
import { useGate } from "../features.js";
import { decodeFloat32MultiArray } from "../cdr.js";
import { Chart, makeBuffer, push } from "../chart.js";
import { input, useInputTick } from "../drive.js";
import { act, useRawTopic, useStore } from "../ros.js";
import { Gate, Banner, HelpBox, SettingsForm } from "../ui.js";
import { DM, NO_REPLY } from "./base.js";

const N = 4;
const RPM = 60 / (2 * Math.PI);          // rad/s -> rpm

const RAW_S = 60;                        // s of raw rows kept for Save CSV
const RAW_COLS = ["t", ...[1, 2, 3, 4].flatMap((i) => [`c${i}`, `w${i}`, `i${i}`])];

/** Every wheel's asked and measured speed and its current, 10 s of the MCU's 200 Hz samples (filled whichever
    wheels are shown), and the last RAW_S s as rows for the CSV. */
function useWheelSamples() {
  const bufs = useRef(Array.from({ length: N }, () => makeBuffer(2))).current;
  const cur = useRef(Array.from({ length: N }, () => makeBuffer(1))).current;
  const raw = useRef(/** @type {number[][]} */ ([])).current;
  useRawTopic("/mcu/samples", "std_msgs/msg/Float32MultiArray", (bytes) => {
    const { dims, data } = decodeFloat32MultiArray(bytes);
    const cols = (dims[1]?.label || "").split(",");
    const n = cols.length;
    const k = Object.fromEntries(cols.map((c, i) => [c, i]));
    if (k.c1 === undefined) return;                       // an mcu_link from before the wheel columns
    const hasI = k.i1 !== undefined;                      // protocol 9's currents
    for (let o = 0; o + n <= data.length; o += n) {
      const t = data[o + k.t];
      const row = [t];
      for (let i = 0; i < N; i++) {
        const c = data[o + k[`c${i + 1}`]], w = data[o + k[`w${i + 1}`]], a = hasI ? data[o + k[`i${i + 1}`]] : NaN;
        push(bufs[i], t, [w, c]);
        if (hasI) push(cur[i], t, [a]);
        row.push(c, w, a);
      }
      raw.push(row);
    }
    while (raw.length && raw[raw.length - 1][0] - raw[0][0] > RAW_S) raw.shift();
  });
  return { bufs, cur, raw };
}

/** The last RAW_S s of samples as a CSV download. @param {number[][]} raw */
function saveCsv(raw) {
  const lines = [RAW_COLS.join(","), ...raw.map((r) => r.map((v) => (Number.isFinite(v) ? +v.toFixed(5) : "")).join(","))];
  const a = document.createElement("a");
  a.href = URL.createObjectURL(new Blob([lines.join("\n") + "\n"], { type: "text/csv" }));
  a.download = `motors-${new Date().toISOString().replace(/[:.]/g, "-").slice(0, 19)}.csv`;
  a.click();
  setTimeout(() => URL.revokeObjectURL(a.href), 1000);
}

/** The on-screen bar: drag up / down (the dead man is touching it). */
function Bar() {
  const on = useRef(false);
  const [v, setV] = useState(0);
  useEffect(() => () => { input.pad = null; }, []);          // leaving the tab mid-drag lets go
  /** @param {any} e */
  const handle = (e) => {
    const el = /** @type {HTMLElement} */ (e.currentTarget);
    if (e.type === "pointerdown") { el.setPointerCapture(e.pointerId); on.current = true; }
    if (e.type === "pointerup" || e.type === "pointercancel") on.current = false;
    if (!on.current) { input.pad = null; setV(0); return; }
    const r = el.getBoundingClientRect();
    const half = r.height / 2 - 22;
    const u = Math.max(-1, Math.min(1, -(e.clientY - (r.top + r.height / 2)) / half));
    input.pad = { vx: u, vy: 0, wz: 0, boost: false, source: "screen" };
    setV(u);
  };
  const ev = { onPointerDown: handle, onPointerMove: handle, onPointerUp: handle, onPointerCancel: handle };
  return html`<div class=${on.current ? "wheelbar active" : "wheelbar"} ...${ev}>
    <div class="knob" style=${`top: calc(50% - ${v * 50}% + ${v * 22}px)`}></div>
    <div class="lbl">drag</div></div>`;
}

/** One wheel's speed: commanded (outline) and measured (fill), -max..max. @param {{cmd: number, vel: number, max: number}} p */
function Gauge({ cmd, vel, max }) {
  const pos = (/** @type {number} */ x) => 50 + 50 * Math.max(-1, Math.min(1, x / max));
  const span = (/** @type {number} */ x) => `left: ${Math.min(50, pos(x))}%; width: ${Math.abs(pos(x) - 50)}%`;
  return html`<div class="wheelgauge">
    <div class="vel" style=${span(vel)}></div>
    <div class="cmd" style=${span(cmd)}></div>
    <div class="zero"></div></div>`;
}

export function MotorsTab() {
  useInputTick();
  const { bufs, cur, raw } = useWheelSamples();
  const s = useStore("status");
  const st = s.status;
  const g = useGate("motors");
  const m = st?.mcu;
  const on = st?.mission === "MOTORS";
  const mo = st?.motors;
  const prof = mo?.profile ?? null;
  const sel = /** @type {number[]} */ (mo?.wheels ?? [0]);
  const max = mo?.max_speed ?? 3;
  const scale = Math.max(max, 1);
  const choose = (/** @type {number[]} */ w) => act("motors", { wheels: w }, true);
  const toggle = (/** @type {number} */ i) => {
    const next = sel.includes(i) ? sel.filter((k) => k !== i) : [...sel, i];
    if (next.length) choose(next);
  };
  const cmd = input.last;
  const temps = /** @type {number[] | null} */ (st?.motor_temp ?? null);
  // the motors that aren't answering on CAN (STATE dm_err 255, protocol 8): nothing to test there
  const gone = /** @type {number[]} */ ((m?.dm_err || []).map((/** @type {number} */ e, /** @type {number} */ i) => (e === NO_REPLY ? i : -1)).filter((/** @type {number} */ i) => i >= 0));
  const here = [...Array(N).keys()].filter((i) => !gone.includes(i));
  return html`<div class="cols">
    <div class="side">
      <${HelpBox} tab="motors">
        Drive each wheel motor by itself. <b>Lift the wheels off the ground first</b>: one wheel turning on its own drags
        the robot round.
        <ol>
          <li>Press <b>Take control</b> (mode WHEELS: the base controller rests, each wheel gets its own speed).</li>
          <li>Pick the wheel(s). They all get the same speed.</li>
          <li>Hold a dead man and push forward / back: <b>gamepad</b> LB or RB + left stick, <b>flight stick</b> thumb
            button + stick, <b>W / S</b> on this tab, or drag the bar.</li>
        </ol>
        Full stick is <b>Top wheel speed</b> (below), up to the firmware's cap (37.2 rad/s, 355 rpm on the AK40-10:
        0.95 of its own top speed on 24 V). Every change is a ramp at <b>Ramp</b> (20 rad/s²: rest to full speed in about
        2 s; up to 2000, a step for a bare wheel); letting go ramps the wheels to rest at 20, and so do STOP and any
        other mode. <b>Profiles</b> play timed speeds on the chosen wheels at that ramp and top speed: <i>Steps</i>
        (rest, +, rest, −, + reversal, rest) shows how the motor speeds the wheel up and stops it; <i>Sweep</i> holds
        steady speeds both ways (friction against speed). Holding a dead man takes over. The charts show each chosen
        wheel's speed (<b>orange dashed</b> asked, <b>blue</b> measured) and its motor's current, the last 10 s at the
        MCU's 200 Hz; <b>Save CSV</b> writes the last 60 s.
      <//>
      ${m?.mode === "MANUAL" && html`<${Banner} title="RC has control.">The transmitter's switch is on MANUAL.<//>`}
      ${on ? html`<div class="card"><b class="ok">Motor test.</b> <span class="dim">${mo?.active && cmd
        ? `Driving (${cmd.source}).` : "Hold a dead man to turn the wheels."}</span></div>`
        : html`<${Gate} a=${g} /><button class="btn go big" disabled=${!g.ok} onClick=${() => act("mission", { name: "MOTORS" })}>Take control</button>`}
      <h3>Wheels</h3>
      <div class="seg">${[...Array(N).keys()].map((i) => html`<button class=${sel.includes(i) ? "btn on" : "btn"}
        disabled=${!on || gone.includes(i)} onClick=${() => toggle(i)}
        title=${gone.includes(i) ? `CAN id ${i + 1}: not answering (not connected?)` : `CAN id ${i + 1}`}>${i + 1}</button>`)}
        <button class=${sel.length === here.length ? "btn on" : "btn"} disabled=${!on || !here.length}
          onClick=${() => choose(here)}>${gone.length ? "all here" : "all"}</button></div>
      <div class="hint">Wheel <i>n</i> is the motor with CAN id <i>n</i> (the firmware's order).
        ${gone.length > 0 && html`<br /><span class="warn">Not answering on CAN: wheel ${gone.map((i) => i + 1).join(", ")}.</span>${" "}
          Driving needs all four; this test turns the ones that answer.`}</div>
      <div class="row" style="align-items: center; gap: 14px; margin: 12px 0">
        <${Bar} />
        <div class="dim" style="font-size: 13px">Stick ${cmd ? cmd.vx.toFixed(2) : "0.00"}<br />
          asks ${((cmd?.vx ?? 0) * max).toFixed(2)} rad/s<br />of ${max} rad/s at full stick,
          ramp ${mo?.ramp ?? 20} rad/s²</div>
      </div>
      <h3>Profiles</h3>
      ${prof ? html`<div class="card"><b>${prof.name}</b> <span class="dim">${prof.t.toFixed(1)} of ${prof.total.toFixed(1)} s</span>
          <button class="btn" style="margin-left: 8px" onClick=${() => act("motors_run", { stop: true })}>Stop</button></div>`
        : html`<div class="seg">
          <button class="btn" disabled=${!on} title="Rest, +top, rest, −top, +top (a reversal), rest; 1 s each"
            onClick=${() => act("motors_run", { preset: "steps" })}>Steps</button>
          <button class="btn" disabled=${!on} title="Steady speeds up to the top and back, both ways, 1.5 s each"
            onClick=${() => act("motors_run", { preset: "sweep" })}>Sweep</button>
          <button class="btn" onClick=${() => saveCsv(raw)} title="The last 60 s of the MCU's 200 Hz samples">Save CSV</button>
        </div>`}
      <${SettingsForm} tab="motors" />
    </div>
    <div class="main full">
      ${m ? html`<div class="card">
        <h2>Wheel speeds <span class="dim" style="font-weight: 400; font-size: 13px">outline: asked, fill: measured</span></h2>
        ${[...Array(N).keys()].map((i) => {
          const c = m.wheel_cmd?.[i] ?? 0, v = m.wheel_vel?.[i] ?? 0;
          const e = m.dm_err?.[i];
          const d = /** @type {any} */ (DM)[e];
          return html`<div class="wheelrow" title=${d?.[1] ?? ""} style=${gone.includes(i) ? "opacity: .35" : ""}>
            <b class=${sel.includes(i) && on ? "" : "dim"}>${i + 1}</b>
            <${Gauge} cmd=${c} vel=${v} max=${scale} />
            <span class="mono">${v.toFixed(2)} / ${c.toFixed(2)} rad/s</span>
            <span class="mono dim">${(m.wheel_cur?.[i] ?? 0).toFixed(2)} A</span>
            <span class="mono dim">${(v * RPM).toFixed(0)} rpm</span>
            <span class=${e >= 8 ? "warn" : "dim"}>${d?.[0] ?? `err ${e}`}${temps ? `, ${temps[i]} °C` : ""}</span>
          </div>`;
        })}
        <div class="hint" style="margin-top: 8px">MCU ${m.mode}${m.faults?.length ? ` · ${m.faults.map((/** @type {any} */ f) => f.name ?? f).join(", ")}` : ""}
          · drive mode ${m.drive_mode === 4 ? "wheels" : m.drive_mode}</div>
      </div>` : html`<p class="dim">No MCU state yet.</p>`}
      <div class="charts">${sel.slice().sort().map((i) => html`<div class="card" key=${i}>
        <${Chart} title=${`Wheel ${i + 1}: asked vs measured`} unit="rad/s" span=${2} buf=${bufs[i]} series=${[
          { label: "measured", color: "blue" }, { label: "asked", color: "orange", dash: true }]} />
        <${Chart} title=${`Wheel ${i + 1}: motor current`} unit="A" span=${2} buf=${cur[i]} series=${[
          { label: "current", color: "green" }]} /></div>`)}</div>
    </div>
  </div>`;
}
