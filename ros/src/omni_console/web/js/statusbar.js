// @ts-check
/* The status bar: always there, whatever the tab. Who has control (the whole ladder in its tooltip: e-stop, RC switch,
   stick, task, AI), the mode and current task, how fast it may go and why, localization, the MCU, battery, Wi-Fi,
   preflight, faults, the latest message (click for the list) and STOP (also the space bar). On a phone the second
   line folds away. */
import { html, useEffect, useState } from "../vendor/htm-preact-standalone.mjs";
import { act, update, useStore } from "./ros.js";
import { clock } from "./ui.js";

const QUIET_S = 1.0;                // no status for this long: the robot (or the Wi-Fi) has gone quiet

export function stop() { act("stop", {}, true); }

export function StatusBar() {
  const s = useStore("status", "conn", "events", "rtt", "supervisor");
  const [drawer, setDrawer] = useState(false);
  const [, tick] = useState(0);
  useEffect(() => {
    const t = setInterval(() => tick((n) => n + 1), 250);       // notices when updates *stop*
    /** @param {KeyboardEvent} e */
    const key = (e) => {
      const el = /** @type {HTMLElement} */ (e.target);
      if (e.code !== "Space" || (el && ["INPUT", "TEXTAREA", "SELECT"].includes(el.tagName))) return;
      e.preventDefault();
      stop();
    };
    window.addEventListener("keydown", key);
    return () => { clearInterval(t); window.removeEventListener("keydown", key); };
  }, []);

  const st = s.status;
  const age = (performance.now() - s.tStatus) / 1000;
  let conn;
  const sup = s.supervisor;
  const cur = sup?.current;
  const running = cur && (cur.state === "running" || cur.state === "stopping");
  if (s.conn !== "open" && sup && !sup.unreachable && !running) {
    conn = html`<span class="sb" title="The page server answers, but the robot's software isn't running">
      <span class="dot err"></span>Robot software stopped${cur?.state === "exited" && cur.code ? ` (exit ${cur.code})` : ""}
      <a href="#home">Start</a></span>`;
  } else if (s.conn !== "open" && running) {
    conn = html`<span class="sb" title=${`${cur.label}: waiting for rosbridge ${s.url}`}><span class="dot warn"></span>
      Starting ${cur.label}... <a href="#system">log</a></span>`;
  } else if (s.conn !== "open") {
    conn = html`<span class="sb" title=${`rosbridge ${s.url}`}><span class="dot err"></span>
      ${s.conn === "connecting" ? "Connecting..." : "OFFLINE: reconnecting"}</span>`;
  } else if (!st || age > QUIET_S) {
    conn = html`<span class="sb" title="Connected to rosbridge, but the console node isn't sending (omni_console stopped, or the Wi-Fi is struggling)">
      <span class="dot warn"></span>NO DATA ${st ? `(${age.toFixed(1)} s)` : ""}</span>`;
  } else {
    conn = html`<span class="sb" title=${`rosbridge ${s.url}`}><span class="dot ok"></span>
      ${location.hostname || "robot"} <span class="dim">${s.rtt != null ? `${s.rtt} ms` : ""}</span></span>`;
  }

  const mcu = st?.mcu;
  const last = s.events[s.events.length - 1];
  return html`
    ${drawer && html`<${Drawer} close=${() => setDrawer(false)} />`}
    <div class="statusbar">
      <div class="cells">
        <div class="line">
          ${conn}
          ${control(st)}
          <span class="sb"><span class="k">Mode</span><span class="badge mission">${st ? missionName(st.mission) : "?"}</span></span>
          <span class="sb task" title=${st?.task || ""}>${st?.task || ""}
            ${st?.progress != null && html`<span class="bar"><div style=${`width: ${Math.round(st.progress * 100)}%`}></div></span>`}</span>
          ${mcu && html`<span class="sb" title=${mcu.mode_hint}><span class="k">MCU</span>
            <span class=${`badge ${mcuClass(mcu.mode)}`}>${mcu.mode}</span>
            <span class="dim">${mcu.profile}</span></span>`}
          ${speedCap(st)}
          ${battery(mcu)}
        </div>
        <div class="line more">
          ${localization(st)}
          ${preflight(st)}
          ${wifi(st)}
          ${mcu && mcu.age > 1 && html`<span class="badge red" title="No state from the MCU: the link (USB) is down. The MCU stops the robot by itself after 0.5 s without commands.">MCU SILENT ${mcu.age.toFixed(0)} s</span>`}
          ${(mcu?.faults || []).map((/** @type {any} */ f) => html`<span class="badge red" title=${f.hint}>${f.name}</span>`)}
          ${st?.sys && st.sys.load != null && html`<span class="sb dim" title="The robot computer's load (1 min) and its hottest sensor">
            CPU ${st.sys.load}${st.sys.temp_c != null ? ` · ${st.sys.temp_c} °C` : ""}</span>`}
          ${st && st.consoles > 1 && html`<span class="sb dim" title="Pages open on this robot (PC, iPad...)">${st.consoles} consoles</span>`}
          <span class="sb ticker" onClick=${() => setDrawer(!drawer)} title="All messages">
            ${last ? html`<span class=${levelClass(last.level)}>${last.level === "info" ? "•" : "⚠"}</span>
              ${clock(last.t)} ${last.text}` : "No messages yet"}</span>
        </div>
      </div>
      <button class="stop" onClick=${stop} title="Software stop: the robot ramps to a stop and disarms its motors (mode IDLE). The red button on the robot is the hardware e-stop (it cuts power).">
        STOP<small>space bar</small></button>
    </div>`;
}

/** @param {{close: () => void}} p */
function Drawer({ close }) {
  const s = useStore("events");
  const evs = s.events.slice().reverse();
  return html`<div class="drawer">
    <div class="row"><h2 class="grow">Messages from the robot and this page</h2>
      <button class="btn small" onClick=${() => { update({ events: [] }); close(); }}>Clear</button>
      <button class="btn small" onClick=${close}>Close</button></div>
    ${evs.length ? evs.map((/** @type {any} */ e) => html`<div class="ev">
      <span class="mono dim">${clock(e.t)}</span><span class=${`lvl ${levelClass(e.level)}`}>${e.level}</span>
      <span class="src dim">${e.source}</span><span>${e.text}</span></div>`) : html`<p class="dim">Nothing yet.</p>`}
  </div>`;
}

/** @param {string} m */
export const missionName = (m) => ({ GOTO: "GO TO", THROWER: "THROWER TEST" })[m] || m;
/** @param {string} m */
const mcuClass = (m) => (m === "AUTO" ? "green" : m === "ESTOP" ? "red" : m === "MANUAL" ? "orange" : "");
/** @param {string} l */
const levelClass = (l) => (l === "error" ? "err" : l === "warn" ? "warn" : "ok");

/** @param {any} mcu */
function battery(mcu) {
  if (!mcu) return null;
  const pct = mcu.battery_pct;
  const col = mcu.battery_low || (pct != null && pct < 15) ? "var(--red)" : pct != null && pct < 35 ? "var(--orange)" : "var(--green)";
  return html`<span class="sb" title="Estimated from the pack voltage (6S): reads low while the motors pull hard. The base stops below 21 V.">
    <span class="batt"><div style=${`width: ${pct ?? 0}%; background: ${col}`}></div></span>
    ${mcu.v_bus.toFixed(1)} V <span class="dim">${pct != null ? `${pct}%` : ""}</span></span>`;
}

/** @param {any} st */
function wifi(st) {
  if (!st) return null;
  const w = st.wifi;
  if (!w) return html`<span class="sb dim" title="The robot reports no Wi-Fi interface (in the sim it runs in WSL)">Wi-Fi n/a</span>`;
  const col = w.dbm > -60 ? "ok" : w.dbm > -72 ? "warn" : "err";
  return html`<span class="sb" title=${`${w.iface}, link quality ${w.quality}`}><span class="k">Wi-Fi</span>
    <span class=${col}>${w.dbm} dBm</span></span>`;
}

const WHO_CLS = { estop: "red", rc: "orange", stick: "green", ai: "blue", task: "green", ready: "", idle: "" };

/** Who has the robot, and the override ladder (the highest first) in the tooltip. @param {any} st */
function control(st) {
  const c = st?.control;
  if (!c) return null;
  const ladder = c.ladder.map((/** @type {any} */ x, /** @type {number} */ i) =>
    `${i + 1}. ${x.label}${x.id === c.who ? "  ◀ now" : ""}`).join("\n");
  const tip = `Who drives the robot, highest first (each one beats those below it):\n${ladder}` +
    (c.ai_hold_s ? `\n\nA person took over: the AI waits ${c.ai_hold_s} s more.` : "") +
    (c.ai_lease ? "\nAn AI client holds a lease." : "");
  return html`<span class="sb" title=${tip}><span class="k">Control</span>
    <span class=${`badge ${WHO_CLS[/** @type {keyof typeof WHO_CLS} */ (c.who)] ?? ""}`}>${c.text}</span>
    ${c.ai_hold_s && html`<span class="dim">AI held ${Math.ceil(c.ai_hold_s)} s</span>`}</span>`;
}

/** @param {any} st */
function speedCap(st) {
  const c = st?.cap;
  if (!c) return null;
  return html`<span class="sb" title="The fastest the robot may drive now: the lowest of the firmware's profile, the task's own speed, the arm being out and the map's slow bands">
    <span class="k">Max</span><span class="mono">${c.speed} m/s</span> <span class="dim">${c.reason}</span></span>`;
}

/** @param {any} st */
function localization(st) {
  const l = st?.localization;
  if (!st || !l) return null;
  const col = l.level === "LOST" ? "red" : l.level === "POOR" ? "orange" : "green";
  return html`<span class="sb" title=${(l.reasons || []).join("; ") || "omni_slam's localization health"}><span class="k">Loc</span>
    <span class=${`badge ${col}`}>${l.level}</span>${l.sigma_xy != null && html`<span class="dim">±${Math.round(l.sigma_xy * 100)} cm</span>`}</span>`;
}

/** @param {any} st */
function preflight(st) {
  const p = st?.preflight;
  if (!p) return null;
  const col = p.fail ? "red" : p.warn ? "orange" : "green";
  return html`<a class="sb" href="#preflight" style="text-decoration: none" title=${p.worst?.length ? p.worst.join(", ") : "All checks pass"}>
    <span class="k">Preflight</span><span class=${`badge ${col}`}>${p.fail ? `${p.fail} failing` : p.warn ? `${p.warn} !` : "ok"}</span></a>`;
}
