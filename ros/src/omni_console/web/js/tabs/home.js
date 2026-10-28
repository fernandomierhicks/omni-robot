// @ts-check
/* Home: what the robot is doing and a card per mode. The first page a person sees, on the iPad too: pick a mode and
   its tab opens (or it starts, for the simple ones). When the robot's software isn't running, the supervisor starts
   it from here. */
import { html } from "../../vendor/htm-preact-standalone.mjs";
import { act, useStore } from "../ros.js";
import { missionName } from "../statusbar.js";
import { runningStep, startStep, stepArgs } from "../system.js";
import { Banner } from "../ui.js";

/** id, title, what it does, its tab, the mission it runs (for the live dot), what it needs */
const CARDS = [
  ["catch", "Play catch", "Throw the ball to the robot: it catches it and throws it back. Pass the ball between two people too.", "catch", "CATCH", "catch"],
  ["manual", "Drive", "Gamepad, keyboard or the pads on the screen.", "manual", "MANUAL", null],
  ["follow", "Follow me / Tag", "The robot follows the nearest person, or chases you at kid speed.", "follow", "FOLLOW", null],
  ["program", "Program it", "Line up moves (forward, turn, spin, wait, go to a place) and press Run.", "program", "PROGRAM", null],
  ["goto", "Go to a place", "Click on the map, or pick a saved place.", "goto", "GOTO", "nav"],
  ["explore", "Explore", "Maps the house by itself, room by room.", "explore", "EXPLORE", "nav"],
  ["patrol", "Patrol", "Drives a saved route, once or in a loop.", "patrol", "PATROL", "nav"],
  ["thrower", "Thrower test", "Flywheels, tilt and fire one ball, with the base parked.", "thrower", "THROWER", null],
];

export function HomeTab() {
  const s = useStore("status", "conn", "supervisor", "schema");
  const st = s.status;
  const sup = s.supervisor;
  const run = runningStep();
  const caps = st?.caps || {};
  const kid = !!st?.kid;
  const stepOf = (/** @type {string} */ id) => sup?.steps?.find((/** @type {any} */ x) => x.id === id);
  const start = (/** @type {string} */ id) => { const step = stepOf(id); if (step) startStep(id, stepArgs(step)); };
  const up = s.conn === "open" && !!st;
  const robotStep = sup && (stepOf("robot") ? "robot" : sup.steps?.[0]?.id);
  return html`<div class="full">
    <div class="home-top">
      <div class="card grow">
        <h2>${up ? `The robot is ${missionName(st.mission).toLowerCase()}` : run ? "Starting the robot's software..." : "The robot's software isn't running"}</h2>
        <div class="dim">${up ? st.task : run ? `${run.label}: ${run.state}, ${run.uptime_s ?? 0} s (System tab: its log)` : sup ? "Start it here." : "Start a step in the Launcher (or omni.sh)."}</div>
        <div class="row" style="margin-top: 10px">
          ${!up && !run && robotStep && html`<button class="btn go" onClick=${() => start(robotStep)}>Start the robot</button>`}
          ${up && st.mission !== "IDLE" && html`<button class="btn danger" onClick=${() => act("stop")}>Stop (motors off)</button>`}
          ${up && html`<label class="row" title="The firmware's kid limits in every mode: 0.8 m/s, gentle accelerations, lobbed returns">
            <span class="switch"><input type="checkbox" checked=${kid}
              onChange=${(/** @type {any} */ e) => act("set", { key: "profile.kid", value: e.currentTarget.checked })} /><span
              onClick=${() => act("set", { key: "profile.kid", value: !kid })}></span></span> Kid mode</label>`}
          ${run && html`<span class="dim">Software: <b>${run.label}</b>${run.uptime_s != null ? `, up ${fmtUp(run.uptime_s)}` : ""}</span>`}
        </div>
      </div>
    </div>
    ${up && st.mcu?.faults?.length > 0 && html`<${Banner} red=${true} title="Faults:">${st.mcu.faults.map((/** @type {any} */ f) => `${f.name} (${f.hint})`).join("; ")}<//>`}
    <div class="cards">
      ${CARDS.map(([id, title, text, tab, mission, need]) => {
        const live = st?.mission === mission;
        const ok = !need || caps[/** @type {string} */ (need)];
        let why = "";
        if (!up) why = "Start the robot first";
        else if (!ok) why = need === "catch" ? "Needs the catch code" : "Needs navigation";
        const catchSwitch = up && id === "catch" && !ok && stepOf("catch");
        return html`<div class=${`mode-card${live ? " live" : ""}`} key=${id}>
          <div class="t">${title}${live && html`<span class="badge green" style="margin-left: 8px">running</span>`}</div>
          <div class="d">${text}</div>
          ${why && html`<div class="why">${why}${catchSwitch ? ": in the sim it plays in its own world" : ""}</div>`}
          <div class="row">
            <button class="btn primary" onClick=${() => { location.hash = /** @type {string} */ (tab); }}>Open</button>
            ${catchSwitch && html`<button class="btn" onClick=${() => start("catch")}>Switch to the catch world</button>`}
            ${up && id === "catch" && ok && run?.step === "catch" && stepOf("robot") && html`<button class="btn small"
              onClick=${() => start("robot")} title="Back to the house world, where the other modes are">Back to the house</button>`}
          </div>
        </div>`;
      })}
    </div>
  </div>`;
}

/** @param {number} s */
export function fmtUp(s) {
  if (s < 90) return `${s} s`;
  if (s < 5400) return `${Math.round(s / 60)} min`;
  return `${(s / 3600).toFixed(1)} h`;
}
