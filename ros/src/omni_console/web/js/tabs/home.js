// @ts-check
/* Home › Overview, the console's mission control and the first page a person sees (the iPad too): what the robot is
   doing now (Stop, and Cancel for a Nav2 task) and what it knows about its own health (a chip per preflight check),
   the map and its camera side by side, the modes (a card per feature the robot lists with one, driving by hand
   among them: Start sends its verb, Open shows its page), the ones it can run now first and the rest greyed with the
   robot's reason, and the activity history: every task that ran, whoever started it (this page, the voice, an AI
   client), and how it ended. When the robot's software isn't running, the supervisor starts it from here. */
import { html, useEffect, useState } from "../../vendor/htm-preact-standalone.mjs";
import { availability, features, start, visible } from "../features.js";
import { MapView } from "../mapview.js";
import { act, command, useStore } from "../ros.js";
import { missionName } from "../statusbar.js";
import { runningStep, startStep, stepArgs } from "../system.js";
import { Banner, clock } from "../ui.js";
import { streamUrl } from "./vision.js";

const NAV = ["GOTO", "EXPLORE", "PATROL"];
/** "The robot is ..." */
const DOING = /** @type {Record<string, string>} */ ({ IDLE: "idle (motors off)", MANUAL: "ready to drive",
  GOTO: "going to a place", EXPLORE: "exploring", PATROL: "patrolling", CATCH: "playing catch",
  THROWER: "on the thrower test", FOLLOW: "following", PROGRAM: "running a program", CALIBRATE: "calibrating",
  TIDY: "tidying up", MOTORS: "on the motor test" });
const STATE_CLS = { running: "green", done: "", failed: "red", stopped: "orange", replaced: "" };

/** The camera Home shows: the floor camera's colour first. @param {string[]} imgs */
export function mainCamera(imgs) {
  const score = (/** @type {string} */ t) => (/depth|ir|infra/i.test(t) ? 0 : /d435.*colou?r|color|front/i.test(t) ? 3
    : /arm_cam|wrist/i.test(t) ? 1 : 2);
  return imgs.slice().sort((a, b) => score(b) - score(a))[0];
}

export function HomeTab() {
  const s = useStore("status", "conn", "supervisor", "schema");
  const st = s.status;
  const sup = s.supervisor;
  const run = runningStep();
  const list = features();
  const kid = !!st?.kid;
  const stepOf = (/** @type {string} */ id) => sup?.steps?.find((/** @type {any} */ x) => x.id === id);
  const startSw = (/** @type {string} */ id) => { const step = stepOf(id); if (step) startStep(id, stepArgs(step)); };
  const up = s.conn === "open" && !!st;
  const robotStep = sup && (stepOf("robot") ? "robot" : sup.steps?.[0]?.id);
  const cards = list.filter((x) => x.card && visible(x, st));
  const ready = cards.filter((x) => availability(x, st).ok || (!!x.mission && st?.mission === x.mission));
  const notNow = cards.filter((x) => !ready.includes(x));
  // driving can't start but some motors answer: the motor test is what can be controlled, so it's offered here too
  const motorsF = list.find((x) => x.id === "motors");
  const testOnly = up && !!motorsF && notNow.some((x) => x.id === "manual") && availability(motorsF, st).ok
    && (st.mcu?.dm_err || []).some((/** @type {number} */ e) => e !== 255);
  const catchWorld = up && !list.some((x) => x.id === "catch" && availability(x, st).ok) && stepOf("catch");
  const v = st?.vision;
  const cam = v?.video_port && v.images?.length ? mainCamera(v.images) : null;
  const pf = st?.preflight;
  return html`<div class="full home">
    <div class="card now">
      <div class="row" style="align-items: flex-start; gap: 12px; flex-wrap: wrap">
        <div class="grow" style="flex: 1 1 260px">
          <h2>${up ? `The robot is ${DOING[st.mission] || missionName(st.mission).toLowerCase()}` : run ? "Starting the robot's software..." : "The robot's software isn't running"}</h2>
          <div class="dim">${up ? st.task : run ? `${run.label}: ${run.state}, ${run.uptime_s ?? 0} s (System: its log)` : sup ? "Start it here." : "Start a step in the Launcher (or omni.sh)."}</div>
          ${up && st.progress != null && html`<div class="bar big"><div style=${`width: ${Math.round(st.progress * 100)}%`}></div></div>`}
          ${up && pf?.rows && html`<${Health} rows=${pf.rows} />`}
        </div>
        <div class="row" style="gap: 8px; flex-wrap: wrap">
          ${!up && !run && robotStep && html`<button class="btn go" onClick=${() => startSw(robotStep)}>Start the robot</button>`}
          ${up && NAV.includes(st.mission) && html`<button class="btn" onClick=${() => act("cancel")} title="Stop the task; the motors stay on">Cancel</button>`}
          ${up && st.mission !== "IDLE" && html`<button class="btn danger" onClick=${() => act("stop")}>Stop (motors off)</button>`}
          ${up && html`<label class="row" title="The firmware's kid limits in every mode: 0.8 m/s, gentle accelerations">
            <span class="switch"><input type="checkbox" checked=${kid}
              onChange=${(/** @type {any} */ e) => act("set", { key: "profile.kid", value: e.currentTarget.checked })} /><span
              onClick=${() => act("set", { key: "profile.kid", value: !kid })}></span></span> Kid mode</label>`}
          ${pf && html`<a href="#preflight" class=${`badge ${pf.fail ? "red" : pf.warn ? "orange" : "green"}`} style="text-decoration: none; white-space: nowrap"
            title=${pf.worst?.join(", ") || "All checks pass"}>Preflight: ${pf.fail ? `${pf.fail} failing` : pf.warn ? `${pf.warn} to look at` : "ok"}</a>`}
        </div>
      </div>
    </div>
    ${up && st.mcu?.faults?.length > 0 && html`<${Banner} red=${true} title="Faults:">${st.mcu.faults.map((/** @type {any} */ f) => `${f.name} (${f.hint})`).join("; ")}<//>`}
    <div class="home-views">
      <div class="card view-map"><${MapView} zones=${s.schema?.zones} edges=${s.schema?.edges}
        hint=${up ? (st.goal ? `Going to ${st.goal.name || "a point"}` : "") : ""} /></div>
      <div class="card view-cam">
        ${cam ? html`<img alt=${cam} title=${cam} src=${streamUrl(location.hostname || "localhost", v.video_port, cam)} />`
          : html`<p class="dim">${up ? (v?.video_port ? "No camera stream" : "No camera: web_video_server isn't running") : "–"}</p>`}
        ${cam && html`<div class="dim mono" style="font-size: 11px">${cam} · <a href="#cameras">all cameras</a></div>`}
      </div>
    </div>
    <h3>${up && notNow.length ? "Ready now" : "Do"}</h3>
    <div class="cards">${ready.map((x) => html`<${Card} key=${x.id} x=${x} st=${st} />`)}
      ${testOnly && html`<div class="mode-card" key="motors">
        <div class="t">Motor test</div>
        <div class="d">Driving is off until all four motors answer. Turn the ones that do, one at a time, with the
          wheels off the ground.</div>
        <div class="row"><button class="btn primary" onClick=${() => { location.hash = "motors"; }}>Open</button></div>
      </div>`}</div>
    ${up && !ready.length && !testOnly && html`<p class="dim">Nothing can start right now: the reasons are on each card below.</p>`}
    ${notNow.length > 0 && html`<h3 class="cards-sub">${up ? "Not available now" : "Modes"}</h3>
      <div class="cards">${notNow.map((x) => html`<${Card} key=${x.id} x=${x} st=${st} />`)}</div>`}
    ${catchWorld && html`<p class="dim">The parked catch robot plays in its own sim world:
      <button class="btn small" onClick=${() => startSw("catch")}>Switch to the catch world</button></p>`}
    ${up && list.some((x) => x.id === "catch" && availability(x, st).ok) && run?.step === "catch" && stepOf("robot") && html`<p class="dim">
      <button class="btn small" onClick=${() => startSw("robot")} title="Back to the house world, where the other modes are">Back to the house</button></p>`}
    <${Activity} n=${st?.activity_n} up=${up} />
  </div>`;
}

/** A mode's card: Start (greyed with the robot's reason when it can't), Stop while it runs, Open its page.
 *  @param {{x: any, st: any}} p */
function Card({ x, st }) {
  const a = availability(x, st);
  const live = !!x.mission && st?.mission === x.mission;
  // the card says what it waits on ("Drive motors · IMU · Lidar"); the whole reason on hover and on its page
  const short = a.why.split("; ").map((w) => (/^[A-Z][\w ]{1,24}: /.test(w) ? w.split(": ")[0] : w)).join(" · ");
  return html`<div class=${`mode-card${live ? " live" : ""}${a.ok || live ? "" : " unavail"}`}>
    <div class="t">${x.label}${live && html`<span class="badge green" style="margin-left: 8px">running</span>`}</div>
    ${!a.ok && !live && html`<div class="why" title=${a.why}>Waiting on: ${short}</div>`}
    <div class="d">${x.card}</div>
    <div class="row">
      ${x.start && !live && html`<button class="btn go" disabled=${!a.ok} onClick=${() => start(x, act)}>Start</button>`}
      ${live && html`<button class="btn danger" onClick=${() => act("stop")}>Stop</button>`}
      <button class=${x.start || !a.ok ? "btn" : "btn primary"} onClick=${() => { location.hash = x.id; }}>Open</button>
    </div>
  </div>`;
}

const DOT = { ok: "ok", warn: "warn", fail: "err", na: "" };
/** What the robot knows about itself: a chip per preflight check (status preflight.rows), the details on hover; the
 *  Preflight page has them all. @param {{rows: any[]}} p */
function Health({ rows }) {
  const order = { fail: 0, warn: 1, ok: 2, na: 3 };
  const shown = rows.slice().sort((a, b) => order[/** @type {keyof typeof order} */ (a.state)] - order[/** @type {keyof typeof order} */ (b.state)]);
  return html`<div class="health">${shown.map((r) => html`<a key=${r.id} href="#preflight"
    class=${`chip ${r.state}`} title=${r.detail || (r.state === "na" ? "Not seen (not running, or not fitted)" : "ok")}>
    <span class=${`dot ${DOT[/** @type {keyof typeof DOT} */ (r.state)]}`}></span>${r.label}</a>`)}</div>`;
}

/** What ran, newest first. Asked for again when the status's activity_n changes. @param {{n: number | undefined, up: boolean}} p */
function Activity({ n, up }) {
  const [items, setItems] = useState(/** @type {any[]} */ ([]));
  useEffect(() => {
    if (!up) return;
    command("activity").then((r) => { if (r.ok) setItems(r.data.items); });
  }, [n, up]);
  if (!items.length) return null;
  return html`<h3>Activity</h3><div class="card activity">
    ${items.slice(0, 12).map((x) => html`<div class="ev" key=${x.n}>
      <span class="mono dim">${clock(x.t0)}</span>
      <span class=${`badge ${STATE_CLS[/** @type {keyof typeof STATE_CLS} */ (x.state)] ?? ""}`}>${x.state}</span>
      <span class="badge" title="Who started it">${x.who}</span>
      <span>${x.label}${x.result && x.result !== x.label ? html`<span class="dim"> → ${x.result}</span>` : ""}
        ${x.t1 ? html`<span class="dim"> (${fmtUp(Math.round(x.t1 - x.t0))})</span>` : ""}</span></div>`)}
  </div>`;
}

/** @param {number} s */
export function fmtUp(s) {
  if (s < 90) return `${s} s`;
  if (s < 5400) return `${Math.round(s / 60)} min`;
  return `${(s / 3600).toFixed(1)} h`;
}
