// @ts-check
/* Home › Overview, the console's mission control and the first page a person sees (the iPad too): what the robot is
   doing now (Stop, and Cancel for a Nav2 task), the map and its camera side by side, the tasks anyone can start (a
   card per feature the robot lists with one: Start sends its verb, Open shows its page) and the activity history:
   every task that ran, whoever started it (this page, the voice, an AI client), and how it ended. When the robot's
   software isn't running, the supervisor starts it from here. */
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
  TIDY: "tidying up" });
const STATE_CLS = { running: "green", done: "", failed: "red", stopped: "orange", replaced: "" };

/** The camera Home shows: the floor camera's colour first. @param {string[]} imgs */
function mainCamera(imgs) {
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
    <h3>Do</h3>
    <div class="cards">
      ${cards.map((x) => {
        const a = availability(x, st);
        const live = !!x.mission && st?.mission === x.mission;
        return html`<div class=${`mode-card${live ? " live" : ""}`} key=${x.id}>
          <div class="t">${x.label}${live && html`<span class="badge green" style="margin-left: 8px">running</span>`}</div>
          <div class="d">${x.card}</div>
          ${!a.ok && html`<div class="why">${a.why}</div>`}
          <div class="row">
            ${x.start && !live && html`<button class="btn go" disabled=${!a.ok} onClick=${() => start(x, act)}>Start</button>`}
            ${live && html`<button class="btn danger" onClick=${() => act("stop")}>Stop</button>`}
            <button class=${x.start ? "btn" : "btn primary"} onClick=${() => { location.hash = x.id; }}>Open</button>
          </div>
        </div>`;
      })}
    </div>
    ${catchWorld && html`<p class="dim">The parked catch robot plays in its own sim world:
      <button class="btn small" onClick=${() => startSw("catch")}>Switch to the catch world</button></p>`}
    ${up && list.some((x) => x.id === "catch" && availability(x, st).ok) && run?.step === "catch" && stepOf("robot") && html`<p class="dim">
      <button class="btn small" onClick=${() => startSw("robot")} title="Back to the house world, where the other modes are">Back to the house</button></p>`}
    <${Activity} n=${st?.activity_n} up=${up} />
  </div>`;
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
