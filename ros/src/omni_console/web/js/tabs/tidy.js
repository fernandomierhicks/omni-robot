// @ts-check
/* Tidy (design.md 11): the arm robot picks up small things from the floor and drops them in its bin. omni_arm's
   arm_node runs it; this tab starts and stops it, shows what it's doing and the arm, and moves the arm by hand: named
   poses, the gripper, one joint at a time (arm_node refuses a jog that would hit the robot, the bumper or the floor). */
import { html, useState } from "../../vendor/htm-preact-standalone.mjs";
import { act, useStore } from "../ros.js";
import { Banner, HelpBox, SettingsForm } from "../ui.js";
import { streamUrl } from "./vision.js";

const WRIST = "/arm_cam/image_raw";
const STATES = {
  STOW: "folding the arm", LOOK: "looking at the floor", TURN: "turning to look", APPROACH: "driving to it",
  CHECK: "a closer look", HOVER: "over it", REFINE: "wrist camera look", DESCEND: "reaching down",
  CLOSE: "closing the gripper", LIFT: "lifting", TO_BIN: "to the bin", RELEASE: "dropping it in", DONE: "done",
  IDLE: "idle",
};

export function TidyTab() {
  const s = useStore("status");
  const [step, setStep] = useState(5);
  const st = s.status;
  const a = st?.arm;
  const on = st?.mission === "TIDY";
  const t = a?.tidy;
  const v = st?.vision;
  const host = location.hostname || "localhost";
  const wrist = v?.video_port && (v?.images || []).includes(WRIST);
  /** @param {object} args */
  const arm = (args) => act("arm", args);
  return html`<div class="cols">
    <div class="side">
      <${HelpBox} tab="tidy">
        The arm robot tidies up: it looks at the floor with its stereo cameras, drives to each small thing it sees,
        picks it up (a last look through the wrist camera first) and drops it in the bin on its back.
        <ol>
          <li>Clear a path: the tidy drives straight to each thing, slowly, and stops on its bumper (no Nav2 yet).</li>
          <li>Press <b>Tidy up</b>. It turns to look round when nothing is in view and comes back to where it
            started before it gives up.</li>
          <li><b>Stop</b> (or STOP, or another mode) folds the arm; a thing already lifted goes into the bin first.</li>
        </ol>
        With the arm out (beyond the bumper or up high) the base drives at 1 m/s and half its acceleration at most,
        whoever drives. Things wider than 6.5 cm, or bigger than 12 cm, are left where they are.
        <p>By hand (not while it tidies): the poses go round the robot on a checked path; − / + move one joint by
          the step, and a move that would hit the robot, the bumper or the floor is refused (the Logbook says why).</p>
      <//>
      ${!a && html`<${Banner} title="No arm.">This needs the arm robot: start 'Arm robot: tidy up' (sim) or omni_arm's arm_node.<//>`}
      ${on ? html`<button class="btn big" onClick=${() => act("stop")}>Stop</button>`
        : html`<button class="btn go big" disabled=${!a} onClick=${() => act("tidy")}>Tidy up</button>`}
      <h3>The arm by hand</h3>
      <div class="row" style="flex-wrap: wrap; gap: 6px">
        <button class="btn small" disabled=${!a || on} onClick=${() => arm({ do: "pose", name: "stow" })}>Fold (stow)</button>
        <button class="btn small" disabled=${!a || on} onClick=${() => arm({ do: "pose", name: "ready" })}>Ready</button>
        <button class="btn small" disabled=${!a || on} onClick=${() => arm({ do: "pose", name: "high" })}>High</button>
        <button class="btn small" disabled=${!a || on} onClick=${() => arm({ do: "pose", name: "drop" })}>Over the bin</button>
        <button class="btn small" disabled=${!a || on} onClick=${() => arm({ do: "gripper", gap_mm: 70 })}>Open</button>
        <button class="btn small" disabled=${!a || on} onClick=${() => arm({ do: "gripper", gap_mm: 0 })}>Close</button>
        <button class="btn small" disabled=${!a || on} onClick=${() => arm({ do: "hold" })}>Hold</button>
        <button class="btn small" disabled=${!a || on} onClick=${() => {
          if (confirm("Torque off? The arm drops: hold it first (it has no brakes).")) arm({ do: "relax" });
        }}>Torque off</button>
      </div>
      <h3>One joint at a time</h3>
      ${a?.names && a?.joints ? html`<div class="kv">
        ${a.names.map((/** @type {string} */ n, /** @type {number} */ k) => html`<span class="k">${n.replace("_", " ")}</span>
          <span class="row" style="gap: 4px">
            <button class="btn small" disabled=${on} onClick=${() => arm({ do: "jog", joint: n, deg: -step })}>−</button>
            <span class="mono" style="min-width: 4.5em; text-align: center">${a.joints[k].toFixed(1)}°</span>
            <button class="btn small" disabled=${on} onClick=${() => arm({ do: "jog", joint: n, deg: step })}>+</button>
          </span>`)}
        <span class="k">step</span>
        <select value=${step} onChange=${(/** @type {any} */ e) => setStep(Number(e.currentTarget.value))}>
          ${[1, 5, 15].map((v) => html`<option value=${v}>${v}°</option>`)}</select>
      </div>` : html`<p class="dim">No arm state.</p>`}
      <${SettingsForm} tab="tidy" />
    </div>
    <div class="main full">
      <div class="row" style="align-items: stretch; gap: 10px">
        <div class="card grow">
          <h2>Tidy ${on && html`<span class="badge green" style="margin-left: 6px">running</span>`}</h2>
          ${t ? html`<div class="kv">
            <span class="k">doing</span><span>${STATES[t.state] || t.state}</span>
            <span class="k">in the bin</span><span class="mono">${t.picked}</span>
            <span class="k">not picked</span><span class="mono">${t.failed}${t.skipped ? ` (+${t.skipped} too big)` : ""}</span>
            <span class="k">seen, still to do</span><span class="mono">${t.known}</span>
            <span class="k">now</span><span class="mono">${t.target ? `#${t.target.id} at (${t.target.x}, ${t.target.y}), ${Math.round(t.target.size[0] * 1000)} × ${Math.round(t.target.size[1] * 1000)} mm${t.tries ? `, try ${t.tries + 1}` : ""}` : "–"}</span>
            ${t.reason && html`<span class="k">ended</span><span>${t.reason}</span>`}
          </div>` : html`<p class="dim">${a ? "Not running." : "–"}</p>`}
        </div>
        <div class="card grow">
          <h2>Arm ${a && html`<span class=${a.torque ? "badge green" : "badge"} style="margin-left: 6px">${a.torque ? "torque on" : "limp"}</span>`}
            ${a?.out && html`<span class="badge" style="margin-left: 4px" title="The base is capped">out</span>`}</h2>
          ${a?.joints ? html`<div class="kv">
            ${a.names.map((/** @type {string} */ n, /** @type {number} */ k) => html`<span class="k">${n.replace("_", " ")}</span>
              <span class="mono">${a.joints[k].toFixed(1)}°  load ${Math.round(a.loads[k] * 100)}%  ${a.temps[k]} °C</span>`)}
            <span class="k">supply</span><span class="mono">${a.volts} V</span>
            <span class="k">moved by</span><span>${a.source}</span>
          </div>` : html`<p class="dim">No arm state.</p>`}
        </div>
      </div>
      <div class="card" style="max-width: 680px">
        <h2>Wrist camera</h2>
        ${wrist ? html`<img style="width: 100%; border-radius: 6px; background: var(--map-bg)" alt="wrist camera"
            src=${streamUrl(host, v.video_port, WRIST)} />`
          : html`<p class="dim">No wrist camera picture (${v?.video_port ? "no /arm_cam/image_raw" : "web_video_server isn't running"}).</p>`}
      </div>
    </div>
  </div>`;
}
