// @ts-check
/* Tidy (design.md 11): the arm robot picks up small things from the floor and drops them in its bin. omni_arm's
   arm_node runs it; this page starts and stops it and shows what it's doing. Moving the arm by hand is on the Arm
   page (the payload workspace); Drive has the short version. */
import { html } from "../../vendor/htm-preact-standalone.mjs";
import { act, useStore } from "../ros.js";
import { Banner, HelpBox, SettingsForm } from "../ui.js";
import { WRIST } from "./arm.js";
import { streamUrl } from "./vision.js";

const STATES = {
  STOW: "folding the arm", LOOK: "looking at the floor", TURN: "turning to look", APPROACH: "driving to it",
  CHECK: "a closer look", CREEP: "edging in", HOVER: "over it", REFINE: "wrist camera look", DESCEND: "reaching down",
  CLOSE: "closing the gripper", LIFT: "lifting", TO_BIN: "to the bin", RELEASE: "dropping it in", DONE: "done",
  IDLE: "idle",
};

export function TidyTab() {
  const s = useStore("status");
  const st = s.status;
  const a = st?.arm;
  const on = st?.mission === "TIDY";
  const t = a?.tidy;
  const v = st?.vision;
  const host = location.hostname || "localhost";
  const wrist = v?.video_port && (v?.images || []).includes(WRIST);
  return html`<div class="cols">
    <div class="side">
      <${HelpBox} tab="tidy">
        The arm robot tidies up: it looks at the floor with the RealSense D435 on its riser, drives to each small thing
        it sees, picks it up (a last look through the wrist camera first) and drops it in the bin on its back.
        <ol>
          <li>Press <b>Tidy up</b>. It covers the circle set below (How far): it drives itself, slowly, only through
            floor its camera has seen free (not Nav2), turns to look round, and comes back to where it started.</li>
          <li><b>Stop</b> (or STOP, or another mode) folds the arm; a thing already lifted goes into the bin first.</li>
        </ol>
        With the arm out the base drives at 1 m/s and half its acceleration at most, whoever drives. Things wider
        than 6.5 cm, or bigger than 12 cm, are left where they are. The arm by hand: the <a href="#arm">Arm</a> page.
      <//>
      ${!a && html`<${Banner} title="No arm.">This needs the arm robot: start 'Arm robot: tidy up' (sim) or omni_arm's arm_node.<//>`}
      ${on ? html`<button class="btn big" onClick=${() => act("stop")}>Stop</button>`
        : html`<button class="btn go big" disabled=${!a} onClick=${() => act("tidy")}>Tidy up</button>`}
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
