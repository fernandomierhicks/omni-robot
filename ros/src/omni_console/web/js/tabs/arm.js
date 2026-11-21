// @ts-check
/* Arm (the payload workspace on the arm robot): the SO-101 by hand and its state. omni_arm's arm_node does the moving:
   the named poses go round the robot on a checked path, a jog moves one joint, and arm_node refuses a move that would
   hit the robot, the bumper or the floor (the Logbook says why). Nothing here works while the tidy runs: it owns the
   arm. ArmPad is the short version Drive shows beside the base. */
import { html, useState } from "../../vendor/htm-preact-standalone.mjs";
import { useGate } from "../features.js";
import { act, useStore } from "../ros.js";
import { saveStepArgs, startStep, stepArgs } from "../system.js";
import { Banner, Gate, HelpBox } from "../ui.js";
import { streamUrl } from "./vision.js";

/** No arm state: say why and offer to start it. On the robot computer the robot step's arm option (hardware.launch.py
 *  arm:=true: the servo bus's driver and arm_node join the step, which restarts); in the sim the arm robot's step. */
function ArmStart() {
  const s = useStore("supervisor");
  const [busy, setBusy] = useState(false);
  const sup = s.supervisor;
  const steps = /** @type {any[]} */ (sup?.steps || []);
  const robot = steps.find((x) => x.id === "robot");
  const withArm = robot?.args?.some((/** @type {any} */ a) => a.key === "arm");
  const simArm = steps.find((x) => x.id === "arm");
  const cur = sup?.current;
  const armOn = cur?.step === "robot" && cur?.state === "running" && String(cur?.args?.arm) === "true";
  const go = async (/** @type {any} */ step, /** @type {Record<string, string>} */ args) => {
    setBusy(true); saveStepArgs(step.id, args); await startStep(step.id, args); setBusy(false);
  };
  if (withArm && armOn) {
    return html`<${Banner} title="The arm is switched on, but it isn't answering yet.">arm_node starts with the robot's
      software and holds the arm where it is once it finds the servo bus. Still nothing after half a minute: the
      SO-101's bus adapter isn't plugged in, its 12 V is off, or there's no arm calibration (the step's log in
      <a href="#system">Robot › System</a> says which).<//>`;
  }
  if (withArm) {
    return html`<div class="gate"><b>The arm is off.</b>
      <div>The robot's software runs without the arm. Switching it on restarts the robot's software with the SO-101's
        driver and arm_node (the base stops for the restart, about half a minute).</div>
      <button class="btn go" style="margin-top: 8px" disabled=${busy || cur?.state === "stopping"}
        onClick=${() => { if (confirm("Restart the robot's software with the arm on? Whatever it's doing stops.")) go(robot, { ...stepArgs(robot), arm: "true" }); }}>
        ${busy ? "Restarting..." : "Switch the arm on"}</button></div>`;
  }
  if (simArm) {
    return html`<div class="gate"><b>No arm in this sim world.</b>
      <div>The arm robot runs in its own step ('${simArm.label}').</div>
      <button class="btn go" style="margin-top: 8px" disabled=${busy}
        onClick=${() => go(simArm, stepArgs(simArm))}>${busy ? "Starting..." : `Start '${simArm.label}'`}</button></div>`;
  }
  return html`<${Banner} title="No arm.">This needs the arm robot's software (omni_arm's arm_node), and this page
    can't start it from here: start it from the Launcher or omni.sh.<//>`;
}

export const WRIST = "/arm_cam/image_raw";
const POSES = [["stow", "Fold (stow)"], ["ready", "Ready"], ["high", "High"], ["look", "Look ahead"], ["drop", "Over the bin"]];

/** @param {object} args */
const arm = (args) => act("arm", args);

/** The poses and the gripper (greyed while the robot refuses arm moves: arm feature unavailable). Hold is never.
 *  @param {{a: any, busy: boolean}} p */
export function ArmPad({ a, busy }) {
  const g = useGate("arm");
  const off = !a || busy || !g.ok;
  return html`<div class="row" style="flex-wrap: wrap; gap: 6px">
    ${POSES.map(([n, label]) => html`<button class="btn small" disabled=${off} onClick=${() => arm({ do: "pose", name: n })}>${label}</button>`)}
    <button class="btn small" disabled=${off} onClick=${() => arm({ do: "gripper", gap_mm: 70 })}>Open</button>
    <button class="btn small" disabled=${off} onClick=${() => arm({ do: "gripper", gap_mm: 0 })}>Close</button>
    <button class="btn small" disabled=${!a} onClick=${() => arm({ do: "hold" })} title="Hold where it is (torque on)">Hold</button>
  </div>`;
}

export function ArmTab() {
  const s = useStore("status");
  const [step, setStep] = useState(5);
  const st = s.status;
  const a = st?.arm;
  const g = useGate("arm");
  const busy = st?.mission === "TIDY" || !g.ok;
  const v = st?.vision;
  const host = location.hostname || "localhost";
  const wrist = v?.video_port && (v?.images || []).includes(WRIST);
  return html`<div class="cols">
    <div class="side">
      <${HelpBox} tab="arm">
        The SO-101 by hand. The poses go round the robot on a checked path; − / + move one joint by the step. A move
        that would hit the robot, the bumper or the floor is refused (the Logbook says why). With the arm out (beyond
        the bumper or up high) the base drives at 1 m/s and half its acceleration at most, whoever drives. The tidy
        owns the arm while it runs: stop it first. <b>Torque off</b> drops the arm: hold it.
      <//>
      ${!a && html`<${ArmStart} />`}
      ${a && !g.ok && html`<${Gate} a=${g} />`}
      ${st?.mission === "TIDY" && html`<${Banner} title="The tidy has the arm.">Stop the tidy (Home › Tidy) to move it by hand.<//>`}
      <h3>Poses and gripper</h3>
      <${ArmPad} a=${a} busy=${busy} />
      <div class="row" style="margin-top: 6px">
        <button class="btn small danger" disabled=${!a || st?.mission === "TIDY"} onClick=${() => {
          if (confirm("Torque off? The arm drops: hold it first (it has no brakes).")) arm({ do: "relax" });
        }}>Torque off</button>
      </div>
      <h3>One joint at a time</h3>
      ${a?.names && a?.joints ? html`<div class="kv">
        ${a.names.map((/** @type {string} */ n, /** @type {number} */ k) => html`<span class="k">${n.replace("_", " ")}</span>
          <span class="row" style="gap: 4px">
            <button class="btn small" disabled=${busy} onClick=${() => arm({ do: "jog", joint: n, deg: -step })}>−</button>
            <span class="mono" style="min-width: 4.5em; text-align: center">${a.joints[k].toFixed(1)}°</span>
            <button class="btn small" disabled=${busy} onClick=${() => arm({ do: "jog", joint: n, deg: step })}>+</button>
          </span>`)}
        <span class="k">step</span>
        <select value=${step} onChange=${(/** @type {any} */ e) => setStep(Number(e.currentTarget.value))}>
          ${[1, 5, 15].map((x) => html`<option value=${x}>${x}°</option>`)}</select>
      </div>` : html`<p class="dim">No arm state.</p>`}
    </div>
    <div class="main full">
      <div class="card">
        <h2>Arm ${a && html`<span class=${a.torque ? "badge green" : "badge"} style="margin-left: 6px">${a.torque ? "torque on" : "limp"}</span>`}
          ${a?.out && html`<span class="badge orange" style="margin-left: 4px" title="The base is capped at 1 m/s">out</span>`}
          ${a && a.ok === false && html`<span class="badge red" style="margin-left: 4px">servo fault</span>`}</h2>
        ${a?.joints ? html`<div class="kv">
          ${a.names.map((/** @type {string} */ n, /** @type {number} */ k) => html`<span class="k">${n.replace("_", " ")}</span>
            <span class="mono">${a.joints[k].toFixed(1)}°  load ${Math.round(a.loads[k] * 100)}%  ${a.temps[k]} °C${
              a.external && k < 5 ? `  push ${a.external[k] >= 0 ? "+" : ""}${a.external[k].toFixed(2)} N m` : ""}${
              a.strained?.[k] ? "  STRAINED" : ""}</span>`)}
          <span class="k">supply</span><span class="mono">${a.volts} V</span>
          ${a.force && html`<span class="k" title="The outside torques as one force at the fingertip (design.md 4.5)">outside force</span>
            <span class="mono">${Math.hypot(...a.force).toFixed(1)} N (${a.force.map((/** @type {number} */ f) => f.toFixed(1)).join(", ")})</span>`}
          ${a.load_model && html`<span class="k">load model</span><span>${a.load_model}${a.reflexes ? `, ${a.reflexes} reflexes${a.reflex ? ` (last: ${a.reflex})` : ""}` : ""}</span>`}
          <span class="k">moved by</span><span>${a.source}</span>
        </div>` : html`<p class="dim">No arm state.</p>`}
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
