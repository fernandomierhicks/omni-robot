// @ts-check
/* The generic page: a feature the robot lists (console_defs.py FEATURES) that this page has no page of its own for. It
   shows what the robot says about it, whether it can run, its start (and a stop while it runs) and its settings
   (every Setting whose tabs name it). A new robot skill is usable from the console before anyone writes its page. */
import { html } from "../../vendor/htm-preact-standalone.mjs";
import { availability, start } from "../features.js";
import { act, useStore } from "../ros.js";
import { Banner, SettingsForm } from "../ui.js";

/** @param {{feature?: import("../features.js").Feature}} p */
export function FeatureTab({ feature }) {
  const s = useStore("status", "schema");
  const st = s.status;
  if (!feature) return html`<div class="full"><p class="dim">No such page.</p></div>`;
  const a = availability(feature, st);
  const live = !!feature.mission && st?.mission === feature.mission;
  return html`<div class="full" style="max-width: 760px">
    <div class="card">
      <h2>${feature.label} ${live && html`<span class="badge green" style="margin-left: 6px">running</span>`}</h2>
      <p>${feature.card || feature.hint || "A feature the robot offers."}</p>
      ${!a.ok && html`<${Banner} title="Not available.">${a.why}<//>`}
      <div class="row">
        ${feature.start && !live && html`<button class="btn go" disabled=${!a.ok} onClick=${() => start(feature, act)}>Start</button>`}
        ${live && html`<button class="btn danger" onClick=${() => act("stop")}>Stop</button>`}
      </div>
    </div>
    <div class="card"><${SettingsForm} tab=${feature.id} /></div>
    <p class="dim" style="font-size: 12px">This page is drawn from what the robot reports (its features and settings):
      the console has no page of its own for '${feature.id}' yet.</p>
  </div>`;
}
