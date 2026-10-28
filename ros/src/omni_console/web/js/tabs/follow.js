// @ts-check
/* Follow me and Tag: the robot finds a person's legs in its lidar scan (omni_console/people.py) and follows them, or
   chases them at kid speed and backs off after each tag. */
import { html } from "../../vendor/htm-preact-standalone.mjs";
import { MapView } from "../mapview.js";
import { act, useStore } from "../ros.js";
import { Banner, HelpBox, SettingsForm } from "../ui.js";

export function FollowTab() {
  const s = useStore("status");
  const st = s.status;
  const f = st?.mission === "FOLLOW" ? st.follow : null;
  const pose = st?.pose;
  /** @type {{x: number, y: number, color?: string, label?: string}[]} */
  const marks = [];
  if (f?.person && pose) {
    const [d, b] = f.person;
    marks.push({ x: pose.x + d * Math.cos(pose.yaw + b), y: pose.y + d * Math.sin(pose.yaw + b), color: "yellow",
      label: f.mode === "tag" ? "it!" : "you" });
  }
  return html`<div class="cols">
    <div class="side">
      <${HelpBox} tab="follow">
        <ol>
          <li>Stand 1 to 2 m in front of the robot, facing it.</li>
          <li>Press <b>Follow me</b>: walk, and the robot keeps the distance below behind you. Or press <b>Tag</b>:
            run! The robot chases you; when it gets within half a metre it says "Tagged!", backs off, counts three
            seconds and chases again.</li>
        </ol>
        The robot sees legs in its lidar (about 15 cm off the floor), so long skirts and pets confuse it. Furniture
        on the map is ignored. If it loses you for 2 s it stops and waits: step in front of it again.
        <b>Stop</b> or <kbd>Space</kbd> ends it. With navigation running, its collision monitor still stops it short of
        walls.
      <//>
      ${st && !st.caps?.collision && html`<${Banner} title="No collision monitor.">Navigation isn't running, so nothing but the firmware's limits stops the robot short of things: follow in an open space.<//>`}
      ${f ? html`<div class="card">
          <div style="font-size: 18px; font-weight: 700; color: var(--white)">${f.mode === "tag" ? "Tag" : "Follow me"}</div>
          <div style="margin: 6px 0">${f.text}</div>
          ${f.person && html`<div class="dim">You're ${f.person[0].toFixed(1)} m away.</div>`}
          ${f.mode === "tag" && html`<div class="stats" style="margin-top: 8px"><div class="stat"><div class="v">${f.tags}</div><div class="k">tags</div></div></div>`}
          <button class="btn big danger" style="margin-top: 10px" onClick=${() => act("stop")}>Stop</button>
        </div>`
        : html`<div class="row" style="gap: 10px; margin-bottom: 10px">
          <button class="btn go big grow" style="width: auto" disabled=${!st} onClick=${() => act("follow", { mode: "follow" })}>Follow me</button>
          <button class="btn primary big grow" style="width: auto" disabled=${!st} onClick=${() => act("follow", { mode: "tag" })}>Tag</button></div>`}
      <${SettingsForm} tab="follow" />
    </div>
    <div class="main">
      <${MapView} marks=${marks} hint=${f ? f.text : "Press Follow me or Tag"} />
    </div>
  </div>`;
}
