// @ts-check
/* Tabs whose robot side comes in phase 2 (design.md, operator console): each says what it will do, so the layout is
   final now. The Version tab already shows what the page and the robot report. */
import { html } from "../../vendor/htm-preact-standalone.mjs";
import { PAGE_VERSION, useStore } from "../ros.js";

export const LATER = {
  explore: ["Explore", "Maps an unknown house by itself: it drives to the edges of what it has seen (frontiers) until nothing is left or a limit is reached.",
    ["Max distance from the start", "Max time", "Smallest gap worth visiting", "Speed", "Return home when done"]],
  patrol: ["Patrol", "Drives a route of points you click on the map, once, in a loop or back and forth, waiting at each point.",
    ["Routes saved per map", "Laps, wait time per point, speed", "Status: 'waypoint 3 of 6'"]],
  base: ["Base", "The drive: each wheel's motor status and errors, commanded vs measured speed charts, slip, tilt of the body, odometry.",
    ["Per-wheel DM-2325 state", "Speed and acceleration charts (200 Hz data in 20 Hz batches)", "Profile and limits"]],
  thrower: ["Thrower", "Bench control of the catch module with the base parked: spin the flywheels, tilt the module, fire one ball (the firmware's interlocks still apply).",
    ["Flywheel rpm", "Tilt angle", "Fire one ball (with a confirmation)", "Feed and beam states"]],
  vision: ["Vision", "What the stereo cameras see: detection rate, processing time, the tracked ball and its predicted landing point; the camera image as a video stream.",
    ["Detections and track", "Predicted crossing and its uncertainty", "Camera stream (MJPEG, not through rosbridge)"]],
  slam: ["SLAM", "Maps: start a new map, save it, load one and localize on it, set the robot's position by dragging on the map, localization quality.",
    ["New map / Save as / Load", "Set initial pose", "Scan-match quality"]],
  rc: ["RC", "The radio transmitter: whether it has taken over (its switch), its channels and link quality.",
    ["Takeover state (now: the MCU's MANUAL mode, in the status bar)", "Channel bars and link quality need the CRSF driver"]],
};

/** @param {{id: keyof typeof LATER}} p */
export function LaterTab({ id }) {
  const [title, what, items] = LATER[id];
  return html`<div class="full"><div class="card" style="max-width: 720px">
    <h2>${title} <span class="badge orange" style="margin-left: 8px">phase 2</span></h2>
    <p>${what}</p>
    <ul>${items.map((i) => html`<li>${i}</li>`)}</ul>
    <p class="dim">Its robot side isn't written yet. Manual, Go To, Catch and ROS work now.</p>
  </div></div>`;
}

export function VersionTab() {
  const s = useStore("status", "url");
  const st = s.status;
  return html`<div class="full"><div class="card" style="max-width: 720px">
    <h2>Versions</h2>
    <div class="kv">
      <span class="k">This page</span><span class="mono">${PAGE_VERSION}</span>
      <span class="k">Robot's console</span><span class="mono">${st?.version || "–"}</span>
      <span class="k">rosbridge</span><span class="mono">${s.url}</span>
      <span class="k">Browser</span><span class="mono" style="font-size: 12px">${navigator.userAgent}</span>
    </div>
    <p class="dim">Phase 2 adds the robot software's git commit and build time, the firmware's build, and a warning when
      the firmware and the robot software don't match.</p>
  </div></div>`;
}
