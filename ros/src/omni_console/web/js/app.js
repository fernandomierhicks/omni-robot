// @ts-check
/* omni console: one page to watch and command the robot. Mode tabs on the left of the bar, subsystem tabs on the
   right, the status bar always at the bottom. The robot serves this page (omni_console's console_web) and the page
   talks to it through rosbridge; see ros.js. */
import { html, render, useEffect, useState } from "../vendor/htm-preact-standalone.mjs";
import { PAGE_VERSION, connect, useStore } from "./ros.js";
import { StatusBar } from "./statusbar.js";
import { CatchTab } from "./tabs/catch.js";
import { GotoTab } from "./tabs/goto.js";
import { LaterTab, VersionTab } from "./tabs/later.js";
import { ManualTab } from "./tabs/manual.js";
import { RosTab } from "./tabs/rostab.js";
import { Banner } from "./ui.js";

const MODES = [
  ["manual", "Manual", "MANUAL"], ["goto", "Go To", "GOTO"], ["explore", "Explore", null], ["patrol", "Patrol", null],
  ["catch", "Catch", "CATCH"],
];
const SUBSYSTEMS = [["base", "Base"], ["thrower", "Thrower"], ["vision", "Vision"], ["slam", "SLAM"], ["rc", "RC"],
  ["version", "Version"], ["ros", "ROS"]];
const READY = new Set(["manual", "goto", "catch", "version", "ros"]);

function App() {
  const s = useStore("status", "conn");
  const [tab, setTab] = useState(() => location.hash.slice(1) || "manual");
  useEffect(() => {
    const f = () => setTab(location.hash.slice(1) || "manual");
    window.addEventListener("hashchange", f);
    return () => window.removeEventListener("hashchange", f);
  }, []);
  const st = s.status;
  const mismatch = st && st.version && st.version !== PAGE_VERSION;
  /** @param {string} id @param {string} label @param {string | null} [mission] */
  const tabBtn = (id, label, mission = null) => html`<button key=${id}
    class=${`tab${tab === id ? " sel" : ""}${READY.has(id) ? "" : " later"}`}
    onClick=${() => { location.hash = id; }} title=${READY.has(id) ? "" : "Coming in phase 2"}>
    ${label}${mission && st?.mission === mission && html`<span class="live" title="This mode is running"></span>`}</button>`;
  return html`
    <nav class="tabbar">
      <span class="brand">omni console</span>
      ${MODES.map(([id, label, m]) => tabBtn(/** @type {string} */ (id), /** @type {string} */ (label), m))}
      <span class="sep"></span>
      ${SUBSYSTEMS.map(([id, label]) => tabBtn(id, label))}
    </nav>
    ${mismatch && html`<${Banner} red=${true} title="Version mismatch.">The robot's console is ${st.version}, this page ${PAGE_VERSION}: reload the page (Ctrl+F5).<//>`}
    <main class="page">${page(tab)}</main>
    <${StatusBar} />`;
}

/** @param {string} tab */
function page(tab) {
  switch (tab) {
    case "goto": return html`<${GotoTab} />`;
    case "catch": return html`<${CatchTab} />`;
    case "ros": return html`<${RosTab} />`;
    case "version": return html`<${VersionTab} />`;
    case "explore": case "patrol": case "base": case "thrower": case "vision": case "slam": case "rc":
      return html`<${LaterTab} id=${tab} />`;
    default: return html`<${ManualTab} />`;
  }
}

render(html`<${App} />`, /** @type {HTMLElement} */ (document.getElementById("app")));
connect();
