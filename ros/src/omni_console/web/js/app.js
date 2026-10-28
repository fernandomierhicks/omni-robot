// @ts-check
/* omni console: one page to watch and command the robot. Home and the mode tabs on the left of the bar, the robot's
   subsystems and tools under "Robot" on the right, the status bar always at the bottom. The robot serves this page
   (omni_console's supervisor, or console_web) and the page talks to it through rosbridge (ros.js) and, for starting
   and stopping its software, to the supervisor (system.js). */
import { html, render, useEffect, useState } from "../vendor/htm-preact-standalone.mjs";
import { useDriveLoop } from "./drive.js";
import { PAGE_VERSION, command, connect, useStore } from "./ros.js";
import { StatusBar } from "./statusbar.js";
import "./system.js";
import { BaseTab } from "./tabs/base.js";
import { CalibrateTab } from "./tabs/calibrate.js";
import { CatchTab } from "./tabs/catch.js";
import { ExploreTab } from "./tabs/explore.js";
import { FaceTab } from "./tabs/face.js";
import { FollowTab } from "./tabs/follow.js";
import { GotoTab } from "./tabs/goto.js";
import { HealthTab } from "./tabs/health.js";
import { HomeTab } from "./tabs/home.js";
import { LogbookTab } from "./tabs/logbook.js";
import { ManualTab } from "./tabs/manual.js";
import { MapEditTab } from "./tabs/mapedit.js";
import { PatrolTab } from "./tabs/patrol.js";
import { ProgramTab } from "./tabs/program.js";
import { RcTab } from "./tabs/rc.js";
import { RosTab } from "./tabs/rostab.js";
import { SlamTab } from "./tabs/slam.js";
import { SystemTab } from "./tabs/system.js";
import { ThrowerTab } from "./tabs/thrower.js";
import { TidyTab } from "./tabs/tidy.js";
import { VersionTab, mismatch } from "./tabs/version.js";
import { VisionTab } from "./tabs/vision.js";
import { Banner } from "./ui.js";

/** [id, label, mission (the live dot), component] */
const MODES = [
  ["home", "Home", null, HomeTab],
  ["manual", "Manual", "MANUAL", ManualTab], ["goto", "Go To", "GOTO", GotoTab],
  ["explore", "Explore", "EXPLORE", ExploreTab], ["patrol", "Patrol", "PATROL", PatrolTab],
  ["catch", "Catch", "CATCH", CatchTab], ["follow", "Follow", "FOLLOW", FollowTab],
  ["program", "Program", "PROGRAM", ProgramTab], ["tidy", "Tidy", "TIDY", TidyTab],
];
const TOOLS = [
  ["base", "Base", null, BaseTab], ["thrower", "Thrower", "THROWER", ThrowerTab], ["face", "Face", null, FaceTab],
  ["vision", "Vision", null, VisionTab],
  ["slam", "SLAM", null, SlamTab], ["mapedit", "Map edit", null, MapEditTab],
  ["calibrate", "Calibrate", "CALIBRATE", CalibrateTab], ["health", "Health", null, HealthTab],
  ["logbook", "Logbook", null, LogbookTab], ["rc", "RC", null, RcTab],
  ["system", "System", null, SystemTab], ["version", "Version", null, VersionTab], ["ros", "ROS", null, RosTab],
];
const PAGES = Object.fromEntries([...MODES, ...TOOLS].map(([id, , , C]) => [id, C]));

/* The Launcher opens the page with ?drive=1: once the robot's software answers (it starts in Idle) it goes to Manual,
   so the gamepad drives straight away. Tried every few seconds for two minutes (the step may still be starting). */
const drive = { want: new URLSearchParams(location.search).has("drive"), until: Date.now() + 120000, last: 0 };

function App() {
  const s = useStore("status", "conn", "version");
  const [tab, setTab] = useState(() => location.hash.slice(1) || "home");
  const [menu, setMenu] = useState(false);
  useDriveLoop(s.status?.mission, tab === "manual");     // the gamepad / flight stick drives on every tab
  useEffect(() => {
    const st = s.status;
    if (!drive.want || !st || Date.now() - drive.last < 3000) return;
    if (st.mission !== "IDLE" || Date.now() > drive.until) {
      drive.want = false;
      history.replaceState(null, "", location.pathname + location.hash);
      return;
    }
    drive.last = Date.now();
    command("mission", { name: "MANUAL" }).then((r) => { if (r.ok) drive.want = false; });   // else: the next status
  }, [s.status]);
  useEffect(() => {
    const f = () => { setTab(location.hash.slice(1) || "home"); setMenu(false); };
    window.addEventListener("hashchange", f);
    return () => window.removeEventListener("hashchange", f);
  }, []);
  const st = s.status;
  const pageMismatch = st && st.version && st.version !== PAGE_VERSION;
  const fw = mismatch(s.version);
  /** @param {any} t */
  const tabBtn = ([id, label, mission]) => html`<button key=${id} class=${`tab${tab === id ? " sel" : ""}`}
    onClick=${() => { location.hash = id; }}>
    ${label}${mission && st?.mission === mission && html`<span class="live" title="This mode is running"></span>`}</button>`;
  const tool = TOOLS.find(([id]) => id === tab);
  const toolLive = TOOLS.some(([, , m]) => m && st?.mission === m);
  const Page = PAGES[tab] || HomeTab;
  return html`
    <nav class="tabbar">
      <span class="brand">omni console</span>
      ${MODES.map(tabBtn)}
      <span class="sep"></span>
      <span class="menu-wrap">
        <button class=${`tab${tool ? " sel" : ""}`} onClick=${() => setMenu(!menu)} aria-haspopup="true" aria-expanded=${menu}>
          ${tool ? `Robot: ${tool[1]}` : "Robot"} ▾${toolLive && html`<span class="live"></span>`}</button>
        ${menu && html`<div class="menu" onMouseLeave=${() => setMenu(false)}>
          ${TOOLS.map(([id, label, m]) => html`<button class=${`item${tab === id ? " sel" : ""}`} onClick=${() => { location.hash = id; }}>
            ${label}${m && st?.mission === m ? " •" : ""}</button>`)}</div>`}
      </span>
    </nav>
    ${pageMismatch && html`<${Banner} red=${true} title="Version mismatch.">The robot's console is ${st.version}, this page ${PAGE_VERSION}: reload the page (Ctrl+F5).<//>`}
    ${fw && html`<${Banner} red=${true} title="Firmware mismatch.">${fw} (Version tab)<//>`}
    <main class="page" onClick=${() => menu && setMenu(false)}><${Page} /></main>
    <${StatusBar} />`;
}

render(html`<${App} />`, /** @type {HTMLElement} */ (document.getElementById("app")));
connect();
