// @ts-check
/* omni console: one page to watch and command the robot, arranged by what the operator wants to do:
     Home   have it do something: what it's doing now, the map and its camera, the tasks, what ran (and each task's page)
     Drive  drive it by hand (the gamepad also drives from every page)
     Map    places, routes, rooms, SLAM, and the tasks that use them (Go To, Patrol, Explore)
     Arm    the fitted payload (Catch on the catch robot)
     See    cameras and vision
     Test   check it works: preflight, bring-up, calibration, the base's samples, the sim's faults
     Robot  look inside or tune it (a menu): health, logbook, face, settings, RC, system, version, ROS
   The robot lists its features and their workspaces (features.js); each page's address is its feature id (#goto,
   #tidy, ...), the same as before the workspaces, so links and ros/tools/console_smoke.py keep working. The status bar
   is always at the bottom. The robot serves this page (omni_console's supervisor, or console_web) and the page talks
   to it through rosbridge (ros.js) and, for starting and stopping its software, to the supervisor (system.js). */
import { html, render, useEffect, useState } from "../vendor/htm-preact-standalone.mjs";
import { useDriveLoop } from "./drive.js";
import { availability, features, payloadLabel, visible, WORKSPACES } from "./features.js";
import { PAGE_VERSION, command, connect, useStore } from "./ros.js";
import { StatusBar } from "./statusbar.js";
import "./system.js";
import { ArmTab } from "./tabs/arm.js";
import { BaseTab } from "./tabs/base.js";
import { BringupTab } from "./tabs/bringup.js";
import { CalibrateTab } from "./tabs/calibrate.js";
import { CallTab } from "./tabs/call.js";
import { CamerasTab } from "./tabs/cameras.js";
import { CatchTab } from "./tabs/catch.js";
import { ExploreTab } from "./tabs/explore.js";
import { FaceTab } from "./tabs/face.js";
import { FaultsTab } from "./tabs/faults.js";
import { FeatureTab } from "./tabs/feature.js";
import { FollowTab } from "./tabs/follow.js";
import { DanceTab, FetchTab, GoalieTab, MessageTab, TowerTab } from "./tabs/fun.js";
import { GotoTab } from "./tabs/goto.js";
import { HealthTab } from "./tabs/health.js";
import { HomeTab } from "./tabs/home.js";
import { LogbookTab } from "./tabs/logbook.js";
import { ManualTab } from "./tabs/manual.js";
import { MapEditTab } from "./tabs/mapedit.js";
import { PatrolTab } from "./tabs/patrol.js";
import { PreflightTab } from "./tabs/preflight.js";
import { ProgramTab } from "./tabs/program.js";
import { RcTab } from "./tabs/rc.js";
import { RosTab } from "./tabs/rostab.js";
import { SettingsTab } from "./tabs/settings.js";
import { SlamTab } from "./tabs/slam.js";
import { SystemTab } from "./tabs/system.js";
import { ThrowerTab } from "./tabs/thrower.js";
import { TidyTab } from "./tabs/tidy.js";
import { VersionTab, mismatch } from "./tabs/version.js";
import { VisionTab } from "./tabs/vision.js";
import { Banner } from "./ui.js";

/** Each feature's page (feature id -> component). A feature the robot lists without one here: FeatureTab. */
const PAGES = /** @type {Record<string, any>} */ ({
  home: HomeTab,
  tidy: TidyTab,
  fetch: FetchTab,
  tower: TowerTab,
  message: MessageTab,
  dance: DanceTab,
  goalie: GoalieTab,
  follow: FollowTab,
  program: ProgramTab,
  manual: ManualTab,
  call: CallTab,
  goto: GotoTab,
  patrol: PatrolTab,
  explore: ExploreTab,
  slam: SlamTab,
  mapedit: MapEditTab,
  arm: ArmTab,
  catch: CatchTab,
  thrower: ThrowerTab,
  cameras: CamerasTab,
  vision: VisionTab,
  preflight: PreflightTab,
  bringup: BringupTab,
  calibrate: CalibrateTab,
  base: BaseTab,
  faults: FaultsTab,
  face: FaceTab,
  health: HealthTab,
  logbook: LogbookTab,
  rc: RcTab,
  settings: SettingsTab,
  system: SystemTab,
  version: VersionTab,
  ros: RosTab,
});

/* The Launcher opens the page with ?drive=1: once the robot's software answers (it starts in Idle) it goes to Manual,
   so the gamepad drives straight away. Tried every few seconds for two minutes (the step may still be starting). */
const drive = { want: new URLSearchParams(location.search).has("drive"), until: Date.now() + 120000, last: 0 };

/** The page last opened in each workspace (a workspace's button goes back there). */
const lastIn = /** @type {Record<string, string>} */ ((() => {
  try { return JSON.parse(sessionStorage.getItem("console.last") || "{}"); } catch { return {}; }
})());

function App() {
  const s = useStore("status", "conn", "version", "schema");
  const [tab, setTab] = useState(() => location.hash.slice(1) || "home");
  const [menu, setMenu] = useState(false);
  const list = features();
  useDriveLoop(s.status?.mission, tab === "manual" || tab === "call");   // the gamepad drives on every tab
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
  const cur = list.find((x) => x.id === tab);
  const group = cur?.group || (tab === "home" ? "home" : "");
  useEffect(() => {
    if (!group) return;
    lastIn[group] = tab;
    try { sessionStorage.setItem("console.last", JSON.stringify(lastIn)); } catch { /* */ }
  }, [tab, group]);
  const pageMismatch = st && st.version && st.version !== PAGE_VERSION;
  const fw = mismatch(s.version);
  const shown = list.filter((x) => visible(x, st));
  const inGroup = (/** @type {string} */ g) => shown.filter((x) => x.group === g);
  const live = (/** @type {any} */ x) => !!x.mission && st?.mission === x.mission;
  /** @param {string} g */
  const open = (g) => {
    const pages = inGroup(g);
    const back = pages.find((x) => x.id === lastIn[g]);
    const first = back || pages.find((x) => availability(x, st).ok) || pages[0];
    if (first) location.hash = first.id;
  };
  const labels = /** @type {Record<string, string>} */ (Object.fromEntries(WORKSPACES));
  labels.payload = payloadLabel(list, st);
  const bar = WORKSPACES.filter(([g]) => g !== "robot" && inGroup(g).length);
  const robotPages = inGroup("robot");
  const sub = group && group !== "robot" ? inGroup(group) : [];
  const Page = PAGES[tab] || (cur ? FeatureTab : HomeTab);
  return html`
    <nav class="tabbar">
      <span class="brand">omni console</span>
      ${bar.map(([g]) => html`<button key=${g} class=${`tab${group === g ? " sel" : ""}`} onClick=${() => open(g)}>
        ${labels[g]}${inGroup(g).some(live) && html`<span class="live" title="A task here is running"></span>`}</button>`)}
      <span class="sep"></span>
      <span class="menu-wrap">
        <button class=${`tab${group === "robot" ? " sel" : ""}`} onClick=${() => setMenu(!menu)} aria-haspopup="true" aria-expanded=${menu}>
          ${group === "robot" && cur ? `Robot: ${cur.label}` : "Robot"} ▾</button>
        ${menu && html`<div class="menu" onMouseLeave=${() => setMenu(false)}>
          ${robotPages.map((x) => html`<button class=${`item${tab === x.id ? " sel" : ""}`} title=${x.hint}
            onClick=${() => { location.hash = x.id; }}>${x.label}</button>`)}</div>`}
      </span>
    </nav>
    ${sub.length > 1 && html`<nav class="subbar">${sub.map((x) => {
      const a = availability(x, st);
      return html`<button key=${x.id} class=${`sub${tab === x.id ? " sel" : ""}${a.ok ? "" : " off"}`}
        title=${a.ok ? x.hint : a.why} onClick=${() => { location.hash = x.id; }}>
        ${x.label}${live(x) && html`<span class="live"></span>`}</button>`;
    })}</nav>`}
    ${pageMismatch && html`<${Banner} red=${true} title="Version mismatch.">The robot's console is ${st.version}, this page ${PAGE_VERSION}: reload the page (Ctrl+F5).<//>`}
    ${fw && html`<${Banner} red=${true} title="Firmware mismatch.">${fw} (Version tab)<//>`}
    <main class="page" onClick=${() => menu && setMenu(false)}><${Page} feature=${cur} /></main>
    <${StatusBar} />`;
}

render(html`<${App} />`, /** @type {HTMLElement} */ (document.getElementById("app")));
connect();
