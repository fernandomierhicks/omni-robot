// @ts-check
/* The fun missions' pages (omni_console/console_fun.py): Fetch, Messenger, Block tower, Dance party, Goalie. Each
   starts its mission with the robot's own verb, shows how it's going and stops it; their settings come from the robot's
   schema like every page's. */
import { html, useRef, useState } from "../../vendor/htm-preact-standalone.mjs";
import { act, useStore } from "../ros.js";
import { Banner, HelpBox, SettingsForm } from "../ui.js";
import { WRIST } from "./arm.js";
import { mainCamera } from "./home.js";
import { streamUrl } from "./vision.js";

const COLOURS = [["red", "#e53935"], ["orange", "#fb8c00"], ["yellow", "#fdd835"], ["green", "#43a047"],
  ["blue", "#1e88e5"], ["purple", "#8e24aa"], ["pink", "#ec407a"], ["any", "transparent"]];
const SHAPES = [["any", "any shape"], ["square", "square"], ["round", "round"], ["long", "long"]];

/** The running mission's line, its Stop. @param {{mission: string, idle: string}} p */
function Running({ mission, idle }) {
  const s = useStore("status");
  const st = s.status;
  const on = st?.mission === mission;
  return html`<div class="card">
    <h2>${on ? st.task : idle} ${on && html`<span class="badge green" style="margin-left: 6px">running</span>`}</h2>
    ${on && st.progress != null && html`<div class="bar big"><div style=${`width: ${Math.round(st.progress * 100)}%`}></div></div>`}
    ${on && html`<div class="row" style="margin-top: 8px"><button class="btn danger" onClick=${() => act("stop")}>Stop</button></div>`}
  </div>`;
}

/** The camera that shows the task best. @param {{topic?: string}} p */
function Camera({ topic }) {
  const s = useStore("status");
  const v = s.status?.vision;
  const t = topic && (v?.images || []).includes(topic) ? topic : v?.images?.length ? mainCamera(v.images) : null;
  if (!v?.video_port || !t) return html`<div class="card"><p class="dim">No camera stream (web_video_server isn't running).</p></div>`;
  return html`<div class="card" style="max-width: 680px"><img style="width: 100%; border-radius: 6px; background: var(--map-bg)"
    alt=${t} src=${streamUrl(location.hostname || "localhost", v.video_port, t)} /><div class="dim mono" style="font-size: 11px">${t}</div></div>`;
}

export function FetchTab() {
  const s = useStore("status");
  const [color, setColor] = useState("red");
  const [shape, setShape] = useState("any");
  const st = s.status;
  const on = st?.mission === "FETCH";
  const what = [color !== "any" ? color : "", shape !== "any" ? shape : ""].filter(Boolean).join(" ") || "anything";
  return html`<div class="cols">
    <div class="side">
      <${HelpBox} tab="fetch">
        Pick a colour and a shape and press <b>Fetch</b>. It looks for it with the D435 (turning a quarter at a time,
        four looks at most, and driving closer to a patch of the colour), picks it up and keeps it in the gripper, comes to you (the nearest person it sees; it looks
        round for you if nobody's in view) and puts it down in front of you. Things wider than 6.5 cm are too big.
        By voice: "Omni, bring me the red block" when the voice commands are on.
      <//>
      ${!st?.arm && html`<${Banner} title="No arm.">Fetch needs the arm robot (omni_arm's arm_node).<//>`}
      <h3>Colour</h3>
      <div class="row" style="flex-wrap: wrap; gap: 6px">${COLOURS.map(([c, hex]) => html`<button
        class=${`btn small${color === c ? " on" : ""}`} aria-pressed=${color === c} onClick=${() => setColor(c)}>
        <span class="swatch" style=${`background: ${hex}`}></span>${c}</button>`)}</div>
      <h3>Shape</h3>
      <div class="seg">${SHAPES.map(([k, label]) => html`<button class=${`btn${shape === k ? " on" : ""}`}
        aria-pressed=${shape === k} onClick=${() => setShape(k)}>${label}</button>`)}</div>
      <div class="row" style="margin-top: 12px">
        ${on ? html`<button class="btn big danger" onClick=${() => act("stop")}>Stop</button>`
          : html`<button class="btn go big" disabled=${!st?.arm} onClick=${() => act("fetch", { color: color === "any" ? null : color, shape })}>
            Fetch ${what}</button>`}
      </div>
      <${SettingsForm} tab="fetch" />
    </div>
    <div class="main full"><${Running} mission="FETCH" idle="Not fetching" /><${Camera} /></div>
  </div>`;
}

const PRESETS = ["Dinner's ready!", "Time for bed.", "Come to the living room, please.", "Someone's at the door."];

export function MessageTab() {
  const s = useStore("status", "schema");
  const [text, setText] = useState("");
  const rooms = Object.keys(s.schema?.rooms || {});
  const [pick, setPick] = useState(/** @type {string[] | null} */ (null));
  const chosen = pick ?? rooms;
  const st = s.status;
  const on = st?.mission === "MESSAGE";
  return html`<div class="cols">
    <div class="side">
      <${HelpBox} tab="message">
        Type a message and press <b>Send</b>. The robot looks round where it is first, then goes room by room (the map's
        rooms, nearest first) and looks round each one until it sees someone, goes up to them and says it. Nobody in any
        room: it tells you so. Rooms are drawn in Map › Map edit.
      <//>
      <h3>Message</h3>
      <textarea rows="3" style="width: 100%" placeholder="What should it say?" value=${text}
        onInput=${(/** @type {any} */ e) => setText(e.currentTarget.value)}></textarea>
      <div class="row" style="flex-wrap: wrap; gap: 6px; margin-top: 6px">${PRESETS.map((p) => html`<button class="btn small"
        onClick=${() => setText(p)}>${p}</button>`)}</div>
      <h3>Rooms to try</h3>
      ${rooms.length ? rooms.map((r) => html`<label class="row" style="padding: 2px 0"><input type="checkbox" checked=${chosen.includes(r)}
        onChange=${(/** @type {any} */ e) => setPick(e.currentTarget.checked ? [...chosen, r] : chosen.filter((x) => x !== r))} /> ${r}</label>`)
        : html`<p class="dim">No rooms on this map: it only looks round where it is. Draw them in <a href="#mapedit">Map edit</a>.</p>`}
      <div class="row" style="margin-top: 12px">
        ${on ? html`<button class="btn big danger" onClick=${() => act("stop")}>Stop</button>`
          : html`<button class="btn go big" disabled=${!text.trim() || !st} onClick=${() => act("message", { text, rooms: chosen })}>Send</button>`}
      </div>
    </div>
    <div class="main full"><${Running} mission="MESSAGE" idle="No message on its way" /><${Camera} /></div>
  </div>`;
}

export function TowerTab() {
  const s = useStore("status");
  const st = s.status;
  const tw = st?.arm?.tidy?.tower;
  const on = st?.mission === "TOWER";
  return html`<div class="cols">
    <div class="side">
      <${HelpBox} tab="tower">
        Scatter some building blocks (cubes, 3-5 cm) on the floor near the robot and press <b>Build</b>. The first block
        it finds stays where it is and becomes the base; it brings each other one over and puts it on top, checking the
        tower with the D435 before each block. It stops at the height set below, or when it finds no more blocks.
      <//>
      ${!st?.arm && html`<${Banner} title="No arm.">Block tower needs the arm robot (omni_arm's arm_node).<//>`}
      ${on ? html`<button class="btn big danger" onClick=${() => act("stop")}>Stop</button>`
        : html`<button class="btn go big" disabled=${!st?.arm} onClick=${() => act("tower", {})}>Build a tower</button>`}
      <${SettingsForm} tab="tower" />
    </div>
    <div class="main full">
      <${Running} mission="TOWER" idle="No tower building" />
      ${tw && html`<div class="card"><div class="stats" style="grid-template-columns: repeat(2, 1fr)">
        <div class="stat"><div class="v">${tw.n}</div><div class="k">blocks high</div></div>
        <div class="stat"><div class="v">${Math.round(tw.top * 1000)}</div><div class="k">mm tall</div></div></div></div>`}
      <${Camera} topic=${WRIST} />
    </div>
  </div>`;
}

export function DanceTab() {
  const s = useStore("status", "schema");
  const st = s.status;
  const on = st?.mission === "DANCE";
  const taps = useRef(/** @type {number[]} */ ([]));
  const [bpm, setBpm] = useState(/** @type {number | null} */ (null));
  const tap = () => {
    const now = performance.now();
    taps.current = taps.current.filter((t) => now - t < 3000).concat([now]);
    const t = taps.current;
    if (t.length >= 4) {
      const b = Math.round(60000 / ((t[t.length - 1] - t[0]) / (t.length - 1)) / 5) * 5;
      const v = Math.max(60, Math.min(160, b));
      setBpm(v);
      act("set", { key: "dance.bpm", value: v }, true);
    }
  };
  return html`<div class="cols">
    <div class="side">
      <${HelpBox} tab="dance">
        Put some music on, set the tempo (or tap it: press <b>Tap</b> on the beat four times or more) and press
        <b>Dance</b>. It slides, circles, spins and shimmies on the beat and sings along; every move ends where it
        started, so it never strays more than 0.6 m. Keep a clear circle round it.
      <//>
      ${on ? html`<button class="btn big danger" onClick=${() => act("stop")}>Stop</button>`
        : html`<button class="btn go big" disabled=${!st} onClick=${() => act("dance", {})}>Dance</button>`}
      <div class="row" style="margin-top: 8px"><button class="btn" onClick=${tap}>Tap the beat</button>
        <span class="dim">${bpm ? `${bpm} bpm` : ""}</span></div>
      <${SettingsForm} tab="dance" />
    </div>
    <div class="main full"><${Running} mission="DANCE" idle="Not dancing" /></div>
  </div>`;
}

export function GoalieTab() {
  const s = useStore("status");
  const st = s.status;
  const g = st?.goalie;
  const on = st?.mission === "GOALIE";
  return html`<div class="cols">
    <div class="side">
      <${HelpBox} tab="goalie">
        Mark two posts (the goal width below, centred on the robot), stand 3 m or more away and press <b>Play</b>. Roll
        a ball at the goal: the D435 tracks it and the robot slides sideways to block it, facing you all the time.
        It keeps score. The ball: a play ball, 15-25 cm, in the colour ball_track is set to (red by default:
        omni_bringup's robot.yaml). Keep it to rolling: it can't stop a ball in the air.
      <//>
      ${on ? html`<button class="btn big danger" onClick=${() => act("stop")}>Stop</button>`
        : html`<button class="btn go big" disabled=${!st} onClick=${() => act("goalie", {})}>Play</button>`}
      <${SettingsForm} tab="goalie" />
    </div>
    <div class="main full">
      <${Running} mission="GOALIE" idle="Not in goal" />
      ${g && html`<div class="card"><div class="stats" style="grid-template-columns: repeat(3, 1fr)">
        <div class="stat"><div class="v ok">${g.saves}</div><div class="k">saved</div></div>
        <div class="stat"><div class="v err">${g.goals}</div><div class="k">let in</div></div>
        <div class="stat"><div class="v">${g.shots}</div><div class="k">shots</div></div></div>
        <div class="dim" style="margin-top: 6px">${g.ball ? "Ball in sight" : "No ball in sight"} · last: ${g.state}</div></div>`}
      <${Camera} />
    </div>
  </div>`;
}
