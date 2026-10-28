// @ts-check
// Face: what the robot's screen shows and its voice says (omni_face), the Say box, its tricks, how often it does its
// antics, and the voice commands (what it heard, a typed test).
import { html, useState } from "../../vendor/htm-preact-standalone.mjs";
import { act, useStore, useTopic } from "../ros.js";
import { SayBox } from "../say.js";
import { HelpBox, SettingsForm, clock } from "../ui.js";

const TRICKS = [["sing", "Sing"], ["hum", "Hum"], ["whistle", "Whistle"], ["dance", "Dance"], ["giggle", "Giggle"],
  ["smile", "Smile"], ["wink", "Wink"], ["love", "Love"], ["yawn", "Yawn"], ["nap", "Nap"], ["sneeze", "Sneeze"],
  ["hiccup", "Hiccup"], ["babble", "Beep-boop"], ["look", "Look around"], ["dizzy", "Dizzy"], ["sigh", "Sigh"],
  ["quip", "Say something silly"]];
const EXAMPLES = ["omni follow me", "omni stop", "hey omni tell me a joke", "omni sing a song",
  "omni what's your battery", "omni go to the kitchen", "omni let's play catch", "omni be quiet"];

export function FaceTab() {
  const s = useStore("events");
  const [face, setFace] = useState(/** @type {any} */ (null));
  const [hear, setHear] = useState("");
  useTopic("/face/state", setFace);
  const heard = s.events.filter((/** @type {any} */ e) => e.source === "voice").slice(-12).reverse();
  /** @param {string} t */
  const test = (t) => { if (t.trim()) act("hear", { text: t.trim() }, true); };
  return html`<div class="cols">
    <div class="side">
      <${HelpBox} tab="face">
        The robot's face is the 5 in screen at its front (on the arm robot, over the bin); its voice is the speaker in
        the audio box. In the sim both are the face window on this PC.
        <ul>
          <li><b>Say</b>: type a line, pick a face, press Enter. It is said at once (a fault still interrupts it).
            The Manual tab has the same box: press <kbd>Enter</kbd> while driving to type, <kbd>Esc</kbd> to drive again.</li>
          <li><b>Tricks</b> play one of its antics now. On its own it does them now and then when nothing else is going
            on (<b>Antics</b> below), never during a throw or a fault.</li>
          <li><b>Voice commands</b>: say its name first, "Omni, follow me" (or "Omni?", wait for the chime, then the
            command). It stops, follows, plays catch, goes to a saved place, explores, tells jokes, sings, says its
            battery. Stop always works unless voice commands are off. In the sim it listens through this PC's
            microphone (Windows' recognizer, the mic icon on the face window; <kbd>L</kbd> there switches it off).</li>
        </ul>
      <//>
      <h3>Say</h3>
      <${SayBox} />
      <h3>Tricks</h3>
      <div class="row" style="gap: 4px">
        ${TRICKS.map(([v, l]) => html`<button class="btn small" onClick=${() => act("say", { act: v }, true)}>${l}</button>`)}
      </div>
      <${SettingsForm} tab="face" title="Face and voice" />
      <h3>Test a voice command</h3>
      <div class="card">
        <div class="row">
          <input type="text" class="grow" placeholder="omni follow me" value=${hear}
            onInput=${(/** @type {any} */ e) => setHear(e.currentTarget.value)}
            onKeyDown=${(/** @type {any} */ e) => { if (e.key === "Enter") { test(hear); setHear(""); } }} />
          <button class="btn" onClick=${() => { test(hear); setHear(""); }}>Hear</button>
        </div>
        <div class="row" style="margin-top: 6px; gap: 4px">
          ${EXAMPLES.map((x) => html`<button class="btn small" onClick=${() => test(x)}>${x}</button>`)}
        </div>
        <div class="hint dim" style="font-size: 12px; margin-top: 4px">Taken as if the microphones heard it: it
          moves the robot like the spoken command would.</div>
      </div>
    </div>
    <div class="main full">
      <div class="card">
        <h2>On the screen now</h2>
        ${face ? html`<div class="kv">
          <span class="k">mood</span><span>${face.mood}${face.act ? html` · <b>${face.act}</b>` : ""}</span>
          <span class="k">top line</span><span>${face.label || "—"}${face.score ? `, ${face.score.player} ${face.score.caught}/${face.score.throws}` : ""}</span>
          <span class="k">prompt</span><span>${face.prompt || face.sub || "—"}</span>
          <span class="k">heard</span><span>${face.heard ? `“${face.heard}”` : face.listening ? "listening..." : "—"}</span>
          <span class="k">card</span><span class=${face.card ? (face.card.level === "error" ? "err" : "warn") : "dim"}>${face.card ? face.card.title : "none"}</span>
        </div>` : html`<p class="dim">The face node isn't running (it runs with the console's nodes in every step).</p>`}
      </div>
      <div class="card">
        <h2>What it heard</h2>
        ${heard.length ? html`<ul class="list">${heard.map((/** @type {any} */ e) => html`<li>
          <span class="dim mono">${clock(e.t)}</span><span class=${e.level === "warn" ? "warn" : ""}>${e.text}</span></li>`)}</ul>`
          : html`<p class="dim">Nothing yet. Say "Omni, hello", or test a command on the left.</p>`}
      </div>
    </div>
  </div>`;
}
