// @ts-check
/* Catch: start the game, the throw radar (each throw's speed, height and aim, replayed), who's throwing, and the
   leaderboard (omni_console/throws.py). The pass game is a setting: the robot throws each ball to the other player. */
import { html, useEffect, useState } from "../../vendor/htm-preact-standalone.mjs";
import { MapView } from "../mapview.js";
import { act, command, useStore, useTopic } from "../ros.js";
import { ThrowView, kmh } from "../throwview.js";
import { Banner, HelpBox, SettingsForm } from "../ui.js";

export function CatchTab() {
  const s = useStore("status", "schema");
  const st = s.status;
  const c = st?.catch;
  const can = !!st?.caps?.catch;
  const playing = st?.mission === "CATCH";
  const [last, setLast] = useState(/** @type {any} */ (null));
  const [stats, setStats] = useState(/** @type {any} */ (null));
  const [player, setPlayer] = useState("");
  const [view, setView] = useState("radar");
  const refresh = () => command("stats").then((r) => { if (r.ok) setStats(r.data); });
  useEffect(() => { refresh(); }, []);
  useTopic("/console/throw", (d) => { setLast(d); refresh(); });
  const who = stats?.player || s.schema?.player || "";
  return html`<div class="cols">
    <div class="side">
      <${HelpBox} tab="catch">
        <ol>
          <li>Stand facing the robot about 6 to 7 m away (the robot refines where you are from each throw). In the sim's
            house ('Robot (all modes)') a thrower steps up on Start: straight ahead if there's room, else where there
            is (the robot turns to face them), as far as the room allows (3.1 m at least; it says so if there isn't).</li>
          <li>Type your name under <b>Who's throwing</b> so your throws count for you.</li>
          <li>Press <b>Start</b>. The robot arms and waits ("Waiting for a throw").</li>
          <li>Throw the foam ball toward it. It moves under the ball, catches it in the funnel and, with <b>Throw
            back</b> on, throws it back.</li>
        </ol>
        <b>The pass game</b>: set <b>Throw it to</b> to "the other player"; two people stand apart and take turns, and
        the robot throws each ball to the one who didn't throw it. The radar shows how fast and how high each throw
        was and how close it came to the robot's spot. <b>Pause</b> disarms the motors.
      <//>
      ${!can && html`<${Banner} title="The catch code isn't running.">In the sim: Home, 'Switch to the catch world'. On the robot it starts with it.<//>`}
      ${playing
        ? html`<button class="btn big danger" onClick=${() => act("mission", { name: "IDLE" })}>Pause</button>`
        : html`<button class="btn go big" disabled=${!can} onClick=${() => act("mission", { name: "CATCH" })}>Start</button>`}
      <h3>Who's throwing</h3>
      <div class="row"><input type="text" class="grow" placeholder=${who || "Your name"} value=${player}
        onInput=${(/** @type {any} */ e) => setPlayer(e.currentTarget.value)}
        onKeyDown=${(/** @type {any} */ e) => { if (e.key === "Enter" && player.trim()) act("player_set", { name: player }).then((r) => { if (r.ok) { setStats(r.data); setPlayer(""); } }); }} />
        <button class="btn small primary" disabled=${!player.trim()}
          onClick=${() => act("player_set", { name: player }).then((r) => { if (r.ok) { setStats(r.data); setPlayer(""); } })}>Set</button></div>
      <div class="dim" style="margin-top: 4px">Now: <b>${who || "–"}</b>${(stats?.board || []).length > 1 ? html` · <span>switch:</span>
        ${stats.board.filter((/** @type {any} */ p) => p.player !== who).slice(0, 4).map((/** @type {any} */ p) => html` <a href="#catch"
          onClick=${(/** @type {any} */ e) => { e.preventDefault(); act("player_set", { name: p.player }).then((r) => r.ok && setStats(r.data)); }}>${p.player}</a>`)}` : ""}</div>
      <h3>Game</h3>
      <div class="card">
        <div style="font-size: 16px; margin-bottom: 8px"><b>${playing ? (c ? c.text : "Starting...") : "Not playing"}</b></div>
        <div class="stats">
          <div class="stat"><div class="v">${c ? c.tosses : "–"}</div><div class="k">throws</div></div>
          <div class="stat"><div class="v">${c ? c.catches : "–"}</div><div class="k">catches</div></div>
          <div class="stat"><div class="v">${c && c.rate != null ? `${c.rate}%` : "–"}</div><div class="k">rate</div></div>
        </div>
      </div>
      <${SettingsForm} tab="catch" />
    </div>
    <div class="main full">
      <div class="row" style="margin-bottom: 8px">
        ${[["radar", "Throw radar"], ["board", "Leaderboard"], ["field", "Field"]].map(([k, l]) => html`<button
          class=${`btn small${view === k ? " primary" : ""}`} onClick=${() => setView(k)}>${l}</button>`)}</div>
      ${view === "radar" && html`<div>
        <div class="radar">
          <div class="stat"><div class="v big">${kmh(last?.speed)}</div><div class="k">throw speed</div></div>
          <div class="stat"><div class="v big">${last?.apex != null ? `${last.apex.toFixed(1)} m` : "–"}</div><div class="k">how high</div></div>
          <div class="stat"><div class="v big">${last?.aim != null ? `${Math.round(last.aim * 100)} cm` : "–"}</div><div class="k">from the robot's spot</div></div>
          <div class="stat"><div class=${`v big ${last ? (last.caught ? "ok" : "warn") : ""}`}>${last ? (last.caught ? "Caught!" : "Missed") : "–"}</div>
            <div class="k">${last ? `${last.player}, throw ${last.toss}` : "the last throw"}</div></div>
        </div>
        <div class="card"><${ThrowView} rec=${last} /></div>
        ${stats?.recent?.length > 0 && html`<div class="card"><h2>Recent throws</h2><table class="t"><thead><tr>
          <th>who</th><th>speed</th><th>high</th><th>aim</th><th></th></tr></thead><tbody>
          ${stats.recent.slice().reverse().map((/** @type {any} */ r) => html`<tr><td>${r.player}</td><td>${kmh(r.speed)}</td>
            <td>${r.apex != null ? `${r.apex.toFixed(1)} m` : "–"}</td><td>${r.aim != null ? `${Math.round(r.aim * 100)} cm` : "–"}</td>
            <td class=${r.caught ? "ok" : "dim"}>${r.caught ? "caught" : "missed"}</td></tr>`)}</tbody></table></div>`}
      </div>`}
      ${view === "board" && html`<div class="card"><h2>Leaderboard</h2>
        ${stats?.board?.length ? html`<table class="t"><thead><tr><th>#</th><th>player</th><th>caught</th><th>throws</th>
          <th>best streak</th><th>fastest</th><th>highest</th><th>average aim</th></tr></thead><tbody>
          ${stats.board.map((/** @type {any} */ p, /** @type {number} */ i) => html`<tr class=${p.player === who ? "me" : ""}><td>${i + 1}</td><td><b>${p.player}</b></td>
            <td>${p.caught}</td><td>${p.throws}</td><td>${p.best_streak}</td><td>${p.best_kmh != null ? `${p.best_kmh} km/h` : "–"}</td>
            <td>${p.best_apex != null ? `${p.best_apex} m` : "–"}</td><td>${p.aim != null ? `${Math.round(p.aim * 100)} cm` : "–"}</td></tr>`)}
          </tbody></table>` : html`<p class="dim">No throws yet.</p>`}
        <p class="dim" style="font-size: 12px">Ranked by throws the robot caught, then aim (closer to the robot's spot is better).</p></div>`}
      ${view === "field" && html`<div style="position: relative; height: 70vh; border-radius: 8px; overflow: hidden">
        <${MapView} catchView=${true} hint="The catch field (the robot's odometry frame: it starts at 0, 0 facing you)"
          empty=${can ? "Waiting for the robot's position..." : "Start the catch game to see the field."} /></div>`}
    </div>
  </div>`;
}
