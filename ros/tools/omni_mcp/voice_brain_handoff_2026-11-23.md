# Voice brain: handoff (Nov 23)

Status: **built Nov 23, steps 1-4 (Fernando: "defaults ok"), tested with a scripted model only (no API key on the PC
yet); step 5 (handshake) not started.** Decisions taken: restricted Python skills; wait for "go" on invented or
multi-step plans (enforced in code); "me" = the nearest person seen; Claude Opus 5.5 invents; voice plus the Face
tab's typed Hear. Where it lives now: `toolkit.py`, `robot_client.py`, `skillbox.py`, `skill_check.py`,
`skill_child.py`, `inventor.py`, `skills/` here; `omni_face/brain.py`, `voice_server.py`, `face_node.py`,
`commands.py`; this README's "Skills" and omni_face README "Its voice brain". Not done from the plan: invented skills
on the console (the voice service serves `/skills/page` instead), the photo on the screen, a real wave gesture.
**Changed the same day (Fernando): the live brain is a Claude Code session kept open on the PC** (no API key; every
unknown sentence goes to it), replacing recommendation 1's "not Claude Code" below: `../voice_brain.cmd`,
`voice_desk.py`, `voice_tools.py`, `server.py --voice`. The API brain stays selectable.

Fernando (Nov 23): the voice should be able to run commands. Not just the canned ones
but "random things that are not too technically challenging" ("take a picture of me", "go dance over there", "spin
three turns, go forward 2 m, come back to me"). When there is no ready action, the model should work one out (about
30 s), write it as a script and run it. Start a fresh session from this file.

## What exists today
- Voice: `omni_face` `commands.py` matches single commands (stop, follow, tag, explore, go to <place>, home, tidy,
  fetch <colour> <shape>, tower, party, goalie, catch, humor, sleep). Any other sentence goes to `chat.py` on the PC
  (Claude Haiku 5.5, low effort, structured JSON `{say, emotion, intensity, remember}`). It can only talk.
- AI toolkit: `omni_mcp` (`server.py`) gives Claude Code / Codex the robot's actions and senses: status, events,
  capabilities, look (camera image), places, goto, move (step programs: forward/back/left/right, turn, spin, wiggle,
  wait, repeat), find (objects by colour/shape), pick, fetch, message, tower, dance (groove/party/robot), goalie,
  task/cancel/stop, say, express, arm_pose (stow/ready/high/drop/look), map, scan, people, tags, floor labels, tf,
  topics. Safety lives in the console: AI lease, AI speed ceilings, "a person always wins" (HUMAN_VERBS, RC switch,
  HUMAN_HOLD), collision monitor for driving, arm stowed for driving.
- Feelings: `omni_face` affect/expression; body gestures only when the body is free (never during a task).

## Recommended architecture
1. **One brain, on the main PC, in the voice service.** A sentence that is not a single command goes to Claude
   Haiku 5.5 with **tool use**, and the tools are the `omni_mcp` actions themselves. Factor `server.py`'s robot
   client and tool table into a module that both the MCP server and the voice brain import: Claude Code and the
   voice see one toolkit, written once. Haiku runs the loop: plan, call, read the result, next step, done. It ends
   with a spoken line and a feeling, as the chat does now. Plain conversation stays a reply with no tools.
   - Not Claude Code as the live voice brain: it isn't an always-on background service, it's slow to start and it
     is tied to an open editor. Claude Code stays the developer/operator client through `omni_mcp`.
   - Credentials: `ANTHROPIC_API_KEY` on the PC (todo.md). One key covers Haiku and the inventor below.
2. **Ready actions ("skills").** A skill is a named, described routine built from the toolkit's actions; the brain sees
   the list as tools. Hand-written skills for the common asks (list below), stored in a skills folder.
3. **Inventing a skill.** When no tool or skill fits and the ask is feasible, Haiku calls `invent_skill(request)`.
   That asks Claude Opus 5.5 (higher effort) to write a new skill. The face shows thinking and it says "Let me work
   that out." The new skill is then checked, run and saved for next time, listed on the console where Fernando can
   rename, keep or delete it.
   - **The skill format is the main decision.** Recommended: **restricted Python.** The skill file can only call a
     `robot` object exposing the toolkit (move, goto, people, look, say, express, arm_pose, wait, ...) and runs in
     its own process with a time limit. It gets no imports beyond a whitelist, no file or network access, and no ROS. Every
     action still goes through the console's AI verbs, so the robot enforces the speed caps, the lease, the collision
     monitor, "a person always wins" and stop; nothing is trusted from the script. Alternative: generate the
     existing Program / dance step lists (JSON). That is safer still but can't loop on what it senses ("turn until you
     see me").
   - First run of a newly invented skill: say what it will do and wait for "go" (or run it in the sim first).
4. **Confirmation.** Plans that move: say the plan in one sentence and wait for "go" (or start after 3 s unless
   told "no": decide). Single ready actions start at once. "Stop" / "Omni, stop" always wins (existing).
5. **Feelings.** Unchanged rule: a running skill owns the body (no gestures). Face and voice react to its steps. The
   brain may call `express` before or after. The skill's result feeds the affect (done → proud, failed → frustrated).

## Proposed skill list
Already in the toolkit (expose to the voice brain as is): go to a place; follow me; play tag; explore; patrol a route;
tidy up; fetch the <colour> <thing>; build a tower; take a message to someone; dance party (groove, party, robot);
goalie; stop; moves (forward/back/sideways X m, turn X°, spin N, wiggle, wait, repeat); arm poses; say; show a
feeling; what do you see (look → the model describes); find things on the floor; who's around (people).

New, small, scripted on those actions:
| Ask | Skill |
| --- | --- |
| "Come here" / "come to me" | nearest confirmed person → drive to ~1 m in front, face them |
| "Look at me" / "face me" | turn to the nearest person |
| "Take a picture of me" | face the person, arm `look` / D435 colour frame, save to a photos folder (outside Dropbox source), show it on the screen and in the console logbook |
| "Go dance over there" / "dance in the kitchen" | goto (place or "there" = where the person points? start with places and relative moves) + dance |
| "Turn around" / "back up" / "spin N times" | move |
| "Celebrate" | wiggle + flourish + chirp (the gestures) |
| "Wave" / "nod" / "shake your head" | arm gestures (wave is new) |
| "Show me around" | patrol of saved places with a line at each |
| "Is anyone in the kitchen?" / "go check the door" | goto + people/look + report |
| "Where's the red block?" | find + say where |
| "Remind me in 10 minutes to ..." | a timer that says it (and comes to find the person: later) |
| "Watch the door" | park facing it, report people (time limit) |
| "Look sad / happy" | express |
| "Shake my hand" / "high five" | new arm action: offer the hand, wait for a tug (force sensing, `arm-force-awareness`), shake gently. Needs the load calibration: sim first |

## Open decisions for Fernando
1. Skill format: restricted Python (recommended) or step lists only.
2. Confirm moving plans: wait for "go" (recommended for invented or multi-step plans) or auto-start after 3 s.
3. "Me" = the nearest person the camera confirms (recommended for now); voice direction / recognition later.
4. Inventor model: Claude Opus 5.5 (recommended) or Haiku for everything.
5. Scope: voice plus the Face tab's typed "Hear" (recommended; easy to test without a mic).

## Build order (each verified in the v11 sim through the console, as `ros/tools/ai_smoke.py` does)
1. Shared toolkit module (MCP server and voice brain import it); the MCP tests still pass.
2. Voice brain: Haiku tool loop in the voice service, routed from `chat`; confirmation; spoken result + feeling;
   tests with a fake client (as test_emotion.py), then one real call.
3. Scripted skills from the table (not the handshake).
4. `invent_skill` + skill runner sandbox + console list; first run asks.
5. Handshake / high five after the arm load calibration.

Keep: one implementation per robot node (AGENTS.md), robot packages never import `omni_sim`, protocol untouched, a
person always wins, `ai-control-toolkit` rule (primitives and senses in the robot, intelligence in the model).
