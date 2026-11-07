# omni_face: the robot's face, voice and ears (design.md 2, 12.2)

The 5 in 800×480 display at the robot's front (between the cameras; on the arm robot over the bin, also facing
forward) shows its status, eyes while it plays, prompts and the score in catch, its antics, and a card with the
reason when it faults. The speaker plays short sounds, songs and speech. The mic array hears voice commands. No
engineering data: that's the console page.

```
/console/status, /console/schema, /console/events, /mcu/state_10hz, /face/say, /face/hear
        │
   face_node (ROS; on the Jetson, WSL in the sim) ── moments.Brain: what to show and say, the antics, by priority
        │  TCP 9881, JSON lines both ways          ── commands.py: what a heard phrase means → /console/command
   display.py (pygame; no ROS) ── the panel full screen + the speaker + the mics (robot), a window + the PC's (sim)
        │  listen.py: Vosk (robot) or Windows' recognizer (sim), a closed grammar of the commands
        │  while face_node is away: the supervisor's /api/state ("software stopped", "starting ...")
```

**On the robot:** the Jetson boots without a desktop. `systemd/omni-face.service` starts the display at boot (SDL
straight on the panel through KMS/DRM; fallback: a bare X server with only the face). The face node runs with the
console's nodes in every step. Voice: Kokoro-82M (offline neural TTS, Apache 2.0, `pip install kokoro-onnx`, its
model in `OMNI_KOKORO_DIR` via `python -m omni_face.voice fetch`; VERIFY its speed on the robot computer's CPU), else
Piper, else espeak-ng; phrases are cached as WAVs. Play everything through the mic array's speaker output so its echo cancelling hears it. Ears: Vosk
with its small English model (`OMNI_VOSK_MODEL`, ~40 MB; `pip install vosk sounddevice`), the mic array's capture
device (`OMNI_MIC`; VERIFY its ALSA name and that Vosk keeps up on the Orin's CPU).

**In the sim:** every step that runs the console also runs the face node, which opens the window on Windows (the
same program, with Kokoro's voice if it is installed in the sim's venv, else Windows' own, and Windows' recognizer,
through the PC's speakers and microphone; `python ros/src/omni_face/omni_face/voice.py fetch` once). The Launcher's
**Robot face** button opens it, **Face demo** plays a scripted session with no robot. Keys: M mutes, L switches the
microphone off, Esc closes. `OMNI_FACE_ARGS="--listen none"` (or the Face tab's voice commands "off") keeps the
microphone off.

Windows' SAPI voice fallback can be selected with `OMNI_SAPI_VOICE` (an installed voice name). The WAV recognition
test uses Microsoft David Desktop as a fixed speech fixture: this PC's recognizer rejects Zira's synthesized
"follow me". This fixture result does not score microphone recognition of human speech.

**Its voice** (`voice.PRESETS`, the Face tab's **Voice**; `OMNI_VOICE_PRESET` until the node says): **deadpan**
(default, Nov 5, TARS-like): a calm, low, dry voice (Kokoro am_michael blended with am_fenrir), a touch slower and
lower, through a light "small speaker in a metal box" effect (band-limited, a faint metallic ring, a short reflection).
**bright** (EVE-like): af_bella with af_heart, a little higher, a faint shimmer. **cute**: the earlier af_heart raised
12%. Each persona shifts the preset a little (patrol lower and slower). The effects are numpy only (nothing to install
on the robot); `OMNI_VOICE_FX=0` turns them off, `OMNI_KOKORO_VOICE` forces one stock voice. To listen and choose:
`python -m omni_face.voice audition --play` (from `ros/src/omni_face`, the sim venv) writes every preset saying the
same lines into `~/omni_voice_audition`; `python -m omni_face.voice say "text" --preset bright --play` tries a line.
Heavier neural voices (Chatterbox, Orpheus, F5) sound richer but want a CUDA GPU; the robot computer has none.

**Its character** (`moments.py`, Nov 5: "more real, funny, cool, not so hardcoded, like TARS or EVE"): a capable
machine with a dry sense of humour. It says little and means it, understated, honest about being a robot, never mean,
now and then warm. Every line set has plain lines and wry ones (`moments.Lines`); the **Humor** setting (0-100 %,
default 75; the Face tab, or "Omni, humor sixty percent", "Omni, what's your humor setting") is the chance of a wry
one; at 0 it is plain, never volunteers a quip and declines jokes. No line repeats back to back, and lines carry what
is true now (time of day, things tidied today, metres driven, failed grabs, the battery when it's under half); a line
whose fact isn't known is skipped. Faults are always said plainly.

**What it says and shows** (`moments.py`): faults (a card, an alert, said once: priority 3, interrupts anything);
catches, misses, streaks, records, "here it comes!", the operator's Say box and answers to voice commands (2);
"throw it to me!" three times at most while waiting (after 8 s, then 30 s and 60 s apart; then only the screen asks),
mission greetings, antics' quips (one spoken quip per 5 min at most), `/face/say` as plain text (1, dropped if stale).
Moods: idle, happy, sad, focused, surprised, sleepy (2 min idle), worried (a warning, lost), alert (an error). The
eyes follow the ball in catch and the person in Follow, worked out in the screen's own direction (`facing_deg`, 0:
forward).

**Personas** (`moments.PERSONAS`, `display.PALETTES`, `voice.STYLES`): some missions change who it is. **Patrol** is
stern: red-orange eyes narrowed under angry brows on a dark red background, a hard mouth, radar sweeps (a scanner bar
across the eyes), a lower, slower voice, "Nothing gets past me", a word after each lap. **Explore** is adventurous:
green, wide eyes with big pupils, an open grin, looking around, "What's over there?". **Tidy** is a contented
worker: sunny yellow smiling eyes with blush, pink sparkles, whistling and humming, a "ta-da" for each thing in the
bin. Faults and the calm-breaking moods (sad, worried, alert, sleepy) keep their own colours.

**How it's going** (`moments.MISSION_SAY`): each mode says its progress when it's useful. Go To: "The kitchen? I know
the way. Mostly." at the start, "halfway there" (on a trip long enough for it), "final approach" under 1.5 m, then
"Arrived. Hold your applause." or "I couldn't reach the kitchen". Patrol: "checkpoint 2 of 4, all clear" (40 s apart
at most), a lap, the end. Explore: every 25 m² mapped, the way back, the area at the end. Follow: found, "slow down, I
have wheels, not legs" when the person is 2.6 m off, lost; Tag's moments. Program: "step 3 of 5" (20 s apart at most),
the end. Tidy: a word for the first thing in the bin and about half of the rest ("5 in the bin. Not that I'm counting.
I am counting."), a failed grab ("That one fought back."), one too big, and the end with the count and the misses (or
"nothing to pick up"). Each mode (Go To, Follow, Manual, Program, Catch too) has its own quips. "Be quiet" hushes all of
it.

**The Say box** (the console's Manual and Face tabs): a line typed on the operator's computer is said at once (game
priority) with an expression (happy, excited, silly, love, wink, thinking, sad, worried, sleepy), the face showing it
while it speaks. `/face/say` takes plain text or JSON `{"text", "mood", "act"}`.

**A caller's clips** (the console's Drive › Call): `/face/clip` `{"path", "caption"}` names a WAV the page server kept
(the visitor's push-to-talk); face_node sends its bytes to the displays (`"kind": "clip"`, base64, at most 2 MB) and the
display plays it as it is, at a caller's priority, its caption on the face.

**Antics** (`moments.ANTICS`; the Face tab's "Antics": off / quiet / normal / lively, and its Tricks buttons): now
and then (normal: every 2–5 min), when nothing else is going on, mostly quiet ones: it looks around, thinks, smiles,
hums, whistles, yawns, naps (Zzz, snores, "I wasn't sleeping. I was thinking."), sings a tune on "la" (Twinkle
Twinkle, Frère Jacques, Ode to Joy, its own jingle), winks, sighs, babbles in beeps or says something dry; giggles,
sneezes and hiccups are rare. Driven fast it says so ("Speed. I like it."), after a fast spin it gets dizzy (spiral
eyes). Only light ones while it drives or waits for a throw; none during a fault, a throw, the
Thrower bench mode or a calibration. The sounds and songs are synthesized (`sounds.py`): nothing to ship or license.

**Voice commands** (`commands.py`; the Face tab's "Voice commands": off / chat only / on). Its name first: "Omni,
follow me", or "Omni?", a chime, then the command within 6 s. Stop, follow me / come here, let's play tag, let's play
catch, go to the <saved place>, go home, explore, go to sleep, wake up, be quiet / you can talk, tell me a joke, sing,
dance, what's your battery, what's your name, how are you, what time is it, hello, thank you, good robot, humor
sixty percent (sets the console's Humor, 0-100), what's your humor setting. The
console's commands go through `/console/command` as if clicked (the console's checks and the MCU's interlocks
apply); "chat only" refuses the ones that drive, stop always works. While it is speaking only "stop" is taken. The
Face tab can type a command as if heard (`/face/hear`), and lists what it heard.

Tests: `python -m pytest -q ros/src/omni_face/test` (on Windows it also synthesizes "Hey omni, follow me" and
recognizes it with the sim's recognizer).
