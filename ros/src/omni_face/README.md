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
model in `OMNI_KOKORO_DIR` via `python -m omni_face.voice fetch`; VERIFY its speed on the Orin's CPU), else Piper,
else espeak-ng; the voice is af_heart raised 12% in pitch (cute: a small speaker), each persona in its own style;
phrases are cached as WAVs. Play everything through the mic array's speaker output so its echo cancelling hears it. Ears: Vosk
with its small English model (`OMNI_VOSK_MODEL`, ~40 MB; `pip install vosk sounddevice`), the mic array's capture
device (`OMNI_MIC`; VERIFY its ALSA name and that Vosk keeps up on the Orin's CPU).

**In the sim:** every step that runs the console also runs the face node, which opens the window on Windows (the
same program, with Kokoro's voice if it is installed in the sim's venv, else Windows' own, and Windows' recognizer,
through the PC's speakers and microphone; `python ros/src/omni_face/omni_face/voice.py fetch` once). The Launcher's
**Robot face** button opens it, **Face demo** plays a scripted session with no robot. Keys: M mutes, L switches the
microphone off, Esc closes. `OMNI_FACE_ARGS="--listen none"` (or the Face tab's voice commands "off") keeps the
microphone off.

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
green, wide eyes with big pupils, an open grin, looking around, "Ooh, what's over there?". **Tidy** is a joyful
worker: sunny yellow smiling eyes with blush, pink sparkles, whistling and humming, a "ta-da" and a cheer for each
thing in the bin. Faults and the calm-breaking moods (sad, worried, alert, sleepy) keep their own colours.

**How it's going** (`moments.MISSION_SAY`): each mode says its progress when it's useful, picked at random from a few
funny lines. Go To: "Yes, my master!" / "Onward to the kitchen!" at the start, "halfway there" (on a trip long enough
for it), "almost there, I can see the goal!" under 1.5 m, then "mission accomplished" or "I can't get there". Patrol:
"checkpoint 2 of 4, all clear" (40 s apart at most), "lap 1 done, all secure", the end. Explore: every 25 m² mapped,
the way back, the area at the end. Follow: "got you, lead the way!", "wait for me!" when the person is 2.6 m off,
"where are you going? Don't leave me!" when it loses them; Tag: "ready or not, here I come!", "tag, you're it!".
Program: "step 3 of 5" (20 s apart at most), the end. Tidy: a cheer and a count per thing in the bin, the total at the
end. Each mode (Go To, Follow, Manual, Program, Catch too) has its own quips. "Be quiet" hushes all of it.

**The Say box** (the console's Manual and Face tabs): a line typed on the operator's computer is said at once (game
priority) with an expression (happy, excited, silly, love, wink, thinking, sad, worried, sleepy), the face showing it
while it speaks. `/face/say` takes plain text or JSON `{"text", "mood", "act"}`.

**Antics** (`moments.ANTICS`; the Face tab's "Antics": off / quiet / normal / lively, and its Tricks buttons): now
and then (normal: every 90–240 s), when nothing else is going on, it yawns, naps (Zzz, snores, "huh? I'm awake!"), sings a tune on "la"
(Twinkle Twinkle, Frère Jacques, Ode to Joy, its own jingle), hums, whistles, giggles, winks, sneezes, hiccups,
babbles in beeps, looks around, sighs or says something silly. Driven fast it says "wheee!", after a fast spin it
gets dizzy (spiral eyes). Only light ones while it drives or waits for a throw; none during a fault, a throw, the
Thrower bench mode or a calibration. The sounds and songs are synthesized (`sounds.py`): nothing to ship or license.

**Voice commands** (`commands.py`; the Face tab's "Voice commands": off / chat only / on). Its name first: "Omni,
follow me", or "Omni?", a chime, then the command within 6 s. Stop, follow me / come here, let's play tag, let's play
catch, go to the <saved place>, go home, explore, go to sleep, wake up, be quiet / you can talk, tell me a joke, sing,
dance, what's your battery, what's your name, how are you, what time is it, hello, thank you, good robot. The
console's commands go through `/console/command` as if clicked (the console's checks and the MCU's interlocks
apply); "chat only" refuses the ones that drive, stop always works. While it is speaking only "stop" is taken. The
Face tab can type a command as if heard (`/face/hear`), and lists what it heard.

Tests: `python -m pytest -q ros/src/omni_face/test` (on Windows it also synthesizes "Hey omni, follow me" and
recognizes it with the sim's recognizer).
