why
characterize lidar

Voice brain (Nov 23, built; first real start to do, attended): double-click ros/tools/voice_brain.cmd and leave the window open (Claude Code on the subscription, no API key). Check: the first time, approve any permission it still asks for with "always"; then from the Face tab's Hear try "take a picture of me", "go dance in the kitchen", "spin three times, go forward 2 m, come back to me" (it should ask, then wait for "go"), "do a figure of eight" (it writes a skill; http://127.0.0.1:9884/skills/page). Leave it an hour to see the Monitor re-arm. Still to do: invented skills on the console, the photo on the screen, a real wave gesture, the handshake (after the arm load calibration). ros/src/omni_face/README.md "Its voice brain".

Optional, only for the faster API brain (OMNI_VOICE_BRAIN=api): Robot chat key (Nov 23): create an Anthropic API key at https://platform.claude.com (Settings > API keys; add a little credit; the chat uses Claude Haiku 5.5, about $0.10 / $0.50 per million tokens in/out, so a reply costs well under a cent). On the main PC set it once in a new terminal: `setx ANTHROPIC_API_KEY "sk-ant-..."`, then restart the display / voice server (they start from the console). Check: `C:\Users\ferna\.venvs\omni-voice\Scripts\python.exe -c "from omni_face.chat import Chat; print(Chat().reply('hello'))"` from ros/src/omni_face should answer in character, not "my thinking module is offline".

CubeMars settings: read the motor's servo-mode acceleration and ERPM limit in the CubeMars tool, so the cap's source is confirmed.
Bom, DC-DC, presents christmas. 





RGBD light without killing control loop. 

Door opening on the ROS stack (arm_brain `door` verb, cabinet in the ROS sim, console + omni_mcp): handoff in simulation/v11 sil/studies/results/door_open_ros_handoff_2026-11-21.md

CAD:
Arm pan datum block (proposal, Nov 20): findings and the CAD plan in simulation/v11 sil/studies/results/arm_datums_2026-11-20.md


v15 cad "8. Bumper: five strategies are compared in the README. I recommend a light-gray TPU skirt clamped in a PLA ring, laid out so switches can be added later. Not drawn."


"9. Service stand: four Ø24 pads with Ø8.4 holes through the deck, placed where nothing hangs or drops. A stand holds them 185 mm over the bench, so a pod or the battery lowered onto the bench slides out under the bumper.
