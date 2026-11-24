#!/usr/bin/env bash
# The voice brain session's Monitor (voice_brain_prompt.md): one line per sentence heard by the robot (voice_desk.py).
exec "/c/Users/ferna/.venvs/omni-mcp/Scripts/python.exe" -u "$(dirname "$0")/voice_desk.py" follow
