@echo off
rem Omni's voice brain: a Claude Code session kept open on this PC (ros/tools/omni_mcp/voice_brain_prompt.md).
rem Double-click it and leave the window open; close it to stop. Uses your Claude subscription's Claude Code login.
rem The voice service (voice_server.py, OMNI_VOICE_BRAIN=claude_code, the default) hands it what the robot hears.
setlocal
cd /d "%~dp0\..\.."
set "CLAUDE=claude"
where claude >nul 2>nul
if errorlevel 1 (
  for /d %%D in ("%USERPROFILE%\.vscode\extensions\anthropic.claude-code-*-win32-x64") do (
    if exist "%%D\resources\native-binary\claude.exe" set "CLAUDE=%%D\resources\native-binary\claude.exe"
  )
)
title Omni voice brain
"%CLAUDE%" "Read ros/tools/omni_mcp/voice_brain_prompt.md and follow it." -n "Omni voice brain" ^
  --mcp-config ros\tools\omni_mcp\voice_mcp.json --strict-mcp-config ^
  --allowedTools "mcp__omni_voice" "Monitor" "Bash(bash ros/tools/omni_mcp/voice_follow.sh)" "Read(ros/**)"
endlocal
