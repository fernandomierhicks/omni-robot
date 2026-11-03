# RealSense D435 characterization launcher (README.md): one button per step, in order. Each step opens its own
# console window (several ask for a tape distance there) and stays open to show its result; results\ keeps
# the JSON, the plots and REPORT.md.

Add-Type -AssemblyName System.Windows.Forms
Add-Type -AssemblyName System.Drawing
[System.Windows.Forms.Application]::EnableVisualStyles()

$Here = Split-Path -Parent $MyInvocation.MyCommand.Path
$Py = Join-Path $env:USERPROFILE ".venvs\realsense\Scripts\python.exe"
$Results = Join-Path $Here "results"

function Start-Step([string]$Title, [string]$PyArgs) {
    if (-not (Test-Path $Py)) {
        [System.Windows.Forms.MessageBox]::Show("Missing $Py`r`nSee README.md, Setup.", "RealSense") | Out-Null
        return
    }
    $cmd = "title $Title && cd /d `"$Here`" && `"$Py`" -u $PyArgs & echo. & echo (done - close this window) & pause > nul"
    Start-Process -FilePath "C:\Windows\System32\cmd.exe" -ArgumentList "/c `"$cmd`"" -WorkingDirectory $Here
}

$form = New-Object System.Windows.Forms.Form
$form.Text = "RealSense D435 characterization"
$form.Size = New-Object System.Drawing.Size(720, 640)
$form.StartPosition = "CenterScreen"
$form.Font = New-Object System.Drawing.Font("Segoe UI", 9.5)
$tt = New-Object System.Windows.Forms.ToolTip

$root = New-Object System.Windows.Forms.FlowLayoutPanel
$root.Dock = "Fill"; $root.FlowDirection = "TopDown"; $root.WrapContents = $false; $root.AutoScroll = $true
$root.Padding = New-Object System.Windows.Forms.Padding(8)
$form.Controls.Add($root)

function Add-Group([string]$Title, [string]$Hint) {
    $g = New-Object System.Windows.Forms.GroupBox
    $g.Text = $Title; $g.AutoSize = $true; $g.MinimumSize = New-Object System.Drawing.Size(680, 0)
    $f = New-Object System.Windows.Forms.FlowLayoutPanel
    $f.Dock = "Fill"; $f.AutoSize = $true; $f.WrapContents = $true; $f.MaximumSize = New-Object System.Drawing.Size(670, 0)
    $g.Controls.Add($f)
    if ($Hint) {
        $l = New-Object System.Windows.Forms.Label
        $l.Text = $Hint; $l.AutoSize = $true; $l.ForeColor = [System.Drawing.Color]::DimGray
        $l.MaximumSize = New-Object System.Drawing.Size(660, 0)
        $f.Controls.Add($l); $f.SetFlowBreak($l, $true)
    }
    $root.Controls.Add($g)
    return $f
}

function Add-Button($Panel, [string]$Text, [scriptblock]$Do, [string]$Tip = "", [int]$W = 150) {
    $b = New-Object System.Windows.Forms.Button
    $b.Text = $Text; $b.Width = $W; $b.Height = 34
    $b.Add_Click($Do)
    if ($Tip) { $tt.SetToolTip($b, $Tip) }
    $Panel.Controls.Add($b)
    return $b
}

function Add-Text($Panel, [string]$Label, [string]$Value, [int]$W = 50, [string]$Tip = "") {
    $l = New-Object System.Windows.Forms.Label
    $l.Text = $Label; $l.AutoSize = $true; $l.Margin = New-Object System.Windows.Forms.Padding(8, 9, 0, 0)
    $Panel.Controls.Add($l)
    $t = New-Object System.Windows.Forms.TextBox
    $t.Text = $Value; $t.Width = $W; $t.Margin = New-Object System.Windows.Forms.Padding(3, 6, 3, 3)
    if ($Tip) { $tt.SetToolTip($t, $Tip) }
    $Panel.Controls.Add($t)
    return $t
}

$p0 = Add-Group "0. Setup" "Once: the camera on a USB 3 port (rear of the PC, its own cable). 'Enable metadata' (admin) gives frames the camera's own timestamps; replug after it."
[void](Add-Button $p0 "Camera info" { Start-Step "rs_info" "rs_info.py" } "Device, firmware, USB link, modes, baseline, timestamps; the static checks against design.md")
[void](Add-Button $p0 "Enable metadata (admin)" {
    Start-Process powershell.exe -Verb RunAs -ArgumentList "-NoProfile -ExecutionPolicy Bypass -NoExit -File `"$Here\enable_metadata.ps1`""
} "Registry change Windows needs for hardware timestamps; then unplug and replug the camera" 180)
[void](Add-Button $p0 "Run the tests" { Start-Step "tests" "-m pytest -q" } "The maths on synthetic data (no camera)")

$p1 = Add-Group "1. Latency and frame rate (the catch: <= 60 ms capture to detection)" ""
$tbMode = Add-Text $p1 "mode:" "" 170 "empty = the default set; e.g. ir:848x480@90"
$tbExp = Add-Text $p1 "exposure us:" "" 50 "empty = auto; 1000-2000 freezes a thrown ball"
[void](Add-Button $p1 "Timing per mode" {
    $a = "rs_timing.py"
    if ($tbMode.Text) { $a += " --mode $($tbMode.Text)" }
    if ($tbExp.Text) { $a += " --exposure-us $($tbExp.Text)" }
    Start-Step "rs_timing" $a
} "fps, drops, jitter, capture -> host latency, CPU")
[void](Add-Button $p1 "Flash test (screen)" { Start-Step "rs_flash" "rs_flash_latency.py" } "Point the camera at this monitor: photons -> host, split into the display's and the camera's parts")

$p2 = Add-Group "2. Depth" "Wall: square to a flat matte wall at 0.5, 1, 2, 3, 4, 6 m; SPACE captures, then type the tape distance. Floor: the camera looking down ~40 deg like the arm's camera bar."
[void](Add-Button $p2 "Wall: depth noise" { Start-Step "rs_plane" "rs_depth_plane.py" })
$tbMin = Add-Text $p2 "min object mm:" "15" 40
[void](Add-Button $p2 "Floor: pose + objects" { Start-Step "rs_floor" "rs_depth_plane.py --floor --min-height-mm $($tbMin.Text)" })

$p3 = Add-Group "3. The ball" "A ball on a stand at 1-7 m: click it, SPACE, type the distance to its front surface. Throws: the camera still, throw toward it; the first second is the background."
$tbBall = Add-Text $p3 "ball mm:" "74" 40 "the design's foam ball is 74 mm"
[void](Add-Button $p3 "Ball at range" { Start-Step "rs_ball" "rs_ball.py --ball-mm $($tbBall.Text)" })
$tbSec = Add-Text $p3 "seconds:" "20" 40
[void](Add-Button $p3 "Record throws" {
    $a = "rs_throw.py record --preview --analyze --seconds $($tbSec.Text) --ball-mm $($tbBall.Text)"
    if ($tbExp.Text) { $a += " --exposure-us $($tbExp.Text)" }
    Start-Step "rs_throw" $a
} "~110 MB/s into ~\realsense_recordings (not Dropbox), analyzed when it stops")
[void](Add-Button $p3 "Analyze last recording" { Start-Step "rs_throw_analyze" "rs_throw.py analyze --latest --ball-mm $($tbBall.Text)" })

$p4 = Add-Group "4. Calibration drift (decides the catch: 0.04 deg -> 97%, 0.3 deg -> 78%)" "Camera clamped, facing a wall or the floor 1.5-3 m away, started cold. Repeat on other days with the same setup."
$tbMin2 = Add-Text $p4 "minutes:" "60" 40
$tbLabel = Add-Text $p4 "label:" "" 160 "e.g. cold start, after knocks, day 2"
[void](Add-Button $p4 "Drift run" {
    $a = "rs_drift.py --minutes $($tbMin2.Text)"
    if ($tbLabel.Text) { $a += " --label `"$($tbLabel.Text)`"" }
    Start-Step "rs_drift" $a
})

$p5 = Add-Group "5. Report" "Everything measured so far against design.md's requirements, and the values to carry into database\cameras.csv and the sim."
[void](Add-Button $p5 "Write + open report" {
    Start-Step "rs_report" "rs_report.py"
    Start-Sleep -Seconds 4
    $r = Join-Path $Results "REPORT.md"
    if (Test-Path $r) { Start-Process $r }
})
[void](Add-Button $p5 "Open results folder" { New-Item -ItemType Directory -Force $Results | Out-Null; Start-Process explorer.exe $Results })

# The camera on the Pi (rpi.md): rs_pi.sh and the ball view (characterization\ball_view) through Git Bash.
$Repo = Split-Path -Parent (Split-Path -Parent $Here)
$Bash = "C:\Program Files\Git\bin\bash.exe"
function Start-Pi([string]$Title, [string]$Cmd) {
    if (-not (Test-Path $Bash)) {
        [System.Windows.Forms.MessageBox]::Show("Missing $Bash (Git for Windows)", "RealSense") | Out-Null
        return
    }
    $c = "title $Title && cd /d `"$Repo`" && `"$Bash`" -c `"$Cmd`" & echo. & echo (done - close this window) & pause > nul"
    Start-Process -FilePath "C:\Windows\System32\cmd.exe" -ArgumentList "/c `"$c`"" -WorkingDirectory $Repo
}

$p6 = Add-Group "6. On the Pi (omni-cam, rpi.md)" "The D435 and the Arducam are on the Pi. The ball views run there and open in your browser: sliders for the colour window, blob size and exposure, the 3D position. Ctrl-C in their window stops them; only one view at a time (one process per camera)."
[void](Add-Button $p6 "Pi sees the camera?" { Start-Pi "rs_pi usb" "bash characterization/realsense/rs_pi.sh usb" } "lsusb / link speed (5000M = USB 3) / dmesg / power, no librealsense")
$tbPiSec = Add-Text $p6 "seconds:" "10" 40 "per mode; 60 for a sustained run"
[void](Add-Button $p6 "Pi check" { Start-Pi "rs_pi check" "bash characterization/realsense/rs_pi.sh check --seconds $($tbPiSec.Text)" } "link, camera, Pi temperature, then fps / lost / latency / CPU per mode")
[void](Add-Button $p6 "Ball view: Arducam" { Start-Pi "ball view picam" "bash characterization/ball_view/bv.sh picam" } "Position from the ball's size")
[void](Add-Button $p6 "Ball view: RealSense" { Start-Pi "ball view realsense" "bash characterization/ball_view/bv.sh realsense" } "Colour blob + the depth map: position from depth, compared with the size")
[void](Add-Button $p6 "Stop ball view" { Start-Pi "ball view stop" "bash characterization/ball_view/bv.sh stop" } "If a view was left running on the Pi")

[void]$form.ShowDialog()
