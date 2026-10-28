# omni-robot launcher, every step (design.md 28): buttons for the sims, the ROS 2 stack in WSL, the studies, the scored
# runs, Foxglove and the joystick. Launcher.cmd opens the sim's controller (launcher.ps1); this one is its
# More > "All steps and studies". Every button starts its job without a console; its output goes to logs\<job>.log,
# shown in the panel at the bottom.

Add-Type -AssemblyName System.Windows.Forms
Add-Type -AssemblyName System.Drawing
[System.Windows.Forms.Application]::EnableVisualStyles()

$Repo = Split-Path -Parent $MyInvocation.MyCommand.Path
$Py = Join-Path $env:USERPROFILE ".venvs\omni-robot-sim\Scripts\python.exe"
$Wsl = "C:\Windows\System32\wsl.exe"
$Distro = "Ubuntu-24.04"
$OmniSh = "/mnt/c/Dropbox/Personal Projects/Robotics/omni-robot/ros/omni.sh"
$Logs = Join-Path $Repo "logs"
New-Item -ItemType Directory -Force $Logs | Out-Null
$script:Jobs = @{}          # name -> System.Diagnostics.Process
$script:Current = $null     # the log shown

function Start-Job2([string]$Name, [string]$CmdLine) {
    # cmd /c "<command> > log 2>&1", hidden; the log is shown below
    $log = Join-Path $Logs "$Name.log"
    "== $(Get-Date -Format 'HH:mm:ss') $CmdLine`r`n" | Set-Content -Encoding utf8 $log
    $psi = New-Object System.Diagnostics.ProcessStartInfo
    $psi.FileName = "C:\Windows\System32\cmd.exe"
    $psi.Arguments = "/c `"$CmdLine >> `"$log`" 2>&1`""
    $psi.WorkingDirectory = $Repo
    $psi.CreateNoWindow = $true
    $psi.UseShellExecute = $false
    $p = [System.Diagnostics.Process]::Start($psi)
    $script:Jobs[$Name] = $p
    $script:Current = $Name
    Update-Jobs
}

function Start-Omni([string]$Name, [string]$Step, [string]$Extra = "") {
    $opts = "viewer:=$(($cbViewer.Checked).ToString().ToLower()) foxglove:=$(($cbFox.Checked).ToString().ToLower()) " +
            "rviz:=$(($cbRviz.Checked).ToString().ToLower()) fast:=$(($cbFast.Checked).ToString().ToLower()) " +
            "map:=$($tbMap.Text) firmware:=$(($cbFw.Checked).ToString().ToLower()) kid:=$(($cbKid.Checked).ToString().ToLower()) " +
            "console:=$(($cbConsole.Checked).ToString().ToLower())"
    if ($Step -in "sim", "map", "localize") { $opts += " joy:=$(($cbJoy.Checked).ToString().ToLower())" }
    if ($Step -in "robot", "arm") { $opts += " person:=$(($cbPerson.Checked).ToString().ToLower())" }
    Start-Job2 $Name "`"$Wsl`" -d $Distro -- bash `"$OmniSh`" $Step $opts $Extra"
}

function Start-Py([string]$Name, [string]$Dir, [string]$PyArgs) {
    Start-Job2 $Name "cd /d `"$Repo\$Dir`" && `"$Py`" -u $PyArgs"
}

function Update-Jobs {
    $items = @()
    foreach ($k in @($script:Jobs.Keys)) {
        $p = $script:Jobs[$k]
        $state = if ($p.HasExited) { "done" } else { "running" }
        $items += "$k ($state)"
    }
    $lbJobs.Items.Clear()
    foreach ($i in ($items | Sort-Object)) { [void]$lbJobs.Items.Add($i) }
}

function Show-Log {
    if (-not $script:Current) { return }
    $log = Join-Path $Logs "$($script:Current).log"
    if (-not (Test-Path $log)) { return }
    try {
        $fs = [System.IO.File]::Open($log, 'Open', 'Read', 'ReadWrite')
        $sr = New-Object System.IO.StreamReader($fs)
        $text = $sr.ReadToEnd(); $sr.Close()
    } catch { return }
    $text = [regex]::Replace($text, "\x1b\[[0-9;]*m", "")
    $lines = $text -split "`r?`n"
    $tail = ($lines | Select-Object -Last 60) -join "`r`n"
    if ($tbLog.Text -ne $tail) {
        $tbLog.Text = $tail
        $tbLog.SelectionStart = $tbLog.Text.Length
        $tbLog.ScrollToCaret()
    }
    $lblLog.Text = "Log: logs\$($script:Current).log"
}

function Stop-Ros {
    Start-Job2 "stop" "`"$Wsl`" -d $Distro -- bash `"$OmniSh`" stop"
}

# -- the window ---------------------------------------------------------------------------------------------
$form = New-Object System.Windows.Forms.Form
$form.Text = "omni-robot launcher: all steps and studies"
$form.Size = New-Object System.Drawing.Size(980, 980)
$form.StartPosition = "CenterScreen"
$form.Font = New-Object System.Drawing.Font("Segoe UI", 9.5)

$root = New-Object System.Windows.Forms.FlowLayoutPanel
$root.Dock = "Top"; $root.Height = 670; $root.FlowDirection = "TopDown"; $root.WrapContents = $false
$root.AutoScroll = $true; $root.Padding = New-Object System.Windows.Forms.Padding(8)
$form.Controls.Add($root)

function Add-Group([string]$Title, [string]$Hint) {
    $g = New-Object System.Windows.Forms.GroupBox
    $g.Text = $Title; $g.Width = 940; $g.AutoSize = $true; $g.MinimumSize = New-Object System.Drawing.Size(930, 0); $g.Padding = New-Object System.Windows.Forms.Padding(6)
    $f = New-Object System.Windows.Forms.FlowLayoutPanel
    $f.Dock = "Fill"; $f.AutoSize = $true; $f.WrapContents = $true; $f.MaximumSize = New-Object System.Drawing.Size(925, 0)
    $g.Controls.Add($f)
    if ($Hint) {
        $l = New-Object System.Windows.Forms.Label
        $l.Text = $Hint; $l.AutoSize = $true; $l.ForeColor = [System.Drawing.Color]::DimGray
        $l.MaximumSize = New-Object System.Drawing.Size(915, 0); $l.Margin = New-Object System.Windows.Forms.Padding(3, 0, 3, 4)
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
    $script:lastButton = $b
    return $b
}

function Add-Check($Panel, [string]$Text, [bool]$On, [string]$Tip = "") {
    $c = New-Object System.Windows.Forms.CheckBox
    $c.Text = $Text; $c.Checked = $On; $c.AutoSize = $true; $c.Margin = New-Object System.Windows.Forms.Padding(6, 8, 6, 3)
    if ($Tip) { $tt.SetToolTip($c, $Tip) }
    $Panel.Controls.Add($c)
    return $c
}

function Add-Label($Panel, [string]$Text) {
    $l = New-Object System.Windows.Forms.Label
    $l.Text = $Text; $l.AutoSize = $true; $l.Margin = New-Object System.Windows.Forms.Padding(8, 9, 0, 0)
    $Panel.Controls.Add($l)
}

function Add-Text($Panel, [string]$Label, [string]$Value, [int]$W = 40, [string]$Tip = "") {
    Add-Label $Panel $Label
    $t = New-Object System.Windows.Forms.TextBox
    $t.Text = $Value; $t.Width = $W; $t.Margin = New-Object System.Windows.Forms.Padding(3, 6, 3, 3)
    if ($Tip) { $tt.SetToolTip($t, $Tip) }
    $Panel.Controls.Add($t)
    return $t
}

function Add-Combo($Panel, [string]$Label, [string[]]$Items, [int]$W = 110, [string]$Tip = "") {
    Add-Label $Panel $Label
    $c = New-Object System.Windows.Forms.ComboBox
    $c.DropDownStyle = "DropDownList"; $c.Width = $W; $c.Margin = New-Object System.Windows.Forms.Padding(3, 6, 3, 3)
    foreach ($i in $Items) { [void]$c.Items.Add($i) }
    $c.SelectedIndex = 0
    if ($Tip) { $tt.SetToolTip($c, $Tip) }
    $Panel.Controls.Add($c)
    return $c
}

$tt = New-Object System.Windows.Forms.ToolTip

# 1. the ROS stack
$p1 = Add-Group "1. ROS 2 stack in WSL: the robot's software driving the MuJoCo sim" `
    "Start here: 'Console' opens the operator page, which starts and switches everything (modes, the catch world, scored runs) like the robot will at boot. The step buttons below still work on their own (then 'Open console')."
$cbViewer = Add-Check $p1 "MuJoCo 3D view" $true "MuJoCo's own window of the running sim (follows the robot; red dots = the lidar scan)"
$cbFox = Add-Check $p1 "Foxglove bridge" $true "ws://localhost:8765 for the Foxglove app"
$cbConsole = Add-Check $p1 "Operator console" $true "The robot's own web page (modes, map, driving, status bar): rosbridge + omni_console. Open it with 'Open console' below"
$cbJoy = Add-Check $p1 "Joystick teleop" $true "teleop_twist_joy in ROS; also click 'Joystick -> ROS' below"
$cbRviz = Add-Check $p1 "RViz" $false "RViz in a WSLg window"
$cbFast = Add-Check $p1 "Fast (~real time)" $true "firmware at 1 kHz, lidar in 2 ms batches: ~0.95-1.0x real time (off: v10's published settings, ~0.6-0.7x)"
$cbFw = Add-Check $p1 "Firmware in the loop" $true "v11: the Teensy's C++ firmware runs inside the sim; the ROS side talks to it through mcu_link (off: v10's Python firmware)"
$cbKid = Add-Check $p1 "Kid mode" $false "The firmware's kid profile: 0.8 m/s, gentle accelerations, lobs only"
$cbPerson = Add-Check $p1 "Person walking" $false "A person walks round the house, for the console's Follow tab (Console, 'Robot (all modes)' and 'Arm robot')"
$lm = New-Object System.Windows.Forms.Label; $lm.Text = "map:"; $lm.AutoSize = $true; $lm.Margin = New-Object System.Windows.Forms.Padding(12, 9, 0, 0); $p1.Controls.Add($lm)
$tbMap = New-Object System.Windows.Forms.TextBox; $tbMap.Text = "house"; $tbMap.Width = 90; $tbMap.Margin = New-Object System.Windows.Forms.Padding(3, 6, 3, 3); $p1.Controls.Add($tbMap)
$p1.SetFlowBreak($tbMap, $true)
# the console as the launcher (design.md 2): its supervisor serves the page and starts / stops the steps from it
$cbStart = Add-Combo $p1 "console starts:" @("Robot (all modes)", "Arm robot: tidy up") 150 "The step the console starts with: the catch robot (every mode, Catch in the house) or the arm robot (Tidy). Switch any time in the page (Robot > System)"
$bCon = Add-Button $p1 "Console (start here)" {
    $opts = "viewer:=$(($cbViewer.Checked).ToString().ToLower()) foxglove:=$(($cbFox.Checked).ToString().ToLower()) " +
            "fast:=$(($cbFast.Checked).ToString().ToLower()) firmware:=$(($cbFw.Checked).ToString().ToLower()) " +
            "kid:=$(($cbKid.Checked).ToString().ToLower()) person:=$(($cbPerson.Checked).ToString().ToLower()) map:=$($tbMap.Text)"
    $step = if ($cbStart.SelectedIndex -eq 1) { "arm" } else { "robot" }
    Start-Job2 "ros_console" "`"$Wsl`" -d $Distro -- bash `"$OmniSh`" console --autostart $step $opts"
    $script:OpenTab = if ($step -eq "arm") { "tidy" } else { "home" }
    $script:OpenWhenUp = 120           # the page opens once port 8080 answers (after the sync and build)
} "The operator console as the launcher: starts the step picked at the left and opens the page (the Tidy tab for the arm robot); every mode, the catch world, the scored runs and the logs are then in the page (Home and Robot > System). The options above are its defaults" 170
$bCon.BackColor = [System.Drawing.Color]::PaleGreen
[void](Add-Button $p1 "Stop console" { Start-Job2 "stop_console" "`"$Wsl`" -d $Distro -- bash `"$OmniSh`" stop-console" } "Stops the console's page server and the step it runs")
$p1.SetFlowBreak($script:lastButton, $true)
[void](Add-Button $p1 "Drive (sim only)" { Start-Omni "ros_drive" "sim" } "The bridge and its wheel odometry; no localization")
[void](Add-Button $p1 "Map the house" { Start-Omni "ros_map" "map" } "slam_toolbox mapping: drive every room, then 'Save map'")
[void](Add-Button $p1 "Save map" { Start-Job2 "ros_save_map" "`"$Wsl`" -d $Distro -- bash `"$OmniSh`" save-map $($tbMap.Text)" } "While 'Map the house' runs: writes ros\maps\<map>")
[void](Add-Button $p1 "Localize on map" { Start-Omni "ros_localize" "localize" } "slam_toolbox localization on ros\maps\<map>")
$bRobot = Add-Button $p1 "Robot (all modes)" { Start-Omni "ros_robot" "robot" } "Everything at once for the operator console: Nav2, SLAM run by the console (new map / save / load in its SLAM tab), Manual, Go To, Explore, Patrol, Thrower, Catch (a thrower steps up where the room allows). Then 'Open console'" 160
$bRobot.BackColor = [System.Drawing.Color]::Honeydew
[void](Add-Button $p1 "Arm robot: tidy up" { Start-Omni "ros_arm" "arm" } "design.md 11: the arm robot (an SO-101 on the nose instead of the catch module) in the house with toys on the floor; everything 'Robot' does plus the console's Tidy tab. Then 'Open console' -> Tidy" 150)
[void](Add-Button $p1 "Navigate (Nav2)" { Start-Omni "ros_nav" "nav" } "Localization + Nav2: send goals from the console's Go To tab (or Foxglove / RViz)")
[void](Add-Button $p1 "Play catch" { Start-Omni "ros_catch" "catch" "dist:=$($tbDist.Text)" } "v11: the catch game on ROS: the thrower throws, the robot's own code (perception, behavior, MCU link, firmware) catches and throws back")
$ld = New-Object System.Windows.Forms.Label; $ld.Text = "thrower m:"; $ld.AutoSize = $true; $ld.Margin = New-Object System.Windows.Forms.Padding(6, 9, 0, 0); $p1.Controls.Add($ld)
$tbDist = New-Object System.Windows.Forms.TextBox; $tbDist.Text = "6.5"; $tbDist.Width = 40; $tbDist.Margin = New-Object System.Windows.Forms.Padding(3, 6, 3, 3); $p1.Controls.Add($tbDist)
$bStop = Add-Button $p1 "STOP ROS" { Stop-Ros } "Stops every ROS step this launcher (or omni.sh) started"
$bStop.BackColor = [System.Drawing.Color]::MistyRose

# 2. views and input
$p2 = Add-Group "2. Views and input" `
    "Console: the robot's operator page (modes, map, driving, status bar) while a step runs. Foxglove (debugging): Open connection -> Foxglove WebSocket -> ws://localhost:8765. First time: Layout -> Import from file -> ros\foxglove\omni_layout.json."
$bConsole = Add-Button $p2 "Open console" { Start-Process "http://localhost:8080" } "The operator console in your browser, served by the running step ('Operator console' ticked). A tab per mode, each with how-to help"
$bConsole.BackColor = [System.Drawing.Color]::Honeydew
[void](Add-Button $p2 "Open Foxglove" {
    $url = "foxglove://open?ds=foxglove-websocket&ds.url=ws%3A%2F%2Flocalhost%3A8765"
    $app = (Test-Path "Registry::HKEY_CLASSES_ROOT\foxglove") -or (Test-Path "HKCU:\Software\Classes\foxglove")
    if ($app) { Start-Process $url }
    else { Start-Process "https://app.foxglove.dev/view?ds=foxglove-websocket&ds.url=ws%3A%2F%2Flocalhost%3A8765" }
} "The Foxglove desktop app if installed, else the web app (free account)")
[void](Add-Button $p2 "MuJoCo 3D view" { Start-Py "mujoco_view" "ros\tools" "mujoco_view.py" } "Reopens MuJoCo's window of the running ROS sim (a step opens it by itself with 'MuJoCo 3D view' ticked; this is for after you closed it). It waits for the bridge, then follows the robot; drag to orbit")
[void](Add-Button $p2 "Joystick -> ROS" { Start-Py "joystick" "ros\tools" "joystick_sender.py" } "Logitech Extreme 3D into ROS (3Dconnexion ignored). Hold the thumb button to drive; trigger = 2 m/s")
[void](Add-Button $p2 "Stop joystick" { if ($script:Jobs["joystick"] -and -not $script:Jobs["joystick"].HasExited) { $script:Jobs["joystick"].Kill() }; Update-Jobs })
$bFace = Add-Button $p2 "Robot face" { Start-Py "face" "ros\tools" "face_view.py" } "The robot's 5 in face and speaker as a window with sound; it hears voice commands on this PC's microphone ('Omni, follow me'; any running step opens it by itself, this reopens it). M mutes, L mic off, Esc closes"
$bFace.BackColor = [System.Drawing.Color]::Honeydew
[void](Add-Button $p2 "Face demo" { Start-Py "face_demo" "ros\tools" "face_view.py --demo" } "The face and voice on a scripted session (catch, a fault, follow, driving fast and a spin, antics, low battery): no ROS needed")
[void](Add-Button $p2 "Foxglove layout file" { Start-Process explorer.exe "/select,`"$Repo\ros\foxglove\omni_layout.json`"" } "Shows the layout to import once")

# 3. scored runs
$p3 = Add-Group "3. Scored ROS runs" "They drive themselves, write simulation\v11 sil\studies\results, then stop."
[void](Add-Button $p3 "Auto-map (tour)" { Start-Omni "ros_auto_map" "auto-map" } "Maps the house along v9's tour and saves ros\maps\<map>")
[void](Add-Button $p3 "Score the map" { Start-Job2 "ros_score_map" "`"$Wsl`" -d $Distro -- bash `"$OmniSh`" score-map $($tbMap.Text)" })
[void](Add-Button $p3 "Localization tour" { Start-Omni "ros_tour" "tour" } "slam_toolbox + EKF vs the truth and v9's estimators")
[void](Add-Button $p3 "Nav2 goals" { Start-Omni "ros_goals" "goals" } "8 goals through the doors and onto the rugs")
[void](Add-Button $p3 "Soak test (2 h)" { Start-Job2 "soak" "`"$Wsl`" -d $Distro -- bash `"$OmniSh`" soak --hours 2" } "With 'Robot (all modes)' or the console running: the robot patrols the house for 2 hours; watches for freezes, crashes, memory growth and localization drift. Results in ~/omni_soak (WSL)")
[void](Add-Button $p3 "Report" { Start-Py "report" "simulation\v11 sil\studies" "ros_report.py" } "The tables in design.md 28")
[void](Add-Button $p3 "Catch score (24)" { Start-Omni "ros_catch_score" "catch-score" "dist:=$($tbDist.Text)" } "v11: 24 throws through the whole ROS stack with the firmware in the loop; writes ros_catch.json")
[void](Add-Button $p3 "Catch: SIL vs sim" { Start-Py "sil_catch" "simulation\v11 sil\studies" "sil_catch.py both" } "v11: the 216-throw grid two ways, the sim's own game and the firmware + robot code (~3 min)")
[void](Add-Button $p3 "Catch report" { Start-Py "sil_catch_report" "simulation\v11 sil\studies" "sil_catch.py report" } "The tables in design.md 30")

# 4. the catch game on Windows, any variation (v11, design.md 30-31)
$pv = Add-Group "4. Catch game on Windows: pick the variation (v11: C++ firmware + the robot's code, one process)" `
    "Scene: the netted arena or a furnished house. Cameras: sparse = the sim's pixel lists (fast, real time); rendered = MuJoCo renders through a camera sensor model (~0.06x real time). Acquisition: how the detector starts a track (1 = the largest blob, 40 = the moving one among 40)."
$cbScene = Add-Combo $pv "scene:" @("house", "arena") 80 "house: a furnished great room around the same 12 x 6 m field (decoration only, the same physics)"
$cbFrames = Add-Combo $pv "cameras:" @("rendered", "sparse") 90 "What the robot's cameras deliver"
$cbAcq = Add-Combo $pv "acquisition:" @("code default", "40", "1") 100 "DetectorConfig.acquire_candidates"
$tbVDist = Add-Text $pv "thrower m:" "6.5" 40 "Thrower's hand to the robot's home"
$tbApex = Add-Text $pv "apex m:" "0.75" 40 "Apex above the hand (lowest of the range)"
$tbApexMax = Add-Text $pv "to:" "2" 40 "Highest apex (throws are drawn between)"
$tbTosses = Add-Text $pv "throws:" "12" 40
$tbSeed = Add-Text $pv "seed:" "9000" 50 "Same seed = the same throws and cameras"
$cbView = Add-Check $pv "MuJoCo window" $true "MuJoCo's window; with rendered cameras it shows the robot's left camera at the bottom left (red: the detector's blob, blue: the true ball)"
$cbRet = Add-Check $pv "Return throws" $false "The robot throws each caught ball back"
$pv.SetFlowBreak($cbRet, $true)
function Get-VariantArgs {
    $a = "--scene $($cbScene.Text) --frames $($cbFrames.Text)"
    if ($cbAcq.Text -ne "code default") { $a += " --acquire $($cbAcq.Text)" }
    return $a
}
[void](Add-Button $pv "Play catch (these options)" {
    $a = "sil_game.py $(Get-VariantArgs) --dist $($tbVDist.Text) --apex $($tbApex.Text) --apex-max $($tbApexMax.Text) --tosses $($tbTosses.Text) --seed $($tbSeed.Text)"
    if ($cbView.Checked) { $a += " --view" }
    if ($cbRet.Checked) { $a += " --returns" }
    Start-Py "catch_$($cbScene.Text)_$($cbFrames.Text)" "simulation\v11 sil" $a
} "The game with the options above; the log shows each throw, then the detector against the truth" 190)
$tbSeeds = Add-Text $pv "study seeds:" "1" 30 "Each seed = 72 throws of the 6-7 m grid (sil_catch.py's seeds); rendered: ~10 min per seed on 12 workers"
[void](Add-Button $pv "Study: run the grid" {
    $w = if ($cbFrames.Text -eq "rendered") { " --workers 12" } else { "" }
    Start-Py "rendered_catch_$($cbFrames.Text)" "simulation\v11 sil\studies" "rendered_catch.py run $(Get-VariantArgs) --seeds $($tbSeeds.Text)$w"
} "studies\rendered_catch.py: the 6-7 m grid with the scene / cameras / acquisition above (design.md 31); adds to results\rendered_catch.json" 170)
[void](Add-Button $pv "Study report" { Start-Py "rendered_catch_report" "simulation\v11 sil\studies" "rendered_catch.py report" } "Every variation run so far: catch rate, why balls were missed, detection, centre error, false blobs, render time")
[void](Add-Button $pv "Camera pictures" { Start-Py "rendered_pictures" "simulation\v11 sil\studies" "rendered_catch.py pictures" } "Rendered frames from both cameras mid-flight with the detector's blob, and the house overview, into simulation\v11 sil\renders")
[void](Add-Button $pv "House overview" { Start-Py "house_overview" "simulation\v11 sil" "house_scene.py overview" } "A cutaway picture of the house: renders\v11_house_overview.png")
[void](Add-Button $pv "Fetch house textures" { Start-Py "house_fetch" "simulation\v11 sil" "house_scene.py fetch" } "Once per PC: the CC0 photo textures from Poly Haven into assets\house (without them the house uses flat stand-ins)")
[void](Add-Button $pv "Renders folder" { Start-Process explorer.exe "`"$Repo\simulation\v11 sil\renders`"" })

# 5. Windows MuJoCo sims
$p4 = Add-Group "5. Other MuJoCo sims on Windows (no ROS)" ""
[void](Add-Button $p4 "Catch: firmware + robot code" { Start-Py "sil_game" "simulation\v11 sil" "sil_game.py --view --returns --apex 0.75 --apex-max 2 --tosses 30" } "v11 in one process: the C++ firmware and the Jetson's code, MuJoCo's window at real time (the arena, sparse cameras)" 190)
[void](Add-Button $p4 "Catch: rendered cameras, house" { Start-Py "sil_game_rendered" "simulation\v11 sil" "sil_game.py --view --frames rendered --scene house --apex 0.75 --apex-max 2 --tosses 12" } "design.md 31: the robot's code sees only rendered images of a furnished house (inset: its left camera, red the detector's blob, blue the true ball). About 0.06x real time" 210)
[void](Add-Button $p4 "House: v7 + sensors" { Start-Py "house_v9" "simulation\v9 sensors" "house.py --base v7 --sensors --camera top" } "Drive the v7 base with the fused estimate live (keyboard, gamepad or the stick)" 170)
[void](Add-Button $p4 "House: v8 swerve" { Start-Py "house_v8" "simulation\v8 swerve" "house.py" } "" 170)
[void](Add-Button $p4 "House: v7 (v7 sim)" { Start-Py "house_v7" "simulation\v7 home floor" "house.py" } "" 170)
[void](Add-Button $p4 "Catch demo (v2)" { Start-Py "catch_v2" "simulation\v2 detailed physics" "run.py" } "" 170)
[void](Add-Button $p4 "Floor watch (v6)" { Start-Py "floor_v6" "simulation\v6 shallow throw" "watch_floor.py" } "" 170)
[void](Add-Button $p4 "Arm (SO-101) on the base" { Start-Py "arm_view" "simulation\v11 sil" "arm_view.py --payload 0.25" } "design.md 11: the catch module swapped for an SO-101 arm holding 0.25 kg. Keys 1-6 poses, space drives 2 m/s and stops, arrows pan the arm, R resets" 190)
[void](Add-Button $p4 "Arm study" { Start-Py "arm_study" "simulation\v11 sil\studies" "arm_study.py" } "Reach, static tipping, stops at 1 / 2 m/s, swings, rug edges, the firmware check (~10 min): results\arm_study.json, arm_reach.png" 170)
[void](Add-Button $p4 "Arm study report" { Start-Py "arm_study_report" "simulation\v11 sil\studies" "arm_study.py report" } "The tables in design.md 11" 170)
[void](Add-Button $p4 "Arm: tidy up (watch)" { Start-Py "sil_pick" "simulation\v11 sil" "sil_pick.py --view --world house --n 5" } "design.md 11: the arm robot's own code (omni_arm, the mode mux, the C++ firmware) picks toys off the house's floor into its bin, in MuJoCo's window" 170)
[void](Add-Button $p4 "Arm: tidy study" { Start-Py "arm_pick" "simulation\v11 sil\studies" "arm_pick.py" } "The tidy scored over seeds, two object sets, the arena and the house (~25 min): results\arm_pick.json" 170)
[void](Add-Button $p4 "Arm: mount study" { Start-Py "arm_mount" "simulation\v11 sil\studies" "arm_mount.py" } "Where the arm goes: floor reach, bin drop and stow for nose mounts vs the plate (results\arm_mount.json)" 170)
[void](Add-Button $p4 "Fetch arm meshes" { Start-Py "arm_fetch" "simulation\v11 sil" "arm_so101.py fetch" } "Once per PC: the SO-101's visual meshes (~16 MB, Apache-2.0) into assets\so101 (without them the arm draws as capsules)" 170)

# 6. tools
$p5 = Add-Group "6. Tools" ""
[void](Add-Button $p5 "Build ROS" { Start-Job2 "ros_build" "`"$Wsl`" -d $Distro -- bash `"$OmniSh`" build" })
[void](Add-Button $p5 "Tests (v11)" { Start-Py "tests_v11" "simulation\v11 sil" "-m pytest -q -p no:cacheprovider" } "Windows; the same in WSL: omni.sh test")
[void](Add-Button $p5 "Build firmware" { Start-Job2 "fw_build" "cd /d `"$Repo\firmware`" && `"$env:USERPROFILE\.platformio\penv\Scripts\pio.exe`" run" } "PlatformIO, Teensy 4.1: checks the pin map, prints flash / RAM (flashing: attended sessions only)")
[void](Add-Button $p5 "Pin map" { Start-Process "$Repo\firmware\PINS.md" } "firmware\PINS.md, from the checked pin map")
[void](Add-Button $p5 "ROS status" { Start-Job2 "ros_status" "`"$Wsl`" -d $Distro -- bash `"$OmniSh`" status" })
[void](Add-Button $p5 "WSL terminal" { Start-Process $Wsl -ArgumentList "-d", $Distro, "--cd", "~" } "A Linux terminal, for when you need one")
[void](Add-Button $p5 "Open repo folder" { Start-Process explorer.exe "`"$Repo`"" })
[void](Add-Button $p5 "design.md" { Start-Process "$Repo\design.md" })
[void](Add-Button $p5 "Build CAD (v8)" { Start-Py "cad_v8" "cad\v8" "build.py" } "STEP / STL of both robots, BOM, print list, clash checks over tilt and suspension (~15 min)")
[void](Add-Button $p5 "Render CAD (v8)" { Start-Py "cad_v8_render" "cad\v8" "render.py --meshes" } "Renders into cad\v8\out\renders")
[void](Add-Button $p5 "View CAD 3D (v8)" { Start-Py "cad_v8_view" "cad\v8" "view.py" } "Orbit the robot; keys 0-5 toggle base / pods / module / mounts / screws / foam; Control panel 'tilt' turns the module")
[void](Add-Button $p5 "View arm robot 3D" { Start-Py "cad_v8_view_arm" "cad\v8" "view.py --arm" } "The arm robot: the arm kit and the SO-101 stowed")
[void](Add-Button $p5 "Open CAD STEP" { Start-Process "$Repo\cad\v8\out\assembly.step" } "cad\v8\out\assembly.step in its default app (eDrawings or SolidWorks): the catch robot, a coloured assembly with sub-assemblies")
[void](Add-Button $p5 "Open arm STEP" { Start-Process "$Repo\cad\v8\out\assembly_arm.step" } "cad\v8\out\assembly_arm.step: the arm robot (arm kit, camera bar at its arm index)")
[void](Add-Button $p5 "STEP as one part" { Start-Py "cad_v8_bodies" "cad\v8" "step_export.py --bodies" } "Writes cad\v8\out\assembly_bodies.step: a single multi-body part, the fastest to open in SolidWorks")
[void](Add-Button $p5 "CAD outputs" { Start-Process explorer.exe "`"$Repo\cad\v8\out`"" })

# jobs + log
$bottom = New-Object System.Windows.Forms.Panel
$bottom.Dock = "Fill"
$form.Controls.Add($bottom)
$bottom.BringToFront()
$lbJobs = New-Object System.Windows.Forms.ListBox
$lbJobs.Dock = "Left"; $lbJobs.Width = 200
$lbJobs.Add_SelectedIndexChanged({ if ($lbJobs.SelectedItem) { $script:Current = ($lbJobs.SelectedItem -split " ")[0]; Show-Log } })
$tt.SetToolTip($lbJobs, "Jobs this window started: click one to see its log")
$lblLog = New-Object System.Windows.Forms.Label
$lblLog.Dock = "Top"; $lblLog.Height = 20; $lblLog.Text = "Log"
$tbLog = New-Object System.Windows.Forms.TextBox
$tbLog.Multiline = $true; $tbLog.ReadOnly = $true; $tbLog.ScrollBars = "Vertical"; $tbLog.Dock = "Fill"
$tbLog.Font = New-Object System.Drawing.Font("Consolas", 8.5); $tbLog.WordWrap = $false
$bottom.Controls.Add($tbLog); $bottom.Controls.Add($lblLog); $bottom.Controls.Add($lbJobs)

$timer = New-Object System.Windows.Forms.Timer
$timer.Interval = 1000
$script:OpenWhenUp = 0
$script:OpenTab = "home"
function Test-Console {
    try { $c = New-Object System.Net.Sockets.TcpClient; $c.Connect("localhost", 8080); $c.Close(); return $true }
    catch { return $false }
}
$timer.Add_Tick({
    Update-Jobs; Show-Log
    if ($script:OpenWhenUp -gt 0) {
        $script:OpenWhenUp--
        if (Test-Console) { $script:OpenWhenUp = 0; Start-Process "http://localhost:8080/#$($script:OpenTab)" }
    }
})
$timer.Start()

$form.Add_FormClosing({
    $running = @($script:Jobs.Keys | Where-Object { -not $script:Jobs[$_].HasExited -and $_ -like "ros_*" })
    if ($running.Count -gt 0) {
        $r = [System.Windows.Forms.MessageBox]::Show("Stop the ROS steps still running ($($running -join ', '))?",
             "omni-robot launcher", "YesNoCancel", "Question")
        if ($r -eq "Cancel") { $_.Cancel = $true; return }
        if ($r -eq "Yes") {
            & $Wsl -d $Distro -- bash $OmniSh stop-console | Out-Null
            & $Wsl -d $Distro -- bash $OmniSh stop | Out-Null
        }
    }
    if ($script:Jobs["joystick"] -and -not $script:Jobs["joystick"].HasExited) { $script:Jobs["joystick"].Kill() }
})

[void]$form.ShowDialog()
