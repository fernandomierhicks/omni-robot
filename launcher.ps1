# omni-robot launcher: the sim's controller (design.md 2). Start runs the robot's whole software (the firmware in the
# loop, ROS 2 in WSL) on the MuJoCo house at real time and opens the operator console in the browser (it drives in
# Manual: a gamepad or joystick, the keyboard or its on-screen pads; every mode is a tab there), the 3D view and the
# robot's face. The rest changes the sim: the robot (catch or arm) and the map restart it; the person, the clutter
# and the thrower change live (simulation/v11 sil/sim_world.py, TCP 9873); the camera (ros/tools/mujoco_view.py,
# UDP 9872) and the face (omni_face display.py, UDP 9883) are local windows. The choices are kept in
# logs\controller.json, which the 3D view and the face read when they open. More: CAD, firmware, tests, and the old
# launcher with every step and study (launcher_full.ps1). Open it with Launcher.cmd (double-click).

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
$SettingsFile = Join-Path $Logs "controller.json"
$Api = "http://localhost:8080"
$SimPort, $ViewPort, $FacePort = 9873, 9872, 9883
$script:Jobs = @{}              # name -> System.Diagnostics.Process
$script:LogJob = ""             # the job whose log is shown ("" = the running step's, from the console)
$script:OpenWhenUp = 0          # seconds left to wait for the console before opening the browser
$script:Busy = $false           # a poll is running (a slow answer never stacks them)
$script:Quiet = $false          # the controls are being set from the settings (no events)

# -- the settings (remembered) ----------------------------------------------------------------------------------
$S = @{ robot = "catch"; map = "house"; kid = $false; person = $false; thrower = $true
        clutter = @{ catch = "off"; arm = "toys" }; view = "follow"; face = @{ size = "normal"; mute = $false; hidden = $false } }
if (Test-Path $SettingsFile) {
    try {
        $j = Get-Content -Raw $SettingsFile | ConvertFrom-Json
        foreach ($k in "robot", "map", "kid", "person", "thrower", "view") { if ($null -ne $j.$k) { $S[$k] = $j.$k } }
        if ($j.clutter) { foreach ($k in "catch", "arm") { if ($j.clutter.$k) { $S.clutter[$k] = $j.clutter.$k } } }
        if ($j.face) { foreach ($k in "size", "mute", "hidden") { if ($null -ne $j.face.$k) { $S.face[$k] = $j.face.$k } } }
    } catch { }
}
function Save-Settings {
    $json = ConvertTo-Json -Depth 4 $S
    [System.IO.File]::WriteAllText($SettingsFile, $json, (New-Object System.Text.UTF8Encoding $false))   # no BOM (Python reads it)
}
function Clutter { return $S.clutter[$S.robot] }

# -- talking to the sim ------------------------------------------------------------------------------------------
function Test-Port([int]$Port, [int]$Ms = 150) {
    $c = New-Object System.Net.Sockets.TcpClient
    try {
        $ar = $c.BeginConnect("127.0.0.1", $Port, $null, $null)
        if (-not $ar.AsyncWaitHandle.WaitOne($Ms)) { return $false }
        $c.EndConnect($ar); return $true
    } catch { return $false } finally { $c.Close() }
}

function Invoke-Http([string]$Method, [string]$Path, $Body = $null) {
    # the console's supervisor (omni_console/supervisor.py): /api/state, /api/start, /api/stop, /api/log
    try {
        $req = [System.Net.HttpWebRequest]::Create("$Api$Path")
        $req.Method = $Method; $req.Proxy = $null; $req.Timeout = 3000
        if ($null -ne $Body) {
            $b = [System.Text.Encoding]::UTF8.GetBytes((ConvertTo-Json -Depth 4 -Compress $Body))
            $req.ContentType = "application/json"; $req.ContentLength = $b.Length
            $st = $req.GetRequestStream(); $st.Write($b, 0, $b.Length); $st.Close()
        }
        $resp = $req.GetResponse()
        $r = New-Object System.IO.StreamReader($resp.GetResponseStream())
        $text = $r.ReadToEnd(); $resp.Close()
        return ($text | ConvertFrom-Json)
    } catch [System.Net.WebException] {
        if ($_.Exception.Response) {
            $r = New-Object System.IO.StreamReader($_.Exception.Response.GetResponseStream())
            try { return ($r.ReadToEnd() | ConvertFrom-Json) } catch { return $null }
        }
        return $null
    } catch { return $null }
}

function Invoke-Sim($Obj) {
    # the bridge's control port (sim_world.ControlPort): one JSON line each way
    $c = New-Object System.Net.Sockets.TcpClient
    try {
        $ar = $c.BeginConnect("127.0.0.1", $SimPort, $null, $null)
        if (-not $ar.AsyncWaitHandle.WaitOne(200)) { return $null }
        $c.EndConnect($ar)
        $s = $c.GetStream(); $s.ReadTimeout = 1500
        $b = [System.Text.Encoding]::UTF8.GetBytes((ConvertTo-Json -Depth 3 -Compress $Obj) + "`n")
        $s.Write($b, 0, $b.Length)
        $line = (New-Object System.IO.StreamReader($s)).ReadLine()
        if ($line) { return ($line | ConvertFrom-Json) }
        return $null
    } catch { return $null } finally { $c.Close() }
}

function Send-Udp([int]$Port, $Obj) {
    $u = New-Object System.Net.Sockets.UdpClient
    try {
        $b = [System.Text.Encoding]::UTF8.GetBytes((ConvertTo-Json -Depth 3 -Compress $Obj))
        [void]$u.Send($b, $b.Length, "127.0.0.1", $Port)
    } catch { } finally { $u.Close() }
}

function Start-Job2([string]$Name, [string]$CmdLine) {
    # cmd /c "<command> > log 2>&1", hidden
    $log = Join-Path $Logs "$Name.log"
    "== $(Get-Date -Format 'HH:mm:ss') $CmdLine`r`n" | Set-Content -Encoding utf8 $log
    $psi = New-Object System.Diagnostics.ProcessStartInfo
    $psi.FileName = "C:\Windows\System32\cmd.exe"
    $psi.Arguments = "/c `"$CmdLine >> `"$log`" 2>&1`""
    $psi.WorkingDirectory = $Repo
    $psi.CreateNoWindow = $true
    $psi.UseShellExecute = $false
    $script:Jobs[$Name] = [System.Diagnostics.Process]::Start($psi)
    Update-LogList
}
function Start-Py([string]$Name, [string]$Dir, [string]$PyArgs) { Start-Job2 $Name "cd /d `"$Repo\$Dir`" && `"$Py`" -u $PyArgs" }
function Job-Running([string]$Name) { return ($script:Jobs[$Name] -and -not $script:Jobs[$Name].HasExited) }

# -- the sim on, off, again ----------------------------------------------------------------------------------------
function Step-Id { if ($S.robot -eq "arm") { return "arm" } else { return "robot" } }
function Step-Args {
    # the step's options (supervisor.SIM_STEPS): always the 3D view, real time (fast), the firmware in the loop
    $a = [ordered]@{ map = $S.map; kid = [bool]$S.kid; person = [bool]$S.person; clutter = (Clutter)
                     viewer = $true; foxglove = $false; fast = $true; firmware = $true }
    if ($S.robot -ne "arm") { $a.thrower = [bool]$S.thrower }
    return $a
}

function Start-Sim {
    Save-Settings
    $step = Step-Id
    if (Test-Port 8080) {                                  # the console runs: it restarts the step with these options
        $r = Invoke-Http "POST" "/api/start" @{ step = $step; args = (Step-Args) }
        if ($r -and $r.ok) { Set-Note "Restarting: $($r.message)" } else { Set-Note "The console refused: $($r.message)" }
        return
    }
    $opts = ((Step-Args).GetEnumerator() | ForEach-Object { "$($_.Key):=$(([string]$_.Value).ToLower())" }) -join " "
    Start-Job2 "sim" "`"$Wsl`" -d $Distro -- bash `"$OmniSh`" console --autostart $step $opts"
    $script:OpenWhenUp = 240                                # the browser opens once the console answers
    Set-Note "Starting: syncing and building in WSL, then the sim (about a minute)"
}

function Stop-Sim {
    if (Test-Port 8080) { [void](Invoke-Http "POST" "/api/stop" @{}) }
    Start-Job2 "stop" "`"$Wsl`" -d $Distro -- bash `"$OmniSh`" stop-console"
    $script:OpenWhenUp = 0
    Set-Note "Stopping the sim and the robot's software"
}

function Open-Console { Start-Process "$Api/?drive=1#manual" }

# -- the window ---------------------------------------------------------------------------------------------------
$tt = New-Object System.Windows.Forms.ToolTip
$form = New-Object System.Windows.Forms.Form
$form.Text = "omni-robot sim"
$form.Size = New-Object System.Drawing.Size(760, 830)
$form.StartPosition = "CenterScreen"
$form.Font = New-Object System.Drawing.Font("Segoe UI", 10)

$root = New-Object System.Windows.Forms.FlowLayoutPanel
$root.Dock = "Fill"; $root.FlowDirection = "TopDown"; $root.WrapContents = $false; $root.AutoScroll = $true
$root.Padding = New-Object System.Windows.Forms.Padding(10)
$form.Controls.Add($root)

function Add-Group([string]$Title) {
    $g = New-Object System.Windows.Forms.GroupBox
    $g.Text = $Title; $g.AutoSize = $true; $g.MinimumSize = New-Object System.Drawing.Size(715, 0)
    $g.Padding = New-Object System.Windows.Forms.Padding(8, 4, 8, 6)
    $f = New-Object System.Windows.Forms.FlowLayoutPanel
    $f.Dock = "Fill"; $f.AutoSize = $true; $f.WrapContents = $true; $f.MaximumSize = New-Object System.Drawing.Size(700, 0)
    $g.Controls.Add($f)
    $root.Controls.Add($g)
    return $f
}
function Add-Button($Panel, [string]$Text, [scriptblock]$Do, [string]$Tip = "", [int]$W = 120, [int]$H = 34) {
    $b = New-Object System.Windows.Forms.Button
    $b.Text = $Text; $b.Width = $W; $b.Height = $H
    $b.Add_Click($Do)
    if ($Tip) { $tt.SetToolTip($b, $Tip) }
    $Panel.Controls.Add($b)
    return $b
}
function Add-Check($Panel, [string]$Text, [bool]$On, [string]$Tip = "") {
    $c = New-Object System.Windows.Forms.CheckBox
    $c.Text = $Text; $c.Checked = $On; $c.AutoSize = $true; $c.Margin = New-Object System.Windows.Forms.Padding(6, 8, 14, 3)
    if ($Tip) { $tt.SetToolTip($c, $Tip) }
    $Panel.Controls.Add($c)
    return $c
}
function Add-Radios($Panel, [string]$Label, [string[]]$Texts, [string[]]$Values, [string]$Value, [scriptblock]$Changed) {
    # radio buttons in their own little panel (one choice), $Changed gets the value
    $p = New-Object System.Windows.Forms.FlowLayoutPanel
    $p.AutoSize = $true; $p.WrapContents = $false; $p.Margin = New-Object System.Windows.Forms.Padding(0)
    if ($Label) {
        $l = New-Object System.Windows.Forms.Label
        $l.Text = $Label; $l.AutoSize = $true; $l.Margin = New-Object System.Windows.Forms.Padding(6, 9, 2, 0)
        $p.Controls.Add($l)
    }
    for ($i = 0; $i -lt $Texts.Count; $i++) {
        $r = New-Object System.Windows.Forms.RadioButton
        $r.Text = $Texts[$i]; $r.Tag = @{ v = $Values[$i]; f = $Changed }; $r.AutoSize = $true
        $r.Checked = ($Values[$i] -eq $Value)
        $r.Margin = New-Object System.Windows.Forms.Padding(6, 7, 6, 3)
        $r.Add_CheckedChanged({ param($sender) if ($sender.Checked -and -not $script:Quiet) { & $sender.Tag.f $sender.Tag.v } })
        $p.Controls.Add($r)
    }
    $Panel.Controls.Add($p)
    return $p
}
function Set-Radios($P, [string]$Value) {
    $script:Quiet = $true
    foreach ($c in $P.Controls) { if ($c -is [System.Windows.Forms.RadioButton]) { $c.Checked = ($c.Tag.v -eq $Value) } }
    $script:Quiet = $false
}
function Add-Hint($Panel, [string]$Text) {
    $l = New-Object System.Windows.Forms.Label
    $l.Text = $Text; $l.AutoSize = $true; $l.ForeColor = [System.Drawing.Color]::DimGray
    $l.MaximumSize = New-Object System.Drawing.Size(690, 0); $l.Margin = New-Object System.Windows.Forms.Padding(4, 2, 4, 4)
    $Panel.Controls.Add($l); $Panel.SetFlowBreak($l, $true)
}

$script:Quiet = $true            # building the controls from the settings: no events until the window shows

# 1. the sim
$p1 = Add-Group "Simulation"
$bStart = Add-Button $p1 "Start" { Start-Sim } "Runs the robot's software on the sim (real time, firmware in the loop) and opens the console in Manual (a gamepad drives once you hold its dead man), the 3D view and the face. While it runs: restarts it with the options below" 110
$bStart.BackColor = [System.Drawing.Color]::PaleGreen
$bRestart = Add-Button $p1 "Restart" { Start-Sim } "The sim and all the robot's software from scratch, with the options below (the robot back at its start)" 110
$bStop = Add-Button $p1 "Stop" { Stop-Sim } "Stops the sim, the robot's software and the console" 110
$bStop.BackColor = [System.Drawing.Color]::MistyRose
$bConsole = Add-Button $p1 "Open console" { Open-Console } "The operator console in the browser (Manual tab): every mode, the map, driving, the status" 130
$lblState = New-Object System.Windows.Forms.Label
$lblState.AutoSize = $true; $lblState.MaximumSize = New-Object System.Drawing.Size(690, 0)
$lblState.Font = New-Object System.Drawing.Font("Segoe UI", 10.5, [System.Drawing.FontStyle]::Bold)
$lblState.Margin = New-Object System.Windows.Forms.Padding(4, 8, 4, 0)
$p1.SetFlowBreak($bConsole, $true)
$p1.Controls.Add($lblState); $p1.SetFlowBreak($lblState, $true)
$lblNote = New-Object System.Windows.Forms.Label
$lblNote.AutoSize = $true; $lblNote.MaximumSize = New-Object System.Drawing.Size(690, 0); $lblNote.ForeColor = [System.Drawing.Color]::DimGray
$lblNote.Margin = New-Object System.Windows.Forms.Padding(4, 2, 4, 2)
$p1.Controls.Add($lblNote)
function Set-Note([string]$t) { $lblNote.Text = $t }

# 2. the robot and the map (they restart the sim)
$p2 = Add-Group "Robot (a change restarts the sim)"
$rbRobot = Add-Radios $p2 "" @("Catch robot (catches and throws)", "Arm robot (tidies up)") @("catch", "arm") $S.robot {
    param($v) $S.robot = $v; Set-Radios $rbClutter (Clutter); Update-Enabled; Restart-IfRunning "the $v robot" }
$p2.SetFlowBreak($rbRobot, $true)
$lm = New-Object System.Windows.Forms.Label; $lm.Text = "Map:"; $lm.AutoSize = $true; $lm.Margin = New-Object System.Windows.Forms.Padding(6, 9, 2, 0)
$p2.Controls.Add($lm)
$cbMap = New-Object System.Windows.Forms.ComboBox
$cbMap.DropDownStyle = "DropDownList"; $cbMap.Width = 170; $cbMap.Margin = New-Object System.Windows.Forms.Padding(3, 6, 12, 3)
$maps = @(Get-ChildItem (Join-Path $Repo "ros\maps") -Filter *.yaml -ErrorAction SilentlyContinue | ForEach-Object { $_.BaseName })
if ($maps -notcontains "house") { $maps = @("house") + $maps }
foreach ($m in $maps) { [void]$cbMap.Items.Add($m) }
$cbMap.SelectedItem = $(if ($maps -contains $S.map) { $S.map } else { "house" })
$tt.SetToolTip($cbMap, "The saved map of the house the robot localizes on (ros\maps; make more in the console's SLAM tab)")
$cbMap.Add_SelectedIndexChanged({ if (-not $script:Quiet) { $S.map = $cbMap.SelectedItem; Restart-IfRunning "map $($S.map)" } })
$p2.Controls.Add($cbMap)
$cbKid = Add-Check $p2 "Kid mode" ([bool]$S.kid) "The firmware's kid profile: 0.8 m/s, gentle accelerations, lobs only"
$cbKid.Add_CheckedChanged({ if (-not $script:Quiet) { $S.kid = $cbKid.Checked; Restart-IfRunning "kid mode" } })

# 3. the world (live)
$p3 = Add-Group "World (changes live)"
$cbPerson = Add-Check $p3 "Person walking" ([bool]$S.person) "A person walks in front of the robot (wherever it is) and back: for Follow. The lidar sees their legs"
$cbPerson.Add_CheckedChanged({ if (-not $script:Quiet) { $S.person = $cbPerson.Checked; Save-Settings; Push-World } })
$cbThrower = Add-Check $p3 "Thrower" ([bool]$S.thrower) "Catch robot: when its mode turns Catch a thrower steps up where the room allows and throws. Off: no catch game"
$cbThrower.Add_CheckedChanged({ if (-not $script:Quiet) { $S.thrower = $cbThrower.Checked; Save-Settings; Push-World } })
$rbClutter = Add-Radios $p3 "Clutter:" @("none", "toys", "toys + obstacles") @("off", "toys", "all") (Clutter) {
    param($v) $S.clutter[$S.robot] = $v; Save-Settings; Push-World }
$tt.SetToolTip($rbClutter, "Toys: small things in the hallway (the arm robot's job; below the catch robot's bumper). Obstacles: boxes, a bag, shoes, a basket, a stool and a play ball in the rooms (the lidar sees the tall ones)")

# 4. the 3D view
$p4 = Add-Group "3D view"
foreach ($v in @(@("Follow", "follow", "Orbiting behind the robot"), @("Chase", "chase", "Low behind the robot, turning with it"),
                 @("Free", "free", "The mouse moves it: drag to orbit, right-drag to pan, scroll to zoom"),
                 @("Top", "top", "The whole house from above"), @("Robot's camera", "robot", "What the robot's left camera sees"))) {
    $b = Add-Button $p4 $v[0] { param($sender) $S.view = $sender.Tag; Save-Settings; Send-Udp $ViewPort @{ view = $sender.Tag } } $v[2] 105
    $b.Tag = $v[1]
}
[void](Add-Button $p4 "Reopen" { Start-Py "view" "ros\tools" "mujoco_view.py" } "Opens the 3D view again after you closed it (it waits for the running sim)" 90)

# 5. the face
$p5 = Add-Group "Face (the robot's 5 in screen and voice)"
$rbSize = Add-Radios $p5 "Size:" @("small", "normal", "large") @("small", "normal", "large") $S.face.size {
    param($v) $S.face.size = $v; Push-Face }
$cbMute = Add-Check $p5 "Mute" ([bool]$S.face.mute) "No voice or sounds (M in its window does the same)"
$cbMute.Add_CheckedChanged({ if (-not $script:Quiet) { $S.face.mute = $cbMute.Checked; Push-Face } })
$cbHide = Add-Check $p5 "Hide" ([bool]$S.face.hidden) "The window out of the way (it keeps talking unless muted)"
$cbHide.Add_CheckedChanged({ if (-not $script:Quiet) { $S.face.hidden = $cbHide.Checked; Push-Face } })
[void](Add-Button $p5 "Reopen" { Start-Py "face" "ros\tools" "face_view.py" } "Opens the face again after you closed it (any running sim opens it by itself)" 90)
function Push-Face { Save-Settings; Send-Udp $FacePort @{ size = $S.face.size; mute = [bool]$S.face.mute; hidden = [bool]$S.face.hidden } }

# 6. more, and the log
$p6 = Add-Group ""
$menu = New-Object System.Windows.Forms.ContextMenuStrip
function Add-Item($Parent, [string]$Text, [scriptblock]$Do) {
    $it = New-Object System.Windows.Forms.ToolStripMenuItem($Text)
    if ($Do) { $it.Add_Click($Do) }
    if ($Parent -is [System.Windows.Forms.ContextMenuStrip]) { [void]$Parent.Items.Add($it) } else { [void]$Parent.DropDownItems.Add($it) }
    return $it
}
$cad = Add-Item $menu "CAD (v8)" $null
[void](Add-Item $cad "View in 3D (catch robot)" { Start-Py "cad_v8_view" "cad\v8" "view.py" })
[void](Add-Item $cad "View in 3D (arm robot)" { Start-Py "cad_v8_view_arm" "cad\v8" "view.py --arm" })
[void](Add-Item $cad "Build (STEP, STL, BOM, checks; ~15 min)" { Start-Py "cad_v8" "cad\v8" "build.py" })
[void](Add-Item $cad "Render pictures" { Start-Py "cad_v8_render" "cad\v8" "render.py --meshes" })
[void](Add-Item $cad "Open the STEP assembly (catch robot)" { Start-Process "$Repo\cad\v8\out\assembly.step" })
[void](Add-Item $cad "Open the STEP assembly (arm robot)" { Start-Process "$Repo\cad\v8\out\assembly_arm.step" })
[void](Add-Item $cad "Write STEP as one part" { Start-Py "cad_v8_bodies" "cad\v8" "step_export.py --bodies" })
[void](Add-Item $cad "Outputs folder" { Start-Process explorer.exe "`"$Repo\cad\v8\out`"" })
$fw = Add-Item $menu "Firmware" $null
[void](Add-Item $fw "Build for the Teensy" { Start-Job2 "fw_build" "cd /d `"$Repo\firmware`" && `"$env:USERPROFILE\.platformio\penv\Scripts\pio.exe`" run" })
[void](Add-Item $fw "Pin map" { Start-Process "$Repo\firmware\PINS.md" })
$setup = Add-Item $menu "Setup (once per PC)" $null
[void](Add-Item $setup "Fetch the house's textures" { Start-Py "house_fetch" "simulation\v11 sil" "house_scene.py fetch" })
[void](Add-Item $setup "Fetch the arm's meshes" { Start-Py "arm_fetch" "simulation\v11 sil" "arm_so101.py fetch" })
[void](Add-Item $setup "Build ROS" { Start-Job2 "ros_build" "`"$Wsl`" -d $Distro -- bash `"$OmniSh`" build" })
[void](Add-Item $menu "Face demo (no robot)" { Start-Py "face_demo" "ros\tools" "face_view.py --demo" })
[void](Add-Item $menu "Tests (v11 sim)" { Start-Py "tests_v11" "simulation\v11 sil" "-m pytest -q -p no:cacheprovider" })
[void](Add-Item $menu "WSL terminal" { Start-Process $Wsl -ArgumentList "-d", $Distro, "--cd", "~" })
[void](Add-Item $menu "design.md" { Start-Process "$Repo\design.md" })
[void](Add-Item $menu "Repo folder" { Start-Process explorer.exe "`"$Repo`"" })
[void]$menu.Items.Add((New-Object System.Windows.Forms.ToolStripSeparator))
[void](Add-Item $menu "All steps and studies (the old launcher)" {
    Start-Process "C:\Windows\System32\WindowsPowerShell\v1.0\powershell.exe" -ArgumentList "-NoProfile", "-ExecutionPolicy", "Bypass", "-WindowStyle", "Hidden", "-File", "`"$Repo\launcher_full.ps1`"" })
$bMore = Add-Button $p6 "More..." { $menu.Show($bMore, 0, $bMore.Height) } "CAD, firmware, setup, tests, the old launcher with every step and study" 100
$cbLog = Add-Check $p6 "Show log" $false "The running step's log (or a More job's)"
$cbLogJob = New-Object System.Windows.Forms.ComboBox
$cbLogJob.DropDownStyle = "DropDownList"; $cbLogJob.Width = 200; $cbLogJob.Margin = New-Object System.Windows.Forms.Padding(3, 6, 3, 3)
[void]$cbLogJob.Items.Add("the sim (running step)"); $cbLogJob.SelectedIndex = 0
$cbLogJob.Add_SelectedIndexChanged({ $script:LogJob = $(if ($cbLogJob.SelectedIndex -le 0) { "" } else { $cbLogJob.SelectedItem }) })
$p6.Controls.Add($cbLogJob)
$tbLog = New-Object System.Windows.Forms.TextBox
$tbLog.Multiline = $true; $tbLog.ReadOnly = $true; $tbLog.ScrollBars = "Vertical"; $tbLog.WordWrap = $false
$tbLog.Font = New-Object System.Drawing.Font("Consolas", 8.5); $tbLog.Size = New-Object System.Drawing.Size(700, 230); $tbLog.Visible = $false
$root.Controls.Add($tbLog)
$cbLog.Add_CheckedChanged({ $tbLog.Visible = $cbLog.Checked })
function Update-LogList {
    foreach ($k in $script:Jobs.Keys) { if (-not $cbLogJob.Items.Contains($k)) { [void]$cbLogJob.Items.Add($k) } }
}

# -- the state, every second -------------------------------------------------------------------------------------
$script:SimState = $null
function Update-Enabled {
    $cbThrower.Enabled = ($S.robot -ne "arm")
}
function Restart-IfRunning([string]$What) {
    Save-Settings
    if (Test-Port 8080) { Set-Note "Restarting the sim for $What"; Start-Sim } else { Set-Note "$($What): applies at Start" }
}
function Push-World {
    # the live world, if the sim runs (else it applies at Start)
    $want = @{ person = [bool]$S.person; clutter = (Clutter) }
    if ($S.robot -ne "arm") { $want.thrower = [bool]$S.thrower }
    $r = Invoke-Sim $want
    if ($r -and $r.message) { Set-Note $r.message } elseif (-not $r) { Set-Note "Applies when the sim runs" }
}

function Format-Time([double]$s) {
    $t = [TimeSpan]::FromSeconds([math]::Floor($s))
    if ($t.TotalHours -ge 1) { return $t.ToString("h\:mm\:ss") } else { return $t.ToString("m\:ss") }
}

function Poll {
    if ($script:Busy) { return }
    $script:Busy = $true
    try {
        $supUp = Test-Port 8080
        $cur = $null
        if ($supUp) { $st = Invoke-Http "GET" "/api/state"; if ($st) { $cur = $st.current } }
        $sim = $null
        if ($supUp -and $cur -and $cur.state -eq "running") { $sim = Invoke-Sim @{ get = 1 } }
        $script:SimState = $sim
        if ($sim -and $sim.world) {                         # the live world as chosen here (a restart, a reconnect)
            $w = $sim.world
            $differs = ($w.person -ne [bool]$S.person) -or ($w.clutter -ne (Clutter)) -or
                       ($null -ne $w.thrower -and $w.thrower -ne [bool]$S.thrower)
            if ($differs) { Push-World }
        }
        $robot = $(if ($S.robot -eq "arm") { "Arm robot" } else { "Catch robot" })
        if ($sim) {
            $rtf = $(if ($null -ne $sim.rtf) { "{0:0.00}x real time" -f $sim.rtf } else { "measuring speed" })
            $lblState.Text = "Running: $robot, map $($S.map) | $rtf | sim time $(Format-Time $sim.t)"
            $lblState.ForeColor = [System.Drawing.Color]::DarkGreen
        } elseif ($cur -and $cur.state -in "starting", "running") {
            $lblState.Text = "Starting: $($cur.label) (the robot's software and the sim come up)"
            $lblState.ForeColor = [System.Drawing.Color]::DarkGoldenrod
        } elseif ($cur -and $cur.state -eq "exited") {
            $lblState.Text = "The sim stopped (exit $($cur.code)): Show log, then Restart"
            $lblState.ForeColor = [System.Drawing.Color]::Firebrick
        } elseif ($supUp) {
            $lblState.Text = "The console runs, no sim: press Restart"
            $lblState.ForeColor = [System.Drawing.Color]::DarkGoldenrod
        } elseif (Job-Running "sim") {
            $lblState.Text = "Starting: syncing and building in WSL"
            $lblState.ForeColor = [System.Drawing.Color]::DarkGoldenrod
        } else {
            $lblState.Text = "Stopped. Press Start."
            $lblState.ForeColor = [System.Drawing.Color]::DimGray
        }
        $bStart.Enabled = -not ($supUp -or (Job-Running "sim"))
        $bRestart.Enabled = $supUp
        if ($script:OpenWhenUp -gt 0) {
            $script:OpenWhenUp--
            if ($supUp) { $script:OpenWhenUp = 0; Open-Console }
        }
        if ($tbLog.Visible) { Show-Log $supUp }
    } finally { $script:Busy = $false }
}

function Show-Log([bool]$SupUp) {
    $text = ""
    if ($script:LogJob -eq "" -and $SupUp) {
        $r = Invoke-Http "GET" "/api/log?n=80"
        if ($r -and $r.lines) { $text = ($r.lines -join "`r`n") }
    }
    if (-not $text) {
        $name = $(if ($script:LogJob) { $script:LogJob } else { "sim" })
        $log = Join-Path $Logs "$name.log"
        if (Test-Path $log) {
            try {
                $fs = [System.IO.File]::Open($log, 'Open', 'Read', 'ReadWrite')
                $sr = New-Object System.IO.StreamReader($fs); $all = $sr.ReadToEnd(); $sr.Close()
                $text = (($all -split "`r?`n") | Select-Object -Last 80) -join "`r`n"
            } catch { }
        }
    }
    $text = [regex]::Replace($text, "\x1b\[[0-9;]*m", "")
    if ($tbLog.Text -ne $text) { $tbLog.Text = $text; $tbLog.SelectionStart = $tbLog.Text.Length; $tbLog.ScrollToCaret() }
}

Update-Enabled
$script:Quiet = $false
$timer = New-Object System.Windows.Forms.Timer
$timer.Interval = 1000
$timer.Add_Tick({ Poll })
$timer.Start()
$form.Add_Shown({ Poll })

$form.Add_FormClosing({
    if (Test-Port 8080) {
        $r = [System.Windows.Forms.MessageBox]::Show("Stop the sim and the robot's software?", "omni-robot sim", "YesNoCancel", "Question")
        if ($r -eq "Cancel") { $_.Cancel = $true; return }
        if ($r -eq "Yes") {
            [void](Invoke-Http "POST" "/api/stop" @{})
            & $Wsl -d $Distro -- bash $OmniSh stop-console | Out-Null
        }
    }
})

[void]$form.ShowDialog()
