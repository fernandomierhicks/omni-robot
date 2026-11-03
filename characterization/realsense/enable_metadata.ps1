# Turns on per-frame metadata for RealSense cameras on Windows (README.md, Setup).
#
# Without it librealsense stamps frames on arrival at the host (timestamp domain "system_time") and has no
# exposure, frame counter or laser fields, so rs_timing.py / rs_flash_latency.py can't split the latency.
# Windows only passes UVC metadata through when the camera's interfaces have MetadataBufferSizeInKB<pin>
# set; this is what librealsense's scripts/realsense_metadata_win10.ps1 (and the RealSense Viewer's
# first-run prompt) does.
#
# Run in an administrator PowerShell, with the camera plugged in and working (it must have enumerated at
# least once), then unplug and replug it:
#     powershell -ExecutionPolicy Bypass -File enable_metadata.ps1
#Requires -RunAsAdministrator

$classes = '{e5323777-f976-4f5b-9b55-b94699c46e44}', '{65E8773D-8F56-11D0-A3B9-00A0C9223196}'
$done = 0
foreach ($c in $classes) {
    $root = "HKLM:\SYSTEM\CurrentControlSet\Control\DeviceClasses\$c"
    if (-not (Test-Path $root)) { continue }
    # RealSense D400 interfaces: VID 8086, PID 0Axx / 0Bxx, the depth (MI_00) and RGB (MI_03) functions
    Get-ChildItem $root | Where-Object { $_.PSChildName -match 'vid_8086&pid_0[ab][0-9a-f]{2}&mi_0[03]' } | ForEach-Object {
        $p = Join-Path $_.PSPath '#GLOBAL\Device Parameters'
        if (Test-Path $p) {
            # two pins on the depth function (depth + IR), one on RGB; an extra key is harmless
            New-ItemProperty -Path $p -Name MetadataBufferSizeInKB0 -Value 5 -PropertyType DWord -Force | Out-Null
            New-ItemProperty -Path $p -Name MetadataBufferSizeInKB1 -Value 5 -PropertyType DWord -Force | Out-Null
            Write-Host "metadata on: $($_.PSChildName)"
            $done++
        }
    }
}
if ($done -eq 0) {
    Write-Host "No RealSense interface found in the registry. Plug the camera into a USB 3 port, let Windows finish installing it (Device Manager shows 'Intel(R) RealSense(TM) Depth Camera 435 ...' under Cameras), then run this again."
} else {
    Write-Host "$done interface(s) set. Unplug and replug the camera, then run rs_info.py: the timestamps should read global_time."
}
