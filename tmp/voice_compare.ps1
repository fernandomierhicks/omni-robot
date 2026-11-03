Add-Type -AssemblyName System.Speech
$s=New-Object System.Speech.Synthesis.SpeechSynthesizer
$s.GetInstalledVoices() | ForEach-Object { $_.VoiceInfo.Name }
foreach ($v in $s.GetInstalledVoices()) {
  $s.SelectVoice($v.VoiceInfo.Name)
  $s.Rate=0
  $s.SetOutputToWaveFile("C:\Dropbox\Personal Projects\Robotics\omni-robot\tmp\"+$v.VoiceInfo.Name+".wav")
  $s.Speak('Hey omni, follow me.')
  $s.SetOutputToNull()
}