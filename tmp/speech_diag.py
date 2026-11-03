import sys, wave, subprocess
from pathlib import Path
sys.path.insert(0, 'ros/src/omni_face')
from omni_face import listen, commands
p=Path(r'C:/Users/ferna/AppData/Local/Temp/pytest-of-ferna/pytest-250/test_windows_hears_a_spoken_co0/omni_follow.wav')
with wave.open(str(p),'rb') as w:
    params=w.getparams(); data=w.readframes(w.getnframes())
print(params)
q=Path('tmp/speech_padded.wav')
with wave.open(str(q),'wb') as w:
    w.setparams(params); w.writeframes(bytes(params.framerate*params.sampwidth//2)+data+bytes(params.framerate*params.sampwidth))
s=listen.Sapi()
for f in (p,q):
    args=[listen.POWERSHELL,'-NoProfile','-File',str(s.script),'-Grammar',str(s._grammar_file(commands.phrases({'kitchen':[0,0,0]}))),'-Wav',str(f.resolve())]
    r=subprocess.run(args,capture_output=True,text=True)
    print(f,repr(r.stdout),r.stderr)
from omni_face.voice import Sapi as Voice
v=Voice()
for text in ('Follow me.', 'Robot follow me.', 'Hey omni, follow me.', 'Stop.'):
    q=Path('tmp/speech_test.wav').resolve()
    v.synth(text,q)
    print(text,s.recognize_wav(commands.phrases({}),q))
for text in ('follow me', 'robot follow me', 'hey omni follow me'):
    q=Path('tmp/speech_test.wav').resolve(); v.synth(text,q)
    print('restricted',text,s.recognize_wav(['follow me'],q))
s.SCRIPT=s.SCRIPT.replace('$e.LoadGrammar(', '$e.BabbleTimeout=[TimeSpan]::FromSeconds(10); $e.InitialSilenceTimeout=[TimeSpan]::FromSeconds(10); $e.EndSilenceTimeout=[TimeSpan]::FromSeconds(1); $e.EndSilenceTimeoutAmbiguous=[TimeSpan]::FromSeconds(1); $e.LoadGrammar(')
s.script.write_text(s.SCRIPT)
print('timeouts',s.recognize_wav(commands.phrases({}),p))
original=listen.Sapi.SCRIPT
for grammar in ('$e.LoadGrammar((New-Object System.Speech.Recognition.DictationGrammar))', '$e.LoadGrammar((New-Object System.Speech.Recognition.Grammar (New-Object System.Speech.Recognition.GrammarBuilder "follow me")))'):
    code=original.replace('$e.LoadGrammar((New-Object System.Speech.Recognition.Grammar (New-Object System.Speech.Recognition.GrammarBuilder $all)))',grammar)
    s.script.write_text(code)
    print('grammar',grammar,s.recognize_wav(['follow me'],p))
code=original.replace('while ($true) {','for ($trial=0; $trial -lt 5; $trial++) {').replace('elseif ($Wav) { break }','elseif ($Wav) { [Console]::Out.WriteLine("NULL " + $e.AudioPosition.TotalSeconds) }')
s.script.write_text(code)
a=[listen.POWERSHELL,'-NoProfile','-File',str(s.script),'-Grammar',str(s._grammar_file(commands.phrases({}))),'-Wav',str(p)]
r=subprocess.run(a,capture_output=True,text=True,timeout=15); print('retry',r.stdout,r.stderr)
s.script.write_text(original)
for p2 in Path('tmp').glob('Microsoft*.wav'):
    print(p2,s.recognize_wav(commands.phrases({}),p2.resolve()))