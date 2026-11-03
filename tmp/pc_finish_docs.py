from pathlib import Path
import json,math,hashlib
root=Path.cwd();p=root/'design.md';s=p.read_text(encoding='utf-8')
old=next(x for x in s.splitlines() if x.startswith('**PC regression, Nov 3:**'))
new='**PC regression, Nov 3:** current ARM/Python/D435/wall clock reached 8/8 house goals (0.113 m mean true arrival error). ARM/stereo explored 115.4 m² and returned within 0.073 m of its start in 130.7 wall seconds. SLAM restart rejects cached/old map transforms and reactivates an inactive lifecycle node; console stop/takeover tracks launch descendants, including separately grouped SLAM children, preventing orphan publishers and self-termination. In the final single-localizer recovery check the PID stayed unchanged and the next goal arrived 0.061 m from truth. Loading a saved map still assumes its start unless the operator supplies a pose. The installed composed Nav2 can crash or require forced termination during shutdown; this remains open, separate from completed driving/picking results. See the PC regression report.'
if old!=new:
    with (root/'design full history.md').open('a',encoding='utf-8') as f:f.write('\n\n### Nov 3 PC regression — interim map-switch statement corrected\n\nThe earlier multi-switch diagnostic was later found to have an orphan localizer publishing concurrently. Retain this statement as superseded debugging context, not qualified evidence:\n\n'+old+'\n')
    s=s.replace(old,new)
active=next(x for x in s.splitlines() if x.startswith('- **Active baseline:**'))
if 'PC stack regression' not in active:s=s.replace(active,active+' Nov 3 PC stack regression: 4.5–4.6 (simulation only).')
p.write_text(s,encoding='utf-8')
p=root/'simulation/v11 sil/studies/results/pc_stack_2026-11-03.json';r=json.loads(p.read_text(encoding='utf-8'))
for key,name in [('arm_autonomous_explore','pc_arm_explore.json'),('arm_return_map_and_inactive_recovery','pc_return_goal.json')]:r['runs'][key]=json.loads((root/'tmp'/name).read_text(encoding='utf-8'))
r['runs']['slam_switches_with_initial_pose']['qualification']='Debug only: later orphan-publisher discovery invalidates this as a clean multi-switch qualification run.'
for name in ['pc_general_smoke_clean.log','pc_general_console_clean.log','pc_return_goal.log','pc_console_final.log']:r['logs'][name]=(root/'tmp'/name).read_text(encoding='utf-8',errors='replace')
r['additional_validation']['clean_browser']='console_smoke.py --slam --tabs --faults: all passed'
e=r['runs']['arm_autonomous_explore'];a=e['truth_start'];b=e['rows'][-1]['truth'];r['additional_validation']['explore_return_error_m']=round(math.hypot(a[0]-b[0],a[1]-b[1]),4)
files=['ros/src/omni_console/omni_console/supervisor.py','ros/src/omni_console/omni_console/mission_node.py','ros/src/omni_console/omni_console/tflite.py','ros/src/omni_arm/omni_arm/pick.py','ros/src/omni_arm/omni_arm/motion.py','simulation/v11 sil/robot_export.py','simulation/v11 sil/params.py','ros/src/omni_bringup/omni_bringup/stack.py','ros/src/omni_bringup/config/nav2.yaml','firmware/lib/omni_fw/fw_params_arm.h','firmware/lib/omni_fw/protocol.h']
r['source_sha256']={f:hashlib.sha256((root/f).read_bytes()).hexdigest() for f in files}
p.write_text(json.dumps(r,indent=1),encoding='utf-8')
print('Explore return error:',r['additional_validation']['explore_return_error_m'])