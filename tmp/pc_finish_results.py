from pathlib import Path
import json,hashlib
root=Path.cwd();p=root/'simulation/v11 sil/studies/results/pc_stack_2026-11-03.json';r=json.loads(p.read_text(encoding='utf-8'))
r['logs']['pc_general_console_clean.log']=(root/'tmp/pc_general_console_clean.log').read_text(encoding='utf-8',errors='replace')
r['logs']['pc_general_smoke_clean.log']=(root/'tmp/pc_general_smoke_clean.log').read_text(encoding='utf-8',errors='replace')
r['cleanup']={'robot_launches_remaining':False,'slam_children_remaining':False,'supervisor_remaining':False,'windows_viewers_remaining':False,'temporary_checkout_removed':True,'synthetic_depth_trace_removed':True}
r['package_versions']={'nav2-collision-monitor':'1.3.13-1noble.20260921.030324','rclcpp-components':'28.1.22-1noble.20260911.115213','slam-toolbox':'2.8.5-1noble.20260915.204151'}
for f in ['ros/tools/arm_load.py','ros/tools/console_smoke.py','ros/src/omni_face/omni_face/voice.py']:r['source_sha256'][f]=hashlib.sha256((root/f).read_bytes()).hexdigest()
p.write_text(json.dumps(r,indent=1),encoding='utf-8')
assert r['runs']['arm_return_map_and_inactive_recovery']['inactive_recovery']['ok']
assert r['runs']['arm_return_map_and_inactive_recovery']['goal']['err_m']<.15
assert r['runs']['arm_autonomous_explore']['ended'] and r['runs']['arm_autonomous_explore']['known_m2']>100
assert 'all passed' in r['logs']['pc_general_smoke_clean.log']
assert '63 passed' in r['logs']['pc_console_final.log']
print('Saved and checked final results')