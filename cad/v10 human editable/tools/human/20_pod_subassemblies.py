"""The v10 wheel pod as sub-assemblies (replaces the flat 123-component V10H_wheel_pod.SLDASM).

    cadenv python 20_pod_subassemblies.py roller|hub|wheel|chassis|fork|shock|pod [more stages]

  V10H_wheel_pod            top: chassis (fixed), fork (hinge, travel limit), wheel (bearings), motor, 2 shocks
    V10H_pod_chassis        wheel guard, pod mount (tower), pivot shaft + spacers, foot and shock-pin hardware
    V10H_pod_fork           inner / outer fork plates, motor cup, cross bar, pivot bearings, fork hardware
    V10H_pod_wheel          hub assembly + 12 rollers (each on its axle bolt, axial position FIXED: no end float)
      V10H_pod_hub          hubs A / B / C, bearing cartridge, 2 x 6804, joint and axle bolts, nuts
      V10H_pod_roller x12   roller core + TPU tread
    V10H_dummy_motor        stator, rotor, 687, washer, screw (unchanged file)
    V10H_pod_shock x2       case + rod, sliding

The flat pod's mates are reproduced at the level where their two parts meet. Each stage rebuilds its own file.
"""
import sys
from subasm import *

G = {}   # key -> group, filled below
HUB_KEYS = None


def part_path(rec):
    return PARTS / ('V10H_' + rec['part'] + '.SLDPRT')


def records_where(pred):
    return [r for r in RECORDS if pred(r)]


def is_hw(r):
    return r['part'].startswith(('bolt_', 'nut_', 'washer_', 'bearing_'))


# ---- group membership ------------------------------------------------------------------------------------------------
def group_of(r):
    k, p = r['key'], r['part']
    if k == 'Bench':
        return None                                           # the test bench: the robot's deck replaces it
    if k.startswith('Roller_'):
        return 'roller'
    if k in ('Guard', 'Tower', 'Pivot_shaft', 'Inner_pivot_spacer', 'Outer_pivot_spacer'):
        return 'chassis'
    if k.startswith(('Guard_foot_bolt', 'Tower_foot_bolt')):
        return None                                           # the bench's screws: the robot's deck screws (v9 base fasteners) do this job
    if k in ('Inner_fork', 'Outer_fork', 'Motor_cup', 'Fork_bar'):
        return 'fork'
    if k in ('Hub_A', 'Hub_B', 'Hub_C', 'Cartridge'):
        return 'hub'
    if k.endswith('_shock_case') or k.endswith('_shock_rod'):
        return 'shock'
    if k in ('Motor_stator', 'Motor_rotor', 'Motor_bearing', 'Motor_screw', 'Motor_washer'):
        return 'motor'
    if r['parent'] == 'Guard' and is_hw(r):
        return 'chassis'
    if r['parent'] == 'Hub_B' and is_hw(r):
        return 'hub'
    if r['parent'] == 'Inner_fork' and is_hw(r):
        return 'fork'
    raise RuntimeError('no group for ' + k)


for _r in RECORDS:
    G[_r['key']] = group_of(_r)
# hardware that joins two sub-assemblies is mated at the pod level: the cup's rear screws into the motor stator, the
# motor flange screws into hub A, and the shock-eye pins (through a fork or the tower / guard AND the shock)
POD_HW = {'bolt_M3_20_PAN_1', 'bolt_M3_16_PAN_2', 'nut_M3_nyloc_1', 'bolt_M3_14_PAN_1', 'washer_M3_1', 'bolt_M3_16_PAN_1'}
for _r in RECORDS:
    if _r['part'] == 'bolt_M2p5_10_PAN' or _r['key'] in POD_HW:
        G[_r['key']] = 'pod'


def members(group):
    return [r for r in RECORDS if G[r['key']] == group and r['key'] not in ('Motor_stator', 'Motor_rotor', 'Motor_bearing', 'Motor_screw', 'Motor_washer')]


def build_group(sw, group, path, fixed_key, lock_to, lock_exempt=()):
    """Flat build of one rigid sub-assembly: every member at its pod pose; `lock_to` maps key -> parent for the rigid
    members; hardware by geometry."""
    a = Asm(sw, path)
    recs = members(group)
    for r in recs:
        a.add(r['key'], part_path(r), T(r), fixed=r['key'] == fixed_key)
    a.refresh()
    a.save(check=False)
    return a, recs


def rigid_locks(a, recs, skip=()):
    """Lock every non-hardware member to its manifest parent (hinge-free groups)."""
    for r in recs:
        if is_hw(r) or r['key'] in skip or not r['parent'] or r['parent'] not in a.parts:
            continue
        a.lock(r['key'], r['key'], r['parent'])


def hardware_mates(a, recs, hosts, only=None):
    """Fasteners and bearings by geometry. Failures are collected so one run shows them all."""
    failed = []
    hw = [r for r in recs if is_hw(r) and (only is None or only(r))]
    for i, r in enumerate(hw):
        try:
            if r['part'].startswith('bearing_'):
                mate_bearing(a, r, hosts)
            else:
                mate_fastener(a, r, hosts)
        except Exception as e:                                   # noqa: BLE001
            failed.append((r['key'], str(e)))
            print('   !! FAILED', r['key'], e, flush=True)
        if i % 8 == 7:
            a.save(check=False)
    if failed:
        raise RuntimeError('hardware mates failed: ' + str(failed))


# ---- stages -----------------------------------------------------------------------------------------------------------
def roller(sw):
    """One roller: the PLA core fixed at the origin, the TPU tread locked to it. Instanced 12 times by the wheel."""
    rc, rt = BYKEY['Roller_1_1'], BYKEY['Roller_1_1_TPU']
    a = Asm(sw, ASSEMBLIES / 'V10H_pod_roller.SLDASM')
    a.add('Core', part_path(rc), np.eye(4), fixed=True)
    a.add('Tread', part_path(rt), np.eye(4))
    a.refresh()
    a.lock('Tread_to_core', 'Tread', 'Core')
    a.view()
    a.save()
    print(a.report(), flush=True)
    return a


def finish(a):
    a.view()
    a.save()
    print(json.dumps({k: v for k, v in a.report().items() if k != 'status'}, indent=1), flush=True)
    st = a.report()['status']
    print('  not fully constrained:', {k: v for k, v in st.items() if v != 'fully'}, flush=True)


def hub(sw):
    """Hubs A / B / C, the bearing cartridge, the two 6804s, the joint, cartridge and axle screws and the axle nuts: one rigid body."""
    a, recs = build_group(sw, 'hub', ASSEMBLIES / 'V10H_pod_hub.SLDASM', 'Hub_B', None)
    rigid_locks(a, recs)
    hardware_mates(a, recs, recs)
    finish(a)


def wheel(sw):
    """The hub assembly fixed at the origin and twelve rollers, each concentric with its axle bolt and held at the nominal
    axial position (the flat pod's 23.2-23.8 mm end-float limit is now a plain 23.5 mm distance)."""
    a = Asm(sw, ASSEMBLIES / 'V10H_pod_wheel.SLDASM')
    a.add('Hub', ASSEMBLIES / 'V10H_pod_hub.SLDASM', np.eye(4), fixed=True)
    a.alias_nested('Hub', keys_of_file('V10H_pod_hub'))
    rollers = [(f'Roller_{row}_{j}', f'bolt_M3_50_PAN_{(row - 1) * 6 + j}') for row in (1, 2) for j in range(1, 7)]
    for key, _ in rollers:
        a.add(key, ASSEMBLIES / 'V10H_pod_roller.SLDASM', T(BYKEY[key]))
    a.refresh()
    a.save(check=False)
    for key, bolt in rollers:
        Tm = T(BYKEY[key])
        a.concentric(key + '_axle', key, Tm[:3, 2], Tm[:3, 3], 1.8, bolt, 1.2)
        a.mater.mate(key + '_axial', 'distance', a.plane(key, 'Front Plane'), a.plane_of(bolt, 'Front Plane'), dist=23.5)
    finish(a)


def chassis(sw):
    """Wheel guard (fixed), pod mount (tower), the pivot shaft with its two spacers, the foot bolts and the shock pins.
    The bench plate of the flat pod is not here: in the robot the deck holds the feet."""
    a, recs = build_group(sw, 'chassis', ASSEMBLIES / 'V10H_pod_chassis.SLDASM', 'Guard', None)
    a.lock('Tower_to_guard', 'Tower', 'Guard')
    pivot = MANIFEST['pivot']
    a.mater.mate('Shaft_axial_location', 'distance', a.plane('Pivot_shaft', 'Right Plane'), a.plane('Guard', 'Right Plane'), dist=3)
    a.concentric('Shaft_in_guard', 'Pivot_shaft', [1, 0, 0], [0, *pivot], 3.5, 'Guard', 3.55, lock=True)
    for key in ('Inner_pivot_spacer', 'Outer_pivot_spacer'):
        x = BYKEY[key]['matrix'][0][3]
        a.mater.mate(key + '_axial', 'distance', a.plane(key, 'Front Plane'), a.plane('Guard', 'Right Plane'), dist=abs(x))
        a.concentric(key + '_axis', key, [1, 0, 0], [0, *pivot], 3.6, 'Pivot_shaft', 3.5, lock=True)
    a.save(check=False)
    hardware_mates(a, recs, recs)
    finish(a)


def fork(sw):
    """Inner fork (reference), outer fork cap, motor cup and cross bar locked to it, the two 627 pivot bearings, fork screws."""
    a, recs = build_group(sw, 'fork', ASSEMBLIES / 'V10H_pod_fork.SLDASM', 'Inner_fork', None)
    rigid_locks(a, recs)
    hardware_mates(a, recs, recs)
    finish(a)


def shock(sw):
    """Telescoping 11 mm shock mock-up: case at the origin (axis +z), rod sliding in it. Inner and outer shocks are both instances."""
    case, rod = BYKEY['Inner_shock_case'], BYKEY['Inner_shock_rod']
    rel = np.linalg.inv(T(case)) @ T(rod)
    a = Asm(sw, ASSEMBLIES / 'V10H_pod_shock.SLDASM')
    a.add('Case', part_path(case), np.eye(4), fixed=True)
    a.add('Rod', part_path(rod), rel)
    a.refresh()
    a.concentric('Rod_slide', 'Rod', [0, 0, 1], [0, 0, 0], 2.0, 'Case', 2.2)
    finish(a)


def place_nested(a, key, pose):
    """Move a part nested in a flexible sub-assembly to `pose` (free degrees of freedom only) and check it stuck."""
    cp = a.comp_of(key)
    M.set_placement(a.sw, cp, np.array(pose)[:3, :])
    a.model.EditRebuild3()
    got = placement(cp)
    if np.abs(got - np.array(pose)[:3, :]).max() > 1e-3:
        raise RuntimeError(f'{key} did not move to its pose: {np.round(got, 3)}')


def pod(sw):
    """Top of the pod: chassis fixed; fork hinges on the pivot shaft (travel limit); wheel on its bearings; the dummy
    motor's stator locked to the fork and its rotor to the wheel; two shocks between fork and chassis."""
    import shutil
    flat = ROOT / 'V10H_wheel_pod.SLDASM'
    keep = ROOT / 'superseded' / 'flat_wheel_pod'
    if flat.exists() and not (keep / flat.name).exists():
        keep.mkdir(parents=True, exist_ok=True)
        for f in (flat, ROOT / 'validation/human/V10H_wheel_pod.json', ROOT / 'validation/human/V10H_wheel_pod_progress.json'):
            if f.exists():
                shutil.copy2(f, keep / f.name)
    pivot = MANIFEST['pivot']
    lo_deg, hi_deg = MANIFEST['limits_deg']
    a = Asm(sw, flat)
    a.add('Chassis', ASSEMBLIES / 'V10H_pod_chassis.SLDASM', np.eye(4), fixed=True)
    a.add('Fork', ASSEMBLIES / 'V10H_pod_fork.SLDASM', np.eye(4))
    a.add('Wheel', ASSEMBLIES / 'V10H_pod_wheel.SLDASM', np.eye(4))
    a.add('Motor', ROOT / 'V10H_dummy_motor.SLDASM', T(BYKEY['Motor_stator']))
    for pre in ('Inner', 'Outer'):
        a.add(pre + '_shock', ASSEMBLIES / 'V10H_pod_shock.SLDASM', T(BYKEY[pre + '_shock_case']))
    for k in ('Wheel', 'Motor', 'Inner_shock', 'Outer_shock'):
        a.flexible(k)
    a.refresh()
    a.alias_nested('Chassis', keys_of_file('V10H_pod_chassis'))
    a.alias_nested('Fork', keys_of_file('V10H_pod_fork'))
    a.alias_nested('Wheel', keys_of_file('V10H_pod_wheel'))
    a.alias_nested('Hub', keys_of_file('V10H_pod_hub'))
    mk = json.loads((ROOT / 'validation/human/V10H_dummy_motor_progress.json').read_text())
    a.alias_nested('Motor', {'Motor_stator': mk['Stator'], 'Motor_rotor': mk['Rotor'], 'Motor_bearing': mk['Bearing_687'],
                             'Motor_screw': mk['Screw'], 'Motor_washer': mk['Washer']})
    for pre in ('Inner', 'Outer'):
        sk = keys_of_file('V10H_pod_shock')
        a.alias_nested(pre + '_shock', {pre + '_shock_case': sk['Case'], pre + '_shock_rod': sk['Rod']})
    # the outer shock's rod sits 0.3 mm nearer its case than the inner one's: move it inside the instance
    place_nested(a, 'Outer_shock_rod', T(BYKEY['Outer_shock_rod']))
    a.save(check=False)
    m = a.mater
    # --- the fork on the pivot shaft -------------------------------------------------------------------------------
    a.mater.mate('Fork_axial_location', 'coincident', a.plane('Inner_fork', 'Right Plane'), a.plane('Guard', 'Right Plane'))
    a.concentric('Suspension_pivot', 'Inner_fork', [1, 0, 0], [0, *pivot], 11, 'Pivot_shaft', 3.5)
    a.limit('Suspension_travel', a.plane('Inner_fork', 'Top Plane'), a.plane('Guard', 'Front Plane'), 'angle', 90, lo_deg, hi_deg)
    # --- the wheel on the fork's stub, through the two 6804s -----------------------------------------------------------
    a.mater.mate('Wheel_axial_location', 'coincident', a.plane('Hub_B', 'Right Plane'), a.plane('Inner_fork', 'Right Plane'))
    a.concentric('Wheel_bearings', 'Hub_B', [1, 0, 0], [0, 0, 0], 30, 'Inner_fork', 10)
    a.save(check=False)
    # --- the motor: stator on the fork, rotor on the wheel ----------------------------------------------------------
    a.lock('Motor_stator_on_fork', 'Motor_stator', 'Inner_fork')
    a.lock('Motor_rotor_on_wheel', 'Motor_rotor', 'Hub_B')
    a.save(check=False)
    # --- shocks: eye on the fork, eye on the chassis, sliding inside (the shock's own mate) ----------------------------
    for pre, lower, upper, rad in (('Inner', 'Inner_fork', 'Tower', 1.7), ('Outer', 'Outer_fork', 'Guard', 1.4)):
        sk = pre + '_shock'
        case, rod = T(BYKEY[sk + '_case']), T(BYKEY[sk + '_rod'])
        lo, hi = case[:3, 3], rod[:3, 3]
        a.concentric(pre + '_lower_eye', sk + '_case', [1, 0, 0], lo, 1.6, lower, 1.4)
        a.mater.mate(pre + '_shock_lateral', 'distance', a.plane(sk, 'Right Plane'), a.plane('Inner_fork', 'Right Plane'), dist=abs(lo[0]))
        a.concentric(pre + '_upper_eye', sk + '_rod', [1, 0, 0], hi, 1.6, upper, rad)
    a.save(check=False)
    # --- fasteners between sub-assemblies --------------------------------------------------------------------------
    cross = [r for r in RECORDS if G[r['key']] == 'pod']
    for r in cross:
        a.add(r['key'], part_path(r), T(r))
    a.refresh()
    hosts = [r for r in RECORDS if a.has(r['key'])]
    hardware_mates(a, cross, hosts)
    finish(a)


def main():
    sw = connect()
    for stage in sys.argv[1:]:
        globals()[stage](sw)


if __name__ == '__main__':
    main()
