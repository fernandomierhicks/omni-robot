"""The rest of the robot (cad/v9, arm kit) as sub-assemblies of imported STEP parts, plus the SO-101 arm's links.

    cadenv python 40_robot_subassemblies.py groups|arm_links [...]

groups:    V10H_base (deck / bumper / electronics / fasteners), V10H_mast, V10H_head, V10H_arm_kit - every part FIXED at its
           ROBOT-frame pose inside its file (rigid bodies; the robot assembly decides how they sit)
arm_links: one sub-assembly per SO-101 link, origin = the ROBOT frame (so a link sits where the robot puts it)
The four pods are the native SolidWorks pods (20_pod_subassemblies.py), not these.
"""
import sys
from subasm import *

STEP_PARTS = ROOT / 'parts' / 'step'
INSTANCE_TABLE = ROOT / 'reference' / 'robot_sim_instances.json'
if not INSTANCE_TABLE.exists():
    INSTANCE_TABLE = ROOT / 'reference' / 'robot_v9_instances.json'
INST = json.loads(INSTANCE_TABLE.read_text())
LINKS = json.loads((ROOT / 'reference' / 'so101_links.json').read_text())

GROUPS = {'base': 'base', 'mast': 'face mast (face, speaker, mics)', 'head': 'head (camera bar, indexed)',
          'arm_kit': 'mounts (yokes or arm kit, cheeks)'}
BUMPER = ('bumper_', 'REC_bumper_')       # REC_bumper_corner_pivot: side screws at 17 deg (Nov 2 level pivot)
DECK = ('deck_', 'REC_deck_', 'battery_tray', 'payload_plate', 'lidar_riser', 'switch_riser', 'payload_ports', 'standoff', 'pin_4_x_16')


def pick(group, fastener, pred=lambda i: True):
    return [i for i in INST if i['group'] == GROUPS[group] and i['fastener'] == fastener and pred(i)]


def fixed_assembly(sw, name, instances):
    """Every instance inserted at its pose and fixed: one rigid file."""
    a = Asm(sw, ASSEMBLIES / f'V10H_{name}.SLDASM')
    for n, i in enumerate(instances):
        a.add(f'{n:03d}_{i["product"]}', STEP_PARTS / f'{i["product"]}.SLDPRT', np.array(i['matrix']), fixed=True)
        if n % 40 == 39:
            a.save(check=False)
    a.close()
    print(f'{name}: {len(instances)} parts', flush=True)


def composite(sw, name, subs):
    """A file made of the named sub-assembly files, each fixed at the origin (they were built in the ROBOT frame)."""
    a = Asm(sw, ASSEMBLIES / f'V10H_{name}.SLDASM')
    for s in subs:
        a.add(s, ASSEMBLIES / f'V10H_{s}.SLDASM', np.eye(4), fixed=True)
    a.close()
    print(f'{name}: {subs}', flush=True)


def base_parts(sw):
    fixed_assembly(sw, 'base_deck', pick('base', False, lambda i: i['product'].startswith(DECK)))
    fixed_assembly(sw, 'base_bumper', pick('base', False, lambda i: i['product'].startswith(BUMPER)))
    fixed_assembly(sw, 'base_electronics', pick('base', False, lambda i: not i['product'].startswith(DECK) and not i['product'].startswith(BUMPER)))
    fixed_assembly(sw, 'base_fasteners', pick('base', True))


def base(sw):
    composite(sw, 'base', ['base_deck', 'base_bumper', 'base_electronics', 'base_fasteners'])


def kit_parts(sw):
    for g in ('mast', 'head', 'arm_kit'):
        fixed_assembly(sw, f'{g}_parts', pick(g, False))
        fixed_assembly(sw, f'{g}_fasteners', pick(g, True))


def kits(sw):
    for g in ('mast', 'head', 'arm_kit'):
        composite(sw, g, [f'{g}_parts', f'{g}_fasteners'])


def groups(sw):
    base_parts(sw)
    base(sw)
    kit_parts(sw)
    kits(sw)


LINK_NAMES = {'arm_base': 'arm_link0_base', 'arm_shoulder': 'arm_link1_shoulder', 'arm_upper_arm': 'arm_link2_upper_arm',
              'arm_lower_arm': 'arm_link3_lower_arm', 'arm_wrist': 'arm_link4_wrist', 'arm_gripper': 'arm_link5_gripper',
              'arm_jaw': 'arm_link6_jaw'}


def arm_links(sw):
    for link in LINKS:
        insts = [dict(product=p['product'], matrix=p['matrix']) for p in link['parts']]
        fixed_assembly(sw, LINK_NAMES[link['body']], insts)


def main():
    sw = connect()
    for stage in sys.argv[1:]:
        globals()[stage](sw)


if __name__ == '__main__':
    main()
