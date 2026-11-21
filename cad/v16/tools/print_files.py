"""print_files/First floor: the printed parts of the first floor and the four pods, sorted by filament colour, plus an Excel print tracker.
(Fernando, Nov 16: "so I can easily print the same part across the 4 pods".)

    python -B print_files.py                 rebuild the colour folders (and the tracker only if it does not exist yet)
    python -B print_files.py --bumper        also the four bumper quarters (left out for now)
    python -B print_files.py --new-tracker   write a fresh tracker next to the old one ("Print tracker (new).xlsx"): your counts are never overwritten

omni-robot-sim interpreter (build123d, openpyxl). Reads step_export/ only (run robot_step / the export stages first, so the STEPs carry the
current colours); writes only inside print_files/First floor/:

    White/  Gray/  Orange/  Black/  TPU gray/  Agnostic/      one folder per part type inside each, the STEPs of every pod in it
    README.md  Print tracker.xlsx

The colour folders are rebuilt on every run (do not keep your own files in them); the tracker and nothing else is kept.
Colours come from palette.py. Agnostic = printed parts nobody sees once the robot is built (measured: README "Filament colours").
Pod numbers are Fernando's wheel numbers: 1 FL, 2 FR, 3 RL, 4 RR (top view, front up), engraved on the big parts. The FL and RR pods are the
mirror hand: the labelled parts are already in their pod's hand in step_export; the lug and the wheel hub are written once per pod here.
"""
import argparse
import shutil
import sys
from pathlib import Path

import build123d as b

sys.path.insert(0, str(Path(__file__).resolve().parent))
import palette as PAL                                                  # noqa: E402
from step_colour import write_part                                     # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
POD = ROOT / 'step_export' / 'pod_mid' / 'parts'
BASE = ROOT / 'step_export' / 'chassis_mid'
WHEEL = ROOT / 'step_export' / 'wheel_motor' / 'parts'
OUT = ROOT / 'print_files' / 'First floor'
TRACKER = 'Print tracker.xlsx'
PODS = ((1, 'FL'), (2, 'FR'), (3, 'RL'), (4, 'RR'))
MIRRORED = ('FL', 'RR')                                                # base.mirrored(): the 45 and 225 deg pods
FOLDER = dict(white='White', gray='Gray', orange='Orange', black='Black', tpu='TPU gray', any='Agnostic')
LABEL = dict(white='White', gray='Dark Gray', orange='Orange', black='Black', tpu='TPU gray', any='Any colour')
ORDER = ('white', 'gray', 'orange', 'black', 'tpu', 'any')

# ---- what is printed -------------------------------------------------------------------------------------------------------------------------
# one entry per part type. 'how': numbered = <stem>_<n>_<pod>.step exists per pod in step_export/pod_mid/parts; hands = written once per pod here;
# one = a single STEP serves every pod (the part is its own mirror image: checked). per_pod = copies per pod. agnostic = nobody sees it assembled.
POD_PARTS = (
    dict(folder='roller_core', title='Roller core', stem='roller_core', how='one', src=WHEEL / 'roller_core.step', per_pod=12),
    dict(folder='tower_outer', title='Tower, outer', stem='rx_tower_outer', how='numbered', per_pod=1, note='the two plunger guides Ø4.3 (clean them with a 4.3 mm drill if they print under)'),
    dict(folder='tower_inner', title='Tower, inner', stem='rx_tower_inner', how='numbered', per_pod=1, note='the two plunger guides Ø4.3 (clean them with a 4.3 mm drill if they print under)'),
    dict(folder='fork_outer', title='Fork, outer half', stem='pod_fork_outer', how='numbered', per_pod=1),
    dict(folder='fork_inner', title='Fork, inner half', stem='pod_fork_inner', how='numbered', per_pod=1),
    dict(folder='beam', title='Beam', stem='rx_beam', how='numbered', per_pod=1, note='barely seen from outside (from below only): a spare gray is fine'),
    dict(folder='hanger_outer', title='Hanger, outer', stem='rx_hanger_outer', how='numbered', per_pod=1, note='barely seen from outside (from below only)'),
    dict(folder='lug', title='Droop lug', stem='rx_lug', how='lug', per_pod=1, note='two hands: pods 1 FL and 4 RR are the mirrored file; barely seen from outside'),
    dict(folder='wheel_hub', title='Wheel hub', stem='wheel_hub', how='hub', per_pod=1, note='two hands: pods 1 FL and 4 RR are the mirrored hub (it is not its own mirror image)'),
    dict(folder='wheel_cartridge_cap', title='Wheel cartridge cap', stem='wheel_cartridge_cap', how='one', src=WHEEL / 'wheel_cartridge_cap.step', per_pod=1),
    dict(folder='roller_tread', title='Roller tread (TPU)', stem='roller_tread', how='one', src=WHEEL / 'roller_tread.step', per_pod=12),
    dict(folder='hanger_inner', title='Hanger, inner', stem='rx_hanger_inner', how='numbered', per_pod=1, any=True),
    dict(folder='ring_inner', title='Ring, inner', stem='rx_ring_inner', how='one', src=POD / 'rx_ring_inner.step', per_pod=1, any=True),
    dict(folder='ring_outer', title='Ring, outer', stem='rx_ring_outer', how='one', src=POD / 'rx_ring_outer.step', per_pod=1, any=True),
    dict(folder='pivot_pin', title='Pivot pin', stem='rx_pin', how='one', src=POD / 'rx_pin.step', per_pod=1, any=True),
    dict(folder='retainer_6804', title='6804 retainer', stem='pod_retainer', how='one', src=POD / 'pod_retainer.step', per_pod=1, any=True),
    dict(folder='wheel_cartridge_cup', title='Wheel cartridge cup', stem='wheel_cartridge', how='one', src=WHEEL / 'wheel_cartridge.step', per_pod=1, any=True),
    dict(folder='roller_sleeve', title='Roller sleeve', stem='roller_sleeve', how='one', src=WHEEL / 'roller_sleeve.step', per_pod=12, any=True),
    # Nov 19 (Fernando: the M3 plunger is the baseline): its PETG cap and printed sleeve, 4 per pod. optional: only while plunger.KIND has them
    # (Nov 20, the M4 set has no printed parts; pod_mid's export then removes their STEPs)
    dict(folder='plunger_cap', title='Plunger cap (PETG)', stem='plunger_cap_m3', how='one', src=POD / 'plunger_cap_m3.step', per_pod=4, any=True, optional=True,
         note='PETG; pocket down on the bed'),
    dict(folder='plunger_sleeve', title='Plunger sleeve (PETG)', stem='plunger_sleeve_m3', how='one', src=POD / 'plunger_sleeve_m3.step', per_pod=4,
         any=True, optional=True, note='PETG; standing, either end down; Ø4.0, 0.5 wall, the M3 threads through its Ø3.0 bore; open the tower guides with a 4.2 mm drill'),
)
# first floor: every file is its own piece, one of each
FLOOR_PARTS = (
    ('deck', 'Deck piece', ('deck_FL', 'deck_FR', 'deck_RL', 'deck_RR'), ''),
    ('deck_key', 'Deck key', ('deck_key',), ''),
    ('fin', 'Fin', ('support_spine', 'support_side_L', 'support_side_R', 'support_front_L', 'support_front_R'), ''),
    ('pedestal', 'Lidar pedestal', ('pedestal',), ''),
    ('battery_clip', 'Battery clip', tuple(f'bracket_battery_clip_{i}' for i in range(4)), 'seen from below only'),
)
BUMPER = ('bumper', 'Bumper quarter', ('bumper_FL', 'bumper_FR', 'bumper_RL', 'bumper_RR'), '')
AGNOSTIC_FLOOR = ('battery_clip',)


def colour_key(stem, any_=False):
    if any_:
        return 'any'
    f = PAL.filament(stem)
    if f is None:
        raise SystemExit(f'palette.py has no filament for {stem}')
    return f


def put(dest_dir, name, src):
    dest_dir.mkdir(parents=True, exist_ok=True)
    shutil.copy2(src, dest_dir / name)


def build_folders(with_bumper):
    for d in FOLDER.values():                                          # the generated folders only; the tracker and README stay
        shutil.rmtree(OUT / d, ignore_errors=True)
    OUT.mkdir(parents=True, exist_ok=True)
    pod_rows, floor_rows = [], []
    hub_src = None
    for p in POD_PARTS:
        if p.get('optional') and not p['src'].exists():
            continue
        c = colour_key(p['stem'], p.get('any'))
        d = OUT / FOLDER[c] / p['folder']
        files = []
        if p['how'] == 'numbered':
            for n, pod in PODS:
                name = f"{p['stem']}_{n}_{pod}.step"
                put(d, name, POD / name)
                files.append(name)
        elif p['how'] == 'lug':
            for n, pod in PODS:
                name = f'rx_lug_{n}_{pod}.step'
                put(d, name, POD / ('rx_lug_m.step' if pod in MIRRORED else 'rx_lug.step'))
                files.append(name)
        elif p['how'] == 'hub':
            hub_src = WHEEL / 'wheel_hub.step'
            mirror = None
            for n, pod in PODS:
                name = f'wheel_hub_{n}_{pod}.step'
                if pod in MIRRORED:
                    if mirror is None:
                        mirror = b.import_step(str(hub_src)).mirror(b.Plane.XZ)     # sw_robot_parts.mirror(): the mirror hand
                    d.mkdir(parents=True, exist_ok=True)
                    write_part(mirror, PAL.colour('wheel_hub'), d / name, f'wheel_hub_{n}_{pod}')
                else:
                    put(d, name, hub_src)
                files.append(name)
        else:
            put(d, p['src'].name, p['src'])
            files.append(p['src'].name)
        pod_rows.append(dict(colour=c, title=p['title'], folder=f'{FOLDER[c]}\\{p["folder"]}', per_pod=p['per_pod'], note=p.get('note', ''), files=files))
    for folder, title, stems, note in FLOOR_PARTS + ((BUMPER,) if with_bumper else ()):
        for stem in stems:
            c = colour_key(stem if not stem.startswith('bracket_battery') else 'bracket_battery', folder in AGNOSTIC_FLOOR)
            put(OUT / FOLDER[c] / folder, stem + '.step', BASE / (stem + '.step'))
            floor_rows.append(dict(colour=c, title=f'{title}: {stem}', folder=f'{FOLDER[c]}\\{folder}', note=note, files=[stem + '.step']))
    return pod_rows, floor_rows


def write_readme(pod_rows, floor_rows, with_bumper):
    lines = [
        '# First floor and the four pods: print files', '',
        'Generated by `cad/v16/tools/print_files.py` from `step_export/` (rerun it after the STEPs change). The colour folders are rebuilt each time:',
        'keep your own files out of them. **`Print tracker.xlsx` is yours and is never overwritten.**', '',
        '**Folders:** `White/`, `Gray/` (Dark Gray PLA), `Orange/`, `Black/`, `TPU gray/` (the treads: TPU, a different printer setup), and `Agnostic/`',
        '(parts nobody sees once the robot is built: any colour, use up spare filament). Inside each, one folder per part type with every pod\'s file in it.', '',
        '**Pod numbers** (top view, front up, as engraved on the parts): **1 = FL, 2 = FR, 3 = RL, 4 = RR**. A file named `..._2_FR.step` is the part for pod 2.',
        'Parts that are the same on every pod have one file: print it as many times as the tracker says.', '',
        '**Two parts come in two hands** (pods 1 FL and 4 RR are the mirror image of 2 FR and 3 RL): the wheel hub and the droop lug. Their files are written',
        'once per pod, so print the file with the pod\'s number. Everything else that is not numbered is its own mirror image (checked on the CAD).', '',
        '| Folder | Part | Files | Per pod | In all |', '| --- | --- | --- | --- | --- |']
    for r in pod_rows:
        lines.append(f"| `{r['folder']}` | {r['title']} | {len(r['files'])} | {r['per_pod']} | {r['per_pod'] * len(PODS)} |")
    lines += ['', '**First floor (one of each):**', '', '| Folder | Pieces |', '| --- | --- |']
    seen = {}
    for r in floor_rows:
        seen.setdefault(r['folder'], []).append(r['files'][0].replace('.step', ''))
    for f, names in seen.items():
        lines.append(f"| `{f}` | {', '.join(names)} |")
    if not with_bumper:
        lines += ['', 'The four bumper quarters are left out for now (`python -B print_files.py --bumper` adds them).']
    (OUT / 'README.md').write_text('\n'.join(lines) + '\n', encoding='utf-8')


def write_tracker(path, pod_rows, floor_rows):
    from openpyxl import Workbook
    from openpyxl.formatting.rule import CellIsRule, FormulaRule
    from openpyxl.styles import Alignment, Border, Font, PatternFill, Side
    from openpyxl.utils import get_column_letter
    from openpyxl.worksheet.datavalidation import DataValidation

    F = lambda **k: Font(**{'name': 'Arial', 'size': 10, **k})          # noqa: E731
    thin = Side(style='thin', color='BFBFBF')
    box = Border(left=thin, right=thin, top=thin, bottom=thin)
    head_fill = PatternFill('solid', fgColor='1F2933')
    in_fill = PatternFill('solid', fgColor='FFF2CC')
    sw = dict(white=('FFFFFF', '000000'), gray=('545454', 'FFFFFF'), orange=('FF6A13', '000000'), black=('151515', 'FFFFFF'),
              tpu=('8E9089', '000000'), any=('E7E6E6', '000000'))

    wb = Workbook()
    ws = wb.active
    ws.title = 'Pods'
    npod = len(PODS)

    def header(sheet, row, names):
        for i, n in enumerate(names, 1):
            c = sheet.cell(row, i, n)
            c.font, c.fill, c.border = F(bold=True, color='FFFFFF'), head_fill, box
            c.alignment = Alignment(horizontal='center', vertical='center', wrap_text=True)

    def colour_cell(sheet, row, key):
        c = sheet.cell(row, 1, LABEL[key])
        c.fill, c.font, c.border = PatternFill('solid', fgColor=sw[key][0]), F(bold=True, color=sw[key][1]), box

    def legend(sheet, lines):
        for i, t in enumerate(lines, 1):
            sheet.cell(i, 1, t).font = F(bold=(i == 1), size=12 if i == 1 else 10)

    # ---- Pods ----
    legend(ws, ['Pods: parts printed once for each pod',
                'Type how many you have printed in the yellow cells (one column per pod). Printed, Left and Status work themselves out.',
                'Pods: 1 = FL, 2 = FR, 3 = RL, 4 = RR (top view, front up; the numbers engraved on the parts). Example: two roller cores printed for pod 1 = type 2 under Pod 1, Roller core.'])
    HR = 5
    header(ws, HR, ['Colour', 'Part', 'Per pod'] + [f'Pod {n} {p}' for n, p in PODS] + ['Printed', f'Needed ({npod} pods)', 'Left', 'Status', 'Folder', 'Notes'])
    first, last = HR + 1, HR + len(pod_rows)
    cP0, cP1 = 4, 3 + npod                                               # first and last pod column
    L = get_column_letter
    for r, row in enumerate(pod_rows, first):
        colour_cell(ws, r, row['colour'])
        ws.cell(r, 2, row['title'])
        ws.cell(r, 3, row['per_pod'])
        for c in range(cP0, cP1 + 1):
            cell = ws.cell(r, c, 0)
            cell.fill, cell.font = in_fill, F(color='0000FF')
            cell.alignment = Alignment(horizontal='center')
        pc = cP1 + 1
        ws.cell(r, pc, f'=SUM({L(cP0)}{r}:{L(cP1)}{r})')
        ws.cell(r, pc + 1, f'=C{r}*COLUMNS(${L(cP0)}${HR}:${L(cP1)}${HR})')
        ws.cell(r, pc + 2, f'=MAX(0,{L(pc + 1)}{r}-{L(pc)}{r})')
        ws.cell(r, pc + 3, f'=IF({L(pc)}{r}>{L(pc + 1)}{r},"extra",IF({L(pc)}{r}={L(pc + 1)}{r},"done",IF({L(pc)}{r}=0,"to print","in progress")))')
        ws.cell(r, pc + 4, row['folder'])
        ws.cell(r, pc + 5, row['note'])
        for c in range(2, pc + 6):
            if c not in range(cP0, cP1 + 1):
                ws.cell(r, c).font = F()
            ws.cell(r, c).border = box
        for c in (3, pc, pc + 1, pc + 2, pc + 3):
            ws.cell(r, c).alignment = Alignment(horizontal='center')
    pc = cP1 + 1
    tr = last + 1
    ws.cell(tr, 2, 'Parts still to print').font = F(bold=True)
    ws.cell(tr, pc + 2, f'=SUM({L(pc + 2)}{first}:{L(pc + 2)}{last})').font = F(bold=True)
    ws.cell(tr, pc + 2).alignment = Alignment(horizontal='center')
    dv = DataValidation(type='whole', operator='between', formula1='0', formula2='99', allow_blank=False, showErrorMessage=True,
                        errorTitle='Whole number', error='Type how many you have printed (0 to 99).')
    ws.add_data_validation(dv)
    dv.add(f'{L(cP0)}{first}:{L(cP1)}{last}')
    cf = lambda x: PatternFill(start_color=x, end_color=x, fill_type='solid')                      # noqa: E731  (conditional-format fills)
    green, amber, grey = cf('C6EFCE'), cf('FFEB9C'), cf('D9D9D9')
    ws.conditional_formatting.add(f'{L(cP0)}{first}:{L(cP1)}{last}', FormulaRule(formula=[f'{L(cP0)}{first}>=$C{first}'], fill=green))
    st = f'{L(pc + 3)}{first}:{L(pc + 3)}{last}'
    ws.conditional_formatting.add(st, CellIsRule(operator='equal', formula=['"done"'], fill=green, font=Font(name='Arial', size=10, bold=True, color='006100')))
    ws.conditional_formatting.add(st, CellIsRule(operator='equal', formula=['"in progress"'], fill=amber))
    ws.conditional_formatting.add(st, CellIsRule(operator='equal', formula=['"extra"'], fill=grey))
    for col, w in zip('ABC', (12, 24, 9)):
        ws.column_dimensions[col].width = w
    for c in range(cP0, cP1 + 1):
        ws.column_dimensions[L(c)].width = 9
    for c, w in ((pc, 9), (pc + 1, 13), (pc + 2, 8), (pc + 3, 12), (pc + 4, 28), (pc + 5, 70)):
        ws.column_dimensions[L(c)].width = w
    ws.row_dimensions[HR].height = 32
    ws.freeze_panes = ws.cell(first, 4)

    # ---- First floor ----
    ff = wb.create_sheet('First floor')
    legend(ff, ['First floor: one of each piece',
                'Type 1 in the yellow cell when the piece is printed. Left and Status work themselves out.',
                'Pieces are named as in their STEP files (deck_FL = the deck piece under pod 1).'])
    header(ff, HR, ['Colour', 'Part', 'Needed', 'Printed', 'Left', 'Status', 'Folder', 'Notes'])
    f0, f1 = HR + 1, HR + len(floor_rows)
    for r, row in enumerate(floor_rows, f0):
        colour_cell(ff, r, row['colour'])
        ff.cell(r, 2, row['title'])
        ff.cell(r, 3, 1)
        cell = ff.cell(r, 4, 0)
        cell.fill, cell.font = in_fill, F(color='0000FF')
        ff.cell(r, 5, f'=MAX(0,C{r}-D{r})')
        ff.cell(r, 6, f'=IF(D{r}>C{r},"extra",IF(D{r}=C{r},"done",IF(D{r}=0,"to print","in progress")))')
        ff.cell(r, 7, row['folder'])
        ff.cell(r, 8, row['note'])
        for c in range(2, 9):
            if c != 4:
                ff.cell(r, c).font = F()
            ff.cell(r, c).border = box
        for c in (3, 4, 5, 6):
            ff.cell(r, c).alignment = Alignment(horizontal='center')
    ff.cell(f1 + 1, 2, 'Pieces still to print').font = F(bold=True)
    ff.cell(f1 + 1, 5, f'=SUM(E{f0}:E{f1})').font = F(bold=True)
    ff.cell(f1 + 1, 5).alignment = Alignment(horizontal='center')
    dv2 = DataValidation(type='whole', operator='between', formula1='0', formula2='99', showErrorMessage=True, errorTitle='Whole number',
                         error='Type how many you have printed (0 to 99).')
    ff.add_data_validation(dv2)
    dv2.add(f'D{f0}:D{f1}')
    ff.conditional_formatting.add(f'D{f0}:D{f1}', FormulaRule(formula=[f'D{f0}>=$C{f0}'], fill=green))
    st2 = f'F{f0}:F{f1}'
    ff.conditional_formatting.add(st2, CellIsRule(operator='equal', formula=['"done"'], fill=green, font=Font(name='Arial', size=10, bold=True, color='006100')))
    ff.conditional_formatting.add(st2, CellIsRule(operator='equal', formula=['"in progress"'], fill=amber))
    ff.conditional_formatting.add(st2, CellIsRule(operator='equal', formula=['"extra"'], fill=grey))
    for col, w in zip('ABCDEFGH', (12, 38, 9, 9, 8, 12, 28, 40)):
        ff.column_dimensions[col].width = w
    ff.row_dimensions[HR].height = 22
    ff.freeze_panes = ff.cell(f0, 3)

    # ---- Summary (by colour: how much of each filament is still to print) ----
    sm = wb.create_sheet('Summary', 0)
    legend(sm, ['Print tracker: what is left, by colour', 'Counts of printed parts (not grams). Nothing to type here: it reads the Pods and First floor sheets.',
                'Colour folders: print_files\\First floor\\<colour>\\<part>\\.'])
    header(sm, HR, ['Colour', 'Parts to print in all', 'Printed', 'Left', 'Done'])
    for i, k in enumerate(ORDER):
        r = HR + 1 + i
        colour_cell(sm, r, k)
        lab = f'$A{r}'
        need = f"SUMIF(Pods!$A${first}:$A${last},{lab},Pods!${L(pc + 1)}${first}:${L(pc + 1)}${last})+SUMIF('First floor'!$A${f0}:$A${f1},{lab},'First floor'!$C${f0}:$C${f1})"
        left = f"SUMIF(Pods!$A${first}:$A${last},{lab},Pods!${L(pc + 2)}${first}:${L(pc + 2)}${last})+SUMIF('First floor'!$A${f0}:$A${f1},{lab},'First floor'!$E${f0}:$E${f1})"
        sm.cell(r, 2, '=' + need)
        sm.cell(r, 3, f'=B{r}-D{r}')
        sm.cell(r, 4, '=' + left)
        sm.cell(r, 5, f'=IF(B{r}=0,0,C{r}/B{r})')
        sm.cell(r, 5).number_format = '0%'
        for c in range(2, 6):
            sm.cell(r, c).font, sm.cell(r, c).border = F(), box
            sm.cell(r, c).alignment = Alignment(horizontal='center')
    tot = HR + 1 + len(ORDER)
    sm.cell(tot, 1, 'All').font = F(bold=True)
    for c, col in ((2, 'B'), (3, 'C'), (4, 'D')):
        sm.cell(tot, c, f'=SUM({col}{HR + 1}:{col}{tot - 1})').font = F(bold=True)
        sm.cell(tot, c).alignment = Alignment(horizontal='center')
    sm.cell(tot, 5, f'=IF(B{tot}=0,0,C{tot}/B{tot})').number_format = '0%'
    sm.cell(tot, 5).font = F(bold=True)
    sm.cell(tot, 5).alignment = Alignment(horizontal='center')
    for col, w in zip('ABCDE', (14, 20, 10, 10, 9)):
        sm.column_dimensions[col].width = w
    sm.row_dimensions[HR].height = 32
    from openpyxl.worksheet.properties import PageSetupProperties
    for sheet in wb.worksheets:                                          # prints on one page wide, landscape
        sheet.page_setup.orientation = 'landscape'
        sheet.page_setup.fitToWidth, sheet.page_setup.fitToHeight = 1, 0
        sheet.sheet_properties.pageSetUpPr = PageSetupProperties(fitToPage=True)
    wb.save(path)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--bumper', action='store_true', help='also the four bumper quarters')
    ap.add_argument('--new-tracker', action='store_true', help='write "Print tracker (new).xlsx" instead of keeping the existing tracker')
    a = ap.parse_args()
    pod_rows, floor_rows = build_folders(a.bumper)
    write_readme(pod_rows, floor_rows, a.bumper)
    tr = OUT / TRACKER
    if a.new_tracker:
        tr = OUT / 'Print tracker (new).xlsx'
    if a.new_tracker or not tr.exists():
        write_tracker(tr, pod_rows, floor_rows)
        print('wrote', tr)
    else:
        print('kept', tr, '(your counts); rows are not changed by this run: use --new-tracker if the part list changed')
    n = sum(len(r['files']) for r in pod_rows + floor_rows)
    print(f'{n} STEP files in {OUT}')
    for d in FOLDER.values():
        if (OUT / d).exists():
            print(f'  {d}: ' + ', '.join(f'{p.name} ({len(list(p.glob("*.step")))})' for p in sorted((OUT / d).iterdir())))


if __name__ == '__main__':
    main()
