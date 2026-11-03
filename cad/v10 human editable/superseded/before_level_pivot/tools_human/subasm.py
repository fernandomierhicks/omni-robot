"""Hierarchical assemblies for the v10 pod and the robot: helpers on top of sw_native / mate_api.

Run with C:/Users/ferna/cadenv/Scripts/python.exe. Every sub-assembly is its own .SLDASM; a parent inserts the file and
mates to it. Component names are the keys of the build scripts (Name2 = key, so a nested part reads 'Fork-1/Inner_fork').
Faces are found by geometry (mate_api.Mater), so mates look the same at any depth.
"""
import sys, json, math
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent
TASK_ROOT = HERE.parents[1]
sys.path.insert(0, str(TASK_ROOT / 'tools'))
from sw_native import *            # noqa: E402,F401  (wrap, sld, c, connect, ROOT, MM, placement)
import mate_api as M               # noqa: E402

PARTS = ROOT / 'parts' / 'human'
ASSEMBLIES = ROOT / 'assemblies'
ASSEMBLIES.mkdir(exist_ok=True)
KEYS = ASSEMBLIES / '_keys'          # per assembly: record key -> component name (so a parent can name nested parts)


def keys_of_file(name):
    return json.loads((KEYS / (name + '.json')).read_text())
MANIFEST = json.loads((ROOT / 'reference' / 'assembly_manifest.json').read_text())
RECORDS = MANIFEST['parts']
BYKEY = {r['key']: r for r in RECORDS}


def T(rec_or_matrix):
    m = rec_or_matrix['matrix'] if isinstance(rec_or_matrix, dict) else rec_or_matrix
    return np.array(m, float)


def restart(sw=None):
    """Quit SolidWorks and start a fresh session (it leaks memory over hundreds of opens and then refuses files).
    Refuses if a document outside this project's folder or the scratchpad is open."""
    import time
    sw = sw or connect()
    for raw in sw.GetDocuments() or []:
        try:
            d = wrap(raw, sld.IModelDoc2)
            path = d.GetPathName()
        except Exception:
            continue
        if path and not str(Path(path).resolve()).lower().startswith(str(ROOT.resolve()).lower()):
            raise RuntimeError('will not restart SolidWorks: another document is open: ' + path)
        if not path:
            raise RuntimeError('will not restart SolidWorks: an unsaved document is open: ' + d.GetTitle())
    try:
        sw.ExitApp()
    except Exception:
        pass
    time.sleep(10)
    return connect()


def close_all_but(sw, keep=()):
    """Close every open document whose title is not in keep (only the ones this script opened: call it on a clean session)."""
    for raw in sw.GetDocuments() or []:
        d = wrap(raw, sld.IModelDoc2)
        if d.GetTitle() not in keep:
            sw.CloseDoc(d.GetTitle())


def discard(sw, *names):
    """Close the named documents without saving (a test run leaves its temporary mates in the loaded copy)."""
    for raw in sw.GetDocuments() or []:
        try:
            d = wrap(raw, sld.IModelDoc2)
            title, path = d.GetTitle(), d.GetPathName()
        except Exception:                                           # a document SolidWorks has already dropped
            continue
        if title in names or Path(path).name in names:
            try:
                sw.CloseDoc(title)
            except Exception:
                pass


def open_doc(sw, path, kind):
    """kind 1 = part, 2 = assembly. Returns the loaded document (opened silently, read-only is not needed)."""
    d, e, w = sw.OpenDoc6(str(path), kind, 1, '', 0, 0)
    if d is None:
        raise RuntimeError(f'cannot open {path} ({e}/{w})')
    return wrap(d, sld.IModelDoc2)


class Asm:
    """One assembly document being built. Components are added at a 4x4 pose (mm), named by key."""

    def __init__(self, sw, path, fresh=True):
        self.sw, self.path = sw, Path(path)
        self.name = self.path.stem
        self.parts = {}
        if fresh:
            for raw in sw.GetDocuments() or []:                  # a loaded copy of the same file would block the save
                d = wrap(raw, sld.IModelDoc2)
                if Path(d.GetPathName()).name.lower() == self.path.name.lower() or d.GetTitle().replace('.SLDASM', '') == self.name:
                    sw.CloseDoc(d.GetTitle())
            tmpl = sw.GetUserPreferenceStringValue(c.swDefaultTemplateAssembly)
            self.model = wrap(sw.NewDocument(tmpl, 0, 0, 0), sld.IModelDoc2)
            for _ in range(5):                                       # SolidWorks sometimes answers None while busy
                if self.model is not None:
                    break
                import time
                time.sleep(5)
                self.model = wrap(sw.NewDocument(tmpl, 0, 0, 0), sld.IModelDoc2)
            if self.model is None:
                raise RuntimeError('SolidWorks would not create a new assembly (template %r)' % tmpl)
            self.model.SetTitle2(self.name)
            self.asm = wrap(self.model, sld.IAssemblyDoc)
            self.save(check=False)
        else:                                                      # an existing file, opened for inspection or edits
            self.model = open_doc(sw, self.path, 2)
            self.asm = wrap(self.model, sld.IAssemblyDoc)
            sw.ActivateDoc3(self.model.GetTitle(), False, 0, 0)
        self.alias = {}                    # record key -> component name of a part nested in a sub-assembly
        self.mater = M.Mater(sw, self.model, prefix='')
        self.quiet()

    # -- view bookkeeping ---------------------------------------------------------------------------------------------
    def quiet(self):
        self.model.FeatureManager.EnableFeatureTree = False
        self.model.FeatureManager.EnableFeatureTreeWindow = False
        wrap(self.model.ActiveView, sld.IModelView).EnableGraphicsUpdate = False

    def view(self):
        self.model.FeatureManager.EnableFeatureTree = True
        self.model.FeatureManager.EnableFeatureTreeWindow = True
        wrap(self.model.ActiveView, sld.IModelView).EnableGraphicsUpdate = True
        self.model.ShowNamedView2('*Isometric', 7)
        self.model.ViewZoomtofit2()
        self.model.GraphicsRedraw2()

    def activate(self):
        self.sw.ActivateDoc3(self.model.GetTitle(), False, 0, 0)

    # -- components ---------------------------------------------------------------------------------------------------
    def add(self, key, path, pose=None, fixed=False, kind=None):
        """Insert a part or assembly file as component `key` at `pose` (4x4 mm). Free unless fixed."""
        path = Path(path)
        if key in self.parts:
            return self.parts[key].Name2
        if not path.exists():
            raise RuntimeError('missing file ' + str(path))
        kind = kind or (2 if path.suffix.lower() == '.sldasm' else 1)
        open_doc(self.sw, path, kind)
        self.activate()
        cp = wrap(self.asm.AddComponent5(str(path), 0, '', False, '', 0, 0, 0), sld.IComponent2)
        if cp is None:
            raise RuntimeError('cannot insert ' + str(path))
        cp.Name2 = key
        self.parts[key] = cp
        self.model.ClearSelection2(True)
        cp.Select4(False, None, False)
        self.asm.UnfixComponent()
        M.set_placement(self.sw, cp, (np.eye(4) if pose is None else np.array(pose))[:3, :])
        if fixed:
            self.model.ClearSelection2(True)
            cp.Select4(False, None, False)
            self.asm.FixComponent()
        self.model.ClearSelection2(True)
        return cp.Name2

    def name(self, key):
        return self.parts[key].Name2

    def flexible(self, key, on=True):
        """Let a sub-assembly's own mates move (a rigid sub-assembly is a single body to the parent). 1 = flexible, 0 = rigid."""
        cp = self.parts[key]
        self.model.ClearSelection2(True)
        cp.Select4(False, None, False)
        ok = self.asm.CompConfigProperties6(cp.GetSuppression2(), 1 if on else 0, True, False, '', False, False, 0)
        self.model.ClearSelection2(True)
        if not ok or cp.Solving != (1 if on else 0):
            raise RuntimeError(f'cannot set {key} flexible={on} (Solving={cp.Solving})')

    def plane(self, key, plane):
        """SelectByID2 name of a datum plane of component `key` (any depth)."""
        return self.plane_of(key, plane)

    def refresh(self):
        self.mater._comps = M.comps(self.model)
        self.mater._cache = {}

    # -- mates --------------------------------------------------------------------------------------------------------
    def lock(self, name, a, b):
        return self.mater.lock(name, self.name_of(a), self.name_of(b))

    def keys_of(self):
        """record key -> Name2 of every direct component (saved next to the file so a parent can find nested parts)"""
        return {k: cp.Name2 for k, cp in self.parts.items()}

    def name_of(self, key):
        if key in self.alias:
            return self.alias[key]
        return self.parts[key].Name2 if key in self.parts else key

    def has(self, key):
        return key in self.parts or key in self.alias

    def comp_of(self, key):
        return self.mater.comp(self.name_of(key))

    def plane_of(self, key, plane):
        return f'{plane}@{self.comp_of(key).GetSelectByIDString()}'

    def alias_nested(self, parent_key, mapping):
        """mapping: record key -> nested component name relative to the parent ('V10H_x-1'); registers the full names."""
        base = self.name_of(parent_key)
        for k, n in mapping.items():
            self.alias[k] = base + '/' + n

    def concentric(self, name, a, axis, point, ra, b, rb, lock=False):
        m = self.mater
        return m.mate(name, 'concentric', m.cyl(self.name_of(a), axis, point, ra), m.cyl(self.name_of(b), axis, point, rb), lock=lock)

    def coincident_planes(self, name, a, b, plane='Right Plane'):
        return self.mater.mate(name, 'coincident', self.plane(a, plane), self.plane(b, plane))

    def distance_planes(self, name, a, plane_a, b, plane_b, dist):
        return self.mater.mate(name, 'distance', self.plane(a, plane_a), self.plane(b, plane_b), dist=dist)

    def limit(self, name, first, second, kind, value, lo, hi, flip=False):
        """Limit mate (distance or angle) between two named planes; refused if it moves anything."""
        existing = wrap(self.asm.FeatureByName(name), sld.IFeature)
        if existing is not None:
            return existing
        m = self.mater
        poses = M.placements(self.model)
        for direction in (flip, not flip):
            before = set(f.Name for f in M.mates_of(self.model))
            m._select([first, second])
            mate, error = self.asm.AddMate5(6 if kind == 'angle' else 5, c.swMateAlignCLOSEST, direction,
                                            value * MM if kind == 'distance' else 0, hi * MM if kind == 'distance' else 0,
                                            lo * MM if kind == 'distance' else 0, 0, 0,
                                            math.radians(value) if kind == 'angle' else 0, math.radians(hi) if kind == 'angle' else 0,
                                            math.radians(lo) if kind == 'angle' else 0, False, False, 0)
            self.model.ClearSelection2(True)
            self.model.EditRebuild3()
            new = [f for f in M.mates_of(self.model) if f.Name not in before]
            if mate is not None and error == 1 and len(new) == 1 and not new[0].GetErrorCode2()[0] \
                    and not M.compare(poses, M.placements(self.model))[0]:
                new[0].Name = name
                return new[0]
            for f in new:
                self.model.ClearSelection2(True)
                f.Select2(False, 0)
                self.model.Extension.DeleteSelection2(0)
            self.model.EditRebuild3()
            if m._restore(poses):
                raise RuntimeError('cannot restore rejected limit ' + name)
        raise RuntimeError('limit mate failed or moved components: ' + name)

    # -- save ---------------------------------------------------------------------------------------------------------
    def save(self, check=True):
        m = self.model
        m.EditRebuild3()
        if check:
            bad = M.mate_errors(m)
            if bad:
                raise RuntimeError('mate errors ' + str(bad))
        ok, e, w = m.Extension.SaveAs3(str(self.path), 0, 1, None, None, 0, 0)
        if not ok or e:
            raise RuntimeError(f'assembly save {self.path} {(e, w)}')
        KEYS.mkdir(exist_ok=True)
        (KEYS / (self.name + '.json')).write_text(json.dumps(self.keys_of(), indent=1))
        return self.path

    def close(self):
        """Save and close this document (keeps SolidWorks' memory down while many files are built)."""
        self.save(check=False)
        self.sw.CloseDoc(self.model.GetTitle())

    def report(self):
        cs = M.comps(self.model)
        return dict(assembly=str(self.path), components=len(cs), top=len(M.comps(self.model, True)),
                    mates=len(M.mates_of(self.model)), mate_errors=M.mate_errors(self.model),
                    status=M.status(self.model))


# ---- fasteners and bearings by geometry (the flat pod's rules, on any hierarchy) -------------------------------------
def mate_fastener(a, rec, hosts):
    """Seat + concentric mates for a screw / nut / washer record against host records (dicts with key, part).
    Host faces come from the components named in `a.parts` (any depth)."""
    m = a.mater
    cp_name = a.name_of(rec['key'])
    Tm = T(rec)
    axis, point = Tm[:3, 2], Tm[:3, 3]
    part = rec['part']
    nut = part.startswith('nut_')
    washer = part.startswith('washer_M3')
    flat = part.endswith('_FLAT')
    radius = 1.4 if nut else 1.6 if washer else 1.0 if 'M2p5' in part else 1.2
    cands = []
    for h in hosts:
        if h['key'] == rec['key'] or h['part'].startswith(('bolt_', 'nut_', 'bearing_', 'roller_')) or not a.has(h['key']):
            continue
        for f in m.faces(a.name_of(h['key'])):
            if f['kind'] != 'cyl' or abs(abs(f['axis'] @ axis) - 1) > 1e-6 or f['r'] < .95 or f['r'] > 3.65:
                continue
            d = f['pt'] - point
            if np.linalg.norm(d - (d @ axis) * axis) > .015:
                continue
            cands.append((h['part'].startswith('shock_'), abs(f['r'] - radius), -f['area'], h, f))
    if not cands:
        raise RuntimeError('no concentric host for ' + rec['key'])
    cands.sort(key=lambda x: x[:3])
    face = cands[0][4]
    own = m.cyl(cp_name, axis, point, radius)
    fastnormal = axis if flat else -axis
    own_plane = m.plane(cp_name, fastnormal, float(fastnormal @ point))
    seats = []
    for h in hosts:
        if h['key'] == rec['key'] or h['part'].startswith(('bolt_', 'nut_', 'bearing_', 'roller_')) or not a.has(h['key']):
            continue
        for f in m.faces(a.name_of(h['key'])):
            if f['kind'] != 'plane' or f['n'] @ axis < .99999:
                continue
            gap = float(axis @ point) - f['off']
            if abs(gap) > .06:
                continue
            box = f['aabb']
            if np.any(point < box[0] - .1) or np.any(point > box[1] + .1):
                continue
            seats.append((abs(gap), h['key'] != rec.get('parent'), -f['area'], f, gap))
    if not seats:
        raise RuntimeError('no seating face for ' + rec['key'])
    seats.sort(key=lambda x: x[:3])
    seat, gap = seats[0][3:]
    m.mate(rec['key'] + '_seat', 'distance' if abs(gap) > .001 else 'coincident', own_plane, seat, dist=abs(gap))
    m.mate(rec['key'] + '_axis', 'concentric', own, face, lock=True)


def mate_bearing(a, rec, hosts):
    spec = {'bearing_627': (3.5, 11, 7), 'bearing_6804': (10, 16, 7), 'bearing_687': (3.5, 7, 5)}[rec['part']]
    radius, outer, width = spec
    m = a.mater
    Tm = T(rec)
    axis, pt = Tm[:3, 2], Tm[:3, 3]
    name = a.name_of(rec['key'])
    options = []
    for h in hosts:
        if h['part'].startswith(('bolt_', 'nut_', 'washer_', 'bearing_', 'roller_', 'shock_')) or not a.has(h['key']):
            continue
        for f in m.faces(a.name_of(h['key'])):
            if f['kind'] != 'cyl' or abs(f['r'] - outer) > .01 or abs(abs(f['axis'] @ axis) - 1) > 1e-6:
                continue
            d = f['pt'] - pt
            if np.linalg.norm(d - (d @ axis) * axis) > .015:
                continue
            options.append((abs((f['ctr'] - pt) @ axis), h, f))
    if not options:
        raise RuntimeError('no bearing housing ' + rec['key'])
    options.sort(key=lambda x: x[0])
    _, host, hole = options[0]
    seats = []
    for sign in (-1, 1):
        end = pt + axis * sign * width / 2
        normal = axis * sign
        own = m.plane(name, normal, float(normal @ end))
        for f in m.faces(a.name_of(host['key'])):
            if f['kind'] == 'plane' and f['n'] @ (-normal) > .99999 and abs(f['off'] - float((-normal) @ end)) < .02:
                seats.append((abs(f['off'] - float((-normal) @ end)), own, f))
    if not seats:
        raise RuntimeError('no bearing shoulder ' + rec['key'])
    seats.sort(key=lambda x: x[0])
    gap, own, seat = seats[0]
    m.mate(rec['key'] + '_shoulder', 'distance' if gap > .001 else 'coincident', own, seat, dist=gap)
    m.mate(rec['key'] + '_housing', 'concentric', m.cyl(name, axis, pt, outer), hole, lock=True)
