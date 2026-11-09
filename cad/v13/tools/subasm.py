"""Hierarchical assemblies for the whole robot: helpers on top of sw_native / mate_api (sw_robot.py uses them).

Run with C:/Users/ferna/cadenv/Scripts/python.exe. Every sub-assembly is its own .SLDASM; a parent inserts the file and mates to it.
Component names are the keys of the build scripts (Name2 = key, so a nested part reads 'Fork-1/Inner_fork'). Faces are found by geometry
(mate_api.Mater), so mates look the same at any depth.

Copied from cad/v10's tools/human/subasm.py for cad/v13 (cad/solidworks.md section 17: a version reads only its own files) and trimmed:
the v10 assembly manifest, the v10 fastener / bearing mating (mate_fastener, mate_bearing) and the two helpers that close every open
document or quit SolidWorks (close_all_but, restart) are left out, so nothing here can touch a document it did not build.
"""
import sys, json, math
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from sw_native import *            # noqa: E402,F401,F403  (wrap, sld, c, connect, release, ROOT, MM, placement, bodies)
import mate_api as M               # noqa: E402

KEYS = ROOT / 'solidworks' / '_keys'          # per assembly: record key -> component name (so a parent can name nested parts); sw_robot.py sets it


def keys_of_file(name):
    return json.loads((KEYS / (name + '.json')).read_text())


def T(rec_or_matrix):
    m = rec_or_matrix['matrix'] if isinstance(rec_or_matrix, dict) else rec_or_matrix
    return np.array(m, float)


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
        """mapping: record key -> nested component name relative to the parent ('V13_x-1'); registers the full names."""
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
        KEYS.mkdir(parents=True, exist_ok=True)
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
