# SolidWorks and CAD: the main working reference for agents

The single reference for a Claude session doing CAD for Fernando: native SolidWorks parts through the API (sections 1-14), STEP exports and
purchased-part rules (15-16), and the build123d → STEP → SolidWorks assembly pipeline plus printed-part design rules (17-18, from cad/v11).
Read this first, then `cad/AGENTS.md` and the active version's README (which holds that version's geometry, rounds and open items, not general rules).
It is part-agnostic. Sections 1-14 were verified on SolidWorks 2023 SP5 (API revision 31.5) by building a 40-feature part whose
geometry matched an independently built Fusion 360 twin exactly (boolean difference 0.0 mm³ both ways) and its reference STEP.
Section 12 carries older, less-tested notes about assemblies and shared coordinates. **Section 13 is a working helper library:
copy it to a scratch folder as `swhelpers.py` before you start.**

**Which route?** Native, human-editable features (sections 1-14) when Fernando will open and edit the part. Imported STEP bodies from build123d (section 17)
when the goal is review, fit, clearance or motion of a proposed assembly; those bodies are not editable features, so say which one you made.

## 1. What "human editable" means (the contract)

Fernando opens the part and edits it like a part he drew himself. So:

- **Design intent, not CSG replay.** One revolve for a turned body, one spoke + a circular pattern for six spokes, one roller + a pattern
  for twelve rollers. Never one sketch-and-feature per primitive of a boolean recipe: a part built that way ended up with ~70 numbered
  sketches (`0044_revolve_Sketch`…) and was rejected as unreadable.
- **Every sketch fully constrained** (status 3) with real dimensions. Never use *Fix* to hide an under-defined sketch.
- **Every dimension that matters is driven by a named global variable** (Tools > Equations). The variable list is the part's control panel.
  Give each variable a comment.
- **Native features only**: Revolve, Extrude, Cut, Circular Pattern, Mirror, Fillet, Chamfer, Combine. No imported STEP/BREP as a base, no
  rolled-up "base feature". Fillets and chamfers are real Fillet/Chamfer features, never sketch arcs standing in for them.
- **Everything is named** in plain words: `Spoke Sketch`, `Spoke Extrude`, `Spoke Pattern x6`, `Roller Pockets Cut`. No numeric prefixes.
  Datum planes you add are named for what they are (`Row Plane`), not `Plane1`.
- **Repeats are patterns/mirrors** of one well-built original. Build the original in the most natural place (on an axis, on a centreline).
- **No hidden tricks in the tree**: no suppressed leftovers, no dead sketches, no dangling equations.

## 2. Environment and hygiene

- **Limit open documents.** The v10 session accumulated at least 40 task documents and SolidWorks reported memory pressure before Fernando saved everything and restarted it. For part work, keep only the master and the current part open (normally 2 documents; at most 5 during a necessary comparison). Save and close each task-owned part before opening the next. Do not preload the whole registry. For assembly work, open only one assembly at a time; its referenced components may necessarily remain loaded. Close the assembly before unrelated part work. Never close an unrelated or unsaved user document to meet this limit.

- Python with COM: `C:/Users/ferna/cadenv/Scripts/python.exe` (Python 3.13, pywin32). Geometry/analysis scripts use
  `C:/Users/ferna/.venvs/omni-robot-sim/Scripts/python.exe`. Never use the PlatformIO `python` on PATH.
- Bind to the type libraries with `gencache.EnsureModule(...)` (version 0.31), **not** `EnsureDispatch` (fails with `TYPE_E_ELEMENTNOTFOUND`
  on this install). `connect()` tries `GetActiveObject` then `Dispatch`; set `Visible = True`.
- Most returned objects arrive untyped. Wrap them: `QueryInterface(cls.CLSID, IID_IDispatch)` (`wrap()` in the library). Points from
  `GetStartPoint2()` etc. come back as bare `CDispatch` and must be wrapped as `ISketchPoint` before `Select4`.
- **API lengths are metres, angles radians.** The library takes mm and degrees.
- **Disable the "Modify value" popup** while scripting (`swInputDimValOnCreate = False`; `new_part` does it) or the first dimension blocks on a
  modal dialog. **Restore it to True when you finish.**
- Each script run reconnects; state lives in the open SolidWorks document. Do not rely on Python globals between runs.
- **Never touch Fernando's documents.** SolidWorks keeps one loaded document per basename. Use a unique title for your part
  (`SetTitle2`, unsaved), close only documents you created, never *Save All*, never save over an existing `.SLDPRT` unless asked.
  (The old `tools/sw_native.py:NativePart` closes any open document called `V10_<name>`: do not use it next to Fernando's files.)
- Do not save at all unless asked. When asked, `m.Extension.SaveAs3(path, 0, 1, None, None, 0, 0)`; a save can fail with error 1 if an older
  document with that name is loaded.

## 3. Workflow that worked

1. **Understand the part as design intent first** (read the source CAD/Python/recipe, then list: bodies of revolution, repeated features,
   rows of holes, symmetric halves, fillets, tool bodies). Decide the feature order a person would use. Write the variable list.
2. **Stage scripts, one stage per script**, each starting with `delete_from(m, "<first feature of this stage>")` so a stage can be re-run
   after a fix. Stages: variables + base body → holes/bosses → repeated element (+pattern) → cuts (+pattern) → fillets/chamfers → tool bodies
   → combine.
3. **Verify after every stage**: print volume, compare with the expected number (hand calculation or a twin model). A feature that "succeeds"
   can still be in the wrong place or direction; volume and bounding box catch it, `GetErrorCode2()` does not. Volume alone is not enough for
   position: also check a few coordinates (face/edge centres, hole axes).
4. **Name each feature immediately after creating it** (`named(m, "...")`) and link its dimensions *after* naming (equations reference names).
5. **Finish with `audit(m)`**: no feature errors, every sketch status 3, no dead equations, bodies valid (`Check2()==0`), then a final
   comparison against the reference (section 9).
6. Restore preferences, leave the document open and unsaved, and say so.

## 4. Turning shapes into features (design-intent recipes)

| Shape | Human way |
| --- | --- |
| Turned body (drum, plates, bore) | One half-section profile on a datum plane + construction centreline on the axis, **Revolve**. Dimension every ordinate from the origin. |
| N identical spokes/lugs/holes around an axis | Build **one**, then **Circular Pattern** with a reference axis. |
| Left/right or top/bottom twin | Build one, **Mirror** about a datum plane. |
| Tapered or chamfered blank | Extrude the profile, then a **Cut** with a small triangle sketch on the orthogonal plane (mid-plane, slightly wider than the part). |
| Block that only exists on one side of a plane | Draw it as a half profile (arc + closing line), not a full circle plus a trim. |
| Counterbored/stepped hole | A through-hole cut plus a counterbore cut from the same offset plane (Hole Wizard was not tried; `HoleWizard5` takes 30+ arguments). |
| Hex pocket | The Polygon sketch tool, dimensioned across flats (5.10). |
| Pocket with rounded ends / scooped relief | Revolve a **tool body** (not merged), apply a real Fillet to it, **body-pattern** it, then **Combine > Subtract** (8.7). |
| Equal-pitch circle of features | Dimensioned radial centreline + angle dimension, then pattern (5.8). |
| Repeated cuts that must see all repeated material | Create **all joins first, then all cuts** (as a CSG would), so a later cut also trims earlier-patterned material. |

## 5. Sketches

### 5.1 Plane axes (verified by transforming sketch points to model space)

| Datum plane | normal | sketch x = world | sketch y = world |
| --- | --- | --- | --- |
| Front | +Z | +X | +Y |
| Top | +Y | +X | **−Z** |
| Right | +X | **−Z** | +Y |

Work out the mapping before placing anything, and sketch on **datum or reference planes, not on faces**: planes are selected by name and
never go stale after a rebuild.

### 5.2 Creating geometry

- `sm.InsertSketch(True)` after selecting the plane, then `sm.AddToDB = True` while drawing (stops accidental snapping). Lines created with
  identical end coordinates **share** their end points; arc end points too. Draw closed outlines as chains.
- Close with `sm.AddToDB = False; sm.InsertSketch(True)`, then re-fetch the feature (`last_feature`) and rename it. The old `ActiveSketch`
  interface is stale after exit.
- Construction/centre lines: `CreateCenterLine`. A free end on a pure reference centreline (e.g. the mirror axis of a symmetric outline)
  does not stop the sketch reaching status 3.
- `CreateArc(cx,cy,0, x1,y1,0, x2,y2,0, dir)`: centre, start, end, `1` = counter-clockwise.

### 5.3 Selection

- `ISketchSegment.Select4(append, None)` for lines/arcs/circles (wrap an `ISketchLine`/`ISketchArc` as `ISketchSegment`); `ISketchPoint.Select4`
  for points; the origin is `SelectByID2('Point1@Origin','EXTSKETCHPOINT',…)`.
- Sketch features are selected as features: `feat.Select2(False, 0)` before an extrude/revolve.

### 5.4 Relations (`m.SketchAddConstraints('…')`)

`sgHORIZONTAL2D`, `sgVERTICAL2D`, `sgCOINCIDENT` (point+point, point+line, point+origin), `sgTANGENT`, `sgCONCENTRIC`, `sgEQUAL`,
`sgCOLLINEAR`, `sgATMIDDLE` (point + line), `sgSYMMETRIC` (**two points, then the centreline**), `sgHORIZONTALPOINTS2D` /
`sgVERTICALPOINTS2D` (two points aligned; with the origin selected these give a zero coordinate). Guessed shorter names are silently refused.
A single-point `sgCOINCIDENT` does nothing: select the point **and** the origin.

### 5.5 Dimensions

Select the entities, then `AddHorizontalDimension2` / `AddVerticalDimension2` / `AddDiameterDimension2` / `AddRadialDimension2` /
`AddDimension2` (smart: one line = length, two lines = angle, circle = diameter). The arguments are only a text position (metres). The
returned display dimension's `.GetDimension2(0).Name` is the short name (`D3`); its full reference is `D3@<Sketch Name>`.

- Non-zero coordinate: select the point and the origin, add a horizontal/vertical dimension. Zero coordinate: `sgHORIZONTALPOINTS2D` /
  `sgVERTICALPOINTS2D` with the origin.
- Dimensions are always positive magnitudes; a point to the left of the origin just has a positive horizontal dimension. Link the
  dimension to a variable with an equation (section 6) and the geometry keeps its side.
- Prefer dimensioning **ordinates from the origin** on orthogonal profiles (add H/V relations to every line, then one dimension per
  distinct x and y). Fewer dimensions, nothing over-defined.

### 5.6 Status codes (`ISketch.GetConstrainedStatus()`)

`3` fully constrained · `2` under-constrained · **`6` = some entities over- and others under-constrained: almost always a wrong relation pair**
(e.g. a symmetric relation picked on the wrong end point of a line, which made one point symmetric to two others). `FullyDefineSketch`
returned success but left sketches under-defined: do not rely on it. A successful API return code does not prove full constraint:
read the status.

### 5.7 Revolve profile

Profile as a closed outline on a datum plane, a **construction centreline** on the axis (horizontal relation, coincident with the origin,
ends dimensioned if the sketch needs them), every line H/V, each outline point that sits on the axis coincident with the centreline.
A profile edge that lies on the axis can also be selected as the axis. Select the sketch (mark 0) then the axis segment with
`seg.Select4(True, mark_data(m, 4))`.

### 5.8 Polar placement (circle at radius r, angle a)

Draw a construction radial line from the origin to the centre, a construction reference line along the sketch axis the angle is measured from,
make them coincident at the origin, put the circle centre coincident with the radial line's end, then `sdim` the radial line (length) and `sdim`
both lines (angle; place the text point **inside** the wanted sector). `polar_circle_sketch` does this for Right-plane sketches. Pattern the result
rather than drawing N circles.

### 5.9 Arcs of a given radius with a displaced centre

An arc that is part of a circle of radius R whose centre is *not* on the part (a barrel profile, a relief): dimension its radius, dimension the
centre's offset from the origin (centre y = 0 via `sgHORIZONTALPOINTS2D` with the origin), and dimension its end points' ordinates. Do not
approximate it with a polyline.

### 5.10 Regular polygon

`CreatePolygon(cx,cy,0, vx,vy,0, sides, True)`: the second point is a corner. The sketch also gets a **construction circle tangent to the flats**
(radius = across-flats/2). Add `sgHORIZONTAL2D` to one flat to fix the rotation, dimension that circle's diameter (across flats) and the centre
from the origin: status 3. Verify corner direction by reading the line coordinates.

## 6. Global variables and equations

- Variable: `"name"= 41mm 'comment`. Angle: `"a"= 30deg`. Unitless: `"n"= 6`. Add with `IEquationMgr.Add2(-1, text, False)`.
- Drive a dimension: `"D1@Sketch Name"= "a" - "b" / 2`. **Every variable reference needs quotes** (the library adds them: write
  `a - b / 2`). `Add2` returns `-1` if refused; the library raises.
- Feature dimensions are linked the same way: `"D1@Spoke Extrude"` is the depth. A start offset shows up as a strangely numbered
  dimension (e.g. `D19@…`). `feature_own_dims(feat)` lists the feature's own dimensions in order (depth first, start offset second).
- Link **after renaming** the sketch/feature, then `EvaluateAll()` and `EditRebuild3()`.
- Define variables before the equations that use them (they evaluate top to bottom).
- **Negative values are refused** for dimensions driven this way (e.g. a plane offset to the other side): flip the geometry, not the sign
  (7.1).
- **Editing a variable from the API breaks the equations that reference it** (delete + re-add makes dependents evaluate to `0.0` and the model
  silently stops following). `set_var` re-adds the dependents. `SetEquationAndConfigurationOption` returned `-1` and changed nothing.
  Editing in the Equations dialog should be fine (not API-tested).
- `dead_equations(m)` finds dimension equations that evaluate to 0.0: run it in every audit.
- `ForceRebuild3(False)` after equation changes; cross-part equations need two rebuilds (section 12).

## 7. Reference geometry

### 7.1 Offset planes

`select(datum plane)` then `m.CreatePlaneAtOffset3(distance_m, flip, True)`. **Use this, not `InsertRefPlane`**: the flip bit and negative
distances of `InsertRefPlane` did not move the plane to the negative side. Flipping moves the plane to the other side of the datum but its
**normal keeps the datum's direction** (Right-based planes always have normal +X). Drive the distance with a positive variable
(`D1@<Plane>` = `end_dist`): the side is kept.

### 7.2 Axes

Select two datum planes, `m.InsertAxis2(True)`: the intersection is the axis. Front+Top = X, Top+Right = Z, Front+Right = Y.
Name it (`Hub Axis (X)`); patterns select it by name with mark 1.

## 8. Features

### 8.1 Direction rules (empirical, consistent across 40 features)

- **Boss extrude** default direction = **along** the sketch-plane normal. `reverse=True` flips it.
- **Cut extrude** default direction = **opposite** the plane normal (into the part if the part is behind the plane). `reverse=True` flips it.
- Normals: Front +Z, Top +Y, Right +X (offset planes keep them, even when flipped to the other side).
- **Start offset** (`start_offset` = magnitude, `flip_start=True` puts the start on the negative-normal side). To cut outward from a start at
  −d, heading −normal: `flip_start=True, reverse=False`. To start at +d heading +normal: `start_offset=d, reverse=True`.
- Don't reason, **test**: after any directional feature print the volume change and compare with the expected number. A wrong direction
  either fails (see below) or adds/removes the wrong material.

### 8.2 Boss and cut calls

- `FeatureExtrusion3(Sd, Flip, Dir, T1, T2, D1, D2, …, Merge, UseFeatScope, UseAutoSelect, T0, StartOffset, FlipStartOffset)`:
  `T1`: 0 blind, 6 mid-plane (depth = **total** length), 1 through-all; `T0 = 3` start offset.
- `FeatureCut4(…)`: **the three flags NormalCut, UseFeatScope, UseAutoSelect must all be `False`**; with them `True` the call returns
  `None` and creates nothing (it took a 16-combination test to find that). Everything else as above.
- Both return `None` on failure with no message: always check and raise. A failed cut is also what you get when the cut direction heads
  into empty space.
- Select the closed profile sketch first (`feat.Select2(False, 0)`); the sketch must have exactly the contour you expect (construction
  geometry is ignored).

### 8.3 Revolve

`FeatureRevolve2(SingleDir, IsSolid, IsThin, IsCut, Reverse, BothSame, Dir1Type, Dir2Type, Angle1, Angle2, OffRev1, OffRev2, Off1, Off2,
ThinType, Thin1, Thin2, Merge, UseFeatScope, UseAutoSelect)`. `Merge=False` gives a separate **tool body**. Axis selected with mark 4.

### 8.4 Circular patterns

`FeatureCircularPattern5(N, spacing_rad, flip, "", geometryPattern, equalSpacing, vary, sync, bdir2, bsym, N2, spacing2, "", eq2)` with
`equalSpacing=True` and `spacing = 2π` for a full circle. Selection: the axis by name with **mark 1**, every feature by name
(`'BODYFEATURE'`) with **mark 4**. Patterns of boss+cut mixes, holes, and fillet-less cut groups all worked.
- Pattern the features you built (extrude, cut, chamfer, hole…) in one pattern; list them all in the selection.
- **Geometry pattern (`True`) of a tool body fuses the copies into the main body**: never use it for cutters.
- A *feature* pattern of a non-merged revolve returned `None`. Pattern tool **bodies** instead: `circ_pattern_bodies` selects the axis with
  mark 1 and the body with `IBody2.Select2(True, mark_data(m, 256))`; this yields N separate bodies.

### 8.5 Mirror

Plane with mark 2, features with mark 1, `InsertMirrorFeature2(False, False, True, False, 0)`. Mirror a half-feature rather than re-drawing it.

### 8.6 Fillets and chamfers (real features, edges selected by geometry)

- Find edges by geometry, not by name (names are not persistent): `edges_where(body, pred)` with `edge_geom` (lines by end points, circles by
  centre/axis/radius). Select them with mark 1.
- Fillet: `FeatureFillet(2, r, 0, 0, None, None, None)`. Chamfer: `InsertFeatureChamfer(4, 16, …)` alone makes a zero-distance no-op; the library
  creates it to keep the selection, then edits its `IChamferFeatureData2` (`Type=2`, equal distance, both distances, `ModifyDefinition`).
- Periodic surfaces (cylinders) have seam edges that look like real corners. Check the edge **count** against what you expect (e.g. 12 real
  seams) and exclude seams.
- Fillet radius is a feature dimension `D1@<Fillet name>`: link it to a variable.
- Fillet the **tool** when the rounding belongs to the cutter (it leaves coves in the part) and the part when it belongs to the part.

### 8.7 Tool bodies and Combine

Revolve/extrude the cutter with `Merge=False`, fillet it, body-pattern it, then
`FeatureManager.InsertCombineFeature(15902, main_body, disp_arr([tool._oleobj_ …]))` (15902 = subtract; `c.SWBODYCUT`). Re-fetch the bodies
(`bodies(m)`) after every boolean: body objects go stale. Tool bodies stay editable in the tree.

### 8.8 Edge selection cost

`body.GetEdges()` plus per-edge wrapping handled ~1500 edges in seconds. For tens of thousands use the raw dispatch route
(`body._oleobj_.InvokeTypes(125,0,1,(12,0),())`) and cache curve descriptions; pywin32's dynamic dispatch does a type-info lookup per object.

## 9. Verification and comparison

- Per stage: volume (`GetMassProperties(1)[3]`, m³; index 4 is surface area, 0–2 centre of mass per the API docs), plus coordinates of key
  features.
- Final: `audit(m)`; `Check2()` must be 0; one solid body (unless you intend tool bodies).
- **`IBody2.GetBodyBox()` is a loose box**: it reported ±61.80 where the true box was ±61.62. Do not use it for tolerance checks; use the exported
  STEP's bounding box.
- Export a throw-away copy for comparison: `m.Extension.SaveAs3(path_step, 0, 3, None, None, 0, 0)` (3 = silent + copy; the document keeps its name).
- Compare against an independent reference by **boolean differences both ways**, not volume alone (a misplaced hole has the right volume).
  `build123d` booleans on these STEPs failed ("unable to clean", whole-body result): many coincident faces. Fusion 360's
  `TemporaryBRepManager.booleanOperation` handled them (import both STEPs into a scratch design, subtract both ways, expect 0.0).
  If a reference *intentionally* differs (e.g. a removed lump), compute the expected difference separately and check the remainder is tiny.
- Mass-property volume of an exported/re-imported STEP differs from SolidWorks' own by ~0.01% (tessellation); that is not a mismatch.

## 10. Failure table

| Symptom | Cause | Fix |
| --- | --- | --- |
| `FeatureCut4` / extrude returns `None` | wrong flags, wrong direction, sketch not selected | flags in 8.2; test `reverse`; `feat.Select2(False,0)` |
| Boss goes the wrong way | default is along the normal | `reverse=True` |
| Cut removes nothing / fails | default is **against** the normal | `reverse=True` |
| Offset plane lands on the wrong side | `InsertRefPlane` ignores flips | `CreatePlaneAtOffset3(d, flip, True)` |
| Equation `Add2` returns −1 | unquoted variable, negative value, unknown name | quote names; use positive magnitudes |
| Dimension equation shows 0.0 / model stops following a variable | variable was deleted and re-added | `set_var` |
| Sketch status 6 | wrong relation (often a symmetric pair) | recheck which end points were picked |
| `Select4`: "Type mismatch" | bare `CDispatch` point | wrap as `ISketchPoint` |
| `Select4`: "Member not found" on a body | wrong interface | `IBody2.Select2(append, data)` |
| Pattern returns `None` | feature creates a new body / names not selectable / axis mark | pattern the bodies (8.4); axis mark 1; features mark 4 |
| Patterned cutters fused into the part | geometry pattern | delete (`delete_from`), use a body pattern |
| Chamfer has zero size | single-call chamfer is a no-op | `chamfer_equal` (create + edit data) |
| `CreateSelectData` "member not found" | it lives on `ISelectionMgr` | `wrap(m.SelectionManager, sld.ISelectionMgr)` |
| Arrays rejected | plain Python list | `VARIANT(VT_ARRAY|VT_DISPATCH / VT_R8, …)` |
| First dimension hangs | modal Modify popup | `swInputDimValOnCreate = False` |
| `Select by name` fails after you reused a name | SolidWorks kept a generic name (`Revolve7`) | rename immediately; read the returned name |
| Feature "succeeded" but geometry wrong | `GetErrorCode2` only reports rebuild errors | check volume and coordinates |
| Rebuild feels slow on big parts | graphics/feature tree updates | `FeatureManager.EnableFeatureTree = False` and `ModelView.EnableGraphicsUpdate = False` during batches (restore both); queue equations with `Add2(…, False)` then `EvaluateAll` |

## 11. Quick checklist before you hand a part over

- [ ] `audit(m)`: no errors, all sketches 3, no dead equations, `Check2()==0`.
- [ ] Every repeated element is a pattern/mirror; no duplicated sketches for identical features.
- [ ] Every feature and plane has a descriptive name; every variable has a comment.
- [ ] Changing a few variables (with `set_var`) rebuilds without errors and the volume moves as predicted; restore the originals.
- [ ] Final comparison against the reference (boolean both ways, or documented expected differences).
- [ ] `swInputDimValOnCreate` restored to True; no unrelated documents closed or saved; part left unsaved unless asked.

## 12. Multi-part notes carried over from the earlier SolidWorks work (not re-tested in the single-part session)

- **Shared coordinates:** keep common fastener patterns in one master part (`…_master_layout.SLDPRT`). Slave parts pull its sketches with
  `IPartDoc.InsertPart3(path, 64 | 1024, '')` (sketches and their dimensions, no solids): a live native reference, not imported geometry.
  Reorder the link feature before the slave's first construction sketch. Delete the slave circle centre's coincident to its old origin, then
  `sgCONCENTRIC` it with the derived master circle: status stays 3. Model-origin dimensions do not transfer: dimension to a second
  construction circle that is coincident with the origin. A dimension of 0 is refused: dimension coordinates from a nonzero construction
  datum. Keep the reference one-way: master → parts → assembly.
- **Cross-part equations need two rebuild passes** after a master edit (first updates derived sketch dimensions, second evaluates consumers).
  Test real tool positions instead of trusting `EditRebuild3()==True`.
- **Saving:** save master, affected parts, then containing assemblies, bottom-up. A saved part with unsaved parent mates is not a checkpoint.
- **Assembly verification:** read component transforms back after moving through the range; `EditRebuild3()` returning True does not prove motion.
  A failed `AddMate` can leave a broken mate: delete only that mate and restore the last valid pose. A limit angle crossing 0 is solver-ambiguous: use an
  offset 90° helper driver and suppress it afterwards.
- **Move/Copy Body** cannot translate and rotate in one feature (rotate, then translate); the positional `RotAngleX/Y/Z` arguments of
  `InsertMoveCopyBody2` come out as Z/Y/X (pass them reversed and check); combined rotations are intrinsic XYZ; its dimensions are unsigned.
- Arrays of bodies must be `VT_ARRAY|VT_DISPATCH`; numeric arrays `VT_ARRAY|VT_R8`.
- Match disjoint boolean operands by minimum-distance centre assignment, not by sorted order, when symmetric bodies can permute.
- Stock/derived guide sketches report status 2 on copies (model-origin relations don't transfer); require 3 only on editable sketches.
- Set `ISldWorks.CommandInProgress = True` for long API runs and restore it; disable the feature tree and `EnableGraphicsUpdate` while changing many
  features (restore both before saving).

## 13. Helper library (`swhelpers.py`)

Copy this block to a scratch folder as `swhelpers.py` and run scripts with the cadenv Python from that folder. It was smoke-tested end to end
(variables, sketches, boss, cut, pattern, `set_var`, `audit`).

```python
"""swhelpers.py - SolidWorks 2023 helpers for building human-editable parts. Run with C:/Users/ferna/cadenv/Scripts/python.exe.
All lengths in the helpers are mm (the API itself is metres); angles in degrees at the helper level, radians in the API."""
import sys, math, re
sys.stdout.reconfigure(encoding='utf-8', line_buffering=True)
import pythoncom, win32com.client as wc
from win32com.client import gencache, constants as c
sld = gencache.EnsureModule('{83A33D31-27C5-11CE-BFD4-00400513BB57}', 0, 31, 0)
gencache.EnsureModule('{4687F359-55D0-4CD3-B6CF-2EB42C11F989}', 0, 31, 0)
MM = .001
ORIGIN = ('Point1@Origin', 'EXTSKETCHPOINT')


def wrap(o, cls):
    """bare COM object -> typed wrapper (needed for most returned objects)"""
    if o is None: return None
    d = o._oleobj_
    try: d = d.QueryInterface(cls.CLSID, pythoncom.IID_IDispatch)
    except pythoncom.com_error: pass
    return cls(d)


def connect():
    try: raw = wc.GetActiveObject('SldWorks.Application')
    except pythoncom.com_error: raw = wc.Dispatch('SldWorks.Application')
    sw = sld.ISldWorks(raw._oleobj_); sw.Visible = True
    return sw


def new_part(sw, title):
    """new unsaved part in mm. Closes only an open document with the SAME title (yours from an earlier run)."""
    for raw in sw.GetDocuments() or []:
        d = wrap(raw, sld.IModelDoc2)
        if d.GetTitle().replace('.SLDPRT', '') == title: sw.CloseDoc(d.GetTitle())
    m = wrap(sw.NewDocument(sw.GetUserPreferenceStringValue(c.swDefaultTemplatePart), 0, 0, 0), sld.IModelDoc2)
    m.SetUserPreferenceIntegerValue(c.swUnitsLinear, c.swMM)
    m.SetTitle2(title)
    sw.SetUserPreferenceToggle(c.swInputDimValOnCreate, False)      # no modal "Modify" popup; restore True when done
    return m


def open_part(sw, title):
    for raw in sw.GetDocuments() or []:
        d = wrap(raw, sld.IModelDoc2)
        if d.GetTitle().replace('.SLDPRT', '') == title: return d
    raise RuntimeError("part not open: " + title)


def disp_arr(objs): return wc.VARIANT(pythoncom.VT_ARRAY | pythoncom.VT_DISPATCH, objs)
def num_arr(x): return wc.VARIANT(pythoncom.VT_ARRAY | pythoncom.VT_R8, x)


def features(m):
    out, f = [], wrap(m.FirstFeature(), sld.IFeature)
    while f:
        out.append(f); f = wrap(f.GetNextFeature(), sld.IFeature)
    return out


def feat_by_name(m, name):
    return next((f for f in features(m) if f.Name == name), None)


def last_feature(m): return wrap(m.FeatureByPositionReverse(0), sld.IFeature)


def named(m, name):
    """rename the feature just created"""
    f = last_feature(m); f.Name = name; return f


def select(m, name, typ, mark=0, append=False):
    return m.Extension.SelectByID2(name, typ, 0, 0, 0, append, mark, None, 0)


def mark_data(m, mark):
    d = wrap(m.SelectionManager, sld.ISelectionMgr).CreateSelectData(); d.Mark = mark; return d


def bodies(m): return [wrap(b, sld.IBody2) for b in (wrap(m, sld.IPartDoc).GetBodies2(0, False) or [])]
def volume_mm3(m): return sum(b.GetMassProperties(1)[3] * 1e9 for b in bodies(m))


# ---------------------------------------------------------------- equations (global variables + dimension links)
class Eqs:
    def __init__(self, m):
        self.m = m; self.mgr = wrap(m.GetEquationMgr(), sld.IEquationMgr)

    def var(self, name, value, unit="mm", comment=""):
        """global variable. unit 'mm' or 'deg' or ''. comment becomes the equation's comment."""
        txt = '"%s"= %s%s' % (name, value, unit) + ((" '" + comment) if comment else "")
        for i in range(self.mgr.GetCount()):
            if self.mgr.Equation(i).startswith('"%s"' % name):
                self.mgr.Delete(i); self.mgr.Add2(i, txt, False); return      # NB: dependents lose their link -> use set_var
        self.mgr.Add2(-1, txt, False)

    def link(self, dimname, expr):
        """drive a dimension (e.g. 'D1@Sketch name') by an expression of variables, written WITHOUT quotes: 'a - b / 2'"""
        q = re.sub(r'[A-Za-z_][A-Za-z0-9_]*', lambda t: t.group(0) if t.group(0) in ('sqr', 'sin', 'cos', 'tan', 'abs', 'deg') else '"%s"' % t.group(0), expr)
        txt = '"%s"= %s' % (dimname, q)
        if self.mgr.Add2(-1, txt, False) < 0: raise RuntimeError('equation refused: ' + txt)

    def solve(self):
        self.mgr.EvaluateAll()


def set_var(m, name, value, unit="mm"):
    """change a global variable AND re-add every equation that references it (a plain edit breaks the dependents)"""
    mgr = wrap(m.GetEquationMgr(), sld.IEquationMgr)
    deps, idx, old = [], None, ""
    for i in range(mgr.GetCount()):
        t = mgr.Equation(i)
        if t.startswith('"%s"' % name): idx, old = i, t
        elif ('"%s"' % name) in t.split('=', 1)[1]: deps.append((i, t))
    comment = (" '" + old.split(" '", 1)[1]) if " '" in old else ""
    mgr.Delete(idx); mgr.Add2(idx, '"%s"= %s%s%s' % (name, value, unit, comment), False)
    for i, t in deps: mgr.Delete(i); mgr.Add2(i, t, False)
    mgr.EvaluateAll(); m.ForceRebuild3(False)


def dead_equations(m):
    """dimension equations that evaluate to 0.0 are broken links"""
    mgr = wrap(m.GetEquationMgr(), sld.IEquationMgr)
    return [mgr.Equation(i) for i in range(mgr.GetCount()) if '@' in mgr.Equation(i) and mgr.Value(i) == 0.0]


def feature_own_dims(feat):
    """names ('D1@Feature') of the dimensions that belong to the feature itself, in order: depth first, start offset second (named like D19)"""
    out, dd = [], wrap(feat.GetFirstDisplayDimension(), sld.IDisplayDimension)
    while dd:
        parts = wrap(dd.GetDimension2(0), sld.IDimension).FullName.split('@')    # 'D1@Feature@Part.Part'
        if parts[1] == feat.Name: out.append(parts[0] + '@' + parts[1])
        dd = wrap(feat.GetNextDisplayDimension(dd), sld.IDisplayDimension)
    return out


# ---------------------------------------------------------------- sketches
class Sketch:
    """Coordinates in mm in SKETCH space. Datum-plane sketch axes (world): Front x=X,y=Y | Top x=X,y=-Z | Right x=-Z,y=Y."""

    def __init__(self, m, plane, name):
        self.m, self.name, self.sm = m, name, m.SketchManager
        m.ClearSelection2(True)
        if not select(m, plane, 'PLANE'): raise RuntimeError("plane not found: " + plane)
        self.sm.InsertSketch(True); self.sm.AddToDB = True       # AddToDB: no accidental snapping; chained lines still share points
        self.dims = []; self.sk = wrap(self.sm.ActiveSketch, sld.ISketch)

    def line(self, x1, y1, x2, y2):  return wrap(self.sm.CreateLine(x1 * MM, y1 * MM, 0, x2 * MM, y2 * MM, 0), sld.ISketchLine)
    def cline(self, x1, y1, x2, y2): return wrap(self.sm.CreateCenterLine(x1 * MM, y1 * MM, 0, x2 * MM, y2 * MM, 0), sld.ISketchLine)
    def circle(self, x, y, r):       return wrap(self.sm.CreateCircleByRadius(x * MM, y * MM, 0, r * MM), sld.ISketchArc)

    def arc(self, cx, cy, x1, y1, x2, y2, ccw=True):
        return wrap(self.sm.CreateArc(cx * MM, cy * MM, 0, x1 * MM, y1 * MM, 0, x2 * MM, y2 * MM, 0, 1 if ccw else -1), sld.ISketchArc)

    def polygon(self, cx, cy, vx, vy, sides):
        """regular polygon; the point (vx,vy) is a corner. Result: a construction circle TANGENT to the flats (r = across-flats/2)."""
        return self.sm.CreatePolygon(cx * MM, cy * MM, 0, vx * MM, vy * MM, 0, sides, True)

    def segments(self): return [wrap(s, sld.ISketchSegment) for s in (self.sk.GetSketchSegments() or [])]

    def pick(self, *objs, origin=False):
        self.m.ClearSelection2(True); first = True
        for o in objs:
            if type(o).__name__ == 'CDispatch': o = wrap(o, sld.ISketchPoint)                  # points come back untyped
            if isinstance(o, (sld.ISketchLine, sld.ISketchArc)): o = wrap(o, sld.ISketchSegment)  # Select4 lives on ISketchSegment
            o.Select4(not first, None); first = False
        if origin: self.m.Extension.SelectByID2(ORIGIN[0], ORIGIN[1], 0, 0, 0, not first, 0, None, 0)

    def rel(self, kind, *objs, origin=False):
        """kind: sgHORIZONTAL2D sgVERTICAL2D sgCOINCIDENT sgTANGENT sgCONCENTRIC sgSYMMETRIC (2 points + centreline) sgEQUAL sgCOLLINEAR
        sgATMIDDLE sgHORIZONTALPOINTS2D sgVERTICALPOINTS2D (points aligned with the origin / each other)"""
        self.pick(*objs, origin=origin); self.m.SketchAddConstraints(kind); self.m.ClearSelection2(True)

    def _rec(self, dd, expr):
        if dd is None: raise RuntimeError("dimension failed: " + str(expr))
        self.dims.append((wrap(wrap(dd, sld.IDisplayDimension).GetDimension2(0), sld.IDimension).Name, expr))

    def hdim(self, a, b, expr, at, origin_b=False):
        self.pick(a, *([] if origin_b else [b]), origin=origin_b); self._rec(self.m.AddHorizontalDimension2(at[0] * MM, at[1] * MM, 0), expr)

    def vdim(self, a, b, expr, at, origin_b=False):
        self.pick(a, *([] if origin_b else [b]), origin=origin_b); self._rec(self.m.AddVerticalDimension2(at[0] * MM, at[1] * MM, 0), expr)

    def ddim(self, circ, expr, at): self.pick(circ); self._rec(self.m.AddDiameterDimension2(at[0] * MM, at[1] * MM, 0), expr)
    def rdim(self, arc, expr, at): self.pick(arc); self._rec(self.m.AddRadialDimension2(at[0] * MM, at[1] * MM, 0), expr)

    def sdim(self, expr, at, *objs, origin=False):
        """smart dimension: one line = length, two lines = angle (put `at` inside the wanted sector), circle = diameter"""
        self.pick(*objs, origin=origin); self._rec(self.m.AddDimension2(at[0] * MM, at[1] * MM, 0), expr)

    def finish(self, eqs=None):
        """exit, name the sketch, and link every recorded dimension to its expression"""
        self.sm.AddToDB = False; self.sm.InsertSketch(True)
        self.feat = last_feature(self.m); self.feat.Name = self.name
        if eqs is not None:
            for short, expr in self.dims: eqs.link("%s@%s" % (short, self.name), expr)
            eqs.solve(); self.m.EditRebuild3()
        return self.feat

    def status(self):
        """3 = fully constrained, 2 = under, 6 = both over- and under-constrained entities (a wrong relation)"""
        return wrap(self.feat.GetSpecificFeature2(), sld.ISketch).GetConstrainedStatus()


def polar_circle_sketch(m, E, plane, name, r_mm, ang_deg, d_mm, r_expr, ang_expr, d_expr):
    """Right-plane-based sketch: circle at polar (radius, angle from world +Y toward +Z), via a radial construction line, a vertical
    reference line and an angle dimension. Pattern it afterwards with a circular pattern."""
    a = math.radians(ang_deg); cx, cy = -r_mm * math.sin(a), r_mm * math.cos(a)
    s = Sketch(m, plane, name)
    ref = s.cline(0, 0, 0, r_mm * 1.3); rad = s.cline(0, 0, cx, cy)
    s.rel('sgVERTICAL2D', ref); s.rel('sgCOINCIDENT', ref.GetStartPoint2(), origin=True); s.rel('sgCOINCIDENT', rad.GetStartPoint2(), origin=True)
    circ = s.circle(cx, cy, d_mm / 2); s.rel('sgCOINCIDENT', circ.GetCenterPoint2(), rad.GetEndPoint2())
    s.sdim(r_expr, (cx - 4, cy / 2), rad); s.sdim(ang_expr, (cx / 3, cy * 0.6), rad, ref); s.ddim(circ, d_expr, (cx - d_mm, cy + d_mm))
    return s


# ---------------------------------------------------------------- reference geometry
def ref_plane(m, E, base, dist_mm, name, expr):
    """plane offset from a datum plane; the sign of dist picks the side (the equation then drives the positive magnitude)"""
    m.ClearSelection2(True); select(m, base, 'PLANE')
    if m.CreatePlaneAtOffset3(abs(dist_mm) * MM, dist_mm < 0, True) is None: raise RuntimeError("plane failed")
    named(m, name); E.link("D1@" + name, expr)


def make_axis(m, name, plane_a='Front Plane', plane_b='Top Plane'):
    """axis = intersection of two datum planes (Front + Top = the X axis)"""
    m.ClearSelection2(True); select(m, plane_a, 'PLANE'); select(m, plane_b, 'PLANE', 0, True)
    m.InsertAxis2(True); return named(m, name)


# ---------------------------------------------------------------- features
def boss_extrude(m, depth_mm, reverse=False, mid=False, start_offset=None, flip_start=False, name=None):
    """closed sketch must be selected. Default direction = ALONG the sketch-plane normal. mid: depth is the total length."""
    f = m.FeatureManager.FeatureExtrusion3(True, False, reverse, 6 if mid else 0, 0, depth_mm * MM, 0, False, False, False, False, 0, 0, False, False, False, False,
                                           True, True, True, 3 if start_offset is not None else 0, (start_offset or 0) * MM, flip_start)
    if f is None: raise RuntimeError("boss failed: " + str(name))
    return named(m, name) if name else last_feature(m)


def cut_extrude(m, depth_mm=0, through=False, mid=False, reverse=False, start_offset=None, flip_start=False, name=None):
    """default direction = OPPOSITE the sketch-plane normal. The three False flags (NormalCut, UseFeatScope, UseAutoSelect) are required."""
    f = m.FeatureManager.FeatureCut4(True, False, reverse, 6 if mid else (1 if through else 0), 0, depth_mm * MM, 0, False, False, False, False, 0, 0,
                                     False, False, False, False, False, False, False, True, False, False,
                                     3 if start_offset is not None else 0, (start_offset or 0) * MM, flip_start, False)
    if f is None: raise RuntimeError("cut failed: " + str(name))
    return named(m, name) if name else last_feature(m)


def revolve(m, as_new_body=False, name=None):
    """select the sketch (mark 0) first, then the axis line (mark 4 via mark_data). as_new_body = tool body, no merge."""
    f = m.FeatureManager.FeatureRevolve2(True, True, False, False, False, False, 0, 0, 2 * math.pi, 0, False, False, 0, 0, 0, 0, 0, not as_new_body, True, True)
    if f is None: raise RuntimeError("revolve failed")
    return named(m, name) if name else last_feature(m)


def circ_pattern(m, axis_name, feature_names, n, name, total_deg=360.0):
    m.ClearSelection2(True)
    if not select(m, axis_name, 'AXIS', 1): raise RuntimeError("axis " + axis_name)
    for fn in feature_names:
        if not select(m, fn, 'BODYFEATURE', 4, True): raise RuntimeError("feature " + fn)
    if m.FeatureManager.FeatureCircularPattern5(n, math.radians(total_deg), False, "", False, True, False, False, False, False, 1, 0, "", False) is None:
        raise RuntimeError("pattern failed: " + name)
    return named(m, name)


def circ_pattern_bodies(m, axis_name, body, n, name):
    """pattern a tool BODY (a geometry pattern would fuse the copies into the main body)"""
    m.ClearSelection2(True); select(m, axis_name, 'AXIS', 1); body.Select2(True, mark_data(m, 256))
    if m.FeatureManager.FeatureCircularPattern5(n, 2 * math.pi, False, "", False, True, False, False, False, False, 1, 0, "", False) is None:
        raise RuntimeError("body pattern failed")
    return named(m, name)


def mirror_features(m, plane, feature_names, name):
    m.ClearSelection2(True); select(m, plane, 'PLANE', 2)
    for fn in feature_names: select(m, fn, 'BODYFEATURE', 1, True)
    if m.FeatureManager.InsertMirrorFeature2(False, False, True, False, 0) is None: raise RuntimeError("mirror failed")
    return named(m, name)


def combine_cut(m, main_body, tool_bodies, name):
    m.ClearSelection2(True)
    if m.FeatureManager.InsertCombineFeature(15902, main_body, disp_arr([t._oleobj_ for t in tool_bodies])) is None: raise RuntimeError("combine failed")
    return named(m, name)


# ---------------------------------------------------------------- edges, fillets, chamfers
def edge_geom(e):
    """('line', p0, p1) or ('circle', centre, axis, radius), mm world coordinates"""
    cv = wrap(e.GetCurve(), sld.ICurve)
    if cv.IsLine():
        a = wrap(e.GetStartVertex(), sld.IVertex).GetPoint(); b = wrap(e.GetEndVertex(), sld.IVertex).GetPoint()
        return ('line', [v * 1000 for v in a], [v * 1000 for v in b])
    if cv.IsCircle():
        cp = cv.CircleParams
        return ('circle', [v * 1000 for v in cp[0:3]], list(cp[3:6]), cp[6] * 1000)
    return ('other',)


def edges_where(body, pred):
    return [e for e in (wrap(x, sld.IEdge) for x in (body.GetEdges() or [])) if pred(edge_geom(e))]


def select_edges(m, edges, mark=1):
    m.ClearSelection2(True); sd = mark_data(m, mark)
    for i, e in enumerate(edges): wrap(e, sld.IEntity).Select4(i > 0, sd)


def fillet_uniform(m, r_mm, name):
    if m.FeatureManager.FeatureFillet(2, r_mm * MM, 0, 0, None, None, None) is None: raise RuntimeError("fillet failed")
    return named(m, name)


def chamfer_equal(m, dist_mm, name):
    """inserting alone makes a zero-distance no-op: create it to keep the selection, then edit its definition"""
    f = m.FeatureManager.InsertFeatureChamfer(4, 16, 0, 0, dist_mm * MM, 0, 0, 0)
    if f is None: raise RuntimeError("chamfer failed")
    cd = wrap(f.GetDefinition(), sld.IChamferFeatureData2)
    if not cd.AccessSelections(m, None): raise RuntimeError("chamfer selection access")
    cd.Type = 2; cd.EqualDistance = True; cd.SetEdgeChamferDistance(0, dist_mm * MM); cd.SetEdgeChamferDistance(1, dist_mm * MM); cd.TangentPropagation = False
    if not f.ModifyDefinition(cd, m, None): raise RuntimeError("chamfer definition")
    return named(m, name)


def delete_from(m, name):
    """delete the named feature and everything after it (redo a stage); also drops equations of the deleted features"""
    fs = features(m); names = [f.Name for f in fs]
    if name not in names: return
    for f in reversed(fs[names.index(name):]):
        m.ClearSelection2(True); f.Select2(False, 0); m.Extension.DeleteSelection2(0)
    m.EditRebuild3()
    mgr = wrap(m.GetEquationMgr(), sld.IEquationMgr); alive = {f.Name for f in features(m)}
    for k in range(mgr.GetCount() - 1, -1, -1):
        t = mgr.Equation(k)
        if t.startswith('"D') and '@' in t and t.split('@')[1].split('"')[0] not in alive: mgr.Delete(k)


def audit(m):
    """print problems: feature errors, sketches not fully constrained, dead equations, invalid bodies"""
    fs = features(m)
    print("feature errors:", [(f.Name, f.GetErrorCode2()) for f in fs if f.GetErrorCode2()[0]])
    print("sketches not fully constrained:", [(f.Name, wrap(f.GetSpecificFeature2(), sld.ISketch).GetConstrainedStatus()) for f in fs
                                             if f.GetTypeName2() == 'ProfileFeature' and wrap(f.GetSpecificFeature2(), sld.ISketch).GetConstrainedStatus() != 3])
    print("dead equations:", dead_equations(m))
    print("bodies (mm3, Check2 == 0 is valid):", [(round(b.GetMassProperties(1)[3] * 1e9, 1), b.Check2()) for b in bodies(m)])
```

### 13.1 Minimal example (the smoke test)

```python
from swhelpers import *

sw = connect()
m = new_part(sw, "smoke_test")                      # unsaved, mm, popup off
E = Eqs(m)
E.var("r_out", 20, "mm", "outer radius")
E.var("hole_d", 4, "mm", "hole diameter")
E.var("h", 10, "mm", "height")
make_axis(m, "Z Axis", 'Top Plane', 'Right Plane')  # axis for the pattern

s = Sketch(m, "Front Plane", "Disc Sketch")
c1 = s.circle(0, 0, 20)
s.rel('sgCOINCIDENT', c1.GetCenterPoint2(), origin=True)
s.ddim(c1, "r_out * 2", (25, 25))
s.finish(E)                                         # exits, names, links dimensions to variables
assert s.status() == 3
m.ClearSelection2(True); s.feat.Select2(False, 0)
boss_extrude(m, 10, name="Disc")                    # default: along the Front plane normal (+Z)
E.link("D1@Disc", "h"); E.solve(); m.EditRebuild3()

hs = Sketch(m, "Front Plane", "Hole Sketch")
c2 = hs.circle(12, 0, 2)
hs.hdim(c2.GetCenterPoint2(), None, "r_out - 8", (12, -6), origin_b=True)
hs.rel('sgHORIZONTALPOINTS2D', c2.GetCenterPoint2(), origin=True)
hs.ddim(c2, "hole_d", (16, 4))
hs.finish(E)
m.ClearSelection2(True); hs.feat.Select2(False, 0)
cut_extrude(m, 10, through=True, reverse=True, name="Hole")   # cuts default to -Z, the disc is at +Z: reverse
circ_pattern(m, "Z Axis", ["Hole"], 4, "Hole Pattern x4")
set_var(m, "hole_d", 6)                              # edit a variable; dependents are re-linked
audit(m)
sw.SetUserPreferenceToggle(c.swInputDimValOnCreate, True)
```

Expected: volumes 12,440.7 (one hole) → 12,063.7 (four holes) → 11,435.4 mm³ (holes 6 mm), no audit findings.

## 14. Further verified API findings from the v10 design-intent rebuild (Nov 1)

- `tools/human/common.py` keeps an explicit registry of documents created by this task. Its `start()` reuses only those documents and refuses an unmanaged name collision. Replacement parts have the distinct `V10H_` prefix. Fernando's `V10_hubA_human.SLDPRT` is opened read-only for inspection and never saved.
- Reject every unlinked dimension in the audit, including plane offsets, revolve angles, pattern counts and chamfer distances. Check body validity and sketch status separately. A successful save does not establish that the whole assembly is finished.
- The literal global name `thickness` was refused on this installation. A descriptive name such as `retainer_thickness` works. Check that `Add2` actually created the requested variable.
- Do not delete and re-add an unchanged global while constructing a later stage. That can sever existing dependent equations. Compare its current literal first; use `set_var` for intentional edits.
- Construction-axis endpoints must not land on physical profile vertices. A 10 mm reference half-span merged with the end of a 10 mm screw, producing an equation feature error 51 despite sketch status 3. A deliberately distinct 10.123 mm half-span avoids the collision.
- Concentric arcs should share one centre control: constrain the second arc concentric to the first and dimension only its radius. Independently dimensioning both coincident centres produced equation error 51 in the bench clearance sketch. Sketch status alone did not expose it.
- Unequal-distance chamfers on opposite ends of a cylinder can have reversed selected-face ordering. Applying the same distance pair to both edges failed. Create one end, verify the actual tip radius, then create the other with the same named controls and verify its radius. Mirroring the unequal core-tip chamfer was also refused on this geometry.
- For concave boss-root fillets, a radius/centre edge filter is insufficient when a boss partly overhangs a cutout. Two arc edges may have identical circle parameters: one is the concave web junction, the other the convex free underside. Inspect `IEdge.GetTwoAdjacentFaces2()` and the adjacent planar face's `IFace2.Normal`; select only the concave junction. This fixed the inner fork's native shock-boss root fillet.
- `CreatePlaneAtAngle3` works with a datum plane and the reference axis selected. With Front then the X axis, `FlipDir=True` gave the intended positive rotation about X in the v10 hub. Read the sketch transform to verify the sign before cutting. Its angle is a normal editable plane dimension.
- Tangent hulls of round lobes can be made as actual sketch arcs plus tangent connecting lines. Dimension each arc centre/radius, constrain both ends' connecting lines tangent, and verify status 3. Sharp corner vertices get coordinate dimensions. This replaces a many-facet convex-hull polygon with a readable native outline.
- Keep independent spatial checks. A misplaced mounting hole has the right volume. Use the frozen assembly manifest and master-coordinate plan for screw locations, then check the exported native geometry through travel. The true-arc v10 wheel measured at least 1.177 mm of core-body/TPU clearance at both axial stops (0.5 mm required).
- Activate the target document **before obtaining/editing its equation manager**. `IEquationMgr.SetEquation` silently left the old equation unchanged for a background document in this session. Activating it first made the same edit work. Read the equation back to verify acceptance; `EvaluateAll` and a successful save alone do not establish that the edit happened.
- Derive the master with `4 | 8 | 64 | 1024` when its guides use reference planes/axes (axes, planes, unabsorbed sketches, dimensions). Omitting the planes left a copied staggered-row sketch in its old orientation. Check master/slave sketch rotation matrices before adding concentric relations; nonparallel references can project the centre into an incorrect location without an immediate constraint error.
- A reference axis' direction cannot be inferred from its name. Creating it from Top then Front and Front then Top produced different angle-plane directions. Changing the selections of an existing axis did not reverse that direction. Inspect the actual sketch transform; edit the angle plane's `IRefPlaneFeatureData.ReverseDirection` when necessary.

## 15. Exporting STEP files (Fernando's rule, Nov 1)

- **When asked to export STEP files, export only the 3D printed parts.** No fasteners (screws, nuts), no washers, no bearings, no shafts or other purchased parts, unless he names them. This applies to assemblies too: leave the hardware out of the assembly STEP.
- "Only the wheel, no hub pieces" meant the printed wheel parts without the hubs: roller cores, roller treads and the bearing cartridge. Do not add the hubs, the axles or the screws that join them.
- Put the files in `step_export/<group>/` (one folder per request group, loose parts in `parts/` when there is also an assembly). Export parts from SolidWorks (`SaveAs3(path, 0, 3, ...)` on the open part, which keeps the colours). An assembly of printed parts can be written from the exported part files at their nominal positions with build123d (`tools/human/export_wheel_step.py` does this, with labels and colours) instead of suppressing components in the live assembly.
- Re-import each assembly STEP to check the solid count and bounding box before reporting it.

## 16. Purchased parts: model their mating connectors and harness (Fernando's rule, Nov 2)

- **Every purchased part with a connector is modelled with its mating plug in place, and with the leads leaving the plug, before any packaging around it is drawn.** Use the real STEP of the plug (vendor or measured), not a guessed box, and find its mating direction from the model. Model the leads from the plug's rear: their OD (VERIFY if not measured), a short straight and then the tightest allowed bend (sharp 90 deg is acceptable unless the part says otherwise).
- Keep about **4 mm clearance all round** the plug and leads to anything that moves, and route the leads to where they leave the assembly. Report the outermost radius/extent of plug + leads and the clearance in the version's checks; an overlap or a missing clearance is a clash, not an open item.
- Why: cad/v11 drew a compact hub around the AK40-10 assuming a tangential XT30 plug of guessed size. The real XT30(2+2) mates radially and with its leads reaches r 36 against a hub bore of r 30, which broke the motor-in-hub packaging after the hub had been designed around it.
- This applies to build123d STEP stages as well as native SolidWorks work, and also to sensors, cameras, servos, boards and batteries.

## 17. Assemblies from build123d STEPs (the cad/v11 pipeline)

Lessons from building the v11 pod and the whole proposed robot (660 components) in SolidWorks from imported STEPs. The bodies are imported, not native features.

- **Pipeline:** build123d scripts (`omni-robot-sim` venv, every dimension through `dims.py`) write STEPs and checks → `sw_parts.py` / `sw_robot_parts.py` write per-part STEPs and a `manifest.json`
  → cadenv `sw_build.py` / `sw_robot.py` run named stages (`import`, then component groups, then `test`). Reuse cad/v10's helpers (`subasm`, `mate_api`, `sw_native`) unchanged.
  After a geometry change regenerate the STEPs, then re-run the SolidWorks stages for the changed parts: the SolidWorks parts do not follow the STEPs on their own, so record in the README which were rebuilt.
- **Run a few stages per call, one big group per call.** SolidWorks leaks memory and died (RPC server unavailable) building several big groups in one call. Each stage first closes the robot documents
  (SolidWorks threw internal errors without that). Build the groups one per call (frame, mounts, bumper, reference, upper, electronics, arm, fasteners), then the top assembly and its test.
- **Saves** can fail with error 1 while Dropbox is still uploading the previous save; the scripts retry. (This is for the pipeline's own generated files; the rule of never saving over Fernando's documents in section 2 stands.)
- **Names and colours:** component names are SolidWorks' file names (`V11_motor-1`); scripts map them by file, not by display name. STEP colours are written per face, in sRGB (`tools/step_colour.py`); face-only colours are what survives.
- **Structure:** one sub-assembly per group (frame, mounts, upper, electronics, fasteners, each pod...). Make flexible only what must move (the wheel, motor and pods in v11); the rest stays fixed where build123d put it.
  Orient the top assembly (robot z = SolidWorks Y so the standard views are upright, front facing the Front view) and leave the sub-assemblies in their build123d frames.
- **Mates only where something moves.** Hinge: concentric on the pin + a coincident/Right plane to locate it axially. Rotating part: concentric with a small thrust gap. Sliding follower (a spring plunger): concentric in its guide + dome **tangent** to the pad it rides on.
  Do not mate the rest.
- **Put a travel limit on a driven follower, not on an angle.** An angle limit between fork and frame planes left the fork fully defined (it would not drag). A distance limit on the plunger worked (`Suspension_travel`).
  API `Transform2` cannot move a part held by a limit mate; a GUI-style drag (`IDragOperator`) can. Springs stay at their nominal length (they do not compress in SolidWorks).
- **Motion test, always written to a JSON next to the assembly:** set each end pose and the nominal with the limit suppressed, read the component transforms back, compare with the build123d numbers (v11: to 0.01 mm, no mate errors);
  then turn the limit on and drag past each end and check it stays inside the travel. Do this for both hands and every flexible instance. `EditRebuild3() == True` proves nothing (section 12).
- **Vendor parts** (D435, RPLIDAR, the SO-101's official STEP/STL): copy them into `parts/` when the source is newer and record their poses in `manifest.json`. Check which end of the model faces your plug or cable and mark it VERIFY if the vendor model lacks the port.
  Check that the vendor model actually has the holes your design assumed (the SO-101 base has none where v9 assumed four M3) and take positions from its official geometry, not from a guess.
- **Mirrored hands:** a pod built from mirrored STEPs is a reflection, not the real opposite-hand part. If a connector or lead position differs on the other hand (motor plug), say so as an open item and build the real one.
- **Print bed planes:** every printed part gets a `PRINT_BED (±X/Y/Z up)` reference plane at its bed face, and the README lists each part's orientation, supports and why (the table in `v11/README.md`, generated by `tools/print_beds.py`).
- **Fasteners are records**, not hand-placed geometry (`tools/hardware.py`: type, length, position, axis). The assembly instances magenta library parts (`fst_*`) from them, so counts per length can be listed (and checked against the BOM) from one source.
  A screw without a record, or a record that does not land, is a finding.
- **Not modelled means said so:** cables and the harness, strength, printing. A viewing assembly inherits the status of the proposals it shows; do not present it as an adopted or validated design.

## 18. Designing printed parts (rules and lessons from cad/v11)

- **Choose the print orientation first** and write it down with the part. Load in the layer planes, not across them: pivots, arms, spring seats and side-loaded walls lie so the bending is in-plane
  (print the part on its outer face, an L with its plane in the layer plane, a bar standing upright with bolts pre-compressing its layers across it). Compression members can print on end. Walls that bend across layers only locate; screws must hold.
  Never lift or load a part along its weak (layer) direction without saying so.
- **Avoid supports.** Overhang beyond 45° is the metric: report the overhang area as printed (mm², per part, with the hole roofs listed) in the checks, and fix large ones with 45° cones, gables and chamfered roofs.
  Fernando prefers plain round holes over teardrop points; small round-hole roofs are accepted bridges. Split big parts on a plane so each half prints flat on a big face without support.
- **Fit the 256 mm bed:** every printed part one valid solid that fits the bed (the chassis is nine parts with half-laps; the second floor is four quadrants). Check in the script, not by eye.
- **Fasteners:** M3 thread-forming into a Ø2.8 pilot, ≥ 6 mm engagement (count it per screw, between the pilot and clearance radii, in the check), Ø3.4 through, flush counterbores for pan or flat heads, and ≥ 2.2 mm of wall between a counterbore and a bearing seat.
  A long bolt that must clamp two parts passes the first in Ø3.4 and threads the second; a bolt that holds a layered part pre-compressed threads into a Ø2.9 pilot its whole length. Drive screws from where a driver reaches: check the access from the assembled side, not just the fit.
- **Service access is a requirement:** ask whether each module (a pod, battery, computer) can be removed without taking the robot apart, and sweep the part out along its path. v11's pods could not, because every vertical screw was under the second floor; that is a finding to fix, not a note to leave.
  Members and pocket walls are clipped about 1 mm off the path a pod's parts sweep going out.
- **Clearance to moving parts is checked in every pose** (droop / nominal / bump, both hands, the arm's planned moves): ≥ 3 mm for the chassis to anything that moves, about 4 mm for plugs and leads (section 16).
  Quote the number (2.86 mm is under a 3 mm rule: say so, do not round it). Use the official meshes swept through the planned motion for arm clearance; the planner's capsules missed the SO-101's shoulder bracket.
- **Hard stops and locating features:** a stop that acts near the pivot turns 0.2 mm of print error into a larger error at the end of travel (0.2 mm at a 23-26 mm lever moved the axle by about 0.6 mm). Put clearances (0.1-0.2 mm) where the fit matters, a seat 0.1 mm under the bearing OD, and note what was never tested (snap fits, TPU walking, how hard a part is to press on).
- **Fillets in OCC:** the kernel often refuses fillets on a finished solid (tangent seams, joins). Fillet each feature before it joins the body, exclude the print and mating faces, and report how many edges took (R0.8 on 15 of 29, R1 on 36 of 48). Do not claim all edges rounded.
- **Mark the status honestly:** nominal CAD, not printed; estimates and assumed rates (a spring rate, a thread depth in a vendor part) are VERIFY items, not measured. Proposals stay proposals until Fernando adopts them ("go with F"), and the parts that follow a changed part (the chassis follows the pod) must be regenerated and re-checked.
- **Every round gets a `check`** in its build script: no interference at every pose, one valid solid per part, fits the bed, every screw lands (count landed/total), clearances, overhang area. Rerun the full check after any change to a part another part depends on.
