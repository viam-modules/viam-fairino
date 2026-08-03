# Fairino FR5-WML: Kinematics, Geometries, Get3DModels

## Context

*(This section describes the starting state before any of this plan's work — see the STATUS
section below for where things actually ended up; some of it, notably the EndPosition decision
right below, was later superseded by what actually happened in Step 9.)*

`viam-fairino` has joint-position read/write working and tested on hardware. The last commit
started wiring in a kinematics model but never added the file that defines it
(`makeJointLimitModel` is referenced in `module.go:124` but doesn't exist anywhere in the repo),
so the module currently fails to compile with `undefined: makeJointLimitModel`. `Kinematics()`
today returns a placeholder that the code's own doc comment says is not trustworthy for FK or
geometry. `EndPosition`, `Geometries`, and `Get3DModels` are stubbed out behind `errPhase3`.

The goal of this scope is to replace all of that with a real kinematic model of the FR5-WML arm,
sourced from Fairino's own ROS description (`fairino_description/urdf/FR5WM.urdf`, confirmed to
be the FR5-WML — Fairino's own asset naming truncates it to "FR5WM" everywhere, filename and
`<robot name>` alike). That URDF is not directly usable as-is: it has a dangling `tool` joint
whose child link is commented out (with the mesh files it references never delivered), and its
visual meshes are `.dae` files that neither RDK's URDF parser nor its `Get3DModels`/3D-viewer
path can consume directly. This plan cleans up the source URDF, wires up
`Kinematics()`/`Geometries()`/`Get3DModels()` from that URDF, and adds an independent, non-RDK
verification step so the converted assets can be sanity-checked before they're trusted inside the
module — with the embedded-JSON kinematics source dobot/xarm both also ship added last, once the
URDF path is proven out (see Confirmed decisions below).

Confirmed decisions (from user):
- The dangling `tool`/`gripper_Link` joint becomes a zero-transform **flange** frame (xarm's
  `link_eef` pattern), not a hard stop at `wrist3_Link` (dobot's pattern) — reusing the original
  joint's exact origin (`xyz="0 0 0" rpy="0 0 1.570796"`, parent `wrist3_Link`).
- Kinematics will eventually follow the **dual-source pattern** (embedded default JSON + on-disk
  URDF behind a `use_urdf` flag, matching dobot/xarm), but sequenced differently: **the on-disk
  URDF is built and validated first and is the only source initially**; the embedded JSON is
  generated from the validated URDF as a later addition (see Step 10), not built in parallel.
- `EndPosition` will be implemented late in this scope (Step 9), not deferred out of scope.
  Decision: read `TCPPose` directly from the existing hardware status feed (`constants.go`
  `offTCPPose`, x/y/z mm + a/b/c deg) — mirrors how `JointPositions` already works (status feed,
  no model math) — rather than computing it via `s.model.Transform(inputs)`. A follow-on
  (explicitly out of scope for now, revisit once the URDF/model is trusted) is to additionally
  compute the pose via the kinematics model and cross-check/log divergence against the
  hardware-reported `TCPPose`, as a live, ongoing validation of the model against real hardware.

**Pacing: work through one step at a time. Stop after each step and wait for explicit go-ahead
before starting the next — do not auto-continue.**

## STATUS: source URDF is wrong — this pass validated the pipeline, not the final geometry

All 10 steps below are DONE and the module works end-to-end on hardware (kinematics, collision
geometry, 3D scene rendering, packaging). **But `FR5WM.urdf` turned out not to match the physical
FR5-WML's actual CAD** — visually, its links are noticeably too short compared to the real arm
(discovered by eye once the 3D render was working — see Step 3). `fairino5_v6.urdf` was checked as
an alternative and also rejected for the same reason (also too short/wrong proportions). Neither
of the two `<Model>WM`-suffixed URDFs Fairino ships (`FR5WM`, matching this arm's name minus the
"L") nor the plain standard-arm URDF is confirmed correct for the FR5-**WML** specifically — the
user is sourcing the correct one separately; it wasn't found in `fairino_test/frcobot_ros2/` as of
this writing.

**This whole pass should be treated as a dry run that proved the pipeline, not as final geometry.**
When the correct URDF is found, redo Steps 0-4 and Step 10 against it (see the redo checklist
below) — Steps 5-9's *code* (kinematics.go/module.go structure, the `Geometries`/`Get3DModels`
implementations, the RDK bug workarounds, the tests) should need only path/filename updates, not a
rewrite, since none of that logic is FR5WM-specific.

### Redo checklist, once the correct URDF is found

Reusable as-is (no FR5WM-specific assumptions baked in):
- `tools/render_baseline.py`, `tools/verify_urdf.py`, `tools/convert_dae_to_glb.py`,
  `tools/compare_dae_glb_meshes.py`, `tools/render_assembled_glb.py`, `tools/_render_utils.py`,
  `tools/render_glb_visual_check.py` — all take `--urdf`/`--out-dir`/similar args or are generic
  enough to point at a new source with minor edits (check each script's `DEFAULT_*` constants).
  `tools/.venv` + `tools/requirements.txt` still apply.
- `cmd/gen-kinematics-json` — genuinely generic (reads `arm/fr5wm.urdf`, writes
  `arm/fr5wm_kinematics.json`; only those two path constants would need renaming if the new arm's
  file naming differs).
- Every gotcha documented below (the RDK bugs/quirks are RDK bugs, not FR5WM-URDF bugs):
  - **Fixed-joint naming collision** (Step 1/module.go): if the new URDF also has a
    dangling/placeholder tool joint needing a flange-style replacement, the joint and its child
    link MUST have different names, or RDK's URDF→JSON converter self-loops (see the flange
    comment in whatever URDF replaces `arm/fr5wm.urdf`, and Step 1 below for the full mechanism).
  - **Mesh path resolution** (Step 4): RDK resolves relative/`package://` mesh paths via
    `filepath.Join(urdfDir, meshPath)` — meshes must be reachable that way from wherever the new
    urdf file lands in `arm/`, or the module fails to construct at all.
  - **STL-mesh-mislabeled-as-PLY bug** (Step 6, `module.go`'s `rebuildMeshForPLYBug`): this is
    generic (any STL-sourced mesh geometry hits it), so the workaround doesn't need to change —
    just confirm it's still being exercised (i.e. the new arm's collision geometry is still
    STL-sourced mesh, not primitives).
  - **JSON vs. URDF kinematics format for the app's joint-limit sliders** (Step 6/10): the
    embedded-JSON-as-default requirement (for the app widget's `getJointPositionLimits` to work)
    still applies regardless of which arm's geometry it is.

Needs redoing against the new source:
- Steps 0-3: baseline render, URDF cleanup (check for the same kind of dangling-joint/commented-out
  link issue, or different issues entirely), `.dae`→`.glb` conversion, independent verification.
- Step 4: re-verify mesh path resolution for whatever directory layout the new source uses.
- Step 10: regenerate `arm/fr5wm_kinematics.json` (or its renamed equivalent) via
  `cmd/gen-kinematics-json` from the new, validated URDF — check the resulting file size again;
  `decimationRatio` may need retuning for a differently-sized mesh set.
- If the new arm's file/link naming differs from `fr5wm`/`FR5WM`/`base_link`/`shoulder_Link`/etc.,
  every reference to those literal strings needs updating: `urdfRelPath`/`meshVisualDir` constants
  in `kinematics.go`/`module.go`, the `arm/` directory layout itself, `wantLinkNames` in
  `get3dmodels_test.go`, and the Makefile's `TAR_FILES`.

## Step 0 — Baseline visualization of the original, unmodified URDF + meshes — DONE

Before touching anything, render `fairino_test/frcobot_ros2/fairino_description/urdf/FR5WM.urdf`
and its `.dae`/`.STL` meshes exactly as Fairino shipped them (broken dangling `tool` joint and
all — this will likely need to be worked around just to get a renderer to load it, e.g. by
temporarily stripping the dangling joint in a throwaway copy rather than editing the real source).
This baseline render is what Step 3's end-to-end verification will later be compared against, so
the eventual cleaned-and-converted result can be checked against a known-good starting point
rather than only checked for internal consistency. Script lives in `tools/` (see Step 2).

## Step 1 — Clean up the source URDF (outside the module, in a staging location) — DONE

Work from a copy of `fairino_test/frcobot_ros2/fairino_description/urdf/FR5WM.urdf` and its
`meshes/FR5WM/{visual,collision}/` directory (14 files: 7 `.dae` visual + 7 `.STL` collision,
one pair per real link — `base_link`, `shoulder_Link`, `upperarm_Link`, `forearm_Link`,
`wrist1_Link`, `wrist2_Link`, `wrist3_Link`).

Edits to the URDF:
1. Delete the commented-out `gripper_Link` block (lines ~490-529) entirely — it references mesh
   files (`../tool/gripper.dae`, `../tool/gripper.STL`) that don't exist anywhere in the repo.
2. Replace the `tool` joint's `<child link="gripper_Link"/>` with a new massless, geometry-less
   `flange` link, and rename the joint accordingly (or keep `tool` as the joint name — either is
   fine, just keep the origin `xyz="0 0 0" rpy="0 0 1.570796"` and parent `wrist3_Link"` intact).
3. Leave all 6 revolute joints (`j1`-`j6`) and their `base_link`→`shoulder_Link`→`upperarm_Link`→
   `forearm_Link`→`wrist1_Link`→`wrist2_Link`→`wrist3_Link` chain untouched — verified valid.

## Step 2 — Convert visual meshes for Get3DModels — DONE

RDK's URDF loader (`referenceframe/model_urdf.go: buildMeshMapFromURDF`) only loads `.stl`/`.ply`
from `<collision><mesh>` tags — it never touches `<visual>` meshes, and doesn't support `.dae` at
all. The 7 `.STL` collision meshes need no conversion and drive `Kinematics()`/`Geometries()`
directly. The 7 `.dae` visual meshes are only needed for `Get3DModels`, which (per dobot/xarm
precedent) serves pre-baked binary glTF (`.glb`, `ContentType: "model/gltf-binary"`), not raw
mesh files. Convert `.dae` → `.glb` offline with a Python `trimesh` script (same tool dobot's
GLBs were baked with) — this is a one-time asset-generation step, run and committed once, not
runtime code. Put this script under `tools/` (e.g. `tools/convert_dae_to_glb.py`) — same directory
as the Step 0/Step 3 visualization and verification scripts.

## Step 3 — Independent verification (outside viam/RDK) — DONE

The visual sanity check below is what later caught the wrong-URDF issue (see STATUS section
above) — worth keeping even though it only made a difference once the 3D render was actually
working, several steps later.

Before trusting the cleaned URDF inside the module, validate it standalone. Scripts live in
`tools/` alongside the Step 0 baseline-visualization script and Step 2's conversion script, and
the visual sanity check here should render against the Step 0 baseline so the two can be compared
side by side (same joint configurations, same camera framing where practical):
- Structural validity: `check_urdf` (liburdfdom-tools) if available, otherwise a small Python
  script using `urdfpy` or `yourdfpy` to load the URDF and confirm every joint's parent/child
  resolves (this alone would have caught the original dangling `tool` joint).
- Joint limits cross-check: compare the parsed `<limit lower/upper>` per joint against the URDF
  source values in this plan's Step-1 table (no independent numeric source exists — the Fairino
  Python SDK only exposes live-robot getters like `GetJointSoftLimitDeg()`, no static tables — so
  the URDF is the effective source of truth; this check just guards against transcription error
  in later steps).
- Visual sanity check: load the cleaned URDF + STL/GLB meshes in `trimesh` or `pybullet`
  (`pybullet.loadURDF`), render the zero pose and 2-3 non-trivial joint configurations against
  the Step 0 baseline, and eyeball that links don't look flipped/offset and the flange frame sits
  where expected.
- These scripts live in `tools/`, not shipped inside `module.tar.gz` — their job is to catch
  mesh/URDF authoring mistakes before the URDF is trusted by the module at all.

## Step 4 — Module layout (mirrors `viam-dobot`/`viam-ufactory-xarm`) — DONE

Confirmed RDK's resolution: `buildMeshMapFromURDF` does `filepath.Join(urdfDir, meshPath)` for
any non-absolute mesh path — literally the URDF file's own directory. That killed the idea of
reusing the original `../meshes/FR5WM/...` walk-up paths, and also surfaced that RDK's own
`package://` handling (`normalizeURDFMeshPath`) only strips the package-name segment and still
joins against `urdfDir`, so it only works when meshes are direct children of the urdf's own
directory, not a sibling of a `urdf/` subdirectory (confirmed the hard way: this is exactly why
`fairino5_v6.urdf`'s `package://fairino_description/...` refs failed to resolve when tried).

Also settled while validating the `.dae`→`.glb` conversion (see Step 2 update below): `<visual>`
and `Get3DModels()` now share one canonical set of `.glb` files rather than shipping two copies,
since RDK's own URDF parser (`linkXML` in `model_urdf.go`) never reads `<visual>` at all — only
`<collision>`. Final layout, flat urdf placement (matches dobot/xarm), Fairino's original
`visual`/`collision` split preserved under a single `meshes/` tree:

```
arm/fr5wm.urdf                              # cleaned URDF, flat in arm/, no package:// or ../ paths
arm/meshes/fr5wm/collision/*.STL            # 7 collision meshes — <collision> AND Kinematics()/Geometries() read these
arm/meshes/fr5wm/visual/*.glb                # 7 visual meshes — <visual> AND Get3DModels() both read these (one copy)
arm/fr5wm_kinematics.json                   # generated in Step 10 (embedded JSON, added later), go:embed'd
```

All mesh paths verified to resolve exactly the way RDK's `filepath.Join(urdfDir, meshPath)` would,
via a standalone check (no rendering needed for this part, just existence).

## Step 5 — `kinematics.go` in `viamfairino` — DONE

This single new file both unblocks the compile error and delivers the real scope. URDF-only for
now — no `use_urdf` flag or dual-source branching yet, that's added in Step 10 once this is
validated:

- `makeModelFrame(name string) (referenceframe.Model, error)`: resolve
  `$VIAM_MODULE_ROOT/arm/fr5wm.urdf` and call
  `referenceframe.ParseModelXMLFile(path, name, meshDecimationRatios)`.
- Delete `makeJointLimitModel` and its call site — this function replaces it.
- Wire `s.model, err = makeModelFrame(name)` into the constructor/reconfigure path, same spot
  `makeJointLimitModel()` is called today (`module.go:124`).

## Step 6 — Implement `Kinematics`/`Geometries`/`Get3DModels` in `module.go` — DONE

`Geometries` is the dobot/xarm thin wrapper exactly as sketched: `CurrentInputs` →
`s.model.Geometries(inputs)` → `.Geometries()`. `Get3DModels` ended up simpler than dobot's
version since we're URDF-only with no `use_urdf` toggle yet (Step 10) — no dual frame-name
mapping table needed. Instead of a hardcoded per-link table like dobot's `cr10aMeshParts`, it
just lists `arm/meshes/fr5wm/visual/*.glb` and keys the map by filename minus extension, which
matches the URDF's own link names exactly since that's how the files were generated (Steps 2/4) —
avoids a second place that could drift out of sync with the model. Cached on the struct after the
first successful build (`meshMu sync.Mutex` + `meshCache`), with dobot's graceful
empty-map-plus-warning fallback when `VIAM_MODULE_ROOT` is unset.

`errPhase3` was renamed to `errEndPositionUndecided` since `EndPosition` (Step 9) is now its only
user — the old name stopped meaning anything once `Geometries`/`Get3DModels` no longer used it.

Leave `MoveToPosition`, `MoveThroughJointPositions`, `GoToInputs`, `Stop` stubbed as `errPhase2` —
out of scope here. `EndPosition` is handled separately in Step 9.

**RDK bug found and worked around — `Geometries()` broke the app's 3D scene viewer entirely**
(`getGeometries` error: `error reading mesh: Invalid ply file`). Root cause, confirmed via
`TestGeometryProtoRoundTrip` (`geometries_test.go`): `spatialmath.Mesh` loaded from STL has its
`rawBytes` field populated with the *original binary STL bytes*; `Mesh.ensurePLYBytes()` only
regenerates real PLY text when `rawBytes` is nil, so for an STL-sourced mesh it just returns the
raw STL bytes unchanged — and `Mesh.ToProtobuf()` unconditionally labels whatever
`ensurePLYBytes()` returns as `ContentType: "ply"` regardless. Net effect: `GetGeometries` sends
raw binary STL data mislabeled as PLY, and `goply` (RDK's PLY reader, whose own source comment
calls it "fragile") panics trying to read the STL binary header as PLY's `"ply"` magic string.
This isn't specific to our mesh data — it'd hit any module serving `Geometries()` for STL-sourced
collision meshes (dobot, xarm included) once round-tripped through `ToProtobuf()`. It's a genuine
RDK bug, but reporting it upstream is explicitly not this module's or this user's problem to chase
— the local workaround (below) is the intended permanent fix here, not a stopgap waiting on
upstream. Non-urgent, no action item.
Workaround added in `module.go`: `rebuildMeshForPLYBug` reconstructs each `*spatialmath.Mesh` via
`spatialmath.NewMesh(pose, triangles, label)` before returning it from `Geometries()` — that
constructor (unlike the STL/PLY-file ones) never sets `rawBytes`, so `ensurePLYBytes()` correctly
regenerates PLY on demand. `TestGeometryProtoRoundTrip` proves both halves: the raw model geometry
still exhibits the bug (so the test would flag it if that ever silently stops being true, e.g. if
RDK fixes it upstream and the workaround becomes removable dead code), and `Geometries()`'s actual
post-workaround output round-trips cleanly.

**App joint-limit slider gap — CONFIRMED FIXED by Step 10.** Originally found here: the Viam app's
joint-position sliders showed a generic ±180° range instead of real per-joint limits, because
`Kinematics()` came from URDF, and `@viamrobotics/test-widgets`'s `getJointPositionLimits` only
parses `kinematic_param_type === 'SVA'` — the SDK's `getKinematics()` throws trying to
`JSON.parse` the raw URDF text RDK sends for URDF-sourced models (`KinematicModelToProtobuf`,
`referenceframe/model.go:70-79`, tags wire format by `OriginalFile.Extension`: `"urdf"` → raw URDF
text, `"json"` → real JSON). Server-side `DoF()` was always correct — purely a frontend display
gap. Step 10 made the embedded JSON the default source, and the user confirmed in the app: the
sliders now show real per-joint ranges. No further action here.

## Step 7 — Packaging — DONE (moved up ahead of Step 6, caught by real-hardware reload testing)

`TAR_FILES` in `Makefile` now includes `arm/fr5wm.urdf` and `arm/meshes` (collision `.STL` +
visual `.glb`, per Step 4's layout) alongside `meta.json`/the binary/`$(FIRST_RUN)`, and
`module.tar.gz`'s prerequisites include `arm/fr5wm.urdf` so editing it triggers a rebuild. Found
this was missing when reloading onto real hardware: `VIAM_MODULE_ROOT/arm/fr5wm.urdf` didn't
exist because nothing packaged it — Step 5's `kinematics.go` code was correct, there was just
nothing shipping the file it looks for.

## Step 8 — Tests — DONE

`Geometries()` coverage came for free out of the Step 6 PLY-bug investigation:
`TestGeometryProtoRoundTrip` (`geometries_test.go`) already exercises real geometry at the zero
pose, asserts it's non-empty, and specifically guards the `rebuildMeshForPLYBug` fix (documents
the underlying RDK bug still reproducing on the raw model output, and that `Geometries()`'s actual
post-workaround output round-trips cleanly).

Added `get3dmodels_test.go` for the other originally-planned piece:
- `TestGet3DModelsUnsetModuleRoot` — empty map, no error, when `VIAM_MODULE_ROOT` is unset.
- `TestGet3DModelsReturnsAllLinkMeshes` — all 7 links present, correct `ContentType`
  (`model/gltf-binary`), and each mesh's bytes start with the real glTF magic header (`"glTF"`) —
  not just non-empty, actually plausibly-valid glb.
- `TestGet3DModelsCaches` — second call reuses the cache, and returned maps are independent copies
  (`cloneMeshMap`) so a caller mutating one can't corropt the cache or another caller's copy.

All of these construct a bare `&viamFairinoFairino{logger: ...}` directly rather than going
through `NewFairino`, since `Geometries`/`Get3DModels` only touch `model`/`meshCache`/`logger` —
no live hardware connection needed, matching dobot's own hardware-independent test style.

(The JSON/URDF FK-agreement test, dobot-style, is still deferred to Step 10 once the embedded
JSON exists — nothing to cross-check against until then.)

## Step 9 — `EndPosition` — DONE, staying with FK for now (decided, not just deferred)

Original plan (below) was to serve `TCPPose` from the status feed. Turned out to be moot for now:
hardware testing showed the app's 3D scene viewer calls `getEndPosition` and refuses to render
anything at all if it errors — so `EndPosition` wasn't a Step-9-later thing, it was blocking the
actual point of this whole scope (seeing the arm rendered). Implemented the minimal unblock
instead: `s.model.Transform(inputs)`, i.e. dobot's own exact `EndPosition` pattern
(`arm/cr10a.go`), reusing the already-validated kinematic model rather than chasing Fairino's
undocumented `TCPPose` Euler convention (`rx,ry,rz` — grepped `fairino-python-sdk`, no explicit
RPY/axis-order documentation found in Python source, since `GetForwardKin`/pose math is behind a
compiled extension; would need the manual PDF or hardware experimentation to pin down) under
time pressure to unblock rendering.

**Decided (user, this session): staying with the FK-based `s.model.Transform` implementation.**
Not revisiting the `TCPPose`-vs-FK question or Fairino's undocumented Euler convention unless a
concrete reason comes up (e.g. FK and the device-reported pose are ever observed to disagree on
hardware) — dobot uses exactly this FK approach unconditionally, so it's not a stopgap being
tolerated, it's a legitimate long-term design.

## Step 10 — Generate and wire in the embedded kinematics JSON (dual-source, added last) — DONE

- `cmd/gen-kinematics-json/main.go` (new): parses `arm/fr5wm.urdf` via `ParseModelXMLFile`, then
  **explicitly nils out `cfg.OriginalFile` before marshaling** — this field is a real, marshaled
  JSON key (`original_file,omitempty`), and if left set to the URDF's bytes/extension,
  `UnmarshalModelJSON`'s own `json.Unmarshal` call would let it overwrite that function's pre-set
  `OriginalFile{Extension: "json"}` on load. Missing this would silently defeat the entire point
  of this step: `GetKinematics` would still tag the wire format URDF instead of SVA, and the app's
  joint-limit sliders (the Step 6 finding) would still be broken even after switching to JSON.
- `kinematics.go`: `//go:embed arm/fr5wm_kinematics.json` into `fr5wmKinematicsJSON`.
  `makeModelFrame` gained a `useURDF bool` parameter — `false` (new default) parses the embedded
  JSON, `true` does the on-disk URDF path from Step 5 unchanged.
- `module.go`: `Config` gained `UseURDF bool` and `MeshDecimationRatios []float64` (URDF-mode
  only); `NewFairino`'s call site updated to `makeModelFrame(name.Name, conf.UseURDF,
  conf.MeshDecimationRatios)`.
- `kinematics_test.go` (new): `TestJSONURDFJointLimitsAgree` and
  `TestJSONURDFForwardKinematicsAgree`, dobot-style — build both sources, compare `DoF()` limits
  and `Transform()` output (zero pose + two non-trivial poses, ±20° per joint, comfortably inside
  even `j3`'s tightest limit of ±138°) within tolerance.

**File size**: undecimated, the generator produced a 21.5MB JSON (full STL triangle data
re-encoded as ASCII JSON — nothing like dobot/xarm's actual embedded default, which is hand-fitted
capsule/box primitives, never real mesh data). Added a `decimationRatio` constant to
`cmd/gen-kinematics-json` (applied to all 7 collision meshes, same ordering rule as
`ParseModelXMLFile`'s `meshDecimationRatios`); `0.1` (matching dobot's own URDF-mode default as a
first try) brought it to 693K, a ~30x reduction. Decided against chasing dobot's 5.3K further:
that gap is a representation difference (decimated mesh vs. hand-fit primitives), not a decimation
knob, and would need real new per-link primitive-fitting work (AABB computation, capsule
axis/orientation rules, box fallback for near-cubic links). 693K is a non-issue for a Go binary
already tens of MB, and parses in single-digit milliseconds — the actual problem (21.5MB) is
solved. `go test ./...` passes with the embedded JSON in place, including the
`TestJSONURDFForwardKinematicsAgree`/`TestJSONURDFJointLimitsAgree` guards.

## Redo pass 1 — swapped in a CAD-corrected URDF (fr5wml), still not the final one

The correct FR5-WML URDF still hasn't been found. In the meantime, the user manually edited a
copy of Fairino's `fairino5_v6.urdf` (the plain non-WM standard-arm variant, not `FR5WM.urdf`) to
fix 5 joint-origin translations (`j2`-`j6`) against a real CAD drawing — confirmed via `diff`
against the untouched source that ONLY those 5 lines changed, nothing else. **Meshes and
mass/inertia are still `fairino5_v6`'s originals and known-inaccurate** — link lengths are right,
visual/collision fidelity isn't. Also fixed one unrelated pre-existing bug in Fairino's own file:
an `<origins>` typo (should be `<origin>`) on the wrist2 collision block, confirmed present in the
untouched original too (not something the manual edit introduced).

Ran the equivalent of Steps 1-4/10 against this new source (Step 0/2/3's *tooling* — the
render/verify/convert scripts — was reused as-is, not re-run as formal gated steps, since this
pass moved faster once the pipeline was already proven):
- No dangling/commented joint this time (this URDF just ends cleanly at `wrist3_link`, unlike
  `FR5WM.urdf`'s broken `tool` joint) — no flange frame needed, no naming-collision risk.
- Mesh paths rewritten from `package://fairino_description/meshes/fairino5_v6/*.STL` (would hit
  the same RDK package:// resolution gotcha as before, since meshes/ and urdf/ are siblings there
  too) to the same flat `meshes/<name>/{visual,collision}/` scheme as before.
- Only `.STL` exists for this variant (no `.dae`) — visual `.glb`s generated by a new
  `tools/convert_stl_to_glb.py` (direct STL→glb, untextured/gray but geometrically correct),
  companion to the existing `convert_dae_to_glb.py`.
- **Renamed `fr5wm`→`fr5wml` throughout** (`arm/fr5wml.urdf`, `arm/meshes/fr5wml/`,
  `arm/fr5wml_kinematics.json`, `urdfRelPath`/`meshVisualDir` in `kinematics.go`/`module.go`,
  `cmd/gen-kinematics-json`, `Makefile` `TAR_FILES`, `wantLinkNames` in `get3dmodels_test.go`) —
  matches the real FR5-WML designation and this file's own lowercase link naming (`shoulder_link`,
  not `shoulder_Link`), and avoids confusion with the old, confirmed-wrong `FR5WM.urdf`/`fr5wm`
  artifacts, which were removed (git-tracked, recoverable). Old `arm/fr5wm_kinematics.json`
  deleted; regenerated fresh under the new name via `cmd/gen-kinematics-json`.
- Confirmed working end-to-end on hardware: 3D scene renders (faster too, once the embedded JSON
  was regenerated with real decimation applied to the new mesh set).

**Still not final** — same caveat as before, redo again once the correct FR5-WML source is found.

## Redo checklist update

The Step-0-13 redo checklist above was written before this pass happened and is now itself
slightly stale (e.g. mentions `fr5wm`/`FR5WM` literal strings to search for — now `fr5wml`). Any
future redo should diff against whatever's currently in `arm/` rather than trust the checklist's
literal old names.

## `MoveToPosition` via IK — motion.Service delegation, not self-contained armplanning

Implemented `MoveToPosition` (previously stubbed) using RDK's motion planner, but **not** via
`armplanning.MoveArm` inside this module directly (viam-dobot's pattern) — via delegating to a
`motion.Service` instead (viam-ufactory-xarm's pattern). Both give the same IK-solved,
self-collision-aware (via `Geometries()`) joint-space planning; the difference is only which
binary the `libnlopt`/cgo dependency ends up in.

**Why the switch, mid-implementation:** the `armplanning.MoveArm` version was implemented first
(matching dobot) and appeared to work, but crashed on hot-reload: `libnlopt.so.0: cannot open
shared object file`. Confirmed via direct inspection (not just docs) that:
- `go-nlopt` (the actual cgo binding `motionplan/ik/nlopt.go` uses) resolves nlopt via `pkg-config`
  at build time — nothing nlopt-related existed anywhere on the deployment host (no package, no
  `.so`, no `pkg-config` entry), so a *fresh* build here would fail to link, not just crash later.
- Confirmed xarm's actual `MoveToPosition` (not just its docs) does NOT call `armplanning.MoveArm`
  — it calls `x.motion.Move(...)`, keeping the whole `motionplan`/`ik`/`go-nlopt` import chain out
  of the arm module's own binary entirely. Confirmed this by checking a live xarm process's
  Makefile/source, not by assumption.
- The dependency doesn't disappear — RDK's `builtin` motion service (which normally runs inside
  viam-server itself) calls the exact same `armplanning.PlanMotion` internally. It moves off this
  module's build/deploy surface onto viam-server's, which Viam ships pre-built with nlopt already
  resolved — a real, deliberate relocation, not an illusion of avoiding the dependency.

**Implementation** (`module.go`):
- `Config.Motion string` (new): names the `motion.Service` to use; empty means "this robot's
  default `builtin` service if available." `Validate` declares it as a required dependency if
  named explicitly, or an *optional* one (`motion.Named("builtin")`) otherwise — this is what
  actually gets it into the `resource.Dependencies` map handed to `NewFairino`; the service
  existing on the robot isn't sufficient by itself (confirmed against RDK's dependency-resolution
  code, not assumed).
- `NewFairino` resolves it via `motion.FromProvider(deps, ...)`, non-fatal if the optional
  `builtin` fallback isn't found (logs and leaves `s.motion` nil — `MoveToPosition` then errors
  clearly at call time instead of failing construction).
- `MoveToPosition`: guards on `s.motion == nil`, then `s.motion.Move(ctx, motion.MoveReq{
  ComponentName: s.Name().Name, Destination: referenceframe.NewPoseInFrame(fmt.Sprintf("%s_origin",
  s.Name().Name), pose)})`.
- `GoToInputs` (previously stubbed) now delegates to `MoveThroughJointPositions` — **required**,
  not optional: confirmed the `builtin` motion service's plan executor calls `GoToInputs`
  specifically (via the `framesystem.InputEnabled` interface), not `MoveThroughJointPositions`
  directly, so this had to be filled in for `MoveToPosition` to work at all, not just for
  completeness.
- `moveToJointPositionsOnce` (the shared per-waypoint primitive, factored out of
  `MoveToJointPositions` for this work) deliberately doesn't call `s.opMgr.New` itself, so
  multi-waypoint callers (`MoveThroughJointPositions`, hence `GoToInputs`, hence `MoveToPosition`)
  can wrap the whole sequence in one operation — nesting `opMgr.New` per-waypoint would make each
  waypoint immediately cancel the one before it (`SingleOperationManager` semantics).
- `errPhase2` now only guards `Stop`.

**Critical requirement this surfaces, not fixable in module code**: `MoveToPosition` requires the
arm's robot-config JSON entry to include a `"frame"` block (minimum: `"parent"`, e.g. `"world"`).
Confirmed directly in RDK source: a component is only added to the frame system (and gets its
auto-generated `"<name>_origin"` frame, which `Destination` above targets) if its config has a
non-nil `Frame`, and the `builtin` motion service looks up `ComponentName` by exact string match
against the frame system — no fallback. Without this, `MoveToPosition` fails with `component named
<name> not found in robot frame system`. Documented in README.md's "Cartesian motion" section.
`MoveToJointPositions`/`MoveThroughJointPositions`/`GoToInputs` don't need this — only
`MoveToPosition` does, since only it goes through the frame-system-aware motion service.

**Not yet confirmed on hardware** (code written, not yet run) — verify: `go build`/`go vet`/`go
test` pass now that `armplanning` is gone from this module's imports (should also confirm the
`libnlopt` runtime crash is actually resolved, i.e. `go build`/module start succeed without the
system dependency installed), and that `MoveToPosition` actually drives a real Cartesian move on
hardware once the `"frame"` config block is added.

## Verification — all passed for the FR5WM pipeline pass; repeat these once the correct URDF lands

- `go build ./...`, `go vet ./...`, `go test ./...` all pass (confirmed by the user).
- The Step-3 verification scripts (`tools/verify_urdf.py`) passed against the cleaned URDF,
  compared against the Step 0 baseline render.
- Manual, on real hardware: confirmed in the Viam app's CONTROL/3D-scene view — the arm renders
  with correct mesh placement, `EndPosition`/`Geometries`/`Get3DModels` all work, and the
  joint-limit sliders show real per-joint ranges (embedded-JSON default from Step 10).
- Build/test commands throughout this plan were run by Paige, not automatically — that stays true
  for the redo pass too: report what needs running and wait for her to run it and report results,
  rather than executing builds/generators/test runs directly. Same for pacing: work through steps
  one at a time, stop after each, wait for explicit go-ahead before starting the next.
