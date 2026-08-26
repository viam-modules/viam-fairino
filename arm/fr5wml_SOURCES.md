# FR5-WML URDF provenance

`arm/fr5wml.urdf` is generated from FAIRINO's published FR5-WML Denavit-Hartenberg
table, not from a CAD export. It replaces an earlier placeholder that was a
manually link-corrected copy of Fairino's `fairino5_v6.urdf` (whose wrist
offsets, some joint limits, and rated velocities were still fairino5_v6's and
inaccurate for the FR5-WML).

## DH parameters used

Standard DH transform per axis: `Rz(q) * Tz(d) * Tx(a) * Rx(alpha)`.

| Joint | a (mm) | d (mm) | alpha | limit | max speed |
|---|---:|---:|---:|---:|---:|
| J1 | 180 | 0 | +pi/2 | +/-175 deg | 120 deg/s |
| J2 | 0 | -970 | 0 | -265..+85 deg | 120 deg/s |
| J3 | 0 | -816 | 0 | +/-170 deg | 180 deg/s |
| J4 | 159 | 0 | +pi/2 | -265..+85 deg | 180 deg/s |
| J5 | 114 | 0 | -pi/2 | +/-175 deg | 180 deg/s |
| J6 | 160 | 0 | 0 | +/-360 deg | 180 deg/s |

At all-zero joint angles the model places `tool0` about 1.888 m from the base
origin, consistent with the robot's ~1.9 m rated reach envelope.

## Meshes

Visual meshes (`meshes/fr5wml/visual/*.glb`) and collision meshes
(`meshes/fr5wml/collision/*.stl`) are **simplified generated meshes**, not
tessellations of FAIRINO's proprietary/official STEP geometry. They preserve the
link/joint locations and overall scale but not the exact castings, covers,
fasteners, cable ports, etc. If exact appearance/collision geometry is needed,
replace the mesh files while preserving the link frames in `fr5wml.urdf` —
FAIRINO/Inlux publicly expose an official FR5-WML STEP download.

The per-link GLBs were produced by merging the source package's `linkN_body.stl`
(fairino_white) and `linkN_ring.stl` (fairino_orange) into one colored GLB per
link, plus `base_link.glb`.

## Sources

- FAIRINO Collaborative Robot User Manual 3.9.4 — robot DH parameters, FR5-WML table:
  https://fairino-doc-de.readthedocs.io/3.9.4/CobotsManual/robot_brief_introduction.html
- FAIRINO FR5-WML product page:
  https://www.fairino.com/FR/11.html
- FAIRINO engineering/catalog drawing (FR5-WML on printed page 15/16):
  https://pub-mediabox-storage.rxweb-prd.com/exhibitor/document/exh-205293cc-94b3-471a-843c-876cda6ee4c6/7b338ba7-2471-4b2d-b1db-c19e19cd0401.pdf
- Official STEP download exposed by Inlux Robotics product page:
  https://inluxrobotics.eu/products/fairino-fr5-wml-cobot
