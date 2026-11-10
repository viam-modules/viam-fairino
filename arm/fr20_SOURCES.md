# FR20 URDF provenance

`arm/fr20.urdf` is the FAIRINO **FR20** (20 kg rated payload), taken from the
vendor's official description package:

- Source: [FAIR-INNOVATION/frcobot_ros2](https://github.com/FAIR-INNOVATION/frcobot_ros2)
  → `fairino_description/urdf/fairino20_v6.urdf` (`main` branch, commit
  `eb6d7ce`).

It is the vendor's SolidWorks export, used **unmodified** except for:

- Mesh `<filename>` paths rewritten from `package://fairino_description/...` to
  this module's repo-relative layout:
  - visual → `meshes/fr20/visual/*.glb` — the vendor CAD meshes
    (`fairino_description/meshes/fairino20_v6/*.STL`) converted to glTF with
    trimesh for the app's 3D scene viewer (`Get3DModels`), same as the FR5's.
  - collision → `meshes/fr20/collision/*.STL` — the vendor CAD meshes, as-is.
- A fixed `joint_eef` → `link_eef` flange frame appended 120 mm along
  `wrist3_link`'s z. The vendor URDF ends at the J6 axis and has no frame for
  the DH table's d6, so without it forward kinematics stops 120 mm short of the
  flange the controller reports `tl_cur_pos` at (tool 0). The 120 mm is the
  DH table's d6 and matches the vendor's `wrist3_link` mesh, whose flange face
  sits at z = 0.120.

Kinematics, joint limits, and inertials are the vendor's originals. They match
FAIRINO's published FR20 DH table (Collaborative Robot User Manual 3.9.4,
Table 2.4-9), including the link masses:

Standard DH transform per axis: `Rz(theta) * Tz(d) * Tx(a) * Rx(alpha)`.

| Joint | a (mm) | d (mm) | alpha | URDF limit |
|---|---:|---:|---:|---:|
| J1 | 0     | 215 | +pi/2 | +/-175 deg |
| J2 | -1000 | 0   | 0     | -265..+85 deg |
| J3 | -716  | 0   | 0     | +/-160 deg |
| J4 | 0     | 166 | +pi/2 | -265..+85 deg |
| J5 | 0     | 138 | -pi/2 | +/-175 deg |
| J6 | 0     | 120 | 0     | +/-175 deg |

The J1/J5/J6 ranges also match the manual's FR20 movement-range figure
(Figure 2.1-9). FAIRINO's separate Gazebo package
([FAIR-INNOVATION/fairino_gazebo](https://github.com/FAIR-INNOVATION/fairino_gazebo)
`fr20v6_ros2_control/urdf/fr20v6.urdf`) has identical link offsets and axes
but gives J3 as +/-162 deg — the FR5's value, apparently copied over. This
module uses `frcobot_ros2`'s more conservative +/-160 deg.

## Sources

- FAIRINO Collaborative Robot User Manual 3.9.4 — robot DH parameters, FR20 table:
  https://fairino-doc-en.readthedocs.io/3.9.4/CobotsManual/robot_brief_introduction.html
- FAIRINO robot model table (the `FR20-V1-001(V6.0)` robotModel string
  auto-detection matches): `FAIR-INNOVATION/FAIR-documentation`
  `source/CobotsManual/system.rst`, "Robot model configuration".
