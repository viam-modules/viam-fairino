# FR5 URDF provenance

`arm/fr5.urdf` is the standard FAIRINO **FR5** (5 kg payload, ~922 mm reach),
taken from the vendor's official description package:

- Source: [FAIR-INNOVATION/frcobot_ros2](https://github.com/FAIR-INNOVATION/frcobot_ros2)
  → `fairino_description/urdf/fairino5_v6.urdf` (`main` branch).

It is the vendor's SolidWorks export, used **unmodified** except that the mesh
`<filename>` paths are rewritten from `package://fairino_description/...` to this
module's repo-relative layout:

- visual → `meshes/fr5/visual/*.glb` — the vendor CAD meshes converted to glTF
  for the app's 3D scene viewer (`Get3DModels`).
- collision → `meshes/fr5/collision/*.STL` — the vendor CAD collision meshes.

Kinematics, joint limits, and inertials are the vendor's originals. They match
FAIRINO's published FR5 DH table (Collaborative Robot User Manual 3.9.4):

Standard DH transform per axis: `Rz(theta) * Tz(d) * Tx(a) * Rx(alpha)`.

| Joint | a (mm) | d (mm) | alpha | limit | max speed |
|---|---:|---:|---:|---:|---:|
| J1 | 0    | 152 | +pi/2 | +/-175 deg | 180 deg/s |
| J2 | -425 | 0   | 0     | -265..+85 deg | 180 deg/s |
| J3 | -395 | 0   | 0     | +/-162 deg | 180 deg/s |
| J4 | 0    | 102 | +pi/2 | -265..+85 deg | 183 deg/s |
| J5 | 0    | 102 | -pi/2 | +/-175 deg | 183 deg/s |
| J6 | 0    | 100 | 0     | +/-175 deg | 183 deg/s |

Unlike `arm/fr5wml.urdf` (which is generated from the DH table because no
correct FR5-WML CAD export was available), the FR5 uses the vendor's real
SolidWorks description directly, so its meshes are true CAD geometry aligned to
the kinematic frames.

The `meshes/fr5/visual/*.glb` files were recovered from this repo's own history
(they were committed earlier as the placeholder's `fairino5_v6` originals); the
`meshes/fr5/collision/*.STL` files are the vendor CAD collision meshes.
