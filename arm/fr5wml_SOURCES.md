# FR5-WML URDF provenance

`arm/fr5wml.urdf` is FAIRINO's own official FR5-WML URDF (vendor package
`FR5WML_urdf/FR5WML.urdf`), adapted for this module: link names lowercased,
mesh paths rewritten to `arm/meshes/fr5wml/{visual,collision}/`, and a
zero-offset `link_eef` flange appended. Do not hand-edit the file directly —
re-apply those adaptations to the vendor URDF instead.

It replaces two earlier stand-ins: a placeholder that was a manually
link-corrected copy of Fairino's `fairino5_v6.urdf`, and then a URDF generated
from FAIRINO's published FR5-WML Denavit-Hartenberg table with simplified
generated meshes.

## Meshes

Visual meshes (`meshes/fr5wml/visual/*.glb`) and collision meshes
(`meshes/fr5wml/collision/*.STL`) come from the vendor URDF package's CAD
exports: collision STLs are the vendor's, and the visual GLBs were converted
from the vendor's per-link visual geometry. Link names follow the vendor's
descriptive scheme (`base_link`, `shoulder_link`, `upperarm_link`,
`forearm_link`, `wrist1_link`..`wrist3_link`).

## Sources

- FAIRINO FR5-WML product page:
  https://www.fairino.com/FR/11.html
- FAIRINO Collaborative Robot User Manual 3.9.4 — robot DH parameters, FR5-WML table:
  https://fairino-doc-de.readthedocs.io/3.9.4/CobotsManual/robot_brief_introduction.html
