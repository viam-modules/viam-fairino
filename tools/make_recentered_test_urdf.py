#!/usr/bin/env python3
"""Clone tools/staging/fr5wm/fr5wml.urdf, pointing every <visual>/<collision>
mesh at the RECENTERED FR5-WML STLs (tools/recenter_rotated_meshes.py
output) -- for assembled kinematic-chain rendering.

Joint origins/limits/names are left untouched -- this only swaps which
mesh file each link references.

Usage:
    source tools/.venv/bin/activate
    python3 tools/make_recentered_test_urdf.py

Reads:   tools/staging/fr5wm/fr5wml.urdf
Writes:  tools/staging/fr5wm/fr5wml_recentered_test.urdf
"""
import os
import xml.etree.ElementTree as ET

SRC_URDF = os.path.join(os.path.dirname(__file__), "staging", "fr5wm", "fr5wml.urdf")
OUT_URDF = os.path.join(os.path.dirname(__file__), "staging", "fr5wm", "fr5wml_recentered_test.urdf")


def main() -> int:
    tree = ET.parse(SRC_URDF)
    root = tree.getroot()

    for link in root.findall("link"):
        link_name = link.get("name")
        new_path = f"meshes/fr5wml/recentered/{link_name}.STL"
        for geom_tag in ("visual/geometry/mesh", "collision/geometry/mesh"):
            mesh_el = link.find(geom_tag)
            if mesh_el is not None:
                mesh_el.set("filename", new_path)

    tree.write(OUT_URDF, encoding="utf-8", xml_declaration=True)
    print(f"wrote {OUT_URDF}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
