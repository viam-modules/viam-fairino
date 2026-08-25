#!/usr/bin/env python3
"""Clone tools/staging/fr5wm/fr5wml.urdf, pointing every <visual>/<collision>
mesh at the ROTATED FR5-WML STLs (tools/rotate_fr5wml_meshes.py output)
instead of the current (wrong-orientation) meshes -- so the assembled
kinematic-chain renderer can be used to sanity-check the rotations in
context (a flipped link shows up immediately as a joint that doesn't
connect / points the wrong way, which is hard to see from an isolated
per-mesh comparison).

Joint origins/limits/names are left untouched -- this only swaps which
mesh file each link references.

Usage:
    source tools/.venv/bin/activate
    python3 tools/make_rotated_test_urdf.py

Reads:   tools/staging/fr5wm/fr5wml.urdf
Writes:  tools/staging/fr5wm/fr5wml_rotated_test.urdf
"""
import os
import xml.etree.ElementTree as ET

SRC_URDF = os.path.join(os.path.dirname(__file__), "staging", "fr5wm", "fr5wml.urdf")
OUT_URDF = os.path.join(os.path.dirname(__file__), "staging", "fr5wm", "fr5wml_rotated_test.urdf")


def main() -> int:
    tree = ET.parse(SRC_URDF)
    root = tree.getroot()

    for link in root.findall("link"):
        link_name = link.get("name")
        new_path = f"meshes/fr5wml/rotated/{link_name}.STL"
        for geom_tag in ("visual/geometry/mesh", "collision/geometry/mesh"):
            mesh_el = link.find(geom_tag)
            if mesh_el is not None:
                mesh_el.set("filename", new_path)

    tree.write(OUT_URDF, encoding="utf-8", xml_declaration=True)
    print(f"wrote {OUT_URDF}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
