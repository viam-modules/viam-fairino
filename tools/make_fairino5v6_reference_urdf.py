#!/usr/bin/env python3
"""Clone fairino5_v6.urdf ITSELF (its own original joint origins, not
fr5wml.urdf's CAD-corrected/longer ones), pointing every <visual>/
<collision> mesh at fairino5_v6's own mesh files (mm-scaled copies) -- a
fully self-consistent ground-truth assembled render, to check whether an
odd-looking joint (e.g. wrist1 hanging perpendicular off the forearm tube)
is inherent to this design or an artifact of the FR5WML mesh rotation
choices. (Pairing fairino5_v6's short meshes with fr5wml.urdf's
CAD-corrected long offsets, as an earlier version of this script did, is
guaranteed to look disconnected regardless of rotation correctness --
that's not what we want to check here.)

Usage:
    source tools/.venv/bin/activate
    python3 tools/make_fairino5v6_reference_urdf.py

Reads:   /home/paige-trexel/projects/fairino_test/frcobot_ros2/fairino_description/urdf/fairino5_v6.urdf
Writes:  tools/staging/fr5wm/fr5wml_fairino5v6_reference.urdf
"""
import os
import xml.etree.ElementTree as ET

SRC_URDF = "/home/paige-trexel/projects/fairino_test/frcobot_ros2/fairino_description/urdf/fairino5_v6.urdf"
OUT_URDF = os.path.join(os.path.dirname(__file__), "staging", "fr5wm", "fr5wml_fairino5v6_reference.urdf")
FAIRINO5V6_MM_DIR = os.path.join(os.path.dirname(__file__), "staging", "fr5wm", "meshes", "fairino5v6_mm")


def main() -> int:
    tree = ET.parse(SRC_URDF)
    root = tree.getroot()

    for link in root.findall("link"):
        link_name = link.get("name")
        new_path = os.path.join(FAIRINO5V6_MM_DIR, f"{link_name}.STL")
        for geom_tag in ("visual/geometry/mesh", "collision/geometry/mesh"):
            mesh_el = link.find(geom_tag)
            if mesh_el is not None:
                mesh_el.set("filename", new_path)

    tree.write(OUT_URDF, encoding="utf-8", xml_declaration=True)
    print(f"wrote {OUT_URDF}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
