#!/usr/bin/env python3
"""Translate each ROTATED FR5-WML mesh (tools/rotate_fr5wml_meshes.py
output) so its own local origin sits where its INCOMING joint places it --
matching URDF convention, where a link's mesh is expressed directly in its
own frame with <origin xyz="0 0 0"/>, and the link's origin IS wherever its
parent joint's <origin> puts it.

For a link with a single "bar" outgoing joint offset d along axis a (its
own outgoing joint's <origin xyz>, already CAD-corrected/trusted in
fr5wml.urdf): shift that axis so the mesh's extreme point in the direction
OPPOSITE d sits at 0 (i.e. the proximal end, attaching to the parent, lands
at the origin, and the mesh extends toward the child in direction d). The
two non-dominant axes are just centered (assumes near-rotationally-
symmetric cross-section, true for all of these parts).

base_link (no parent -- shoulder attaches at its own origin, offset 0) and
wrist3_link (terminal, no outgoing joint) are handled as the same
"one face at the origin" case using their own main axis (Z, per their
confirmed X-axis rotation).

This does NOT re-derive lengths -- it only shifts already-rotated meshes
into position using the offsets already established (and cross-checked
against mesh extents) in fr5wml.urdf. Any leftover gap/overlap at a joint
in the resulting assembled render reflects a real mismatch (wrong end
mapped as proximal, wrong body, etc.), not a bug in this recentering step.

Usage:
    source tools/.venv/bin/activate
    python3 tools/recenter_rotated_meshes.py

Reads:   tools/staging/fr5wm/meshes/fr5wml/rotated/<link>.STL
Writes:  tools/staging/fr5wm/meshes/fr5wml/recentered/<link>.STL
"""
import os

import numpy as np

ROTATED_DIR = os.path.join(
    os.path.dirname(__file__), "staging", "fr5wm", "meshes", "fr5wml", "rotated"
)
OUT_DIR = os.path.join(os.path.dirname(__file__), "staging", "fr5wm", "meshes", "fr5wml", "recentered")

# link -> (dominant axis index, signed offset in mm to its OWN outgoing
# joint, per fr5wml.urdf's CAD-corrected <origin>). base_link/wrist3_link
# have no outgoing joint -- see NO_OUTGOING below.
OUTGOING_OFFSET_MM = {
    "shoulder_link": (2, 180.0),
    "upperarm_link": (0, -970.0),
    "forearm_link": (0, -816.0),
    "wrist1_link": (2, 114.0),
    "wrist2_link": (2, 106.0),
}

# link -> dominant axis index only, treated as "one face at the origin,
# no far end to aim at" (base_link: shoulder mounts at its own origin;
# wrist3_link: terminal link, mounts at wrist2's outgoing offset).
NO_OUTGOING = {
    "base_link": 2,
    "wrist3_link": 2,
}

LINK_NAMES = [
    "base_link",
    "shoulder_link",
    "upperarm_link",
    "forearm_link",
    "wrist1_link",
    "wrist2_link",
    "wrist3_link",
]


def load_as_single_mesh(path: str):
    import trimesh

    loaded = trimesh.load(path, force=None)
    if isinstance(loaded, trimesh.Scene):
        return loaded.dump(concatenate=True)
    return loaded


def main() -> int:
    os.makedirs(OUT_DIR, exist_ok=True)

    for link_name in LINK_NAMES:
        path = os.path.join(ROTATED_DIR, f"{link_name}.STL")
        mesh = load_as_single_mesh(path)
        bounds = mesh.bounds  # (2,3): [min, max]

        shift = np.zeros(3)
        if link_name in OUTGOING_OFFSET_MM:
            axis, d = OUTGOING_OFFSET_MM[link_name]
        else:
            axis, d = NO_OUTGOING[link_name], -1.0  # sign only used to pick min/max below

        # dominant axis: proximal end (opposite the offset direction) -> 0
        if d >= 0:
            shift[axis] = -bounds[0][axis]
        else:
            shift[axis] = -bounds[1][axis]

        # non-dominant axes: center the bounding box
        for other in range(3):
            if other == axis:
                continue
            shift[other] = -(bounds[0][other] + bounds[1][other]) / 2.0

        recentered = mesh.copy()
        recentered.vertices = mesh.vertices + shift
        out_path = os.path.join(OUT_DIR, f"{link_name}.STL")
        recentered.export(out_path)

        new_bounds = recentered.bounds
        print(
            f"  {link_name:16s} shift=({shift[0]:8.1f},{shift[1]:8.1f},{shift[2]:8.1f})  "
            f"new_bounds=({new_bounds[0][0]:7.1f}..{new_bounds[1][0]:7.1f}, "
            f"{new_bounds[0][1]:7.1f}..{new_bounds[1][1]:7.1f}, "
            f"{new_bounds[0][2]:7.1f}..{new_bounds[1][2]:7.1f})"
        )

    print(f"\nWrote recentered meshes to {OUT_DIR}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
