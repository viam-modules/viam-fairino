#!/usr/bin/env python3
"""Rotate each FR5-WML (from_step) mesh to match its fairino5_v6 reference
counterpart's orientation, per the rotation axes confirmed by visual/extent
comparison (see tools/compare_fr5wml_vs_fairino5v6.py and
tools/ortho_compare_fr5wml_vs_fairino5v6.py):

    base_link, shoulder_link, wrist3_link : single 90 deg rotation about X
    forearm_link, upperarm_link           : single 90 deg rotation about Z
    wrist1_link, wrist2_link              : composite -- a 90 deg rotation
        about X fixes the main housing axis (same Y<->Z swap as the other
        cylindrical links), but the T-junction stub's pointing direction
        needs an additional rotation about Z to line up, since a plain X
        swap alone only fixes the main axis, not the stub's azimuth.

For each link, this tries every remaining sign/quadrant combination
consistent with the CONFIRMED axis (2 options for the single-axis links,
8 for the two composite links) and scores each by how well the rotated
candidate's extent proportions and centroid-offset skew (the feature that
breaks symmetry -- e.g. which way a stub or bolt-hole pattern faces) match
the reference. This is a narrow, targeted search over already-agreed axes,
not an independent re-derivation.

Rotation is applied about the mesh's own native origin (no recentering) --
translation is a separate, later concern once mapping/orientation is
locked in.

Usage:
    source tools/.venv/bin/activate
    python3 tools/rotate_fr5wml_meshes.py

Reads:   tools/staging/fr5wm/meshes/fr5wml/from_step/FR5WML_N.stl
         /home/paige-trexel/projects/fairino_test/frcobot_ros2/fairino_description/meshes/fairino5_v6/*.STL (reference, for scoring only)
Writes:  tools/staging/fr5wm/meshes/fr5wml/rotated/<link_name>.STL
"""
import math
import os

import numpy as np

FAIRINO5V6_DIR = "/home/paige-trexel/projects/fairino_test/frcobot_ros2/fairino_description/meshes/fairino5_v6"
STEP_DIR = os.path.join(
    os.path.dirname(__file__), "staging", "fr5wm", "meshes", "fr5wml", "from_step"
)
OUT_DIR = os.path.join(os.path.dirname(__file__), "staging", "fr5wm", "meshes", "fr5wml", "rotated")

METERS_TO_MM = 1000.0

PAIRS = [
    ("base_link", "FR5WML_1.stl"),
    ("shoulder_link", "FR5WML_2.stl"),
    ("upperarm_link", "FR5WML_3.stl"),
    ("forearm_link", "FR5WML_4.stl"),
    ("wrist1_link", "FR5WML_5.stl"),
    ("wrist2_link", "FR5WML_6.stl"),
    ("wrist3_link", "FR5WML_7.stl"),
]

# link_name -> list of candidate rotation matrices to try (confirmed axis/axes only)
SINGLE_X = [90.0, -90.0]
SINGLE_Z = [90.0, -90.0]
COMPOSITE_XZ_X = [90.0, -90.0]
COMPOSITE_XZ_Z = [0.0, 90.0, 180.0, 270.0]

CONFIRMED_AXES = {
    "base_link": "x",
    "shoulder_link": "x",
    "forearm_link": "z",
    "upperarm_link": "z",
    "wrist1_link": "xz",
    "wrist2_link": "xz",
    "wrist3_link": "x",
}

def rot_x(deg: float) -> np.ndarray:
    t = math.radians(deg)
    c, s = math.cos(t), math.sin(t)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def rot_z(deg: float) -> np.ndarray:
    t = math.radians(deg)
    c, s = math.cos(t), math.sin(t)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


# Override the auto-scorer's pick for links where it got the end-to-end
# orientation wrong -- confirmed via the assembled render: the larger/fatter
# joint flange on both upperarm_link and forearm_link was pointing toward
# the wrist end instead of the base end. The centroid-offset fingerprint
# the scorer uses is dominated by mass along the bar's long axis, not by
# which end's cross-section is larger, so it can't reliably tell these two
# candidates apart -- Rz(+90) (the other 90 deg option, same main-axis
# swap) puts the fat end at the opposite end of the bar and is confirmed
# correct by direct visual inspection.
FORCED_ROTATION = {
    "upperarm_link": ("Rz(+90)", rot_z(90.0)),
    "forearm_link": ("Rz(+90)", rot_z(90.0)),
}


def candidate_rotations(axis_kind: str):
    if axis_kind == "x":
        return [(f"Rx({a:+.0f})", rot_x(a)) for a in SINGLE_X]
    if axis_kind == "z":
        return [(f"Rz({a:+.0f})", rot_z(a)) for a in SINGLE_Z]
    if axis_kind == "xz":
        out = []
        for a in COMPOSITE_XZ_X:
            for b in COMPOSITE_XZ_Z:
                out.append((f"Rx({a:+.0f})@Rz({b:+.0f})", rot_x(a) @ rot_z(b)))
        return out
    raise ValueError(axis_kind)


def load_as_single_mesh(path: str):
    import trimesh

    loaded = trimesh.load(path, force=None)
    if isinstance(loaded, trimesh.Scene):
        return loaded.dump(concatenate=True)
    return loaded


def fingerprint(vertices: np.ndarray):
    mn, mx = vertices.min(axis=0), vertices.max(axis=0)
    extent = mx - mn
    center = (mn + mx) / 2.0
    centroid = vertices.mean(axis=0)
    offset_frac = (centroid - center) / np.maximum(extent, 1e-9)
    return extent, offset_frac


def score(cand_extent, cand_offset, ref_extent, ref_offset):
    extent_err = np.abs(cand_extent / max(cand_extent.max(), 1e-9) - ref_extent / max(ref_extent.max(), 1e-9)).sum()
    offset_err = np.abs(cand_offset - ref_offset).sum()
    return extent_err + offset_err


def main() -> int:
    os.makedirs(OUT_DIR, exist_ok=True)

    print(f"{'link':16s} {'chosen rotation':28s} {'score':>8s}")
    for ref_name, step_name in PAIRS:
        ref_path = os.path.join(FAIRINO5V6_DIR, f"{ref_name}.STL")
        step_path = os.path.join(STEP_DIR, step_name)

        ref_mesh = load_as_single_mesh(ref_path)
        ref_mesh.apply_scale(METERS_TO_MM)
        ref_extent, ref_offset = fingerprint(ref_mesh.vertices)

        step_mesh = load_as_single_mesh(step_path)
        cand_verts = step_mesh.vertices

        if ref_name in FORCED_ROTATION:
            label, rot = FORCED_ROTATION[ref_name]
            print(f"{ref_name:16s} {label:28s} {'forced':>8s}")
        else:
            best = None
            for label, rot in candidate_rotations(CONFIRMED_AXES[ref_name]):
                rotated = cand_verts @ rot.T
                cand_extent, cand_offset = fingerprint(rotated)
                s = score(cand_extent, cand_offset, ref_extent, ref_offset)
                if best is None or s < best[0]:
                    best = (s, label, rot)

            s, label, rot = best
            print(f"{ref_name:16s} {label:28s} {s:8.4f}")

        rotated_mesh = step_mesh.copy()
        rotated_mesh.vertices = step_mesh.vertices @ rot.T
        out_path = os.path.join(OUT_DIR, f"{ref_name}.STL")
        rotated_mesh.export(out_path)
        print(f"                 wrote {out_path}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
