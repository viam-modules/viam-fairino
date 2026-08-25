#!/usr/bin/env python3
"""Automatically determine the axis-aligned rotation that best aligns each
FR5-WML (from_step) mesh with its fairino5_v6 reference counterpart.

Rather than eyeballing each pair, this searches all 24 proper (det=+1)
signed-axis-permutation rotations -- i.e. every way of mapping the
candidate's local X/Y/Z axes onto the reference's, including reflections
of sign -- and scores each by how closely the rotated candidate's oriented
extents AND its mass-distribution skew (centroid offset from the bounding
box center, as a fraction of extent per axis -- this is what breaks
symmetry and pins down sign/chirality, e.g. which way a mounting nub or
bolt-hole pattern faces) match the reference's own fingerprint.

Both meshes are first re-centered on their own bounding-box center, so this
only searches for rotation, not translation (translation happens via the
joint-chain <origin>, not the mesh itself, since the STL is referenced with
a fixed <collision><origin xyz="0 0 0"/> in fr5wml.urdf).

Usage:
    source tools/.venv/bin/activate
    python3 tools/fit_link_rotations.py

Reads:   /home/paige-trexel/projects/fairino_test/frcobot_ros2/fairino_description/meshes/fairino5_v6/*.STL (reference, meters)
         tools/staging/fr5wm/meshes/fr5wml/from_step/FR5WML_N.stl (candidate, mm)
Prints:  best-fit rotation (as a matrix + rpy) and match score per link.
"""
import itertools
import os

import numpy as np

FAIRINO5V6_DIR = "/home/paige-trexel/projects/fairino_test/frcobot_ros2/fairino_description/meshes/fairino5_v6"
STEP_DIR = os.path.join(
    os.path.dirname(__file__), "staging", "fr5wm", "meshes", "fr5wml", "from_step"
)

PAIRS = [
    ("base_link", "FR5WML_1.stl"),
    ("shoulder_link", "FR5WML_2.stl"),
    ("forearm_link", "FR5WML_3.stl"),
    ("upperarm_link", "FR5WML_4.stl"),
    ("wrist1_link", "FR5WML_5.stl"),
    ("wrist2_link", "FR5WML_6.stl"),
    ("wrist3_link", "FR5WML_7.stl"),
]

METERS_TO_MM = 1000.0


def load_as_single_mesh(path: str):
    import trimesh

    loaded = trimesh.load(path, force=None)
    if isinstance(loaded, trimesh.Scene):
        return loaded.dump(concatenate=True)
    return loaded


def all_proper_signed_permutations():
    """The 24 rotation matrices that permute/negate the 3 axes with det=+1
    (the symmetry group of a cube) -- exactly the candidates when a mesh's
    native frame is some axis-aligned rotation away from the target frame."""
    mats = []
    for perm in itertools.permutations(range(3)):
        for signs in itertools.product([1, -1], repeat=3):
            m = np.zeros((3, 3))
            for col, (row, sign) in enumerate(zip(perm, signs)):
                m[row, col] = sign
            if abs(np.linalg.det(m) - 1.0) < 1e-9:
                mats.append(m)
    assert len(mats) == 24
    return mats


def fingerprint(vertices: np.ndarray):
    """(extent, centroid_offset_fraction) relative to the vertex bounding box,
    in the mesh's own current axes -- rotation-covariant, used to score
    candidate rotations against a fixed reference fingerprint."""
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


def matrix_to_rpy(rot: np.ndarray):
    """R = Rz(yaw) @ Ry(pitch) @ Rx(roll) convention (matches tools/render_assembled_glb.py)."""
    import math

    pitch = math.asin(max(-1.0, min(1.0, -rot[2, 0])))
    if abs(rot[2, 0]) < 0.999999:
        roll = math.atan2(rot[2, 1], rot[2, 2])
        yaw = math.atan2(rot[1, 0], rot[0, 0])
    else:
        roll = math.atan2(-rot[0, 1], rot[1, 1])
        yaw = 0.0
    return roll, pitch, yaw


def main() -> int:
    candidates = all_proper_signed_permutations()

    print(f"{'link':16s} {'best rpy (rad)':32s} {'score':>8s}  notes")
    results = {}
    for ref_name, step_name in PAIRS:
        ref_path = os.path.join(FAIRINO5V6_DIR, f"{ref_name}.STL")
        step_path = os.path.join(STEP_DIR, step_name)

        ref_mesh = load_as_single_mesh(ref_path)
        ref_mesh.apply_scale(METERS_TO_MM)
        step_mesh = load_as_single_mesh(step_path)

        ref_extent, ref_offset = fingerprint(ref_mesh.vertices)
        cand_verts = step_mesh.vertices

        best = None
        for rot in candidates:
            rotated = cand_verts @ rot.T
            cand_extent, cand_offset = fingerprint(rotated)
            s = score(cand_extent, cand_offset, ref_extent, ref_offset)
            if best is None or s < best[0]:
                best = (s, rot)

        s, rot = best
        roll, pitch, yaw = matrix_to_rpy(rot)
        results[ref_name] = (rot, (roll, pitch, yaw))
        print(f"{ref_name:16s} rpy=({roll:6.3f},{pitch:6.3f},{yaw:6.3f})  score={s:8.4f}")
        print(f"                 R=\n{rot}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
