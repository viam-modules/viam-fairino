#!/usr/bin/env python3
"""Derive per-joint <origin> transforms directly from the FR5-WML STEP file,
and export each body's own native mesh (unmodified, just unit-scaled) as the
corresponding link's collision STL.

Why: fr5wml.urdf's joint origins were inherited from fairino5_v6.urdf
(link-length-corrected by hand against a CAD drawing), but the actual
STEP-derived meshes don't sit at the "zero pose" that skeleton assumes --
confirmed by rendering the STEP assembly with its own native transforms
(tools/output/step_assembly_native.png), which connects perfectly on its
own, in a bent/non-zero configuration. So instead of trying to recenter the
new meshes into the OLD joint structure, this computes a fresh joint
structure directly from the STEP data: for each consecutive body pair, the
relative transform (T_parent^-1 @ T_child) becomes that joint's <origin>,
regardless of what pose the CAD happened to be modeled in -- <origin> is
just "child frame relative to parent frame" by definition, so this is valid
regardless of the assembly's shown pose.

Body-to-link mapping (from PLAN.md's shape-comparison + this file's own
assembly-graph structure -- see the STATUS section for how this was
determined): NAUO1..NAUO7 = base_link, shoulder_link, upperarm_link,
forearm_link, wrist1_link, wrist2_link, wrist3_link. NAUO5's geometry name
is "...-5" and NAUO6's is "...-5_1" -- the same part instanced twice
(confirmed identical geometry), matching wrist1/wrist2 both using the same
housing.

Usage:
    source tools/.venv/bin/activate
    python3 tools/derive_from_step.py

Reads:   tools/staging/fr5wm/meshes/fr5wml/from_step/FR5-WML-V6.0.STEP
Writes:  tools/staging/fr5wm/meshes/fr5wml/collision/*.STL (overwritten)
         prints the 6 joint <origin> xyz/rpy values to apply to fr5wml.urdf
"""
import math
import os

import numpy as np
import trimesh

STEP_PATH = os.path.join(
    os.path.dirname(__file__), "staging", "fr5wm", "meshes", "fr5wml", "from_step", "FR5-WML-V6.0.STEP"
)
COLLISION_DIR = os.path.join(os.path.dirname(__file__), "staging", "fr5wm", "meshes", "fr5wml", "collision")

METERS_TO_MM = 1000.0

# Kinematic chain order, and which STEP graph node/geometry each link comes
# from -- naive sequential mapping, re-confirmed by a length-based
# cross-check (mesh extent vs. each joint's CAD-corrected <origin> distance,
# see tools/compare_fr5wml_vs_fairino5v6.py) after an initial visual-review
# swap of NAUO3/NAUO4 turned out not to hold up numerically.
CHAIN = [
    ("base_link", "NAUO1"),
    ("shoulder_link", "NAUO2"),
    ("upperarm_link", "NAUO3"),
    ("forearm_link", "NAUO4"),
    ("wrist1_link", "NAUO5"),
    ("wrist2_link", "NAUO6"),
    ("wrist3_link", "NAUO7"),
]

JOINT_NAMES = ["j1", "j2", "j3", "j4", "j5", "j6"]


def matrix_to_rpy(rot: np.ndarray):
    """Inverse of the rpy_to_matrix convention used throughout tools/
    (R = Rz(yaw) @ Ry(pitch) @ Rx(roll)) -- standard extraction, valid away
    from the pitch = +/-90deg gimbal-lock singularity."""
    pitch = math.asin(max(-1.0, min(1.0, -rot[2, 0])))
    roll = math.atan2(rot[2, 1], rot[2, 2])
    yaw = math.atan2(rot[1, 0], rot[0, 0])
    return roll, pitch, yaw


def main() -> int:
    scene = trimesh.load(STEP_PATH)
    if not isinstance(scene, trimesh.Scene):
        raise RuntimeError(f"expected a Scene with multiple bodies, got {type(scene)}")

    # node name -> (transform, geometry name)
    node_transform = {n: scene.graph.get(n) for n in scene.graph.nodes}

    world = {}
    for link_name, node in CHAIN:
        if node not in node_transform:
            raise RuntimeError(f"no graph node {node!r} for {link_name!r} -- graph nodes: {list(node_transform)}")
        transform, geom_name = node_transform[node]
        world[link_name] = (transform, scene.geometry[geom_name])

    os.makedirs(COLLISION_DIR, exist_ok=True)

    print("== Exporting collision meshes (native STEP geometry, meters -> mm) ==")
    for link_name, (_, geom) in world.items():
        mesh = geom.copy()
        mesh.apply_scale(METERS_TO_MM)
        out_path = os.path.join(COLLISION_DIR, f"{link_name}.STL")
        mesh.export(out_path)
        ext = mesh.bounds[1] - mesh.bounds[0]
        print(f"  {link_name:16s} verts={len(mesh.vertices):6d} extent_mm=({ext[0]:7.1f},{ext[1]:7.1f},{ext[2]:7.1f}) -> {out_path}")

    print("\n== Derived joint <origin> values (parent^-1 @ child, from STEP placements) ==")
    link_names = [name for name, _ in CHAIN]
    for i, joint_name in enumerate(JOINT_NAMES):
        parent_name, child_name = link_names[i], link_names[i + 1]
        t_parent, _ = world[parent_name]
        t_child, _ = world[child_name]
        t_relative = np.linalg.inv(t_parent) @ t_child

        xyz = t_relative[:3, 3]
        roll, pitch, yaw = matrix_to_rpy(t_relative[:3, :3])
        print(
            f'  {joint_name} ({parent_name} -> {child_name}): '
            f'xyz="{xyz[0]:.6f} {xyz[1]:.6f} {xyz[2]:.6f}" '
            f'rpy="{roll:.6f} {pitch:.6f} {yaw:.6f}"'
        )

    print("\nApply these xyz/rpy values to the matching <joint><origin> in fr5wml.urdf.")
    print("Collision STLs already overwritten in place above; run convert_stl_to_glb.py next.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
