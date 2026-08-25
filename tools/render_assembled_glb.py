#!/usr/bin/env python3
"""Assembled, articulated render of the URDF now that <visual> is .glb.

pybullet's URDF importer rejects .glb outright ("invalid mesh filename
extension"), so it can't drive the usual assembled-pose renders the way
Step 0/Step 3 do for .dae/.stl. This instead computes forward kinematics
directly from the URDF's own joint tree (origin + revolute axis, per link)
and assembles each link's .glb mesh into world space via trimesh, then
renders the whole arm with matplotlib — same no-GL-dependency approach as
compare_dae_glb_meshes.py, extended from one isolated mesh to the full
articulated assembly, at the same style of zero/pose_a/pose_b joint
configurations used by the pybullet-based scripts.

Usage:
    source tools/.venv/bin/activate
    python3 tools/render_assembled_glb.py

Reads:   tools/staging/fr5wm/urdf/fr5wm.urdf (visual now points at .glb)
Writes:  tools/output/glb_assembled/{zero_pose,pose_a,pose_b}.png
"""
import argparse
import os
import sys
import xml.etree.ElementTree as ET

import numpy as np

CLEANED_URDF = os.path.join(
    os.path.dirname(__file__), "staging", "fr5wm", "urdf", "fr5wm.urdf"
)
OUT_DIR = os.path.join(os.path.dirname(__file__), "output", "glb_assembled")

LINK_COLORS = {
    "base_link": "tab:gray",
    "shoulder_Link": "tab:blue",
    "upperarm_Link": "tab:orange",
    "forearm_Link": "tab:green",
    "wrist1_Link": "tab:red",
    "wrist2_Link": "tab:purple",
    "wrist3_Link": "tab:brown",
    "shoulder_link": "tab:blue",
    "upperarm_link": "tab:orange",
    "forearm_link": "tab:green",
    "wrist1_link": "tab:red",
    "wrist2_link": "tab:purple",
    "wrist3_link": "tab:brown",
}


def rpy_to_matrix(r: float, p: float, y: float) -> np.ndarray:
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return rz @ ry @ rx


def axis_angle_matrix(axis, angle: float) -> np.ndarray:
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    c, s = np.cos(angle), np.sin(angle)
    cc = 1 - c
    return np.array(
        [
            [x * x * cc + c, x * y * cc - z * s, x * z * cc + y * s],
            [y * x * cc + z * s, y * y * cc + c, y * z * cc - x * s],
            [z * x * cc - y * s, z * y * cc + x * s, z * z * cc + c],
        ]
    )


def homogeneous(rot: np.ndarray, trans) -> np.ndarray:
    t = np.eye(4)
    t[:3, :3] = rot
    t[:3, 3] = trans
    return t


def parse_chain(urdf_path: str):
    urdf_dir = os.path.dirname(os.path.abspath(urdf_path))
    root = ET.parse(urdf_path).getroot()

    links = {}
    for link in root.findall("link"):
        mesh_el = link.find("visual/geometry/mesh")
        links[link.get("name")] = (
            os.path.abspath(os.path.join(urdf_dir, mesh_el.get("filename"))) if mesh_el is not None else None
        )

    joints = []
    for joint in root.findall("joint"):
        origin = joint.find("origin")
        axis_el = joint.find("axis")
        xyz = [float(v) for v in (origin.get("xyz") if origin is not None else "0 0 0").split()]
        rpy = [float(v) for v in (origin.get("rpy") if origin is not None else "0 0 0").split()]
        axis = [float(v) for v in (axis_el.get("xyz") if axis_el is not None else "0 0 1").split()]
        limit = joint.find("limit")
        lower = float(limit.get("lower")) if limit is not None else 0.0
        upper = float(limit.get("upper")) if limit is not None else 0.0
        joints.append(
            {
                "name": joint.get("name"),
                "type": joint.get("type"),
                "parent": joint.find("parent").get("link"),
                "child": joint.find("child").get("link"),
                "xyz": xyz,
                "rpy": rpy,
                "axis": axis,
                "lower": lower,
                "upper": upper,
            }
        )
    return links, joints


METERS_TO_MM = 1000.0


def forward_kinematics(joints, root_link: str, joint_angles: dict) -> dict:
    """URDF <origin xyz> is in meters, but every mesh in this project is in
    millimeters -- scale joint translations up so they combine correctly
    with mesh-space vertex coordinates in render_pose()."""
    world = {root_link: np.eye(4)}
    remaining = list(joints)
    while remaining:
        progressed = False
        for j in list(remaining):
            if j["parent"] not in world:
                continue
            origin = homogeneous(rpy_to_matrix(*j["rpy"]), np.array(j["xyz"]) * METERS_TO_MM)
            if j["type"] == "revolute":
                angle = joint_angles.get(j["name"], 0.0)
                rot = homogeneous(axis_angle_matrix(j["axis"], angle), np.zeros(3))
                child_tf = world[j["parent"]] @ origin @ rot
            else:
                child_tf = world[j["parent"]] @ origin
            world[j["child"]] = child_tf
            remaining.remove(j)
            progressed = True
        if not progressed:
            unresolved = [j["name"] for j in remaining]
            raise RuntimeError(f"could not resolve joint chain, stuck on: {unresolved}")
    return world


def render_pose(links: dict, joints: list, joint_angles: dict, out_path: str) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    import trimesh

    world = forward_kinematics(joints, "base_link", joint_angles)

    fig = plt.figure(figsize=(7, 7))
    ax = fig.add_subplot(111, projection="3d")

    all_points = []
    for link_name, mesh_path in links.items():
        if mesh_path is None or link_name not in world:
            continue
        loaded = trimesh.load(mesh_path, force=None)
        mesh = loaded.dump(concatenate=True) if isinstance(loaded, trimesh.Scene) else loaded
        verts_h = np.hstack([mesh.vertices, np.ones((len(mesh.vertices), 1))])
        world_verts = (world[link_name] @ verts_h.T).T[:, :3]
        all_points.append(world_verts)
        tri = world_verts[mesh.faces]
        color = LINK_COLORS.get(link_name, "tab:gray")
        ax.add_collection3d(Poly3DCollection(tri, facecolor=color, edgecolor="k", linewidths=0.02, alpha=1.0))

    pts = np.vstack(all_points)
    center = pts.mean(axis=0)
    extent = max((pts.max(axis=0) - pts.min(axis=0)).max() / 2, 1e-6) * 1.2
    ax.set_xlim(center[0] - extent, center[0] + extent)
    ax.set_ylim(center[1] - extent, center[1] + extent)
    ax.set_zlim(center[2] - extent, center[2] + extent)
    ax.set_box_aspect([1, 1, 1])
    ax.view_init(elev=25, azim=45)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    print(f"  wrote {out_path}")


def scaled_pose(joints: list, frac: float) -> dict:
    pose = {}
    for j in joints:
        if j["type"] == "revolute" and j["upper"] > j["lower"]:
            pose[j["name"]] = j["lower"] + (j["upper"] - j["lower"]) * ((frac + 1) / 2)
    return pose


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--urdf", default=CLEANED_URDF)
    ap.add_argument("--out-dir", default=OUT_DIR)
    args = ap.parse_args()

    if not os.path.isfile(args.urdf):
        print(f"URDF not found: {args.urdf}", file=sys.stderr)
        return 1
    os.makedirs(args.out_dir, exist_ok=True)

    links, joints = parse_chain(args.urdf)
    missing = [name for name, path in links.items() if path and not os.path.isfile(path)]
    if missing:
        print(f"Missing mesh files for: {missing}", file=sys.stderr)
        return 1

    render_pose(links, joints, {}, os.path.join(args.out_dir, "zero_pose.png"))
    render_pose(links, joints, scaled_pose(joints, 0.4), os.path.join(args.out_dir, "pose_a.png"))
    render_pose(links, joints, scaled_pose(joints, -0.3), os.path.join(args.out_dir, "pose_b.png"))

    print(f"\nWrote assembled glb renders to {args.out_dir}")
    print("Compare against tools/output/step0_baseline/ and tools/output/step3_verification/ (both .dae-based, pybullet)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
