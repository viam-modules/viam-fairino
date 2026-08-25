#!/usr/bin/env python3
"""Compare each fairino5_v6 reference mesh against its ROTATED FR5-WML
counterpart (tools/rotate_fr5wml_meshes.py output), side by side, each in
its own raw/native coordinates -- to visually confirm the applied
rotations actually aligned each link's orientation.

Usage:
    source tools/.venv/bin/activate
    python3 tools/compare_fr5wml_rotated_vs_fairino5v6.py

Reads:   /home/paige-trexel/projects/fairino_test/frcobot_ros2/fairino_description/meshes/fairino5_v6/*.STL (reference, meters)
         tools/staging/fr5wm/meshes/fr5wml/rotated/<link>.STL (rotated candidate, mm)
Writes:  tools/output/fr5wml_rotated_vs_fairino5v6/<link>_compare.png
"""
import os

FAIRINO5V6_DIR = "/home/paige-trexel/projects/fairino_test/frcobot_ros2/fairino_description/meshes/fairino5_v6"
ROTATED_DIR = os.path.join(
    os.path.dirname(__file__), "staging", "fr5wm", "meshes", "fr5wml", "rotated"
)
OUT_DIR = os.path.join(os.path.dirname(__file__), "output", "fr5wml_rotated_vs_fairino5v6")

LINK_NAMES = [
    "base_link",
    "shoulder_link",
    "forearm_link",
    "upperarm_link",
    "wrist1_link",
    "wrist2_link",
    "wrist3_link",
]

METERS_TO_MM = 1000.0


def load_as_single_mesh(path: str):
    import trimesh

    loaded = trimesh.load(path, force=None)
    if isinstance(loaded, trimesh.Scene):
        return loaded.dump(concatenate=True)
    return loaded


def render_mesh_on_axis(ax, mesh, title: str, color: str) -> None:
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    triangles = mesh.vertices[mesh.faces]
    coll = Poly3DCollection(triangles, facecolor=color, edgecolor="k", linewidths=0.05, alpha=1.0)
    ax.add_collection3d(coll)

    bounds = mesh.bounds
    center = bounds.mean(axis=0)
    extent = max((bounds[1] - bounds[0]).max() / 2, 1e-6) * 1.2
    ax.set_xlim(center[0] - extent, center[0] + extent)
    ax.set_ylim(center[1] - extent, center[1] + extent)
    ax.set_zlim(center[2] - extent, center[2] + extent)
    ax.view_init(elev=25, azim=45)
    ax.set_box_aspect([1, 1, 1])
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title(f"{title}\nverts={len(mesh.vertices)}", fontsize=9)


def main() -> int:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    os.makedirs(OUT_DIR, exist_ok=True)

    for link_name in LINK_NAMES:
        ref_path = os.path.join(FAIRINO5V6_DIR, f"{link_name}.STL")
        rot_path = os.path.join(ROTATED_DIR, f"{link_name}.STL")
        if not os.path.isfile(ref_path) or not os.path.isfile(rot_path):
            print(f"  skipping {link_name}: missing file(s)")
            continue

        ref_mesh = load_as_single_mesh(ref_path)
        ref_mesh.apply_scale(METERS_TO_MM)
        rot_mesh = load_as_single_mesh(rot_path)

        fig = plt.figure(figsize=(11, 5.5))
        ax1 = fig.add_subplot(1, 2, 1, projection="3d")
        ax2 = fig.add_subplot(1, 2, 2, projection="3d")
        render_mesh_on_axis(ax1, ref_mesh, f"fairino5_v6: {link_name} (reference, mm)", "tab:blue")
        render_mesh_on_axis(ax2, rot_mesh, f"fr5wml ROTATED: {link_name}", "tab:orange")
        fig.tight_layout()

        out_path = os.path.join(OUT_DIR, f"{link_name}_compare.png")
        fig.savefig(out_path, dpi=150)
        plt.close(fig)
        print(f"  wrote {out_path}")

    print(f"\nWrote comparisons to {OUT_DIR}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
