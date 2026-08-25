#!/usr/bin/env python3
"""Orthographic (axis-aligned) 3-view comparison of each fairino5_v6
reference mesh against its FR5-WML (from_step) counterpart -- top (looking
down -Z, shows XY), front (looking down -Y, shows XZ), and side (looking
down -X, shows YZ) -- so the specific rotation axis needed to align each
link is visually unambiguous, rather than inferred from a single isometric
view.

Usage:
    source tools/.venv/bin/activate
    python3 tools/ortho_compare_fr5wml_vs_fairino5v6.py

Reads:   /home/paige-trexel/projects/fairino_test/frcobot_ros2/fairino_description/meshes/fairino5_v6/*.STL (reference, meters)
         tools/staging/fr5wm/meshes/fr5wml/from_step/FR5WML_N.stl (candidate, mm)
Writes:  tools/output/fr5wml_vs_fairino5v6_ortho/<link>_ortho.png
"""
import os

FAIRINO5V6_DIR = "/home/paige-trexel/projects/fairino_test/frcobot_ros2/fairino_description/meshes/fairino5_v6"
STEP_DIR = os.path.join(
    os.path.dirname(__file__), "staging", "fr5wm", "meshes", "fr5wml", "from_step"
)
OUT_DIR = os.path.join(os.path.dirname(__file__), "output", "fr5wml_vs_fairino5v6_ortho")

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

# (view name, elev, azim) -- matplotlib's mplot3d elev/azim to get a
# straight-down-the-axis orthographic-like look for each of the 3 principal views.
VIEWS = [
    ("top (down -Z, XY)", 90, -90),
    ("front (down -Y, XZ)", 0, -90),
    ("side (down -X, YZ)", 0, 0),
]


def load_as_single_mesh(path: str):
    import trimesh

    loaded = trimesh.load(path, force=None)
    if isinstance(loaded, trimesh.Scene):
        return loaded.dump(concatenate=True)
    return loaded


def render_mesh_on_axis(ax, mesh, title: str, color: str, elev: float, azim: float) -> None:
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
    ax.set_proj_type("ortho")
    ax.view_init(elev=elev, azim=azim)
    ax.set_box_aspect([1, 1, 1])
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title(title, fontsize=9)


def main() -> int:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    os.makedirs(OUT_DIR, exist_ok=True)

    for ref_name, step_name in PAIRS:
        ref_path = os.path.join(FAIRINO5V6_DIR, f"{ref_name}.STL")
        step_path = os.path.join(STEP_DIR, step_name)
        if not os.path.isfile(ref_path) or not os.path.isfile(step_path):
            print(f"  skipping {ref_name}/{step_name}: missing file(s)")
            continue

        ref_mesh = load_as_single_mesh(ref_path)
        ref_mesh.apply_scale(METERS_TO_MM)
        step_mesh = load_as_single_mesh(step_path)

        fig = plt.figure(figsize=(16, 11))
        for col, (mesh, label, color) in enumerate(
            [(ref_mesh, f"fairino5_v6: {ref_name}", "tab:blue"), (step_mesh, f"from_step: {step_name}", "tab:orange")]
        ):
            for row, (view_name, elev, azim) in enumerate(VIEWS):
                ax = fig.add_subplot(3, 2, row * 2 + col + 1, projection="3d")
                render_mesh_on_axis(ax, mesh, f"{label}\n{view_name}", color, elev, azim)
        fig.tight_layout()

        out_path = os.path.join(OUT_DIR, f"{ref_name}_ortho.png")
        fig.savefig(out_path, dpi=130)
        plt.close(fig)
        print(f"  wrote {out_path}")

    print(f"\nWrote orthographic comparisons to {OUT_DIR}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
