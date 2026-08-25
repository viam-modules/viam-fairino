#!/usr/bin/env python3
"""Compare each fairino5_v6 reference mesh against its FR5-WML (from_step)
counterpart, side by side, each in its own raw/native coordinates (no
assembly, no transform applied) -- so shape correspondence and axis
convention differences can be assessed visually.

fairino5_v6 is the URDF topology/structure fr5wml.urdf is actually built
from (link names, joint chain, CAD-corrected lengths), so its own mesh set
is the correct comparison reference -- not FR5WM (a different URDF family
used earlier by mistake).

Mapping below is the naive sequential-order guess, RE-CONFIRMED after a
length-based cross-check (mesh extent vs. each joint's CAD-corrected
<origin> distance, scaled by the fairino5_v6 reference's own
extent-to-offset ratio) showed the initial visual-review-based 3/4 swap
didn't hold up numerically -- FR5WML_3/FR5WML_4 assigned as upperarm/
forearm (not swapped) fits both links' expected lengths within ~10%,
consistently, whereas the swap gave a wildly inconsistent -25.6%/+8.3% fit.
    FR5WML_1.stl -> base_link
    FR5WML_2.stl -> shoulder_link
    FR5WML_3.stl -> upperarm_link
    FR5WML_4.stl -> forearm_link
    FR5WML_5.stl -> wrist1_link
    FR5WML_6.stl -> wrist2_link
    FR5WML_7.stl -> wrist3_link

Usage:
    source tools/.venv/bin/activate
    python3 tools/compare_fr5wml_vs_fairino5v6.py

Reads:   /home/paige-trexel/projects/fairino_test/frcobot_ros2/fairino_description/meshes/fairino5_v6/*.STL (reference, meters)
         tools/staging/fr5wm/meshes/fr5wml/from_step/FR5WML_N.stl (candidate, mm)
Writes:  tools/output/fr5wml_vs_fairino5v6/<link>_compare.png
"""
import os

FAIRINO5V6_DIR = "/home/paige-trexel/projects/fairino_test/frcobot_ros2/fairino_description/meshes/fairino5_v6"
STEP_DIR = os.path.join(
    os.path.dirname(__file__), "staging", "fr5wm", "meshes", "fr5wml", "from_step"
)
OUT_DIR = os.path.join(os.path.dirname(__file__), "output", "fr5wml_vs_fairino5v6")

# (fairino5_v6 link name, from_step file) -- naive sequential mapping,
# re-confirmed by the length cross-check (see module docstring).
PAIRS = [
    ("base_link", "FR5WML_1.stl"),
    ("shoulder_link", "FR5WML_2.stl"),
    ("upperarm_link", "FR5WML_3.stl"),
    ("forearm_link", "FR5WML_4.stl"),
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

    for ref_name, step_name in PAIRS:
        ref_path = os.path.join(FAIRINO5V6_DIR, f"{ref_name}.STL")
        step_path = os.path.join(STEP_DIR, step_name)
        if not os.path.isfile(ref_path) or not os.path.isfile(step_path):
            print(f"  skipping {ref_name}/{step_name}: missing file(s)")
            continue

        ref_mesh = load_as_single_mesh(ref_path)
        # fairino5_v6 STLs are in meters; scale to mm to match the from_step set.
        ref_mesh.apply_scale(METERS_TO_MM)
        step_mesh = load_as_single_mesh(step_path)

        fig = plt.figure(figsize=(11, 5.5))
        ax1 = fig.add_subplot(1, 2, 1, projection="3d")
        ax2 = fig.add_subplot(1, 2, 2, projection="3d")
        render_mesh_on_axis(ax1, ref_mesh, f"fairino5_v6: {ref_name} (reference, mm)", "tab:blue")
        render_mesh_on_axis(ax2, step_mesh, f"from_step: {step_name} (candidate)", "tab:orange")
        fig.tight_layout()

        out_path = os.path.join(OUT_DIR, f"{ref_name}_compare.png")
        fig.savefig(out_path, dpi=150)
        plt.close(fig)

        ref_ext = ref_mesh.bounds[1] - ref_mesh.bounds[0]
        step_ext = step_mesh.bounds[1] - step_mesh.bounds[0]
        print(
            f"  {ref_name:16s} extent_mm=({ref_ext[0]:7.1f},{ref_ext[1]:7.1f},{ref_ext[2]:7.1f})  "
            f"{step_name:16s} extent_mm=({step_ext[0]:7.1f},{step_ext[1]:7.1f},{step_ext[2]:7.1f})  "
            f"-> {out_path}"
        )

    print(f"\nWrote comparisons to {OUT_DIR}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
