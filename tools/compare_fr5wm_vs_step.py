#!/usr/bin/env python3
"""Compare each known-correct FR5WM mesh against its assumed FR5-WML
(from_step) counterpart, side by side, each in its own raw/native
coordinates (no assembly, no transform applied) -- so shape correspondence
and axis convention differences can be assessed visually rather than
guessed from bounding-box numbers alone.

FR5WM.urdf's meshes are already known-good (this module's very first,
working pass). FR5-WML is expected to be the same "WM" wrist-mount family,
just longer in the first several links per the user -- so this is a much
more apples-to-apples comparison than the fairino5_v6 STL set (a different
URDF family/topology) this module briefly used in between.

Usage:
    source tools/.venv/bin/activate
    python3 tools/compare_fr5wm_vs_step.py

Reads:   tools/staging/fr5wm/meshes/FR5WM/collision/*.STL (known-good)
         tools/staging/fr5wm/meshes/fr5wml/from_step/FR5WML_N.stl (candidate)
Writes:  tools/output/fr5wm_vs_step/<link>_compare.png
"""
import os
import sys

FR5WM_DIR = os.path.join(
    os.path.dirname(__file__), "staging", "fr5wm", "meshes", "FR5WM", "collision"
)
STEP_DIR = os.path.join(
    os.path.dirname(__file__), "staging", "fr5wm", "meshes", "fr5wml", "from_step"
)
OUT_DIR = os.path.join(os.path.dirname(__file__), "output", "fr5wm_vs_step")

# (FR5WM link name, assumed from_step file) -- numbering assumption per the user.
PAIRS = [
    ("base_link", "FR5WML_1.stl"),
    ("shoulder_Link", "FR5WML_2.stl"),
    ("upperarm_Link", "FR5WML_3.stl"),
    ("forearm_Link", "FR5WML_4.stl"),
    ("wrist1_Link", "FR5WML_5.stl"),
    ("wrist2_Link", "FR5WML_6.stl"),
    ("wrist3_Link", "FR5WML_7.stl"),
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

    for fr5wm_name, step_name in PAIRS:
        fr5wm_path = os.path.join(FR5WM_DIR, f"{fr5wm_name}.STL")
        step_path = os.path.join(STEP_DIR, step_name)
        if not os.path.isfile(fr5wm_path) or not os.path.isfile(step_path):
            print(f"  skipping {fr5wm_name}/{step_name}: missing file(s)")
            continue

        fr5wm_mesh = load_as_single_mesh(fr5wm_path)
        # FR5WM STLs are in meters; scale to mm to match the from_step set,
        # so the printed extents/visual scale are directly comparable.
        fr5wm_mesh.apply_scale(METERS_TO_MM)
        step_mesh = load_as_single_mesh(step_path)

        fig = plt.figure(figsize=(11, 5.5))
        ax1 = fig.add_subplot(1, 2, 1, projection="3d")
        ax2 = fig.add_subplot(1, 2, 2, projection="3d")
        render_mesh_on_axis(ax1, fr5wm_mesh, f"FR5WM: {fr5wm_name} (known-good, mm)", "tab:blue")
        render_mesh_on_axis(ax2, step_mesh, f"from_step: {step_name} (candidate)", "tab:orange")
        fig.tight_layout()

        out_path = os.path.join(OUT_DIR, f"{fr5wm_name}_compare.png")
        fig.savefig(out_path, dpi=150)
        plt.close(fig)

        fr5wm_ext = fr5wm_mesh.bounds[1] - fr5wm_mesh.bounds[0]
        step_ext = step_mesh.bounds[1] - step_mesh.bounds[0]
        print(
            f"  {fr5wm_name:16s} extent_mm=({fr5wm_ext[0]:7.1f},{fr5wm_ext[1]:7.1f},{fr5wm_ext[2]:7.1f})  "
            f"{step_name:16s} extent_mm=({step_ext[0]:7.1f},{step_ext[1]:7.1f},{step_ext[2]:7.1f})  "
            f"-> {out_path}"
        )

    print(f"\nWrote comparisons to {OUT_DIR}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
