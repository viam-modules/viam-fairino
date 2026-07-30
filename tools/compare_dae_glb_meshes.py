#!/usr/bin/env python3
"""Confirm the Step 2 .dae -> .glb conversion didn't change mesh appearance.

pybullet's URDF importer rejects .glb outright ("invalid mesh filename
extension"), so the assembled-URDF rendering approach used for Step 0/Step 3
doesn't work here. Instead this renders each link's .dae and .glb mesh
independently, side by side, via matplotlib (no GL/offscreen-renderer
dependency, so it doesn't hit the same headless-rendering risk pyrender
would). This checks each link's mesh in isolation, not the fully assembled,
articulated arm — Step 2's printed vertex-count/bounding-box diagnostics
already gave a numeric per-link check; this adds the visual one.

Usage:
    source tools/.venv/bin/activate
    pip install -r tools/requirements.txt
    python3 tools/compare_dae_glb_meshes.py

Reads:   tools/staging/fr5wm/meshes/FR5WM/visual/*.dae
         tools/staging/fr5wm/meshes/FR5WM/glb/*.glb
Writes:  tools/output/dae_glb_compare/<link>_compare.png
"""
import argparse
import os
import sys

DAE_DIR = os.path.join(
    os.path.dirname(__file__), "staging", "fr5wm", "meshes", "FR5WM", "visual"
)
GLB_DIR = os.path.join(
    os.path.dirname(__file__), "staging", "fr5wm", "meshes", "FR5WM", "glb"
)
OUT_DIR = os.path.join(os.path.dirname(__file__), "output", "dae_glb_compare")


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
    ax.set_title(f"{title}\nverts={len(mesh.vertices)} faces={len(mesh.faces)}", fontsize=9)


def compare_one(dae_path: str, glb_path: str, out_path: str) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    dae_mesh = load_as_single_mesh(dae_path)
    glb_mesh = load_as_single_mesh(glb_path)

    fig = plt.figure(figsize=(10, 5))
    ax1 = fig.add_subplot(1, 2, 1, projection="3d")
    ax2 = fig.add_subplot(1, 2, 2, projection="3d")
    render_mesh_on_axis(ax1, dae_mesh, "dae (original)", "tab:blue")
    render_mesh_on_axis(ax2, glb_mesh, "glb (converted)", "tab:orange")
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    print(f"  wrote {out_path}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dae-dir", default=DAE_DIR)
    ap.add_argument("--glb-dir", default=GLB_DIR)
    ap.add_argument("--out-dir", default=OUT_DIR)
    args = ap.parse_args()

    if not os.path.isdir(args.dae_dir):
        print(f"dae dir not found: {args.dae_dir}", file=sys.stderr)
        return 1
    if not os.path.isdir(args.glb_dir):
        print(f"glb dir not found: {args.glb_dir} — run tools/convert_dae_to_glb.py first", file=sys.stderr)
        return 1
    os.makedirs(args.out_dir, exist_ok=True)

    dae_files = sorted(f for f in os.listdir(args.dae_dir) if f.lower().endswith(".dae"))
    missing = []
    for fname in dae_files:
        stem = os.path.splitext(fname)[0]
        dae_path = os.path.join(args.dae_dir, fname)
        glb_path = os.path.join(args.glb_dir, stem + ".glb")
        if not os.path.isfile(glb_path):
            missing.append(glb_path)
            continue
        out_path = os.path.join(args.out_dir, f"{stem}_compare.png")
        compare_one(dae_path, glb_path, out_path)

    if missing:
        print(f"\n{len(missing)} .glb files missing (run tools/convert_dae_to_glb.py first):")
        for path in missing:
            print(f"  - {path}")
        return 1

    print(f"\nWrote {len(dae_files)} side-by-side comparisons to {args.out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
