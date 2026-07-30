#!/usr/bin/env python3
"""Step 2: convert FR5WM's .dae visual meshes to .glb for Get3DModels.

RDK's URDF loader only reads <collision><mesh> geometry (.stl/.ply) — it
never touches <visual> meshes, and doesn't parse .dae at all. The arm
module's Get3DModels method (per the viam-dobot/viam-ufactory-xarm
precedent) instead serves pre-baked binary glTF (.glb) files for the app's
3D scene viewer, read from disk at runtime and keyed by link name. This
script does that one-time conversion: every <visual> .dae mesh becomes a
same-named .glb, offline, committed once — not something the module does at
runtime.

Usage:
    source tools/.venv/bin/activate
    pip install -r tools/requirements.txt
    python3 tools/convert_dae_to_glb.py

Reads from:  tools/staging/fr5wm/meshes/FR5WM/visual/*.dae
Writes to:   tools/staging/fr5wm/meshes/FR5WM/glb/*.glb
"""
import argparse
import os
import sys

DEFAULT_IN_DIR = os.path.join(
    os.path.dirname(__file__), "staging", "fr5wm", "meshes", "FR5WM", "visual"
)
DEFAULT_OUT_DIR = os.path.join(
    os.path.dirname(__file__), "staging", "fr5wm", "meshes", "FR5WM", "glb"
)


def convert_one(dae_path: str, glb_path: str) -> None:
    import trimesh

    loaded = trimesh.load(dae_path, force=None)

    if isinstance(loaded, trimesh.Scene):
        geom_count = len(loaded.geometry)
        total_verts = sum(g.vertices.shape[0] for g in loaded.geometry.values())
        bounds = loaded.bounds
    else:
        geom_count = 1
        total_verts = loaded.vertices.shape[0]
        bounds = loaded.bounds

    loaded.export(glb_path, file_type="glb")

    size_kb = os.path.getsize(glb_path) / 1024
    print(
        f"  {os.path.basename(dae_path):24s} -> {os.path.basename(glb_path):24s} "
        f"geoms={geom_count} verts={total_verts:6d} "
        f"bounds=[{bounds[0][0]:.3f},{bounds[0][1]:.3f},{bounds[0][2]:.3f}]..."
        f"[{bounds[1][0]:.3f},{bounds[1][1]:.3f},{bounds[1][2]:.3f}] "
        f"({size_kb:.1f} KB)"
    )


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--in-dir", default=DEFAULT_IN_DIR)
    ap.add_argument("--out-dir", default=DEFAULT_OUT_DIR)
    args = ap.parse_args()

    if not os.path.isdir(args.in_dir):
        print(f"Input dir not found: {args.in_dir}", file=sys.stderr)
        return 1
    os.makedirs(args.out_dir, exist_ok=True)

    dae_files = sorted(f for f in os.listdir(args.in_dir) if f.lower().endswith(".dae"))
    if not dae_files:
        print(f"No .dae files found in {args.in_dir}", file=sys.stderr)
        return 1

    print(f"Converting {len(dae_files)} .dae files from {args.in_dir}")
    failures = []
    for fname in dae_files:
        dae_path = os.path.join(args.in_dir, fname)
        glb_path = os.path.join(args.out_dir, os.path.splitext(fname)[0] + ".glb")
        try:
            convert_one(dae_path, glb_path)
        except Exception as exc:  # noqa: BLE001 - report and continue the batch
            failures.append((fname, exc))
            print(f"  {fname:24s} -> FAILED: {exc}")

    print()
    if failures:
        print(f"{len(failures)}/{len(dae_files)} conversions failed:")
        for fname, exc in failures:
            print(f"  - {fname}: {exc}")
        return 1

    print(f"All {len(dae_files)} conversions succeeded. Output: {args.out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
