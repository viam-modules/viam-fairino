#!/usr/bin/env python3
"""Convert collision .STL meshes to .glb for Get3DModels, when no separate
textured visual source (.dae) exists for a variant.

Companion to convert_dae_to_glb.py — same trimesh-based approach, same
one-time-offline/committed-output pattern, just a different source format.
Untextured/gray output (no color/material data in a bare STL), but
geometrically accurate.

Usage:
    source tools/.venv/bin/activate
    python3 tools/convert_stl_to_glb.py

Reads from:  tools/staging/fr5wm/meshes/fr5wml/collision/*.STL
Writes to:   tools/staging/fr5wm/meshes/fr5wml/visual/*.glb
"""
import argparse
import os
import sys

DEFAULT_IN_DIR = os.path.join(
    os.path.dirname(__file__), "staging", "fr5wm", "meshes", "fr5wml", "collision"
)
DEFAULT_OUT_DIR = os.path.join(
    os.path.dirname(__file__), "staging", "fr5wm", "meshes", "fr5wml", "visual"
)


def convert_one(stl_path: str, glb_path: str) -> None:
    import trimesh

    loaded = trimesh.load(stl_path, force=None)

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
        f"  {os.path.basename(stl_path):24s} -> {os.path.basename(glb_path):24s} "
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

    stl_files = sorted(f for f in os.listdir(args.in_dir) if f.lower().endswith(".stl"))
    if not stl_files:
        print(f"No .stl files found in {args.in_dir}", file=sys.stderr)
        return 1

    print(f"Converting {len(stl_files)} .stl files from {args.in_dir}")
    failures = []
    for fname in stl_files:
        stl_path = os.path.join(args.in_dir, fname)
        stem = fname[: -len(".STL")] if fname.endswith(".STL") else fname[: -len(".stl")]
        glb_path = os.path.join(args.out_dir, stem + ".glb")
        try:
            convert_one(stl_path, glb_path)
        except Exception as exc:  # noqa: BLE001 - report and continue the batch
            failures.append((fname, exc))
            print(f"  {fname:24s} -> FAILED: {exc}")

    print()
    if failures:
        print(f"{len(failures)}/{len(stl_files)} conversions failed:")
        for fname, exc in failures:
            print(f"  - {fname}: {exc}")
        return 1

    print(f"All {len(stl_files)} conversions succeeded. Output: {args.out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
