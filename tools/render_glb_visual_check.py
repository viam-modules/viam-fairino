#!/usr/bin/env python3
"""Confirm the Step 2 .dae -> .glb conversion didn't change visual appearance.

Step 3's render never actually exercised the .glb output — it rendered the
cleaned URDF, whose <visual> tags still point at the original .dae files
(Step 1 never touched them). This script builds a throwaway URDF variant
that instead points <visual> at the converted .glb files (leaving
<collision>, which drives RDK's actual kinematics, untouched) and renders
it through the same pybullet harness at the same poses as Step 0/Step 3, so
the two image sets can be compared directly.

If this looks right, the eventual shipped arm/fr5wm.urdf should point
<visual> at these same .glb files (the ones Get3DModels also serves) rather
than shipping .dae at all — one asset for both purposes.

Usage:
    source tools/.venv/bin/activate
    python3 tools/render_glb_visual_check.py

Reads:   tools/staging/fr5wm/urdf/fr5wm.urdf (Step 1 output)
         tools/staging/fr5wm/meshes/FR5WM/glb/*.glb (Step 2 output)
Writes:  tools/output/glb_visual_check/*.png
"""
import argparse
import os
import sys
import tempfile
import xml.etree.ElementTree as ET

CLEANED_URDF = os.path.join(
    os.path.dirname(__file__), "staging", "fr5wm", "urdf", "fr5wm.urdf"
)
GLB_DIR = os.path.join(
    os.path.dirname(__file__), "staging", "fr5wm", "meshes", "FR5WM", "glb"
)
OUT_DIR = os.path.join(os.path.dirname(__file__), "output", "glb_visual_check")


def build_glb_visual_urdf(urdf_path: str, glb_dir: str) -> str:
    urdf_dir = os.path.dirname(os.path.abspath(urdf_path))
    tree = ET.parse(urdf_path)
    root = tree.getroot()

    rewritten, missing = [], []
    for link in root.findall("link"):
        visual = link.find("visual")
        if visual is None:
            continue
        mesh_el = visual.find("geometry/mesh")
        if mesh_el is None:
            continue
        original = mesh_el.get("filename")
        glb_path = os.path.join(glb_dir, os.path.splitext(os.path.basename(original))[0] + ".glb")
        if not os.path.isfile(glb_path):
            missing.append((link.get("name"), glb_path))
            continue
        mesh_el.set("filename", glb_path)
        rewritten.append((link.get("name"), glb_path))

    # Collision meshes are untouched — they're what RDK actually consumes,
    # and aren't the thing being checked here. Still need to be resolvable
    # for pybullet to load the body at all.
    for mesh_el in root.iter("mesh"):
        fn = mesh_el.get("filename")
        if fn and not os.path.isabs(fn):
            mesh_el.set("filename", os.path.abspath(os.path.join(urdf_dir, fn)))

    print(f"Rewrote {len(rewritten)} <visual> mesh refs to .glb:")
    for name, path in rewritten:
        print(f"  {name:16s} -> {path}")
    if missing:
        print(f"WARNING: {len(missing)} links have no matching .glb (run tools/convert_dae_to_glb.py first):")
        for name, path in missing:
            print(f"  {name:16s} expected {path}")

    fd, tmp_path = tempfile.mkstemp(suffix=".urdf", prefix="fr5wm_glb_visual_")
    os.close(fd)
    tree.write(tmp_path)
    return tmp_path


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--urdf", default=CLEANED_URDF)
    ap.add_argument("--glb-dir", default=GLB_DIR)
    ap.add_argument("--out-dir", default=OUT_DIR)
    args = ap.parse_args()

    if not os.path.isfile(args.urdf):
        print(f"URDF not found: {args.urdf}", file=sys.stderr)
        return 1
    if not os.path.isdir(args.glb_dir):
        print(f"GLB dir not found: {args.glb_dir} — run tools/convert_dae_to_glb.py first", file=sys.stderr)
        return 1

    os.makedirs(args.out_dir, exist_ok=True)
    tmp_urdf = build_glb_visual_urdf(args.urdf, args.glb_dir)
    try:
        from _render_utils import render_poses

        render_poses(tmp_urdf, args.out_dir, search_path=os.path.dirname(args.urdf))
    finally:
        os.remove(tmp_urdf)

    print(f"\nWrote renders to {args.out_dir}")
    print(f"Compare against tools/output/step0_baseline/ and tools/output/step3_verification/ (both .dae-based)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
