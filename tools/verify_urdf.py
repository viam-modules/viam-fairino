#!/usr/bin/env python3
"""Step 3: independent verification of the cleaned FR5WM URDF.

Three checks, run outside viam/RDK entirely, before the cleaned URDF
(tools/staging/fr5wm/urdf/fr5wm.urdf, produced in Step 1) is trusted as an
input to the module:

  1. Structural validity — every joint's parent/child link is defined.
     (This alone would have caught the original dangling "tool" joint.)
  2. Joint limits diff — j1-j6's type/origin/axis/limit values in the
     cleaned file must exactly match the untouched original source, since
     Step 1 was only supposed to touch the tool/gripper_Link joint and
     nothing else. No independent numeric source exists (Fairino's SDK only
     exposes live-robot getters, no static tables), so the pristine source
     file is the ground truth this compares against, not a hardcoded table.
  3. Visual sanity render — same procedure as Step 0's baseline, pointed at
     the cleaned file, so the two image sets can be flipped between and
     compared by eye.

Usage:
    source tools/.venv/bin/activate
    pip install -r tools/requirements.txt
    python3 tools/verify_urdf.py
"""
import argparse
import os
import sys
import xml.etree.ElementTree as ET

ORIGINAL_URDF = (
    "/home/paige-trexel/projects/fairino_test/frcobot_ros2/"
    "fairino_description/urdf/FR5WM.urdf"
)
CLEANED_URDF = os.path.join(
    os.path.dirname(__file__), "staging", "fr5wm", "urdf", "fr5wm.urdf"
)
OUT_DIR = os.path.join(os.path.dirname(__file__), "output", "step3_verification")

# Joints Step 1 intentionally changed. Everything else must match the
# original source exactly, or this check should fail loudly.
INTENTIONALLY_CHANGED_JOINTS = {"tool", "flange"}


def check_structural_validity(urdf_path: str) -> bool:
    print("== 1. Structural validity ==")
    root = ET.parse(urdf_path).getroot()
    link_names = {link.get("name") for link in root.findall("link")}
    ok = True
    for joint in root.findall("joint"):
        name = joint.get("name")
        parent = joint.find("parent").get("link")
        child = joint.find("child").get("link")
        joint_ok = parent in link_names and child in link_names
        ok &= joint_ok
        status = "OK" if joint_ok else "MISSING LINK"
        print(f"  joint {name:10s} {parent:14s} -> {child:14s} [{status}]")
    print(f"  -> {'PASS' if ok else 'FAIL'}\n")
    return ok


def _joint_signature(joint: ET.Element) -> dict:
    origin = joint.find("origin")
    axis = joint.find("axis")
    limit = joint.find("limit")
    return {
        "type": joint.get("type"),
        "parent": joint.find("parent").get("link"),
        "child": joint.find("child").get("link"),
        "xyz": origin.get("xyz") if origin is not None else None,
        "rpy": origin.get("rpy") if origin is not None else None,
        "axis": axis.get("xyz") if axis is not None else None,
        "lower": limit.get("lower") if limit is not None else None,
        "upper": limit.get("upper") if limit is not None else None,
    }


def check_joint_limits_diff(original_path: str, cleaned_path: str) -> bool:
    print("== 2. Joint limits diff (cleaned vs. untouched original) ==")
    orig_joints = {j.get("name"): _joint_signature(j) for j in ET.parse(original_path).getroot().findall("joint")}
    clean_joints = {j.get("name"): _joint_signature(j) for j in ET.parse(cleaned_path).getroot().findall("joint")}

    ok = True
    for name, orig_sig in orig_joints.items():
        if name in INTENTIONALLY_CHANGED_JOINTS:
            print(f"  {name:10s} skipped (intentionally changed by Step 1)")
            continue
        clean_sig = clean_joints.get(name)
        if clean_sig is None:
            print(f"  {name:10s} MISSING from cleaned file")
            ok = False
            continue
        if clean_sig != orig_sig:
            ok = False
            print(f"  {name:10s} MISMATCH")
            for key in orig_sig:
                if orig_sig[key] != clean_sig[key]:
                    print(f"      {key}: original={orig_sig[key]!r} cleaned={clean_sig[key]!r}")
        else:
            print(f"  {name:10s} matches original")

    unexpected_new = set(clean_joints) - set(orig_joints) - INTENTIONALLY_CHANGED_JOINTS
    if unexpected_new:
        ok = False
        print(f"  Unexpected new joints not in original and not flagged as intentional: {unexpected_new}")

    print(f"  -> {'PASS' if ok else 'FAIL'}\n")
    return ok


def render_visual_check(cleaned_path: str, out_dir: str) -> None:
    print("== 3. Visual sanity render ==")
    os.makedirs(out_dir, exist_ok=True)
    # <visual> now points at .glb (see render_assembled_glb.py) — pybullet's
    # URDF importer rejects .glb outright, so this uses the same
    # trimesh/matplotlib forward-kinematics renderer as that script rather
    # than _render_utils.render_poses (which is pybullet-based and only
    # works for .dae/.stl visual meshes).
    from render_assembled_glb import parse_chain, render_pose, scaled_pose

    links, joints = parse_chain(cleaned_path)
    render_pose(links, joints, {}, os.path.join(out_dir, "zero_pose.png"))
    render_pose(links, joints, scaled_pose(joints, 0.4), os.path.join(out_dir, "pose_a.png"))
    render_pose(links, joints, scaled_pose(joints, -0.3), os.path.join(out_dir, "pose_b.png"))

    baseline_dir = os.path.join(os.path.dirname(__file__), "output", "step0_baseline")
    print(f"  Wrote renders to {out_dir}")
    print(f"  These are .glb-based (assembled via forward kinematics, not pybullet) — compare")
    print(f"  against the .dae-based Step 0 baseline in {baseline_dir} for overall shape/pose,")
    print(f"  not pixel-for-pixel (different renderer, different lighting).\n")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--original", default=ORIGINAL_URDF)
    ap.add_argument("--cleaned", default=CLEANED_URDF)
    ap.add_argument("--out-dir", default=OUT_DIR)
    args = ap.parse_args()

    for path in (args.original, args.cleaned):
        if not os.path.isfile(path):
            print(f"URDF not found: {path}", file=sys.stderr)
            return 1

    structural_ok = check_structural_validity(args.cleaned)
    limits_ok = check_joint_limits_diff(args.original, args.cleaned)
    render_visual_check(args.cleaned, args.out_dir)

    if structural_ok and limits_ok:
        print("All automated checks PASSED. Visual render still needs eyeballing.")
        return 0
    print("One or more automated checks FAILED — see above.")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
