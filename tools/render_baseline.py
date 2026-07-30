#!/usr/bin/env python3
"""Step 0: baseline render of the FR5WM URDF exactly as Fairino shipped it.

Fairino's FR5WM.urdf has a dangling `tool` joint whose child link
(`gripper_Link`) is commented out, and whose referenced gripper meshes were
never delivered. That makes the file invalid for any strict URDF loader. This
script does NOT touch the source file: it parses it, drops in memory any
joint whose child link isn't defined anywhere in the document (which today
is just `tool`), rewrites mesh `filename` attributes to absolute paths (so
the temp copy can live anywhere, not just next to the original meshes/ dir),
and writes the result to a temp file purely so a renderer can load it.

Usage:
    pip install pybullet pillow
    python3 tools/render_baseline.py \
        --urdf /path/to/frcobot_ros2/fairino_description/urdf/FR5WM.urdf \
        --out-dir tools/output/step0_baseline

Outputs into --out-dir:
    joint_tree.txt        plain-text parsed link/joint/limit manifest
    zero_pose_iso.png      isometric render at all-zero joint angles
    zero_pose_front.png    front-on render at all-zero joint angles
    pose_a_iso.png          isometric render at a non-trivial joint pose
    pose_b_iso.png          isometric render at a second non-trivial pose
"""
import argparse
import os
import sys
import tempfile
import xml.etree.ElementTree as ET

DEFAULT_URDF = (
    "/home/paige-trexel/projects/fairino_test/frcobot_ros2/"
    "fairino_description/urdf/FR5WM.urdf"
)

URDF_PACKAGE_PREFIX = "package://"


def resolve_mesh_path(filename: str, urdf_dir: str) -> str:
    """Resolve a URDF <mesh filename> to a real file on disk.

    For package:// URIs, mirrors RDK's own normalizeURDFMeshPath
    (referenceframe/xml_conversions.go): strip the "package://" prefix and
    the package-name segment, then resolve what's left relative to the
    URDF's own directory — this is exactly what RDK does, and it's only
    correct when meshes/ is a sibling of the urdf file itself. Fairino's
    "WM" variants happen to lay out that way, but the standard variants
    (e.g. fairino5_v6) put urdf/ and meshes/ as siblings under a shared
    package dir instead, which RDK's own resolution would NOT find — so as
    a fallback (for rendering purposes only) also try one directory up from
    the urdf dir, and say clearly which one worked.
    """
    candidate = filename
    if candidate.startswith(URDF_PACKAGE_PREFIX):
        candidate = candidate[len(URDF_PACKAGE_PREFIX):]
        if "/" in candidate:
            candidate = candidate.split("/", 1)[1]

    if os.path.isabs(candidate):
        return candidate

    rdk_style = os.path.abspath(os.path.join(urdf_dir, candidate))
    if os.path.isfile(rdk_style):
        return rdk_style

    parent_style = os.path.abspath(os.path.join(os.path.dirname(urdf_dir), candidate))
    if os.path.isfile(parent_style):
        print(
            f"  NOTE: {filename!r} did not resolve the way RDK's own "
            f"package:// handling would (relative to the urdf file's own "
            f"directory) — found it one directory up instead: {parent_style}"
        )
        return parent_style

    return rdk_style


def load_sanitized_urdf(urdf_path: str) -> tuple[str, list[dict]]:
    """Parse urdf_path, drop joints whose child link is undefined, absolutize
    mesh paths, and return (path_to_temp_urdf, joint_manifest)."""
    urdf_dir = os.path.dirname(os.path.abspath(urdf_path))
    tree = ET.parse(urdf_path)
    root = tree.getroot()

    link_names = {link.get("name") for link in root.findall("link")}

    manifest = []
    dropped = []
    for joint in list(root.findall("joint")):
        name = joint.get("name")
        jtype = joint.get("type")
        parent_el = joint.find("parent")
        child_el = joint.find("child")
        parent = parent_el.get("link") if parent_el is not None else None
        child = child_el.get("link") if child_el is not None else None

        if child not in link_names:
            dropped.append((name, child))
            root.remove(joint)
            continue

        origin_el = joint.find("origin")
        limit_el = joint.find("limit")
        manifest.append(
            {
                "name": name,
                "type": jtype,
                "parent": parent,
                "child": child,
                "xyz": origin_el.get("xyz") if origin_el is not None else "0 0 0",
                "rpy": origin_el.get("rpy") if origin_el is not None else "0 0 0",
                "lower": limit_el.get("lower") if limit_el is not None else None,
                "upper": limit_el.get("upper") if limit_el is not None else None,
            }
        )

    for mesh_el in root.iter("mesh"):
        fn = mesh_el.get("filename")
        if fn:
            mesh_el.set("filename", resolve_mesh_path(fn, urdf_dir))

    fd, tmp_path = tempfile.mkstemp(suffix=".urdf", prefix="fr5wm_baseline_")
    os.close(fd)
    tree.write(tmp_path)

    if dropped:
        print("Dropped joints with undefined child link (expected: 'tool' -> 'gripper_Link'):")
        for name, child in dropped:
            print(f"  - joint {name!r} -> missing child link {child!r}")
    else:
        print("No dangling joints found — nothing dropped.")

    return tmp_path, manifest


def write_manifest(manifest: list[dict], out_dir: str) -> None:
    path = os.path.join(out_dir, "joint_tree.txt")
    with open(path, "w") as f:
        f.write(f"{'joint':16s} {'type':10s} {'parent':16s} {'child':16s} {'xyz':16s} {'rpy':20s} {'lower':>8s} {'upper':>8s}\n")
        for j in manifest:
            f.write(
                f"{j['name']:16s} {j['type']:10s} {j['parent']:16s} {j['child']:16s} "
                f"{j['xyz']:16s} {j['rpy']:20s} "
                f"{str(j['lower']):>8s} {str(j['upper']):>8s}\n"
            )
    print(f"Wrote joint manifest: {path}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--urdf", default=DEFAULT_URDF)
    ap.add_argument("--out-dir", default=os.path.join(os.path.dirname(__file__), "output", "step0_baseline"))
    args = ap.parse_args()

    if not os.path.isfile(args.urdf):
        print(f"URDF not found: {args.urdf}", file=sys.stderr)
        return 1

    os.makedirs(args.out_dir, exist_ok=True)

    tmp_urdf, manifest = load_sanitized_urdf(args.urdf)
    write_manifest(manifest, args.out_dir)
    try:
        from _render_utils import render_poses

        render_poses(tmp_urdf, args.out_dir, search_path=os.path.dirname(args.urdf))
    finally:
        os.remove(tmp_urdf)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
