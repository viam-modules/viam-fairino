"""Shared headless pybullet rendering helper for tools/ scripts.

Not a public script itself — imported by render_baseline.py (Step 0) and
verify_urdf.py (Step 3), which both need "load a URDF, snapshot a few joint
poses" and otherwise differ in what they do before/after that.
"""
import os


def render_poses(urdf_path: str, out_dir: str, search_path: str | None = None) -> None:
    """Load urdf_path in headless pybullet and write PNG snapshots into out_dir:
    zero_pose_iso, zero_pose_front, pose_a_iso, pose_b_iso."""
    import pybullet as p

    client = p.connect(p.DIRECT)
    p.setAdditionalSearchPath(search_path or os.path.dirname(urdf_path))
    body = p.loadURDF(urdf_path, useFixedBase=True, flags=p.URDF_USE_INERTIA_FROM_FILE)

    num_joints = p.getNumJoints(body)
    revolute = [
        i for i in range(num_joints)
        if p.getJointInfo(body, i)[2] == p.JOINT_REVOLUTE
    ]
    print(f"Loaded body with {num_joints} joints, {len(revolute)} revolute.")

    def snapshot(name: str, joint_angles: dict[int, float], eye: tuple, target=(0, 0, 0.4)):
        for i in range(num_joints):
            p.resetJointState(body, i, joint_angles.get(i, 0.0))
        view = p.computeViewMatrix(cameraEyePosition=eye, cameraTargetPosition=target, cameraUpVector=(0, 0, 1))
        proj = p.computeProjectionMatrixFOV(fov=50, aspect=1.33, nearVal=0.05, farVal=5.0)
        width, height, rgb, _, _ = p.getCameraImage(
            960, 720, viewMatrix=view, projectionMatrix=proj, renderer=p.ER_TINY_RENDERER
        )
        from PIL import Image
        import numpy as np

        arr = np.reshape(rgb, (height, width, 4)).astype("uint8")
        img_path = os.path.join(out_dir, f"{name}.png")
        Image.fromarray(arr, "RGBA").convert("RGB").save(img_path)
        print(f"Wrote {img_path}")

    zero: dict[int, float] = {}
    snapshot("zero_pose_iso", zero, eye=(1.2, 1.2, 1.0))
    snapshot("zero_pose_front", zero, eye=(0.0, 2.0, 0.4))

    def scaled_pose(frac: float) -> dict[int, float]:
        pose = {}
        for i in revolute:
            lo, hi = p.getJointInfo(body, i)[8], p.getJointInfo(body, i)[9]
            if hi > lo:
                pose[i] = lo + (hi - lo) * ((frac + 1) / 2)
        return pose

    snapshot("pose_a_iso", scaled_pose(0.4), eye=(1.2, 1.2, 1.0))
    snapshot("pose_b_iso", scaled_pose(-0.3), eye=(1.2, 1.2, 1.0))

    p.disconnect(client)
