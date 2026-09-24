"""Measure how far behind the eef tip the actual grasp zone (between the open jaws) sits.

    python tools/check_grasp_depth.py

Why this exists: the pen-grasp attempt in Stage 4 closes the gripper above the pen rather than
around it. The [GRASP AXES] log shows the raw (unclipped) descent target lands ~2.7cm below the
table top (by design -- main.py's _execute_grasp_action comment says the advance is "deliberately
aimed through the object"), and the workspace-bounds clip (environment.py:666-669, bounds_min z)
then stops the eef short of that, at bounds_min[2] -- a value still copied verbatim from Fetch
(configs/config_widowxai.yaml notes this explicitly). Where the eef ORIGIN (~= the fingertip
plane, per docs/widowxai_bringup_status.md's measurement) ends up is not the same as where the
gripper actually closes around an object: the jaws open along local y and the graspable region
between them sits some distance *behind* the tip along local z (toward the wrist), not at the tip
itself.

This script measures that offset directly, using the same _ag_start_points/_ag_end_points already
hand-authored in robots/widowxai.py (tools/derive_ag_points.py's output) -- their midpoint is
exactly "the region between the jaws where an object is expected to sit" by construction. Reports
that midpoint's local z in the EEF FRAME (not the finger-local frame the GraspingPoints are stored
in), which is directly comparable to grasp_depth and bounds_min[2].
"""
import sys

import numpy as np


def main():
    import omnigibson as og
    import omnigibson.utils.transform_utils as T

    sys.path.insert(0, ".")
    from robots.widowxai import WidowXAI  # noqa: F401

    cfg = {
        "scene": {"type": "Scene"},
        "robots": [{"type": "WidowXAI", "name": "WidowXAI", "position": [0.0, 0.0, 0.0]}],
    }
    env = og.Environment(configs=cfg)
    robot = env.robots[0]
    for _ in range(10):
        og.sim.step()

    def to_np(x):
        return np.asarray(x.cpu() if hasattr(x, "cpu") else x, dtype=float)

    arm = robot.default_arm
    eef_link = robot.eef_links[arm]
    world_to_eef_tf = to_np(T.pose2mat(eef_link.get_position_orientation()))
    eef_to_world_tf = to_np(T.pose_inv(T.pose2mat(eef_link.get_position_orientation())))

    def to_world(points):
        out = []
        for gp in points:
            link = robot.links[gp.link_name]
            link_to_world_tf = to_np(T.pose2mat(link.get_position_orientation()))
            p_h = np.concatenate([np.asarray(gp.position, dtype=float), [1.0]])
            out.append((link_to_world_tf @ p_h)[:3])
        return np.array(out)

    starts_world = to_world(robot._ag_start_points)
    ends_world = to_world(robot._ag_end_points)
    all_world = np.concatenate([starts_world, ends_world], axis=0)
    print("AG points, world frame:")
    for p in all_world:
        print(f"  {np.round(p, 4).tolist()}")

    # Transform into eef-local frame to read off the z (approach-axis) coordinate directly.
    all_h = np.concatenate([all_world, np.ones((len(all_world), 1))], axis=1)
    all_eef = (all_h @ eef_to_world_tf.T)[:, :3]
    print("\nSame points, eef-local frame (column 2 = approach axis, tip = 0):")
    for p in all_eef:
        print(f"  {np.round(p, 4).tolist()}")

    z_local = all_eef[:, 2]
    print(f"\nz_local (eef frame) range: [{z_local.min():.4f}, {z_local.max():.4f}]")
    print(f"z_local (eef frame) mean:  {z_local.mean():.4f}")
    print("(negative = behind the tip, toward the wrist; this is how far past bounds_min the")
    print(" eef origin needs to travel for the JAWS -- not the tip -- to reach an object)")

    og.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
