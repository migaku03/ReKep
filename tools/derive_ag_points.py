"""Compute assisted-grasp ray endpoints for WidowX AI's fingers, print them as GraspingPoints.

    python tools/derive_ag_points.py

Why this exists: OmniGibson's own auto-inference (`ManipulationRobot._infer_finger_properties`,
manipulation_robot.py) finds each finger's "parent" by searching for a joint with nonzero DOF
whose body1 IS the finger link -- i.e. it assumes the finger link is itself the direct child of
the driven joint. That is true for every gripper OmniGibson ships (Fetch, VX300S, Franka's
default gripper), but not for the WidowX AI: the driven prismatic joint's child is the carriage
(`left_carriage_joint` -> `carriage_left`), and the actual contact pad (`gripper_left`) hangs off
the carriage through a second, *fixed* joint. Fixed joints have zero DOF and are excluded from
`self.joints` entirely (entity_prim.py:update_joints only wraps joints with joint_dof_counts > 0),
so the lookup finds nothing and _infer_finger_properties raises:

    AssertionError: Expected articulated parent joint for finger link WidowXAI:gripper_left
    but found none!

That's caught and downgraded to a warning by `_initialize()`, which is why the robot still loads
-- but `_default_ag_start_points`/`_default_ag_end_points` never get populated, and
`assisted_grasp_start_points` (manipulation_robot.py) raises KeyError on the arm name.

Rather than declare `finger_link_names` as the carriages (which would place the grasp rays on the
carriage geometry, short of where the pads actually contact an object), this script reproduces
the same geometric derivation the built-in inference uses, substituting `link_6` for the
"joint-connected parent" -- correct here because link_6 is body0 of *both* carriage joints, i.e.
the true common base the fingers extend from, joint-lookup aside. The output is meant to be
pasted into `_ag_start_points`/`_ag_end_points` in robots/widowxai.py, following the same
hand-authored-GraspingPoint pattern `omnigibson/robots/franka.py` uses for every one of its
end-effector options.
"""
import sys

import torch as th


def main():
    import omnigibson as og
    from omnigibson.robots import manipulation_robot as mr_module
    m = mr_module.m
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

    arm = robot.default_arm
    eef_link = robot.eef_links[arm]
    parent_link = robot.links["link_6"]
    finger_links = robot.finger_links[arm]
    assert len(finger_links) == 2, finger_links

    world_to_eef_tf = T.pose2mat(eef_link.get_position_orientation())
    eef_to_world_tf = T.pose_inv(world_to_eef_tf)

    parent_pts = parent_link.collision_boundary_points_world
    parent_pts = th.concatenate([parent_pts, th.ones(len(parent_pts), 1)], dim=-1)
    parent_pts = (parent_pts @ eef_to_world_tf.T)[:, :3]
    parent_max_z = parent_pts[:, 2].max().item()

    finger_pts_in_eef_frame = []
    for finger_link in finger_links:
        pts = finger_link.collision_boundary_points_world
        pts = th.concatenate([pts, th.ones(len(pts), 1)], dim=-1)
        pts = (pts @ eef_to_world_tf.T)[:, :3]
        finger_pts_in_eef_frame.append(pts)

    means = [p[:, 1].mean().item() for p in finger_pts_in_eef_frame]
    first_is_lower = means[0] < means[1]
    is_lower_y = [first_is_lower, not first_is_lower]

    tags = ("start (_ag_start_points)", "end (_ag_end_points)")
    for i, (finger_link, pts, lower_y, tag) in enumerate(
        zip(finger_links, finger_pts_in_eef_frame, is_lower_y, tags)
    ):
        finger_max_z = pts[:, 2].max().item()
        print(f"{finger_link.body_name}: finger_max_z={finger_max_z:.5f}  parent_max_z={parent_max_z:.5f}")
        assert finger_max_z > parent_max_z, "finger does not extend past link_6 along the eef z axis"
        finger_range = finger_max_z - parent_max_z

        y_min, y_max = pts[:, 1].min().item(), pts[:, 1].max().item()
        y_offset = y_max if lower_y else y_min
        y_sign = 1.0 if lower_y else -1.0

        # Upstream's formula (manipulation_robot.py:381-394) additionally clamps both ends into
        # +/-finger_range * AG_DEFAULT_GRASP_POINT_Z_PROP and asserts the pair straddles the eef's
        # z=0 plane. That assumes the eef origin sits well *behind* the fingertip, with room to
        # spare on both sides -- true for Fetch/Panda/VX300S, where the eef link is set back at
        # the wrist. WidowX AI's `ee_gripper` offset instead puts the eef origin right at the
        # pad's own tip (finger_max_z came out at +0.00037 m -- a hair's width past zero), so
        # there is no room in front of z=0 for a point to land in and the straddle assertion is
        # unsatisfiable. Dropped here in favour of two points spanning the same 20%-95% window of
        # the finger's own physical length, measured from its base at the parent (link_6) rather
        # than from the eef's z=0 plane -- geometrically the same idea (well up the finger, short
        # of the very tip and short of the very base), it just does not require that span to
        # straddle a specific plane.
        z_lower = parent_max_z + finger_range * m.MIN_AG_DEFAULT_GRASP_POINT_PROP
        z_upper = parent_max_z + finger_range * m.MAX_AG_DEFAULT_GRASP_POINT_PROP
        assert z_lower < z_upper <= finger_max_z, (z_lower, z_upper, finger_max_z)

        grasp_pts_eef = th.tensor(
            [
                [0, y_offset + 0.002 * y_sign, z_lower, 1],
                [0, y_offset + 0.002 * y_sign, z_upper, 1],
            ]
        )
        finger_to_world_tf = T.pose_inv(T.pose2mat(finger_link.get_position_orientation()))
        finger_to_eef_tf = finger_to_world_tf @ world_to_eef_tf
        grasp_pts_local = (grasp_pts_eef @ finger_to_eef_tf.T)[:, :3]

        print(f"  # {tag}")
        for p in grasp_pts_local:
            print(
                f'  GraspingPoint(link_name="{finger_link.body_name}", '
                f"position=th.tensor([{p[0]:.5f}, {p[1]:.5f}, {p[2]:.5f}])),"
            )

    og.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
