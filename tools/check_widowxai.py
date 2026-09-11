"""Bring-up checks for the WidowX AI, run before pointing ReKep at it.

    python tools/check_widowxai.py

Four things, in the order they can fail:

1. **It loads.** The USD converted from the URDF is actually a valid articulated robot, and the
   link and joint names the robot class declares all exist in it. Most import mistakes surface
   here as a KeyError on a name that did not survive the conversion.

2. **The end-effector frame follows OmniGibson's convention** -- z out of the fingertips, y from
   the left finger to the right. This is the check chapter 11 had to make for Fetch the hard way,
   after a grasp failure traced back to `subgoal_solver.py` optimising the wrong column of the
   eef rotation matrix; upstream issue #24 is a third party hitting the same thing porting to
   Franka. The URDF's native approach axis is x, and the import config rotates a generated
   eef_link to fix that -- but the URDF saying so is not evidence that the conversion produced
   it, so measure the loaded robot.

3. **Assisted grasping has ray endpoints.** ReKep requires `grasping_mode: assisted`, and
   ManipulationRobot derives the start/end points from the finger link geometry rather than from
   anything hand-authored. That derivation asserts the points straddle the eef z axis
   (manipulation_robot.py, "Expected computed z_lower / z_upper bounds ... to be negative /
   positive"), so it fails loudly if the eef frame and the fingers disagree -- which makes it a
   second, independent check on item 2.

4. **The IK controller moves the arm.** A small Cartesian delta in, measurable end-effector
   motion out. This is the path `_move_to_waypoint` drives, so if it does not work here it will
   not work under ReKep either.

Exit status is 0 only if all four pass. Nothing here touches the Fetch setup.
"""
import sys

import numpy as np


def main():
    import omnigibson as og
    from omnigibson.macros import gm

    gm.USE_GPU_DYNAMICS = True
    gm.ENABLE_FLATCACHE = False

    # Registers the class with OmniGibson under the name config uses.
    sys.path.insert(0, ".")
    from robots.widowxai import WidowXAI  # noqa: F401

    failures = []

    print("=" * 72)
    print("1. load")
    print("=" * 72)
    cfg = {
        "scene": {"type": "Scene"},
        "robots": [
            {
                "type": "WidowXAI",
                "name": "WidowXAI",
                "obs_modalities": ["rgb"],
                "action_normalize": False,
                "grasping_mode": "assisted",
                "position": [0.0, 0.0, 0.0],
                "controller_config": {
                    "arm_0": {"name": "InverseKinematicsController", "mode": "pose_delta_ori"},
                    "gripper_0": {
                        "name": "MultiFingerGripperController",
                        "command_input_limits": [0.0, 1.0],
                        "mode": "smooth",
                    },
                },
            }
        ],
    }
    env = og.Environment(configs=cfg)
    robot = env.robots[0]
    for _ in range(10):
        og.sim.step()

    arm = robot.default_arm
    print(f"   loaded {robot.name}: {robot.n_joints} joints, action_dim {robot.action_dim}")
    print(f"   arm joints   : {robot.arm_joint_names[arm]}")
    print(f"   finger joints: {robot.finger_joint_names[arm]}")
    print(f"   eef link     : {robot.eef_link_names[arm]}")
    print(f"   controllers  : {list(robot.controller_action_idx.keys())}")
    print(f"   action layout: { {k: np.asarray(v).tolist() for k, v in robot.controller_action_idx.items()} }")

    arm_dim = len(np.asarray(robot.controller_action_idx[f"arm_{arm}"]))
    if arm_dim != 6:
        failures.append(f"arm controller takes {arm_dim} commands, ReKep's _move_to_waypoint sends 6")

    print()
    print("=" * 72)
    print("2. end-effector frame")
    print("=" * 72)
    eef_pos, eef_quat = robot.get_eef_position(arm), robot.get_eef_orientation(arm)
    eef_pos = np.asarray(eef_pos.cpu() if hasattr(eef_pos, "cpu") else eef_pos, dtype=float)
    q = np.asarray(eef_quat.cpu() if hasattr(eef_quat, "cpu") else eef_quat, dtype=float)
    x, y, z, w = q
    R = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
    print(f"   eef world position  {np.round(eef_pos, 4).tolist()}")
    print(f"   column 0 (local x) -> {np.round(R[:, 0], 4).tolist()}")
    print(f"   column 1 (local y) -> {np.round(R[:, 1], 4).tolist()}")
    print(f"   column 2 (local z) -> {np.round(R[:, 2], 4).tolist()}")

    # Where do the fingers actually sit? Their separation direction is the ground truth for the
    # y axis, and the direction from the eef toward their midpoint is the ground truth for z.
    fl, fr = robot.finger_links[arm][0], robot.finger_links[arm][1]
    fpos = []
    for link in (fl, fr):
        p = link.get_position_orientation()[0]
        fpos.append(np.asarray(p.cpu() if hasattr(p, "cpu") else p, dtype=float))
    sep = fpos[0] - fpos[1]
    sep = sep / np.linalg.norm(sep)
    print(f"   finger links: {fl.body_name} at {np.round(fpos[0], 4).tolist()}")
    print(f"                 {fr.body_name} at {np.round(fpos[1], 4).tolist()}")
    print(f"   finger separation axis {np.round(sep, 4).tolist()}")
    align_y = abs(float(R[:, 1] @ sep))
    print(f"   |dot(column 1, finger separation)| = {align_y:.4f}   (want ~1: y separates the fingers)")
    if align_y < 0.9:
        failures.append(f"eef y axis is not the finger separation axis (|dot| = {align_y:.3f})")

    print()
    print("=" * 72)
    print("3. assisted-grasp ray endpoints")
    print("=" * 72)
    try:
        starts = robot.assisted_grasp_start_points[arm]
        ends = robot.assisted_grasp_end_points[arm]
        if not starts or not ends:
            failures.append("assisted grasp points are empty; assisted grasping will never trigger")
        else:
            print(f"   {len(starts)} start points, {len(ends)} end points")
            for tag, pts in (("start", starts), ("end", ends)):
                for p in pts:
                    print(f"     {tag:<5} {p.link_name:<16} {np.round(np.asarray(p.position), 5).tolist()}")
    except Exception as e:
        failures.append(f"assisted grasp point derivation failed: {type(e).__name__}: {e}")
        print(f"   FAILED: {e}")

    print()
    print("=" * 72)
    print("4. IK controller moves the arm")
    print("=" * 72)
    before = np.asarray((lambda p: p.cpu() if hasattr(p, "cpu") else p)(robot.get_eef_position(arm)), dtype=float)
    action = np.zeros(robot.action_dim)
    arm_idx = np.asarray(robot.controller_action_idx[f"arm_{arm}"]).astype(int)
    action[arm_idx[:3]] = [0.02, 0.0, 0.0]   # 2 cm along the robot frame's x
    for _ in range(30):
        env.step(action)
    after = np.asarray((lambda p: p.cpu() if hasattr(p, "cpu") else p)(robot.get_eef_position(arm)), dtype=float)
    moved = float(np.linalg.norm(after - before))
    print(f"   before {np.round(before, 4).tolist()}")
    print(f"   after  {np.round(after, 4).tolist()}")
    print(f"   moved {moved * 100:.2f} cm over 30 steps of a 2 cm/step +x command")
    if moved < 0.005:
        failures.append(f"eef barely moved ({moved * 100:.2f} cm); the IK controller is not driving the arm")

    print()
    print("=" * 72)
    if failures:
        print(f"FAILED ({len(failures)})")
        for f in failures:
            print(f"  - {f}")
    else:
        print("all four checks passed")
    print("=" * 72)

    og.shutdown()
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
