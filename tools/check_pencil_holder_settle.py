"""Report pencil_holder_1's resting position after a full ReKepOGEnv load, for either robot.

    python tools/check_pencil_holder_settle.py --config ./configs/config.yaml
    python tools/check_pencil_holder_settle.py --config ./configs/config_widowxai.yaml

Used to root-cause and then verify the fix for pencil_holder_1 ending up on the floor under the
WidowX AI config -- see docs/widowxai_bringup_status.md, "Stage 4, attempt 4". Short version: the
scene file (configs/og_scene_file_pen.json) was never the problem, despite an earlier, wrong
conclusion in this doc that it was. The actual cause was configs/config_widowxai.yaml spawning the
robot's base_link ~0.7cm into the table (an approximated z, never checked against the table's own
measured aabb or the robot's own base geometry); resolving that overlap during the scene's settle
sequence was disturbing pencil_holder_1's contact with the table via the physics solver, even
though the table's own reported position never visibly moved. Fixed by correcting the robot's
spawn height, not by moving the object -- two earlier attempts to fix this by editing the object's
position directly (in og_scene_file_pen.json) did not work, and the second one made it worse.

This script does NOT modify anything. It loads the real scene through ReKepOGEnv (the same path
main.py uses) for whichever --config is given, reports table_1/pencil_holder_1's position and aabb
right after load, then steps physics 60 more times and reports again, so a regression shows up as
the second report differing from the first (or from the equivalent run for the other robot).
"""
import argparse
import sys

import numpy as np

sys.path.insert(0, ".")


def main():
    import omnigibson as og

    p = argparse.ArgumentParser()
    p.add_argument("--config", default="./configs/config.yaml")
    args = p.parse_args()

    # Reuse ReKep's own environment wrapper (same path main.py uses) rather than hand-building
    # an og.Environment config -- ReKepOGEnv knows how to translate config.yaml's `env` block
    # and scene_file into what og.Environment actually needs.
    from environment import ReKepOGEnv
    from robots.widowxai import WidowXAI  # noqa: F401  -- registers the class; needed for --config config_widowxai.yaml
    from utils import get_config

    global_config = get_config(config_path=args.config)
    rekep_env = ReKepOGEnv(global_config['env'], './configs/og_scene_file_pen.json', verbose=False)
    env = rekep_env.og_env
    table = env.scene.object_registry("name", "table_1")
    holder = env.scene.object_registry("name", "pencil_holder_1")

    def to_np(x):
        return np.asarray(x.cpu() if hasattr(x, "cpu") else x, dtype=float)

    def aabb(obj):
        lo, hi = obj.aabb
        return to_np(lo), to_np(hi)

    def report(tag, obj):
        lo, hi = aabb(obj)
        pos, orn = obj.get_position_orientation()
        print(f"{tag}: pos={np.round(to_np(pos),4).tolist()} "
              f"aabb x=[{lo[0]:.4f},{hi[0]:.4f}] y=[{lo[1]:.4f},{hi[1]:.4f}] z=[{lo[2]:.4f},{hi[2]:.4f}]")
        return lo, hi, to_np(pos), to_np(orn)

    report("table_1 (after ReKepOGEnv load)", table)
    report("pencil_holder_1 (after ReKepOGEnv load)", holder)

    robot = env.robots[0]
    r_pos, r_orn = robot.get_position_orientation()
    print(f"robot: type={type(robot).__name__} pos={np.round(to_np(r_pos),4).tolist()}")
    base_link = robot.links.get("base_link", None)
    if base_link is not None:
        b_lo, b_hi = to_np(base_link.aabb[0]), to_np(base_link.aabb[1])
        t_lo, t_hi = aabb(table)
        overlap_z = min(b_hi[2], t_hi[2]) - max(b_lo[2], t_lo[2])
        print(f"robot base_link aabb: x=[{b_lo[0]:.4f},{b_hi[0]:.4f}] y=[{b_lo[1]:.4f},{b_hi[1]:.4f}] "
              f"z=[{b_lo[2]:.4f},{b_hi[2]:.4f}]  vs table z-overlap: {overlap_z:.4f} m "
              f"({'INTERPENETRATING' if overlap_z > 0 else 'clear'})")

    print("\nstepping 60 more physics steps...")
    for _ in range(60):
        og.sim.step()
    report("pencil_holder_1 (after 60 more steps)", holder)

    og.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
