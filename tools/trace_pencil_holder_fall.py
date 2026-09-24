"""Step-by-step trace of what knocks pencil_holder_1 off the table when WidowX AI loads.

    python tools/trace_pencil_holder_fall.py --config ./configs/config_widowxai.yaml
    python tools/trace_pencil_holder_fall.py --config ./configs/config.yaml

Earlier measurement (tools/check_pencil_holder_settle.py) found pencil_holder_1 resting correctly ON
the table when loaded through the Fetch config, but already on the floor with the WidowX config --
even though configs/og_scene_file_pen.json is byte-identical either way (confirmed via git diff
against port/behavior1k-v3.7.2). That measurement ran through ReKepOGEnv, whose __init__ already
runs og.Environment(...) AND a 30-step "let everything come to rest" settle loop before control
returns -- so "before physics" in that script was actually already 30+ steps in. This script
builds the same og.Environment config directly (bypassing ReKepOGEnv) so it can inspect state
frame-by-frame from t=0, to see exactly when and why the holder starts moving.

This script alone was inconclusive: a bare og.Environment() + 40 steps under the WidowX config did
NOT reproduce the fall (the holder stayed put throughout), even though base_link did measure as
interpenetrating table_1 by 0.0073m at t=0 (see the printout below). The disturbance turned out to
be specific to ReKepOGEnv's own settle sequence (_zero_object_velocities() plus the 30/10-step
loops) -- confirmed by temporarily instrumenting ReKepOGEnv.__init__ per-step, which showed
velocity already nonzero on step 0 (right after being explicitly zeroed) under the WidowX config
but at floating-point noise under Fetch, then a slow ~20-step drift off the table edge under
WidowX only. table_1's own reported position never moved in either trace, ruling out "the table
visibly gets knocked" -- the interpenetration most likely perturbs the holder's contact with the
table via the physics solver treating bodies sharing a contact with table_1 as one island, without
needing table_1's own reported pose to move. Root cause and fix (the robot's spawn z was ~0.7cm
too low) are in docs/widowxai_bringup_status.md, "Stage 4, attempt 4". This script remains useful
for re-confirming the base/table interpenetration number and the t=0 aabbs on demand.
"""
import argparse
import sys

import numpy as np

sys.path.insert(0, ".")


def main():
    import omnigibson as og

    p = argparse.ArgumentParser()
    p.add_argument("--config", default="./configs/config_widowxai.yaml")
    p.add_argument("--steps", type=int, default=40)
    args = p.parse_args()

    from robots.widowxai import WidowXAI  # noqa: F401 -- registers the class
    from utils import get_config

    global_config = get_config(config_path=args.config)
    env_config = global_config['env']
    env_config['scene']['scene_file'] = './configs/og_scene_file_pen.json'

    env = og.Environment(dict(scene=env_config['scene'], robots=[env_config['robot']['robot_config']], env=env_config['og_sim']))

    def to_np(x):
        return np.asarray(x.cpu() if hasattr(x, "cpu") else x, dtype=float)

    table = env.scene.object_registry("name", "table_1")
    holder = env.scene.object_registry("name", "pencil_holder_1")
    robot = env.robots[0]

    def aabb(obj):
        lo, hi = obj.aabb
        return to_np(lo), to_np(hi)

    t_lo, t_hi = aabb(table)
    print(f"table_1 aabb (t=0, before any step): x=[{t_lo[0]:.4f},{t_hi[0]:.4f}] "
          f"y=[{t_lo[1]:.4f},{t_hi[1]:.4f}] z=[{t_lo[2]:.4f},{t_hi[2]:.4f}]")

    pen = env.scene.object_registry("name", "pen_1")
    if pen is not None:
        p_lo, p_hi = aabb(pen)
        print(f"pen_1 aabb (t=0, before any step): x=[{p_lo[0]:.4f},{p_hi[0]:.4f}] "
              f"y=[{p_lo[1]:.4f},{p_hi[1]:.4f}] z=[{p_lo[2]:.4f},{p_hi[2]:.4f}]")

    r_pos, r_orn = robot.get_position_orientation()
    print(f"robot pos (t=0): {np.round(to_np(r_pos), 4).tolist()}")
    base_link = robot.links.get("base_link", None)
    if base_link is not None:
        b_lo, b_hi = to_np(base_link.aabb[0]), to_np(base_link.aabb[1])
        print(f"robot base_link aabb (t=0): x=[{b_lo[0]:.4f},{b_hi[0]:.4f}] "
              f"y=[{b_lo[1]:.4f},{b_hi[1]:.4f}] z=[{b_lo[2]:.4f},{b_hi[2]:.4f}]")
        overlap_z = min(b_hi[2], t_hi[2]) - max(b_lo[2], t_lo[2])
        print(f"base_link vs table_1 z-overlap at t=0: {overlap_z:.4f} m "
              f"({'INTERPENETRATING' if overlap_z > 0 else 'clear'})")

    h_lo, h_hi = aabb(holder)
    print(f"pencil_holder_1 aabb (t=0): x=[{h_lo[0]:.4f},{h_hi[0]:.4f}] "
          f"y=[{h_lo[1]:.4f},{h_hi[1]:.4f}] z=[{h_lo[2]:.4f},{h_hi[2]:.4f}]")

    print(f"\nstepping {args.steps} times, logging pencil_holder_1 z and contacts each step...")
    for i in range(args.steps):
        og.sim.step()
        lo, hi = aabb(holder)
        pos, _ = holder.get_position_orientation()
        contacts = list(holder.states[og.object_states.ContactBodies].get_value()) if hasattr(holder, "states") else []
        contact_names = [c.prim_path for c in contacts] if contacts else []
        moved_flag = ""
        if i == 0 or abs(lo[2] - prev_z) > 0.002:
            moved_flag = "  <== z jump"
        prev_z = lo[2]
        print(f"  step {i:3d}: pos={np.round(to_np(pos),4).tolist()} z_aabb=[{lo[2]:.4f},{hi[2]:.4f}] "
              f"contacts={contact_names}{moved_flag}")

    og.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
