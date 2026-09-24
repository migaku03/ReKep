"""Report the physical dimensions of every task object and the robot, for a scale sanity check.

    python tools/measure_scene_scale.py --config ./configs/config_widowxai.yaml

Prints each object's aabb extents (world-frame, so table/pen/holder read as axis-aligned boxes
regardless of orientation) plus the robot's kinematic reach and base footprint, so the numbers can
be compared directly against real-world reference sizes.
"""
import argparse
import sys

import numpy as np

sys.path.insert(0, ".")


def main():
    import omnigibson as og

    p = argparse.ArgumentParser()
    p.add_argument("--config", default="./configs/config_widowxai.yaml")
    args = p.parse_args()

    from environment import ReKepOGEnv
    from robots.widowxai import WidowXAI  # noqa: F401
    from utils import get_config

    global_config = get_config(config_path=args.config)
    rekep_env = ReKepOGEnv(global_config['env'], './configs/og_scene_file_pen.json', verbose=False)
    env = rekep_env.og_env

    def to_np(x):
        return np.asarray(x.cpu() if hasattr(x, "cpu") else x, dtype=float)

    def report(name):
        obj = env.scene.object_registry("name", name)
        if obj is None:
            print(f"{name}: NOT FOUND")
            return
        lo, hi = obj.aabb
        lo, hi = to_np(lo), to_np(hi)
        ext = hi - lo
        print(f"{name}:")
        print(f"  aabb: x=[{lo[0]:.4f},{hi[0]:.4f}] y=[{lo[1]:.4f},{hi[1]:.4f}] z=[{lo[2]:.4f},{hi[2]:.4f}]")
        print(f"  extents (W x D x H, world axes): {ext[0]*100:.1f} x {ext[1]*100:.1f} x {ext[2]*100:.1f} cm")

    for name in ("table_1", "pen_1", "pencil_holder_1"):
        report(name)

    robot = env.robots[0]
    r_pos, _ = robot.get_position_orientation()
    print(f"\nWidowXAI robot:")
    print(f"  base position: {np.round(to_np(r_pos), 4).tolist()}")
    base_link = robot.links.get("base_link", None)
    if base_link is not None:
        b_lo, b_hi = to_np(base_link.aabb[0]), to_np(base_link.aabb[1])
        b_ext = b_hi - b_lo
        print(f"  base_link aabb: x=[{b_lo[0]:.4f},{b_hi[0]:.4f}] y=[{b_lo[1]:.4f},{b_hi[1]:.4f}] z=[{b_lo[2]:.4f},{b_hi[2]:.4f}]")
        print(f"  base_link footprint: {b_ext[0]*100:.1f} x {b_ext[1]*100:.1f} cm, height {b_ext[2]*100:.1f} cm")

    # Overall robot extent: union of every link's aabb, at the current (reset) pose.
    lo_all, hi_all = None, None
    for link in robot.links.values():
        lo, hi = to_np(link.aabb[0]), to_np(link.aabb[1])
        lo_all = lo if lo_all is None else np.minimum(lo_all, lo)
        hi_all = hi if hi_all is None else np.maximum(hi_all, hi)
    ext_all = hi_all - lo_all
    print(f"  full-robot aabb at current pose: x=[{lo_all[0]:.4f},{hi_all[0]:.4f}] "
          f"y=[{lo_all[1]:.4f},{hi_all[1]:.4f}] z=[{lo_all[2]:.4f},{hi_all[2]:.4f}]")
    print(f"  full-robot extents at current pose: {ext_all[0]*100:.1f} x {ext_all[1]*100:.1f} x {ext_all[2]*100:.1f} cm")

    # eef position relative to base, at the current (reset) pose -- one data point for reach.
    eef_pos = to_np(robot.get_eef_position(robot.default_arm))
    dist = np.linalg.norm(eef_pos - to_np(r_pos))
    print(f"  eef position: {np.round(eef_pos, 4).tolist()}, distance from base: {dist*100:.1f} cm "
          f"(at the reset joint pose -- not the maximum reach)")

    og.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
