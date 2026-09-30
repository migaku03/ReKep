"""Stage 1 of the object-rescale plan: report each task object's native (scale=1) bbox, its
current scale, and the resulting world extents, so target scale values are derived rather than
eyeballed -- the same source (`ig:nativeBB`, i.e. DatasetObject.native_bbox) commit 81b41ea used to
recompute the pen's own scale.

    python tools/measure_native_scale.py --config ./configs/config_widowxai.yaml

Also reports each object's orientation (quaternion) so axis-alignment between native-bbox local
axes and world axes can be confirmed rather than assumed -- table_1's scale is currently a bare
scalar, and turning it into an anisotropic [sx,sy,sz] requires knowing which local axis maps to
which world dimension.
"""
import argparse
import sys

import numpy as np

sys.path.insert(0, ".")


def main():
    import omnigibson as og

    p = argparse.ArgumentParser()
    p.add_argument("--config", default="./configs/config_widowxai.yaml")
    p.add_argument("--scene_file", default="./configs/og_scene_file_pen.json")
    args = p.parse_args()

    from environment import ReKepOGEnv
    from robots.widowxai import WidowXAI  # noqa: F401
    from utils import get_config

    global_config = get_config(config_path=args.config)
    rekep_env = ReKepOGEnv(global_config['env'], args.scene_file, verbose=False)
    env = rekep_env.og_env

    def to_np(x):
        return np.asarray(x.cpu() if hasattr(x, "cpu") else x, dtype=float)

    import traceback

    for name in ("table_1", "pen_1", "pencil_holder_1"):
        try:
            obj = env.scene.object_registry("name", name)
            if obj is None:
                print(f"{name}: NOT FOUND", flush=True)
                continue
            scale = to_np(obj.scale)
            native = to_np(obj.native_bbox)
            lo, hi = obj.aabb
            lo, hi = to_np(lo), to_np(hi)
            world_ext = hi - lo
            pos, ori = obj.get_position_orientation()
            pos, ori = to_np(pos), to_np(ori)
            print(f"{name}:", flush=True)
            print(f"  scale (current):        {np.round(scale, 4).tolist()}", flush=True)
            print(f"  native_bbox (scale=1):  {np.round(native * 100, 2).tolist()} cm  (local frame)", flush=True)
            print(f"  native_bbox * scale:    {np.round(native * scale * 100, 2).tolist()} cm  (local frame, sanity check)", flush=True)
            print(f"  world aabb extents:     {np.round(world_ext * 100, 2).tolist()} cm  (world frame, current pose)", flush=True)
            print(f"  orientation (x,y,z,w):  {np.round(ori, 4).tolist()}", flush=True)
            print(f"  position:               {np.round(pos, 4).tolist()}", flush=True)
            print(flush=True)
        except Exception:
            print(f"{name}: EXCEPTION", flush=True)
            traceback.print_exc()
            sys.stdout.flush()
            sys.stderr.flush()

    og.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
