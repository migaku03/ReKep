"""Where along the WidowX AI fingers is there actually collision geometry to squeeze with?

    python -u tools/measure_finger_geometry.py

Written after the rescaled-pen grasp kept closing the jaws to 0.000 straight past a 0.97 cm pen
(environment.py [CLOSE TRACE]): the pen did not move out of the way, only one finger ever touched
it, and it sits in the last ~1 cm before the fingertips. That points at the fingers' collision
shapes not reaching the tip. Measure it: sample each finger's collision meshes, express the points
in the eef frame (z out of the fingertips, y = finger separation), and for each band of depth
behind the tip print how close to the jaw centreline (|y| min) the finger's collider comes and
how wide it is along x. Also prints the visual mesh for comparison.
"""
import os
import sys

import numpy as np

sys.path.insert(0, ".")


def main():
    import omnigibson as og
    from omnigibson.macros import gm
    from omnigibson.utils.usd_utils import PoseAPI, mesh_prim_mesh_to_trimesh_mesh, mesh_prim_shape_to_trimesh_mesh
    import transform_utils as T

    gm.USE_GPU_DYNAMICS = True
    gm.ENABLE_FLATCACHE = False
    from robots.widowxai import WidowXAI  # noqa: F401

    cfg = {"scene": {"type": "Scene"},
           "robots": [{"type": "WidowXAI", "name": "WidowXAI", "obs_modalities": ["rgb"],
                       "action_normalize": False, "grasping_mode": "assisted",
                       "position": [0.0, 0.0, 0.0]}]}
    env = og.Environment(configs=cfg)
    robot = env.robots[0]
    for _ in range(10):
        og.sim.step()
    arm = robot.default_arm
    T_eef = T.pose2mat((T.to_numpy(robot.get_eef_position(arm)), T.to_numpy(robot.get_eef_orientation(arm))))
    inv = T.pose_inv(T_eef)

    def pts_of(meshes):
        out = []
        for m in meshes.values():
            tm = (mesh_prim_mesh_to_trimesh_mesh(m.prim) if m.prim.GetPrimTypeInfo().GetTypeName() == 'Mesh'
                  else mesh_prim_shape_to_trimesh_mesh(m.prim))
            tm.apply_transform(PoseAPI.get_world_pose_with_scale(m.prim_path))
            out.append(np.asarray(tm.sample(20000)))
            out.append(np.asarray(tm.vertices))
        p = np.concatenate(out)
        return (inv[:3, :3] @ p.T).T + inv[:3, 3]

    finger_q = float(T.to_numpy(robot.get_joint_positions())[list(robot.joints.keys()).index(robot.finger_joint_names[arm][0])])
    print(f"finger joint = {finger_q:.4f} (0.044 = fully open)", flush=True)
    for link in robot.finger_links[arm]:
        for kind, meshes in (("collision", link.collision_meshes), ("visual", link.visual_meshes)):
            if not meshes:
                print(f"{link.body_name} {kind}: NO MESHES", flush=True)
                continue
            p = pts_of(meshes)
            print(f"--- {link.body_name} {kind}: {len(meshes)} mesh(es), eef-frame z range "
                  f"[{p[:, 2].min():.4f}, {p[:, 2].max():.4f}]", flush=True)
            for z_hi in np.arange(0.002, -0.030, -0.004):
                band = p[(p[:, 2] <= z_hi) & (p[:, 2] > z_hi - 0.004)]
                if len(band) == 0:
                    print(f"   z in ({z_hi - 0.004:+.3f},{z_hi:+.3f}]: nothing", flush=True)
                    continue
                print(f"   z in ({z_hi - 0.004:+.3f},{z_hi:+.3f}]: |y| min {np.abs(band[:, 1]).min():.4f}  "
                      f"y [{band[:, 1].min():+.4f},{band[:, 1].max():+.4f}]  x [{band[:, 0].min():+.4f},{band[:, 0].max():+.4f}]",
                      flush=True)
    sys.stdout.flush()
    os._exit(0)


if __name__ == "__main__":
    main()
