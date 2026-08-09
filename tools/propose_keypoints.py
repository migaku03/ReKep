"""Phase 1: run only the keypoint proposal, save what a VLM query needs, and exit. No API call.

main.py fuses three things into one process -- keypoint proposal, the paid VLM query, and
execution -- so anything that breaks downstream of the query wastes the call. This is the free
half. It boots the sim, runs KeypointProposer.get_keypoints(), and writes the annotated image
plus the keypoint array to disk; tools/query_vlm.py picks those up without needing the sim at all.

The split also matters because get_keypoints() has never actually run in this port.
--use_cached_query short-circuits it at main.py:66, so DINOv2, kmeans_pytorch, MeanShift and the
cv2 overlay are all untested against the local numpy/torch, which is exactly the class of drift
that produced the earlier torch-vs-numpy breakages. Exercising them costs nothing here.

Construction order mirrors Main.__init__ (seeds -> KeypointProposer -> env) so the RNG state
entering get_keypoints() is the one main.py would have produced. The IK/subgoal/path solvers
main.py also builds are skipped; they do not draw from numpy's global RNG.
"""
import argparse
import datetime
import json
import os
import sys

REKEP_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REKEP_DIR)
os.chdir(REKEP_DIR)  # the config paths inside the repo are all relative to it

import cv2
import numpy as np
import torch

import environment
from environment import ReKepOGEnv
from keypoint_proposal import KeypointProposer
from utils import get_config


def describe_keypoints(env, keypoints, bounds_min, bounds_max):
    """Report, per keypoint, which object it belongs to and how far it sits off that mesh.

    register_keypoints() snaps each keypoint onto the nearest visual mesh and does it *in place*
    on the array it is handed, so pass it a copy -- the raw proposal is what has to go into
    metadata.json, since main.py:92 re-registers from there at execution time. The snap distance
    is the useful number: a keypoint several centimetres off every mesh means the depth
    reprojection is wrong, not that the VLM will be.
    """
    rows = []
    snapped = np.asarray(keypoints).copy()
    try:
        env.register_keypoints(snapped)
    except Exception as e:
        print(f"[phase1] keypoint->object mapping failed: {type(e).__name__}: {e}", flush=True)
        return rows
    for idx, kp in enumerate(keypoints):
        obj = env.get_object_by_keypoint(idx)
        inside = bool(np.all(kp >= bounds_min) and np.all(kp <= bounds_max))
        rows.append(dict(idx=idx,
                         world=np.round(kp, 4).tolist(),
                         object=obj.name,
                         snap_cm=round(float(np.linalg.norm(snapped[idx] - kp)) * 100, 2),
                         in_bounds=inside))
    return rows


def write_comparison(projected_bgr, out_dir):
    """Stack the fresh proposal against the author's 2024 query image for eyeball comparison."""
    upstream_path = os.path.join(REKEP_DIR, 'vlm_query', 'pen', 'query_img.png')
    if not os.path.exists(upstream_path):
        return None
    ref = cv2.imread(upstream_path)
    if ref is None:
        return None
    h = max(ref.shape[0], projected_bgr.shape[0])
    fit = lambda im: cv2.resize(im, (int(im.shape[1] * h / im.shape[0]), h))
    path = os.path.join(out_dir, 'compare_with_upstream.png')
    cv2.imwrite(path, np.hstack([fit(ref), fit(projected_bgr)]))
    return path


def main(args):
    cfg = get_config(config_path="./configs/config.yaml")
    seed = cfg['main']['seed']
    np.random.seed(seed)
    torch.manual_seed(seed)
    torch.cuda.manual_seed(seed)

    kp_cfg = cfg['keypoint_proposer']
    # recorded in keypoints.json as well: if the appearance override turns out to cost keypoints,
    # the next question is whether these two recover them, and that comparison needs the baseline
    print(f"[phase1] num_candidates_per_mask {kp_cfg['num_candidates_per_mask']} | "
          f"min_dist_bt_keypoints {kp_cfg['min_dist_bt_keypoints']} | "
          f"max_mask_ratio {kp_cfg['max_mask_ratio']}", flush=True)

    overrides = dict(environment.APPEARANCE_OVERRIDES)
    if args.no_appearance_override:
        # tint and reflection blanking both flatten the surface the proposer clusters on, so this
        # is the A/B that says whether the black holder costs keypoints
        environment.APPEARANCE_OVERRIDES.clear()
        overrides = {}
        print("[phase1] APPEARANCE_OVERRIDES disabled for this run", flush=True)

    proposer = KeypointProposer(kp_cfg)
    env = ReKepOGEnv(cfg['env'], args.scene_file, verbose=False)

    env.reset()
    cam_obs = env.get_cam_obs()
    cam_id = cfg['main']['vlm_camera']
    rgb = cam_obs[cam_id]['rgb']
    points = cam_obs[cam_id]['points']
    mask = cam_obs[cam_id]['seg']
    print(f"[phase1] rgb {rgb.shape} {rgb.dtype} | points {points.shape} | "
          f"seg {mask.shape} with {len(np.unique(mask))} distinct ids", flush=True)

    keypoints, projected = proposer.get_keypoints(rgb, points, mask)
    keypoints = np.asarray(keypoints)
    print(f"[phase1] {len(keypoints)} keypoints proposed", flush=True)

    bounds_min = np.array(cfg['main']['bounds_min'])
    bounds_max = np.array(cfg['main']['bounds_max'])
    rows = describe_keypoints(env, keypoints, bounds_min, bounds_max)

    out_dir = args.out or os.path.join(
        REKEP_DIR, 'keypoint_proposals', datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S'))
    os.makedirs(out_dir, exist_ok=True)
    # projected comes out of KeypointProposer as RGB; cv2 writes BGR
    projected_bgr = np.asarray(projected)[..., ::-1]
    cv2.imwrite(os.path.join(out_dir, 'projected_img.png'), projected_bgr)
    with open(os.path.join(out_dir, 'keypoints.json'), 'w', encoding='utf-8') as f:
        json.dump({'init_keypoint_positions': keypoints.tolist(),
                   'num_keypoints': int(len(keypoints)),
                   'scene_file': args.scene_file,
                   'keypoint_proposer': {k: kp_cfg[k] for k in
                                         ('num_candidates_per_mask', 'min_dist_bt_keypoints',
                                          'max_mask_ratio')},
                   'appearance_overrides': overrides,
                   'keypoint_details': rows}, f, indent=2)
    comparison = write_comparison(projected_bgr, out_dir)

    print(f"\n[phase1] workspace bounds {bounds_min.tolist()} .. {bounds_max.tolist()}", flush=True)
    print(f"[phase1] {'idx':>3}  {'world xyz':<28} {'object':<24} {'snap':>7}  bounds", flush=True)
    for r in rows:
        flag = '' if r['in_bounds'] else '  <-- OUTSIDE'
        print(f"[phase1] {r['idx']:>3}  {str(r['world']):<28} {r['object']:<24} "
              f"{r['snap_cm']:>5.2f}cm  {'ok' if r['in_bounds'] else 'no'}{flag}", flush=True)

    print(f"\n[phase1] wrote {out_dir}", flush=True)
    print(f"[phase1]   projected_img.png   <- this is the image the VLM will see", flush=True)
    print(f"[phase1]   keypoints.json      <- init_keypoint_positions for metadata.json", flush=True)
    if comparison:
        print(f"[phase1]   compare_with_upstream.png", flush=True)
    print(f"\n[phase1] Phase 2: open compare_with_upstream.png and check that the pen body and "
          f"the holder rim both carry a keypoint before spending anything.", flush=True)

    # Everything is on disk and flushed by now. og.shutdown() regularly fails to bring the Kit
    # process all the way down on Windows -- the run reaches "Simulation App Shutting Down" and
    # then the process just sits there, holding whatever pipe is reading it. Nothing here needs a
    # graceful teardown, so leave immediately rather than hang.
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(0)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--scene_file', default='./configs/og_scene_file_pen.json')
    parser.add_argument('--out', default=None,
                        help='output directory (default: ./keypoint_proposals/<timestamp>)')
    parser.add_argument('--no-appearance-override', dest='no_appearance_override',
                        action='store_true',
                        help='clear environment.APPEARANCE_OVERRIDES before building the scene, '
                             'to measure what the black holder costs in keypoints')
    args = parser.parse_args()
    try:
        main(args)
    except Exception:
        import traceback
        traceback.print_exc()
        sys.stdout.flush()
        raise
