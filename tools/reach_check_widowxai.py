"""Can the WidowX AI actually reach the poses the pen task asks for? numpy + URDF only, no sim.

    python tools/reach_check_widowxai.py [--base 0.15 0.0 0.70] [--pen_dir_deg 90]

The pen task needs three poses from one fixed-base 6-DOF arm, and each run that discovers one of
them is out of reach costs 5 minutes of simulation (the Stage 4 attempt-5 stall: joint_2/joint_3
pinned at their limits for the whole run). This checks them up front:

  grasp    jaw centre on the pen's grasp point, approach straight down, finger-separation axis
           perpendicular to the pen (so the jaws close across it, not along it)
  upright  the same grasp rotated so the pen stands vertical: link_6's z (the axis perpendicular to
           both approach and finger separation, i.e. the axis the held pen lies along) vertical,
           approach therefore horizontal, yaw free
  drop     'upright', at the stage-3 target: grasp point 20 cm above the holder opening

Frames (URDF): link_6 x = approach, y = finger separation, z = the third axis. ee_gripper_link is
0.156 m along x from link_6 (the fingertip); the region between the open jaws is ~3 cm behind the
tip (tools/check_grasp_depth.py), so the target point here is link_6 + 0.126 m along x.

IK is damped least squares with random restarts inside the joint limits -- crude, but it only has
to answer "does any in-limit configuration exist", and it reports the best residual when not.
"""
import argparse
import os
import sys
import xml.etree.ElementTree as ET

import numpy as np

URDF = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                    'assets', 'widowxai', 'wxai_follower.urdf')
JOINTS = [f'joint_{i}' for i in range(6)]
JAW_OFFSET = 0.156062 - 0.03


def _vec(el, attr, default):
    if el is None or el.get(attr) is None:
        return np.array(default, float)
    return np.array([float(v) for v in el.get(attr).split()])


def _rot(axis, t):
    a = np.asarray(axis, float) / np.linalg.norm(axis)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(t) * K + (1 - np.cos(t)) * K @ K


def load_chain():
    root = ET.parse(URDF).getroot()
    by = {j.get('name'): j for j in root.findall('joint')}
    chain = []
    for n in JOINTS:
        j = by[n]
        lim = j.find('limit')
        chain.append(dict(xyz=_vec(j.find('origin'), 'xyz', [0, 0, 0]),
                          axis=_vec(j.find('axis'), 'xyz', [0, 0, 1]),
                          lo=float(lim.get('lower')), hi=float(lim.get('upper'))))
    return chain


def fk(chain, q):
    """link_6 pose in the robot base frame (all joint origins in this URDF have rpy = 0)."""
    R, p = np.eye(3), np.zeros(3)
    for j, qi in zip(chain, q):
        p = p + R @ j['xyz']
        R = R @ _rot(j['axis'], qi)
    return p, R


def residual(chain, q, target, mode, pen_dir):
    p, R = fk(chain, q)
    jaw = p + R[:, 0] * JAW_OFFSET
    r = [jaw - target]
    if mode == 'grasp':
        r.append(R[:, 0] - np.array([0, 0, -1.0]))          # approach straight down
        r.append(0.5 * np.array([R[:, 1] @ pen_dir]))        # fingers close across the pen
    else:
        r.append(0.5 * np.cross(R[:, 2], [0, 0, 1.0]))       # held pen vertical, either sign
    return np.concatenate(r)


def solve(chain, target, mode, pen_dir, restarts=60, seed=0):
    rng = np.random.default_rng(seed)
    lo = np.array([j['lo'] for j in chain])
    hi = np.array([j['hi'] for j in chain])
    # among in-tolerance solutions keep the one furthest from every joint limit -- a pose that is
    # only reachable with some joint pinned at its stop is what stalled Stage 4 attempt 5
    best = (np.inf, None)
    best_ok = (-np.inf, None, None)
    for _ in range(restarts):
        q = rng.uniform(lo, hi)
        for _ in range(150):
            r = residual(chain, q, target, mode, pen_dir)
            J = np.zeros((len(r), 6))
            for k in range(6):
                dq = np.zeros(6)
                dq[k] = 1e-5
                J[:, k] = (residual(chain, q + dq, target, mode, pen_dir) - r) / 1e-5
            step = -J.T @ np.linalg.solve(J @ J.T + 1e-3 * np.eye(len(r)), r)
            q = np.clip(q + step, lo, hi)
            if np.linalg.norm(step) < 1e-7:
                break
        err = np.linalg.norm(residual(chain, q, target, mode, pen_dir))
        if err < best[0]:
            best = (err, q.copy())
        if err < 5e-3 and margin(chain, q) > best_ok[0]:
            best_ok = (margin(chain, q), err, q.copy())
    if best_ok[1] is not None:
        return best_ok[1], best_ok[2]
    return best


def margin(chain, q):
    return min(min(qi - j['lo'], j['hi'] - qi) for j, qi in zip(chain, q))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--base', type=float, nargs=3, default=[0.15, 0.0, 0.70])
    ap.add_argument('--base_yaw_deg', type=float, default=180.0)
    ap.add_argument('--grasp', type=float, nargs=3, action='append',
                    help='world grasp point(s) on the pen (repeatable)')
    ap.add_argument('--pen_dir_deg', type=float, default=90.0,
                    help='pen long-axis yaw in world (90 = along world y, the authored layout)')
    ap.add_argument('--holder', type=float, nargs=3, default=[-0.2975, 0.1511, 0.8157],
                    help='holder opening (centre of rim) in world')
    args = ap.parse_args()

    chain = load_chain()
    base = np.array(args.base)
    Rb = _rot([0, 0, 1], np.radians(args.base_yaw_deg))
    to_base = lambda w: Rb.T @ (np.asarray(w) - base)
    pen_dir = Rb.T @ np.array([np.cos(np.radians(args.pen_dir_deg)), np.sin(np.radians(args.pen_dir_deg)), 0])

    cases = []
    for g in (args.grasp or [[-0.2608, -0.13, 0.698]]):
        cases.append(('grasp', g))
        cases.append(('upright', np.array(g) + [0, 0, 0.12]))
    for dz in (0.20, 0.15, 0.10):
        cases.append((f'drop+{int(dz * 100)}cm', np.array(args.holder) + [0, 0, dz]))

    print(f'base {base.tolist()} yaw {args.base_yaw_deg} deg, pen axis yaw {args.pen_dir_deg} deg')
    for name, w in cases:
        mode = 'grasp' if name == 'grasp' else 'upright'
        t = to_base(w)
        err, q = solve(chain, t, mode, pen_dir)
        horiz = np.linalg.norm(t[:2])
        ok = err < 5e-3
        print(f'  {name:<11} world {np.round(w, 3).tolist()}  (base-frame horiz {horiz:.3f} m, '
              f'up {t[2]:+.3f} m)  -> {"REACHABLE" if ok else "NOT reachable"}  residual {err:.4f}'
              f'  limit margin {margin(chain, q):.3f} rad  q={np.round(q, 2).tolist()}')


if __name__ == '__main__':
    sys.exit(main())
