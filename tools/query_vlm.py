"""Phase 3: spend exactly one VLM query on the keypoints tools/propose_keypoints.py produced.

Imports no OmniGibson on purpose. The sim takes minutes to boot and contributes nothing to the
query, so a bad key, a wrong model name or a parse failure should not cost that wait. What comes
out is a directory in the same shape main.py expects, so Phase 4 is just pointing
main.py:373's rekep_program_dir at it and running with --use_cached_query.

Two ways to avoid paying twice:

  --model gpt-4o-mini   the Phase 2.5 rehearsal. Exercises auth, image encoding, streaming and
                        both parsers for a fraction of the cost. It raises the odds that the real
                        call parses; it does not prove it, since formatting habits differ across
                        models even inside one family.

  --reparse DIR         rebuild the constraint files from a response already on disk.
                        ConstraintGenerator writes output_raw.txt before it parses
                        (constraint_generation.py:151), so a parse failure never costs the answer.
"""
import argparse
import json
import os
import sys

REKEP_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REKEP_DIR)
os.chdir(REKEP_DIR)

import cv2
import numpy as np

from constraint_generation import ConstraintGenerator
from utils import get_config

# kept verbatim in step with main.py:370 -- the constraints are written against this wording,
# and "the black pen holder" is the reason the appearance override in environment.py exists
INSTRUCTION = 'reorient the white pen and drop it upright into the black pen holder'


def load_proposal(proposal_dir):
    img_path = os.path.join(proposal_dir, 'projected_img.png')
    kp_path = os.path.join(proposal_dir, 'keypoints.json')
    for p in (img_path, kp_path):
        if not os.path.exists(p):
            raise FileNotFoundError(f"{p} not found -- run tools/propose_keypoints.py first")
    bgr = cv2.imread(img_path)
    if bgr is None:
        raise ValueError(f"could not decode {img_path}")
    with open(kp_path, encoding='utf-8') as f:
        saved = json.load(f)
    metadata = {'init_keypoint_positions': saved['init_keypoint_positions'],
                'num_keypoints': saved['num_keypoints']}
    # generate() flips back to BGR itself before writing, so hand it RGB as main.py does
    return np.ascontiguousarray(bgr[..., ::-1]), metadata


def summarise(task_dir):
    """Report whether the result is actually loadable by main.py, rather than just non-empty."""
    meta_path = os.path.join(task_dir, 'metadata.json')
    if not os.path.exists(meta_path):
        print(f"[phase3] no metadata.json in {task_dir}", flush=True)
        return
    with open(meta_path, encoding='utf-8') as f:
        meta = json.load(f)
    n = meta.get('num_stages')
    print(f"\n[phase3] num_stages       {n}", flush=True)
    print(f"[phase3] grasp_keypoints  {meta.get('grasp_keypoints')}", flush=True)
    print(f"[phase3] release_keypoints {meta.get('release_keypoints')}", flush=True)
    print(f"[phase3] num_keypoints    {meta.get('num_keypoints')}", flush=True)
    # main.py:98 loads stage{N}_subgoal_constraints.txt for every stage; a missing subgoal file
    # is silently treated as "no constraints" there, which is far worse than an error
    missing = [f'stage{s}_subgoal_constraints.txt' for s in range(1, (n or 0) + 1)
               if not os.path.exists(os.path.join(task_dir, f'stage{s}_subgoal_constraints.txt'))]
    if missing:
        print(f"[phase3] WARNING missing subgoal constraints: {missing}", flush=True)
        print(f"[phase3] main.py treats a missing file as 'no constraints' without complaining. "
              f"Fix output_raw.txt and re-run with --reparse {task_dir}", flush=True)
    else:
        print(f"[phase3] all {n} stages have subgoal constraints", flush=True)
    bad = [k for k in (meta.get('grasp_keypoints') or []) + (meta.get('release_keypoints') or [])
           if k != -1 and not (0 <= k < meta.get('num_keypoints', 0))]
    if bad:
        print(f"[phase3] WARNING keypoint indices out of range: {bad}", flush=True)


def main(args):
    cfg = get_config(config_path="./configs/config.yaml")['constraint_generator']
    if args.model:
        cfg['model'] = args.model
    if args.max_tokens:
        cfg['max_tokens'] = args.max_tokens

    img, metadata = load_proposal(args.proposal)

    if args.reparse:
        # ConstraintGenerator.__init__ reads OPENAI_API_KEY unconditionally, even though nothing
        # here will reach the network
        os.environ.setdefault('OPENAI_API_KEY', 'unused-for-reparse')
        gen = ConstraintGenerator(cfg)
        task_dir = os.path.abspath(args.reparse)
        with open(os.path.join(task_dir, 'output_raw.txt'), encoding='utf-8') as f:
            output = f.read()
        gen.task_dir = task_dir
        gen._parse_and_save_constraints(output, task_dir)
        metadata.update(gen._parse_other_metadata(output))
        gen._save_metadata(metadata)
        print(f"[phase3] reparsed {task_dir} without calling the API", flush=True)
        summarise(task_dir)
        return

    if args.key_file:
        # a file lets the key reach the process without being typed into a shell that logs its
        # history, and without depending on an env var that an already-running parent cannot see.
        # Keep the file outside this repository.
        with open(os.path.expanduser(args.key_file), encoding='utf-8') as f:
            os.environ['OPENAI_API_KEY'] = f.read().strip()
    if not os.environ.get('OPENAI_API_KEY'):
        sys.exit("OPENAI_API_KEY is not set (pass --key-file, or export it)")

    print(f"[phase3] model       {cfg['model']}", flush=True)
    print(f"[phase3] max_tokens  {cfg['max_tokens']}   temperature {cfg['temperature']}", flush=True)
    print(f"[phase3] image       {args.proposal}/projected_img.png {img.shape}", flush=True)
    print(f"[phase3] keypoints   {metadata['num_keypoints']}", flush=True)
    print(f"[phase3] instruction {INSTRUCTION!r}", flush=True)
    if not args.yes:
        if input("[phase3] send this billable request? [y/N] ").strip().lower() not in ('y', 'yes'):
            sys.exit("aborted")

    task_dir = gen_and_report(cfg, img, metadata)
    print(f"\n[phase3] Phase 4: point rekep_program_dir at this directory in main.py:373, then\n"
          f"[phase3]   python main.py --task pen --use_cached_query", flush=True)
    print(f"[phase3]   {os.path.relpath(task_dir, REKEP_DIR)}", flush=True)


def gen_and_report(cfg, img, metadata):
    gen = ConstraintGenerator(cfg)
    task_dir = gen.generate(img, INSTRUCTION, metadata)
    summarise(task_dir)
    return task_dir


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('proposal', help='directory written by tools/propose_keypoints.py')
    parser.add_argument('--model', default=None,
                        help="override configs/config.yaml for this call only "
                             "(e.g. gpt-4o-mini for the cheap rehearsal)")
    parser.add_argument('--max-tokens', dest='max_tokens', type=int, default=None)
    parser.add_argument('--key-file', dest='key_file', default=None, metavar='PATH',
                        help='read OPENAI_API_KEY from this file instead of the environment; '
                             'keep it outside the repository')
    parser.add_argument('--reparse', default=None, metavar='DIR',
                        help='re-parse DIR/output_raw.txt instead of calling the API')
    parser.add_argument('-y', '--yes', action='store_true', help='skip the confirmation prompt')
    args = parser.parse_args()
    try:
        main(args)
    except Exception:
        import traceback
        traceback.print_exc()
        sys.stdout.flush()
        raise
