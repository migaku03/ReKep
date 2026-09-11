"""Move joints out of a flat `joints` scope and under their parent link, where OmniGibson looks.

    python tools/nest_usd_joints.py <path to robot .usda>

Isaac Sim 4.5's URDF importer collects every joint into one top-level `def Scope "joints"`.
OmniGibson infers the kinematic tree by a different rule: `EntityPrim.update_links` walks each
link prim's *children* for joint prims, records each joint's `physics:body1` as "has a parent",
and expects exactly one link to be left over -- the root. With the joints off in their own scope
that walk finds nothing, every link looks like a root, and loading dies with

    AssertionError: Exactly one single root link should have been found for WidowXAI,
    but found none/multiple instead: [... all 23 links ...]

The robots OmniGibson ships predate this. vx300s.usda nests the `waist` joint inside `base_link`,
which is its `physics:body0` -- so the convention is: a joint lives under its parent link. This
script restores that convention on an already-converted USD.

Reparenting is safe because joints address their bodies by absolute path (`physics:body0` /
`physics:body1`), so where the joint prim itself sits is organisational, not semantic. Joints with
no body0 target are left alone: those tie a body to the world, and have no parent link to move to.

Needs pxr, which is only importable once the simulator has been launched -- hence the og.launch()
below, even though nothing is simulated. Idempotent: a USD with no `joints` scope is left as-is.
"""
import argparse
import sys


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("usd_path", help="robot .usda to rewrite in place")
    p.add_argument("--scope", default="joints", help="name of the flat scope to empty")
    p.add_argument("--dry-run", action="store_true", help="report the moves without writing")
    args = p.parse_args(argv)

    import omnigibson as og

    og.launch()
    from pxr import Sdf, Usd

    stage = Usd.Stage.Open(args.usd_path)
    root = stage.GetDefaultPrim()
    if not root:
        raise SystemExit(f"{args.usd_path} has no default prim")

    scope = next((c for c in root.GetChildren() if c.GetName() == args.scope), None)
    if scope is None:
        print(f"no '{args.scope}' scope under {root.GetPath()} -- nothing to do")
        return 0

    moves, skipped = [], []
    for joint in scope.GetChildren():
        rels = {r.GetName(): r for r in joint.GetRelationships()}
        body0 = rels["physics:body0"].GetTargets() if "physics:body0" in rels else []
        if not body0:
            # World-attached: no parent link exists to nest it under.
            skipped.append(joint.GetName())
            continue
        moves.append((joint.GetPath(), body0[0].AppendChild(joint.GetName())))

    for src, dst in moves:
        print(f"  {src}  ->  {dst}")
    for name in skipped:
        print(f"  {name}: no physics:body0 target, left in place")

    if args.dry_run:
        print(f"\ndry run: {len(moves)} joints would move")
        return 0

    layer = stage.GetRootLayer()
    for src, dst in moves:
        Sdf.CopySpec(layer, src, layer, dst)
    for src, _ in moves:
        parent_spec = layer.GetPrimAtPath(src.GetParentPath())
        del parent_spec.nameChildren[src.name]

    # Drop the scope if emptying it left nothing behind.
    if not skipped:
        scope_spec = layer.GetPrimAtPath(scope.GetPath())
        if scope_spec is not None and not scope_spec.nameChildren:
            del layer.GetPrimAtPath(root.GetPath()).nameChildren[args.scope]
            print(f"removed the now-empty '{args.scope}' scope")

    layer.Save()
    print(f"\nmoved {len(moves)} joints; wrote {args.usd_path}")
    og.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
