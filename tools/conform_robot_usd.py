"""Make a freshly-imported robot USD match what OmniGibson expects of a robot.

    python tools/conform_robot_usd.py <path to robot .usda>

Isaac Sim 4.5's URDF importer and OmniGibson v3.7.2's robot loader disagree about the shape of a
robot stage in three ways. None is visible while you only use the robots OmniGibson ships, because
those USDs were produced by an older importer that still matched. Import anything new through the
documented path and all three bite, in this order:

1. **Joints are collected into one flat `def Scope "joints"`.** OmniGibson infers the kinematic
   tree by walking each *link* prim's children for joints, recording each joint's `physics:body1`
   as "this link has a parent", and expecting exactly one link to be left over -- the root. With
   the joints off in their own scope that walk finds nothing and every link looks like a root:

       AssertionError: Exactly one single root link should have been found for WidowXAI,
       but found none/multiple instead: [... all 23 links ...]

   vx300s.usda nests its `waist` joint inside `base_link`, which is that joint's own
   `physics:body0`. So the convention is: a joint lives under its parent link.

2. **Moving a joint under its link leaves other relationships pointing at its old address.**
   `physics:body0`/`body1` are absolute-path targets, already resolved, so the move above keeps
   those working. But a joint can also be *named* by something else, and that reference does not
   get updated by the move. WidowX AI's gripper has exactly this: `right_carriage_joint` carries
   `<mimic joint="left_carriage_joint"/>` in the URDF, which the importer turns into
   `PhysxMimicJointAPI:rotY:referenceJoint` pointing at `left_carriage_joint`'s path at import
   time -- i.e. inside the flat scope. After step 1 moves the referenced joint out from under it,
   the relationship still says `.../joints/left_carriage_joint`, which no longer exists:

       [Error] [omni.physx.plugin] Usd Physics: PhysxMimicJointAPI at
       .../right_carriage_joint points to a non existing prim at .../joints/left_carriage_joint
           in attribute "referenceJoint".

   Fixed generically rather than special-cased to mimic joints: any relationship target that
   does not resolve to a live prim is looked up by name across the rest of the stage and
   repointed there, since a reparent changes a prim's path but not its name.

3. **The robot's root prim has no transform ops.** `XFormPrim._post_load` reads
   `xformOp:scale` unconditionally:

       RuntimeError: Could not infer dtype of NoneType

   vx300s's root carries translate/orient/scale plus an `xformOpOrder`. The fix reuses
   OmniGibson's own `_add_xform_properties`, which is what the importer applies to the prims it
   does touch.

Reparenting joints is safe because they address their bodies by absolute path
(`physics:body0` / `physics:body1`), so where the joint prim sits is organisational rather than
semantic. Joints with no `body0` target tie a body to the world and are left alone.

Needs pxr, which is only on the path once the simulator has been launched -- hence og.launch() in
a script that simulates nothing. Idempotent: re-running on a conformed USD reports no work.
"""
import argparse
import sys


def _nest_joints(stage, root, scope_name, dry_run):
    """Move joints out of the flat scope and under their physics:body0 link."""
    scope = next((c for c in root.GetChildren() if c.GetName() == scope_name), None)
    if scope is None:
        print(f"  joints: no '{scope_name}' scope -- already conformed")
        return 0

    from pxr import Sdf

    moves, skipped = [], []
    for joint in scope.GetChildren():
        rels = {r.GetName(): r for r in joint.GetRelationships()}
        body0 = rels["physics:body0"].GetTargets() if "physics:body0" in rels else []
        if not body0:
            skipped.append(joint.GetName())
            continue
        moves.append((joint.GetPath(), body0[0].AppendChild(joint.GetName())))

    for src, dst in moves:
        print(f"    {src.name}  ->  {dst.GetParentPath().name}")
    for name in skipped:
        print(f"    {name}: no physics:body0 target, left in place")
    if dry_run:
        return len(moves)

    layer = stage.GetRootLayer()
    for src, dst in moves:
        Sdf.CopySpec(layer, src, layer, dst)
    for src, _ in moves:
        del layer.GetPrimAtPath(src.GetParentPath()).nameChildren[src.name]

    if not skipped:
        scope_spec = layer.GetPrimAtPath(scope.GetPath())
        if scope_spec is not None and not scope_spec.nameChildren:
            del layer.GetPrimAtPath(root.GetPath()).nameChildren[scope_name]
            print(f"  joints: removed the now-empty '{scope_name}' scope")
    return len(moves)


def _fix_dangling_relationship_targets(stage, layer, dry_run):
    """Repoint any relationship whose target no longer exists, by matching on prim name.

    Reparenting in _nest_joints changes a joint's path but not its name, and any relationship
    naming it by absolute path (PhysxMimicJointAPI's referenceJoint being the case in point) goes
    stale. Safe as a name-based lookup because joint names are unique in a robot URDF -- if two
    prims in the stage ever shared a name this refuses to guess and reports it instead.
    """
    from pxr import Sdf

    by_name = {}
    for prim in stage.Traverse():
        by_name.setdefault(prim.GetName(), []).append(prim.GetPath())

    fixed = 0

    def visit(path):
        nonlocal fixed
        if not path.IsPropertyPath():
            return
        spec = layer.GetPropertyAtPath(path)
        if not isinstance(spec, Sdf.RelationshipSpec):
            return
        targets = list(spec.targetPathList.GetAddedOrExplicitItems())
        if not targets:
            return
        new_targets, changed_here = [], False
        for t in targets:
            if stage.GetPrimAtPath(t):
                new_targets.append(t)
                continue
            candidates = by_name.get(t.name, [])
            if len(candidates) == 1:
                print(f"    {path}: {t}  ->  {candidates[0]}")
                new_targets.append(candidates[0])
                changed_here = True
            else:
                print(f"    {path}: {t} does not resolve and name '{t.name}' is not "
                      f"unique ({len(candidates)} candidates) -- left as is")
                new_targets.append(t)
        if changed_here:
            fixed += 1
            if not dry_run:
                spec.targetPathList.ClearEditsAndMakeExplicit()
                for t in new_targets:
                    spec.targetPathList.explicitItems.append(t)

    layer.Traverse(Sdf.Path.absoluteRootPath, visit)
    if fixed == 0:
        print("  relationships: none dangling -- already conformed")
    return fixed


def _add_root_xform_ops(root, dry_run):
    """Give the robot's root prim the transform ops XFormPrim._post_load assumes are there."""
    if root.GetAttribute("xformOp:scale").IsValid():
        print("  root xform: already has xformOp:scale -- already conformed")
        return 0
    print(f"  root xform: adding translate/orient/scale to {root.GetPath()}")
    if dry_run:
        return 1
    # OmniGibson's own helper, so the result matches what its importer produces elsewhere:
    # it also strips the rotate*/transform ops that would conflict, and sets xformOpOrder.
    from omnigibson.utils.asset_conversion_utils import _add_xform_properties

    _add_xform_properties(root)
    return 1


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("usd_path", help="robot .usda to rewrite in place")
    p.add_argument("--scope", default="joints", help="name of the flat joint scope to empty")
    p.add_argument("--dry-run", action="store_true", help="report what would change without writing")
    args = p.parse_args(argv)

    import omnigibson as og

    og.launch()
    from pxr import Usd

    stage = Usd.Stage.Open(args.usd_path)
    root = stage.GetDefaultPrim()
    if not root:
        raise SystemExit(f"{args.usd_path} has no default prim")
    print(f"conforming {root.GetPath()} in {args.usd_path}")

    changed = _nest_joints(stage, root, args.scope, args.dry_run)
    changed += _fix_dangling_relationship_targets(stage, stage.GetRootLayer(), args.dry_run)
    changed += _add_root_xform_ops(root, args.dry_run)

    if args.dry_run:
        print(f"\ndry run: {changed} change(s) pending")
        return 0
    if changed:
        stage.GetRootLayer().Save()
        print(f"\nwrote {args.usd_path}")
    else:
        print("\nnothing to do")
    og.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
