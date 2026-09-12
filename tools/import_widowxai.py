"""Run OmniGibson's custom-robot importer, working around two bugs in the pinned version.

OmniGibson v3.7.2 ships `omnigibson/examples/robots/import_custom_robot.py`, which is the
supported way to turn a URDF into an OmniGibson-compatible USD. In this version the path does not
run at all. Two links in the chain disagree about the same argument, and both have to be bridged:

  1. The example script calls `import_og_asset_from_urdf()` without `dataset_root`, which that
     function requires:

        TypeError: import_og_asset_from_urdf() missing 1 required positional argument: 'dataset_root'

  2. Fix that and it fails one level deeper. `import_og_asset_from_urdf` forwards `dataset_root`
     to `convert_urdf_to_usd`, which never took it -- it takes `dataset_name`, a bare name it
     resolves itself via `get_dataset_path()`:

        TypeError: convert_urdf_to_usd() got an unexpected keyword argument 'dataset_root'

This looks like a half-finished rename of `dataset_name` to `dataset_root`: the callers were
updated, the callee was not. Since `get_dataset_path(name)` is just `DATA_PATH / name`, the
bridge is to pass the basename -- there is no information loss, only a mismatched convention.

The fixes are two keyword arguments, but they belong to files inside the BEHAVIOR-1K checkout,
and the whole point of the WidowX AI work is that the existing Fetch stack keeps working
untouched -- see docs/sim_platform_decision.md. So this driver substitutes both functions inside
the namespaces that call them and then invokes the original entry point. Nothing under
BEHAVIOR-1K is modified.

    cd external/ReKep
    python tools/import_widowxai.py --config assets/widowxai/widowxai_source_config.yaml --install

Output lands in `<gm.DATA_PATH>/custom_dataset/objects/robot/<name>/`. That is *not* where
OmniGibson looks for robots at runtime -- `robot_base.py` resolves
`<gm.DATA_PATH>/omnigibson-robot-assets/models/<lowercased class name>/usd/<same>.usda` -- so the
output still has to be moved. `--install` does that move.

This whole file becomes unnecessary the moment the upstream signatures agree; check before
carrying it forward to a newer BEHAVIOR-1K.
"""
import argparse
import functools
import os
import shutil
import sys


def _patch_single_xform_prim():
    """Point `XFormPrim(prim_path=...)` at the class that still takes a single prim_path.

    add_sensor() constructs `lazy.isaacsim.core.prims.xform_prim.XFormPrim(prim_path=...)` twice.
    In Isaac Sim 4.5 that name is the *batched* view class and takes `prim_paths_expr`; the
    single-prim class it used to be was split out as `SingleXFormPrim`. So:

        TypeError: XFormPrim.__init__() got an unexpected keyword argument 'prim_path'

    Only calls that pass `prim_path` are redirected -- anything using the batched signature still
    reaches the real class. Has to be applied after og.launch(), since isaacsim cannot be
    imported before the simulator boots, which is why this is not done up front with the others.
    """
    # Reached by attribute, not by import. `isaacsim.core.prims` does `from .impl import *`, which
    # copies the *name* xform_prim into the package namespace without making
    # `isaacsim.core.prims.xform_prim` an importable module path -- so `import` raises
    # ModuleNotFoundError on the very path the example script successfully attribute-walks
    # through `lazy`. Walk it the same way it does.
    import isaacsim.core.prims as prims

    SingleXFormPrim = prims.SingleXFormPrim
    module = prims.xform_prim

    original = module.XFormPrim

    class _Shim:
        """Callable stand-in that redirects prim_path= calls and proxies everything else.

        It cannot simply be a function. `SingleXFormPrim.__init__` delegates to
        `XFormPrim.__init__`, which reaches back through its own module global to call
        `XFormPrim.set_local_poses(...)` -- so replacing the name with a function breaks the
        real class from the inside:

            AttributeError: 'function' object has no attribute 'set_local_poses'

        Forwarding attribute lookups to the original class keeps those internal class-level
        references working while still intercepting construction.
        """

        def __call__(self, *args, **kwargs):
            if "prim_path" in kwargs:
                return SingleXFormPrim(*args, **kwargs)
            return original(*args, **kwargs)

        def __getattr__(self, name):
            return getattr(original, name)

    module.XFormPrim = _Shim()


def _patched_importer(dataset_root):
    """`import_og_asset_from_urdf` with @dataset_root bound, patching Isaac's API on the way out.

    The eef/camera link creation that runs after this returns needs the XFormPrim fix, and that
    fix needs a booted simulator -- which this call is what provides.
    """
    from omnigibson.utils.asset_conversion_utils import import_og_asset_from_urdf

    def run(*args, **kwargs):
        result = import_og_asset_from_urdf(*args, dataset_root=dataset_root, **kwargs)
        _patch_single_xform_prim()
        return result

    return run


def _patch_convert_urdf_to_usd():
    """Teach `convert_urdf_to_usd` to accept the `dataset_root` its caller insists on passing.

    It wants `dataset_name` and derives the root itself with `get_dataset_path(name)`, which is
    `DATA_PATH / name`. So a root maps back to a name by taking its last component, and the two
    spellings carry identical information. Patched in the module that calls it, since
    `import_og_asset_from_urdf` resolves it as a module global.
    """
    from omnigibson.utils import asset_conversion_utils as acu

    original = acu.convert_urdf_to_usd

    @functools.wraps(original)
    def shim(*args, dataset_root=None, **kwargs):
        if dataset_root is not None:
            kwargs.setdefault("dataset_name", os.path.basename(os.path.normpath(dataset_root)))
        return original(*args, **kwargs)

    acu.convert_urdf_to_usd = shim


def install(name, data_path, urdf_src=None):
    """Move the importer's output into the location robot_base.py actually reads.

    Also places a URDF, which the importer does not. `robot_base.py` exposes
    `urdf_path` as `models/<name>/urdf/<name>.urdf`, and ReKep hands exactly that to lula when it
    builds the IK solver for its reachability cost -- so without it the robot loads fine and then
    main.py dies constructing the solver. Every shipped robot has this directory; vx300s keeps
    its meshes under `urdf/meshes/`, and the source URDF's mesh references are already relative
    in that shape, so copying the pair across preserves them.

    The URDF copied is the source one, not the collision-decomposed variant the conversion
    produced. lula reads it only for the kinematic chain.
    """
    src = os.path.join(data_path, "custom_dataset", "objects", "robot", name)
    dst = os.path.join(data_path, "omnigibson-robot-assets", "models", name)
    if not os.path.isdir(src):
        raise SystemExit(f"nothing to install: {src} does not exist")
    if os.path.isdir(dst):
        # Refuse rather than clobber. Re-importing is cheap; losing a hand-tuned USD is not.
        raise SystemExit(f"refusing to overwrite existing {dst} -- remove it first if that is what you want")
    print(f"installing {src}\n        -> {dst}")
    shutil.copytree(src, dst)

    if urdf_src is not None:
        urdf_dir = os.path.join(dst, "urdf")
        os.makedirs(urdf_dir, exist_ok=True)
        shutil.copy(urdf_src, os.path.join(urdf_dir, f"{name}.urdf"))
        meshes_src = os.path.join(os.path.dirname(urdf_src), "meshes")
        if os.path.isdir(meshes_src):
            shutil.copytree(meshes_src, os.path.join(urdf_dir, "meshes"))
        print(f"  placed urdf/{name}.urdf (+ meshes) for the IK solver")

    for entry in sorted(os.listdir(dst)):
        print(f"  {entry}")
    return dst


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("--config", required=True, help="robot source config yaml")
    p.add_argument("--install", action="store_true",
                   help="after importing, copy the result into omnigibson-robot-assets/models/")
    p.add_argument("--install-only", action="store_true",
                   help="skip the import and only do the copy (for when the import already ran)")
    args = p.parse_args(argv)

    from omnigibson.macros import gm

    import yaml
    with open(args.config, "r") as f:
        cfg = yaml.safe_load(f)
    name = cfg["name"]
    urdf_src = os.path.abspath(cfg["urdf_path"])

    if not args.install_only:
        # Import from a throwaway copy, never from the vendored source.
        #
        # The importer rewrites the URDF it is handed -- it swaps each collision mesh for the
        # CoACD-decomposed set and drops a *_with_meta_links.urdf beside it -- and it resolves
        # mesh paths relative to that file, so the decomposition lands in the source tree too.
        # Run it twice in place and the second run is converting the first run's output. That
        # defeats the whole point of vendoring the asset, which was to fix what gets converted.
        work = os.path.join(gm.DATA_PATH, "custom_dataset", "_import_work", name)
        if os.path.isdir(work):
            shutil.rmtree(work)
        shutil.copytree(os.path.dirname(urdf_src), work)
        cfg["urdf_path"] = os.path.join(work, os.path.basename(urdf_src))
        work_config = os.path.join(work, "source_config.yaml")
        os.makedirs(os.path.dirname(work_config), exist_ok=True)
        with open(work_config, "w") as f:
            yaml.safe_dump(cfg, f)
        args.config = work_config
        print(f"importing from a copy at {work} (vendored source left untouched)")

        dataset_root = os.path.join(gm.DATA_PATH, "custom_dataset")
        os.makedirs(dataset_root, exist_ok=True)

        import omnigibson.examples.robots.import_custom_robot as importer
        # Bind dataset_root in the example module's namespace only. The real function in
        # asset_conversion_utils is untouched, so anything else importing it is unaffected.
        importer.import_og_asset_from_urdf = _patched_importer(dataset_root)
        # ...and reconcile the name/root mismatch one level down.
        _patch_convert_urdf_to_usd()

        # .callback is the undecorated function; calling the click command directly would try to
        # parse our argv as its own.
        importer.import_custom_robot.callback(config=args.config)

    if args.install or args.install_only:
        install(name, gm.DATA_PATH, urdf_src=urdf_src)
    return 0


if __name__ == "__main__":
    sys.exit(main())
