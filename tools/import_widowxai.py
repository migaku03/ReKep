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


def _patched_importer(dataset_root):
    """Return `import_og_asset_from_urdf` with @dataset_root bound, keeping the rest as-is."""
    from omnigibson.utils.asset_conversion_utils import import_og_asset_from_urdf

    return functools.partial(import_og_asset_from_urdf, dataset_root=dataset_root)


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


def install(name, data_path):
    """Move the importer's output into the location robot_base.py actually reads."""
    src = os.path.join(data_path, "custom_dataset", "objects", "robot", name)
    dst = os.path.join(data_path, "omnigibson-robot-assets", "models", name)
    if not os.path.isdir(src):
        raise SystemExit(f"nothing to install: {src} does not exist")
    if os.path.isdir(dst):
        # Refuse rather than clobber. Re-importing is cheap; losing a hand-tuned USD is not.
        raise SystemExit(f"refusing to overwrite existing {dst} -- remove it first if that is what you want")
    print(f"installing {src}\n        -> {dst}")
    shutil.copytree(src, dst)
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
        name = yaml.safe_load(f)["name"]

    if not args.install_only:
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
        install(name, gm.DATA_PATH)
    return 0


if __name__ == "__main__":
    sys.exit(main())
