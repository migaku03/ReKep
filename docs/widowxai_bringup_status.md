# WidowX AI bring-up — where this branch stands

Working state for `robot/widowx-ai`, written to survive a context compaction. Read this first
when resuming; it is the state of *this branch*, which is why it lives here and not in the parent
repo's `docs/`.

Last updated: 2026-09-12 (Stage 2 completed and verified)

## What this is

Bringing the Trossen WidowX AI Follower -- the arm this research will use on hardware -- into
OmniGibson, per `docs/sim_platform_decision.md` (parent repo) option (b). Scope agreed with the
user: **get as far as ReKep starting up without crashing.** The pen task is *not* expected to
succeed; its layout was authored for a mobile base with a lifting torso and has not been
redesigned for a bolted-down 0.7 m arm. That redesign is a separate piece of work.

## Ground rules being held to

- **This work edits nothing in BEHAVIOR-1K.** Every workaround for the seven incompatibilities
  below lives in this repo.

  Note when checking: that checkout is *not* clean, and never was. It carries a pre-existing
  modification to `OmniGibson/omnigibson/controllers/osc_controller.py` -- the `lambda_damping`
  option added during chapter 18's singularity work, which `configs/config.yaml` still refers to.
  Plus two untracked setup logs. So `git -C ../../BEHAVIOR-1K status` showing changes is expected;
  what matters is that nothing new appears there. Confirm with
  `git -C ../../BEHAVIOR-1K diff --stat` -- it should list only `osc_controller.py`.
- **The Fetch baseline stays runnable.** It is preserved on `port/behavior1k-v3.7.2`, and on this
  branch too: `configs/config.yaml` is unchanged apart from an additive `ik:` block naming the
  descriptor and target link it always used, and `main.py --config` defaults to it.
- Robot choice is a config file, not a code edit:
  `python main.py --use_cached_query --config ./configs/config_widowxai.yaml`

## Environment

| | |
|---|---|
| Python | `C:\Users\smiga\miniconda3\envs\behavior\python.exe` |
| Parent repo branch | `docs/sim-vs-real` |
| This repo branch | `robot/widowx-ai`, cut from `port/behavior1k-v3.7.2` |
| Installed robot asset | `C:\Users\smiga\research\BEHAVIOR-1K\datasets\omnigibson-robot-assets\models\widowxai\` (`misc/`, `urdf/`, `usd/`) |
| USD backup before conforming | `/tmp/widowxai_backup.usda` |

## Stage progress

| Stage | What | State |
|---|---|---|
| 0 | Branch cut, Fetch preserved | **done** |
| 1 | URDF → USD conversion and install | **done** |
| 2 | Robot class + standalone bring-up checks | **done** -- all four checks in `tools/check_widowxai.py` pass |
| 3 | De-Fetch the ReKep environment layer, WidowX config, Lula descriptor | written, **not yet verified** |
| 4 | `main.py` startup check | not started |

Stage 2's checker is `tools/check_widowxai.py`. It took four runs to get through, each pushing the
failure further:

```
run 1   update_links: no unique root link                    -> fixed by nesting joints
run 2   root prim has no xformOp:scale                        -> fixed by adding root xform ops
run 3   og.sim.play() rejects the gripper (mimic joint drive)  -> fixed: drop the mimic joint from
                                                                   finger_joint_names (below)
run 4   PhysxMimicJointAPI referenceJoint points nowhere       -> fixed: tools/conform_robot_usd.py
                                                                   now retargets stale relationships
run 5   assisted-grasp point inference: KeyError on arm name  -> fixed: hand-authored AG points
        (all four checks pass)                                   (below)
```

## Blockers hit and resolved, in order

### 1. Gripper controller rejects the mimic joint

```
AssertionError: Controllers should only control driveable joints!
```

Of the eight movable joints in the USD, exactly one had no drive:

```
joint_0 .. joint_5        DRIVE
left_carriage_joint       DRIVE
right_carriage_joint      *** NO DRIVE ***
```

`right_carriage_joint` carries `<mimic joint="left_carriage_joint"/>` in the URDF. Isaac's
importer said so during conversion ("*Joint right_carriage_joint has a velocity limit defined but
is set to mimic joint left_carriage_joint*") and gives mimic joints no drive -- PhysX makes them
follow by constraint. `robots/widowxai.py` declared **both** carriages in `finger_joint_names`, so
`MultiFingerGripperController` tried to drive one that cannot be driven.

**Resolved as planned:** `finger_joint_names` now lists only `left_carriage_joint`.
`finger_link_names` still lists both `gripper_left` and `gripper_right` -- both pads exist and
still carry contact geometry. Reasoning for not touching the vendored URDF's `<mimic>` instead is
unchanged from the previous version of this document (the gripper is a real rack-and-pinion
single-actuator mechanism; removing the mimic would model a two-motor gripper that does not
exist). **The previously-unverified assumption is now confirmed:** nothing in OmniGibson requires
`len(finger_joint_names) == len(finger_link_names)`. Checked directly:
`assisted_grasp_start_points`/`_end_points` are built from `finger_link_names` only
(`manipulation_robot.py:340-424`, via `self.finger_links`), and `gripper_control_idx` /
`MultiFingerGripperController` accept a `dof_idx` of whatever length `finger_joint_names` happens
to be (`manipulation_robot.py:851-858`, `multi_finger_gripper_controller.py`). `_default_joint_pos`
was not touched either -- it is sized to the robot's full 8 movable joints (6 arm + 2 carriages),
not to `finger_joint_names`; `right_carriage_joint` is still a real, simulated joint, just not one
the controller drives directly.

### 2. `conform_robot_usd.py` left a dangling relationship after reparenting

```
[Error] [omni.physx.plugin] Usd Physics: PhysxMimicJointAPI at .../right_carriage_joint points to
a non existing prim at .../joints/left_carriage_joint in attribute "referenceJoint".
```

A bug in our own tooling, not upstream. `_nest_joints` moves each joint from the flat `Scope
"joints"` to live under its `physics:body0` link, and fixes up that joint's *own*
`physics:body0`/`body1` targets (already-resolved absolute paths, untouched by the move). But
`right_carriage_joint`'s `PhysxMimicJointAPI:rotY:referenceJoint` relationship names
`left_carriage_joint` by its **old** path, and that reference was never updated when
`left_carriage_joint` moved.

**Fixed generically, not special-cased to mimic joints:** `conform_robot_usd.py` gained
`_fix_dangling_relationship_targets`, which walks every relationship in the layer after nesting
and, for any target that no longer resolves to a live prim, looks it up by name elsewhere in the
stage and repoints it there (safe because reparenting changes a path but not a name, and joint
names are unique in a robot URDF). Re-running the tool on the already-installed USD applied the
fix in place -- no need to redo the ~20-minute CoACD conversion.

### 3. Assisted-grasp point auto-inference cannot find the fingers' parent joint

```
[WARNING] [omnigibson.robots.manipulation_robot] Could not infer relevant finger link properties
because: Expected articulated parent joint for finger link WidowXAI:gripper_left but found none!
```
followed by check 3 failing with `KeyError: '0'` when reading `assisted_grasp_start_points`.

Not a bug, but a real mismatch between this robot's link topology and what
`ManipulationRobot._infer_finger_properties` assumes. It finds each finger's parent by searching
`self.joints` for a **driven** (nonzero-DOF) joint whose `body1` is the finger link. On Fetch,
VX300S, and Franka's default gripper, the finger link *is* the direct child of the driven
prismatic/revolute joint, so this works. On WidowX AI it is not: `left_carriage_joint`'s child is
`carriage_left`, and `gripper_left` (the actual contact pad) hangs off *that* through a second,
**fixed** joint (`left_gripper_joint`). Fixed joints carry zero DOF and are entirely excluded from
`self.joints` (`entity_prim.py:update_joints` only wraps joints with `joint_dof_counts > 0`), so
the search finds nothing. `_initialize()` catches the resulting `AssertionError` and downgrades it
to a `log.warning`, which is why the robot still loads -- but
`_default_ag_start_points`/`_default_ag_end_points` are left unpopulated, and
`assisted_grasp_start_points` (which falls back to them) then raises `KeyError` on the arm name.

**Fixed by hand-authoring the grasp points**, following the exact pattern every end-effector
option in `omnigibson/robots/franka.py` uses (`_ag_start_points`/`_ag_end_points` class attributes
plus `_assisted_grasp_start_points`/`_end_points` property overrides). The four points are not
guessed: `tools/derive_ag_points.py` reproduces `_infer_finger_properties`'s own geometric
derivation, substituting `link_6` for the joint-connected parent the search cannot find (correct
here because `link_6` is `body0` of *both* carriage joints -- the true common base the fingers
extend from, joint-lookup aside).

One deliberate deviation from upstream's formula, forced by a second finding from the same script:
the built-in derivation additionally clamps both z-values into `+/-finger_range *
AG_DEFAULT_GRASP_POINT_Z_PROP` and asserts they straddle the eef's z=0 plane -- an assumption that
the eef origin sits well *behind* the fingertip with room on both sides. WidowX AI's `ee_gripper`
offset instead places the eef origin right at the pad's own tip (measured `finger_max_z` =
+0.0004 m, i.e. a hair's width past zero, not the multi-centimetre margin Fetch/Panda have), so
there is no room in front of z=0 for a point to land in and the straddle assertion is
unsatisfiable as written. Replaced with two points spanning the same 20%-95% window of the
finger's own physical length, measured from its base at `link_6` rather than from the eef's z=0
plane. Full reasoning is in the comment above `_ag_start_points` in `robots/widowxai.py`.

## The end-effector frame -- measured, not assumed, and confirmed correct

This was flagged as the top-priority open item in the previous version of this document, on the
strength of chapter 11 (parent repo) and upstream issue #24, where one axis fix alone was not
enough and a second, independent check on the other axis was required. Both are now measured on
the *loaded* robot, not read off the URDF or the import config:

- **Column 1 (y) -- finger separation axis.** `|dot(column 1, gripper_right -> gripper_left)| =
  1.0000`. Exact.
- **Column 2 (z) -- approach axis, should point out of the fingertips.** Measuring this directly
  from the eef link itself is unreliable here: `eef_link`'s own origin sits right at the pads'
  tip (see finding 3 above), so the vector from it to the fingertip midpoint is only a few
  millimetres long and its sign is dominated by noise -- in one run it even measured pointing
  *backward* (`dot = -0.998`) purely because the eef origin had ended up a hair on the far side of
  the tip. Measuring instead from `link_6` (which sits well back from the tip, giving an
  unambiguous multi-centimetre vector) gives `dot(column 2, link_6 -> fingertip midpoint) =
  0.9903`. `tools/check_widowxai.py` now asserts this measurement (from `link_6`, not from the
  eef) as part of check 2, so it will catch a regression rather than silently pass or silently
  flip sign next time the asset is rebuilt.

Practical consequence: the import config's rotation derivation in `assets/widowxai/README.md` (eef
z = link_6's +x, eef y = link_6's -y) is confirmed correct as built. `grasp_approach_axis: z` in
`configs/config_widowxai.yaml` and the column index in `subgoal_solver.py` (already on column 2
from the Fetch-era fix, chapter 11) need **no further change** for this robot.

## Vendoring hygiene (committed)

The vendoring-hygiene fixes described in the previous version of this document are committed
(`ff34f4b`). Nothing outstanding there.

## Next step

**Stage 3.** `configs/config_widowxai.yaml`, `configs/widowxai_descriptor.yaml`, and the
de-Fetching of `environment.py`/`main.py` were written before Stage 2 finished and have not been
exercised. With Stage 2 now fully green, the next action is:

```
cd external/ReKep
export OPENAI_API_KEY="sk-dummy-not-used-cached-query-only"
python main.py --use_cached_query --config ./configs/config_widowxai.yaml
```

Success criterion (per the approved plan): reaching the first solver iteration without crashing --
`[DEBUG keypoints]` and `execute_action` log lines appearing. `[OUTCOME] SUCCESS` is *not*
expected and is out of scope; the pen-task layout was authored for Fetch's mobile base and lifting
torso and has not been redesigned for a bolted-down 0.7 m fixed arm. Whatever the first failure
turns out to be (if any), record it here in the same "blockers hit and resolved" format before
attempting a fix, the same way each Stage 2 blocker was recorded above as it was found.

## Reference: the seven incompatibilities found so far

Five are upstream defects; two were mistakes of mine in fixing them. Worth keeping because #6 and
#7 mean *nobody* can import a custom robot into v3.7.2 through the documented path -- they are
invisible while you only use the robots OmniGibson ships, whose USDs predate the current importer.

| # | Symptom | Cause |
|---|---|---|
| 1 | `import_og_asset_from_urdf() missing 1 required positional argument: 'dataset_root'` | the example script never passes it |
| 2 | `convert_urdf_to_usd() got an unexpected keyword argument 'dataset_root'` | half-finished rename of `dataset_name`; callers updated, callee not |
| 3 | `XFormPrim.__init__() got an unexpected keyword argument 'prim_path'` | Isaac 4.5 split the single-prim class out as `SingleXFormPrim` |
| 4 | `ModuleNotFoundError: isaacsim.core.prims.xform_prim` | **mine.** `lazy` attribute-walks that path; it is not importable |
| 5 | `'function' object has no attribute 'set_local_poses'` | **mine.** replaced a class with a function; the real class calls itself through that module global |
| 6 | `Exactly one single root link should have been found ... found none/multiple` | Isaac 4.5 puts joints in a flat `Scope "joints"`; OmniGibson infers the tree from joints nested under links |
| 7 | `RuntimeError: Could not infer dtype of NoneType` | robot root prim has no `xformOp:scale`; `XFormPrim._post_load` reads it unconditionally |

## Reproducing each step

```
cd external/ReKep

# 1. convert + install  (~20 min: CoACD re-decomposes every mesh every run, no cache;
#    one mesh alone takes 225 s. Do not set a timeout under ~30 min.)
python tools/import_widowxai.py --config assets/widowxai/widowxai_source_config.yaml --install

# 2. conform the USD to what OmniGibson expects  (idempotent, ~1 min)
python tools/conform_robot_usd.py \
    C:\Users\smiga\research\BEHAVIOR-1K\datasets\omnigibson-robot-assets\models\widowxai\usd\widowxai.usda

# 3. bring-up checks
python tools/check_widowxai.py

# 4. ReKep startup (stage 4, not reached yet)
export OPENAI_API_KEY="sk-dummy-not-used-cached-query-only"
python main.py --use_cached_query --config ./configs/config_widowxai.yaml
```

`install()` refuses to overwrite an existing `models/widowxai`; delete it first to re-install.

### Log noise that is not a failure

- `"/World/viewer_camera" is not a valid Usd.Prim or UsdGeom.Camera` and
  `'NoneType' object has no attribute 'GetCamera'` -- headless viewport, harmless.
- `Windows fatal exception: access violation` at shutdown -- the pre-existing Windows Isaac Sim
  teardown problem from chapter 16.5. It fires *after* work is saved. Check whether the output
  exists before treating it as a failure.
- hdf5 / `omni.sensors.nv.common` DLL load errors at startup -- pre-existing on this machine,
  also reported upstream in issue #19.

## Numbers chosen deliberately (do not re-derive by eye)

- **Reset pose** `[0.0, 1.38, 1.12, -1.28, 0.0, 0.0, 0.044, 0.044]`. Found by search over the
  kinematics, not guessed: approach within 1.8 deg of straight down, Jacobian condition number
  11.5, tightest joint-limit margin 0.291 rad. The same six arm values are the seed in
  `configs/widowxai_descriptor.yaml`; **the two must move together.**
- **Base pose** `position [0.15, 0.0, 0.69]`, `orientation [0, 0, 1, 0]` (180 deg yaw). The table
  spans x in [-0.364, 0.631] and both task objects sit within 10 cm of its -x edge, so there is no
  table behind them to mount on; from +x they are 0.45-0.48 m away. The yaw is required because
  joint_0 travels only +/-175 deg and cannot turn the 180 the layout would otherwise need.
- **CoACD, not convex hulls.** Measured volume/convex-hull ratios run 0.28-0.56 across the links;
  hulls would fatten every link and fill in the finger pads' gripping recess.
