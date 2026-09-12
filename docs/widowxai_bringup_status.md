# WidowX AI bring-up — where this branch stands

Working state for `robot/widowx-ai`, written to survive a context compaction. Read this first
when resuming; it is the state of *this branch*, which is why it lives here and not in the parent
repo's `docs/`.

Last updated: 2026-09-12

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
| 2 | Robot class + standalone bring-up checks | **in progress** -- one blocker, below |
| 3 | De-Fetch the ReKep environment layer, WidowX config, Lula descriptor | **written, unverified** |
| 4 | `main.py` startup check | not started |

Stage 2's checker is `tools/check_widowxai.py`. It has four checks and currently dies in check 1.
Successive runs have pushed the failure further each time:

```
run 1   update_links: no unique root link          -> fixed by nesting joints
run 2   root prim has no xformOp:scale             -> fixed by adding root xform ops
run 3   loads; og.sim.play() rejects the gripper   <- CURRENT
```

## The current blocker

```
AssertionError: Controllers should only control driveable joints!
```

Cause is established. Of the eight movable joints in the USD, exactly one has no drive:

```
joint_0 .. joint_5        DRIVE
left_carriage_joint       DRIVE
right_carriage_joint      *** NO DRIVE ***
```

`right_carriage_joint` carries `<mimic joint="left_carriage_joint"/>` in the URDF. Isaac's
importer said so during conversion ("*Joint right_carriage_joint has a velocity limit defined but
is set to mimic joint left_carriage_joint*") and gives mimic joints no drive -- PhysX makes them
follow by constraint. `robots/widowxai.py` declares **both** carriages in `finger_joint_names`, so
`MultiFingerGripperController` tries to drive one that cannot be driven.

### Decision: drop the joint from the class, do not edit the URDF

**Direction being taken:** remove `right_carriage_joint` from `finger_joint_names`, leaving
`left_carriage_joint` as the single driven finger joint. `finger_link_names` keeps both
`gripper_left` and `gripper_right` -- both pads still exist and still carry contact geometry.

Why not the other option (deleting `<mimic>` from the vendored URDF so both joints get drives):

- **The mimic is physically accurate.** The WidowX AI gripper is a rack-and-pinion parallel jaw --
  one actuator moves both jaws. Removing the mimic would model a two-motor gripper that does not
  exist, which is exactly the kind of sim/real divergence this whole robot swap is meant to remove.
- It would mean patching Trossen's published asset, and the vendored URDF is supposed to be what
  they shipped.

That VX300S and Fetch both declare two independently driven finger joints (neither URDF contains a
`<mimic>`) is why the alternative was considered at all -- OmniGibson's shipped parallel grippers
are all two-motor. It is not evidence that a one-motor gripper is unsupported.

**Unverified, and the thing to check first when acting on this:** whether
`MultiFingerGripperController` and the assisted-grasp machinery tolerate one driven joint against
two finger links. Specifically whether anything requires
`len(finger_joint_names) == len(finger_link_names)`. If something does, the fallback is to edit
the URDF after all, and the reasoning above should be recorded as overruled rather than quietly
dropped.

## Uncommitted work at the time of writing

All of it is the vendoring-hygiene fix, and it is finished, just not committed:

- **Source URDF restored.** The importer had rewritten `assets/widowxai/wxai_follower.urdf` in
  place (887 insertions, 407 deletions -- collision meshes swapped for the CoACD-decomposed sets)
  and dropped a `widowxai_with_meta_links.urdf` beside it. An earlier claim in this work that the
  source stayed pristine was **wrong**; `git status` caught it.
- **Driver fixed** so `tools/import_widowxai.py` copies the asset directory to a scratch location
  and converts the copy. Without this, a second import converts the first import's output.
- **README corrected** in `assets/widowxai/README.md`, with the earlier wrong claim called out.
- **`.gitignore`** now covers the generated `*_with_meta_links.urdf`.
- `tools/nest_usd_joints.py` renamed to `tools/conform_robot_usd.py` -- its job grew from nesting
  joints to also adding the root prim's transform ops.

## Next step

**Measure the end-effector frame on the loaded robot.** This is the highest-value item and it
still has no result. Everything downstream -- `grasp_approach_axis` in `config_widowxai.yaml`, the
column index in `subgoal_solver.py:75` -- is currently set from what the URDF and the import
config *say* rather than from what the conversion *produced*.

Why it is worth doing properly: chapter 11 of the parent repo's
`docs/behavior1k_rekep_setup.md` traced a grasp failure to exactly this, and upstream issue #24 is
a third party hitting it again porting ReKep to Franka. In that issue the fix was not one axis but
two: `catslashbin` corrected the approach axis to column 2, and then `hanlanqian` found that was
still not enough and had to add a y-axis term as well. **So check both:** that column 2 points out
of the fingertips, and that column 1 is the finger separation direction.

`tools/check_widowxai.py` already does both -- it takes the vector between the two finger links as
ground truth for y rather than trusting the config. It just has not run past the gripper assert
yet.

Expected, from the import config: approach on column 2, because the generated `eef_link` rotates
the URDF's native x-forward frame by (x,y,z,w) = (0.7071068, 0, 0.7071068, 0). Confirmed present
in the USD as `localRot0 = (0, 0.7071068, 0, 0.7071068)` in USD's (w,x,y,z) order. **Confirmed in
the file is not confirmed on the robot.**

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
