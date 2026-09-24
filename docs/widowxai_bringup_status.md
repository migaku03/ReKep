# WidowX AI bring-up — where this branch stands

Working state for `robot/widowx-ai`, written to survive a context compaction. Read this first
when resuming; it is the state of *this branch*, which is why it lives here and not in the parent
repo's `docs/`.

Last updated: 2026-09-12 (bring-up complete: all four stages done, see Stage 4 attempt 2)

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
| 1 | URDF → USD conversion and install | **done** -- re-run once, see camera_link fix below |
| 2 | Robot class + standalone bring-up checks | **done** -- all four checks in `tools/check_widowxai.py` pass |
| 3 | De-Fetch the ReKep environment layer, WidowX config, Lula descriptor | **done** -- runs without error across both Stage 4 attempts |
| 4 | `main.py` startup check | **done** -- 5-minute headless run, no crash, no instability. Loops without completing the task (expected, see below) |

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

## Stage 4, attempt 1: the arm exploded -- traced to a Stage 1 asset defect, not a Stage 3/4 bug

First run of
```
cd external/ReKep
export OPENAI_API_KEY="sk-dummy-not-used-cached-query-only"
python main.py --use_cached_query --config ./configs/config_widowxai.yaml
```
reached the solver (an `[OUTCOME]`-style optimisation debug block printed, with
`subgoal_constraint_cost: 286.5` and `msg: Maximum number of function call reached during
annealing` -- i.e. Stage 3's de-Fetched `environment.py`/`main.py` code path ran with no Python
errors). But watching it run, the arm was unstable from the very first physics step: it flailed
immediately, sent the pen and its holder flying, and eventually knocked the table away too.

The `[STALL]` diagnostic block (`environment.py`'s own logging, printed after 120 stalled steps)
had the answer:

```
contacts[robot] = ['base_link <-> table_1/base_link',
                    'camera_link <-> ceilings_*/base_link', 'camera_link <-> floors_*/base_link',
                    'camera_link <-> walls_*/base_link' (all four walls),
                    'camera_link <-> pen_1/base_link', 'camera_link <-> table_1/base_link', ...]
```
joint efforts on every arm joint reading in the hundreds to low thousands (`eff_norm` up to 599x
its own torque limit). One link cannot physically touch the ceiling, the floor, and all four walls
at once unless its collision volume is roughly room-sized -- which is exactly what had happened.

**Root cause, confirmed by reading the converted USD directly:** `camera_link`'s collision
geometry in the source URDF is a `<box size="0.023 0.042 0.042"/>` primitive (2.3 x 4.2 x 4.2 cm)
-- the *only* primitive (non-mesh) collision in the entire URDF; grepped for `<box>`/`<cylinder>`/
`<sphere>` to confirm. The importer does not handle that correctly: instead of a small box, the
USD contained several CoACD-style hull pieces (`camera_link_col_0` .. `_col_N`) each carrying a
per-piece `xformOp:scale` up to roughly 40x, centred multiple metres from the link origin -- in
effect a room-sized invisible collider rigidly bolted to the wrist. `camera_mount_d405`, the only
other camera-branch link with a `<collision>` block, references a plain mesh (not a primitive) and
decomposed normally; the defect is specific to how this importer path handles a `<box>` primitive
when routed through CoACD.

**Fixed at the source**, not by patching the importer: `camera_link` added to `no_collision_links`
in `assets/widowxai/widowxai_source_config.yaml`. Justified rather than a workaround of
convenience -- `assets/widowxai/README.md` already establishes that ReKep never uses this robot's
onboard camera (it reads two world-fixed cameras instead), so the wrist camera housing having no
collision costs nothing functionally. Re-ran Stage 1's import + Stage 2's conform + all four
`check_widowxai.py` checks after the fix: the USD now has an empty `collisions` scope under
`camera_link` (confirmed by reading it directly) and all checks still pass -- notably
check 4 (IK moves the arm) now tracks the commanded +x delta cleanly
(`before [0.2299, 0.0, 0.1051] -> after [0.3432, -0.0, 0.1053]`), where the pre-fix run had shown
a chaotic, mostly-off-axis response even in the checker's own empty scene. The z-axis measurement
also tightened to exactly 1.0000 (was 0.9903).

## Stage 4, attempt 2: no crash, stable physics, stuck in a backtrack loop (expected)

Same command, `OMNIGIBSON_HEADLESS=1`, capped at 5 minutes (`timeout 300`):

```
export OPENAI_API_KEY="sk-dummy-not-used-cached-query-only"
export OMNIGIBSON_HEADLESS=1
python main.py --use_cached_query --config ./configs/config_widowxai.yaml
```

**No explosion this time.** Every `[STALL]` block in this run shows joint efforts in the single-
to-low-double digits (`eff_norm` a few tenths to ~0.8) and a stable Jacobian condition number of
32.6, against attempt 1's efforts in the hundreds to 599x saturation. `base_link <-> table_1` is
still listed as a contact (the base sits essentially flush with the tabletop, z=0.69 vs the
table's own top at z=0.697), but it is no longer producing runaway force -- it is a static,
low-force contact the whole run. The camera_link fix is confirmed as the actual cause of attempt
1's explosion, not a symptom of something else.

Ran for the full 5 minutes without crashing: 1133 physics steps, `[DEBUG keypoints]` printing
every loop iteration, `execute_action`'s OSC path clearly driving the arm (`OSC pose not reached
after N steps` messages with shrinking-then-stalling position error). This satisfies the Stage 4
success criterion from the approved plan -- ReKep starts up and runs its full pipeline (keypoint
read -> backtrack check -> subgoal solver -> path solver -> OSC execution) on the WidowX AI
without crashing.

**It does not make task progress.** The log shows `[stage=2] backtrack to stage 1` nine times in
the 5-minute window, with the end-effector barely moving between them (a 1-2 cm jitter around
`[-0.28, -0.09, 0.72]`). Cause, from the joint state in the `[STALL]` blocks: `joint_2` is pinned
at its upper limit (`2.3562`) and `joint_3` at its lower limit (`-1.5708`) across every stall in
this stretch of the run -- the arm is physically unable to reach the pose stage 2's path
constraint needs, so every loop immediately fails that constraint's tolerance check and backtracks
straight back to stage 1, repeating indefinitely.

**This is the expected failure mode, not a new bug.** It is exactly what
`docs/widowxai_bringup_status.md`'s own scope note and `assets/widowxai/README.md` have said from
the start: the pen-task layout and its workspace bounds (`configs/config_widowxai.yaml`'s
`bounds_min`/`bounds_max`, still copied verbatim from Fetch) were authored for a mobile base with
a lifting torso, not a 0.7 m fixed arm bolted to one spot on the table -- re-deriving them is
explicitly out of scope for this bring-up (see "What this is" at the top of this document). One
data point in favour of this reading over a deeper bug: `contacts[robot]` briefly listed
`gripper_right <-> pen_1` early in the run, i.e. the arm did get physically close to the pen at
least once -- the joint-limit pin-out happening specifically at the stage-2 target rather than
everywhere is consistent with "reachable region is smaller than the task assumes," not with a
broken kinematic chain or controller.

## Stage 4, attempt 3: the grasp itself, fixed -- pen now lifts cleanly off the table

The user ran attempt 2 themselves (GUI, not headless) and reported: motion was smooth, no
instability (matches the headless finding above), but the pencil holder had fallen off the table,
and the gripper closed *above* the pen, missing it.

**Pencil holder: confirmed pre-existing, unrelated to this branch.** `git diff
port/behavior1k-v3.7.2 -- configs/og_scene_file_pen.json` is empty -- this file has not been
touched since before the WidowX AI work started (last changed in `81b41ea`, "Restore the pen to
the size the demo was built around"). Its AABB at scene load (`stage4_run2.log`,
`environment.py`'s own printout, i.e. before the robot has moved at all): `pencil_holder_1
xy=[-0.588,-0.427]x[0.126,0.284]` against `table_1 xy=[-0.376,0.644]x[-1.227,1.182]` -- the
holder's x-range sits entirely outside the table's, a ~5cm gap, and its z-range already reads
`[-0.0000, 0.1243]`, i.e. resting on the *floor*, at the very first printout. This would happen
with Fetch too; it is a defect in the shared scene file's authored object placement, not something
this branch introduced or can fix without touching the scene both robots use. Not fixed here --
flagged for whoever next touches `og_scene_file_pen.json`.

**The grasp-height miss: root-caused, fixed, and verified.** `stage4_run2.log`'s `[GRASP AXES]`
lines (`main.py`'s own diagnostic print, fires once per grasp attempt) show the *raw, unclipped*
descent target landing at world z = 0.6706-0.6924 across several attempts -- deliberately below
the table top (0.6972) by design, per `_execute_grasp_action`'s own comment ("the advance is
deliberately aimed *through* the object"). But `environment.py`'s workspace-bounds clip fired on
every single attempt (`"Target position is out of bounds, clipping to workspace bounds"`,
immediately following each `[GRASP AXES]` line), because `bounds_min[2]` was still 0.698 -- the
Fetch number, already flagged in this doc as not yet re-derived. The eef tip never got to descend
past table height.

That alone would only mean "descends a bit less than intended," not "misses the pen entirely" --
so a second factor had to be involved. Measured directly with the new `tools/check_grasp_depth.py`
(transforms the four hand-authored `_ag_start_points`/`_ag_end_points` from
`robots/widowxai.py` into the eef-local frame): the region *between* the open jaws -- where an
object actually needs to sit to be grasped -- averages z_local = -0.0319 m (range -0.0034 to
-0.0604), i.e. it sits roughly 3cm *behind* the eef tip, toward the wrist. Combined with the
eef's near-vertical approach (`localZ->world` z-component consistently around -0.99), that 3cm
behind-the-tip offset becomes roughly 3cm *above* the tip in world height. With the tip clipped at
table height (0.698) and the pen's own top surface at 0.7172, the jaws were closing at
approximately 0.698 + 0.0316 = 0.7296 -- about 1.2cm above the pen's top. That is the mechanism
behind "grasping above the pen."

**Fix:** lowered `bounds_min[2]` in `configs/config_widowxai.yaml` from 0.698 to 0.665 -- below
every unclipped target logged (0.6706-0.6924) with a small margin, still consistent with the
existing "aim through the object" design rather than a deeper change to it. Full reasoning is
recorded in the config file itself, next to the value.

**Verified**, same 5-minute headless run as attempt 2:
- `[GRASP AXES]` fires exactly once (not 9+ times like attempt 2) -- the grasp succeeds on the
  first try.
- `[GRASP] ag_obj_in_hand=pen_1` immediately after.
- Zero `backtrack to stage 1` for the rest of the run (attempt 2 had 9 in the same window) -- the
  grasp-failure-driven backtrack loop is gone entirely.
- Final `[STALL]` block: `pen_1 aabb z=[0.7325,0.8060] (tabletop is 0.6970)` -- the pen is lifted
  clear of the table -- and `contacts[pen_1] = ['base_link <-> .../gripper_right']` -- its only
  contact is the gripper, nothing else. A clean, held grasp.

**What's left after the grasp is the already-known limitation, not a new one.** Stage 2
(reorientation) now runs into `joint_4` sitting exactly at 1.5708 rad (= pi/2, its own travel
limit and the wrist's documented singularity, `docs/sim_platform_decision.md` section 5), Jacobian
condition number 17615.8. This is the fixed-base reach/layout mismatch this document has flagged
from the start (see "What this is" and the Stage 4 attempt-2 note below) -- not something this fix
was meant to address.

## Next step

**Grasping now works; reorientation still does not, for the reason already on record.** The pen
lifts cleanly on the first attempt. What remains is the stage-2 target driving `joint_4` into its
own singularity/limit -- task-layout and workspace-bounds redesign for a bolted-down 0.7 m arm,
which was out of scope for this bring-up from the start (see "What this is") and still is.

If that redesign is picked up next:
- Start from the joint-limit pin-out above rather than from scratch: `joint_2`-`joint_4` hitting
  their limits at the stage-2 target suggests the current `bounds_min`/`bounds_max` (and possibly
  the base `position` in `configs/config_widowxai.yaml`) reach past what this arm can actually
  cover from where it is bolted down. `tools/analyze_wxai_kinematics.py` (parent repo) already has
  the machinery to map out the real reachable envelope rather than guessing new bounds by trial
  and error.
- Separately, `configs/og_scene_file_pen.json`'s `pencil_holder_1` placement (off the table edge
  at scene load, see above) needs fixing if the pencil-holder part of the task matters going
  forward -- it is shared with the Fetch config, so fixing it affects both robots.

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
