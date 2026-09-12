# WidowX AI Follower — vendored source assets

The robot this research will actually use. Vendored rather than downloaded on demand so that the
input to the USD conversion is fixed: the conversion is not reproducible if the thing being
converted can change under it.

**The importer does not treat its input as read-only.** It rewrites the URDF it is handed --
swapping each collision mesh for the CoACD-decomposed set, and writing a `*_with_meta_links.urdf`
beside it -- and it resolves mesh paths relative to that file, so the decomposition lands in the
same directory. Run it twice in place and the second run converts the first run's output. So
`tools/import_widowxai.py` copies this directory to a scratch location and converts the copy;
what is committed here stays as Trossen published it. (An earlier note in this file claimed the
source was untouched. That was wrong, and `git status` is what caught it.)

## Provenance

| | |
|---|---|
| Source | `https://github.com/TrossenRobotics/ManiSkill-WidowX_AI` (branch `main`) |
| Retrieved | 2026-09-11 |
| Files | `wxai_follower.urdf` + `meshes/` (13 STL + 1 PNG), taken verbatim |
| Upstream last push | 2025-06-06 |

Trossen publishes the same arm twice. `trossen_arm_description` carries it as a **xacro**
(`wxai.urdf.xacro`), which needs ROS to expand; this repo carries an already-expanded **plain
URDF**, which `convert_urdf_to_usd` can read directly. That is the only reason this source was
chosen over the other. The two have not been checked against each other — see "Not verified".

## What is in the URDF

Chain (`tools/analyze_wxai_kinematics.py` in the parent repo reads this file directly):

```
base_link -joint_0-> link_1 -joint_1-> link_2 -joint_2-> link_3
          -joint_3-> link_4 -joint_4-> link_5 -joint_5-> link_6

link_6 -right_carriage_joint (prismatic, 0..0.044)-> carriage_right -fixed-> gripper_right
link_6 -left_carriage_joint  (prismatic, 0..0.044)-> carriage_left  -fixed-> gripper_left
link_6 -ee_gripper (fixed, xyz = 0.156062 0 0)   -> ee_gripper_link
link_6 -camera_mount_joint (fixed)               -> camera_mount_d405 -> ... -> camera_link
```

Facts that drive the import config, all read from the URDF:

- **Mesh paths are relative** (`meshes/*.stl`), no `package://` to rewrite.
- **Visual and collision reference the same STL** for every link except `camera_link`, whose
  collision is a box primitive. So the collision geometry is full-detail render geometry and
  *must* be decomposed.
- The links are substantially concave — measured volume / convex-hull volume:
  `link_1` 0.28, `camera_mount_d405` 0.29, `link_3` 0.34, `carriage_*` 0.35, `link_4` 0.38,
  `link_6` 0.44, `gripper_*` 0.55, `link_2` 0.56. **Convex hulls would inflate every link, and
  in particular would fill in the gripping recess of the finger pads.** Hence CoACD, not convex.
- `d405.stl` is authored in millimetres (extents ~42 x 42 x 23); the URDF already applies
  `scale="0.001 0.001 0.001"`. It is visual-only, so it never reaches the collision pipeline.
- `link_5.stl` is not watertight. CoACD tolerates this.

## Approach axis — why the import config builds an `eef_link`

**OmniGibson's end-effector convention, quoted from `import_custom_robot.py`:**

> Convention for these eef vis links should be tuned such that:
>   z-axis points out from the tips of the fingers
>   y-axis points in the direction from the left finger to the right finger

The WidowX AI URDF does not follow that convention. `ee_gripper_link` is a plain +x offset from
`link_6` with zero rotation, so its **approach axis is local x**, while the fingers separate along
±y. Using `ee_gripper_link` directly as the robot's eef link would therefore put the approach on
x and diverge from every other OmniGibson robot.

That divergence is not cosmetic. Chapter 11 of `docs/behavior1k_rekep_setup.md` traced a grasp
failure to exactly this: `subgoal_solver.py` optimises a *column index* of the eef rotation matrix,
and it was moved from column 0 to column 2 because in this OmniGibson the approach axis is z. The
same class of bug was hit independently by a third party porting ReKep to Franka
(huangwl18/ReKep issue #24). Rather than revert that fix and re-introduce a robot-specific column
index, the import config generates a dedicated `eef_link` rotated into OmniGibson's convention.

The rotation, in `link_6` coordinates:

```
eef z  = +x of link_6    (approach -- matches the ee_gripper offset direction)
eef y  = -y of link_6    (left finger sits at +y, right at -y, so left->right is -y)
eef x  = y cross z       = +z of link_6
  => quaternion (x, y, z, w) = (0.7071068, 0, 0.7071068, 0)
```

`assets/widowxai/widowxai_source_config.yaml` places it at the same offset as `ee_gripper`
(0.156062 m along link_6's x) so the origin is unchanged and only the axes are relabelled.

**This must still be confirmed on the loaded robot, not trusted from the URDF**, because the
conversion may apply its own frame handling. The check is the one chapter 11 used: print the
columns of `get_ee_pose()`'s rotation matrix and confirm column 2 points out of the fingertips.

## Not verified

- The geometry has **not** been cross-checked against `trossen_arm_description`'s xacro.
- The upstream repo is small (2 stars) and last pushed 2025-06-06, though it sits under Trossen's
  own GitHub organisation.
- Inertial values are taken as published; they have not been sanity-checked against the hardware.
- The wrist camera pose in the import config is derived from the ROS `camera_link` convention
  (x forward, z up) mapped onto Isaac's (looks along -z, +y up). ReKep does not use the robot's
  onboard camera -- it uses two world-fixed cameras -- so this has not been visually confirmed.
