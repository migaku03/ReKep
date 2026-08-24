import time
import atexit
import numpy as np
import torch as th
import os
import datetime
import transform_utils as T
import trimesh
import open3d as o3d
import imageio
import omnigibson as og
import omnigibson.lazy as lazy
from omnigibson.macros import gm
from omnigibson.utils.usd_utils import PoseAPI, mesh_prim_mesh_to_trimesh_mesh, mesh_prim_shape_to_trimesh_mesh
from omnigibson.robots.fetch import Fetch
from omnigibson.controllers import IsGraspingState
from og_utils import OGCamera
from utils import (
    bcolors,
    get_clock_time,
    angle_between_rotmat,
    angle_between_quats,
    get_linear_interpolation_steps,
    linear_interpolate_poses,
)
from omnigibson.robots.manipulation_robot import ManipulationRobot
from omnigibson.controllers.controller_base import ControlType, BaseController

# Don't use GPU dynamics and use flatcache for performance boost
gm.USE_GPU_DYNAMICS = True
gm.ENABLE_FLATCACHE = False

# some customization to the OG functions
def custom_clip_control(self, control):
    """
    Clips the inputted @control signal based on @control_limits.

    Args:
        control (Array[float]): control signal to clip

    Returns:
        Array[float]: Clipped control signal
    """
    clipped_control = control.clip(
        self._control_limits[self.control_type][0][self.dof_idx],
        self._control_limits[self.control_type][1][self.dof_idx],
    )
    idx = (
        self._dof_has_limits[self.dof_idx]
        if self.control_type == ControlType.POSITION
        else [True] * self.control_dim
    )
    if len(control) > 1:
        control[idx] = clipped_control[idx]
    return control

Fetch._initialize = ManipulationRobot._initialize
BaseController.clip_control = custom_clip_control

# Deliberate appearance overrides, applied after the scene loads. Maps object name to the
# multiplicative diffuse tint to force onto every material the object owns.
#
# pencil_holder_1 renders as a pale beige vessel, but the demo -- and the cached VLM
# constraints, which talk about "the black pen holder" -- were written against a black one.
# The model id (pencil_holder/muqeud) and scale still match upstream, so this is the asset
# having been re-authored underneath us rather than anything being applied wrongly: the
# diffuse and normal maps share one UV layout and agree with each other, i.e. the mesh really
# does map its outer wall onto the beige part of the atlas now. Nothing to repair here, and
# nothing repairable anyway -- the model ships as muqeud.encrypted.usd and the 2024 asset
# package is gone. Substituting one of the other ten pencil_holder models is worse: their
# diffuse maps are flat light grey, so they are not black either, and moving the mesh would
# strand the cached keypoints that sit on this one.
#
# So state the colour the demo assumes, explicitly, as a scene override.
#
# Tinting alone left the inside of the holder looking like brushed metal. That is a second,
# separate defect in the same asset: muqeud's reflection map is baked against a different UV
# layout than its diffuse and normal maps (which do agree with each other), so it scatters
# bright highlights into the wrong places. omnigibson_vray_mtl.mdl wires the material's
# Reflection straight to reflection_texture's tint and reflection_metalness to
# metalness_texture's mono channel, so pointing both at a black image zeroes the reflection
# lobe outright. Hence 'matte'.
#
# What this still does not fix: the silhouette stays crumpled rather than cylindrical. That
# is mesh geometry, inside the encrypted USD, and out of reach from here.
BLACK_TEXTURE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'assets', 'black.png')

APPEARANCE_OVERRIDES = {
    'pencil_holder_1': dict(diffuse_tint=(0.06, 0.06, 0.06), matte=True),
}

class ReKepOGEnv:
    def __init__(self, config, scene_file, verbose=False):
        self.video_cache = []
        # [STEP TIMING] investigation-only instrumentation (chapter 20; listed for deletion in
        # chapter 7). Three disjoint segments cover every microsecond from the first _step()
        # entry to the last _step() exit, so their sum is a checksum against the wall clock:
        #   sim -- og_env.step()/og.sim.step(): physics + rendering. Budget 1/action_frequency.
        #   cam -- get_cam_obs() + video_cache: the per-step GPU->CPU readback, which upstream
        #          runs on EVERY step and which is not part of the simulator's own cost.
        #   gap -- the interval between two _step() calls, i.e. everything OUTSIDE the sim.
        #          The subgoal/path solvers live here; the sim is not advancing at all.
        # Nothing below touches control flow, the action, or the call order.
        self._t_sim, self._t_cam, self._t_gap = [], [], []
        self._t_step_exit = None
        self._t_first_enter = None
        self._t_timing_reported = False
        atexit.register(self.report_step_timing)
        self.config = config
        self.verbose = verbose
        self.config['scene']['scene_file'] = scene_file
        self.bounds_min = np.array(self.config['bounds_min'])
        self.bounds_max = np.array(self.config['bounds_max'])
        self.interpolate_pos_step_size = self.config['interpolate_pos_step_size']
        self.interpolate_rot_step_size = self.config['interpolate_rot_step_size']
        # create omnigibson environment
        self.step_counter = 0
        self.og_env = og.Environment(dict(scene=self.config['scene'], robots=[self.config['robot']['robot_config']], env=self.config['og_sim']))
        # before anything is stepped or rendered, so the scene is never shown in the wrong
        # colours and no recorded frame catches the transition
        self._apply_appearance_overrides()
        self._zero_object_velocities()
        # let the table drop onto the floor and everything come to rest before snapshotting
        # the initial state, so that reset() restores a settled scene rather than a falling one
        for _ in range(30): og.sim.step()
        self._zero_object_velocities()
        self.og_env.scene.update_initial_file()
        for _ in range(10): og.sim.step()
        # robot vars
        self.robot = self.og_env.robots[0]
        dof_idx = np.concatenate([self.robot.trunk_control_idx,
                                  self.robot.arm_control_idx[self.robot.default_arm]])
        reset_joint_pos = self.robot.reset_joint_pos[dof_idx]
        self.reset_joint_pos = reset_joint_pos.detach().cpu().numpy() if hasattr(reset_joint_pos, "detach") else np.asarray(reset_joint_pos)
        self.world2robot_homo = T.pose_inv(T.pose2mat(self.robot.get_position_orientation()))
        # action vector layout. ReKep originally hardcoded the indices for the arm/gripper
        # commands, but OmniGibson has since added a separate "trunk" controller to Fetch,
        # which shifts every controller after it by one slot. The total action dim happens to
        # stay 12, so the mismatch is silent -- query the mapping from the robot instead.
        action_idx = self.robot.controller_action_idx
        self.arm_action_idx = T.to_numpy(action_idx[f'arm_{self.robot.default_arm}']).astype(int)
        self.gripper_action_idx = T.to_numpy(action_idx[f'gripper_{self.robot.default_arm}']).astype(int)
        assert len(self.arm_action_idx) == 6, f"expected a 6-DOF arm controller, got {len(self.arm_action_idx)}"
        print(f"[environment.py] action_dim={self.robot.action_dim} layout=" +
              str({k: T.to_numpy(v).tolist() for k, v in action_idx.items()}), flush=True)
        # geometry sanity check. The scene file was authored against the 2024 asset package;
        # the table model has since been swapped and the pen mesh's canonical axes rotated, so
        # the placements were recomputed from the current assets' metadata. Verify them.
        for _name in ('table_1', 'pen_1', 'pencil_holder_1'):
            _obj = self.og_env.scene.object_registry("name", _name)
            if _obj is None:
                print(f"[environment.py] AABB {_name}: NOT FOUND", flush=True)
                continue
            _lo, _hi = (T.to_numpy(_v) for _v in _obj.aabb)
            print(f"[environment.py] AABB {_name}: z=[{_lo[2]:.4f}, {_hi[2]:.4f}] "
                  f"xy=[{_lo[0]:.3f},{_lo[1]:.3f}]-[{_hi[0]:.3f},{_hi[1]:.3f}]", flush=True)
        # initialize cameras
        self._initialize_cameras(self.config['camera'])
        self.last_og_gripper_action = 1.0

    def _zero_object_velocities(self):
        """
        Restoring a saved scene puts every object back in place but does NOT restore its
        velocity, so whatever an object picked up while the scene was being assembled
        survives the restore. The pen holder in particular starts out with ~5 m/s upward
        and ~76 rad/s of spin and immediately launches itself off the table -- the scene
        file records zero velocity for it. Clear the leftover motion before the sim runs.
        """
        zero = th.zeros(3, dtype=th.float32)
        for obj in self.og_env.scene.objects:
            # walls/floors/ceilings are kinematic: the entity exposes set_linear_velocity
            # but its root link does not implement it, so check the link itself
            root = getattr(obj, "root_link", None)
            if root is None or not hasattr(root, "set_linear_velocity"):
                continue
            obj.set_linear_velocity(zero)
            obj.set_angular_velocity(zero)

    def _apply_appearance_overrides(self):
        """Apply the scene overrides listed in APPEARANCE_OVERRIDES.

        diffuse_tint multiplies the sampled albedo, so it darkens the whole surface while
        keeping its shading, which is what we want here -- albedo_add would just flatten it.
        Only the V-Ray and OmniPBR material classes implement the setter; the base
        MaterialPrim's is a no-op, so read the value back and report what actually stuck
        rather than assuming it applied.
        """
        black = lazy.pxr.Sdf.AssetPath(BLACK_TEXTURE)
        for name, spec in APPEARANCE_OVERRIDES.items():
            obj = self.og_env.scene.object_registry("name", name)
            if obj is None:
                print(f"[environment.py] appearance {name}: NOT FOUND", flush=True)
                continue
            tint = spec['diffuse_tint']
            # VRayMaterialPrim.diffuse_tint's setter calls color.tolist(), so it needs a
            # tensor rather than a plain tuple
            tint_t = th.tensor(tint, dtype=th.float32)
            applied = 0
            for material in obj.materials:
                material.diffuse_tint = tint_t
                readback = material.diffuse_tint
                if readback is not None and np.allclose(T.to_numpy(readback), tint, atol=1e-4):
                    applied += 1
                else:
                    print(f"[environment.py] appearance {name}: {type(material).__name__} at "
                          f"{material.prim_path} did not take the tint (read back "
                          f"{readback})", flush=True)
                if spec.get('matte'):
                    for inp in ('reflection_texture', 'metalness_texture'):
                        try:
                            material.set_input(inp=inp, val=black)
                        except Exception as e:
                            print(f"[environment.py] appearance {name}: could not blank "
                                  f"{inp}: {type(e).__name__}: {e}", flush=True)
            print(f"[environment.py] appearance {name}: tint {tint} applied to {applied}/"
                  f"{len(obj.materials)} materials"
                  f"{', reflection blanked' if spec.get('matte') else ''}", flush=True)

    def _empty_action(self):
        """Zero action vector sized to the robot's actual action space."""
        return np.zeros(self.robot.action_dim)

    # ======================================
    # = exposed functions
    # ======================================
    def get_sdf_voxels(self, resolution, exclude_robot=True, exclude_obj_in_hand=True):
        """
        open3d-based SDF computation
        1. recursively get all usd prim and get their vertices and faces
        2. compute SDF using open3d
        """
        start = time.time()
        exclude_names = ['wall', 'floor', 'ceiling']
        if exclude_robot:
            exclude_names += ['fetch', 'robot']
        if exclude_obj_in_hand:
            assert self.config['robot']['robot_config']['grasping_mode'] in ['assisted', 'sticky'], "Currently only supported for assisted or sticky grasping"
            in_hand_obj = self.robot._ag_obj_in_hand[self.robot.default_arm]
            if in_hand_obj is not None:
                exclude_names.append(in_hand_obj.name.lower())
        trimesh_objects = []
        for obj in self.og_env.scene.objects:
            if any([name in obj.name.lower() for name in exclude_names]):
                continue
            for link in obj.links.values():
                for mesh in link.collision_meshes.values():
                    mesh_type = mesh.prim.GetPrimTypeInfo().GetTypeName()
                    if mesh_type == 'Mesh':
                        trimesh_object = mesh_prim_mesh_to_trimesh_mesh(mesh.prim)
                    else:
                        trimesh_object = mesh_prim_shape_to_trimesh_mesh(mesh.prim)
                    world_pose_w_scale = PoseAPI.get_world_pose_with_scale(mesh.prim_path)
                    trimesh_object.apply_transform(world_pose_w_scale)
                    trimesh_objects.append(trimesh_object)
        # chain trimesh objects
        scene_mesh = trimesh.util.concatenate(trimesh_objects)
        # Create a scene and add the triangle mesh
        scene = o3d.t.geometry.RaycastingScene()
        vertex_positions = scene_mesh.vertices
        triangle_indices = scene_mesh.faces
        vertex_positions = o3d.core.Tensor(vertex_positions, dtype=o3d.core.Dtype.Float32)
        triangle_indices = o3d.core.Tensor(triangle_indices, dtype=o3d.core.Dtype.UInt32)
        _ = scene.add_triangles(vertex_positions, triangle_indices)  # we do not need the geometry ID for mesh
        # create a grid
        shape = np.ceil((self.bounds_max - self.bounds_min) / resolution).astype(int)
        steps = (self.bounds_max - self.bounds_min) / shape
        grid = np.mgrid[self.bounds_min[0]:self.bounds_max[0]:steps[0],
                        self.bounds_min[1]:self.bounds_max[1]:steps[1],
                        self.bounds_min[2]:self.bounds_max[2]:steps[2]]
        grid = grid.reshape(3, -1).T
        # compute SDF
        sdf_voxels = scene.compute_signed_distance(grid.astype(np.float32))
        # convert back to np array
        sdf_voxels = sdf_voxels.cpu().numpy()
        # open3d has flipped sign from our convention
        sdf_voxels = -sdf_voxels
        sdf_voxels = sdf_voxels.reshape(shape)
        self.verbose and print(f'{bcolors.WARNING}[environment.py | {get_clock_time()}] SDF voxels computed in {time.time() - start:.4f} seconds{bcolors.ENDC}')
        return sdf_voxels

    def get_cam_obs(self):
        self.last_cam_obs = dict()
        for cam_id in self.cams:
            self.last_cam_obs[cam_id] = self.cams[cam_id].get_obs()  # each containing rgb, depth, points, seg
        return self.last_cam_obs
    
    def register_keypoints(self, keypoints):
        """
        Args:
            keypoints (np.ndarray): keypoints in the world frame of shape (N, 3)
        Returns:
            None
        Given a set of keypoints in the world frame, this function registers them so that their newest positions can be accessed later.
        """
        if not isinstance(keypoints, np.ndarray):
            keypoints = np.array(keypoints)
        self.keypoints = keypoints
        self._keypoint_registry = dict()
        self._keypoint2object = dict()
        exclude_names = ['wall', 'floor', 'ceiling', 'table', 'fetch', 'robot']
        for idx, keypoint in enumerate(keypoints):
            closest_distance = np.inf
            for obj in self.og_env.scene.objects:
                if any([name in obj.name.lower() for name in exclude_names]):
                    continue
                for link in obj.links.values():
                    for mesh in link.visual_meshes.values():
                        mesh_prim_path = mesh.prim_path
                        mesh_type = mesh.prim.GetPrimTypeInfo().GetTypeName()
                        if mesh_type == 'Mesh':
                            trimesh_object = mesh_prim_mesh_to_trimesh_mesh(mesh.prim)
                        else:
                            trimesh_object = mesh_prim_shape_to_trimesh_mesh(mesh.prim)
                        world_pose_w_scale = PoseAPI.get_world_pose_with_scale(mesh.prim_path)
                        trimesh_object.apply_transform(world_pose_w_scale)
                        points_transformed = trimesh_object.sample(1000)
                        
                        # find closest point
                        dists = np.linalg.norm(points_transformed - keypoint, axis=1)
                        point = points_transformed[np.argmin(dists)]
                        distance = np.linalg.norm(point - keypoint)
                        if distance < closest_distance:
                            closest_distance = distance
                            closest_prim_path = mesh_prim_path
                            closest_point = point
                            closest_obj = obj
            self._keypoint_registry[idx] = (closest_prim_path, PoseAPI.get_world_pose(closest_prim_path))
            self._keypoint2object[idx] = closest_obj
            # overwrite the keypoint with the closest point
            self.keypoints[idx] = closest_point

    def get_keypoint_positions(self):
        """
        Args:
            None
        Returns:
            np.ndarray: keypoints in the world frame of shape (N, 3)
        Given the registered keypoints, this function returns their current positions in the world frame.
        """
        assert hasattr(self, '_keypoint_registry') and self._keypoint_registry is not None, "Keypoints have not been registered yet."
        keypoint_positions = []
        for idx, (prim_path, init_pose) in self._keypoint_registry.items():
            init_pose = T.pose2mat(init_pose)
            centering_transform = T.pose_inv(init_pose)
            keypoint_centered = np.dot(centering_transform, np.append(self.keypoints[idx], 1))[:3]
            curr_pose = T.pose2mat(PoseAPI.get_world_pose(prim_path))
            keypoint = np.dot(curr_pose, np.append(keypoint_centered, 1))[:3]
            keypoint_positions.append(keypoint)
        return np.array(keypoint_positions)

    def get_object_by_keypoint(self, keypoint_idx):
        """
        Args:
            keypoint_idx (int): the index of the keypoint
        Returns:
            pointer: the object that the keypoint is associated with
        Given the keypoint index, this function returns the name of the object that the keypoint is associated with.
        """
        assert hasattr(self, '_keypoint2object') and self._keypoint2object is not None, "Keypoints have not been registered yet."
        return self._keypoint2object[keypoint_idx]

    def get_collision_points(self, noise=True):
        """
        Get the points of the gripper and any object in hand.
        """
        # add gripper collision points
        collision_points = []
        for obj in self.og_env.scene.objects:
            if 'fetch' in obj.name.lower():
                for name, link in obj.links.items():
                    if 'gripper' in name.lower() or 'wrist' in name.lower():  # wrist_roll and wrist_flex
                        for collision_mesh in link.collision_meshes.values():
                            mesh_prim_path = collision_mesh.prim_path
                            mesh_type = collision_mesh.prim.GetPrimTypeInfo().GetTypeName()
                            if mesh_type == 'Mesh':
                                trimesh_object = mesh_prim_mesh_to_trimesh_mesh(collision_mesh.prim)
                            else:
                                trimesh_object = mesh_prim_shape_to_trimesh_mesh(collision_mesh.prim)
                            world_pose_w_scale = PoseAPI.get_world_pose_with_scale(mesh_prim_path)
                            trimesh_object.apply_transform(world_pose_w_scale)
                            points_transformed = trimesh_object.sample(1000)
                            # add to collision points
                            collision_points.append(points_transformed)
        # add object in hand collision points
        in_hand_obj = self.robot._ag_obj_in_hand[self.robot.default_arm]
        if in_hand_obj is not None:
            for link in in_hand_obj.links.values():
                for collision_mesh in link.collision_meshes.values():
                    mesh_type = collision_mesh.prim.GetPrimTypeInfo().GetTypeName()
                    if mesh_type == 'Mesh':
                        trimesh_object = mesh_prim_mesh_to_trimesh_mesh(collision_mesh.prim)
                    else:
                        trimesh_object = mesh_prim_shape_to_trimesh_mesh(collision_mesh.prim)
                    world_pose_w_scale = PoseAPI.get_world_pose_with_scale(collision_mesh.prim_path)
                    trimesh_object.apply_transform(world_pose_w_scale)
                    points_transformed = trimesh_object.sample(1000)
                    # add to collision points
                    collision_points.append(points_transformed)
        collision_points = np.concatenate(collision_points, axis=0)
        return collision_points

    def reset(self):
        self.og_env.reset()
        self._zero_object_velocities()  # og_env.reset() restores poses but not velocities
        self.robot.reset()
        for _ in range(5): self._step()
        self.open_gripper()
        # moving arm to the side to unblock view 
        ee_pose = self.get_ee_pose()
        ee_pose[:3] += np.array([0.0, -0.2, -0.1])
        action = np.concatenate([ee_pose, [self.get_gripper_null_action()]])
        self.execute_action(action, precise=True)
        self.video_cache = []
        print(f'{bcolors.HEADER}Reset done.{bcolors.ENDC}')

    def is_grasping(self, candidate_obj=None):
        return self.robot.is_grasping(candidate_obj=candidate_obj) == IsGraspingState.TRUE

    def get_ee_pose(self):
        ee_pos, ee_xyzw = (T.to_numpy(self.robot.get_eef_position()), T.to_numpy(self.robot.get_eef_orientation()))
        ee_pose = np.concatenate([ee_pos, ee_xyzw])  # [7]
        return ee_pose

    def get_ee_pos(self):
        return self.get_ee_pose()[:3]

    def get_ee_quat(self):
        return self.get_ee_pose()[3:]
    
    def get_arm_joint_postions(self):
        assert isinstance(self.robot, Fetch), "The IK solver assumes the robot is a Fetch robot"
        arm = self.robot.default_arm
        dof_idx = np.concatenate([self.robot.trunk_control_idx, self.robot.arm_control_idx[arm]])
        arm_joint_pos = T.to_numpy(self.robot.get_joint_positions())[dof_idx]
        return arm_joint_pos

    def close_gripper(self):
        """
        Exposed interface: 1.0 for closed, -1.0 for open, 0.0 for no change
        Internal OG interface: 1.0 for open, 0.0 for closed
        """
        if self.last_og_gripper_action == 0.0:
            return
        action = self._empty_action()
        action[self.gripper_action_idx] = 0.0  # gripper: float. 0. for closed, 1. for open.
        for _ in range(30):
            self._step(action)
        self.last_og_gripper_action = 0.0
        self._report_grasp_state()

    def _report_grasp_state(self):
        """Temporary diagnostic: why does assisted grasping not latch onto the pen?"""
        arm = self.robot.default_arm
        try:
            finger_names = self.robot.finger_joint_names[arm]
            jpos = T.to_numpy(self.robot.get_joint_positions())
            jidx = {n: i for i, n in enumerate(self.robot.joints.keys())}
            fingers = {n: round(float(jpos[jidx[n]]), 5) for n in finger_names if n in jidx}
            candidates, contact_links = self.robot._find_gripper_contacts(arm=arm)
            finger_prims = {l.prim_path for l in self.robot.finger_links[arm]}
            per_obj = {c: sorted(p.split('/')[-1] for p in finger_prims.intersection(contact_links.get(c, set())))
                       for c in candidates}
            raycast = self.robot._find_gripper_raycast_collisions(arm=arm)
            in_hand = self.robot._ag_obj_in_hand[arm]
            print(f"[GRASP] fingers={fingers}", flush=True)
            print(f"[GRASP] contacts={sorted(candidates)}", flush=True)
            print(f"[GRASP] fingers_touching_per_obj={per_obj}", flush=True)
            print(f"[GRASP] raycast_hits={sorted(raycast)}", flush=True)
            print(f"[GRASP] ag_obj_in_hand={in_hand.name if in_hand is not None else None}", flush=True)
        except Exception as e:
            print(f"[GRASP] diagnostic failed: {type(e).__name__}: {e}", flush=True)

    def _report_stall(self, target_pose_world, tag=""):
        """Temporary diagnostic: why does the arm stop short of a waypoint?

        The OSC emits *torques*, so a stall can come from four different places and they are
        not distinguishable from the eef error alone:
          - a joint sitting on its position limit           -> look at the limit margins
          - the commanded torque clipped by the motor limit -> look at effort vs effort limit
          - something physically blocking the arm           -> look at the contact list
          - a kinematic singularity (Jacobian rank loss)    -> look at the singular values
        Dump all four at once.
        """
        try:
            robot = self.robot
            arm = robot.default_arm
            names = list(robot.joints.keys())
            dof_idx = np.concatenate([
                T.to_numpy(robot.trunk_control_idx).astype(int),
                T.to_numpy(robot.arm_control_idx[arm]).astype(int),
            ])

            q = T.to_numpy(robot.get_joint_positions())
            qd = T.to_numpy(robot.get_joint_velocities())
            lo = T.to_numpy(robot.joint_lower_limits)
            hi = T.to_numpy(robot.joint_upper_limits)
            eff = T.to_numpy(robot.get_joint_efforts())
            eff_norm = T.to_numpy(robot.get_joint_efforts(normalized=True))

            print(f"[STALL{tag}] ---- joint state (trunk + arm) ----", flush=True)
            print(f"[STALL{tag}] {'joint':<22}{'pos':>9}{'lo':>9}{'hi':>9}"
                  f"{'margin':>9}{'vel':>9}{'effort':>10}{'eff_norm':>10}", flush=True)
            for i in dof_idx:
                n = names[i]
                m_lo, m_hi = q[i] - lo[i], hi[i] - q[i]
                flag = ""
                if min(m_lo, m_hi) < np.deg2rad(2.0):
                    flag += "  <== AT POSITION LIMIT"
                if abs(eff_norm[i]) > 0.98:
                    flag += "  <== TORQUE SATURATED"
                print(f"[STALL{tag}] {n:<22}{q[i]:>9.4f}{lo[i]:>9.4f}{hi[i]:>9.4f}"
                      f"{min(m_lo, m_hi):>9.4f}{qd[i]:>9.4f}{eff[i]:>10.3f}{eff_norm[i]:>10.3f}{flag}",
                      flush=True)

            # --- arm Jacobian conditioning, in the robot base frame -------------------
            # index exactly the way controllable_object.py builds the OSC's jacobian entry:
            # rows are counted from the end, columns skip the 6 floating-base DOFs
            jac = T.to_numpy(robot.get_relative_jacobian())
            link_idx = robot._articulation_view.get_body_index(robot.eef_link_names[arm])
            start = 0 if robot.fixed_base else 6
            j_full = jac[-(robot.n_links - link_idx), :, start:start + robot.n_joints]
            arm_idx = T.to_numpy(robot.arm_control_idx[arm]).astype(int)
            j_eef = j_full[:, arm_idx]  # OSC only actuates the 7 arm joints
            sv = np.linalg.svd(j_eef, compute_uv=False)
            print(f"[STALL{tag}] jacobian singular values = {np.round(sv, 5).tolist()}", flush=True)
            print(f"[STALL{tag}] condition number = {sv[0] / max(sv[-1], 1e-12):.1f} "
                  f"(>1e3 means near-singular)", flush=True)

            # --- what is touching the robot / the grasped object ---------------------
            def contacts_of(entity, label):
                pairs = set()
                for link_name, link in entity.links.items():
                    for c in link.contact_list():
                        other = c.body1 if str(c.body0).startswith(entity.prim_path) else c.body0
                        if str(other).startswith(entity.prim_path):
                            continue  # self-contact between adjacent links, ignore
                        pairs.add(f"{link_name} <-> {str(other)}")
                print(f"[STALL{tag}] contacts[{label}] = {sorted(pairs) if pairs else 'NONE'}",
                      flush=True)

            contacts_of(robot, "robot")
            in_hand = robot._ag_obj_in_hand[arm]
            if in_hand is not None:
                contacts_of(in_hand, in_hand.name)
                lo_b, hi_b = (T.to_numpy(v) for v in in_hand.aabb)
                print(f"[STALL{tag}] {in_hand.name} aabb z=[{lo_b[2]:.4f},{hi_b[2]:.4f}] "
                      f"(tabletop is 0.6970)", flush=True)
            else:
                print(f"[STALL{tag}] contacts[in_hand] = NOTHING GRASPED", flush=True)

            # --- how far the eef actually is, in the robot frame ---------------------
            tgt_robot = np.dot(self.world2robot_homo, T.convert_pose_quat2mat(target_pose_world))
            cur = T.to_numpy(robot.get_relative_eef_position())
            print(f"[STALL{tag}] eef_robot={np.round(cur, 4).tolist()} "
                  f"target_robot={np.round(tgt_robot[:3, 3], 4).tolist()} "
                  f"delta={np.round(tgt_robot[:3, 3] - cur, 4).tolist()}", flush=True)
            print(f"[STALL{tag}] ----------------------------------", flush=True)
        except Exception as e:
            import traceback
            print(f"[STALL{tag}] diagnostic failed: {type(e).__name__}: {e}", flush=True)
            traceback.print_exc()

    def open_gripper(self):
        if self.last_og_gripper_action == 1.0:
            return
        action = self._empty_action()
        action[self.gripper_action_idx] = 1.0  # gripper: float. 0. for closed, 1. for open.
        for _ in range(30):
            self._step(action)
        self.last_og_gripper_action = 1.0

    def get_last_og_gripper_action(self):
        return self.last_og_gripper_action
    
    def get_gripper_open_action(self):
        return -1.0
    
    def get_gripper_close_action(self):
        return 1.0
    
    def get_gripper_null_action(self):
        return 0.0
    
    def compute_target_delta_ee(self, target_pose):
        target_pos, target_xyzw = target_pose[:3], target_pose[3:]
        ee_pose = self.get_ee_pose()
        ee_pos, ee_xyzw = ee_pose[:3], ee_pose[3:]
        pos_diff = np.linalg.norm(ee_pos - target_pos)
        rot_diff = angle_between_quats(ee_xyzw, target_xyzw)
        return pos_diff, rot_diff

    def execute_action(
            self,
            action,
            precise=True,
        ):
            """
            Moves the robot gripper to a target pose by specifying the absolute pose in the world frame and executes gripper action.

            Args:
                action (x, y, z, qx, qy, qz, qw, gripper_action): absolute target pose in the world frame + gripper action.
                precise (bool): whether to use small position and rotation thresholds for precise movement (robot would move slower).
            Returns:
                tuple: A tuple containing the position and rotation errors after reaching the target pose.
            """
            if precise:
                pos_threshold = 0.03
                rot_threshold = 3.0
            else:
                pos_threshold = 0.10
                rot_threshold = 5.0
            action = np.array(action).copy()
            assert action.shape == (8,)
            target_pose = action[:7]
            gripper_action = action[7]

            # ======================================
            # = status and safety check
            # ======================================
            if np.any(target_pose[:3] < self.bounds_min) \
                 or np.any(target_pose[:3] > self.bounds_max):
                print(f'{bcolors.WARNING}[environment.py | {get_clock_time()}] Target position is out of bounds, clipping to workspace bounds{bcolors.ENDC}')
                target_pose[:3] = np.clip(target_pose[:3], self.bounds_min, self.bounds_max)

            # ======================================
            # = interpolation
            # ======================================
            current_pose = self.get_ee_pose()
            pos_diff = np.linalg.norm(current_pose[:3] - target_pose[:3])
            rot_diff = angle_between_quats(current_pose[3:7], target_pose[3:7])
            pos_is_close = pos_diff < self.interpolate_pos_step_size
            rot_is_close = rot_diff < self.interpolate_rot_step_size
            if pos_is_close and rot_is_close:
                self.verbose and print(f'{bcolors.WARNING}[environment.py | {get_clock_time()}] Skipping interpolation{bcolors.ENDC}')
                pose_seq = np.array([target_pose])
            else:
                num_steps = get_linear_interpolation_steps(current_pose, target_pose, self.interpolate_pos_step_size, self.interpolate_rot_step_size)
                pose_seq = linear_interpolate_poses(current_pose, target_pose, num_steps)
                self.verbose and print(f'{bcolors.WARNING}[environment.py | {get_clock_time()}] Interpolating for {num_steps} steps{bcolors.ENDC}')

            # ======================================
            # = move to target pose
            # ======================================
            # move faster for intermediate poses
            intermediate_pos_threshold = 0.10
            intermediate_rot_threshold = 5.0
            for pose in pose_seq[:-1]:
                self._move_to_waypoint(pose, intermediate_pos_threshold, intermediate_rot_threshold)
            # move to the final pose with required precision
            pose = pose_seq[-1]
            self._move_to_waypoint(pose, pos_threshold, rot_threshold, max_steps=20 if not precise else 120)
            # compute error
            pos_error, rot_error = self.compute_target_delta_ee(target_pose)
            self.verbose and print(f'\n{bcolors.BOLD}[environment.py | {get_clock_time()}] Move to pose completed (pos_error: {pos_error}, rot_error: {np.rad2deg(rot_error)}){bcolors.ENDC}\n')

            # ======================================
            # = apply gripper action
            # ======================================
            if gripper_action == self.get_gripper_open_action():
                self.open_gripper()
            elif gripper_action == self.get_gripper_close_action():
                self.close_gripper()
            elif gripper_action == self.get_gripper_null_action():
                pass
            else:
                raise ValueError(f"Invalid gripper action: {gripper_action}")
            
            return pos_error, rot_error
    
    def sleep(self, seconds):
        start = time.time()
        while time.time() - start < seconds:
            self._step()
    
    def save_video(self, save_path=None):
        save_dir = os.path.join(os.path.dirname(__file__), 'videos')
        os.makedirs(save_dir, exist_ok=True)
        if save_path is None:
            save_path = os.path.join(save_dir, f'{datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S")}.mp4')
        video_writer = imageio.get_writer(save_path, fps=30)
        for rgb in self.video_cache:
            video_writer.append_data(rgb)
        video_writer.close()
        return save_path

    # ======================================
    # = internal functions
    # ======================================
    def _check_reached_ee(self, target_pos, target_xyzw, pos_threshold, rot_threshold):
        """
        this is supposed to be for true ee pose (franka hand) in robot frame
        """
        current_pos = T.to_numpy(self.robot.get_eef_position())
        current_xyzw = T.to_numpy(self.robot.get_eef_orientation())
        current_rotmat = T.quat2mat(current_xyzw)
        target_rotmat = T.quat2mat(target_xyzw)
        # calculate position delta
        pos_diff = (target_pos - current_pos).flatten()
        pos_error = np.linalg.norm(pos_diff)
        # calculate rotation delta
        rot_error = angle_between_rotmat(current_rotmat, target_rotmat)
        # print status
        self.verbose and print(f'{bcolors.WARNING}[environment.py | {get_clock_time()}]  Curr pose: {current_pos}, {current_xyzw} (pos_error: {pos_error.round(4)}, rot_error: {np.rad2deg(rot_error).round(4)}){bcolors.ENDC}')
        self.verbose and print(f'{bcolors.WARNING}[environment.py | {get_clock_time()}]  Goal pose: {target_pos}, {target_xyzw} (pos_thres: {pos_threshold}, rot_thres: {rot_threshold}){bcolors.ENDC}')
        if pos_error < pos_threshold and rot_error < np.deg2rad(rot_threshold):
            self.verbose and print(f'{bcolors.WARNING}[environment.py | {get_clock_time()}] OSC pose reached (pos_error: {pos_error.round(4)}, rot_error: {np.rad2deg(rot_error).round(4)}){bcolors.ENDC}')
            return True, pos_error, rot_error
        return False, pos_error, rot_error

    def _move_to_waypoint(self, target_pose_world, pos_threshold=0.02, rot_threshold=3.0, max_steps=10):
        pos_errors = []
        rot_errors = []
        count = 0
        start_eef = T.to_numpy(self.robot.get_relative_eef_position())
        while count < max_steps:
            reached, pos_error, rot_error = self._check_reached_ee(target_pose_world[:3], target_pose_world[3:7], pos_threshold, rot_threshold)
            pos_errors.append(pos_error)
            rot_errors.append(rot_error)
            if reached:
                break
            # convert world pose to robot pose
            target_pose_robot = np.dot(self.world2robot_homo, T.convert_pose_quat2mat(target_pose_world))
            # convert to relative pose to be used with the underlying controller
            rel_eef_pos = T.to_numpy(self.robot.get_relative_eef_position())
            rel_eef_quat = T.to_numpy(self.robot.get_relative_eef_orientation())
            target_quat_robot = T.mat2quat(target_pose_robot[:3, :3])
            relative_position = target_pose_robot[:3, 3] - rel_eef_pos
            relative_quat = T.quat_distance(target_quat_robot, rel_eef_quat)
            relative_axisangle = T.quat2axisangle(relative_quat)
            if count % 10 == 0:
                print(f"[WP {count:>3}] eef_robot={np.round(rel_eef_pos,4).tolist()} "
                      f"cmd_dpos={np.round(relative_position,4).tolist()} "
                      f"pos_err={pos_error:.4f} rot_err={np.rad2deg(rot_error):.2f}deg", flush=True)
            assert isinstance(self.robot, Fetch), "this action space is only for fetch"
            action = self._empty_action()  # base/trunk/camera stay at zero
            action[self.arm_action_idx[:3]] = relative_position
            action[self.arm_action_idx[3:]] = relative_axisangle
            action[self.gripper_action_idx] = self.last_og_gripper_action
            # step the action
            _ = self._step(action=action)
            count += 1
        if count == max_steps:
            print(f'{bcolors.WARNING}[environment.py | {get_clock_time()}] OSC pose not reached after {max_steps} steps (pos_error: {pos_errors[-1].round(4)}, rot_error: {np.rad2deg(rot_errors[-1]).round(4)}){bcolors.ENDC}')
            # a real stall is "the arm barely moved at all", not just "ran out of steps":
            # the intermediate waypoints are deliberately cut off early while still moving
            travelled = float(np.linalg.norm(
                T.to_numpy(self.robot.get_relative_eef_position()) - start_eef))
            print(f'{bcolors.WARNING}[environment.py] eef travelled {travelled * 100:.2f} cm '
                  f'over those {max_steps} steps{bcolors.ENDC}', flush=True)
            if travelled < 0.02 or max_steps >= 60:
                self._report_stall(target_pose_world, tag=f" {max_steps}st")

    def _step(self, action=None):
        _t_enter = time.perf_counter()  # [STEP TIMING]
        if self._t_step_exit is None:
            self._t_first_enter = _t_enter
        else:
            self._t_gap.append(_t_enter - self._t_step_exit)
        if hasattr(self, 'disturbance_seq') and self.disturbance_seq is not None:
            next(self.disturbance_seq)
        _t0 = time.perf_counter()  # [STEP TIMING]
        if action is not None:
            self.og_env.step(action)
        else:
            og.sim.step()
        _t1 = time.perf_counter()  # [STEP TIMING]
        cam_obs = self.get_cam_obs()
        rgb = cam_obs[1]['rgb']
        if len(self.video_cache) < self.config['video_cache_size']:
            self.video_cache.append(rgb)
        else:
            self.video_cache.pop(0)
            self.video_cache.append(rgb)
        self.step_counter += 1
        _t2 = time.perf_counter()  # [STEP TIMING]
        self._t_sim.append(_t1 - _t0)
        self._t_cam.append(_t2 - _t1)
        self._t_step_exit = _t2

    def report_step_timing(self, tag='final'):
        """[STEP TIMING] cumulative summary of where the wall clock went. See __init__.

        Investigation-only. Called at every stage transition as well as at the end, because the
        4-minute cap kills the process outright and atexit does not run then -- a killed run
        must still leave usable data. Safe with no steps recorded, and the 'final' report is
        emitted at most once so the atexit hook cannot duplicate it.
        """
        if not self._t_sim:
            return
        if tag == 'final':
            if getattr(self, '_t_timing_reported', True):
                return
            self._t_timing_reported = True
        budget = 1.0 / float(self.config['og_sim']['action_frequency'])
        print(f'{bcolors.OKBLUE}[STEP TIMING] ---- cumulative @ {tag} ----{bcolors.ENDC}',
              flush=True)
        print(f'[STEP TIMING] sim budget = {budget * 1000:.1f} ms '
              f'(action_frequency {self.config["og_sim"]["action_frequency"]} Hz)', flush=True)
        for label, xs in (('sim', self._t_sim), ('cam', self._t_cam), ('gap', self._t_gap)):
            if not xs:
                continue
            a = np.asarray(xs, dtype=float)
            print(f'[STEP TIMING] {label:<4} n={a.size:<6d} total={a.sum():8.2f}s  '
                  f'med={np.median(a) * 1000:8.2f}ms  p95={np.percentile(a, 95) * 1000:9.2f}ms  '
                  f'max={a.max() * 1000:10.2f}ms', flush=True)
        sim = np.asarray(self._t_sim, dtype=float)
        over = sim > budget
        print(f'[STEP TIMING] sim over budget: {int(over.sum())} / {sim.size} steps '
              f'({100.0 * over.mean():.1f}%), excess total={float((sim[over] - budget).sum()):.2f}s',
              flush=True)
        if self._t_gap:
            gap = np.asarray(self._t_gap, dtype=float)
            big = gap > 1.0
            print(f'[STEP TIMING] gap > 1.0 s: {int(big.sum())} times, '
                  f'total={float(gap[big].sum()):.2f}s  (this is the frozen-sim time)', flush=True)
        total = sim.sum() + float(np.sum(self._t_cam)) + float(np.sum(self._t_gap))
        covered = self._t_step_exit - self._t_first_enter
        print(f'[STEP TIMING] checksum: segments={total:.2f}s vs first-enter..last-exit='
              f'{covered:.2f}s  (diff {abs(total - covered):.3f}s)', flush=True)

    def _initialize_cameras(self, cam_config):
        """
        ::param poses: list of tuples of (position, orientation) of the cameras
        """
        self.cams = dict()
        for cam_id in cam_config:
            cam_id = int(cam_id)
            self.cams[cam_id] = OGCamera(self.og_env, cam_config[cam_id])
        # NOTE: do not warm the sensors with og.sim.step() here. Stepping advances physics,
        # and this runs after the settled scene has been snapshotted, so the extra steps
        # leave the robot and the objects somewhere the rest of the pipeline does not expect
        # -- measured: 374 unreached waypoints and 51 joint-limit hits versus 0 of each.
        for _ in range(10): og.sim.render()