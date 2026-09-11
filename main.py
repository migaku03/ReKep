import torch
import numpy as np
import json
import os
import argparse
from environment import ReKepOGEnv
from keypoint_proposal import KeypointProposer
from constraint_generation import ConstraintGenerator
from ik_solver import IKSolver
from subgoal_solver import SubgoalSolver
from path_solver import PathSolver
from visualizer import Visualizer
import transform_utils as T
# Importing a robot class is what registers it with OmniGibson, so config.yaml can name it.
# WidowXAI lives in this repo rather than under omnigibson/robots/ so that the BEHAVIOR-1K
# checkout the Fetch baseline depends on stays untouched -- see docs/sim_platform_decision.md.
from robots.widowxai import WidowXAI  # noqa: F401  (imported for the side effect of registering)
from utils import (
    bcolors,
    get_config,
    load_functions_from_txt,
    get_linear_interpolation_steps,
    spline_interpolate_poses,
    get_callable_grasping_cost_fn,
    print_opt_debug_dict,
)

class Main:
    def __init__(self, scene_file, visualize=False, config_path="./configs/config.yaml"):
        global_config = get_config(config_path=config_path)
        self.config = global_config['main']
        self.bounds_min = np.array(self.config['bounds_min'])
        self.bounds_max = np.array(self.config['bounds_max'])
        self.visualize = visualize
        # set random seed
        np.random.seed(self.config['seed'])
        torch.manual_seed(self.config['seed'])
        torch.cuda.manual_seed(self.config['seed'])
        # initialize keypoint proposer and constraint generator
        self.keypoint_proposer = KeypointProposer(global_config['keypoint_proposer'])
        self.constraint_generator = ConstraintGenerator(global_config['constraint_generator'])
        # initialize environment
        self.env = ReKepOGEnv(global_config['env'], scene_file, verbose=False)
        # setup ik solver (for reachability cost)
        #
        # Which descriptor and which target link come from config.yaml rather than from a check
        # on the robot's class, so that switching robots is a config edit. The IK here only
        # produces a reachability cost for the solvers; execution goes through OmniGibson's own
        # controller, so the target link needs to be the right *link*, not a perfect eef frame.
        ik_cfg = self.config['ik']
        ik_solver = IKSolver(
            robot_description_path=os.path.join(os.path.dirname(__file__), "configs", ik_cfg['descriptor']),
            robot_urdf_path=self.env.robot.urdf_path,
            eef_name=ik_cfg['eef_name'],
            reset_joint_pos=self.env.reset_joint_pos,
            world2robot_homo=self.env.world2robot_homo,
        )
        # initialize solvers
        self.subgoal_solver = SubgoalSolver(global_config['subgoal_solver'], ik_solver, self.env.reset_joint_pos)
        self.path_solver = PathSolver(global_config['path_solver'], ik_solver, self.env.reset_joint_pos)
        # initialize visualizer
        if self.visualize:
            self.visualizer = Visualizer(global_config['visualizer'], self.env)

    def perform_task(self, instruction, rekep_program_dir=None, disturbance_seq=None):
        self.env.reset()
        cam_obs = self.env.get_cam_obs()
        rgb = cam_obs[self.config['vlm_camera']]['rgb']
        points = cam_obs[self.config['vlm_camera']]['points']
        mask = cam_obs[self.config['vlm_camera']]['seg']
        # ====================================
        # = keypoint proposal and constraint generation
        # ====================================
        if rekep_program_dir is None:
            keypoints, projected_img = self.keypoint_proposer.get_keypoints(rgb, points, mask)
            print(f'{bcolors.HEADER}Got {len(keypoints)} proposed keypoints{bcolors.ENDC}')
            if self.visualize:
                self.visualizer.show_img(projected_img)
            metadata = {'init_keypoint_positions': keypoints, 'num_keypoints': len(keypoints)}
            rekep_program_dir = self.constraint_generator.generate(projected_img, instruction, metadata)
            print(f'{bcolors.HEADER}Constraints generated{bcolors.ENDC}')
        # ====================================
        # = execute
        # ====================================
        self._execute(rekep_program_dir, disturbance_seq)

    def _report_outcome(self):
        """Say whether the pen actually ended up in the holder.

        Nothing else does. The loop above returns as soon as the last stage's action queue is
        empty, which reports "finished" for a run that dropped the pen 21 cm wide of the holder,
        and the recorded video is not a fallback -- one run came out 1037 frames of pure black.
        So measure the end state directly, from the two objects' AABBs (the same access pattern
        as the geometry check at environment.py:134-141).

        Call this after env.sleep(), so the pen has settled wherever it is going to settle.
        """
        scene = self.env.og_env.scene
        pen, holder = (scene.object_registry("name", n) for n in ('pen_1', 'pencil_holder_1'))
        if pen is None or holder is None:
            print(f"{bcolors.FAIL}[OUTCOME] cannot judge: pen_1 or pencil_holder_1 missing{bcolors.ENDC}", flush=True)
            return
        p_lo, p_hi = (T.to_numpy(v) for v in pen.aabb)
        h_lo, h_hi = (T.to_numpy(v) for v in holder.aabb)
        pen_xy = (p_lo[:2] + p_hi[:2]) / 2

        # inserted: the pen's lowest point is below the rim, i.e. it is down inside the vessel
        inserted = bool(p_lo[2] < h_hi[2])
        # centred: the pen's horizontal centre lies within the holder's footprint
        centred = bool(np.all(pen_xy >= h_lo[:2]) and np.all(pen_xy <= h_hi[:2]))
        # upright: a 24.24 cm pen standing up spans most of that in z; lying down it spans ~3 cm
        span_z = float(p_hi[2] - p_lo[2])
        upright = bool(span_z > 0.20)

        print(f"[OUTCOME] pen   aabb z=[{p_lo[2]:.4f}, {p_hi[2]:.4f}] xy_centre=[{pen_xy[0]:.4f}, {pen_xy[1]:.4f}]", flush=True)
        print(f"[OUTCOME] holder aabb z=[{h_lo[2]:.4f}, {h_hi[2]:.4f}] "
              f"xy=[{h_lo[0]:.3f},{h_lo[1]:.3f}]-[{h_hi[0]:.3f},{h_hi[1]:.3f}]", flush=True)
        # how far the pen missed by, horizontally. This is the number worth watching while
        # tuning: "inserted" on its own is satisfied by a pen standing on the table beside the
        # holder, since its bottom is below the rim either way, so it only means anything
        # together with "centred".
        holder_xy = (h_lo[:2] + h_hi[:2]) / 2
        miss_cm = float(np.linalg.norm(pen_xy - holder_xy)) * 100
        print(f"[OUTCOME]   horizontal miss from holder centre: {miss_cm:.2f} cm", flush=True)
        print(f"[OUTCOME]   inserted (pen bottom {p_lo[2]:.4f} < rim {h_hi[2]:.4f}): {inserted}"
              f"   <- only meaningful with centred", flush=True)
        print(f"[OUTCOME]   centred  (xy centre inside holder footprint):           {centred}", flush=True)
        print(f"[OUTCOME]   upright  (z span {span_z * 100:.2f} cm > 20 cm):           {upright}", flush=True)
        ok = inserted and centred and upright
        colour = bcolors.OKGREEN if ok else bcolors.FAIL
        print(f"{colour}[OUTCOME] {'SUCCESS' if ok else 'FAILURE'}{bcolors.ENDC}", flush=True)

    def _update_disturbance_seq(self, stage, disturbance_seq):
        if disturbance_seq is not None:
            if stage in disturbance_seq and not self.applied_disturbance[stage]:
                # set the disturbance sequence, the generator will yield and instantiate one disturbance function for each env.step until it is exhausted
                self.env.disturbance_seq = disturbance_seq[stage](self.env)
                self.applied_disturbance[stage] = True

    def _execute(self, rekep_program_dir, disturbance_seq=None):
        # load metadata
        with open(os.path.join(rekep_program_dir, 'metadata.json'), 'r') as f:
            self.program_info = json.load(f)
        self.applied_disturbance = {stage: False for stage in range(1, self.program_info['num_stages'] + 1)}
        # register keypoints to be tracked
        self.env.register_keypoints(self.program_info['init_keypoint_positions'])
        # load constraints
        self.constraint_fns = dict()
        for stage in range(1, self.program_info['num_stages'] + 1):  # stage starts with 1
            stage_dict = dict()
            for constraint_type in ['subgoal', 'path']:
                load_path = os.path.join(rekep_program_dir, f'stage{stage}_{constraint_type}_constraints.txt')
                get_grasping_cost_fn = get_callable_grasping_cost_fn(self.env)  # special grasping function for VLM to call
                stage_dict[constraint_type] = load_functions_from_txt(load_path, get_grasping_cost_fn) if os.path.exists(load_path) else []
            self.constraint_fns[stage] = stage_dict
        
        # bookkeeping of which keypoints can be moved in the optimization
        self.keypoint_movable_mask = np.zeros(self.program_info['num_keypoints'] + 1, dtype=bool)
        self.keypoint_movable_mask[0] = True  # first keypoint is always the ee, so it's movable

        # main loop
        self.last_sim_step_counter = -np.inf
        self._update_stage(1)
        while True:
            scene_keypoints = self.env.get_keypoint_positions()
            self.keypoints = np.concatenate([[self.env.get_ee_pos()], scene_keypoints], axis=0)  # first keypoint is always the ee
            print(f"[DEBUG keypoints] stage={self.stage} ee={np.round(self.keypoints[0],4)} scene={np.round(scene_keypoints,4).tolist()} movable={self.keypoint_movable_mask.tolist()}", flush=True)
            self.curr_ee_pose = self.env.get_ee_pose()
            self.curr_joint_pos = self.env.get_arm_joint_postions()
            self.sdf_voxels = self.env.get_sdf_voxels(self.config['sdf_voxel_size'])
            self.collision_points = self.env.get_collision_points()
            # ====================================
            # = decide whether to backtrack
            # ====================================
            backtrack = False
            if self.stage > 1:
                path_constraints = self.constraint_fns[self.stage]['path']
                for constraints in path_constraints:
                    violation = constraints(self.keypoints[0], self.keypoints[1:])
                    if violation > self.config['constraint_tolerance']:
                        backtrack = True
                        break
            if backtrack:
                # determine which stage to backtrack to based on constraints
                for new_stage in range(self.stage - 1, 0, -1):
                    path_constraints = self.constraint_fns[new_stage]['path']
                    # if no constraints, we can safely backtrack
                    if len(path_constraints) == 0:
                        break
                    # otherwise, check if all constraints are satisfied
                    all_constraints_satisfied = True
                    for constraints in path_constraints:
                        violation = constraints(self.keypoints[0], self.keypoints[1:])
                        if violation > self.config['constraint_tolerance']:
                            all_constraints_satisfied = False
                            break
                    if all_constraints_satisfied:   
                        break
                print(f"{bcolors.HEADER}[stage={self.stage}] backtrack to stage {new_stage}{bcolors.ENDC}")
                self._update_stage(new_stage)
            else:
                # apply disturbance
                self._update_disturbance_seq(self.stage, disturbance_seq)
                # ====================================
                # = get optimized plan
                # ====================================
                if self.last_sim_step_counter == self.env.step_counter:
                    print(f"{bcolors.WARNING}sim did not step forward within last iteration (HINT: adjust action_steps_per_iter to be larger or the pos_threshold to be smaller){bcolors.ENDC}")
                next_subgoal = self._get_next_subgoal(from_scratch=self.first_iter)
                next_path = self._get_next_path(next_subgoal, from_scratch=self.first_iter)
                self.first_iter = False
                self.action_queue = next_path.tolist()
                self.last_sim_step_counter = self.env.step_counter

                # ====================================
                # = execute
                # ====================================
                # determine if we proceed to the next stage
                count = 0
                while len(self.action_queue) > 0 and count < self.config['action_steps_per_iter']:
                    next_action = self.action_queue.pop(0)
                    precise = len(self.action_queue) == 0
                    self.env.execute_action(next_action, precise=precise)
                    count += 1
                if len(self.action_queue) == 0:
                    if self.is_grasp_stage:
                        self._execute_grasp_action()
                    elif self.is_release_stage:
                        self._execute_release_action()
                    # if completed, save video and return
                    if self.stage == self.program_info['num_stages']:
                        self.env.sleep(2.0)
                        self.env.report_step_timing()  # [STEP TIMING] chapter 20, remove with ch.7
                        self._report_outcome()
                        save_path = self.env.save_video()
                        print(f"{bcolors.OKGREEN}Video saved to {save_path}\n\n{bcolors.ENDC}")
                        return
                    # progress to next stage
                    self._update_stage(self.stage + 1)

    def _get_next_subgoal(self, from_scratch):
        subgoal_constraints = self.constraint_fns[self.stage]['subgoal']
        path_constraints = self.constraint_fns[self.stage]['path']
        subgoal_pose, debug_dict = self.subgoal_solver.solve(self.curr_ee_pose,
                                                            self.keypoints,
                                                            self.keypoint_movable_mask,
                                                            subgoal_constraints,
                                                            path_constraints,
                                                            self.sdf_voxels,
                                                            self.collision_points,
                                                            self.is_grasp_stage,
                                                            self.curr_joint_pos,
                                                            from_scratch=from_scratch)
        subgoal_pose_homo = T.convert_pose_quat2mat(subgoal_pose)
        # if grasp stage, back up a bit to leave room for grasping.
        #
        # Upstream backs off by grasp_depth/2 here and advances by grasp_depth in
        # _execute_grasp_action, so the gripper lands grasp_depth/2 PAST the pose the solver
        # actually returned. Along upstream's local X that overshoot slides along the pen's long
        # axis and is harmless (arguably deliberate: it seats the pen between the fingers). Along
        # local Z -- the real approach axis, see subgoal_solver.py:71 -- the same overshoot drives
        # the gripper into the tabletop. See config.yaml grasp_approach_axis / grasp_standoff_ratio.
        if self.is_grasp_stage:
            _col = {'x': 0, 'z': 2}[self.config.get('grasp_approach_axis', 'x')]
            _ratio = self.config.get('grasp_standoff_ratio', 0.5)
            _standoff = np.zeros(3)
            _standoff[_col] = -self.config['grasp_depth'] * _ratio
            subgoal_pose[:3] += subgoal_pose_homo[:3, :3] @ _standoff
        debug_dict['stage'] = self.stage
        print_opt_debug_dict(debug_dict)
        if self.visualize:
            self.visualizer.visualize_subgoal(subgoal_pose)
        return subgoal_pose

    def _get_next_path(self, next_subgoal, from_scratch):
        path_constraints = self.constraint_fns[self.stage]['path']
        path, debug_dict = self.path_solver.solve(self.curr_ee_pose,
                                                    next_subgoal,
                                                    self.keypoints,
                                                    self.keypoint_movable_mask,
                                                    path_constraints,
                                                    self.sdf_voxels,
                                                    self.collision_points,
                                                    self.curr_joint_pos,
                                                    from_scratch=from_scratch)
        print_opt_debug_dict(debug_dict)
        processed_path = self._process_path(path)
        if self.visualize:
            self.visualizer.visualize_path(processed_path)
        return processed_path

    def _process_path(self, path):
        # spline interpolate the path from the current ee pose
        full_control_points = np.concatenate([
            self.curr_ee_pose.reshape(1, -1),
            path,
        ], axis=0)
        num_steps = get_linear_interpolation_steps(full_control_points[0], full_control_points[-1],
                                                    self.config['interpolate_pos_step_size'],
                                                    self.config['interpolate_rot_step_size'])
        dense_path = spline_interpolate_poses(full_control_points, num_steps)
        # add gripper action
        ee_action_seq = np.zeros((dense_path.shape[0], 8))
        ee_action_seq[:, :7] = dense_path
        ee_action_seq[:, 7] = self.env.get_gripper_null_action()
        return ee_action_seq

    def _update_stage(self, stage):
        self.env.report_step_timing(tag=f'entering stage {stage}')  # [STEP TIMING] ch.20
        # update stage
        self.stage = stage
        self.is_grasp_stage = self.program_info['grasp_keypoints'][self.stage - 1] != -1
        self.is_release_stage = self.program_info['release_keypoints'][self.stage - 1] != -1
        # can only be grasp stage or release stage or none
        assert self.is_grasp_stage + self.is_release_stage <= 1, "Cannot be both grasp and release stage"
        if self.is_grasp_stage:  # ensure gripper is open for grasping stage
            self.env.open_gripper()
        # clear action queue
        self.action_queue = []
        # update keypoint movable mask
        self._update_keypoint_movable_mask()
        self.first_iter = True

    def _update_keypoint_movable_mask(self):
        for i in range(1, len(self.keypoint_movable_mask)):  # first keypoint is ee so always movable
            keypoint_object = self.env.get_object_by_keypoint(i - 1)
            self.keypoint_movable_mask[i] = self.env.is_grasping(keypoint_object)

    def _execute_grasp_action(self):
        pregrasp_pose = self.env.get_ee_pose()
        grasp_pose = pregrasp_pose.copy()
        # Local X, upstream's axis, KEPT ON PURPOSE even though it is the wrong one.
        #
        # subgoal_solver.py:71 had to move from column 0 to column 2 because in this OmniGibson
        # the eef frame's z-axis is the approach axis (chapter 11). By the same argument this
        # line and the standoff in _get_next_subgoal should use z too, and they were changed to
        # z and measured. The result:
        #
        #   - the grasp itself got strictly better: one attempt instead of three, backtracks
        #     5 -> 0, fingers closing symmetrically, ag_obj_in_hand=pen_1 on both the cached and
        #     the live constraint sets
        #   - everything after the grasp got worse: OSC pose not reached 4 -> 23 -> 87, the
        #     trunk pinned at 0.3862, Jacobian condition numbers of 2945 / 4416 / 7722, and the
        #     arm wandering to y = -0.70 with the holder at y = +0.17
        #
        # The reason is geometric. Along x the advance slides the gripper ~5 cm sideways at the
        # pregrasp height and never descends; along z it drives the gripper down onto the
        # tabletop, and holding a pen from that low, extended posture is what pushes the trunk to
        # its stop and the arm into the singularities above. fd397a5 completed this task partly
        # *because* of this bug: it kept the arm high.
        #
        # So the correct axis is known and is not used. What makes the wrong one survive is that
        # local X points along the pen's long axis, so the 5 cm slide stays on the pen -- for a
        # grasp keypoint near the middle. It fails as soon as the keypoint moves toward an end,
        # which is exactly what the live VLM query produced (keypoint 10% along the pen; the
        # slide runs off the end and knocks it away).
        #
        # Fixing this properly means retuning the execution layer, not this line: subgoal_solver's
        # ik_cost carries weight 20.0 against init_pose_cost's 1.0, so the planner will trade
        # large backward motions for slightly easier IK, and nothing corrects the drift on the
        # next iteration. Recorded rather than attempted.
        _axis = self.config.get('grasp_approach_axis', 'x')
        _col = {'x': 0, 'z': 2}[_axis]
        _advance = np.zeros(3)
        _advance[_col] = self.config['grasp_depth']
        grasp_pose[:3] += T.quat2mat(pregrasp_pose[3:]) @ _advance
        # [GRASP AXES] stays: it is what settled the above, and it fires once per grasp.
        _R = T.quat2mat(pregrasp_pose[3:])
        _ratio = self.config.get('grasp_standoff_ratio', 0.5)
        print(f"[GRASP AXES] axis={_axis} standoff_ratio={_ratio} "
              f"net_overshoot={self.config['grasp_depth'] * (1.0 - _ratio):.4f} "
              f"pregrasp={np.round(pregrasp_pose[:3], 4)} "
              f"localX->world={np.round(_R[:, 0], 3)} localZ->world={np.round(_R[:, 2], 3)} "
              f"applied_delta={np.round(grasp_pose[:3] - pregrasp_pose[:3], 4)}",
              flush=True)
        grasp_action = np.concatenate([grasp_pose, [self.env.get_gripper_close_action()]])
        # precise=True is required, not merely tidy. The advance is deliberately aimed *through*
        # the object, and for a pen on a table that means through the tabletop as well, so the
        # move never converges: the gripper bottoms out at z=0.725 while the (clipped) target is
        # 0.698, and _move_to_waypoint burns its full 120 steps on a 3 cm error it cannot close.
        # That is wasteful but harmless. Relaxing it to precise=False is not: the threshold goes
        # from 3 cm to 10 cm, the 8.5 cm standoff already counts as "reached", and the gripper
        # closes in mid-air without descending at all (measured: eef z 0.7827 -> 0.7816,
        # contacts=[]). Leave the wasted steps alone unless the depth itself is reworked.
        self.env.execute_action(grasp_action, precise=True)
    
    def _execute_release_action(self):
        self.env.open_gripper()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--task', type=str, default='pen', help='task to perform')
    parser.add_argument('--use_cached_query', action='store_true', help='instead of querying the VLM, use the cached query')
    parser.add_argument('--program_dir', default=None,
                        help='directory of constraints to execute, overriding the task default. '
                             'Lets repeated runs compare two constraint sets without editing this '
                             'file between them -- which matters because run-to-run variation is '
                             'large enough that single runs cannot separate the two')
    parser.add_argument('--apply_disturbance', action='store_true', help='apply disturbance to test the robustness')
    parser.add_argument('--visualize', action='store_true', help='visualize each solution before executing (NOTE: this is blocking and needs to press "ESC" to continue)')
    parser.add_argument('--config', default='./configs/config.yaml',
                        help='config file to run with. The default is the Fetch setup every '
                             'result in chapters 1-20 was measured on; ./configs/config_widowxai.yaml '
                             'swaps in the WidowX AI. Kept as a flag rather than an edit so both '
                             'robots can be run from the same checkout without touching the baseline')
    args = parser.parse_args()

    if args.apply_disturbance:
        assert args.task == 'pen' and args.use_cached_query, 'disturbance sequence is only defined for cached scenario'

    # ====================================
    # = pen task disturbance sequence
    # ====================================
    def stage1_disturbance_seq(env):
        """
        Move the pen in stage 0 when robot is trying to grasp the pen
        """
        pen = env.og_env.scene.object_registry("name", "pen_1")
        holder = env.og_env.scene.object_registry("name", "pencil_holder_1")
        # disturbance sequence
        pos0, orn0 = pen.get_position_orientation()
        pose0 = np.concatenate([pos0, orn0])
        pos1 = pos0 + np.array([-0.08, 0.0, 0.0])
        orn1 = T.quat_multiply(T.euler2quat(np.array([0, 0, np.pi/4])), orn0)
        pose1 = np.concatenate([pos1, orn1])
        pos2 = pos1 + np.array([0.10, 0.0, 0.0])
        orn2 = T.quat_multiply(T.euler2quat(np.array([0, 0, -np.pi/2])), orn1)
        pose2 = np.concatenate([pos2, orn2])
        control_points = np.array([pose0, pose1, pose2])
        pose_seq = spline_interpolate_poses(control_points, num_steps=25)
        def disturbance(counter):
            if counter < len(pose_seq):
                pose = pose_seq[counter]
                pos, orn = pose[:3], pose[3:]
                pen.set_position_orientation(pos, orn)
                counter += 1
        counter = 0
        while True:
            yield disturbance(counter)
            counter += 1
    
    def stage2_disturbance_seq(env):
        """
        Take the pen out of the gripper in stage 1 when robot is trying to reorient the pen
        """
        apply_disturbance = env.is_grasping()
        pen = env.og_env.scene.object_registry("name", "pen_1")
        holder = env.og_env.scene.object_registry("name", "pencil_holder_1")
        # disturbance sequence
        pos0, orn0 = pen.get_position_orientation()
        pose0 = np.concatenate([pos0, orn0])
        pose1 = np.array([-0.30, -0.15, 0.71, -0.7071068, 0, 0, 0.7071068])
        control_points = np.array([pose0, pose1])
        pose_seq = spline_interpolate_poses(control_points, num_steps=25)
        def disturbance(counter):
            if apply_disturbance:
                if counter < 20:
                    if counter > 15:
                        env.robot.release_grasp_immediately()  # force robot to release the pen
                    else:
                        pass  # do nothing for the other steps
                elif counter < len(pose_seq) + 20:
                    env.robot.release_grasp_immediately()  # force robot to release the pen
                    pose = pose_seq[counter - 20]
                    pos, orn = pose[:3], pose[3:]
                    pen.set_position_orientation(pos, orn)
                    counter += 1
        counter = 0
        while True:
            yield disturbance(counter)
            counter += 1
    
    def stage3_disturbance_seq(env):
        """
        Move the holder in stage 2 when robot is trying to drop the pen into the holder
        """
        pen = env.og_env.scene.object_registry("name", "pen_1")
        holder = env.og_env.scene.object_registry("name", "pencil_holder_1")
        # disturbance sequence
        pos0, orn0 = holder.get_position_orientation()
        pose0 = np.concatenate([pos0, orn0])
        pos1 = pos0 + np.array([-0.02, -0.15, 0.0])
        orn1 = orn0
        pose1 = np.concatenate([pos1, orn1])
        control_points = np.array([pose0, pose1])
        pose_seq = spline_interpolate_poses(control_points, num_steps=5)
        def disturbance(counter):
            if counter < len(pose_seq):
                pose = pose_seq[counter]
                pos, orn = pose[:3], pose[3:]
                holder.set_position_orientation(pos, orn)
                counter += 1
        counter = 0
        while True:
            yield disturbance(counter)
            counter += 1

    task_list = {
        'pen': {
            'scene_file': './configs/og_scene_file_pen.json',
            'instruction': 'reorient the white pen and drop it upright into the black pen holder',
            'rekep_program_dir': './vlm_query/pen',  # the author's 2024 cached query
            'disturbance_seq': {1: stage1_disturbance_seq, 2: stage2_disturbance_seq, 3: stage3_disturbance_seq},
            },
    }
    task = task_list['pen']
    scene_file = task['scene_file']
    instruction = task['instruction']
    main = Main(scene_file, visualize=args.visualize, config_path=args.config)
    main.perform_task(instruction,
                    rekep_program_dir=(args.program_dir or task['rekep_program_dir']) if args.use_cached_query else None,
                    disturbance_seq=task.get('disturbance_seq', None) if args.apply_disturbance else None)