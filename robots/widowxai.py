"""The Trossen WidowX AI Follower -- the arm this research will actually use on hardware.

Lives in the ReKep repo rather than under `omnigibson/robots/` on purpose. The Fetch stack that
chapters 1-20 were built on has to keep working untouched, so nothing in the BEHAVIOR-1K checkout
is modified; OmniGibson's own tutorial sanctions this, saying a robot class may simply be
imported "from your python module at the top of the file" in the end-use script. Registration
happens on import, which `main.py` does.

Modelled on `omnigibson/robots/vx300s.py`, the ViperX 300 S -- same manufacturer, same 6-DOF
fixed-base family, and the closest thing OmniGibson already ships. The differences from it are
the ones that matter here:

  - The wrist is pitch-yaw-roll, not roll-pitch-roll. Its singularity is at joint_4 = +/-90 deg,
    which is exactly joint_4's travel limit, so unlike Fetch there is no hole in the middle of
    the workspace. Measured in docs/sim_platform_decision.md section 5.
  - There is a RealSense D405 on the wrist, which the ViperX does not have.

Asset provenance, the CoACD decision, and the derivation of the end-effector frame rotation are
all in assets/widowxai/README.md. The USD this class loads is produced by
`tools/import_widowxai.py` and must sit at
`<gm.DATA_PATH>/omnigibson-robot-assets/models/widowxai/usd/widowxai.usda` -- that path is
derived from the class name, so renaming this class renames the asset directory it looks for.
"""
import math
from functools import cached_property

import torch as th

from omnigibson.robots.manipulation_robot import ManipulationRobot
from omnigibson.utils.transform_utils import euler2quat


class WidowXAI(ManipulationRobot):
    """
    The WidowX AI Follower, a 6-DOF fixed-base arm from Trossen Robotics.
    Reference: https://www.trossenrobotics.com/widowx-ai
    """

    def __init__(
        self,
        # Shared kwargs in hierarchy
        name,
        relative_prim_path=None,
        scale=None,
        visible=True,
        visual_only=False,
        self_collisions=True,
        link_physics_materials=None,
        load_config=None,
        fixed_base=True,
        # Unique to USDObject hierarchy
        abilities=None,
        # Unique to ControllableObject hierarchy
        control_freq=None,
        controller_config=None,
        action_type="continuous",
        action_normalize=True,
        reset_joint_pos=None,
        # Unique to BaseRobot
        obs_modalities=("rgb", "proprio"),
        include_sensor_names=None,
        exclude_sensor_names=None,
        proprio_obs="default",
        sensor_config=None,
        # Unique to ManipulationRobot
        grasping_mode="assisted",
        finger_static_friction=None,
        finger_dynamic_friction=None,
        **kwargs,
    ):
        """
        Argument semantics are identical to every other OmniGibson robot; see
        `ManipulationRobot.__init__` for the full descriptions. Only the defaults that differ
        from the base class are worth noting here:

            fixed_base (bool): True. The WidowX AI is bolted down -- there is no base to drive.
            grasping_mode (str): "assisted". ReKep requires this: environment.py asserts the mode
                is assisted or sticky, because get_sdf_voxels() reads `_ag_obj_in_hand` to exclude
                the carried object from the scene SDF. The grasp start/end ray points are not
                hand-authored -- ManipulationRobot derives them from the finger link geometry.
        """
        super().__init__(
            relative_prim_path=relative_prim_path,
            name=name,
            scale=scale,
            visible=visible,
            fixed_base=fixed_base,
            visual_only=visual_only,
            self_collisions=self_collisions,
            link_physics_materials=link_physics_materials,
            load_config=load_config,
            abilities=abilities,
            control_freq=control_freq,
            controller_config=controller_config,
            action_type=action_type,
            action_normalize=action_normalize,
            reset_joint_pos=reset_joint_pos,
            obs_modalities=obs_modalities,
            include_sensor_names=include_sensor_names,
            exclude_sensor_names=exclude_sensor_names,
            proprio_obs=proprio_obs,
            sensor_config=sensor_config,
            grasping_mode=grasping_mode,
            finger_static_friction=finger_static_friction,
            finger_dynamic_friction=finger_dynamic_friction,
            **kwargs,
        )

    @property
    def discrete_action_list(self):
        raise NotImplementedError()

    def _create_discrete_action_space(self):
        raise ValueError("WidowXAI does not support discrete actions!")

    @property
    def _raw_controller_order(self):
        return [f"arm_{self.default_arm}", f"gripper_{self.default_arm}"]

    @property
    def _default_controllers(self):
        controllers = super()._default_controllers
        # IK, not OSC. This is the whole point of moving off Fetch: chapters 16-18 traced the
        # flailing to the operational-space inertia matrix blowing up near a singularity, and
        # the real arm is commanded through IK anyway. Note OmniGibson's IK controller is an
        # undamped pseudo-inverse (ik_controller.py:409) whereas Trossen's own controllers use
        # damped least squares -- see docs/sim_platform_decision.md section 6.2.
        controllers[f"arm_{self.default_arm}"] = "InverseKinematicsController"
        controllers[f"gripper_{self.default_arm}"] = "MultiFingerGripperController"
        return controllers

    @property
    def _default_joint_pos(self):
        # Arm out in front with the gripper pointing straight down, chosen by search over the
        # kinematics rather than by eye. At this configuration the approach axis is within
        # 1.8 deg of world -Z (dot = 0.9995), the Jacobian condition number is 11.5, and the
        # tightest joint-limit margin is 0.291 rad (joint_3) -- so the solver's reset
        # regularisation pulls toward a pose that is both well conditioned and already oriented
        # for a top-down grasp. Reproduce with tools/analyze_wxai_kinematics.py in the parent repo.
        #
        # Trailing pair is the two gripper carriages, fully open (travel is 0 to 0.044 m). Both
        # take the same value, so the right-before-left ordering the URDF declares does not
        # matter here -- it would if these ever differed.
        return th.tensor([0.0, 1.38, 1.12, -1.28, 0.0, 0.0, 0.044, 0.044])

    @cached_property
    def arm_link_names(self):
        return {self.default_arm: ["base_link", "link_1", "link_2", "link_3", "link_4", "link_5", "link_6"]}

    @cached_property
    def arm_joint_names(self):
        return {self.default_arm: ["joint_0", "joint_1", "joint_2", "joint_3", "joint_4", "joint_5"]}

    @cached_property
    def eef_link_names(self):
        # Generated during import, not present in the source URDF. The URDF's own
        # `ee_gripper_link` sits at the same origin but with the approach axis on local X, which
        # is not OmniGibson's convention (z out of the fingertips, y from left finger to right)
        # and would invalidate the column index chapter 11 fixed in subgoal_solver.py. See
        # assets/widowxai/README.md.
        return {self.default_arm: "eef_link"}

    @cached_property
    def finger_link_names(self):
        # The pads, not the carriages they bolt to. These carry the contact geometry, and
        # ManipulationRobot measures them to place the assisted-grasp ray endpoints.
        return {self.default_arm: ["gripper_left", "gripper_right"]}

    @cached_property
    def finger_joint_names(self):
        # Prismatic, 0 to 0.044 m each; larger is more open.
        return {self.default_arm: ["left_carriage_joint", "right_carriage_joint"]}

    @property
    def teleop_rotation_offset(self):
        return {self.default_arm: euler2quat([-math.pi, 0, 0])}
