from mani_skill.agents.robots import UnitreeGo2, ANYmalC # imports your robot and registers it
# imports the demo_robot example script and lets you test your new robot
import mani_skill.examples.demo_robot as demo_robot_script
# TODO (stao): Anymal may not be modelled correctly or efficiently at the moment
import numpy as np
import sapien
import torch

from mani_skill import ASSET_DIR
from mani_skill.agents.base_agent import BaseAgent, Keyframe
from mani_skill.agents.controllers import *
from mani_skill.agents.registration import register_agent
from mani_skill.utils import sapien_utils
from mani_skill.utils.structs.articulation import Articulation


@register_agent()
class Unitree_z1(BaseAgent):
    uid = "unitree_z1"
    urdf_path = f"{ASSET_DIR}/robots/unitree_z1/xacro/z1.urdf"

    fix_root_link = True
    disable_self_collisions = True

    keyframes = dict(
        standing=Keyframe(
            pose=sapien.Pose(p=[0, 0, 0.545]),
            qpos=np.array(
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            ),
        )
    )

    joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    @property
    def _controller_configs(self):
        self.arm_stiffness = 80.0
        self.arm_damping = 2.0
        self.arm_force_limit = 100
        print([i.name for i in self.robot.active_joints])
        # delta action scale for Omni Isaac Gym Envs is self.dt * self.action_scale = 1/60 * 13.5. NOTE that their self.dt value is not the same as the actual DT used in sim...., they use default of 1/100
        pd_joint_delta_pos = PDJointPosControllerConfig(
            self.joint_names,
            -0.225,
            0.225,
            self.arm_stiffness,
            self.arm_damping,
            self.arm_force_limit,
            normalize_action=True,
            use_delta=True,
        )
        pd_joint_pos = PDJointPosControllerConfig(
            self.joint_names,
            None,
            None,
            self.arm_stiffness,
            self.arm_damping,
            self.arm_force_limit,
            normalize_action=False,
            use_delta=False,
        )
        # TODO (stao): For quadrupeds perhaps we disable gravit for all links except the root?
        controller_configs = dict(
            pd_joint_delta_pos=dict(
                body=pd_joint_delta_pos, balance_passive_force=False
            ),
            pd_joint_pos=dict(body=pd_joint_pos, balance_passive_force=False),
        )
        return controller_configs

    def _after_init(self):
        # disable gravity / compensate gravity automatically in all links but the root one
        for link in self.robot.links[1:]:
            link.disable_gravity = True

@register_agent()
class Unitree_go2_z1(BaseAgent):
    uid = "unitree_go2_z1"
    urdf_path = f"{ASSET_DIR}/robots/unitree_go2_z1/urdf/go2_z1_description.urdf"

    fix_root_link = True
    disable_self_collisions = True

    keyframes = dict(
        standing=Keyframe(
            pose=sapien.Pose(p=[0, 0, 0.29]),
            qpos=np.array(
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.9, 0.9, 0.9, 0.9, -1.8, -1.8, -1.8, -1.8, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            ),
        )
    )

    joint_names = ['FL_hip_joint', 'FR_hip_joint', 'RL_hip_joint', 'RR_hip_joint', 
                   'go2_z1_joint',
                   'FL_thigh_joint', 'FR_thigh_joint', 'RL_thigh_joint', 'RR_thigh_joint', 
                   'FL_calf_joint', 'FR_calf_joint', 'RL_calf_joint', 'RR_calf_joint', 
                   'z1_joint1', 'z1_joint2', 'z1_joint3', 'z1_joint4', 
                   'z1_joint5', 'z1_joint6']

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    @property
    def _controller_configs(self):
        self.arm_stiffness = 80.0
        self.arm_damping = 2.0
        self.arm_force_limit = 100
        # delta action scale for Omni Isaac Gym Envs is self.dt * self.action_scale = 1/60 * 13.5. NOTE that their self.dt value is not the same as the actual DT used in sim...., they use default of 1/100
        print([i.name for i in self.robot.active_joints])
        pd_joint_delta_pos = PDJointPosControllerConfig(
            self.joint_names,
            -0.225,
            0.225,
            self.arm_stiffness,
            self.arm_damping,
            self.arm_force_limit,
            normalize_action=True,
            use_delta=True,
        )
        pd_joint_pos = PDJointPosControllerConfig(
            self.joint_names,
            None,
            None,
            self.arm_stiffness,
            self.arm_damping,
            self.arm_force_limit,
            normalize_action=False,
            use_delta=False,
        )
        # TODO (stao): For quadrupeds perhaps we disable gravit for all links except the root?
        controller_configs = dict(
            pd_joint_delta_pos=dict(
                body=pd_joint_delta_pos, balance_passive_force=False
            ),
            pd_joint_pos=dict(body=pd_joint_pos, balance_passive_force=False),
        )
        return controller_configs

    def _after_init(self):
        # disable gravity / compensate gravity automatically in all links but the root one
        for link in self.robot.links[1:]:
            link.disable_gravity = True


demo_robot_script.main()