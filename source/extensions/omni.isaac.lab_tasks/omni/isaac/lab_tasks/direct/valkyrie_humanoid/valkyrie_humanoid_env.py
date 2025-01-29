# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from __future__ import annotations

from omni.isaac.lab_assets import VALKYRIE_CFG, VALKYRIE_CFG_NO_HAND_ACTUATOR

import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.assets import ArticulationCfg
from omni.isaac.lab.envs import DirectRLEnvCfg
from omni.isaac.lab.scene import InteractiveSceneCfg
from omni.isaac.lab.sim import SimulationCfg
from omni.isaac.lab.terrains import TerrainImporterCfg
from omni.isaac.lab.utils import configclass

from omni.isaac.lab_tasks.direct.locomotion.locomotion_env import LocomotionEnv


@configclass
class ValkyrieEnvCfg(DirectRLEnvCfg):
    # env
    episode_length_s = 15.0
    decimation = 2
    action_scale = 1.0
    #action_space = 59 # with hands and wrists
    #observation_space = 189 # with hands and wrists
    action_space = 29 # without hands and wrist
    observation_space = 159 # without hands and wrist
    state_space = 0

    # simulation
    sim: SimulationCfg = SimulationCfg(dt=1 / 120, render_interval=decimation)
    terrain = TerrainImporterCfg(
        prim_path="/World/ground",
        terrain_type="plane",
        collision_group=-1,
        physics_material=sim_utils.RigidBodyMaterialCfg(
            friction_combine_mode="average",
            restitution_combine_mode="average",
            static_friction=1.0,
            dynamic_friction=1.0,
            restitution=0.0,
        ),
        debug_vis=False,
    )

    # scene
    scene: InteractiveSceneCfg = InteractiveSceneCfg(
        #num_envs=4096, env_spacing=4.0, replicate_physics=True
        num_envs=1024, env_spacing=4.0, replicate_physics=True
        #num_envs=2048, env_spacing=4.0, replicate_physics=True
    )

    # robot
    robot: ArticulationCfg = VALKYRIE_CFG_NO_HAND_ACTUATOR.replace(prim_path="/World/envs/env_.*/Robot")  # type: ignore
    # joint_gears: list = [
    #     67.5000,  # lower_waist
    #     67.5000,  # lower_waist
    #     67.5000,  # right_upper_arm
    #     67.5000,  # right_upper_arm
    #     67.5000,  # left_upper_arm
    #     67.5000,  # left_upper_arm
    #     67.5000,  # pelvis
    #     45.0000,  # right_lower_arm
    #     45.0000,  # left_lower_arm
    #     45.0000,  # right_thigh: x
    #     135.0000,  # right_thigh: y
    #     45.0000,  # right_thigh: z
    #     45.0000,  # left_thigh: x
    #     135.0000,  # left_thigh: y
    #     45.0000,  # left_thigh: z
    #     90.0000,  # right_knee
    #     90.0000,  # left_knee
    #     22.5,  # right_foot
    #     22.5,  # right_foot
    #     22.5,  # left_foot
    #     22.5,  # left_foot
    # ]
    # @jacob-zietek: I have no clue if this is right, so I set it to the default values.
    # This is as if each joint is directly connected to the motor.
    joint_gears: list = [
        1.0,  # torsoYaw
        1.0,  # torsoPitch
        1.0,  # torsoRoll
        1.0,  # lowerNeckPitch
        1.0,  # neckYaw
        1.0,  # upperNeckPitch
        1.0,  # hokuyo_joint
        1.0,  # rightHipYaw
        1.0,  # rightHipRoll
        1.0,  # rightHipPitch
        1.0,  # rightKneePitch
        1.0,  # rightAnklePitch
        1.0,  # rightAnkleRoll
        1.0,  # leftHipYaw
        1.0,  # leftHipRoll
        1.0,  # leftHipPitch
        1.0,  # leftKneePitch
        1.0,  # leftAnklePitch
        1.0,  # leftAnkleRoll
        1.0,  # rightShoulderPitch
        1.0,  # rightShoulderRoll
        1.0,  # rightShoulderYaw
        1.0,  # rightElbowPitch
        1.0,  # rightForearmYaw
        # 1.0,  # rightWristRoll
        # 1.0,  # rightWristPitch
        # 1.0,  # rightThumbRoll
        # 1.0,  # rightThumbPitch1
        # 1.0,  # rightThumbPitch2
        # 1.0,  # rightThumbPitch3
        # 1.0,  # rightIndexFingerPitch1
        # 1.0,  # rightIndexFingerPitch2
        # 1.0,  # rightIndexFingerPitch3
        # 1.0,  # rightMiddleFingerPitch1
        # 1.0,  # rightMiddleFingerPitch2
        # 1.0,  # rightMiddleFingerPitch3
        # 1.0,  # rightPinkyPitch1
        # 1.0,  # rightPinkyPitch2
        # 1.0,  # rightPinkyPitch3
        1.0,  # leftShoulderPitch
        1.0,  # leftShoulderRoll
        1.0,  # leftShoulderYaw
        1.0,  # leftElbowPitch
        1.0,  # leftForearmYaw
        # 1.0,  # leftWristRoll
        # 1.0,  # leftWristPitch
        # 1.0,  # leftThumbRoll
        # 1.0,  # leftThumbPitch1
        # 1.0,  # leftThumbPitch2
        # 1.0,  # leftThumbPitch3
        # 1.0,  # leftIndexFingerPitch1
        # 1.0,  # leftIndexFingerPitch2
        # 1.0,  # leftIndexFingerPitch3
        # 1.0,  # leftMiddleFingerPitch1
        # 1.0,  # leftMiddleFingerPitch2
        # 1.0,  # leftMiddleFingerPitch3
        # 1.0,  # leftPinkyPitch1
        # 1.0,  # leftPinkyPitch2
        # 1.0,  # leftPinkyPitch3
    ]

    heading_weight: float = 0.5
    up_weight: float = 0.1

    energy_cost_scale: float = 0.05
    actions_cost_scale: float = 0.01
    alive_reward_scale: float = 2.0
    dof_vel_scale: float = 0.1

    death_cost: float = -1.0
    termination_height: float = 0.8

    angular_velocity_scale: float = 0.25
    contact_force_scale: float = 0.01


class ValkyrieEnv(LocomotionEnv):
    cfg: ValkyrieEnvCfg

    def __init__(self, cfg: ValkyrieEnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)
