# @jacob-zietek
# train: ./isaaclab.sh -p source/standalone/workflows/rl_games/train.py --task Isaac-Valkyrie-Direct-v0 --headless
# inference: python3 source/standalone/workflows/rl_games/play.py --task Isaac-Valkyrie-Direct-v0 --num_envs 6

from __future__ import annotations

from omni.isaac.lab_assets import VALKYRIE_CFG, VALKYRIE_CFG_NO_HANDS, VALKYRIE_CFG_NO_HANDS_HIGH_VEL_LIMIT

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
    decimation = 4 # How many times per second the policy can make a decision
    action_scale = 0.15 # This should be 1.0. Stable w decimation 2 at 0.08
    #action_space = 59 # with hands and wrists
    #observation_space = 189 # with hands and wrists
    action_space = 33 # without hands and wrist
    observation_space = 111 # without hands and wrist
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
    )

    # robot
    robot: ArticulationCfg = VALKYRIE_CFG_NO_HANDS.replace(prim_path="/World/envs/env_.*/Robot")  # type: ignore
    # @jacob-zietek: I scaled this by the effort limit in the valkyrie actuator config.
    joint_gears: list = [
        190.0,  # torsoYaw
        150.0,  # torsoPitch
        150.0,  # torsoRoll
        26.0,  # lowerNeckPitch
        26.0,  # neckYaw
        26.0,  # upperNeckPitch
        1.0,  # hokuyo_joint
        190.0,  # rightHipYaw
        350.0,  # rightHipRoll
        350.0,  # rightHipPitch
        350.0,  # rightKneePitch
        205.0,  # rightAnklePitch
        205.0,  # rightAnkleRoll
        190.0,  # leftHipYaw
        350.0,  # leftHipRoll
        350.0,  # leftHipPitch
        350.0,  # leftKneePitch
        205.0,  # leftAnklePitch
        205.0,  # leftAnkleRoll
        350.0,  # rightShoulderPitch
        350.0,  # rightShoulderRoll
        65.0,  # rightShoulderYaw
        65.0,  # rightElbowPitch
        26.0,  # rightForearmYaw
        14.0,  # rightWristRoll
        14.0,  # rightWristPitch
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
        190.0,  # leftShoulderPitch
        350.0,  # leftShoulderRoll
        65.0,  # leftShoulderYaw
        65.0,  # leftElbowPitch
        26.0,  # leftForearmYaw
        14.0,  # leftWristRoll
        14.0,  # leftWristPitch
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
    termination_height: float = 0.6 # set to 0.95 if you want it to stand

    angular_velocity_scale: float = 0.25
    contact_force_scale: float = 0.01


class ValkyrieEnv(LocomotionEnv):
    cfg: ValkyrieEnvCfg

    def __init__(self, cfg: ValkyrieEnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)
