# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Configuration for Unitree robots.

The following configurations are available:

* :obj:`UNITREE_A1_CFG`: Unitree A1 robot with DC motor model for the legs
* :obj:`UNITREE_GO1_CFG`: Unitree Go1 robot with actuator net model for the legs
* :obj:`UNITREE_GO2_CFG`: Unitree Go2 robot with DC motor model for the legs
* :obj:`H1_CFG`: H1 humanoid robot
* :obj:`H1_MINIMAL_CFG`: H1 humanoid robot with minimal collision bodies
* :obj:`G1_CFG`: G1 humanoid robot
* :obj:`G1_MINIMAL_CFG`: G1 humanoid robot with minimal collision bodies

Reference: https://github.com/unitreerobotics/unitree_ros
"""

import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.actuators import ActuatorNetMLPCfg, DCMotorCfg, ImplicitActuatorCfg
from omni.isaac.lab.assets.articulation import ArticulationCfg
from omni.isaac.lab.utils.assets import ISAACLAB_NUCLEUS_DIR, ISAAC_NUCLEUS_DIR

import os

# get current path, remove everything after "personaisaaclab"
root_folder_path = os.getcwd().split("PersonaIsaacLab")[0] + "PersonaIsaacLab/"

VALKYRIE_CFG = ArticulationCfg(
    spawn=sim_utils.UrdfFileCfg(
        fix_base=True,
        merge_fixed_joints=False,
        make_instanceable=False,
        # TODO make this prettier somehow.
        asset_path=root_folder_path + "external/valkyrie/src/main/resources/models/val_description/urdf/valkyrie_sim.urdf",
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True, 
            solver_position_iteration_count=4, 
            solver_velocity_iteration_count=0
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.74),
        joint_pos={
            ".*HipPitch": -0.20,
            ".*KneePitch": 0.42,
            ".*AnklePitch": -0.23,
            ".*ElbowPitch": 0.11,
            "leftShoulderRoll": 0.16,
            "leftShoulderPitch": 0.35,
            "rightShoulderRoll": -0.16,
            "rightShoulderPitch": 0.35,
            "rightIndexFingerPitch1": 1.0,
            "leftIndexFingerPitch1": -0.5,
        },
        joint_vel={".*": 0.0},
    ),
    soft_joint_pos_limit_factor=0.9,
    actuators={
        "legs": ImplicitActuatorCfg(
            joint_names_expr=[
                ".*HipYaw",
                ".*HipRoll",
                ".*HipPitch",
                ".*KneePitch",
                "torsoPitch",
                "torsoRoll",
                "torsoYaw",
            ],
            effort_limit=300,
            velocity_limit=100.0,
            stiffness={
                ".*HipYaw": 150.0,
                ".*HipRoll": 150.0,
                ".*HipPitch": 200.0,
                ".*KneePitch": 200.0,
                "torsoPitch": 200.0,
                "torsoRoll": 200.0,
                "torsoYaw": 200.0,
            },
            damping={
                ".*HipYaw": 5.0,
                ".*HipRoll": 5.0,
                ".*HipPitch": 5.0,
                ".*KneePitch": 5.0,
                "torsoPitch": 5.0,
                "torsoRoll": 5.0,
                "torsoYaw": 5.0,
            },
            armature={
                ".*Hip.*": 0.01,
                ".*KneePitch": 0.01,
                "torsoPitch": 0.01,
                "torsoRoll": 0.01,
                "torsoYaw": 0.01,
            },
        ),
        "feet": ImplicitActuatorCfg(
            effort_limit=20,
            joint_names_expr=[".*AnkleRoll", ".*AnklePitch"],
            stiffness=20.0,
            damping=2.0,
            armature=0.01,
        ),
        "arms": ImplicitActuatorCfg(
            joint_names_expr=[
                ".*ShoulderPitch",
                ".*ShoulderRoll",
                ".*ShoulderYaw",
                ".*ElbowPitch",
                # ".*_five_joint",
                # ".*_three_joint",
                # ".*_six_joint",
                # ".*_four_joint",
                # ".*_zero_joint",
                # ".*_one_joint",
                # ".*_two_joint",
            ],
            effort_limit=300,
            velocity_limit=100.0,
            stiffness=40.0,
            damping=10.0,
            armature={
                ".*Shoulder.*": 0.01,
                ".*Elbow.*": 0.01,
                # ".*_five_joint": 0.001,
                # ".*_three_joint": 0.001,
                # ".*_six_joint": 0.001,
                # ".*_four_joint": 0.001,
                # ".*_zero_joint": 0.001,
                # ".*_one_joint": 0.001,
                # ".*_two_joint": 0.001,
            },
        ),
    },
)
