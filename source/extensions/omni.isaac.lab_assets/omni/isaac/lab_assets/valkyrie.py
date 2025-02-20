"""Configuration for the Valkyrie robot from IHMC Robotics.

@jacob-zietek

Please make sure your github submodules are set up correctly for the PersonaIsaacLab repository.

This is taken from the unitree humanoids example. The VALKYRIE_CFG is the only config available.

Reference: https://github.com/unitreerobotics/unitree_ros
"""

import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.actuators import ActuatorNetMLPCfg, DCMotorCfg, ImplicitActuatorCfg
from omni.isaac.lab.assets.articulation import ArticulationCfg

import os

# get current path, remove everything after "personaisaaclab"
root_folder_path = os.getcwd().split("PersonaIsaacLab")[0] + "PersonaIsaacLab/"

VALKYRIE_CFG = ArticulationCfg(
    spawn=sim_utils.UrdfFileCfg(
        fix_base=False,
        merge_fixed_joints=False,
        make_instanceable=False,
        # TODO make this prettier somehow.
        asset_path=root_folder_path
        + "external/valkyrie/src/main/resources/models/val_description/urdf/valkyrie_sim.urdf",
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True,
            solver_position_iteration_count=4,
            solver_velocity_iteration_count=0,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 1.0),
        joint_vel={".*": 0.0},
    ),
    soft_joint_pos_limit_factor=0.9,
    actuators={
        "torsoYaw": ImplicitActuatorCfg(
            joint_names_expr=["torsoYaw"],
            effort_limit=190.0,
            velocity_limit=5.89,
            stiffness=0.0,
            damping=0.1,
        ),
        "torsoPitch": ImplicitActuatorCfg(
            joint_names_expr=["torsoPitch"],
            effort_limit=150.0,
            velocity_limit=9.0,
            stiffness=0.0,
            damping=0.1,
        ),
        "torsoRoll": ImplicitActuatorCfg(
            joint_names_expr=["torsoRoll"],
            effort_limit=150.0,
            velocity_limit=9.0,
            stiffness=0.0,
            damping=0.1,
        ),
        "lowerNeckPitch": ImplicitActuatorCfg(
            joint_names_expr=["lowerNeckPitch"],
            effort_limit=26.0,
            velocity_limit=5.0,
            stiffness=50.0,
            damping=0.1,
        ),
        "neckYaw": ImplicitActuatorCfg(
            joint_names_expr=["neckYaw"],
            effort_limit=26.0,
            velocity_limit=5.0,
            stiffness=50.0,
            damping=0.1,
        ),
        "upperNeckPitch": ImplicitActuatorCfg(
            joint_names_expr=["upperNeckPitch"],
            effort_limit=26.0,
            velocity_limit=5.0,
            stiffness=50.0,
            damping=0.1,
        ),
        "hokuyo_joint": ImplicitActuatorCfg(
            joint_names_expr=["hokuyo_joint"], stiffness=0.0, damping=0.01
        ),
        "rightHipYaw": ImplicitActuatorCfg(
            joint_names_expr=["rightHipYaw"],
            effort_limit=190.0,
            velocity_limit=5.89,
            stiffness=0.0,
            damping=0.1,
        ),
        "rightHipRoll": ImplicitActuatorCfg(
            joint_names_expr=["rightHipRoll"],
            effort_limit=350.0,
            velocity_limit=7.0,
            stiffness=0.0,
            damping=0.1,
        ),
        "rightHipPitch": ImplicitActuatorCfg(
            joint_names_expr=["rightHipPitch"],
            effort_limit=350.0,
            velocity_limit=6.11,
            stiffness=0.0,
            damping=0.1,
        ),
        "rightKneePitch": ImplicitActuatorCfg(
            joint_names_expr=["rightKneePitch"],
            effort_limit=350.0,
            velocity_limit=6.11,
            stiffness=0.0,
            damping=0.1,
        ),
        "rightAnklePitch": ImplicitActuatorCfg(
            joint_names_expr=["rightAnklePitch"],
            effort_limit=205.0,
            velocity_limit=11.0,
            stiffness=0.0,
            damping=0.1,
        ),
        "rightAnkleRoll": ImplicitActuatorCfg(
            joint_names_expr=["rightAnkleRoll"],
            effort_limit=205.0,
            velocity_limit=11.0,
            stiffness=0.0,
            damping=0.3,
        ),
        "leftHipYaw": ImplicitActuatorCfg(
            joint_names_expr=["leftHipYaw"],
            effort_limit=190.0,
            velocity_limit=5.89,
            stiffness=0.0,
            damping=0.1,
        ),
        "leftHipRoll": ImplicitActuatorCfg(
            joint_names_expr=["leftHipRoll"],
            effort_limit=350.0,
            velocity_limit=7.0,
            stiffness=0.0,
            damping=0.1,
        ),
        "leftHipPitch": ImplicitActuatorCfg(
            joint_names_expr=["leftHipPitch"],
            effort_limit=350.0,
            velocity_limit=6.11,
            stiffness=0.0,
            damping=0.1,
        ),
        "leftKneePitch": ImplicitActuatorCfg(
            joint_names_expr=["leftKneePitch"],
            effort_limit=350.0,
            velocity_limit=6.11,
            stiffness=0.0,
            damping=0.1,
        ),
        "leftAnklePitch": ImplicitActuatorCfg(
            joint_names_expr=["leftAnklePitch"],
            effort_limit=205.0,
            velocity_limit=11.0,
            stiffness=0.0,
            damping=0.1,
        ),
        "leftAnkleRoll": ImplicitActuatorCfg(
            joint_names_expr=["leftAnkleRoll"],
            effort_limit=205.0,
            velocity_limit=11.0,
            stiffness=0.0,
            damping=0.3,
        ),
        "rightShoulderPitch": ImplicitActuatorCfg(
            joint_names_expr=["rightShoulderPitch"],
            effort_limit=190.0,
            velocity_limit=3.0,
            stiffness=0.0,
            damping=0.1,
        ),
        "rightShoulderRoll": ImplicitActuatorCfg(
            joint_names_expr=["rightShoulderRoll"],
            effort_limit=350.0,
            velocity_limit=3.5,
            stiffness=0.0,
            damping=0.1,
        ),
        "rightShoulderYaw": ImplicitActuatorCfg(
            joint_names_expr=["rightShoulderYaw"],
            effort_limit=65.0,
            velocity_limit=1.5,
            stiffness=0.0,
            damping=0.1,
        ),
        "rightElbowPitch": ImplicitActuatorCfg(
            joint_names_expr=["rightElbowPitch"],
            effort_limit=65.0,
            velocity_limit=3.5,
            stiffness=0.0,
            damping=0.1,
        ),
        "rightForearmYaw": ImplicitActuatorCfg(
            joint_names_expr=["rightForearmYaw"],
            effort_limit=26.0,
            velocity_limit=0.8,
            stiffness=1000.0,
            damping=0.1,
        ),
        "rightWristRoll": ImplicitActuatorCfg(
            joint_names_expr=["rightWristRoll"],
            effort_limit=14.0,
            velocity_limit=1.0,
            stiffness=500.0,
            damping=0.1,
        ),
        "rightWristPitch": ImplicitActuatorCfg(
            joint_names_expr=["rightWristPitch"],
            effort_limit=14.0,
            velocity_limit=1.0,
            stiffness=500.0,
            damping=0.1,
        ),
        "rightThumbRoll": ImplicitActuatorCfg(
            joint_names_expr=["rightThumbRoll"],
            effort_limit=10.0,
            velocity_limit=1.0,
            stiffness=5.0,
            damping=0.5,
            armature=0.5,
        ),
        "rightThumbPitch1": ImplicitActuatorCfg(
            joint_names_expr=["rightThumbPitch1"],
            effort_limit=10.0,
            velocity_limit=1.0,
            stiffness=5.0,
            damping=0.5,
            armature=0.5,
        ),
        "rightThumbPitch2": ImplicitActuatorCfg(
            joint_names_expr=["rightThumbPitch2"],
            effort_limit=10.0,
            velocity_limit=1.0,
            stiffness=5.0,
            damping=0.5,
            armature=0.5,
        ),
        "rightThumbPitch3": ImplicitActuatorCfg(
            joint_names_expr=["rightThumbPitch3"],
            effort_limit=10.0,
            velocity_limit=1.0,
            stiffness=5.0,
            damping=0.5,
            armature=0.5,
        ),
        "rightIndexFingerPitch1": ImplicitActuatorCfg(
            joint_names_expr=["rightIndexFingerPitch1"],
            effort_limit=10.0,
            velocity_limit=1.0,
            stiffness=5.0,
            damping=0.5,
            armature=0.5,
        ),
        "rightIndexFingerPitch2": ImplicitActuatorCfg(
            joint_names_expr=["rightIndexFingerPitch2"],
            effort_limit=10.0,
            velocity_limit=1.0,
            stiffness=5.0,
            damping=0.5,
            armature=0.5,
        ),
        "rightIndexFingerPitch3": ImplicitActuatorCfg(
            joint_names_expr=["rightIndexFingerPitch3"],
            effort_limit=10.0,
            velocity_limit=1.0,
            stiffness=5.0,
            damping=0.5,
            armature=0.5,
        ),
        "rightMiddleFingerPitch1": ImplicitActuatorCfg(
            joint_names_expr=["rightMiddleFingerPitch1"],
            effort_limit=10.0,
            velocity_limit=1.0,
            stiffness=5.0,
            damping=0.5,
            armature=0.5,
        ),
        "rightMiddleFingerPitch2": ImplicitActuatorCfg(
            joint_names_expr=["rightMiddleFingerPitch2"],
            effort_limit=10.0,
            velocity_limit=1.0,
            stiffness=5.0,
            damping=0.5,
            armature=0.5,
        ),
        "rightMiddleFingerPitch3": ImplicitActuatorCfg(
            joint_names_expr=["rightMiddleFingerPitch3"],
            effort_limit=10.0,
            velocity_limit=1.0,
            stiffness=5.0,
            damping=0.5,
            armature=0.5,
        ),
        "rightPinkyPitch1": ImplicitActuatorCfg(
            joint_names_expr=["rightPinkyPitch1"],
            effort_limit=10.0,
            velocity_limit=1.0,
            stiffness=5.0,
            damping=0.5,
            armature=0.5,
        ),
        "rightPinkyPitch2": ImplicitActuatorCfg(
            joint_names_expr=["rightPinkyPitch2"],
            effort_limit=10.0,
            velocity_limit=1.0,
            stiffness=5.0,
            damping=0.5,
            armature=0.5,
        ),
        "rightPinkyPitch3": ImplicitActuatorCfg(
            joint_names_expr=["rightPinkyPitch3"],
            effort_limit=10.0,
            velocity_limit=1.0,
            stiffness=5.0,
            damping=0.5,
            armature=0.5,
        ),
        "leftShoulderPitch": ImplicitActuatorCfg(
            joint_names_expr=["leftShoulderPitch"],
            effort_limit=190.0,
            velocity_limit=3.0,
            stiffness=0.0,
            damping=0.1,
        ),
        "leftShoulderRoll": ImplicitActuatorCfg(
            joint_names_expr=["leftShoulderRoll"],
            effort_limit=350.0,
            velocity_limit=3.5,
            stiffness=0.0,
            damping=0.1,
        ),
        "leftShoulderYaw": ImplicitActuatorCfg(
            joint_names_expr=["leftShoulderYaw"],
            effort_limit=65.0,
            velocity_limit=1.5,
            stiffness=0.0,
            damping=0.1,
        ),
        "leftElbowPitch": ImplicitActuatorCfg(
            joint_names_expr=["leftElbowPitch"],
            effort_limit=65.0,
            velocity_limit=3.5,
            stiffness=0.0,
            damping=0.1,
        ),
        "leftForearmYaw": ImplicitActuatorCfg(
            joint_names_expr=["leftForearmYaw"],
            effort_limit=26.0,
            velocity_limit=0.8,
            stiffness=1000.0,
            damping=0.1,
        ),
        "leftWristRoll": ImplicitActuatorCfg(
            joint_names_expr=["leftWristRoll"],
            effort_limit=14.0,
            velocity_limit=1.0,
            stiffness=500.0,
            damping=0.1,
        ),
        "leftWristPitch": ImplicitActuatorCfg(
            joint_names_expr=["leftWristPitch"],
            effort_limit=14.0,
            velocity_limit=1.0,
            stiffness=500.0,
            damping=0.1,
        ),
        "leftThumbRoll": ImplicitActuatorCfg(
            joint_names_expr=["leftThumbRoll"],
            effort_limit=10.0,
            velocity_limit=1.0,
            stiffness=5.0,
            damping=0.5,
            armature=0.5,
        ),
        "leftThumbPitch1": ImplicitActuatorCfg(
            joint_names_expr=["leftThumbPitch1"],
            effort_limit=10.0,
            velocity_limit=1.0,
            stiffness=5.0,
            damping=0.5,
            armature=0.5,
        ),
        "leftThumbPitch2": ImplicitActuatorCfg(
            joint_names_expr=["leftThumbPitch2"],
            effort_limit=10.0,
            velocity_limit=1.0,
            stiffness=5.0,
            damping=0.5,
            armature=0.5,
        ),
        "leftThumbPitch3": ImplicitActuatorCfg(
            joint_names_expr=["leftThumbPitch3"],
            effort_limit=10.0,
            velocity_limit=1.0,
            stiffness=5.0,
            damping=0.5,
            armature=0.5,
        ),
        "leftIndexFingerPitch1": ImplicitActuatorCfg(
            joint_names_expr=["leftIndexFingerPitch1"],
            effort_limit=10.0,
            velocity_limit=1.0,
            stiffness=5.0,
            damping=0.5,
            armature=0.5,
        ),
        "leftIndexFingerPitch2": ImplicitActuatorCfg(
            joint_names_expr=["leftIndexFingerPitch2"],
            effort_limit=10.0,
            velocity_limit=1.0,
            stiffness=5.0,
            damping=0.5,
            armature=0.5,
        ),
        "leftIndexFingerPitch3": ImplicitActuatorCfg(
            joint_names_expr=["leftIndexFingerPitch3"],
            effort_limit=10.0,
            velocity_limit=1.0,
            stiffness=5.0,
            damping=0.5,
            armature=0.5,
        ),
        "leftMiddleFingerPitch1": ImplicitActuatorCfg(
            joint_names_expr=["leftMiddleFingerPitch1"],
            effort_limit=10.0,
            velocity_limit=1.0,
            stiffness=5.0,
            damping=0.5,
            armature=0.5,
        ),
        "leftMiddleFingerPitch2": ImplicitActuatorCfg(
            joint_names_expr=["leftMiddleFingerPitch2"],
            effort_limit=10.0,
            velocity_limit=1.0,
            stiffness=5.0,
            damping=0.5,
            armature=0.5,
        ),
        "leftMiddleFingerPitch3": ImplicitActuatorCfg(
            joint_names_expr=["leftMiddleFingerPitch3"],
            effort_limit=10.0,
            velocity_limit=1.0,
            stiffness=5.0,
            damping=0.5,
            armature=0.5,
        ),
        "leftPinkyPitch1": ImplicitActuatorCfg(
            joint_names_expr=["leftPinkyPitch1"],
            effort_limit=10.0,
            velocity_limit=1.0,
            stiffness=5.0,
            damping=0.5,
            armature=0.5,
        ),
        "leftPinkyPitch2": ImplicitActuatorCfg(
            joint_names_expr=["leftPinkyPitch2"],
            effort_limit=10.0,
            velocity_limit=1.0,
            stiffness=5.0,
            damping=0.5,
            armature=0.5,
        ),
        "leftPinkyPitch3": ImplicitActuatorCfg(
            joint_names_expr=["leftPinkyPitch3"],
            effort_limit=10.0,
            velocity_limit=1.0,
            stiffness=5.0,
            damping=0.5,
            armature=0.5,
        ),
    },
)