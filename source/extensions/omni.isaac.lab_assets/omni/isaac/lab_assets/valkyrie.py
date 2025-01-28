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
)
