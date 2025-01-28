# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""This script demonstrates how to spawn a cart-pole and interact with it.

.. code-block:: bash
    Omid
    # Usage
    ./isaaclab.sh -p source/standalone/tutorials/01_assets/run_articulation.py

"""

"""Launch Isaac Sim Simulator first."""


import argparse

from omni.isaac.lab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Tutorial on spawning and interacting with an articulation.")
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import torch

import omni.isaac.core.utils.prims as prim_utils

import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.assets import Articulation
from omni.isaac.lab.sim import SimulationContext

from omni.isaac.lab.utils.assets import ISAAC_NUCLEUS_DIR

## 
# Pre-defined configs
##
from omni.isaac.lab_assets import CARTPOLE_CFG, VALKYRIE_CFG  # isort:skip

from parse_ihmc import ParseIHMC
from pathlib import Path
path = Path('/home/oheidari/DataAndVideos/Valkyrie/20250128_1127_valkyrie_testFlatGroundWalking/20250128_1127_valkyrie_testFlatGroundWalking_jointStates.mat')
ihmc = ParseIHMC(path, 'valkyrie')

def design_scene() -> tuple[dict, list[list[float]]]:
    """Designs the scene."""
    # Ground-plane
    cfg = sim_utils.GroundPlaneCfg()
    cfg.func("/World/defaultGroundPlane", cfg)
    # Lights
    cfg = sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75))
    cfg.func("/World/Light", cfg)

    # this gives the asset root path
    print('===> ISAAC_NUCLEUS_DIR: ', ISAAC_NUCLEUS_DIR)
    
    # cfg = sim_utils.UsdFileCfg(usd_path=f"{ISAAC_NUCLEUS_DIR}/Robots/SanctuaryAI/Phoenix/phoenix.usd")
    # cfg.func("/World/Robots/Sanct", cfg, translation=(0.0, 0.0, 0.01))

    valkCfg = VALKYRIE_CFG.copy() # type: ignore
    valkCfg.prim_path = "/World/Robots/Valk1"
    valkyrie = Articulation(cfg=valkCfg)

    # to get the parent of the asset root path
    from omni.isaac.core.utils.nucleus import get_assets_root_path
    assets_root_path = get_assets_root_path()
    print('===> assets_root_path: ', assets_root_path)

    # Create separate groups called "Origin1", "Origin2"
    # Each group will have a robot in it
    origins = [[0.0, 0.0, 0.0], [-1.0, 0.0, 0.0]]
    # Origin 1
    # prim_utils.create_prim("/World/Origin1", "Xform", translation=origins[0])
    # # Origin 2
    # prim_utils.create_prim("/World/Origin2", "Xform", translation=origins[1])

    # Articulation
    # cartpole_cfg = CARTPOLE_CFG.copy()
    # cartpole_cfg.prim_path = "/World/Origin.*/Robot"
    # cartpole = Articulation(cfg=cartpole_cfg)

    # return the scene information
    scene_entities = {"valkyrie": valkyrie}


    return scene_entities, origins


def run_simulator(sim: sim_utils.SimulationContext, entities: dict[str, Articulation], origins: torch.Tensor):
    """Runs the simulation loop."""
    # Extract scene entities
    # note: we only do this here for readability. In general, it is better to access the entities directly from
    #   the dictionary. This dictionary is replaced by the InteractiveScene class in the next tutorial.
    robot = entities["valkyrie"]
    joint_names = robot.joint_names
    right_elbow_pitch_index = joint_names.index('rightElbowPitch')
    # Define simulation stepping
    sim_dt = sim.get_physics_dt()
    count = 0
    
    # Simulation loop
    while simulation_app.is_running():
        # Reset
        if count % ihmc.num_time_steps == 0:
            # reset counter
            count = 0
            # reset the scene entities
            # root state
            # we offset the root state by the origin since the states are written in simulation world frame
            # if this is not done, then the robots will be spawned at the (0, 0, 0) of the simulation world
            root_state = robot.data.default_root_state.clone()
            # root_state[:, :3] += origins
            # robot.write_root_link_pose_to_sim(root_state[:, :7])
            # robot.write_root_com_velocity_to_sim(root_state[:, 7:])
            # set joint positions with some noise
            joint_pos, joint_vel = robot.data.default_joint_pos.clone(), robot.data.default_joint_vel.clone()
            print('joint pos size: ', joint_pos.size())
            # joint_pos[0,25] += 0.5
            joint_pos, joint_vel, joint_effort = ihmc.getStateTorqueOneTimeStepAllJoints(count)
            robot.write_joint_state_to_sim(torch.from_numpy(joint_pos), torch.from_numpy(joint_vel) )
            # clear internal buffers
            robot.reset()
            print("[INFO]: Resetting robot state...")
        # Apply random action
        # -- generate random joint efforts
<<<<<<< Updated upstream
        efforts = torch.randn_like(robot.data.joint_pos) * 5.0
        #efforts = torch.zeros_like(robot.data.joint_pos)
        #efforts[0, 25] = 40.0
=======
        # efforts = torch.randn_like(robot.data.joint_pos) 
        q, qd, efforts = ihmc.getStateTorqueOneTimeStepAllJoints(count)
        # efforts[0, 25] = 40.0
>>>>>>> Stashed changes
        # -- apply action to the robot
        robot.set_joint_effort_target(torch.from_numpy(efforts))
        # -- write data to sim
        robot.write_data_to_sim()
        # Perform step
        sim.step()
        # Increment counter
        count += 1
        # Update buffers
        robot.update(sim_dt)


def main():
    """Main function."""
    # Load kit helper
    sim_cfg = sim_utils.SimulationCfg(device=args_cli.device)
    sim = SimulationContext(sim_cfg)
    # Set main camera
    sim.set_camera_view((2.5, 0.0, 4.0), (0.0, 0.0, 2.0))
    # Design scene
    scene_entities, scene_origins = design_scene()
    scene_origins = torch.tensor(scene_origins, device=sim.device)
    # Play the simulator
    sim.reset()
    # Now we are ready!
    print("[INFO]: Setup complete...")
    # Run the simulator
    run_simulator(sim, scene_entities, scene_origins)


if __name__ == "__main__":
    # run the main function
    main()
    # close sim app
    simulation_app.close()
