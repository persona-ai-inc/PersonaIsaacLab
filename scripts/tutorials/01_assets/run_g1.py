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
from pathlib import Path
import time
import numpy as np

from isaaclab.app import AppLauncher

import matplotlib.pyplot as plt

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

import isaacsim.core.utils.prims as prim_utils

import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation, ArticulationCfg
from isaaclab.sim import SimulationContext

from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR

## 
# Pre-defined configs
##
from isaaclab_assets import CARTPOLE_CFG, VALKYRIE_CFG, G1_CFG  # isort:skip

def design_scene() -> dict:
    """Designs the scene."""
    # Ground-plane
    cfg = sim_utils.GroundPlaneCfg()
    cfg.func("/World/defaultGroundPlane", cfg)
    # Lights
    cfg = sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75))
    cfg.func("/World/Light", cfg)

    g1Cfg = G1_CFG.copy() # type: ignore
    g1Cfg.prim_path = "/World/Robots/G1"
    g1 = Articulation(cfg=g1Cfg)

    # return the scene information
    scene_entities = {"g1": g1}
    return scene_entities


def run_simulator(sim: sim_utils.SimulationContext, entities: dict[str, Articulation]):
    """Runs the simulation loop."""
    # Extract scene entities
    # note: we only do this here for readability. In general, it is better to access the entities directly from
    #   the dictionary. This dictionary is replaced by the InteractiveScene class in the next tutorial.
    robot = entities["g1"]
    joint_names = robot.joint_names

    sim_dt = sim.get_physics_dt()
    print('sim_dt: ', sim_dt)
    count = 0

    defatul_joint_pos, default_joint_vel = robot.data.default_joint_pos.clone(), robot.data.default_joint_vel.clone()
    default_root_state = robot.data.default_root_state.clone()

    # joint_id, joint_name = robot.find_joints('rightElbowPitch')
    # knee_joint_id, joint_name = robot.find_joints('rightKneePitch')

    # Simulation loop
    while simulation_app.is_running():
        # Reset
        if count % 50000 == 0:
            # reset counter
            count = 0
            # reset the scene entities
            # root state
            # we offset the root state by the origin since the states are written in simulation world frame
            # if this is not done, then the robots will be spawned at the (0, 0, 0) of the simulation world
            # root_state = robot.data.default_root_state.clone()
            
            # robot.write_root_link_pose_to_sim(default_root_state[:, :7])
            # robot.write_root_com_velocity_to_sim(default_root_state[:, 7:])
            # set joint positions with some noise
            
            # robot.write_joint_state_to_sim(defatul_joint_pos, default_joint_vel )
            # clear internal buffers
            robot.reset()
            print("[INFO]: Resetting robot state...")
        
        
        # Apply random action
        # -- generate random joint efforts
        efforts = 10.0 * torch.randn_like(robot.data.joint_pos) 
        
        # robot.write_joint_state_to_sim(torch.from_numpy(q), torch.from_numpy(qd) )

        # efforts.append(effort)
        # -- apply action to the robot
        robot.set_joint_effort_target(efforts)
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
    sim.set_camera_view((3.5, 0.0, 2.0), (0.0, 0.0, 1.0))
    # Design scene
    scene_entities = design_scene()
    # Play the simulator
    sim.reset()
    # Now we are ready!
    print("[INFO]: Setup complete...")

    # Run the simulator
    run_simulator(sim, scene_entities)


if __name__ == "__main__":
    # run the main function
    main()
    # close sim app
    simulation_app.close()
