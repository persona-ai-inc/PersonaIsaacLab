# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import unittest

"""Launch Isaac Sim Simulator first.

This is only needed because of warp dependency.
"""

from omni.isaac.lab.app import AppLauncher, run_tests

# launch omniverse app in headless mode
simulation_app = AppLauncher(headless=True).app


"""Rest everything follows."""

import torch
import torch.utils.benchmark as benchmark
import numpy as np

import omni.isaac.lab.utils.parse_ihmc as pihmc
from pathlib import Path
import random

class TestParseIHMC(unittest.TestCase):
    """Test fixture for checking parse ihmc in Isaac Lab."""

    def test_(self):
        """Test is_identity_pose method."""
        identity_pos_one_row = torch.zeros(3)
        identity_rot_one_row = torch.tensor((1.0, 0.0, 0.0, 0.0))

        # self.assertTrue(math_utils.is_identity_pose(identity_pos_one_row, identity_rot_one_row))
        
        path = Path('/home/oheidari/DataAndVideos/Valkyrie/20250128_1127_valkyrie_testFlatGroundWalking/20250128_1127_valkyrie_testFlatGroundWalking_jointStates.mat')
        ihmc = pihmc.ParseIHMC(path, 'valkyrie')
        joint_variable = 'leftKneePitch'
        print(ihmc.q)

        ihmc.plotJointPosition(joint_variable)    
        ihmc.plotJointTorque(joint_variable)    
        q, qd, tau = ihmc.getStateTorque(joint_variable, np.array([1,2,3,4,5]))
        print(q)

        ix = ihmc.joint_names.index(joint_variable)
        print('old_indx', ix)

        joint_names = ihmc.joint_names
        print(joint_names)
        
        joint_names_new = joint_names.copy()
        random.shuffle(joint_names_new)
        print(joint_names_new)

        ihmc.setJointOrder(joint_names_new)
        print(ihmc.q)

        ix = ihmc.joint_names.index(joint_variable)
        print('new_indx', ix)

        ihmc.plotJointPosition(joint_variable)    
        ihmc.plotJointTorque(joint_variable)    

        q, qd, tau = ihmc.getStateTorque(joint_variable, np.array([1,2,3,4,5]))
        print(q)

        

        old_js = ['knee','hip','ankle']    
        qs = np.array([[10,45,63],
                       [12,46,64],
                       [13,47,65]
                       ])
        new_js = ['hip','ankle','knee']    
        for index_new, joint in enumerate(new_js):
            print(index_new, ' => ' , joint)
            index_old = old_js.index(joint)
            print('index_old ', index_old)

            q_temp = qs[:,index_new].copy()
            print(q_temp)
            qs[:,index_new] = qs[:,index_old]
            print('qs1\n', qs)
            qs[:,index_old] = q_temp
            print('qs2\n',qs)

        print('qs:\n',qs)
        
        # old_js = ['hip', 'ankle' ,'knee']    
        # qs = np.array([[45, 63  , 10],
        #                [46, 64  , 12],
        #                [47, 65  , 13]
        #                ])


if __name__ == "__main__":
    run_tests()
