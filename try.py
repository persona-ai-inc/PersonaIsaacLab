#!/usr/bin/env python3

from omni.isaac.lab.utils import ParseIHMC
from pathlib import Path
import numpy as np

print('inja 1')
path = Path('/home/oheidari/DataAndVideos/Valkyrie/20250128_1127_valkyrie_testFlatGroundWalking/20250128_1127_valkyrie_testFlatGroundWalking_jointStates.mat')
ihmc = ParseIHMC(path, 'valkyrie')
print(ihmc.joint_names)
print('inja 2')

q, qd, tau = ihmc.getStateTorque("leftElbowPitch", np.array([0,1,2,4,5]))
print(q)    
print(qd)    
print(tau)    
print('inja 3')

ihmc.plotJointPosition('leftKneePitch')    
ihmc.plotJointVelocity('leftKneePitch')    
ihmc.plotJointTorque('leftKneePitch')    


ihmc.plotJointPosition('rightKneePitch')    
ihmc.plotJointVelocity('rightKneePitch')    
ihmc.plotJointTorque('rightKneePitch')    