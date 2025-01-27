import scipy.io as sio

path = '/home/oheidari/Downloads/20250120_Valkyrie_SlopesTestSimData/Valkyrie/20250120_1845_valkyrie_testUpSlopeVal2Scale/20250120_1845_valkyrie_testUpSlopeVal2Scale_jointStates.mat'
dict = sio.loadmat(path)

print(len(dict))

print(dict.keys())


print(dict['__header__'])
print(dict['__version__'])
print(dict['__globals__'])


# print(dict['root'][0][0][0][0][0])

# [[(array([[(array([[-0.00024396]
                   
#                    Pitch', 'O')]),)]]

import numpy as np 

data = dict['root'][0][0][0][0][0]
print("number of steps: ", len(data))

print(data[0].shape)
print(data[0])