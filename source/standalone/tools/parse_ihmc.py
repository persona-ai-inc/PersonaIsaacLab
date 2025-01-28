import scipy.io as sio
import numpy as np 
from pathlib import Path, PosixPath
import matplotlib.pyplot as plt

class ParseIHMC:
    """ Parse IHMC exported joint states that is of type .mat 
    
    TODO: convert this class so it reads from file at each inquiry instead of holding large 
    matrices like self.q, self.qd, self.tau in memory
    """

    def __init__(self, ihmc_mat_file: Path, robot_name:str) -> None:
        """ Initialize the class with given args

        Args:
            ihmc_mat_file: path to the ihmc file
            robot_name: the name of the robot used in simulation
        
        """
        
        self.file = str(ihmc_mat_file)
        self.robot_name = robot_name
        self.joint_names = []
        self.num_joints = 0
        self.num_time_steps = 0
        self.q = np.array([])
        self.qd = np.array([])
        self.tau = np.array([])

        # load data
        contents = sio.loadmat(self.file)

        # get joint names
        fields = contents['root']['valkyrie'][0][0].dtype.names
        
        # get contents. These are related to [qd, q, tau] in order. So the length is (3 x number) of joints
        state_tau = contents['root']['valkyrie'][0][0][0,0]

        self.num_time_steps = state_tau[0].shape[0]
        print('number of time steps: {}'.format(self.num_time_steps))

        self.num_joints, r = divmod(len(state_tau), 3)
        if not r == 0:
            raise Exception('number of values is not divisible by 3')  
        print('number of joints: ', self.num_joints)

        self.joint_names = self.extractJointNames(fields)

        assert len(self.joint_names) == self.num_joints, 'length of joint names: {} vs number of joints: {}'.format(len(self.joint_names), self.num_joints)

        self.extractStateTorque(state_tau)

    def extractJointNames(self, fields: list[str]) -> list[str]:
        """Extract joint names from field names. q_leftHipYaw => leftHipYaw
        Args:
            fields: a list holding names like q_leftHipYaw. 
            ['q_leftHipYaw', 'qd_leftHipYaw', 'tau_leftHipYaw', 'q_leftHipRoll', 'qd_leftHipRoll', 'tau_leftHipRoll' ...]
        
        Returns:
            names: a list of joints actual names
        """

        names = []
        for i, x in enumerate(fields):
            q, r = divmod(i, 3)
            if r == 0:
                names.append(x.split('_')[1])

        return names

    def extractStateTorque(self, state_tau: list[np.ndarray]) -> None:
        """Extract state (q, qd) and torque (tau)
        
        Args:
            state_tau: a list of dim 3*num_joints. Each element is of size num_time_steps. And looks like:
            [q_joint1, qd_joint1, tau_joint1, q_joint2, qd_joint2, tau_joint2 ...]
        
        Returns:
            position (q), velocity (qd) and torque (tau) of the joints.
            q, qd and tau are of size (num_time_steps x num_joints)
        """
        self.q = np.zeros((self.num_time_steps, self.num_joints))
        self.qd = np.zeros((self.num_time_steps, self.num_joints))
        self.tau = np.zeros((self.num_time_steps, self.num_joints))

        # extract values as (num_time_steps x num_joints)
        for ix in range(self.num_joints):
            self.q[:,ix] = state_tau[3*ix].reshape(-1)
            self.qd[:, ix] = state_tau[3*ix + 1].reshape(-1)
            self.tau[:, ix] = state_tau[3*ix + 2].reshape(-1)

    def getStateTorque(self, joint_name: str, time_steps: np.ndarray):
        """ Get joint state and torque for a give arguments
        
        Args:
            joint_name: name of the joint
            time_steps: time steps. This is a numpy array so the chunk of time steps are allowed
        
        Returns:
            q, qd, tau for a given joint and time steps
        """

        index = self.joint_names.index(joint_name)
        
        return self.q[time_steps, index], self.qd[time_steps, index], self.tau[time_steps, index]

    def getStateTorqueOneTimeStepAllJoints(self, time_step):
        return self.q[time_step, :], self.qd[time_step, :], self.tau[time_step, :]

    def plotJointPosition(self, joint_name):
        """ Plot position of a given joint over time"""
        
        index = self.joint_names.index(joint_name)
        plt.plot(range(self.num_time_steps), self.q[:,index])
        plt.xlabel('time')
        plt.ylabel('position')
        plt.show()

    def plotJointVelocity(self, joint_name):
        """ Plot velocity of a given joint over time"""
        pass
    def plotJointTorque(self, joint_name):
        """ Plot torque of a given joint over time"""
        
        index = self.joint_names.index(joint_name)
        plt.plot(range(self.num_time_steps), self.tau[:,index])
        plt.xlabel('time')
        plt.ylabel('torque')
        plt.show()

if __name__ == '__main__':

    path = Path('/home/oheidari/DataAndVideos/Valkyrie/20250128_1127_valkyrie_testFlatGroundWalking/20250128_1127_valkyrie_testFlatGroundWalking_jointStates.mat')
    parser = ParseIHMC(path, 'valkyrie')
    print(parser.joint_names)

    q, qd, tau = parser.getStateTorque("leftElbowPitch", np.array([0,1,2,4,5]))
    print(q)    
    print(qd)    
    print(tau)    

    # parser.plotJointPosition('leftKneePitch')    
    parser.plotJointTorque('leftKneePitch')    

