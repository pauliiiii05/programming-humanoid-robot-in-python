'''In this exercise you need to implement inverse kinematics for NAO's legs

* Tasks:
    1. solve inverse kinematics for NAO's legs by using analytical or numerical method.
       You may need documentation of NAO's leg:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    2. use the results of inverse kinematics to control NAO's legs (in InverseKinematicsAgent.set_transforms)
       and test your inverse kinematics implementation.
'''


from forward_kinematics import ForwardKinematicsAgent
from numpy.matlib import identity
import numpy as np


class InverseKinematicsAgent(ForwardKinematicsAgent):
    def inverse_kinematics(self, effector_name, transform):
        '''solve the inverse kinematics

        :param str effector_name: name of end effector, e.g. LLeg, RLeg
        :param transform: 4x4 transform matrix
        :return: list of joint angles
        '''
        joint_angles = []
        # YOUR CODE HERE
        Px = float(transform[0,3])
        Py = float(transform[1,3])
        Pz = float(transform[2,3])

        if (effector_name == "LLeg"):
            Py -= 0.05
        else:
            Py -= -0.05
        
        Pz += 0.085 + 0.04519

        distance = (Px**2 + Py**2 + Pz**2 - 0.1**2 - 0.1029**2) / (2 * 0.1 * 0.1029)
        distance = np.clip(distance, -1, 1)
        knee = np.arccos(-distance)

        length = 0.1 + 0.1029 * np.cos(knee)
        height = 0.1029 * np.sin(knee)
        hipPitch = np.arctan2(Pz, Px) - np.arctan2(height, length)

        anklePitch = -(hipPitch + knee)

        hipRoll = np.arctan2(Py, abs(Pz))
        ankleRoll = -hipRoll

        hipYawPitch = 0.0

        joint_angles = [
            hipYawPitch,
            hipRoll,
            hipPitch,
            knee,
            anklePitch,
            ankleRoll
        ]
        return joint_angles

    def set_transforms(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        angles = self.inverse_kinematics(effector_name, transform)

        if effector_name == "LLeg":
            joints = ['LHipYawPitch', 'LHipRoll', 'LHipPitch',
                    'LKneePitch', 'LAnklePitch', 'LAnkleRoll']
        else:
            joints = ['RHipYawPitch', 'RHipRoll', 'RHipPitch',
                    'RKneePitch', 'RAnklePitch', 'RAnkleRoll']

        names = []
        times = []
        keys = []

        for jname, ang in zip(joints, angles):
            names.append(jname)
            times.append([1.0, 1.1])
            keys.append([[float(ang), [0, 0.0, 0.0], [0, 0.0, 0.0]], [float(ang), [0, 0.0, 0.0], [0, 0.0, 0.0]]])

        self.keyframes = (names, times, keys)

if __name__ == '__main__':
    agent = InverseKinematicsAgent()
    # test inverse kinematics
    T = identity(4)
    T[-1, 1] = 0.05
    T[-1, 2] = -0.26
    agent.set_transforms('LLeg', T)
    agent.run()
