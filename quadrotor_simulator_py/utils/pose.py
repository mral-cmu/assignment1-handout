import numpy as np

from quadrotor_simulator_py.utils import Rot3
from quadrotor_simulator_py.utils.quaternion import Quaternion


class Pose:

    def __init__(self, T=None):

        self.se3 = np.eye(4)

        if T is None:
            return

        elif isinstance(T, np.ndarray):
            if np.shape(T)[0] == 7:
                self.se3[0:3, 3] = T[0:3]
                self.se3[0:3, 0:3] = Rot3().from_quat(
                    Quaternion(T[3:7]).normalize()).R
            else:
                raise Exception(
                    "[Pose]: Expected input is 7x1 matrix containing [translation; quaternion]")
        elif isinstance(T, list):
            tmp = np.reshape(np.array(T), (7))
            self.se3[0:3, 3] = tmp[0:3]
            self.se3[0:3, 0:3] = Rot3().from_quat(
                Quaternion(tmp[3:7]).normalize()).R
        else:
            raise Exception("[Pose]: not implemented")

    def get_so3(self):
        return self.se3[0:3, 0:3]

    def get_se3(self):
        return self.se3

    def translation(self):
        return self.se3[0:3, 3]

    def set_translation(self, t):
        self.se3[0:3, 3] = t

    def quaternion(self):
        R = Rot3(self.se3[0:3, 0:3])
        return R.to_quat().data

    def set_rotation(self, R):
        self.se3[0:3, 0:3] = R

    def quaternion(self):
        R = Rot3(self.se3[0:3, 0:3])
        return R.to_quat().data

    def compose(self, T):
        """  Return the composition of self.T * T
        """

        # TODO: Assignment 1: Problem 1.8
        return Pose()

    # Take the inverse of a homogeneous transform
    def inverse(self):
        """ Return the inverse of the homogeneous
                transform

        Output:
            Pose: Pose object that represents the inverse
                  of  the input
        """

        # TODO: Assignment 1: Problem 1.9
        return Pose()
