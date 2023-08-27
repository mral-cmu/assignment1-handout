import numpy as np

from numpy import arctan2 as atan2
from numpy import arcsin as asin
from numpy import cos as cos
from numpy import sin as sin

from quadrotor_simulator_py.utils.quaternion import Quaternion


class Rotation3:

    def __init__(self, R=None):
        self.R = None

        if R is None:
            self.R = np.eye(3)
        else:
            self.R = R

    def to_euler_zyx(self):
        """ Convert self.R to Z-Y-X euler angles

        Output:
            zyx: 1x3 numpy array containing euler angles.
                 The order of angles should be phi, theta, psi.
        """

        theta = 0.
        phi = 0.
        psi = 0.

        # TODO: Assignment 1, Problem 1.1
        return np.array([phi, theta, psi])

    @classmethod
    def from_euler_zyx(self, zyx):
        """ Convert euler angle rotation representation to 3x3
                rotation matrix
        Arg:
            zyx: 1x3 numpy array containing euler angles

        Output:
            Rot: 3x3 rotation matrix (numpy)
        """

        # TODO: Assignment 1, Problem 1.2

        Rot = Rotation3()
        Rot.R = np.eye(3)
        return Rot

    def roll(self):
        """ Extracts the phi component from the rotation matrix

        Output:
            phi: scalar value representing phi
        """

        # TODO: Assignment 1, Problem 1.3

        return 0.

    def pitch(self):
        """ Extracts the theta component from the rotation matrix

        Output:
            theta: scalar value representing theta
        """

        # TODO: Assignment 1, Problem 1.4

        return 0.

    def yaw(self):
        """ Extracts the psi component from the rotation matrix

        Output:
            theta: scalar value representing psi
        """

        # TODO: Assignment 1, Problem 1.5

        return 0.

    @classmethod
    def from_quat(self, q):
        """ Calculates the 3x3 rotation matrix from a quaternion
                parameterized as (w,x,y,z).

        Output:
            Rot: 3x3 rotation matrix represented as numpy matrix
        """

        # TODO: Assignment 1, Problem 1.6

        Rot = Rotation3()
        Rot.R = np.eye(3)
        return Rot

    def to_quat(self):
        """ Calculates a quaternion from the class variable
                self.R and returns it

        Output:
            q: An instance of the Quaternion class parameterized
                as [w, x, y, z]
        """

        # TODO: Assignment 1, Problem 1.7

        w = 1.
        x = 0.
        y = 0.
        z = 0.
        return Quaternion([w, x, y, z])
