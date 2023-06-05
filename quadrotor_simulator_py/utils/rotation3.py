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

    @classmethod
    def from_euler_zyx(self, zyx):

        # TODO: Fill me in

        Rot = Rotation3()
        Rot.R = np.eye(3)
        return Rot

    def to_euler_zyx(self):

        # TODO: Fill me in

        theta = 0.
        phi = 0.
        psi = 0.

        return np.array([phi, theta, psi])

    def roll(self):

        # TODO: Fill me in

        return 0.

    def pitch(self):

        # TODO: Fill me in

        return 0.

    def yaw(self):

        # TODO: Fill me in

        return 0.

    @classmethod
    def from_quat(self, q):

        # TODO: Fill me in

        Rot = Rotation3()
        Rot.R = np.eye(3)
        return Rot

    def to_quat(self):

        # TODO: Fill me in

        w = 1.
        x = 0.
        y = 0.
        z = 0.
        return Quaternion([w, x, y, z])
