import numpy as np

from numpy import arctan2 as atan2
from numpy import arcsin as asin
from numpy import cos as cos
from numpy import sin as sin


class Quaternion:

    def __init__(self, q=None):

        # Format: w, x, y, z
        self.data = np.array([1, 0, 0, 0])

        if q is None:
            self.data = np.array([1, 0, 0, 0])
        elif len(q) == 4:
            self.data = np.array(q)
        else:
            raise Exception("Quaternion: Expected input 4x1")

    def w(self):
        return self.data[0]

    def x(self):
        return self.data[1]

    def y(self):
        return self.data[2]

    def z(self):
        return self.data[3]

    def normalize(self):
        self.data = np.divide(self.data, self.norm())
        return Quaternion(self.data)

    def norm(self):
        return np.linalg.norm(self.data)
