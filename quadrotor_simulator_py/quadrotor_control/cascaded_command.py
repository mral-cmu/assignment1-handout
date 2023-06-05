#!/usr/bin/env python
import numpy as np
from quadrotor_simulator_py.utils import Rot3
from quadrotor_simulator_py.utils import Quaternion


class CascadedCommand:

    def __init__(self):
        self.Rdes = np.eye(3)
        self.thrust_des = 0.0
        self.angvel_des = np.zeros((3, 1))
        self.angacc_des = np.zeros((3, 1))
