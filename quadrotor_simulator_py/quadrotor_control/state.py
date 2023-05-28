#!/usr/bin/env python
import numpy as np


class State:
    def __init__(self):
        self.pos = np.zeros((3, 1))
        self.vel = np.zeros((3, 1))
        self.acc = np.zeros((3, 1))
        self.jerk = np.zeros((3, 1))
        self.snap = np.zeros((3, 1))

        self.rot = np.eye(3)
        self.angvel = np.zeros((3, 1))
        self.angacc = np.zeros((3, 1))

        self.yaw = 0.0
        self.dyaw = 0.0
        self.d2yaw = 0.0
        self.d3yaw = 0.0

    def __repr__(self):
        return ('State\n' +
                'pos:    ' + str(self.pos.T) + '\n' +
                'vel:    ' + str(self.vel.T) + '\n' +
                'acc:    ' + str(self.acc.T) + '\n' +
                'jerk:   ' + str(self.jerk.T) + '\n' +
                'snap:   ' + str(self.snap.T) + '\n' +
                'rot:    ' + str(self.rot) + '\n' +
                'angvel: ' + str(self.angvel.T) + '\n' +
                'angacc: ' + str(self.angacc.T) + '\n' +
                'dyaw:   ' + str(self.dyaw) + '\n' +
                'd2yaw:  ' + str(self.d2yaw) + '\n' +
                'd3yaw:  ' + str(self.d3yaw) + '\n')
