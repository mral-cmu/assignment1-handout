#!/usr/bin/env python
import numpy as np

class TrackingError:
    pos_des = np.zeros((3,1))
    vel_des = np.zeros((3,1))
    acc_des = np.zeros((3,1))
    jerk_des = np.zeros((3,1))
    snap_des = np.zeros((3,1))

    yaw_des = 0.0
    dyaw_des = 0.0

    pos_err = np.zeros((3,1))
    vel_err = np.zeros((3,1))
    yaw_err = np.zeros((3,1))
    dyaw_err = np.zeros((3,1))

    def __init__(self):
        pass

    def __repr__(self):
        return('State\n' +
               'pos_des:    ' + str(self.pos_des.flatten()) + '\n' +
               'vel_des:    ' + str(self.vel_des.flatten()) + '\n' + 
               'acc_des:    ' + str(self.acc_des.flatten()) + '\n' +
               'jerk_des:   ' + str(self.jerk_des.flatten()) + '\n' +
               'snap_des:   ' + str(self.snap_des.flatten()) + '\n' +
               'yaw_des:   ' + str(self.yaw_des) + '\n' +
               'dyaw_des:   ' + str(self.dyaw_des) + '\n' +
               'pos_err:    ' + str(self.pos_err.flatten()) + '\n' +
               'vel_err:    ' + str(self.vel_err.flatten()) + '\n' + 
               'yaw_err:   ' + str(self.yaw_err) + '\n' +
               'dyaw_err:   ' + str(self.dyaw_err) + '\n')
