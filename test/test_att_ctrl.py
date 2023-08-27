import os
import unittest
import numpy as np
import math
from sklearn.metrics import mean_squared_error
import matplotlib.pyplot as plt
import sys

sys.path.append('../')

from quadrotor_simulator_py.quadrotor_control import State
from quadrotor_simulator_py.quadrotor_control import CascadedCommand
from quadrotor_simulator_py.utils.quaternion import Quaternion
from quadrotor_simulator_py.quadrotor_control import QuadrotorAttitudeControllerPD
from quadrotor_simulator_py.utils import Rot3
from quadrotor_simulator_py.utils.pose import Pose


def test_att_ctrl(config, npz_filepath):
    attctrl = QuadrotorAttitudeControllerPD(config)

    # Load data
    if os.path.isfile(npz_filepath):
        print('Loading...' + npz_filepath)
    else:
        raise Exception(
            npz_filepath + " not found in current path.")

    # Load inputs
    data = np.load(npz_filepath, allow_pickle=True)
    curr_states = data['curr_states']
    correct_rpms = data['rpms']
    cascaded_commands = data['cascaded_commands']
    times = data['times']

    # Zero out student solution
    rpms = np.zeros((4, len(times)))

    for idx, time in enumerate(times):
        attctrl.set_cascaded_cmd(cascaded_commands[idx])
        attctrl.set_current_state(curr_states[idx])
        rpms[:, idx] = attctrl.run_ctrl().flatten()

    print('Plotting RPMs...')
    fig, axs = plt.subplots(4)
    for i in range(0, 4):
        axs[i].plot(times, rpms[i, :], linewidth=3)
        axs[i].plot(times, correct_rpms[i, :])
    plt.suptitle('RPMs')
    axs[0].legend(['Student solution', 'Correct solution'])
    plt.show()

DATA_DIR = "../data/"
config = "../config/rocky0704_model_params.yaml"
test_att_ctrl(config, DATA_DIR + "2023-05-28-07-18-08_attitudepd.npz")
