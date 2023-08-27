import os
import unittest
import numpy as np
import math
from sklearn.metrics import mean_squared_error
import matplotlib.pyplot as plt
import sys

sys.path.append('../')

from quadrotor_simulator_py.quadrotor_control import QuadrotorPositionControllerPD
from quadrotor_simulator_py.quadrotor_control import State
from quadrotor_simulator_py.quadrotor_control import CascadedCommand
from quadrotor_simulator_py.utils.quaternion import Quaternion
from quadrotor_simulator_py.utils import Rot3
from quadrotor_simulator_py.utils.pose import Pose

def casc_to_vec(c):
    v = np.zeros(16)
    v[0] = c.thrust_des
    v[1:10] = c.Rdes.flatten()
    v[10:13] = c.angvel_des.flatten()
    v[13:16] = c.angacc_des.flatten()
    return v


def test_pos_ctrl(config, npz_filepath):
    posctl = QuadrotorPositionControllerPD(config)

    # Load data
    if os.path.isfile(npz_filepath):
        print('Loading...' + npz_filepath)
    else:
        raise Exception(
            npz_filepath + " not found in current path.")

    # Load inputs
    data = np.load(npz_filepath, allow_pickle=True)
    curr_states = data['curr_states']
    ref_states = data['ref_states']
    correct_cascaded_commands = data['cascaded_commands']
    times = data['times']

    # Zero out student solution
    student_res = np.zeros((16, len(times)))
    correct_res = np.zeros((16, len(times)))

    for idx, time in enumerate(times):
        posctl.set_current_state(curr_states[idx])
        posctl.set_reference_state(ref_states[idx])
        posctl.run_ctrl()
        student_res[:, idx] = casc_to_vec(posctl.get_cascaded_command())
        correct_res[:, idx] = casc_to_vec(correct_cascaded_commands[idx])

    print('Plotting desired thrust...')
    fig, axs = plt.subplots(1)
    axs.plot(times, student_res[0,:], linewidth=3)
    axs.plot(times, correct_res[0,:])
    axs.legend(['Student solution', 'Correct solution'])
    plt.suptitle('Desired thrust')
    plt.show()

    print('Plotting angular velocity desired...')
    fig, axs = plt.subplots(3)
    for i in range(0, 3):
        axs[i].plot(times, student_res[10+i, :], linewidth=3)
        axs[i].plot(times, correct_res[10+i, :])
    plt.suptitle('Desired angular velocity')
    axs[0].legend(['Student solution', 'Correct solution'])
    plt.show()

    print('Plotting angular acceleration desired...')
    fig, axs = plt.subplots(3)
    for i in range(0, 3):
        axs[i].plot(times, student_res[13+i, :], linewidth=3)
        axs[i].plot(times, correct_res[13+i, :])
    plt.suptitle('Desired angular acceleration')
    axs[0].legend(['Student solution', 'Correct solution'])
    plt.show()

DATA_DIR = "../data/"
config = "../config/rocky0704_model_params.yaml"
test_pos_ctrl(config, DATA_DIR + "2023-05-28-07-18-08_positionpd.npz")
