import os
import unittest
import numpy as np
import math
from sklearn.metrics import mean_squared_error
import matplotlib.pyplot as plt
import sys

import os
qs_path = os.path.dirname(os.path.abspath(__file__))+'/..'
sys.path.append(qs_path)

from quadrotor_simulator_py.quadrotor_control import State
from quadrotor_simulator_py.quadrotor_control import CascadedCommand
from quadrotor_simulator_py.utils.quaternion import Quaternion
from quadrotor_simulator_py.quadrotor_control import QuadrotorAttitudeControllerPD
from quadrotor_simulator_py.utils import Rot3
from quadrotor_simulator_py.utils.pose import Pose

def score_results(res, correct, eps, test_name):
    rmse = math.sqrt(mean_squared_error(res, correct))
    if rmse < eps:
        print('Test ' + test_name + ' passed')
        return 1.
    print('Test ' + test_name + ' failed. RMSE is ' +
          str(rmse) + ' but < ' + str(eps) + ' required.')
    return 0.

def test_wrench_to_rotor_forces(config):
    attctrl = QuadrotorAttitudeControllerPD(config)
    thrust = 25.93171469984705
    torque = np.array([[-0.00252562], [-0.005851  ], [-0.00079098]])
    student_soln = attctrl.wrench_to_rotor_forces(thrust, torque)
    correct_soln = np.array([[6.52098229], [6.48149448], [6.472457  ], [6.45678094]])

    score = 0.0
    eps = 1e-5
    score += score_results(student_soln,
                           correct_soln, eps, 'wrench_to_rotor_forces')
    return score

def test_force_to_rpm(config):
    attctrl = QuadrotorAttitudeControllerPD(config)

    forces = np.array([[6.52098229], [6.48149448], [6.472457  ], [6.45678094]])
    correct_soln = np.array([[13264.77092516], [13227.28059562], [13218.68388225], [13203.75781443]])
    student_soln = attctrl.force_to_rpm(forces)

    score = 0.0
    eps = 1
    score += score_results(student_soln,
                           correct_soln, eps, 'force_to_rpm')
    return score

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
        axs[i].set_xlabel('Time (s)')
        axs[i].set_ylabel('RPM')
    plt.suptitle('RPMs')
    axs[0].legend(['Student solution', 'Correct solution'])
    plt.show()

if __name__ == "__main__":
    DATA_DIR = qs_path + "/data/"
    config = qs_path + "/config/rocky0704_model_params.yaml"
    test_wrench_to_rotor_forces(config)
    test_force_to_rpm(config)
    test_att_ctrl(config, DATA_DIR + "2023-05-28-07-18-08_attitudepd.npz")
