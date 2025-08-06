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

from quadrotor_simulator_py.quadrotor_control import QuadrotorPositionControllerPD
from quadrotor_simulator_py.quadrotor_control import State
from quadrotor_simulator_py.quadrotor_control import CascadedCommand
from quadrotor_simulator_py.utils.quaternion import Quaternion
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

def casc_to_vec(c):
    v = np.zeros(16)
    v[0] = c.thrust_des
    v[1:10] = c.Rdes.flatten()
    v[10:13] = c.angvel_des.flatten()
    v[13:16] = c.angacc_des.flatten()
    return v

def test_compute_body_z_accel(config):
    posctl = QuadrotorPositionControllerPD(config)

    a_des = np.array([[-0.64737645], [-0.71837917], [ 9.73053843]])
    R_curr = np.array([[ 0.33676599, -0.93921243, -0.06684812],
                       [ 0.93702956,  0.34126754, -0.07424336],
                       [ 0.09254338, -0.03763603,  0.99499711]])
    score = 0.0
    eps = 1e-5
    correct_soln = [9.77846842]
    student_soln = posctl.compute_body_z_accel(a_des, R_curr)
    score += score_results(student_soln,
                           correct_soln, eps, 'compute_body_z_accel')
    return score

def test_compute_orientation(config):
    posctl = QuadrotorPositionControllerPD(config)

    a_des = np.array([[-0.64737645], [-0.71837917], [ 9.73053843]])
    yaw_ref = 1.225783375762712
    correct_soln = np.array([[ 0.33678727, -0.93925041, -0.06620424],
                              [ 0.93711546,  0.34120003, -0.07346537],
                              [ 0.09159127, -0.03729882,  0.9950979 ]])
    
    student_soln = posctl.compute_orientation(a_des, yaw_ref)
    score = 0.0
    eps = 1e-5
    score += score_results(student_soln,
                           correct_soln, eps, 'compute_orientation')
    return score

def test_compute_hod_refs(config):
    posctl = QuadrotorPositionControllerPD(config)

    acc_vec_des = np.array([[-0.64737645], [-0.71837917], [ 9.73053843]])
    flat_ref = State()
    flat_ref.jerk = np.array([[1.87320902], [1.86528927], [0.29143251]])
    flat_ref.snap = np.array([[-1.91676064], [-1.83913585], [-0.56022513]])
    flat_ref.yaw = 1.225783375762712
    flat_ref.dyaw = 7.450581485102248e-09

    R_des = np.array([[ 0.33678727, -0.93925041, -0.06620424],
                      [ 0.93711546, 0.34120003, -0.07346537],
                      [ 0.09159127, -0.03729882, 0.9950979 ]])
    correct_angvel_des = np.array([[0.11595324], [0.24600537], [0.00922092]])
    correct_angacc_des = np.array([[-0.12049265],[-0.25004275],[-0.03793744]])

    student_angvel_des, student_angacc_des = posctl.compute_hod_refs(acc_vec_des, flat_ref, R_des)

    score = 0.0
    eps = 1e-5
    score += score_results(student_angvel_des,
                           correct_angvel_des, eps, 'angular velocity desired in compute_hod_refs')
    score += score_results(student_angacc_des,
                           correct_angacc_des, eps, 'angular acceleration desired in compute_hod_refs')
    return score


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
    axs.set_xlabel('Time (s)')
    axs.set_ylabel('Thrust')
    plt.suptitle('Desired thrust')
    plt.show()

    print('Plotting angular velocity desired...')
    fig, axs = plt.subplots(3)
    for i in range(0, 3):
        axs[i].plot(times, student_res[10+i, :], linewidth=3)
        axs[i].plot(times, correct_res[10+i, :])
        axs[i].set_xlabel('Time (s)')
        axs[i].set_ylabel('ang vel (rad/s)')
    plt.suptitle('Desired angular velocity')
    axs[0].legend(['Student solution', 'Correct solution'])
    plt.show()

    print('Plotting angular acceleration desired...')
    fig, axs = plt.subplots(3)
    for i in range(0, 3):
        axs[i].plot(times, student_res[13+i, :], linewidth=3)
        axs[i].plot(times, correct_res[13+i, :])
        axs[i].set_xlabel('Time (s)')
        axs[i].set_ylabel('ang acc (rad/s^2)')
    plt.suptitle('Desired angular acceleration')
    axs[0].legend(['Student solution', 'Correct solution'])
    plt.show()

if __name__ == "__main__":
    DATA_DIR = qs_path + "/data/"
    config = qs_path + "/config/rocky0704_model_params.yaml"
    test_compute_body_z_accel(config)
    test_compute_orientation(config)
    test_compute_hod_refs(config)
    test_pos_ctrl(config, DATA_DIR + "2023-05-28-07-18-08_positionpd.npz")
