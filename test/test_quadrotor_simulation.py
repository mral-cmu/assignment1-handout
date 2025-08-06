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
from quadrotor_simulator_py.utils import Rot3
from quadrotor_simulator_py.quadrotor_model import QuadrotorModel
from quadrotor_simulator_py.quadrotor_model import ModelParameters
from quadrotor_simulator_py.utils.quaternion import Quaternion

def score_results(res, correct, eps, test_name):
    rmse = math.sqrt(mean_squared_error(res, correct))
    if rmse < eps:
        print('Test ' + test_name + ' passed')
        return 1.
    print('Test ' + test_name + ' failed. RMSE is ' +
          str(rmse) + ' but < ' + str(eps) + ' required.')
    return 0.

def test_calculate_force_and_torque_from_rpm(config):
    rpms =[13339., 13342., 13443., 13445.]
    correctF = 26.625145226675997
    correctM = np.array([[0.00011068], [0.00056486], [0.00238319]])

    q = QuadrotorModel()
    q.initialize(config)
    F, M = q.calculate_force_and_torque_from_rpm(rpms)

    score = 0.0
    eps = 1e-3
    score += score_results(M, correctM, eps, "calculate moments in calculate_force_and_torque_from_rpm function")
    score += score_results([F], [correctF], eps, "calculate force in calculate_force_and_torque_from_rpm")
    return score/2

def test_quaternion_derivative(config):
    qn = Quaternion([ 0.35243746, -0.01187685,  0.00672484,  0.93573584])
    wb = np.array([[ 0.02712527], [-0.02761497], [-0.00070014]])
    correctDq = np.array([[ 5.81509150e-04],
                          [ 1.76977848e-02],
                          [ 7.82060932e-03],
                          [-5.05956206e-05]])

    q = QuadrotorModel()
    q.initialize(config)
    dq = q.quaternion_derivative(qn, wb)

    score = 0.0
    eps = 1e-4
    score += score_results(dq, correctDq, eps, "quaternion_derivative function")
    return score

def test_calculate_world_frame_linear_acceleration(config):
    m = ModelParameters()
    m.cg_offset = np.zeros((3,1))
    m.mass = 2.652
    m.gravity_norm = 9.80665

    ang_acc = np.array([[8.03615234e-04], [-5.65237829e-01], [-5.05571968e-05]])
    wb = np.array([[-1.25271417e-05], [-5.06917483e-01],  [1.08222736e-07]])
    Rwb = [[ 9.88127534e-01,  9.41751957e-07,  1.53635860e-01],
           [ 1.42814118e-07,  1.00000000e+00, -7.04829277e-06],
           [-1.53635860e-01,  6.98655352e-06,  9.88127534e-01]]
    u1 = 26.319712011351996
    correct_aw = np.array([[ 1.52475550e+00], [-6.99506168e-05], [-1.39024147e-06]])

    q = QuadrotorModel()
    q.initialize(config)
    aw = q.calculate_world_frame_linear_acceleration(m, ang_acc, wb, Rwb, u1)

    score = 0.0
    eps = 1e-5
    score += score_results(aw, correct_aw, eps, "calculate_world_frame_linear_acceleration function")
    return score

def test_calculate_angular_acceleration(config):
    quad_model = QuadrotorModel()
    quad_model.initialize(config)

    moments = np.array([[-0.0028608], [0.00194872], [0.00126825]])
    wb = np.array([[0.05485483], [-0.1093739], [0.4180035]])
    Fdes = np.array([[0.], [0.], [27.17778372]])
    correct_angacc = np.array([[-0.18927783], [0.12048948], [0.06146689]])

    angacc = quad_model.calculate_angular_acceleration(quad_model.model_params,
                                                       moments, wb, Fdes)

    eps = 1e-6
    score = 0.0
    score += score_results(angacc, correct_angacc, eps, "calculate_angular_acceleration function")
    return score

def test_ode_step(config, npz_filepath):
    quadrotor_model_py = QuadrotorModel()
    quadrotor_model_py.initialize(config)
    quadrotor_model_py.reset_simulation()

    # Load data
    if os.path.isfile(npz_filepath):
        print('Loading...' + npz_filepath)
    else:
        raise Exception(
            npz_filepath + " not found in current path.")

    # Load inputs
    data = np.load(npz_filepath)
    uRPMs = data['uRPMs']
    times = data['times']

    # Load solution data
    correct_pos = data['pos']
    correct_ori = data['ori']
    correct_vel = data['vel']
    correct_acc = data['acc']
    correct_angvel = data['angvel']
    correct_angacc = data['angacc']

    # Zero out student solution
    pos = np.zeros(np.shape(correct_pos))
    ori = np.zeros(np.shape(correct_ori))
    vel = np.zeros(np.shape(correct_vel))
    acc = np.zeros(np.shape(correct_acc))
    angvel = np.zeros(np.shape(correct_angvel))
    angacc = np.zeros(np.shape(correct_angacc))

    for idx, time in enumerate(times):
        uRPM = uRPMs[:, idx]
        quadrotor_model_py.apply_command(uRPM)
        quadrotor_model_py.update(time)
        state = quadrotor_model_py.get_state()

        pos[:, idx] = state.pos.flatten()
        ori[:, idx] = Rot3(
            quadrotor_model_py.get_pose().get_so3()).to_euler_zyx()
        vel[:, idx] = state.vel.flatten()
        acc[:, idx] = state.acc.flatten()
        angvel[:, idx] = state.angvel.flatten()
        angacc[:, idx] = state.angacc.flatten()

    print('Plotting position...')
    fig, axs = plt.subplots(3)
    for i in range(0, 3):
        axs[i].plot(times, pos[i,:], linewidth=3)
        axs[i].plot(times, correct_pos[i,:])
        axs[i].set_xlabel('Time (s)')
        axs[i].set_ylabel('position (m)')
    plt.suptitle('position')
    axs[0].legend(['Student solution', 'Correct solution'])
    plt.show()

    print('Plotting velocity...')
    fig, axs = plt.subplots(3)
    for i in range(0, 3):
        axs[i].plot(times, vel[i,:], linewidth=3)
        axs[i].plot(times, correct_vel[i,:])
        axs[i].set_xlabel('Time (s)')
        axs[i].set_ylabel('velocity (m/s)')
    plt.suptitle('velocity')
    axs[0].legend(['Student solution', 'Correct solution'])
    plt.show()

    print('Plotting acceleration...')
    fig, axs = plt.subplots(3)
    for i in range(0, 3):
        axs[i].plot(times, acc[i,:], linewidth=3)
        axs[i].plot(times, correct_acc[i,:])
        axs[i].set_xlabel('Time (s)')
        axs[i].set_ylabel('acc (m/$s^2$)')
    plt.suptitle('acceleration')
    axs[0].legend(['Student solution', 'Correct solution'])
    plt.show()

    print('Plotting angular velocity...')
    fig, axs = plt.subplots(3)
    for i in range(0, 3):
        axs[i].plot(times, angvel[i,:], linewidth=3)
        axs[i].plot(times, correct_angvel[i,:])
        axs[i].set_xlabel('Time (s)')
        axs[i].set_ylabel('ang vel (rad/s)')
    plt.suptitle('angular velocity')
    axs[0].legend(['Student solution', 'Correct solution'])
    plt.show()

    print('Plotting angular acceleration...')
    fig, axs = plt.subplots(3)
    for i in range(0, 3):
        axs[i].plot(times, angacc[i,:], linewidth=3)
        axs[i].plot(times, correct_angacc[i,:])
        axs[i].set_xlabel('Time (s)')
        axs[i].set_ylabel('ang acc (rad/$s^2$)')
    plt.suptitle('angular acceleration')
    axs[0].legend(['Student solution', 'Correct solution'])
    plt.show()

if __name__ == "__main__":
    DATA_DIR = qs_path + "/data/"
    config = qs_path + "/config/rocky0704_model_params.yaml"
    test_calculate_force_and_torque_from_rpm(config)
    test_quaternion_derivative(config)
    test_calculate_world_frame_linear_acceleration(config)
    test_calculate_angular_acceleration(config)
    test_ode_step(config, DATA_DIR + "2023-05-28-07-18-08_quadrotor_model.npz")