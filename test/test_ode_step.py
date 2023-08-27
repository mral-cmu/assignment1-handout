import os
import unittest
import numpy as np
import math
from sklearn.metrics import mean_squared_error
import matplotlib.pyplot as plt
import sys

sys.path.append('../')

from quadrotor_simulator_py.quadrotor_control import State
from quadrotor_simulator_py.utils import Rot3
from quadrotor_simulator_py.quadrotor_model import QuadrotorModel
from quadrotor_simulator_py.utils.quaternion import Quaternion

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
    plt.suptitle('position')
    axs[0].legend(['Student solution', 'Correct solution'])
    plt.show()

    print('Plotting velocity...')
    fig, axs = plt.subplots(3)
    for i in range(0, 3):
        axs[i].plot(times, vel[i,:], linewidth=3)
        axs[i].plot(times, correct_vel[i,:])
    plt.suptitle('velocity')
    axs[0].legend(['Student solution', 'Correct solution'])
    plt.show()

    print('Plotting acceleration...')
    fig, axs = plt.subplots(3)
    for i in range(0, 3):
        axs[i].plot(times, acc[i,:], linewidth=3)
        axs[i].plot(times, correct_acc[i,:])
    plt.suptitle('acceleration')
    axs[0].legend(['Student solution', 'Correct solution'])
    plt.show()

    print('Plotting angular velocity...')
    fig, axs = plt.subplots(3)
    for i in range(0, 3):
        axs[i].plot(times, angvel[i,:], linewidth=3)
        axs[i].plot(times, correct_angvel[i,:])
    plt.suptitle('angular velocity')
    axs[0].legend(['Student solution', 'Correct solution'])
    plt.show()

    print('Plotting angular acceleration...')
    fig, axs = plt.subplots(3)
    for i in range(0, 3):
        axs[i].plot(times, angacc[i,:], linewidth=3)
        axs[i].plot(times, correct_angacc[i,:])
    plt.suptitle('angular acceleration')
    axs[0].legend(['Student solution', 'Correct solution'])
    plt.show()

DATA_DIR = "../data/"
config = "../config/rocky0704_model_params.yaml"
test_ode_step(config, DATA_DIR + "2023-05-28-07-18-08_quadrotor_model.npz")
