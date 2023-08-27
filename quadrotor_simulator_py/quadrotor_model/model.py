#!/usr/bin/env python
import numpy as np
import yaml
import scipy
from scipy import integrate
from numpy import matmul

from quadrotor_simulator_py.quadrotor_control.state import State
from quadrotor_simulator_py.quadrotor_model.mixer import QuadMixer
from quadrotor_simulator_py.utils.quaternion import Quaternion
from quadrotor_simulator_py.utils.pose import Pose
from quadrotor_simulator_py.utils.pose import Rot3


class RPMParameters:
    min = 0.0
    max = 0.0

    def __init__(self):
        pass

    def __repr__(self):
        return ('RPMParameters\n' +
                'min:' + str(self.min) + '\n' +
                'max:' + str(self.max) + '\n')


class ModelParameters:

    def __init__(self):
        self.gravity_norm = 9.81
        self.mass = 2.0
        self.rpm_params = RPMParameters()
        self.mixer = np.zeros((4, 6))
        self.inertia = np.zeros((3, 3))
        self.inertia_inv = np.zeros((3, 3))
        self.aw = np.zeros((3, 1))
        self.ang_acc = np.zeros((3, 1))
        self.uRPM = np.zeros((4, 1))

        self.kmotor_u = 0.0
        self.kmotor_d = 0.0

        # _motor_model(0) * (RPM)^2 + _motor_model(1)*(RPM) + _motor_model(2)
        self.motor_model = np.zeros((3, 1))

        # _cg_offset is the position vector from center of gravity to geometric center.
        self.cg_offset = np.zeros((3, 1))

    def __repr__(self):
        return ('ModelParameters\n' +
                'gravity norm:\t\t' + str(self.gravity_norm) + '\n' +
                'mass:\t\t\t' + str(self.mass) + '\n' +
                'mixer:\n' + np.array2string(self.mixer) + '\n' +
                'inertia:\n' + np.array2string(self.inertia) + '\n' +
                'inertia_inv:\n' + np.array2string(self.inertia_inv) + '\n' +
                'aw:\t\t\t' + np.array2string(np.transpose(self.aw)) + '\n' +
                'ang_acc:\t\t' + np.array2string(np.transpose(self.ang_acc)) + '\n' +
                'uRPM:\t\t\t' + np.array2string(np.transpose(self.uRPM)) + '\n' +
                'kmotor_u:\t\t\t' + str(self.kmotor_u) + '\n' +
                'kmotor_d:\t\t\t' + str(self.kmotor_d) + '\n\n' +
                self.rpm_params.__repr__()
                )


class QuadrotorModel:
    def __init__(self):
        self.mixer = QuadMixer()

        self.Twb = Pose()

        self.vw = np.zeros((3, 1))
        self.aw = np.zeros((3, 1))
        self.wb = np.zeros((3, 1))
        self.ang_acc = np.zeros((3, 1))
        self.rs = np.zeros((4, 1))

        self.model_params = ModelParameters()
        self.rpm_params = RPMParameters()

        self.num_rotors = 4
        self.time = 0.0
        self.tstart = 0.0
        self.zoffset = 0.0

        self.use_cm_offset = True
        self.enable_rotor_inertia = True

    def initialize(self, yaml_file):

        data = []
        with open(yaml_file, 'r') as stream:
            try:
                data = yaml.safe_load(stream)
            except yaml.YamlError as exc:
                print(exc)

            self.model_params.mass = data['mass']
            self.model_params.gravity_norm = data['gravity_norm']

            tmp1 = np.zeros((3, 3))
            tmp2 = np.zeros((3, 3))
            tmp1[0, 0] = data['inertia']['Ixx']
            tmp1[1, 1] = data['inertia']['Iyy']
            tmp1[2, 2] = data['inertia']['Izz']
            tmp2[0, 1] = data['inertia']['Ixy']
            tmp2[0, 2] = data['inertia']['Ixz']
            tmp2[1, 2] = data['inertia']['Iyz']
            self.model_params.inertia = tmp1 + tmp2 + np.transpose(tmp2)
            self.model_params.inertia_inv = np.linalg.inv(
                self.model_params.inertia)

            self.model_params.motor_model[0] = data['rotor']['cT2']
            self.model_params.motor_model[1] = data['rotor']['cT1']
            self.model_params.motor_model[2] = data['rotor']['cT0']

            self.model_params.kmotor_u = data['rotor']['time_constant']
            self.model_params.kmotor_d = data['rotor']['time_constant']

            self.zoffset = data['zoffset']

            # TODO: Fix in next iteration
            self.model_params.cg_offset[0] = 0.0
            self.model_params.cg_offset[1] = 0.0
            self.model_params.cg_offset[2] = 0.0

            self.model_params.rpm_params.min = data['rpm']['min']
            self.model_params.rpm_params.max = data['rpm']['max']

            self.num_rotors = 4

            self.mixer.initialize(yaml_file)
            self.model_params.mixer = self.mixer.construct_mixer()

            self.enable_rotor_inertia = data['rotor']['enable_inertia']

    def __repr__(self):
        return ('QuadrotorModel\n' +
                'Twb:\n' + np.array2string(self.Twb.get_se3().as_matrix()) + '\n' +
                'vw:' + np.array2string(np.transpose(self.vw)) + '\n' +
                'aw:' + np.array2string(np.transpose(self.aw)) + '\n' +
                'wb:' + np.array2string(np.transpose(self.wb)) + '\n' +
                'ang_acc:' + np.array2string(np.transpose(self.ang_acc)) + '\n' +
                'rs:' + np.array2string(np.transpose(self.rs)) + '\n' +
                'num_rotors:\t\t\t' + str(self.num_rotors) + '\n' +
                'time:\t\t\t\t' + str(self.time) + '\n' +
                'tstart:\t\t\t\t' + str(self.tstart) + '\n' +
                'zoffset:\t\t\t' + str(self.zoffset) + '\n' +
                'use cm offset:\t\t\t' + str(self.use_cm_offset) + '\n' +
                self.model_params.__repr__()
                )

    def reset_simulation(self):
        self.model_params.uRPM = np.zeros((4, 1))
        self.set_pose(np.array([0, 0, 0, 1, 0, 0, 0]))
        self.vw = np.zeros((3, 1))
        self.aw = np.zeros((3, 1))
        self.aw[2, 0] = -self.model_params.gravity_norm
        self.wb = np.zeros((3, 1))
        self.rs = np.zeros((4, 1))
        self.time = 0.0

    def apply_command(self, uRPM):
        self.model_params.uRPM = uRPM

    def gravity_norm(self):
        return self.model_params.gravity_norm

    def get_pose(self):
        return self.Twb

    def set_pose(self, Twb):
        self.Twb = Pose(Twb)

    def calculate_world_frame_linear_acceleration(self, model, ang_acc, wb, Rwb, u1):
        """ Calculates the linear acceleration of the aerial robot.
                Hint: Use Equation (4.2) of Daniel Mellinger's PhD thesis
                "Trajectory Generation and Control for Quadrotors"

        Args:
            model: These are the self.model_params. This will give you the
                offset of the center of mass in body frame coordinates (r_{off})
            ang_acc: 3x1 numpy array representing angular acceleration
            wb: 3x1 numpy array representing body-frame angular velocities
            Rwb: 3x3 numpy matrix representing rotation
            u1: scalar representing vehicle thrust

        Output:
            lin_acc: 3x1 numpy array representing linear acceleration
        """

        # TODO: Assignment 1, Problem 2.2

        return np.zeros((3, 1))

    def calculate_angular_acceleration(self, model, moments, wb, Fdes):
        """ Calculates the vehicle angular acceleration.
                Hint: Use Equation (4.3) of Daniel Mellinger's PhD thesis
                "Trajectory Generation and Control for Quadrotors"

        Args:
            model: These are the self.model_params. This will give you the
                offset of the center of mass in body frame coordinates (r_{off})
            moments: 3x1 numpy array representing the moments
            wb: 3x1 numpy array representing body-frame angular velocities
            Fdes: scalar representing desired vehicle thrust

        Output:
            ang acc: 3x1 numpy array representing angular acceleration
        """

        # TODO: Assignment 1, Problem 2.3

        return np.zeros((3, 1))

    def ode_step(self, t, x):
        """ Numerically integrates the robot dynamics given an initialize
                condition.

        Args:
            t: interval over which to integrate [tstart, tstop]
            x: 1x17 numpy array representing the following
                x[0:3]: translation
                x[3:7]: orientation represented as quaternion
                x[7:10]: linear velocity (world frame)
                x[10:13]: body-frame angular velocity
                x[13:17]: rotor speeds (in RPM)

        Output:
            xdot: 1x17 numpy array containing the following
                xdot[0:3]: world frame linear velocity
                xdot[3:7]: derivative of quaternion
                xdot[7:10]: linear acceleration (world frame)
                xdot[10:13]: angular acceleration (body frame)
                xdot[13:17]: derivative of rotor speeds
        """

        # TODO: Assignment 1, Problem 2.4

        xdot = np.zeros((17, 1))
        return xdot.flatten()

    def _saturate_rpms(self, p, rpm_in):
        rpms = np.zeros((4, 1))
        for i in range(0, 4):
            rpms[i] = np.max(np.min(rpm_in[0], p.max), p.min)
        return rpms

    def update(self, t):

        tstart = self.time
        tstop = t
        t_span = [tstart, tstop]

        ss = np.zeros((17))

        ss[0:3] = self.get_pose().translation().flatten()
        ss[3:7] = self.get_pose().quaternion().flatten()
        ss[7:10] = np.transpose(self.vw).flatten()
        ss[10:13] = np.transpose(self.wb).flatten()
        ss[13:17] = np.transpose(self.rs).flatten()

        sol = integrate.solve_ivp(self.ode_step, t_span, ss)
        self.time = tstop

        ss = sol.y[:, -1]
        self.set_pose(ss[0:7])
        self.vw = ss[7:10]
        self.wb = ss[10:13]

        if self.enable_rotor_inertia:
            self.rs = ss[13:17]
        else:
            self.rs = self.model_params.uRPM

        self.aw = self.model_params.aw
        self.ang_acc = self.model_params.ang_acc

        self.check_collision()

    def check_collision(self):
        if self.Twb.translation()[2] <= self.zoffset:
            self.Twb.set_translation(np.array([0, 0, self.zoffset]))
            self.vw = np.array([0, 0, 0])
            self.wb = np.array([0, 0, 0])
            self.aw = np.array([0, 0, -self.model_params.gravity_norm])
            self.ang_acc = np.array([0, 0, 0])

    def get_state(self):
        s = State()
        s.pos = np.reshape(self.get_pose().translation(), (3, 1))
        s.vel = np.reshape(self.vw, (3, 1))
        s.acc = np.reshape(self.aw, (3, 1))
        s.rot = self.get_pose().get_so3()
        s.angvel = np.reshape(self.wb, (3, 1))
        s.angacc = np.reshape(self.ang_acc, (3, 1))
        s.yaw = Rot3(self.get_pose().get_so3()).yaw()
        return s
