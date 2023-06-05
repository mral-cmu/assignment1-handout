#!/usr/bin/env python
import numpy as np
import yaml

from quadrotor_simulator_py.quadrotor_control.state import State
from quadrotor_simulator_py.quadrotor_model.mixer import QuadMixer
from quadrotor_simulator_py.utils import Rot3


class QuadrotorAttitudeControllerPD:

    def __init__(self, yaml_file):
        self.cm_offset = np.zeros((3, 1))
        self.inertia = np.eye(3)
        self.inertia_inf = np.eye(3)
        self.current_state = State()
        self.kR = np.zeros((3, 1))
        self.kOm = np.zeros((3, 1))

        self.des_rot = np.eye(3)
        self.des_angvel = np.zeros((3, 1))
        self.des_angacc = np.zeros((3, 1))
        self.thrust_des = 0.0
        self.mixer = np.zeros((4, 4))
        self.mixer_inv = np.zeros((4, 4))
        self.min_rpm = 0.0
        self.max_rpm = 0.0
        self.cT2 = 0.0
        self.cT1 = 0.0
        self.cT0 = 0.0

        data = []
        with open(yaml_file, 'r') as stream:
            try:
                data = yaml.safe_load(stream)
            except yaml.YamlError as exc:
                print(exc)

        tmp1 = np.zeros((3, 3))
        tmp2 = np.zeros((3, 3))
        tmp1[0, 0] = data['inertia']['Ixx']
        tmp1[1, 1] = data['inertia']['Iyy']
        tmp1[2, 2] = data['inertia']['Izz']
        tmp2[0, 1] = data['inertia']['Ixy']
        tmp2[0, 2] = data['inertia']['Ixz']
        tmp2[1, 2] = data['inertia']['Iyz']
        self.inertia = tmp1 + tmp2 + np.transpose(tmp2)
        self.inertia_inv = np.linalg.inv(self.inertia)

        self.cm_offset = np.zeros((3, 1))
        if (data['center_of_mass']['enable']):
            self.cm_offset[0] = data['center_of_mass']['x']
            self.cm_offset[1] = data['center_of_mass']['y']
            self.cm_offset[2] = data['center_of_mass']['z']

        self.kR[0] = data['gains']['rot']['x']
        self.kR[1] = data['gains']['rot']['y']
        self.kR[2] = data['gains']['rot']['z']

        self.kOm[0] = data['gains']['ang']['x']
        self.kOm[1] = data['gains']['ang']['y']
        self.kOm[2] = data['gains']['ang']['z']

        mixer = QuadMixer()
        mixer.initialize(yaml_file)
        self.mixer = mixer.construct_mixer()
        self.mixer_inv = np.linalg.inv(self.mixer)
        self.min_rpm = data['rpm']['min']
        self.max_rpm = data['rpm']['max']
        self.cT2 = data['rotor']['cT2']
        self.cT1 = data['rotor']['cT1']
        self.cT0 = data['rotor']['cT0']

    def inertia(self):
        return self.inertia

    def inertia_inv(self):
        return self.inertia_inv

    def com(self):
        return self.cm_offset

    def set_cascaded_cmd(self, casc_cmd):
        self.set_des_thrust(casc_cmd.thrust_des)
        self.set_des_rot(casc_cmd.Rdes)
        self.set_des_angvel(casc_cmd.angvel_des)
        self.set_des_angacc(casc_cmd.angacc_des)

    def set_des_thrust(self, thrust):
        self.thrust_des = float(thrust)

    def set_des_rot(self, des_rot):
        self.des_rot = des_rot

    def get_des_rot(self):
        return self.des_rot

    def set_des_angvel(self, des_angvel):
        self.des_angvel = des_angvel

    def get_des_angvel(self):
        return self.des_angvel

    def set_des_angacc(self, des_angacc):
        self.des_angacc = des_angacc

    def set_current_state(self, s):
        self.current_state = s

    def wrench_to_rotor_forces(self, thrust, torque):

        # TODO: Fill me in

        rotor_forces = np.zeros((4, 1))
        return rotor_forces

    def force_to_rpm(self, forces):

        # TODO: Fill me in

        uW = np.zeros((4, 1))
        return uW

    def saturate_rpm(self, rpm_in):
        rpm_out = rpm_in
        for i in range(0, np.shape(rpm_in)[0]):
            if (rpm_out[i, 0] < self.min_rpm):
                rpm_out[i, 0] = self.min_rpm
            elif (rpm_out[i, 0] > self.max_rpm):
                rpm_out[i, 0] = self.max_rpm
        return rpm_out

    def run_ctrl(self):
        return np.zeros((4, 1))
