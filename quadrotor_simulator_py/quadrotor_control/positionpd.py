#!/usr/bin/env python
import numpy as np
from numpy import sin, cos
from numpy.linalg import norm
import yaml

from quadrotor_simulator_py.quadrotor_control.state import State
from quadrotor_simulator_py.quadrotor_control.trackingerror import TrackingError
from quadrotor_simulator_py.quadrotor_model.mixer import QuadMixer
from quadrotor_simulator_py.quadrotor_control.cascaded_command import CascadedCommand
from quadrotor_simulator_py.utils import Quaternion
from quadrotor_simulator_py.utils import shortest_angular_distance


class QuadrotorPositionControllerPD:

    def __init__(self, yaml_file):
        self.zw = np.array([[0], [0], [1]])  # unit vector [0, 0, 1]^{\top}
        self.gravity_norm = 9.81
        self.current_state = State()
        self.state_ref = State()
        self.tracking_error = TrackingError()

        self.Rdes = np.eye(3)
        self.Rcurr = None
        self.accel_des = 0.0
        self.angvel_des = np.zeros((3, 1))
        self.angacc_des = np.zeros((3, 1))
        self.mass = 0.0

        data = []
        with open(yaml_file, 'r') as stream:
            try:
                data = yaml.safe_load(stream)
            except yaml.YamlError as exc:
                print(exc)

        self.mass = data['mass']

        self._Kx = np.eye(3)
        self._Kx[0, 0] = data['gains']['pos']['x']
        self._Kx[1, 1] = data['gains']['pos']['y']
        self._Kx[2, 2] = data['gains']['pos']['z']

        self._Kv = np.eye(3)
        self._Kv[0, 0] = data['gains']['vel']['x']
        self._Kv[1, 1] = data['gains']['vel']['y']
        self._Kv[2, 2] = data['gains']['vel']['z']

        self.gravity_norm = data['gravity_norm']

    def update_state(self):
        self.Rcurr = self.current_state.rot

    def get_reference_state(self):
        pass

    def set_current_state(self, state_in):
        self.current_state = state_in
        self.update_state()

    def get_state(self):
        return self.current_state

    def set_reference_state(self, ref_in):
        self.state_ref = ref_in

    def compute_body_z_accel(self, a_des, R_curr):

        # TODO: Fill me in

        return 0

    def compute_hod_refs(self, acc_vec_des, flat_ref, R_des):

        # TODO: Fill me in

        angvel_des = np.zeros((3, 1))
        angacc_des = np.zeros((3, 1))
        return (angvel_des, angacc_des)

    def compute_orientation(self, a_des, yaw_ref):

        # TODO: Fill me in

        R_des = np.eye(3)
        return R_des

    def compute_command(self):

        # TODO: Fill me in

        pass

    def get_cascaded_command(self):
        casc_cmd = CascadedCommand()
        casc_cmd.thrust_des = self.mass * self.accel_des
        casc_cmd.Rdes = self.Rdes
        casc_cmd.angvel_des = self.angvel_des
        casc_cmd.angacc_des = self.angacc_des
        return casc_cmd

    def get_tracking_error(self):
        return self.tracking_error

    def update_tracking_error(self):
        self.tracking_error = TrackingError()
        self.tracking_error.pos_des = self.state_ref.pos
        self.tracking_error.vel_des = self.state_ref.vel
        self.tracking_error.acc_des = self.state_ref.acc
        self.tracking_error.jerk_des = self.state_ref.jerk
        self.tracking_error.snap_des = self.state_ref.snap
        self.tracking_error.yaw_des = self.state_ref.yaw
        self.tracking_error.dyaw_des = self.state_ref.dyaw
        self.tracking_error.pos_err = self.current_state.pos - self.state_ref.pos
        self.tracking_error.vel_err = self.current_state.vel - self.state_ref.vel
        self.tracking_error.yaw_err = shortest_angular_distance(
            self.current_state.yaw, self.state_ref.yaw)
        self.tracking_error.dyaw_err = self.current_state.dyaw - self.state_ref.dyaw

    def run_ctrl(self):

        # get updated state
        self.update_state()

        # get updated reference
        self.get_reference_state()

        # calculate the command
        self.compute_command()

        # update tracking error
        self.update_tracking_error()
