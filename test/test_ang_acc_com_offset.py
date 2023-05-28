import sys
import numpy as np
from sklearn.metrics import mean_squared_error

from quadrotor_simulator_py.quadrotor_model import QuadrotorModel

def test_ang_acc_com_offset(config):
    quad_model = QuadrotorModel()
    quad_model.initialize(config)

    moments = np.array([[-0.0070535 ], [ 0.00637693], [-0.00206956]])
    wb = np.array([[-0.3478447 ], [-0.38363534], [ 0.5314286 ]])
    Fdes = np.array([[ 0.        ], [ 0.        ], [26.28933891]])
    correct_angacc = np.array([[-0.44374443], [ 0.29319748], [-0.11888338]])
    angacc = quad_model.calculate_angular_acceleration_com_offset(quad_model.model_params,
                                                                  moments, wb, Fdes)
    err = np.sqrt(mean_squared_error(correct_angacc, angacc))
    eps = 1e-3
    if err < eps:
        print('Angular Acceleration Calculation Passed.')
        return 5
    print('Angular Acceleration Calculation Failed. Error ' + str(err) + ' >= ' + str(eps))
    return 0
