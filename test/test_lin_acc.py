import sys
import numpy as np
from sklearn.metrics import mean_squared_error

from quadrotor_simulator_py.quadrotor_model import QuadrotorModel

def test_lin_acc(config):
    quad_model = QuadrotorModel()
    quad_model.initialize(config)

    ang_acc = np.array([[ 1.85237666e-04], [ 4.74824806e-01], [-1.22398858e-04]])
    wb = np.array([[ 1.25978671e-04], [-2.39931021e-01], [ 1.96539658e-06]])
    Rwb = np.array([[ 9.90179391e-01,  3.53055002e-08, -1.39802622e-01],
                    [ 3.99698430e-07,  1.00000000e+00,  3.08348041e-06],
                    [ 1.39802622e-01, -3.10907765e-06,  9.90179391e-01]])
    u1 = 26.266726111863996
    correct_lin_acc = np.array([[-1.38467466e+00], [ 3.05403226e-05], [ 5.78834927e-04]])
    lin_acc = quad_model.calculate_world_frame_linear_acceleration(quad_model.model_params, ang_acc, wb, Rwb, u1)

    err = np.sqrt(mean_squared_error(lin_acc, correct_lin_acc))
    eps = 1e-3

    if err < eps:
        print('Linear Acceleration Calculation Passed.')
        return 5
    print('Linear Acceleration Calculation Failed. Error ' + str(err) + ' >= ' + str(eps))
    return 0




