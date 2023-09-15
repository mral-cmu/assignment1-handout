import numpy as np
import sys

sys.path.append('../')

from quadrotor_simulator_py.quadrotor_model import QuadrotorModel
from quadrotor_simulator_py.quadrotor_model import ModelParameters

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
aw = q.calculate_world_frame_linear_acceleration(m, ang_acc, wb, Rwb, u1)
assert (np.max(np.abs(correct_aw - aw)) < 1e-6)
print('Tests passed')
