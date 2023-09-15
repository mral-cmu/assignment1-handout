import numpy as np
import sys
import yaml

sys.path.append('../')

from quadrotor_simulator_py.quadrotor_model import QuadrotorModel
from quadrotor_simulator_py.quadrotor_model import ModelParameters

m = ModelParameters()

data = []
with open("../config/rocky0704_model_params.yaml", 'r') as stream:
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
m.inertia = tmp1 + tmp2 + np.transpose(tmp2)
m.inertia_inv = np.linalg.inv(m.inertia)

m.cg_offset = np.zeros((3,1))

moments = np.array([[-0.00413571], [-0.03648809],  [0.00625934]])
wb = np.array([[-0.1887015],  [-0.18605857],  [0.58317804]])
Fdes = np.array([[ 0.],          [0.],         [26.81591134]])
correct_ang_acc = np.array([[-0.26308854], [-2.12621283], [ 0.28804729]])

q = QuadrotorModel()
ang_acc = q.calculate_angular_acceleration(m, moments, wb, Fdes)
print(ang_acc)
assert (np.max(np.abs(correct_ang_acc - ang_acc)) < 1e-6)
print('Tests passed')
