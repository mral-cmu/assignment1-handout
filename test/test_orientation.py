import numpy as np
import sys

sys.path.append('../')

from quadrotor_simulator_py.utils import Pose
from quadrotor_simulator_py.utils import Quaternion
from quadrotor_simulator_py.utils import Rot3

correct = np.array([2, -0.8, 3.14])
R = Rot3().from_euler_zyx(correct)
zyx = R.to_euler_zyx()
assert (np.max(np.abs(correct - zyx)) < 1e-8)

quat = R.to_quat()
R2 = Rot3().from_quat(quat)
quat2 = R2.to_quat()
assert (np.max(np.abs(quat2.data - quat.data)))
zyx2 = R2.to_euler_zyx()
assert (np.max(np.abs(zyx2-zyx)))

print('Tests passed')
