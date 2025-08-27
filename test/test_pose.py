import numpy as np
import sys

import os
qs_path = os.path.dirname(os.path.abspath(__file__))+'/..'
sys.path.append(qs_path)

from quadrotor_simulator_py.utils import Pose
from quadrotor_simulator_py.utils import Quaternion
from quadrotor_simulator_py.utils import Rot3

eps = 1e-2
R1 = np.array([[ 0.8253, -0.412 , -0.3862],
               [ 0.0083,  0.6926, -0.7213],
               [ 0.5646,  0.5921,  0.575 ]])
rpy1 = [ 0.8, -0.6,  0.01]
q1 = np.array([0.8793, 0.3734, -0.2703, 0.1195])

R2 = np.array([[0.9999,   -0.0109,   -0.0090],
               [0.0100,    0.9949,   -0.0999],
               [0.0100,    0.0998,    0.9950]])
rpy2 = np.array([ 0.1, -0.01,  0.01])
q2 = np.array([0.9987, 0.0500, -0.0047, 0.0052])

def check_soln(student_soln, correct_soln):
    score = 0.
    try:
        if np.linalg.norm(student_soln-correct_soln) < eps:
            score += 0.25
            print('passed')
        else:
            print('failed')
    except Exception as e:
            print('failed')

    return score

def test_pose_compose():
    print(f"pose compose test...", end="")
    score = 0.
    t1 = np.array([1., 2., 3.])
    data = np.append(t1, q1).T
    pose = Pose(data)

    t2 = np.array([0, 2., 0])

    studentT = pose.compose(Pose(np.append(t2, q2).T))
    correctT = np.array([[ 0.8173114 , -0.45746179 ,-0.35032953  ,0.17592375],
                        [ 0.0080136 ,  0.61696969 ,-0.78694611  ,3.38513088],
                        [ 0.57614048,  0.64037263 , 0.50792228  ,4.18419204],
                        [ 0.        ,  0.         , 0.          ,1.        ]])
    score+=check_soln(studentT.get_se3(),correctT)*4
    return score
    
def test_pose_inverse():
    print(f"pose inverse test...", end="")
    score = 0.
    t1 = np.array([1., 2., 3.])
    data = np.append(t1, q1).T
    pose = Pose(data)
    studentT = pose.inverse()
    correctT = np.array([[ 0.82530456,  0.00829317,  0.56462697, -2.5357718 ],
                         [-0.41203813,  0.69256544,  0.59209602, -2.74938081],
                         [-0.38613077, -0.72130738,  0.57499452,  0.10376198],
                         [ 0.        ,  0.        ,  0.        ,  1.        ]])
    score+=check_soln(studentT.get_se3(),correctT)*4
    return score

if __name__ == "__main__":
    test_pose_compose()
    test_pose_inverse()
