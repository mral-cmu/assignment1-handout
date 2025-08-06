import numpy as np
import sys

import os
qs_path = os.path.dirname(os.path.abspath(__file__))+'/..'
sys.path.append(qs_path)

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

R3 = np.array([[ 0.1691,    0.3851,   -0.9072],
               [-0.0170,    0.9215,    0.3880],
               [ 0.9854,   -0.0502,    0.1624]])
rpy3 = np.array([ -0.3, -1.4,  -0.1])
q3 = np.array([0.7505, -0.1460, -0.6305, -0.1339])

R4 = np.array([[-0.0858,   -0.6806,    0.7276],
               [-0.1467,   -0.7137,   -0.6849],
               [ 0.9854,   -0.1655,   -0.0386]])
rpy4 = np.array([ -1.8, -1.4,  -2.1])
q4 = np.array([-0.2012, -0.6455, 0.3204, -0.6635])

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

def check_quat_soln(student_soln, correct_soln):
    score = 0.
    try:
        if np.linalg.norm(student_soln-correct_soln) < eps:
            score += 0.25
            print('passed')
        elif np.linalg.norm(student_soln-correct_soln*-1) < eps:
            score += 0.25
            print('passed')
        else:
            print('failed')
    except Exception as e:
            print('failed')

    return score

def test_to_euler_zyx():
    score = 0.
    print(f"to_euler_zyx test 1...", end="")
    r = R1
    student_soln = Rot3(r).to_euler_zyx()
    correct_soln = np.array(rpy1)
    score+=check_soln(student_soln,correct_soln)

    print(f"to_euler_zyx test 2...", end="")
    r = R2
    student_soln = Rot3(r).to_euler_zyx()
    correct_soln = rpy2
    score+=check_soln(student_soln,correct_soln)

    print(f"to_euler_zyx test 3...", end="")
    r = R3
    student_soln = Rot3(r).to_euler_zyx()
    correct_soln = rpy3
    score+=check_soln(student_soln,correct_soln)

    print(f"to_euler_zyx test 4...", end="")
    r = R4
    student_soln = Rot3(r).to_euler_zyx()
    correct_soln = rpy4
    score+=check_soln(student_soln,correct_soln)
    return score


def test_from_euler_zyx():
    score = 0.
    print(f"from_euler_zyx test 1...", end="")
    r = rpy1
    student_soln = Rot3().from_euler_zyx(r).R
    correct_soln = R1
    score+=check_soln(student_soln,correct_soln)
 
    print(f"from_euler_zyx test 2...", end="")
    r = rpy2
    student_soln = Rot3().from_euler_zyx(r).R
    correct_soln = R2
    score+=check_soln(student_soln,correct_soln)

    print(f"from_euler_zyx test 3...", end="")
    r = rpy3
    student_soln = Rot3().from_euler_zyx(r).R
    correct_soln = R3
    score+=check_soln(student_soln,correct_soln)

    print(f"from_euler_zyx test 4...", end="")
    r = rpy4
    student_soln = Rot3().from_euler_zyx(r).R
    correct_soln = R4
    score+=check_soln(student_soln,correct_soln)
    return score

def test_roll():
    score = 0.
    print(f"roll test 1...", end="")
    R = R1
    student_soln = np.array([Rot3(R).roll()])
    correct_soln = np.array([rpy1[0]])
    score+=check_soln(student_soln,correct_soln)

    print(f"roll test 2...", end="")
    R = R2
    student_soln = np.array([Rot3(R).roll()])
    correct_soln = np.array(rpy2[0])
    score+=check_soln(student_soln,correct_soln)

    print(f"roll test 3...", end="")
    R = R3
    student_soln = np.array([Rot3(R).roll()])
    correct_soln = np.array(rpy3[0])
    score+=check_soln(student_soln,correct_soln)

    print(f"roll test 4...", end="")
    R = R4
    student_soln = np.array([Rot3(R).roll()])
    correct_soln = np.array(rpy4[0])
    score+=check_soln(student_soln,correct_soln)

    return score

def test_pitch():
    score = 0.
    print(f"pitch test 1...", end="")
    R = R1
    student_soln = np.array([Rot3(R).pitch()])
    correct_soln = np.array([rpy1[1]])
    score+=check_soln(student_soln,correct_soln)

    print(f"pitch test 2...", end="")
    R = R2
    student_soln = np.array([Rot3(R).pitch()])
    correct_soln = np.array(rpy2[1])
    score+=check_soln(student_soln,correct_soln)

    print(f"pitch test 3...", end="")
    R = R3
    student_soln = np.array([Rot3(R).pitch()])
    correct_soln = np.array(rpy3[1])
    score+=check_soln(student_soln,correct_soln)

    print(f"pitch test 4...", end="")
    R = R4
    student_soln = np.array([Rot3(R).pitch()])
    correct_soln = np.array(rpy4[1])
    score+=check_soln(student_soln,correct_soln)
    return score

def test_yaw():
    score = 0.
    print(f"yaw test 1...", end="")
    R = R1
    student_soln = np.array([Rot3(R).yaw()])
    correct_soln = np.array([rpy1[2]])
    score+=check_soln(student_soln,correct_soln)

    print(f"yaw test 2...", end="")
    R = R2
    student_soln = np.array([Rot3(R).yaw()])
    correct_soln = np.array(rpy2[2])
    score+=check_soln(student_soln,correct_soln)

    print(f"yaw test 3...", end="")
    R = R3
    student_soln = np.array([Rot3(R).yaw()])
    correct_soln = np.array(rpy3[2])
    score+=check_soln(student_soln,correct_soln)

    print(f"yaw test 4...", end="")
    R = R4
    student_soln = np.array([Rot3(R).yaw()])
    correct_soln = np.array(rpy4[2])
    score+=check_soln(student_soln,correct_soln)
    return score

def test_to_quat():
    score = 0.
    print(f"quat test 1...", end="")
    R = R1
    student_soln = np.array(Rot3(R).to_quat().data)
    correct_soln = Quaternion(q1).data
    score+= check_quat_soln(student_soln,correct_soln)

    print(f"quat test 2...", end="")
    R = R2
    student_soln = np.array([Rot3(R).to_quat().data])
    correct_soln = Quaternion(q2).data
    score+= check_quat_soln(student_soln,correct_soln)

    print(f"quat test 3...", end="")
    R = R3
    student_soln = np.array([Rot3(R).to_quat().data])
    correct_soln = Quaternion(q3).data
    score+= check_quat_soln(student_soln,correct_soln)

    print(f"quat test 4...", end="")
    R = R4
    student_soln = np.array([Rot3(R).to_quat().data])
    correct_soln = Quaternion(q4).data
    score+= check_quat_soln(student_soln,correct_soln)
    return score

def test_from_quat():
    score = 0.
    print(f"quat test 1...", end="")
    R = R1
    correct_soln = R
    student_soln = Rot3().from_quat(Quaternion(q1)).R
    score+=check_soln(student_soln,correct_soln)

    print(f"quat test 2...", end="")
    R = R2
    correct_soln = R
    student_soln = Rot3().from_quat(Quaternion(q2)).R
    score+=check_soln(student_soln,correct_soln)

    print(f"quat test 3...", end="")
    R = R3
    correct_soln = R
    student_soln = Rot3().from_quat(Quaternion(q3)).R
    score+=check_soln(student_soln,correct_soln)

    print(f"quat test 4...", end="")
    R = R4
    correct_soln = R
    student_soln = Rot3().from_quat(Quaternion(q4)).R
    score+=check_soln(student_soln,correct_soln)
    return score



if __name__ == "__main__":
    test_to_euler_zyx()
    test_from_euler_zyx()
    test_roll()
    test_pitch()
    test_yaw()
    test_to_quat()
    test_from_quat()