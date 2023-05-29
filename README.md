# 16-362 Assignment 1: Quadrotor Dynamics and Control (Total: 90 points)

Goals: In this assignment, you will implement the mathematical model
of a quadrotor's dynamics as well as a position and attitude
controller to track simple trajectories.

## 0. Rotations (20 points)
The directory containing rotation representations is in `rotation3.py`.
In this part of the assignment you will write the code to perform
the following conversions:

* Rotation matrix (3x3) -> Euler ZYX angles: `to_euler_zyx`
* Euler ZYX -> Rotation matrix (3x3): `from_euler_zyx`
* Rotation matrix (3x3) -> Quaternion (w,x,y,z): `to_quat`
* Quaternion (w,x,y,z) -> Rotation matrix (3x3): `from_quat`

All functions are contained within the rotation3.py file.
More information about each function follows.
Your code will be graded using Autolab. See Section 4 for
details about uploading and receiving scores for your
implementations.

### `to_euler_zyx` (4 points)
This function calculates the angles `phi=X`, `theta=Y`, `psi=Z` that
represent the rotation in the Z-Y-X Tait-Bryant
parameterization. The expected output is a 1x3 numpy array.  Note: the
expected output is in reverse order from functions like MATLAB's
`rotm2eul`; however, you can use this function to check your results.

### `from_euler_zyx` (4 points)
This function calculates the 3x3 rotation matrix from the input angles
`phi=X`, `theta=Y`, and `psi=Z`.

### `to_quat` (6 points)
This function calculates the (w,x,y,z) quaternion from a 3x3 rotation
matrix.

### `from_quat` (6 points)
This function calculates the 3x3 rotation matrix from a
(w,x,y,z)-parameterized quaternion.

## 1. Quadrotor Dynamics Simulator (30 points)
The quadrotor dynamics simulator is contained in
`quadrotor_simulator_py`. You will implement functions in this
folder, zip your folder, and upload to Autolab for grading.

Test data and expected solutions are available for local testing.

### 1.0 Setup
Create a python virtual environment.
```python
python3.8 -m .venv venv
```
Source the environment
```python
source .venv/bin/activate
```
You will need to install the following dependencies.
```python
pip install scipy yaml numpy matplotlib scikit-learn
```
Sample data is provided at [TODO](). You may download
this data using the download script in the `data` directory.
```python
./download.sh
```
To receive full credit on this portion of the assignment,
you will need to implement the following four functions:

* `construct_mixer` (5 points)
* `calculate_world_frame_linear_acceleration` (5 points)
* `calculate_angular_acceleration_com_offset` (5 points)
* `ode_step` (15 points)

### 1.1 `construct_mixer` (5 points)
This function implements the mixer matrix as described in the lecture
slides.

### 1.1 `calculate_world_frame_linear_acceleration` (5 points)
In this function you will implement Equation (4.2) from [1].

### 1.2 `calculate_angular_acceleration_com_offset` (5 points)
In this function you will implement Equation (4.3) from [1].

### 1.3 `ode_step` (15 points)
This function implements the equations of motion for the quadrotor
dynamics model. The ODE solver is used to integrate the equations over
a period of time. The other two functions (1.1) and (1.2) will be
called in `ode_step`.

In this function, you will need to implement the following:
* Convert commanded RPMs (coming from the controller) to desired force and torques
* calculate the angular acceleration (see 1.3)
* calculate the linear acceleration (see 1.2)
* calculate the derivative of the quaternion using Equation (7) of [2].
* calculate the achieved RPMs

### 1.4 Testing your code
Scripts are provided to help debug your code in the `tests` folder.

TODO: add image here with expected output.

## 2. Position Controller (25 points)

## 3. Attitude Controller (25 points)

## 4. Grading with AutoLab
To have your solutions graded, you will need to tar the `quadrotor_simulator_py`
folder and upload to autolab.

```
tar -cvf quadrotor_simulator_py handin.tar
```

Autolab will run tests on each function you implement and you will
receive a score out of 100.  You may upload as many times as you like.
Note that we may regrade submissions after the deadline passes.

## References
[1] D. W. Mellinger, "Trajectory Generation and Control for Quadrotors" (2012). Publicly Accessible Penn Dissertations. 547. [https://repository.upenn.edu/edissertations/547](https://repository.upenn.edu/edissertations/547).

[2] E. Fresk and G. Nikolakopoulos, "Full quaternion based attitude control for a quadrotor," 2013 European Control Conference (ECC), Zurich, Switzerland, 2013, pp. 3864-3869, [doi: 10.23919/ECC.2013.6669617](https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=6669617).
