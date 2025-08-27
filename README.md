# 16-362 Assignment 1: Quadrotor Dynamics and Control (Total: 100 points)

Goals: In this assignment, you will implement the mathematical model
of a quadrotor's dynamics as well as a position and attitude
controller to track simple trajectories.

### Academic Integrity
1. Do not publicly share your solution (using GitHub or otherwise)
2. Collaboration is encouraged but you should write final code on your own.

### 0.0 Setup
Create a python virtual environment.
```python
python3.8 -m venv .venv
```
Source the environment
```python
source .venv/bin/activate
```
You will need to install the following dependencies.
```python
pip install scipy pyyaml numpy matplotlib scikit-learn gdown
```
Download the assignment.
```bash
git clone git@github.com:mral-cmu/assignment1-handout.git
```
Note: if the above command results in a `Permission denied (public key)`
error, then try setting up an SSH key in your Github account using the
instructions [here](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account).

If the cloning still does not work, use
```bash
git clone https://github.com/mral-cmu/assignment1-handout.git
```

Sample data is provided to check your answers. You may download
this data using the download script in the `data` directory.
```python
python download.py
```

## 1.0 Rotations and Pose (10 points)
The directory containing rotation representations is in `rotation3.py`.
In this part of the assignment you will write the code to perform
the following conversions:

* Rotation matrix (3x3) -> Euler ZYX angles: `to_euler_zyx`
* Euler ZYX -> Rotation matrix (3x3): `from_euler_zyx`
* Rotation matrix (3x3) -> Quaternion (w,x,y,z): `to_quat`
* Quaternion (w,x,y,z) -> Rotation matrix (3x3): `from_quat`

All functions are contained within the rotation3.py file.
More information about each function follows.
Your code will be graded using Autolab. See Section 5 for
details about uploading and receiving scores for your
implementations.

Functions 1.1 -- 1.7 are found in `rotation3.py`. Functions 1.8 and
1.9 may be found in `pose.py`.

### 1.1 `to_euler_zyx`
This function calculates the angles `phi=X`, `theta=Y`, `psi=Z` that
represent the rotation in the Z-Y-X Tait-Bryant
parameterization. The expected output is a 1x3 numpy array.  Note: the
expected output is in reverse order from functions like MATLAB's
`rotm2eul`; however, you can use this function to check your results.

### 1.2 `from_euler_zyx`
This function calculates the 3x3 rotation matrix from the input angles
`phi=X`, `theta=Y`, and `psi=Z`.

### 1.3 `roll`
This function extracts and returns the phi component from the
rotation matrix.

### 1.4 `pitch`
This function extracts and returns the theta component from the
rotation matrix.

### 1.5 `yaw`
This function extracts and returns the psi component from the
rotation matrix.

### 1.6 `from_quat`
This function calculates the 3x3 rotation matrix from a
(w,x,y,z)-parameterized quaternion.

### 1.7 `to_quat`
This function calculates the (w,x,y,z) quaternion from a 3x3 rotation
matrix.

### 1.8 `compose`
This function calculates the composition of two transforms.

### 1.8 `inverse`
This function calculates the inverse of the homogeneous transform.

## Checking your results
There are two test files for this section. Functions 1.1 -- 1.7
have their test cases in `test/test_orientation.py`. It will print out
```python
Tests Passed
```
on success and assert an error if it fails. Functions 1.8 and 1.9 have
their test cases in `test/test_pose.py`. The test will print out
`passed` or `failure` for each test case.

## 2. Quadrotor Simulator (45 points)
The quadrotor simulator is contained in `quadrotor_simulator_py`. You
will implement functions in this folder, zip your folder, and upload
to Autolab for grading.

Test data and expected solutions are available for local testing.

To receive full credit on this portion of the assignment,
you will need to implement the following four functions:

* `construct_mixer` (5 points)
* `calculate_force_and_torque_from_rpm` (5 points)
* `calculate_quaternion_derivative` (5 points)
* `calculate_world_frame_linear_acceleration` (5 points)
* `calculate_angular_acceleration` (5 points)
* `ode_step` (20 points)

### 2.1 `construct_mixer` (5 points)
This function implements the mixer matrix as described in the lecture
slides. There is no local test for this function. To check your results,
you will need to upload your function to AutoLab.

### 2.2 `calculate_force_and_torque_from_rpm` (5 points)
This function calculates the scalar force and 3x1 torque vector from a
vector of 4 RPM values using the motor model discussed in class.

### 2.3 `quaternion_derivative` (5 points)
This function calculates the derivative of the quaternion using the
formula covered in the slides.

### 2.4 `calculate_world_frame_linear_acceleration` (5 points)
In this function you will implement Equation (4.2) from [1].

### 2.5 `calculate_angular_acceleration` (5 points)
In this function you will implement Equation (4.3) from [1].

### 2.6 `ode_step` (20 points)
This function implements the equations of motion for the quadrotor
dynamics model. The ODE solver is used to integrate the equations over
a period of time. The functions (2.1 -- 2.5) will be called within
this function.

In this function, you will need to implement the following:
* convert commanded RPMs (coming from the controller) to desired force and torques (see 2.2)
* calculate the derivative of the quaternion using the lecture notes. (see 2.3)
* calculate the linear acceleration (see 2.4)
* calculate the angular acceleration (see 2.5)
* calculate the achieved RPMs

## Checking your results
There is a test in `/test/test_ode_step.py`. If your results are
correct, you should see the following output.

![](./img/test_ode_step_pos.png)
![](./img/test_ode_step_vel.png)
![](./img/test_ode_step_acc.png)
![](./img/test_ode_step_angvel.png)
![](./img/test_ode_step_angacc.png)

## 3. Position Controller (25 points)
You will need to write the following functions:

* `compute_body_z_accel`
* `compute_hod_refs`
* `compute_orientation`
* `compute_command`

Detailed instructions for the contents of each function follow:

### 3.1 `compute_body_z_accel`
This function uses the desired acceleration and current rotation to
calculate the body frame z acceleration.  See page 20 of [1] for
implementation details.

The lecture slides on quadrotor control also discuss how to implement
this function.

### 3.2 `compute_orientation`
This function calculates the desired orientation.  Use Equations (33)
-- (36) from [3] to implement this function.

The lecture slides on quadrotor control also discuss how to implement
this function in detail.

### 3.3 `compute_hod_refs`
This function uses the desired acceleration vector, flat reference,
and desired rotation to calculate the desired angular velocities and
accelerations.  Use Equations (14)--(25) of [2] to calculate the
angular velocities.  Use Equation (103)--(105) of [3] to calculate the
angular velocities. Disregard the drag component (i.e., set it to
zero) to make your life easier.

The lecture slides on quadrotor control also discuss how to implement
this function in detail.

### 3.4 `compute_command`
This function contains the following functionality:
1. computes the PD feedback-control terms from the position and
   velocity control errors via Equations (31)--(32) of [2] and where a_{rd} = 0
2. computes the desired rotation `compute_orientation`
3. applies the thrust command to the current body frame `compute_body_z_accel`
4. calculates the desired angular velocities and accelerations, which will
   be used in the inner control loop (detailed in the next section)

![](./img/test_pos_ctrl_angacc.png)
![](./img/test_pos_ctrl_angvel.png)
![](./img/test_pos_ctrl_thrust.png)

## 4. Attitude Controller (20 points)
You will need to write the following functions:

* `wrench_to_rotor_forces`
* `force_to_rpm`
* `run_ctrl`

### 4.1 `wrench_to_rotor_forces`
Uses the inverse of the mixer matrix to calculate rotor forces from
the thrust and torques.

### 4.2 `force_to_rpm`
Uses the forces to calculates the RPMs using the thrust coefficients
and the quadratic formula.

### 4.3 `run_ctrl`
This function contains the following functionality:

1. calculates the rotation error metric (see page 21 of [1])
2. calculates the PD control law discussed in the lecture slides
3. calculates the desired moments by pre-multiplying Equation 2.68 of
   [4] by the inertia matrix.
4. calculates the rotor forces using the `wrench_to_rotor_forces` function
5. calculates rpms from the rotor forces using the `force_to_rpm` function
6. calculates saturated rpms and returns

![](./img/test_att_ctrl.png)

## 5. Grading with AutoLab
To have your solutions graded, you will need to tar the `quadrotor_simulator_py`
folder and upload to autolab.

```
cd assignment1-handout
tar -cvf handin.tar quadrotor_simulator_py
```

Autolab will run tests on each function you implement and you will
receive a score out of 100.  You may upload as many times as you like.
Note that we may regrade submissions after the deadline passes.

## 6. Visualization with ROS2
If you would like to visualize your solution with ROS2, follow the steps
[here](https://github.com/mral-cmu/quadrotor_simulator_ws#readme).

## References
[1] D. W. Mellinger, "Trajectory Generation and Control for Quadrotors" (2012). Publicly Accessible Penn Dissertations. 547. [https://repository.upenn.edu/edissertations/547](https://repository.upenn.edu/edissertations/547).

[2] M. Faessler, A. Franchi, and D. Scaramuzza, "Differential Flatness of Quadrotor Dynamics Subject to Rotor Drag for Accurate Tracking of High-Speed Trajectories." [https://rpg.ifi.uzh.ch/docs/RAL18_Faessler.pdf](https://rpg.ifi.uzh.ch/docs/RAL18_Faessler.pdf)

[3] M. Faessler, A. Franchi, and D. Scaramuzza, "Detailed Derivations of ``Differential Flatness of Quadrotor Dynamics Subject to Rotor Drag for Accurate Tracking of High-Speed Trajectories''". [https://rpg.ifi.uzh.ch/docs/RAL18_Faessler.pdf](https://rpg.ifi.uzh.ch/docs/RAL18_Faessler.pdf)

[4] A. Spitzer, "Dynamical Model Learning and Inversion for Aggressive Quadrotor Flight". CMU-RI-TR-22-03. [https://www.ri.cmu.edu/app/uploads/2022/01/aspitzer_phd_ri_2022.pdf](https://www.ri.cmu.edu/app/uploads/2022/01/aspitzer_phd_ri_2022.pdf)
