# 16-362 Assignment 1: Quadrotor Dynamics and Control (Total: 90 points)

Goals: In this assignment, you will implement the mathematical model
of a quadrotor's dynamics as well as a position and attitude
controller to track simple trajectories.

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
pip install scipy yaml numpy rosbags matplotlib
```
Sample data is provided at [TODO](). You may download
this data using the download script in the `data` directory.
```python
./download.sh
```
To receive full credit on this portion of the assignment,
you will need to implement the following three functions:
`ode_step`
`calculate_world_frame_linear_acceleration`, and
`calculate_angular_acceleration_com_offset`. Each function is worth 10 points.

### 1.1 `calculate_world_frame_linear_acceleration` (10 points)
In this function you will implement Equation (4.2) from Daniel Mellinger's PhD thesis.

### 1.2 `calculate_angular_acceleration_com_offset` (10 points)
In this function you will implement Equation (4.3) from Daniel Mellinger's PhD thesis.

### 1.3 `ode_step` (10 points)
This function implements the equations of motion for the quadrotor
dynamics model. The ODE solver is used to integrate the equations over
a period of time. The other two functions (1.1) and (1.2) will be
called in `ode_step`.

In this function, you will need to implement the following:
* Convert commanded RPMs (coming from the controller) to desired force and torques
* calculate the angular acceleration (see 1.2)
* calculate the linear acceleration (see 1.1)
* calculate the derivative of the quaternion using Equation (7) of [this reference](https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=6669617&tag=1).
* calculate the achieved RPMs

### 1.4 Testing your code
To test your solution, run the `TestQuadrotorModel.py` script in the
`unittest` folder. Your solution will be plotted against a reference
solution. You should expect an error of around 1e8 or for all states.

TODO: add image here with expected output.

## 2. Position Controller (30 points)

## 3. Attitude Controller (30 points)

## 4. Grading with AutoLab
