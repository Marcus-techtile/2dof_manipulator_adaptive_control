# 2DOF Manipulator Control
Implementation of robust RBF control for a 2-DOF robotic manipulator.

## Dependencies
This project uses the robust_rbf_control framework as a submodule.

## Setup
1. Clone this repository
2. Initialize submodules:
   ```bash
   git submodule update --init --recursive
   ```

## Results
The implementation includes three main test scenarios:

### 1. Sinusoidal Noise Test
Located in `results/sinusoidal_noise/`:
- Tracking performance analysis of joint positions and velocities under sinusoidal disturbances
- Error analysis showing the effectiveness of adaptive control
- RBFNN uncertainty approximation demonstrating the learning capability

### 2. External Force Noise Test
Located in `results/external_force_noise/`:
- Performance under external force disturbances
- Error analysis with force disturbances
- RBFNN uncertainty approximation showing robust adaptation

### 3. Non-Adaptive Control Test
Located in `results/no_adaptive/`:
- Baseline performance without RBFNN adaptation
- Comparison of tracking errors with and without adaptation
- Demonstrates the importance of adaptive control in handling uncertainties

## Simulation Demonstrations
The project includes two simulation demonstrations showing the manipulator's motion:

### With Adaptive RBFNN Control
The following animation shows the manipulator's motion with adaptive RBFNN control, demonstrating robust tracking performance under disturbances.

![2DOF Manipulator with Adaptive Control](gif/2dof_manipulator.gif)

### Without Adaptive Control
This animation shows the manipulator's motion without adaptive control, highlighting the performance degradation in the presence of uncertainties.

![2DOF Manipulator without Adaptive Control](gif/2dof_manipulator_no_adaptive.gif)

## Visualization Details
The results include:
- Joint positions (q1, q2) and their desired trajectories
- Joint velocities (q1_dot, q2_dot) and their desired trajectories
- Tracking errors for both position and velocity
- RBFNN's approximation of system uncertainties (except for non-adaptive case)
