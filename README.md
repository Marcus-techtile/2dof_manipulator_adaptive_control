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

## Videos
The project includes two simulation videos demonstrating the manipulator's motion:

### With Adaptive RBFNN Control
The following video shows the manipulator's motion with adaptive RBFNN control, demonstrating robust tracking performance under disturbances.

<video width="640" height="360" controls>
  <source src="video/2dof_manipulator.mp4" type="video/mp4">
  Your browser does not support the video tag.
</video>

### Without Adaptive Control
This video shows the manipulator's motion without adaptive control, highlighting the performance degradation in the presence of uncertainties.

<video width="640" height="360" controls>
  <source src="video/2dof_manipulator_no_adaptive.mp4" type="video/mp4">
  Your browser does not support the video tag.
</video>

## Visualization Details
The results include:
- Joint positions (q1, q2) and their desired trajectories
- Joint velocities (q1_dot, q2_dot) and their desired trajectories
- Tracking errors for both position and velocity
- RBFNN's approximation of system uncertainties (except for non-adaptive case)
