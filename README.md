# 2DOF Manipulator Control
Implementation of robust RBF control for a 2-DOF robotic manipulator.

## Dependencies
This project uses the robust_rbf_control framework as a submodule.

## Setup
1. Clone this repository
2. Initialize submodules:
   \`\`\`bash
   git submodule update --init --recursive
   \`\`\`"

## Results
The implementation includes three main test scenarios:

### 1. Sinusoidal Noise Test
Located in `results/sinusoidal_noise/`:
- Tracking performance of joint positions and velocities
- Tracking errors analysis
- RBFNN uncertainty approximation

### 2. External Force Noise Test
Located in `results/external_force_noise/`:
- Tracking performance under external disturbances
- Error analysis with force disturbances
- RBFNN uncertainty approximation

### 3. Non-Adaptive Control Test
Located in `results/no_adaptive/`:
- Baseline performance without RBFNN adaptation
- Comparison of tracking errors with and without adaptation
- Demonstrates the importance of adaptive control

Each test scenario includes visualization plots:
1. Tracking Performance: Shows the actual vs. desired joint positions and velocities
2. Tracking Errors: Displays the error between desired and actual trajectories
3. Uncertainty Approximation: Illustrates how well the RBFNN approximates the system uncertainties (except for non-adaptive case)

## Visualization
The results are automatically saved as JPG files in their respective directories. The plots show:
- Joint positions (q1, q2) and their desired trajectories
- Joint velocities (q1_dot, q2_dot) and their desired trajectories
- Tracking errors for both position and velocity
- RBFNN's approximation of system uncertainties

## Videos
The project includes two simulation videos in the `video/` directory:
1. `2dof_manipulator.mp4`: Shows the manipulator motion with adaptive RBFNN control
2. `2dof_manipulator_no_adaptive.mp4`: Shows the manipulator motion without adaptive control

These videos provide a visual comparison of the controller's performance with and without the adaptive RBFNN component.
