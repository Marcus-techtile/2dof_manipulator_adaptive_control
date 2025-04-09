%% Example: Adaptive RBFNN Control for a 2-DOF Manipulator using ODE45
% This script demonstrates the implementation of the Adaptive RBFNN Controller
% for a 2-DOF manipulator with model uncertainties using ODE45 solver

% Add paths
current_dir = fileparts(mfilename('fullpath'));
workspace_root = fileparts(current_dir);  % Get the workspace root directory
addpath(current_dir);
addpath(fullfile(workspace_root, 'adaptive_rbfnn_controller')); % Add the correct path to RBFNNController
addpath(fullfile(workspace_root, 'video'));
addpath(fullfile(current_dir, 'model')); % Add model directory for generate_disturbance

clear all;
clc;

%% Setup Simulation Parameters
dt = 0.01;              % Time step for output/plotting
T_final = 15;           % Total simulation time
t = 0:dt:T_final;       % Time vector for output/plotting
n_steps = length(t);    % Number of steps for output/plotting

%% Animation Control
animate = true;  % Set to false to skip animation and speed up simulation

%% Manipulator Parameters (Nominal Values)
m1 = 5.0;                % Mass of link 1 (kg)
m2 = 4.5;                % Mass of link 2 (kg)
l1 = 1.0;                % Length of link 1 (m)
l2 = 0.8;                % Length of link 2 (m)
lc1 = 0.5;               % Distance to center of mass of link 1 (m)
lc2 = 0.4;               % Distance to center of mass of link 2 (m)
I1 = (1/3) * m1 * l1^2;  % Inertia of link 1 (kg*m^2)
I2 = (1/3) * m2 * l2^2;  % Inertia of link 2 (kg*m^2)
g = 9.81;                % Gravity acceleration (m/s^2)

% Create parameter structure
param = struct('m1', m1, 'm2', m2, 'l1', l1, 'l2', l2, 'lc1', lc1, 'lc2', lc2, ...
    'I1', I1, 'I2', I2, 'g', g);

%% Initialize State Variables
stateDim = 2;         % [q1; q2]
controlDim = 2;       % [tau1; tau2]
q = zeros(stateDim, n_steps);      % Joint positions
q_dot = zeros(stateDim, n_steps);  % Joint velocities
tau = zeros(controlDim, n_steps);  % Control inputs
tau_eq = zeros(controlDim, n_steps);  % Equivalent control
tau_rbf = zeros(controlDim, n_steps); % RBFNN control
tau_rob = zeros(controlDim, n_steps); % Robust control
delta_true = zeros(stateDim, n_steps);  % True uncertainty
rho_hat_history = zeros(stateDim, n_steps);  % Store rho_hat diagonal matrices
L_rbf_history = zeros(stateDim, n_steps);  % Store L_rbf values

% Set initial conditions
q(:,1) = [0; 0];
q_dot(:,1) = [0; 0];

%% Reference Trajectory: Sinusoidal path for both joints
omega1 = 0.2;                % Frequency for joint 1 (rad/s)
omega2 = 0.2;                % Frequency for joint 2 (rad/s)
amp1 = 0.5;                  % Amplitude for joint 1 (rad)
amp2 = 0.3;                  % Amplitude for joint 2 (rad)

q_d = zeros(2, n_steps);
q_d_dot = zeros(2, n_steps);
q_d_ddot = zeros(2, n_steps);

for i = 1:n_steps
    % Desired positions
    q_d_0 = [-0.1; 0.1];   % Initial position
    q_d(1,i) = q_d_0(1) + amp1 * sin(omega1 * t(i));
    q_d(2,i) = q_d_0(2) + amp2 * sin(omega2 * t(i));

    % Desired velocities
    q_d_dot(1,i) = amp1 * omega1 * cos(omega1 * t(i));
    q_d_dot(2,i) = amp2 * omega2 * cos(omega2 * t(i));
    
    % Desired accelerations
    q_d_ddot(1,i) = -amp1 * omega1^2 * sin(omega1 * t(i));
    q_d_ddot(2,i) = -amp2 * omega2^2 * sin(omega2 * t(i));
end

%% Initialize Adaptive RBFNN Controller
stateRBFDim = 4;      % [e1; e2; e1_dot; e2_dot]
neuronNum = 50;       % Number of RBF neurons


rbfnn_std = RBFNNController(stateRBFDim, stateDim, neuronNum, ...
    'Centers', [-0.1, 0.1], ...
    'Widths', 5, ...
    'Gamma', 1, ...
    'Sigma', 0.1, ...
    'UseConstraint', false, ...
    'L_des', 1.0, ...
    'Lambda_L', 3, ...
    'LipschitzMethod', 'direct', ...
    'SoftInitial', false);

% Create controller with the RBFNN controller as input
controller = AdaptiveRBFNNControl(stateDim, stateRBFDim, controlDim, neuronNum, ...
    'useRBFNN', true, ...
    'useRobustSMCwithRBF', false, ...
    'Lambda', 3*eye(controlDim), ...
    'SmcGain', 5, ...
    'Eta', 5, ...
    'Epsilon', 0.1, ...
    'GammaRho', 20, ...
    'Kappa', 0.01, ...
    'RBFController', rbfnn_std, ...
    'SoftInitial', false);

%% Create arrays for storing results
t_result = t;
q_result = zeros(stateDim, n_steps);
q_dot_result = zeros(stateDim, n_steps);
tau_result = zeros(controlDim, n_steps);
tau_eq_result = zeros(controlDim, n_steps);
tau_rbf_result = zeros(controlDim, n_steps);
tau_rob_result = zeros(controlDim, n_steps);
delta_true_result = zeros(stateDim, n_steps);
rho_hat_result = zeros(stateDim, n_steps);
L_rbf_result = zeros(1, n_steps);
e_norm_result = zeros(1, n_steps);
s_norm_result = zeros(1, n_steps);
e_all_result = zeros(stateDim, n_steps);
e_dot_all_result = zeros(stateDim, n_steps);
s_all_result = zeros(stateDim, n_steps);

% Set initial conditions
q_result(:,1) = [0; 0];
q_dot_result(:,1) = [0; 0];

% Set ODE solver options
options = odeset('RelTol', 1e-4, 'AbsTol', 1e-4*ones(4,1), 'MaxStep', 0.1);

% Setup disturbance parameters
disturbance_type = 'sinusoidal';
disturbance_params = struct('Frequency', 0.5, 'Magnitude', 5, 'StartTime', 2);

% Setup function handle with parameters
odefun = @(t, x) manipulator_ode_dynamics(t, x, t_result, q_d, q_d_dot, q_d_ddot, controller, param, dt, disturbance_type, disturbance_params);

%% Main Simulation Loop - Combined ODE integration and data processing
tic; % Start timing the simulation
fprintf('Starting simulation with ODE45...\n');

for i = 1:n_steps-1
    % Solve from current time to next time step
    [t_temp, x_temp] = ode45(odefun, [t_result(i), t_result(i+1)], [q_result(:,i); q_dot_result(:,i)], options);
    
    % Store the result at the end of this interval
    q_result(:,i+1) = x_temp(end, 1:2)';
    q_dot_result(:,i+1) = x_temp(end, 3:4)';
    
    % Current states
    q_i = q_result(:,i+1);
    q_dot_i = q_dot_result(:,i+1);
    
    % Compute nominal model dynamics
    [M, C, G, B] = manipulator_dynamics(q_i, q_dot_i, param.m1, param.m2, param.l1, param.l2, param.lc1, param.lc2, param.I1, param.I2, param.g, 0);
    
    % Add external disturbance using the dedicated function
    [disturbance, ~] = generate_disturbance(t(i+1), 'Type', 'sinusoidal', 'Frequency', 0.5, 'Magnitude', 5, 'StartTime', 2);
    [M_true, C_true, G_true, B] = manipulator_dynamics(q_i, q_dot_i, param.m1, param.m2, param.l1, param.l2, param.lc1, param.lc2, param.I1, param.I2, param.g, disturbance);
    
    % Compute control signal and store components
    [tau_result(:,i+1), s, tau_eq_result(:,i+1), tau_rbf_result(:,i+1), tau_rob_result(:,i+1)] = ...
        controller.computeControl(q_i, q_dot_i, q_d(:,i+1), q_d_dot(:,i+1), q_d_ddot(:,i+1), M, C, G, B, t_result(i+1) - t_result(i), t_result(i));
    
    % Store rho_hat value
    rho_hat_result(:,i+1) = controller.rho_hat;
    
    % Store L_rbf value directly from the controller
    L_rbf_result(i+1) = controller.rbfnn.L_rbf;
    
    % Compute the true uncertainty
    q_ddot = M_true \ (tau_result(:,i+1) - C_true * q_dot_i - G_true);
    delta_acc = M_true * q_ddot - M * q_ddot;  
    delta_vel = C_true * q_dot_i - C * q_dot_i;
    delta_pos = G_true - G;
    delta_true_result(:,i+1) = ((B'*B)\B')*(delta_acc + delta_vel + delta_pos);
    
    % Calculate errors
    e = q_d(:,i+1) - q_i;
    e_dot = q_d_dot(:,i+1) - q_dot_i;
    e_norm_result(i+1) = norm(e);
    s_norm_result(i+1) = norm(s);
    
    % Store individual components
    e_all_result(:,i+1) = e;
    e_dot_all_result(:,i+1) = e_dot;
    s_all_result(:,i+1) = s;
    
    % Display progress every 10% of steps
    if mod(i, floor(n_steps/10)) == 0
        sim_time = toc;
        fprintf('Progress: %.1f%% complete', 100*i/(n_steps-1));
        fprintf('. Elapsed time: %.2f seconds\n', sim_time);
    end
end
sim_time = toc; % End timing
fprintf('Simulation complete! Total elapsed time: %.2f seconds\n\n', sim_time);

%% Plot Results
% Animation Control
animate = true;  % Set to false to skip animation and speed up simulation

% Define which plots to show
plotOptions = struct(...
    'tracking', true, ...      % Show tracking performance plots
    'errors', true, ...        % Show error and sliding variable plots
    'control', true, ...       % Show control component plots
    'uncertainty', true, ...   % Show uncertainty approximation plots
    'adaptiveGain', true, ...  % Show adaptive gain plot
    'lipschitz', true, ...    % Show Lipschitz constants plot
    'animation', animate, ...     % Use the existing animation flag
    'saveVideo', true);     % Use the existing animation flag

% Call the plotting function with options
plot_2dof_results(t_result, q_result, q_d, tau_result, tau_eq_result, tau_rbf_result, tau_rob_result, delta_true_result, e_all_result, e_dot_all_result, s_all_result, e_norm_result, s_norm_result, rho_hat_result, L_rbf_result, controller.rbfnn.L_des, animate, l1, l2, plotOptions);