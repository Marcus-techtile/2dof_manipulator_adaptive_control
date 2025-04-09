%% Example: Adaptive RBFNN Control for a 2-DOF Robotic Manipulator
% This script demonstrates the implementation of the Adaptive RBFNN Controller
% for a 2-DOF manipulator with model uncertainties

% Add paths
current_dir = fileparts(mfilename('fullpath'));
addpath(current_dir);
addpath(fullfile(current_dir, '..'));
addpath(fullfile(current_dir, '..', '..', 'video'));
addpath(fullfile(current_dir, 'model')); % Add model directory for generate_disturbance

clear;
clc;

%% Setup Simulation Parameters
dt = 0.01;              % Time step
T_final = 15;             % Total simulation time
t = 0:dt:T_final;        % Time vector
n_steps = length(t);     % Number of steps

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

%% Initialize State Variables
stateDim = 2; 
controlDim = 2;

q = zeros(stateDim, n_steps);       % Joint positions [q1; q2]
q_dot = zeros(stateDim, n_steps);   % Joint velocities [q1_dot; q2_dot]
tau = zeros(controlDim, n_steps);     % Control inputs [tau1; tau2]
tau_eq = zeros(controlDim, n_steps);  % Equivalent control
tau_rbf = zeros(controlDim, n_steps); % RBFNN control
tau_rob = zeros(controlDim, n_steps); % Robust control
delta_true = zeros(stateDim, n_steps);  % True uncertainty
rho_hat_history = zeros(stateDim, n_steps);  % Add this line to store rho_hat values
L_rbf_history = zeros(stateDim, n_steps);  % Store L_rbf values

% Set initial condition
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
stateRBFDim = 4;       % [e1; e2; e1_dot; e2_dot]
controlDim = 2;     % [tau1; tau2]
neuronNum = 50;     % Number of RBF neurons


rbfnn_std = RBFNNController(stateRBFDim, stateDim, neuronNum, ...
    'Centers', [-0.1, 0.1], ...
    'Widths', 5, ...
    'Gamma', 5, ...
    'Sigma', 0.1, ...
    'UseConstraint', false, ...
    'L_des', 1.0, ...
    'Lambda_L', 3, ...
    'LipschitzMethod', 'direct', ...
    'SoftInitial', true);

% Create controller with default parameters
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

%% Store Error and Control Signals
e_norm = zeros(1, n_steps);
s_norm = zeros(1, n_steps); 
% Add arrays for individual components
e_all = zeros(2, n_steps);     % Position errors [e1; e2]
e_dot_all = zeros(2, n_steps); % Velocity errors [e1_dot; e2_dot]
s_all = zeros(2, n_steps);     % Sliding variables [s1; s2]

%% Main Simulation Loop
for i = 1:n_steps-1
    % Current states
    q_i = q(:,i);
    q_dot_i = q_dot(:,i);
    
    % Compute nominal model dynamics
    [M, C, G, B] = manipulator_dynamics(q_i, q_dot_i, m1, m2, l1, l2, lc1, lc2, I1, I2, g, 0);
    
    % Compute true model dynamics with uncertainties
    % Get disturbance from the dedicated function with profile disturbance type
    [disturbance, ~] = generate_disturbance(t(i), 'Type', 'profile', 'Magnitude', 5, 'StartTime', 2);
    
    % m1_true =  1.5*m1;
    % m2_true =  1.5*m2;
    % I1_true =  1.5*I1;
    % I2_true = 1.5*I2;

    m1_true =  m1;
    m2_true =  m2;
    I1_true =  I1;
    I2_true = I2;

    [M_true, C_true, G_true, B] = manipulator_dynamics(q_i, q_dot_i, m1_true, m2_true, l1, l2, lc1, lc2, I1_true, I2_true, g, disturbance);
    
    % Compute control signal
    [tau(:,i), s, tau_eq(:,i), tau_rbf(:,i), tau_rob(:,i)] = controller.computeControl(q_i, q_dot_i, q_d(:,i), q_d_dot(:,i), q_d_ddot(:,i), M, C, G, B, dt, t(i));
    
    % Store rho_hat value
    rho_hat_history(:,i) = controller.rho_hat;
    
    % Store L_rbf value directly from the controller
    L_rbf_history(i) = controller.rbfnn.L_rbf;
    
    % Compute acceleration
    q_ddot = M_true \ (tau(:,i) - C_true * q_dot_i - G_true);
    
    % Compute the true uncertainty (what RBFNN should approximate)
    delta_acc = M_true * q_ddot - M * q_ddot;  
    delta_vel = C_true * q_dot_i - C * q_dot_i;
    delta_pos = G_true - G;
    delta_true(:,i) = delta_acc + delta_vel + delta_pos;
    
    % Update states using Forward Euler integration
    q_dot(:,i+1) = q_dot_i + dt * q_ddot;
    q(:,i+1) = q_i + dt * q_dot_i;
    
    % Calculate errors
    e = q_d(:,i) - q_i;
    e_dot = q_d_dot(:,i) - q_dot_i;
    e_norm(i) = norm(e);
    s_norm(i) = norm(s);
    
    % Store individual components
    e_all(:,i) = e;
    e_dot_all(:,i) = e_dot;
    s_all(:,i) = s;
end

%% Plot Results
% Define which plots to show
plotOptions = struct(...
    'tracking', true, ...      % Show tracking performance plots
    'errors', true, ...        % Show error and sliding variable plots
    'control', true, ...       % Show control component plots
    'uncertainty', true, ...   % Show uncertainty approximation plots
    'adaptiveGain', true, ... % Turn off adaptive gain plot
    'lipschitz', false, ...     % Show Lipschitz constants plot
    'animation', animate);     % Use the existing animation flag

% Call the plotting function with options
plot_2dof_results(t, q, q_d, tau, tau_eq, tau_rbf, tau_rob, delta_true, e_all, e_dot_all, s_all, e_norm, s_norm, rho_hat_history, L_rbf_history, controller.rbfnn.L_des, animate, l1, l2, plotOptions);