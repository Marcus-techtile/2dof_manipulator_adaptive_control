function dxdt = manipulator_ode_dynamics(t, x, t_ref, q_d_ref, q_d_dot_ref, q_d_ddot_ref, controller, param, dt_sim, disturbance_type, disturbance_params)
    % MANIPULATOR_ODE_DYNAMICS ODE function for 2-DOF manipulator dynamics
    %
    % This function is used with ODE45 to simulate the 2-DOF manipulator dynamics
    %
    % Inputs:
    %   t - Current time
    %   x - State vector [q1; q2; q1_dot; q2_dot]
    %   t_ref - Reference time vector
    %   q_d_ref - Reference joint positions
    %   q_d_dot_ref - Reference joint velocities
    %   q_d_ddot_ref - Reference joint accelerations
    %   controller - Controller object
    %   param - Manipulator parameters structure
    %   dt_sim - Simulation time step
    %   disturbance_type - Type of disturbance ('step', 'sinusoidal', etc.)
    %   disturbance_params - Structure with disturbance parameters
    %
    % Outputs:
    %   dxdt - State derivatives [q1_dot; q2_dot; q1_ddot; q2_ddot]
    
    % Extract states from the state vector
    q = x(1:2);
    q_dot = x(3:4);
    
    % Find the closest index in the reference time vector
    [~, idx] = min(abs(t_ref - t));
    
    % Get reference values at the closest time point
    q_d = q_d_ref(:,idx);
    q_d_dot = q_d_dot_ref(:,idx);
    q_d_ddot = q_d_ddot_ref(:,idx);
    
    % Compute nominal model dynamics (without disturbance)
    [M, C, G, B] = manipulator_dynamics(q, q_dot, param.m1, param.m2, param.l1, param.l2, param.lc1, param.lc2, param.I1, param.I2, param.g, 0);
    
    % Generate disturbance using parameters if provided
    if nargin > 9
        % Convert struct to Name-Value pairs
        paramPairs = {};
        paramPairs{1} = 'Type';
        paramPairs{2} = disturbance_type;
        
        % Add all fields from the struct to the parameter list
        fn = fieldnames(disturbance_params);
        for i = 1:numel(fn)
            paramPairs{end+1} = fn{i};
            paramPairs{end+1} = disturbance_params.(fn{i});
        end
        
        % Generate disturbance with the provided parameters
        [disturbance, ~] = generate_disturbance(t, paramPairs{:});
    else
        % Use default parameters
        [disturbance, ~] = generate_disturbance(t, 'Type', 'step', 'Magnitude', 5, 'StartTime', 2);
    end
    
    % Compute true model dynamics with disturbance
    [M_true, C_true, G_true, B] = manipulator_dynamics(q, q_dot, param.m1, param.m2, param.l1, param.l2, param.lc1, param.lc2, param.I1, param.I2, param.g, disturbance);
    
    % Compute control signal
    [tau, ~, ~, ~, ~] = controller.computeControl(q, q_dot, q_d, q_d_dot, q_d_ddot, M, C, G, B, dt_sim, t);
    
    % Compute acceleration using the true model dynamics
    q_ddot = M_true \ (tau - C_true * q_dot - G_true);
    
    % Return the state derivatives
    dxdt = [q_dot; q_ddot];
end