function plot_2dof_results(t, q, q_d, tau, tau_eq, tau_rbf, tau_rob, delta_true, e_all, e_dot_all, s_all, e_norm, s_norm, rho_hat_history, L_rbf_history, L_des, animate, l1, l2, plotOptions, beta_result)
    % Plot results for 2DOF manipulator simulation
    % Inputs:
    %   t - Time vector
    %   q - Joint positions
    %   q_d - Desired joint positions
    %   tau - Control inputs
    %   tau_eq - Equivalent control
    %   tau_rbf - RBFNN control
    %   tau_rob - Robust control
    %   delta_true - True uncertainty
    %   e_all - Position errors
    %   e_dot_all - Velocity errors
    %   s_all - Sliding variables
    %   e_norm - Norm of position errors
    %   s_norm - Norm of sliding variables
    %   rho_hat_history - History of adaptive gain
    %   L_rbf_history - History of Lipschitz constant
    %   L_des - Desired Lipschitz constant
    %   animate - Whether to animate the manipulator
    %   l1, l2 - Link lengths
    %   plotOptions - Structure containing boolean flags for each plot type
    %       .tracking - Tracking performance plots
    %       .errors - Error and sliding variable plots
    %       .control - Control component plots
    %       .uncertainty - Uncertainty approximation plots
    %       .adaptiveGain - Adaptive gain evolution plot
    %       .lipschitz - Lipschitz constants plot
    %       .beta - RBFNN output scaling factor plot
    %       .animation - 2DOF manipulator animation
    %       .saveVideo - Whether to save animation as video
    %   beta_result - History of beta values (scaling factor)

    % Set default plot options if not provided
    if nargin < 20
        plotOptions = struct(...
            'tracking', true, ...
            'errors', true, ...
            'control', true, ...
            'uncertainty', true, ...
            'adaptiveGain', true, ...
            'lipschitz', true, ...
            'beta', false, ...
            'animation', animate, ...
            'saveVideo', false);
    end
    
    % Check if beta_result is provided
    if nargin < 21
        beta_result = ones(size(t));  % Default to 1 if not provided
    end

    %% Plotting Results
    if plotOptions.tracking
        figure('Name', 'Tracking Performance');

        % Joint Position Tracking
        subplot(2,2,1);
        plot(t, q_d(1,:), 'r--', 'LineWidth', 2);
        hold on;
        plot(t, q(1,:), 'b-', 'LineWidth', 1.5);
        xlabel('Time (s)');
        ylabel('Position (rad)');
        title('Joint 1 Position');
        legend('Desired', 'Actual');
        grid on;

        subplot(2,2,2);
        plot(t, q_d(2,:), 'r--', 'LineWidth', 2);
        hold on;
        plot(t, q(2,:), 'b-', 'LineWidth', 1.5);
        xlabel('Time (s)');
        ylabel('Position (rad)');
        title('Joint 2 Position');
        legend('Desired', 'Actual');
        grid on;

        % Control Signals
        subplot(2,2,3);
        plot(t, tau(1,:), 'b-', 'LineWidth', 1.5);
        xlabel('Time (s)');
        ylabel('Torque (N·m)');
        title('Control Input (Joint 1)');
        grid on;

        subplot(2,2,4);
        plot(t, tau(2,:), 'b-', 'LineWidth', 1.5);
        xlabel('Time (s)');
        ylabel('Torque (N·m)');
        title('Control Input (Joint 2)');
        grid on;
    end

    %% Error and Sliding Variable
    if plotOptions.errors
        figure('Name', 'Tracking Errors');

        % Position errors
        y_margin = 0.01;
        subplot(3,1,1);
        plot(t, e_all(1,:), 'b-', 'LineWidth', 1.5);
        hold on;
        plot(t, e_all(2,:), 'r-', 'LineWidth', 1.5);
        xlabel('Time (s)');
        ylabel('Position Error (rad)');
        title('Joint Position Errors');
        legend('e_1', 'e_2');
        ylim([-y_margin, y_margin]);
        grid on;

        % Velocity errors
        subplot(3,1,2);
        plot(t, e_dot_all(1,:), 'b-', 'LineWidth', 1.5);
        hold on;
        plot(t, e_dot_all(2,:), 'r-', 'LineWidth', 1.5);
        xlabel('Time (s)');
        ylabel('Velocity Error (rad/s)');
        title('Joint Velocity Errors');
        legend('ė_1', 'ė_2');
        ylim([-y_margin, y_margin]);
        grid on;

        % Sliding variables
        subplot(3,1,3);
        plot(t, s_all(1,:), 'b-', 'LineWidth', 1.5);
        hold on;
        plot(t, s_all(2,:), 'r-', 'LineWidth', 1.5);
        xlabel('Time (s)');
        ylabel('Sliding Variable');
        title('Sliding Variables');
        legend('s_1', 's_2');
        ylim([-y_margin, y_margin]);
        grid on;
    end

    %% Control Components
    if plotOptions.control
        figure('Name', 'Control Components');

        % Joint 1 Control Components
        subplot(2,1,1);
        plot(t, tau_eq(1,:), 'b-', 'LineWidth', 1.5);
        hold on;
        plot(t, tau_rbf(1,:), 'r-', 'LineWidth', 1.5);
        plot(t, tau_rob(1,:), 'g-', 'LineWidth', 1.5);
        plot(t, tau(1,:), 'k--', 'LineWidth', 1);
        xlabel('Time (s)');
        ylabel('Torque (N·m)');
        title('Joint 1 Control Components');
        legend('τ_{eq} (Equivalent)', 'τ_{rbf} (RBFNN)', 'τ_{rob} (Robust)', 'τ (Total)');
        grid on;

        % Joint 2 Control Components
        subplot(2,1,2);
        plot(t, tau_eq(2,:), 'b-', 'LineWidth', 1.5);
        hold on;
        plot(t, tau_rbf(2,:), 'r-', 'LineWidth', 1.5);
        plot(t, tau_rob(2,:), 'g-', 'LineWidth', 1.5);
        plot(t, tau(2,:), 'k--', 'LineWidth', 1);
        xlabel('Time (s)');
        ylabel('Torque (N·m)');
        title('Joint 2 Control Components');
        legend('τ_{eq} (Equivalent)', 'τ_{rbf} (RBFNN)', 'τ_{rob} (Robust)', 'τ (Total)');
        grid on;
    end

    %% Uncertainty vs RBFNN Approximation
    if plotOptions.uncertainty
        figure('Name', 'Uncertainty Approximation');

        % Joint 1 Uncertainty
        subplot(2,1,1);
        plot(t, delta_true(1,:), 'b-', 'LineWidth', 1.5);
        hold on;
        plot(t, tau_rbf(1,:), 'r-', 'LineWidth', 1.5);
        xlabel('Time (s)');
        ylabel('Torque (N·m)');
        title('Joint 1 Uncertainty Approximation');
        legend('True Uncertainty (δ)', 'RBFNN Approximation (τ_{rbf})');
        grid on;

        % Joint 2 Uncertainty
        subplot(2,1,2);
        plot(t, delta_true(2,:), 'b-', 'LineWidth', 1.5);
        hold on;
        plot(t, tau_rbf(2,:), 'r-', 'LineWidth', 1.5);
        xlabel('Time (s)');
        ylabel('Torque (N·m)');
        title('Joint 2 Uncertainty Approximation');
        legend('True Uncertainty (δ)', 'RBFNN Approximation (τ_{rbf})');
        grid on;
    end

    %% Adaptive Gain
    if plotOptions.adaptiveGain
        figure('Name', 'Adaptive Gain');
        plot(t, rho_hat_history, 'b-', 'LineWidth', 1.5);
        xlabel('Time (s)');
        ylabel('ρ_{hat}');
        title('Evolution of Adaptive Gain');
        grid on;
    end

    %% Lipschitz Constants
    if plotOptions.lipschitz
        figure('Name', 'Lipschitz Constants');
        plot(t, L_rbf_history, 'b-', 'LineWidth', 1.5);
        hold on;
        L_des_line = L_des * ones(size(t));
        plot(t, L_des_line, 'r--', 'LineWidth', 1.5);
        xlabel('Time (s)');
        ylabel('Lipschitz Constant');
        title('Evolution of RBFNN Lipschitz Constant');
        legend('L_{rbf}', 'L_{des}');
        grid on;
    end

    %% Beta Scaling Factor
    if isfield(plotOptions, 'beta') && plotOptions.beta
        figure('Name', 'RBFNN Beta Scaling Factor');
        plot(t, beta_result, 'b-', 'LineWidth', 1.5);
        xlabel('Time (s)');
        ylabel('Beta');
        title('RBFNN Output Scaling Factor (Beta)');
        ylim([0, 1.1]);  % Beta should be between 0 and 1
        grid on;
    end

    %% Animate the 2DOF Manipulator
    if plotOptions.animation
        % Check if saveVideo option exists, default to false if not
        if ~isfield(plotOptions, 'saveVideo')
            plotOptions.saveVideo = false;
        end
        animate_2dof_manipulator(t, q, q_d, l1, l2, e_norm, plotOptions.saveVideo);
    end
end

function animate_2dof_manipulator(t, q, q_d, l1, l2, e_norm, saveVideo)
    % Animate the 2DOF manipulator
    % Inputs:
    %   t - Time vector
    %   q - Joint positions
    %   q_d - Desired joint positions
    %   l1, l2 - Link lengths
    %   e_norm - Norm of position errors
    %   saveVideo - Whether to save as video file (optional)
    
    % Default saveVideo to false if not provided
    if nargin < 7
        saveVideo = false;
    end
    
    % Setup video writer if saving video
    if saveVideo
        % Create a timestamp for the filename
        videoFilename = sprintf('video/2dof_manipulator.mp4');
        
        % Create video writer object
        videoFPS = 30;
        videoQuality = 100; % Maximum quality
        videoObj = VideoWriter(videoFilename, 'MPEG-4');
        videoObj.FrameRate = videoFPS;
        videoObj.Quality = videoQuality;
        
        % Open the video file
        open(videoObj);
        fprintf('Recording video to: %s\n', videoFilename);
    end
    
    figure('Name', 'Manipulator Animation', 'Position', [100, 100, 800, 600]);
    
    % Set axis limits with some margin
    axis_limit = (l1 + l2) * 1.1;
    ax = axes;
    hold(ax, 'on');
    xlim([-axis_limit, axis_limit]);
    ylim([-axis_limit, axis_limit]);
    axis equal;
    grid on;
    title('2DOF Manipulator Animation');
    xlabel('X (m)');
    ylabel('Y (m)');
    
    % Create graphics objects for the links and joints
    link1_line = line('XData', [0, 0], 'YData', [0, 0], 'Color', 'blue', 'LineWidth', 3);
    link2_line = line('XData', [0, 0], 'YData', [0, 0], 'Color', 'red', 'LineWidth', 3);
    joint0_marker = plot(0, 0, 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
    joint1_marker = plot(0, 0, 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b');
    ee_marker = plot(0, 0, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
    
    % Calculate and plot reference trajectory
    x_ref = zeros(1, length(t));
    y_ref = zeros(1, length(t));
    for i = 1:length(t)
        % Forward kinematics for desired trajectory
        x_ref(i) = l1*cos(q_d(1,i)) + l2*cos(q_d(1,i) + q_d(2,i));
        y_ref(i) = l1*sin(q_d(1,i)) + l2*sin(q_d(1,i) + q_d(2,i));
    end
    reference_path = plot(x_ref, y_ref, 'g--', 'LineWidth', 1.5);
    
    % Create trace arrays to store the actual path
    x_trace = [];
    y_trace = [];
    trace_line = plot(0, 0, 'b-', 'LineWidth', 1.5);
    
    % Create trace arrays for first joint
    x_trace_joint1 = [];
    y_trace_joint1 = [];
    trace_line_joint1 = plot(0, 0, 'c-', 'LineWidth', 1.5);
    
    % Animation speed control
    skip_frames = 10;
    
    legend('Link 1', 'Link 2', 'Base', 'Joint 1', 'End Effector', 'Desired Path', 'End-Effector Path', 'Joint 1 Path');
    
    % Animation loop
    for i = 1:skip_frames:length(t)
        % Calculate joint positions using forward kinematics
        x1 = l1*cos(q(1,i));
        y1 = l1*sin(q(1,i));
        x2 = x1 + l2*cos(q(1,i) + q(2,i));
        y2 = y1 + l2*sin(q(1,i) + q(2,i));
        
        % Update link positions
        set(link1_line, 'XData', [0, x1], 'YData', [0, y1]);
        set(link2_line, 'XData', [x1, x2], 'YData', [y1, y2]);
        
        % Update joint and end-effector positions
        set(joint1_marker, 'XData', x1, 'YData', y1);
        set(ee_marker, 'XData', x2, 'YData', y2);
        
        % Update end-effector trace
        x_trace = [x_trace, x2];
        y_trace = [y_trace, y2];
        set(trace_line, 'XData', x_trace, 'YData', y_trace);
        
        % Update first joint trace
        x_trace_joint1 = [x_trace_joint1, x1];
        y_trace_joint1 = [y_trace_joint1, y1];
        set(trace_line_joint1, 'XData', x_trace_joint1, 'YData', y_trace_joint1);
        
        % Display current time and error
        title(sprintf('2DOF Manipulator Animation - Time: %.2f s - Error: %.4f', t(i), e_norm(i)));
        
        % Save the current frame to video if requested
        if saveVideo
            frame = getframe(gcf);
            writeVideo(videoObj, frame);
        end
        
        % Pause for smoother animation
        drawnow;
        pause(0.01);
    end
    
    % Close the video file if it was opened
    if saveVideo
        close(videoObj);
        fprintf('Video saved successfully to: %s\n', videoFilename);
    end
end 