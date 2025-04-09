function [disturbance, force_base] = generate_disturbance(t, varargin)
    % GENERATE_DISTURBANCE Generates external disturbance forces for a 2-DOF manipulator
    %
    % Inputs:
    %   t - Current simulation time
    %   varargin - Optional parameters:
    %     'Type' - Disturbance type: 'step', 'ramp', 'sinusoidal', or 'random' (default: 'step')
    %     'Magnitude' - Base force magnitude (default: 5)
    %     'StartTime' - Time when disturbance starts (default: 2)
    %     'EndTime' - Time when disturbance ends (if applicable) (default: inf)
    %     'Frequency' - Frequency for sinusoidal disturbance (default: 0.5)
    %
    % Outputs:
    %   disturbance - 2x1 vector of disturbance forces applied to joints
    %   force_base - Base magnitude of the disturbance force
    
    % Parse inputs
    p = inputParser;
    p.addParameter('Type', 'step', @ischar);
    p.addParameter('Magnitude', 5, @isnumeric);
    p.addParameter('StartTime', 2, @isnumeric);
    p.addParameter('EndTime', inf, @isnumeric);
    p.addParameter('Frequency', 0.5, @isnumeric);
    p.parse(varargin{:});
    
    opts = p.Results;
    
    % Define base force magnitude
    force_base = opts.Magnitude;
    
    % Initialize disturbance vector
    disturbance = zeros(2,1);
    
    % Generate disturbance based on type
    if t < opts.StartTime || t > opts.EndTime
        % No disturbance outside the specified time window
        return;
    end
    
    switch lower(opts.Type)
        case 'step'
            % Step disturbance
            disturbance(1) = force_base;
            disturbance(2) = force_base;
            
        case 'ramp'
            % Ramp disturbance over 0.2 seconds
            if t < opts.StartTime + 0.2
                % Ramp up over 0.2 seconds
                ramp_factor = (t - opts.StartTime) / 0.2;
                disturbance(1) = force_base * ramp_factor;
                disturbance(2) = force_base * ramp_factor;
            else
                disturbance(1) = force_base;
                disturbance(2) = force_base;
            end
            
        case 'sinusoidal'
            % Sinusoidal disturbance
            disturbance(1) = force_base * sin(2 * pi * opts.Frequency * (t - opts.StartTime));
            disturbance(2) = force_base * sin(2 * pi * opts.Frequency * (t - opts.StartTime));
            
        case 'random'
            % Random disturbance with normal distribution
            disturbance(1) = force_base * randn(1);
            disturbance(2) = force_base * randn(1);
            
        case 'profile'
            % More complex disturbance profile as seen in example_2dof_manipulator.m
            if (t < opts.StartTime + 0.2)  % Ramp up to first force (0.2s)
                ramp_factor = (t - opts.StartTime) / 0.2;  % Linear increase from 0 to force_base
                disturbance(1) = force_base * ramp_factor;
                disturbance(2) = force_base * ramp_factor;
            elseif (t < opts.StartTime + 2)
                disturbance(1) = force_base;
                disturbance(2) = force_base;
            elseif (t < opts.StartTime + 2.2)  % Ramp down to negative force (0.2s)
                ramp_factor = (t - (opts.StartTime + 2)) / 0.2;  % Linear decrease
                disturbance(1) = force_base - (force_base + 3) * ramp_factor;
                disturbance(2) = force_base - (force_base + 3) * ramp_factor;
            elseif (t < opts.StartTime + 4)
                disturbance(1) = -3;
                disturbance(2) = -3;
            elseif (t < opts.StartTime + 4.2)  % Ramp up to final force (0.2s)
                ramp_factor = (t - (opts.StartTime + 4)) / 0.2;  % Linear increase
                disturbance(1) = -3 + 5 * ramp_factor;
                disturbance(2) = -3 + 5 * ramp_factor;
            else
                disturbance(1) = 2;
                disturbance(2) = 2;
            end
            
        otherwise
            warning('Unknown disturbance type. Using step disturbance.');
            disturbance(1) = force_base;
            disturbance(2) = force_base;
    end
end 