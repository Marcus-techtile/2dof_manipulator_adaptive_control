function [M, C, G, B] = manipulator_dynamics(q, q_dot, m1, m2, l1, l2, lc1, lc2, I1, I2, g, noisy_force)
    % This function computes the dynamics matrices for a 2-DOF planar manipulator
    % Inputs:
    %   q     - Joint positions [q1; q2]
    %   q_dot - Joint velocities [q1_dot; q2_dot]
    %   m1, m2 - Masses of links
    %   l1, l2 - Lengths of links
    %   lc1, lc2 - Distances to centers of mass
    %   I1, I2 - Inertias of links
    %   g - Gravity acceleration
    %   noisy_force - External disturbance force
    % Outputs:
    %   M - Inertia matrix
    %   C - Coriolis and centrifugal matrix
    %   G - Gravity vector
    %   B - Input matrix
    
    % Extract joint positions and velocities
    q1 = q(1);
    q2 = q(2);
    q1_dot = q_dot(1);
    q2_dot = q_dot(2);
    
    % Precompute trigonometric terms
    s1 = sin(q1);
    c1 = cos(q1);
    s2 = sin(q2);
    c2 = cos(q2);
    s12 = sin(q1 + q2);
    c12 = cos(q1 + q2);
    
    % Compute inertia matrix M
    M = zeros(2, 2);
    M(1,1) = m1*lc1^2 + m2*(l1^2 + lc2^2 + 2*l1*lc2*c2) + I1 + I2;
    M(1,2) = m2*(lc2^2 + l1*lc2*c2) + I2;
    M(2,1) = M(1,2);
    M(2,2) = m2*lc2^2 + I2;
    
    % Compute Coriolis matrix C
    h = -m2*l1*lc2*s2;
    C = zeros(2, 2);
    C(1,1) = -q2_dot*h;
    C(1,2) = -(q1_dot + q2_dot)*h;
    C(2,1) = q1_dot*h;
    C(2,2) = 0;
    
    % Compute gravity vector G
    G = zeros(2, 1);
    G(1) = (m1*lc1 + m2*l1)*g*c1 + m2*lc2*g*c12;
    G(2) = m2*lc2*g*c12;
    
    % Apply the disturbance forces if provided as a vector
    if isscalar(noisy_force)
        G = G + [noisy_force; noisy_force];
    else
        G = G + noisy_force;
    end

    % Input matrix
    B = [1 0; 0 1];
end 