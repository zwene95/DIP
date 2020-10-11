function [ phi_dot, theta_dot, psi_dot ] = defEOMAttitudeEuler(phi, theta, psi, p, q, r)
    % [ roll_dot, pitch_dot, yaw_dot ] = sysEOMAttitudeEuler(roll, pitch, yaw, p, q, r)
    %
    % Euler attitude propagation.
    %
    % Inputs:
    %   - roll [rad]: roll angle
    %   - pitch [rad]: pitch angle
    %   - yaw [rad]: yaw angle
    %   - p [rad/s]: roll rate
    %   - q [rad/s]: pitch rate
    %   - r [rad/s]: yaw rate
    %
    % Outputs:
    %   - phi_dot [rad/s]: roll angle derivative
    %   - theta_dot [rad/s]: pitch angle derivative
    %   - psi_dot [rad/s]: yaw angle derivative
    
    phi_dot =   p + q * sin(phi) * tan(theta) + r * cos(phi) * tan(theta);
    theta_dot =     q * cos(phi)              - r * sin(phi);
    psi_dot =       q * sin(phi) * sec(theta) + r * cos(phi) * sec(theta);

end
