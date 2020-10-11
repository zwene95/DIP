function [ x_dot, y_dot, z_dot ] = defEOMPosition(u, v, w)
    % [x_dot, y_dot, z_dot] = sysEOMPosition(V, chi, gamma)
    %
    % Cartesian position propagation.
    %
    % Inputs:
    %   - V [m/s]: kinematic velocity
    %   - chi [rad]: course angle
    %   - gamma [rad]: climb angle
    %
    % Outputs:
    %   - x_dot [m/s]: x velocity
    %   - y_dot [m/s]: y velocity
    %   - z_dot [m/s]: z velocity

    x_dot =  u;
    y_dot =  v;
    z_dot =  w;
    
%     V_abs2 = u^2 + v^2 + w^2;
end

