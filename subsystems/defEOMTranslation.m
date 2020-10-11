function [u_dot, v_dot, w_dot] = defEOMTranslation(aDOO)
    % [V_dot, chi_dot, gamma_dot] = sysEOMTranslation(V, gamma, n_T_K, n_G_K, g)
    %
    % Translation dynamics.
    %
    % Inputs:
    %   - V [m/s]:      kinematic velocity
    %   - gamma [rad]:  climb angle
    %   - m [kg]:       vehicle mass
    %   - FBTK [N]:     Total forces expressed in the kinematic frame
    %
    % Outputs:
    %   - V_dot [m/s^2]:        flight-path acceleration
    %   - chi_dot [rad/s]:      turn rate
    %   - gamma_dot [rad/s]:    climb angle derivative
    
    u_dot = aDOO(1);
    v_dot = aDOO(2);
    w_dot = aDOO(3);

end