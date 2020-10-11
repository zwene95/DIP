function [p_dot, q_dot, r_dot] = defEOMRotation(wDOD, Inertia, MDTD)
    % [p_dot, q_dot, r_dot] = defEOMRotation(wDOD, Inertia, MDTD)
    %
    % Rotation dynamics / angular momentum equation.
    %
    % Inputs:
    %   - wBOB [rad/s] (3, 1):
    %     angular velocity of the body frame w.r.t. earth surface, expressed in
    %     body-fixed coordinates
    %   - Inertia [kg m^2] (3, 3): body-fixed inertia tensor
    %   - MBTB [Nm] (3, 1): total moment in body-fixed frame
    %
    % Outputs:
    %   - p_dot [rad/s^2]: roll acceleration (body-fixed)
    %   - q_dot [rad/s^2]: pitch acceleration (body-fixed)
    %   - r_dot [rad/s^2]: yaw acceleration (body-fixed)

    % Hint: implement the equation in vector form first, i.e. omega_OB_dot = ?
    wDOD_dot = Inertia \ (MDTD - cross(wDOD,Inertia*wDOD));
    
    p_dot = wDOD_dot(1);
    q_dot = wDOD_dot(2);
    r_dot = wDOD_dot(3);

end
