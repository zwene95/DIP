function [ T_OD ] = defAttitudeEuler(phi, theta, psi)
    % [ M_BO ] = sysAttitudeEuler(roll, pitch, yaw)
    %
    % NED to body transformation matrix.
    %
    % Inputs:
    %   - phi [rad]: roll angle
    %   - theta [rad]: pitch angle
    %   - psi [rad]: yaw angle
    %
    % Outputs:
    %   - M_BO [1] (3, 3): transformation matrix from NED (O) to body (B) frame
    
    T_OD = [
        cos(psi)*cos(theta) , cos(psi)*sin(theta)*sin(phi)-sin(psi)*cos(phi) , cos(psi)*sin(theta)*cos(phi)+sin(psi)*sin(phi)
        sin(psi)*cos(theta) , sin(psi)*sin(theta)*sin(phi)+cos(psi)*cos(phi) , sin(psi)*sin(theta)*cos(phi)-cos(psi)*sin(phi)
        -sin(theta)         , cos(theta)*sin(phi)                            , cos(theta)*cos(phi)
        ];


end
