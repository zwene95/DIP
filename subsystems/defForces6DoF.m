function [aDOO] = defForces6DoF(...
    FDAD, FDPD,...
    T_OD,...
    m, g)
    %
    % Inputs:
    %   - Transformations:    
    %      - T_OB:          Body-fixed to NED frame
    %   - Configuration:
    %     - m [kg]:         vehicle mass
    %   - Environment:
    %     - g [kg/ms^2]:    gravitational acceleration
    % Outputs:
    %   - FBTO [Nm] (3, 1): total force vector in NED frame
    %   - MBTB [Nm] (3, 1): total moment vector in body-fixed frame

% Gravitational force
FDGO = [0; 0; g]*m;
% Total forces
FDTO = T_OD * (FDAD + FDPD) + FDGO;

% Acceleration 
aDOO = FDTO / m;
end