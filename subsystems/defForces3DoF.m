function [aDOO] = defForces3DoF(    FDAO, FDPO,...
                                    m, g)
    %
    % Inputs:
    %   - Configuration:
    %     - m [kg]:         vehicle mass
    %   - Environment:
    %     - g [kg/ms^2]:    gravitational acceleration
    % Outputs:
    %   - FBTO [Nm] (3, 1): total force vector in NED frame
    %   - MBTB [Nm] (3, 1): total moment vector in body-fixed frame

% Gravitational force
FDGO = [0; 0; g] * m;
% Total forces
FDTO = FDAO + FDPO + FDGO;
% Acceleration 
aDOO = FDTO / m;
end