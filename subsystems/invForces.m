function [aIOO] = invForces(...
    FIPO,...
    mI, g)
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

% Gravitational acceleration
aIGO = [0; 0; g];

% Total acceleration 
aIOO = FIPO / mI + aIGO;

end