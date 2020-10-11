function [MDTD] = defMoments(MDPD)
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


% Total Moments in Body Fixed Frame (B)
MDTD = MDPD;


end