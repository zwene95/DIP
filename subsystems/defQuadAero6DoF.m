function [FDAD] = defQuadAero6DoF(...
    u, v, w,...
    T_OD,...
    cD, d,...
    rho)
    %
    % Inputs:
    %   - Velocity in NED-Frame:
    %     - u [m/s]:  x-component
    %     - v [m/s]:  y-component
    %     - w [m/s]:  z-component
    %   - Transformations:  
    %      - T_OB:          Body-fixed to NED frame
    %   - Configuration:
    %     - cD [1]:         drag coefficient
    %     - d [m]:          vehicle diameter
    
    %   - Environment:
    %     - rho [kg/m^3]:   air density
    %     - g [kg/ms^2]:    gravitational acceleration
    % Outputs:
    %   - FBAB [N] (3, 1):  aerodynamic forces in body-fixed frame

% Aerodynamics forces
vDOD = T_OD.' * [u; v; w];
FDAD = [-tanh(vDOD(1)*100) * 0.25 * vDOD(1)^2;...
        -tanh(vDOD(2)*100) * 0.25 * vDOD(2)^2;...
        -tanh(vDOD(3)*100) * 1.00 * vDOD(3)^2] *0.5 *rho *cD *d;



end