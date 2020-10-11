% function [FDPO,Thrust] = defQuadPropulsionModel3DoF(...
%         delta_c, theta_c, chi_c,...
%         m, g, T2W_max)

function [FDPO] = defQuadPropulsionModel3DoF(...
        T_x, T_y, T_z,...
        m, g, T2W_max)
    %
    % Inputs:
    %   - Motor commands:
    % Outputs:
    %   - FBPB [Nm] (3, 1): propulsion force  vector in body-fixed frame
    %   - MBPB [Nm] (3, 1): propulsion moment vector in body-fixed frame

    % Propulsion forces
%     Thrust = delta_c * m * g * T2W_max;

%     FDPO = [
%             Thrust * sin(theta_c) * cos(chi_c)
%             Thrust * sin(theta_c) * sin(chi_c)
%           - Thrust * cos(theta_c)
%             ];

    FDPO = [T_x; T_y; T_z] * m * g * T2W_max;
    
    
    
end