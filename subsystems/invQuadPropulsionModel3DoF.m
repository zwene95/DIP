function [FIPO] = invQuadPropulsionModel3DoF( T_x_inv, T_y_inv, T_z_inv,...
                                              mI, g, T2W_max_inv)
                                          
% function [FIPO] = invQuadPropulsionModel3DoF( delta_Ic, theta_Ic, chi_Ic,...
%                                               mI, g)                                       
    %
    % Inputs:
    %   - Motor commands:
    % Outputs:
    %   - FIPO [Nm] (3, 1): propulsion force  vector in NED frame
    %   - MVPV [Nm] (3, 1): propulsion moment vector in invader-fixed frame



FIPO = [T_x_inv; T_y_inv; T_z_inv] * mI * g * T2W_max_inv;
    
    % Propulsion forces
%     FIPO = [    sin(theta_Ic) * cos(chi_Ic)
%                 sin(theta_Ic) * sin(chi_Ic)                       
%                 - cos(theta_Ic)
%             ] * delta_Ic * mI * g;

end