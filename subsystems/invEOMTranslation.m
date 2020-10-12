function [ u_inv_dot, v_inv_dot, w_inv_dot] = invEOMTranslation(aIOO)
    % [ x_inv_dot, y_inv_dot, z_inv_dot, V_abs2_inv, V_abs_inv] = sysEOMPosTarget(u_inv, v_inv, w_inv)
    %
    % Target cartesian position propagation.
    %
    % Inputs:
    %   - 
    %
    % Outputs:
    %   - 
    
    u_inv_dot = aIOO(1);
    v_inv_dot = aIOO(2);
    w_inv_dot = aIOO(3);
    
%     u_inv_dot = aI_abs * cos(aI_theta) * cos(aI_chi);
%     v_inv_dot = aI_abs * cos(aI_theta) * sin(aI_chi);
%     w_inv_dot = aI_abs * sin(aI_theta);
                                                                                                     
end

