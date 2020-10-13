function [ x_inv_dot, y_inv_dot, z_inv_dot] = invEOMPosition1stO(vI_abs, vI_theta, vI_chi)
    % [ x_tgt_dot, y_tgt_dot, z_tgt_dot, V_abs2_tgt, V_abs_tgt] = sysEOMPosTarget(u_tgt, v_tgt, w_tgt)
    %
    % Target cartesian position propagation for 1st order control (velocity control).
    %
    % Inputs:
    %   - 
    %
    % Outputs:
    %   -     
    x_inv_dot =  vI_abs * cos(vI_theta) * cos(vI_chi);
    y_inv_dot =  vI_abs * cos(vI_theta) * sin(vI_chi);
    z_inv_dot =  - vI_abs * sin(vI_theta);
                                                                                                     
end

