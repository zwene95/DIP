function [ x_inv_dot, y_inv_dot, z_inv_dot] = invEOMPosition(u_inv, v_inv, w_inv)
    % [ x_tgt_dot, y_tgt_dot, z_tgt_dot, V_abs2_tgt, V_abs_tgt] = sysEOMPosTarget(u_tgt, v_tgt, w_tgt)
    %
    % Target cartesian position propagation.
    %
    % Inputs:
    %   - 
    %
    % Outputs:
    %   -     
    x_inv_dot =  u_inv;
    y_inv_dot =  v_inv;
    z_inv_dot =  w_inv;
end

