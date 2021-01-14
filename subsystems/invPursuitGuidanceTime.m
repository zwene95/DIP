function [ u_inv, v_inv, w_inv] = invPursuitGuidanceTime(   x_inv, y_inv, z_inv,...
                                                            pTOO_x, pTOO_y, pTOO_z,...
                                                            vI_abs_max, T)
    %
    % Target cartesian position propagation.
    %
    % Inputs:
    %   - 
    %
    % Outputs:
    %   -     
    
    pIOO = [
        pTOO_x - x_inv 
        pTOO_y - y_inv 
        pTOO_z - z_inv 
    ];

    dV = [
        0 * sin(2*pi*T/5)
        1 * sin(2*pi*T/5)
        0 * sin(2*pi*T/5)
    ];

    vIOO = pIOO / norm(pIOO) +  dV;
    vIOO = vIOO / norm(vIOO) * vI_abs_max;
    
    u_inv =  vIOO(1);
    v_inv =  vIOO(2);
    w_inv =  vIOO(3);
    
end

