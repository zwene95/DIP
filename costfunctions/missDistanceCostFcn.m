function costs = missDistanceCostFcn(outputs, states, controls)
% Mayer cost function for invader objective

    % Extract outputs
%     FDAD_x       = outputs(1);
%     FDAD_y       = outputs(2);
%     FDAD_z       = outputs(3);

    % Extract states
    x             = states(1);
    y             = states(2);
    z             = states(3);
    u             = states(4);
    v             = states(5);
    w             = states(6);
    x_inv         = states(7);
    y_inv         = states(8);
    z_inv         = states(9);
    phi           = states(10);
    theta         = states(11);
    psi           = states(12);
    p             = states(13);
    q             = states(14);
    r             = states(15);

    % Extract controls
    w1            = controls(1);
    w2            = controls(2);
    w3            = controls(3);
    w4            = controls(4);

    % Cost function
    missDistance =    ( x_inv - x )^2 + ...
                      ( y_inv - y )^2 + ...
                      ( z_inv - z )^2;  
                  
    % Cost values
    costs = [missDistance];
    
    % EoF
    
end