function FovConstraint  = fovConstraintFcn(Outputs, States, Controls)
% Defender seeker FoV constraint function

    % Extract outputs
%     FDAD_x       = outputs(1);
%     FDAD_y       = outputs(2);
%     FDAD_z       = outputs(3);

    % Extract states
    x             = States(1);
    y             = States(2);
    z             = States(3);
    u             = States(4);
    v             = States(5);
    w             = States(6);
    x_inv         = States(7);
    y_inv         = States(8);
    z_inv         = States(9);
    phi           = States(10);
    theta         = States(11);
    psi           = States(12);
    p             = States(13);
    q             = States(14);
    r             = States(15);

    % Extract controls
    w1            = Controls(1);
    w2            = Controls(2);
    w3            = Controls(3);
    w4            = Controls(4);

    pDIO =  [   x_inv - x
                y_inv - y
                z_inv - z
            ];
    
    % Constraint functions
    Azimuth =   atan2(pDIO(2), pDIO(1) + 1e2*eps);
    Elevation = atan2(-pDIO(3), sqrt(pDIO(1)^2 + pDIO(2)^2 + 1e2*eps));    

    ElevationConstraint = Elevation - theta;
    AzimuthConstraint   = Azimuth   - psi;

    % Constraint values
    FovConstraint = [ ElevationConstraint; AzimuthConstraint];

end