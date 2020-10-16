function [meas_true, Jacobian] = obsTrueMeasurementDS(...
    x , y , z, x_inv, y_inv, z_inv, spr)
    % EKF true measurement
    
    x_tmp = x_inv - x;
    y_tmp = y_inv - y;
    z_tmp = z_inv - z;
    
    % True LOS angles
    azimuth_true    =   atan2(y_tmp , x_tmp);
    elevation_true  =   atan2(-z_tmp , sqrt(x_tmp^2 + y_tmp^2 + z_tmp^2));
    
    meas_true = [   
        azimuth_true
        elevation_true
    ];

    Jacobian = [
        -(y - y_inv)/((x - x_inv)^2 + (y - y_inv)^2 + spr)  ,   -((2*x - 2*x_inv)*(z - z_inv))/(2*((x - x_inv)^2 + (y - y_inv)^2)^(1/2)*((x - x_inv)^2 + (y - y_inv)^2 + (z - z_inv)^2) + spr)
        (x - x_inv)/((x - x_inv)^2 + (y - y_inv)^2  + spr)  ,   -((2*y - 2*y_inv)*(z - z_inv))/(2*((x - x_inv)^2 + (y - y_inv)^2)^(1/2)*((x - x_inv)^2 + (y - y_inv)^2 + (z - z_inv)^2) + spr)
        0                                                   ,   ((x - x_inv)^2 + (y - y_inv)^2)^(1/2)/((x - x_inv)^2 + (y - y_inv)^2 + (z - z_inv)^2 + spr)
        (y - y_inv)/((x - x_inv)^2 + (y - y_inv)^2  + spr)  ,   ((2*x - 2*x_inv)*(z - z_inv))/(2*((x - x_inv)^2 + (y - y_inv)^2)^(1/2)*((x - x_inv)^2 + (y - y_inv)^2 + (z - z_inv)^2)  + spr)
        -(x - x_inv)/((x - x_inv)^2 + (y - y_inv)^2 + spr)  ,   ((2*y - 2*y_inv)*(z - z_inv))/(2*((x - x_inv)^2 + (y - y_inv)^2)^(1/2)*((x - x_inv)^2 + (y - y_inv)^2 + (z - z_inv)^2)  + spr)
        0                                                   ,   -((x - x_inv)^2 + (y - y_inv)^2)^(1/2)/((x - x_inv)^2 + (y - y_inv)^2 + (z - z_inv)^2 + spr)
    ]';

    
end

