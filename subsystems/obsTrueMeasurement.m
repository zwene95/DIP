function [meas_true] = obsTrueMeasurement(  x , y , z,...
                                            x_inv, y_inv, z_inv)
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
                                                           
    
end

