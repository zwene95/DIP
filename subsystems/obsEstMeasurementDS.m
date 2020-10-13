function [meas_est, Jacobian] = obsEstMeasurementDS(x_est, y_est, z_est)
    % EKF estimated measurement
    
    % Estimated LOS angles
    azimuth_meas    =   atan2(y_est , x_est);
    elevation_meas  =   atan2(-z_est , sqrt(x_est^2 + y_est^2 + z_est^2));
    
    meas_est = [
        azimuth_meas
        elevation_meas
    ];

     Jacobian  = [  
         -y_est/(x_est^2 + y_est^2 + eps), (x_est*z_est)/((x_est^2 + y_est^2)^(1/2)*(x_est^2 + y_est^2 + z_est^2)    + eps)
         x_est/(x_est^2 + y_est^2  + eps), (y_est*z_est)/((x_est^2 + y_est^2)^(1/2)*(x_est^2 + y_est^2 + z_est^2)    + eps)
         0                               , -(x_est^2 + y_est^2)^(1/2)/(x_est^2 + y_est^2 + z_est^2                   + eps)
     ]';
                                                           
    
end

