function [meas_est, Jacobian] = obsEstMeasurementDS(...
    x_est, y_est, z_est, spr)
    % EKF estimated measurement
    
    % Estimated LOS angles
    azimuth_est     =   atan2(y_est , x_est);
    elevation_est   =   atan2(-z_est , sqrt(x_est^2 + y_est^2 + z_est^2));
    
    meas_est = [
        azimuth_est
        elevation_est
    ];

     Jacobian  = [  
         -y_est/(x_est^2 + y_est^2 + spr), (x_est*z_est)/((x_est^2 + y_est^2)^(1/2)*(x_est^2 + y_est^2 + z_est^2)    + spr)
         x_est/(x_est^2 + y_est^2  + spr), (y_est*z_est)/((x_est^2 + y_est^2)^(1/2)*(x_est^2 + y_est^2 + z_est^2)    + spr)
         0                               , -(x_est^2 + y_est^2)^(1/2)/(x_est^2 + y_est^2 + z_est^2                   + spr)
     ]';
                                                           
    
end

