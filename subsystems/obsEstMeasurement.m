function [meas_est] = estEstMeasurement(x_est, y_est, z_est)
    % EKF estimated measurement
    
    % Estimated LOS angles
    azimuth_true    =   atan2(y_est , x_est);
    elevation_true  =   atan2(-z_est , sqrt(x_est^2 + y_est^2 + z_est^2));
    
    meas_est = [
        azimuth_true
        elevation_true
    ];
                                                           
    
end

