function [meas_est] = obsEstMeasurement(x_est, y_est, z_est)
    % EKF estimated measurement
    
    % Estimated LOS angles
    azimuth_est    =   atan2(y_est , x_est);
    elevation_est  =   atan2(-z_est , sqrt(x_est^2 + y_est^2 + z_est^2));
    
    meas_est = [
        azimuth_est
        elevation_est
    ];
                                                           
    
end

