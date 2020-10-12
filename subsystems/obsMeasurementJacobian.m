function [H] = obsMeasurementJacobian(  x    , y    , z,...
                                        x_inv, y_inv, z_inv)
    % EKF measurement jacobian
    
    x_tmp = x_inv - x;
    y_tmp = y_inv - y;
    z_tmp = z_inv - z;
    
    % Measurement jacobian            
    H = [   -y_tmp/(x_tmp^2 + y_tmp^2 + eps), (x_tmp*z_tmp)/((x_tmp^2 + y_tmp^2)^(1/2)*(x_tmp^2 + y_tmp^2 + z_tmp^2) + eps)   
            x_tmp/(x_tmp^2 + y_tmp^2  + eps), (y_tmp*z_tmp)/((x_tmp^2 + y_tmp^2)^(1/2)*(x_tmp^2 + y_tmp^2 + z_tmp^2) + eps)   
            0                               , -(x_tmp^2 + y_tmp^2)^(1/2)/(x_tmp^2 + y_tmp^2 + z_tmp^2 + eps)                  
            0                               , 0
            0                               , 0
            0                               , 0]';

%     H = zeros(2,6);
end

