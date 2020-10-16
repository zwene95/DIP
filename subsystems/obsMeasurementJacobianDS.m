function [H, Jacobian] = obsMeasurementJacobianDS(...
    x, y , z, x_inv, y_inv, z_inv, spr)
    % EKF measurement jacobian
    
    x_tmp = x_inv - x;
    y_tmp = y_inv - y;
    z_tmp = z_inv - z;     
    
    % Measurement jacobian            
    H = [   
        -y_tmp/(x_tmp^2 + y_tmp^2 + spr), (x_tmp*z_tmp)/((x_tmp^2 + y_tmp^2)^(1/2)*(x_tmp^2 + y_tmp^2 + z_tmp^2) + spr)   
        x_tmp/(x_tmp^2 + y_tmp^2  + spr), (y_tmp*z_tmp)/((x_tmp^2 + y_tmp^2)^(1/2)*(x_tmp^2 + y_tmp^2 + z_tmp^2) + spr)   
        0                               , -(x_tmp^2 + y_tmp^2)^(1/2)/(x_tmp^2 + y_tmp^2 + z_tmp^2 + spr)                  
        0                               , 0
        0                               , 0
        0                               , 0
    ]';
        
            
    Jacobian = [
                                                                                                                                                                                                                                                                       ((2*x - 2*x_inv)*(y - y_inv))/((x - x_inv)^2 + (y - y_inv)^2 + spr)^2,                                                                                                                                                                                                                               1/((x - x_inv)^2 + (y - y_inv)^2 + spr) - ((2*x - 2*x_inv)*(x - x_inv))/((x - x_inv)^2 + (y - y_inv)^2 + spr)^2,                                                                                                                                                                                                                         0,                                                                                                                                                                                                                                                              -((2*x - 2*x_inv)*(y - y_inv))/((x - x_inv)^2 + (y - y_inv)^2 + spr)^2,                                                                                                                                                                                                                               ((2*x - 2*x_inv)*(x - x_inv))/((x - x_inv)^2 + (y - y_inv)^2 + spr)^2 - 1/((x - x_inv)^2 + (y - y_inv)^2 + spr),                                                                                                                                                                                             0
                                                                                                                                                                                                                                       1/((x - x_inv)^2 + (y - y_inv)^2 + spr) - ((2*x - 2*x_inv)*(x - x_inv))/((x - x_inv)^2 + (y - y_inv)^2 + spr)^2,                                                                                                                                                                                                                                                              -((2*y - 2*y_inv)*(x - x_inv))/((x - x_inv)^2 + (y - y_inv)^2 + spr)^2,                                                                                                                                                                                                                         0,                                                                                                                                                                                                                               1/((x - x_inv)^2 + (y - y_inv)^2 + spr) - ((2*y - 2*y_inv)*(y - y_inv))/((x - x_inv)^2 + (y - y_inv)^2 + spr)^2,                                                                                                                                                                                                                                                               ((2*y - 2*y_inv)*(x - x_inv))/((x - x_inv)^2 + (y - y_inv)^2 + spr)^2,                                                                                                                                                                                             0
                                                                                                                                                                                                                                                                                                                             0,                                                                                                                                                                                                                                                                                                                     0,                                                                                                                                                                                                                         0,                                                                                                                                                                                                                                                                                                                     0,                                                                                                                                                                                                                                                                                                                     0,                                                                                                                                                                                             0
                                                                                                                                                                                                                                                                      -((2*x - 2*x_inv)*(y - y_inv))/((x - x_inv)^2 + (y - y_inv)^2 + spr)^2,                                                                                                                                                                                                                               1/((x - x_inv)^2 + (y - y_inv)^2 + spr) - ((2*y - 2*y_inv)*(y - y_inv))/((x - x_inv)^2 + (y - y_inv)^2 + spr)^2,                                                                                                                                                                                                                         0,                                                                                                                                                                                                                                                               ((2*x - 2*x_inv)*(y - y_inv))/((x - x_inv)^2 + (y - y_inv)^2 + spr)^2,                                                                                                                                                                                                                               1/((x - x_inv)^2 + (y - y_inv)^2 + spr) - ((2*x - 2*x_inv)*(x - x_inv))/((x - x_inv)^2 + (y - y_inv)^2 + spr)^2,                                                                                                                                                                                             0
                                                                                                                                                                                                                                       ((2*x - 2*x_inv)*(x - x_inv))/((x - x_inv)^2 + (y - y_inv)^2 + spr)^2 - 1/((x - x_inv)^2 + (y - y_inv)^2 + spr),                                                                                                                                                                                                                                                               ((2*y - 2*y_inv)*(x - x_inv))/((x - x_inv)^2 + (y - y_inv)^2 + spr)^2,                                                                                                                                                                                                                         0,                                                                                                                                                                                                                               1/((x - x_inv)^2 + (y - y_inv)^2 + spr) - ((2*x - 2*x_inv)*(x - x_inv))/((x - x_inv)^2 + (y - y_inv)^2 + spr)^2,                                                                                                                                                                                                                                                              -((2*y - 2*y_inv)*(x - x_inv))/((x - x_inv)^2 + (y - y_inv)^2 + spr)^2,                                                                                                                                                                                             0
                                                                                                                                                                                                                                                                                                                             0,                                                                                                                                                                                                                                                                                                                     0,                                                                                                                                                                                                                         0,                                                                                                                                                                                                                                                                                                                     0,                                                                                                                                                                                                                                                                                                                     0,                                                                                                                                                                                             0
         ((2*x - 2*x_inv)^2*(z - z_inv))/(2*((x - x_inv)^2 + (y - y_inv)^2 + spr)^(1/2)*((x - x_inv)^2 + (y - y_inv)^2 + (z - z_inv)^2)^2 + spr) - (z - z_inv)/(((x - x_inv)^2 + (y - y_inv)^2)^(1/2)*((x - x_inv)^2 + (y - y_inv)^2 + (z - z_inv)^2) + spr) + ((2*x - 2*x_inv)^2*(z - z_inv))/(4*((x - x_inv)^2 + (y - y_inv)^2)^(3/2)*((x - x_inv)^2 + (y - y_inv)^2 + (z - z_inv)^2) + spr),                                                                   ((2*x - 2*x_inv)*(2*y - 2*y_inv)*(z - z_inv))/(2*((x - x_inv)^2 + (y - y_inv)^2)^(1/2)*((x - x_inv)^2 + (y - y_inv)^2 + (z - z_inv)^2)^2 + spr) + ((2*x - 2*x_inv)*(2*y - 2*y_inv)*(z - z_inv))/(4*((x - x_inv)^2 + (y - y_inv)^2)^(3/2)*((x - x_inv)^2 + (y - y_inv)^2 + (z - z_inv)^2) + spr),                             (2*x - 2*x_inv)/(2*((x - x_inv)^2 + (y - y_inv)^2)^(1/2)*((x - x_inv)^2 + (y - y_inv)^2 + (z - z_inv)^2) + spr) - ((2*x - 2*x_inv)*((x - x_inv)^2 + (y - y_inv)^2)^(1/2))/((x - x_inv)^2 + (y - y_inv)^2 + (z - z_inv)^2 + spr)^2, (z - z_inv)/(((x - x_inv)^2 + (y - y_inv)^2)^(1/2)*((x - x_inv)^2 + (y - y_inv)^2 + (z - z_inv)^2) + spr) - ((2*x - 2*x_inv)^2*(z - z_inv))/(2*((x - x_inv)^2 + (y - y_inv)^2)^(1/2)*((x - x_inv)^2 + (y - y_inv)^2 + (z - z_inv)^2)^2 + spr) - ((2*x - 2*x_inv)^2*(z - z_inv))/(4*((x - x_inv)^2 + (y - y_inv)^2)^(3/2)*((x - x_inv)^2 + (y - y_inv)^2 + (z - z_inv)^2) + spr),                                                                 - ((2*x - 2*x_inv)*(2*y - 2*y_inv)*(z - z_inv))/(2*((x - x_inv)^2 + (y - y_inv)^2)^(1/2)*((x - x_inv)^2 + (y - y_inv)^2 + (z - z_inv)^2)^2 + spr) - ((2*x - 2*x_inv)*(2*y - 2*y_inv)*(z - z_inv))/(4*((x - x_inv)^2 + (y - y_inv)^2)^(3/2)*((x - x_inv)^2 + (y - y_inv)^2 + (z - z_inv)^2) + spr), ((2*x - 2*x_inv)*((x - x_inv)^2 + (y - y_inv)^2)^(1/2))/((x - x_inv)^2 + (y - y_inv)^2 + (z - z_inv)^2 + spr)^2 - (2*x - 2*x_inv)/(2*((x - x_inv)^2 + (y - y_inv)^2)^(1/2)*((x - x_inv)^2 + (y - y_inv)^2 + (z - z_inv)^2) + spr)
                                                                           ((2*x - 2*x_inv)*(2*y - 2*y_inv)*(z - z_inv))/(2*((x - x_inv)^2 + (y - y_inv)^2)^(1/2)*((x - x_inv)^2 + (y - y_inv)^2 + (z - z_inv)^2)^2 + spr) + ((2*x - 2*x_inv)*(2*y - 2*y_inv)*(z - z_inv))/(4*((x - x_inv)^2 + (y - y_inv)^2)^(3/2)*((x - x_inv)^2 + (y - y_inv)^2 + (z - z_inv)^2) + spr), ((2*y - 2*y_inv)^2*(z - z_inv))/(2*((x - x_inv)^2 + (y - y_inv)^2)^(1/2)*((x - x_inv)^2 + (y - y_inv)^2 + (z - z_inv)^2)^2 + spr) - (z - z_inv)/(((x - x_inv)^2 + (y - y_inv)^2)^(1/2)*((x - x_inv)^2 + (y - y_inv)^2 + (z - z_inv)^2) + spr) + ((2*y - 2*y_inv)^2*(z - z_inv))/(4*((x - x_inv)^2 + (y - y_inv)^2)^(3/2)*((x - x_inv)^2 + (y - y_inv)^2 + (z - z_inv)^2) + spr),                             (2*y - 2*y_inv)/(2*((x - x_inv)^2 + (y - y_inv)^2)^(1/2)*((x - x_inv)^2 + (y - y_inv)^2 + (z - z_inv)^2) + spr) - ((2*y - 2*y_inv)*((x - x_inv)^2 + (y - y_inv)^2)^(1/2))/((x - x_inv)^2 + (y - y_inv)^2 + (z - z_inv)^2 + spr)^2,                                                                 - ((2*x - 2*x_inv)*(2*y - 2*y_inv)*(z - z_inv))/(2*((x - x_inv)^2 + (y - y_inv)^2)^(1/2)*((x - x_inv)^2 + (y - y_inv)^2 + (z - z_inv)^2)^2 + spr) - ((2*x - 2*x_inv)*(2*y - 2*y_inv)*(z - z_inv))/(4*((x - x_inv)^2 + (y - y_inv)^2)^(3/2)*((x - x_inv)^2 + (y - y_inv)^2 + (z - z_inv)^2) + spr), (z - z_inv)/(((x - x_inv)^2 + (y - y_inv)^2)^(1/2)*((x - x_inv)^2 + (y - y_inv)^2 + (z - z_inv)^2) + spr) - ((2*y - 2*y_inv)^2*(z - z_inv))/(2*((x - x_inv)^2 + (y - y_inv)^2)^(1/2)*((x - x_inv)^2 + (y - y_inv)^2 + (z - z_inv)^2)^2 + spr) - ((2*y - 2*y_inv)^2*(z - z_inv))/(4*((x - x_inv)^2 + (y - y_inv)^2)^(3/2)*((x - x_inv)^2 + (y - y_inv)^2 + (z - z_inv)^2) + spr), ((2*y - 2*y_inv)*((x - x_inv)^2 + (y - y_inv)^2)^(1/2))/((x - x_inv)^2 + (y - y_inv)^2 + (z - z_inv)^2 + spr)^2 - (2*y - 2*y_inv)/(2*((x - x_inv)^2 + (y - y_inv)^2)^(1/2)*((x - x_inv)^2 + (y - y_inv)^2 + (z - z_inv)^2) + spr)
                                                                                                                                 (2*x - 2*x_inv)/(2*((x - x_inv)^2 + (y - y_inv)^2)^(1/2)*((x - x_inv)^2 + (y - y_inv)^2 + (z - z_inv)^2) + spr) - ((2*x - 2*x_inv)*((x - x_inv)^2 + (y - y_inv)^2)^(1/2))/((x - x_inv)^2 + (y - y_inv)^2 + (z - z_inv)^2 + spr)^2,                                                                                                                         (2*y - 2*y_inv)/(2*((x - x_inv)^2 + (y - y_inv)^2)^(1/2)*((x - x_inv)^2 + (y - y_inv)^2 + (z - z_inv)^2) + spr) - ((2*y - 2*y_inv)*((x - x_inv)^2 + (y - y_inv)^2)^(1/2))/((x - x_inv)^2 + (y - y_inv)^2 + (z - z_inv)^2 + spr)^2,                                                                                                                            -((2*z - 2*z_inv)*((x - x_inv)^2 + (y - y_inv)^2)^(1/2))/((x - x_inv)^2 + (y - y_inv)^2 + (z - z_inv)^2 + spr)^2,                                                                                             (2*x - 2*x_inv)/(2*((x - x_inv)^2 + (y - y_inv)^2)^(1/2)*((x - x_inv)^2 + (y - y_inv)^2 + (z - z_inv)^2) + spr) - ((2*x - 2*x_inv)*(2*z - 2*z_inv)*(z - z_inv))/(2*((x - x_inv)^2 + (y - y_inv)^2)^(1/2)*((x - x_inv)^2 + (y - y_inv)^2 + (z - z_inv)^2)^2 + spr),                                                                                             (2*y - 2*y_inv)/(2*((x - x_inv)^2 + (y - y_inv)^2)^(1/2)*((x - x_inv)^2 + (y - y_inv)^2 + (z - z_inv)^2) + spr) - ((2*y - 2*y_inv)*(2*z - 2*z_inv)*(z - z_inv))/(2*((x - x_inv)^2 + (y - y_inv)^2)^(1/2)*((x - x_inv)^2 + (y - y_inv)^2 + (z - z_inv)^2)^2 + spr),                                                                                                 ((2*z - 2*z_inv)*((x - x_inv)^2 + (y - y_inv)^2)^(1/2))/((x - x_inv)^2 + (y - y_inv)^2 + (z - z_inv)^2 + spr)^2
         (z - z_inv)/(((x - x_inv)^2 + (y - y_inv)^2)^(1/2)*((x - x_inv)^2 + (y - y_inv)^2 + (z - z_inv)^2) + spr) - ((2*x - 2*x_inv)^2*(z - z_inv))/(2*((x - x_inv)^2 + (y - y_inv)^2)^(1/2)*((x - x_inv)^2 + (y - y_inv)^2 + (z - z_inv)^2)^2 + spr) - ((2*x - 2*x_inv)^2*(z - z_inv))/(4*((x - x_inv)^2 + (y - y_inv)^2)^(3/2)*((x - x_inv)^2 + (y - y_inv)^2 + (z - z_inv)^2) + spr),                                                                 - ((2*x - 2*x_inv)*(2*y - 2*y_inv)*(z - z_inv))/(2*((x - x_inv)^2 + (y - y_inv)^2)^(1/2)*((x - x_inv)^2 + (y - y_inv)^2 + (z - z_inv)^2)^2 + spr) - ((2*x - 2*x_inv)*(2*y - 2*y_inv)*(z - z_inv))/(4*((x - x_inv)^2 + (y - y_inv)^2)^(3/2)*((x - x_inv)^2 + (y - y_inv)^2 + (z - z_inv)^2) + spr), (2*x - 2*x_inv)/(2*((x - x_inv)^2 + (y - y_inv)^2)^(1/2)*((x - x_inv)^2 + (y - y_inv)^2 + (z - z_inv)^2) + spr) - ((2*x - 2*x_inv)*(2*z - 2*z_inv)*(z - z_inv))/(2*((x - x_inv)^2 + (y - y_inv)^2)^(1/2)*((x - x_inv)^2 + (y - y_inv)^2 + (z - z_inv)^2)^2 + spr), ((2*x - 2*x_inv)^2*(z - z_inv))/(2*((x - x_inv)^2 + (y - y_inv)^2)^(1/2)*((x - x_inv)^2 + (y - y_inv)^2 + (z - z_inv)^2)^2 + spr) - (z - z_inv)/(((x - x_inv)^2 + (y - y_inv)^2)^(1/2)*((x - x_inv)^2 + (y - y_inv)^2 + (z - z_inv)^2) + spr) + ((2*x - 2*x_inv)^2*(z - z_inv))/(4*((x - x_inv)^2 + (y - y_inv)^2)^(3/2)*((x - x_inv)^2 + (y - y_inv)^2 + (z - z_inv)^2) + spr),                                                                   ((2*x - 2*x_inv)*(2*y - 2*y_inv)*(z - z_inv))/(2*((x - x_inv)^2 + (y - y_inv)^2)^(1/2)*((x - x_inv)^2 + (y - y_inv)^2 + (z - z_inv)^2)^2 + spr) + ((2*x - 2*x_inv)*(2*y - 2*y_inv)*(z - z_inv))/(4*((x - x_inv)^2 + (y - y_inv)^2)^(3/2)*((x - x_inv)^2 + (y - y_inv)^2 + (z - z_inv)^2) + spr), (2*x - 2*x_inv)/(2*((x - x_inv)^2 + (y - y_inv)^2)^(1/2)*((x - x_inv)^2 + (y - y_inv)^2 + (z - z_inv)^2) + spr) - ((2*x - 2*x_inv)*((x - x_inv)^2 + (y - y_inv)^2)^(1/2))/((x - x_inv)^2 + (y - y_inv)^2 + (z - z_inv)^2 + spr)^2
                                                                         - ((2*x - 2*x_inv)*(2*y - 2*y_inv)*(z - z_inv))/(2*((x - x_inv)^2 + (y - y_inv)^2)^(1/2)*((x - x_inv)^2 + (y - y_inv)^2 + (z - z_inv)^2)^2 + spr) - ((2*x - 2*x_inv)*(2*y - 2*y_inv)*(z - z_inv))/(4*((x - x_inv)^2 + (y - y_inv)^2)^(3/2)*((x - x_inv)^2 + (y - y_inv)^2 + (z - z_inv)^2) + spr), (z - z_inv)/(((x - x_inv)^2 + (y - y_inv)^2)^(1/2)*((x - x_inv)^2 + (y - y_inv)^2 + (z - z_inv)^2) + spr) - ((2*y - 2*y_inv)^2*(z - z_inv))/(2*((x - x_inv)^2 + (y - y_inv)^2)^(1/2)*((x - x_inv)^2 + (y - y_inv)^2 + (z - z_inv)^2)^2 + spr) - ((2*y - 2*y_inv)^2*(z - z_inv))/(4*((x - x_inv)^2 + (y - y_inv)^2)^(3/2)*((x - x_inv)^2 + (y - y_inv)^2 + (z - z_inv)^2) + spr), (2*y - 2*y_inv)/(2*((x - x_inv)^2 + (y - y_inv)^2)^(1/2)*((x - x_inv)^2 + (y - y_inv)^2 + (z - z_inv)^2) + spr) - ((2*y - 2*y_inv)*(2*z - 2*z_inv)*(z - z_inv))/(2*((x - x_inv)^2 + (y - y_inv)^2)^(1/2)*((x - x_inv)^2 + (y - y_inv)^2 + (z - z_inv)^2)^2 + spr),                                                                   ((2*x - 2*x_inv)*(2*y - 2*y_inv)*(z - z_inv))/(2*((x - x_inv)^2 + (y - y_inv)^2)^(1/2)*((x - x_inv)^2 + (y - y_inv)^2 + (z - z_inv)^2)^2 + spr) + ((2*x - 2*x_inv)*(2*y - 2*y_inv)*(z - z_inv))/(4*((x - x_inv)^2 + (y - y_inv)^2)^(3/2)*((x - x_inv)^2 + (y - y_inv)^2 + (z - z_inv)^2) + spr), ((2*y - 2*y_inv)^2*(z - z_inv))/(2*((x - x_inv)^2 + (y - y_inv)^2)^(1/2)*((x - x_inv)^2 + (y - y_inv)^2 + (z - z_inv)^2)^2 + spr) - (z - z_inv)/(((x - x_inv)^2 + (y - y_inv)^2)^(1/2)*((x - x_inv)^2 + (y - y_inv)^2 + (z - z_inv)^2) + spr) + ((2*y - 2*y_inv)^2*(z - z_inv))/(4*((x - x_inv)^2 + (y - y_inv)^2)^(3/2)*((x - x_inv)^2 + (y - y_inv)^2 + (z - z_inv)^2) + spr), (2*y - 2*y_inv)/(2*((x - x_inv)^2 + (y - y_inv)^2)^(1/2)*((x - x_inv)^2 + (y - y_inv)^2 + (z - z_inv)^2) + spr) - ((2*y - 2*y_inv)*((x - x_inv)^2 + (y - y_inv)^2)^(1/2))/((x - x_inv)^2 + (y - y_inv)^2 + (z - z_inv)^2 + spr)^2
                                                                                                                                 ((2*x - 2*x_inv)*((x - x_inv)^2 + (y - y_inv)^2)^(1/2))/((x - x_inv)^2 + (y - y_inv)^2 + (z - z_inv)^2 + spr)^2 - (2*x - 2*x_inv)/(2*((x - x_inv)^2 + (y - y_inv)^2)^(1/2)*((x - x_inv)^2 + (y - y_inv)^2 + (z - z_inv)^2) + spr),                                                                                                                         ((2*y - 2*y_inv)*((x - x_inv)^2 + (y - y_inv)^2)^(1/2))/((x - x_inv)^2 + (y - y_inv)^2 + (z - z_inv)^2 + spr)^2 - (2*y - 2*y_inv)/(2*((x - x_inv)^2 + (y - y_inv)^2)^(1/2)*((x - x_inv)^2 + (y - y_inv)^2 + (z - z_inv)^2) + spr),                                                                                                                             ((2*z - 2*z_inv)*((x - x_inv)^2 + (y - y_inv)^2)^(1/2))/((x - x_inv)^2 + (y - y_inv)^2 + (z - z_inv)^2 + spr)^2,                                                                                                                         (2*x - 2*x_inv)/(2*((x - x_inv)^2 + (y - y_inv)^2)^(1/2)*((x - x_inv)^2 + (y - y_inv)^2 + (z - z_inv)^2) + spr) - ((2*x - 2*x_inv)*((x - x_inv)^2 + (y - y_inv)^2)^(1/2))/((x - x_inv)^2 + (y - y_inv)^2 + (z - z_inv)^2 + spr)^2,                                                                                                                         (2*y - 2*y_inv)/(2*((x - x_inv)^2 + (y - y_inv)^2)^(1/2)*((x - x_inv)^2 + (y - y_inv)^2 + (z - z_inv)^2) + spr) - ((2*y - 2*y_inv)*((x - x_inv)^2 + (y - y_inv)^2)^(1/2))/((x - x_inv)^2 + (y - y_inv)^2 + (z - z_inv)^2 + spr)^2,                                                                                                -((2*z - 2*z_inv)*((x - x_inv)^2 + (y - y_inv)^2)^(1/2))/((x - x_inv)^2 + (y - y_inv)^2 + (z - z_inv)^2 + spr)^2
                                                                                                                                 
    ];
        
        
end

