function [...  
    x_est_dot, y_est_dot, z_est_dot,...
    u_est_dot, v_est_dot, w_est_dot...
] = obsStateUpdate(...
        u_est, v_est, w_est,...
        u_dot, v_dot, w_dot,...
        meas_true, meas_est, K)
    % EKF - State Predict-Update
    
    
%     R = diag([R11, R22]);
    
    % Measurement Update
    update = K * (meas_true - meas_est);
    
    % Position Prediction-Update
    x_est_dot = u_est + update(1);
    y_est_dot = v_est + update(2);
    z_est_dot = w_est + update(3);
    
    % Velocity Prediction-Update
    u_est_dot = - u_dot + update(4);
    v_est_dot = - v_dot + update(5);
    w_est_dot = - w_dot + update(6);                                  
    
end

