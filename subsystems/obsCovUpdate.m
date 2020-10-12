function [...
    P_11_dot, P_12_dot, P_13_dot, P_14_dot, P_15_dot, P_16_dot,...
              P_22_dot, P_23_dot, P_24_dot, P_25_dot, P_26_dot,...
                        P_33_dot, P_34_dot, P_35_dot, P_36_dot,...
                                  P_44_dot, P_45_dot, P_46_dot,...
                                            P_55_dot, P_56_dot,...
                                                      P_66_dot...
    ] = obsCovUpdate(F_x, F_w, P, H, K, Q)
    % EKF - Covariance Predict-Update
    
%     Q = eye(3); %diag([Q11, Q22, Q33]);
%     R = eye(2); %diag([R11, R22]);
    
%     K = P * H' * R^-1;
    
%     P_dot = zeros(6,6); %F_x * P + P * F_x' - K * H * P + F_w * Q * F_w';                            
    
    P_dot = (F_x * P + P * F_x' - K * H * P + F_w * Q * F_w' );
    
    P_11_dot = P_dot(1,1);
    P_12_dot = P_dot(1,2);
    P_13_dot = P_dot(1,3);
    P_14_dot = P_dot(1,4);
    P_15_dot = P_dot(1,5);
    P_16_dot = P_dot(1,6);
    P_22_dot = P_dot(2,2);
    P_23_dot = P_dot(2,3);
    P_24_dot = P_dot(2,4);
    P_25_dot = P_dot(2,5);
    P_26_dot = P_dot(2,6);
    P_33_dot = P_dot(3,3);
    P_34_dot = P_dot(3,4);
    P_35_dot = P_dot(3,5);
    P_36_dot = P_dot(3,6);
    P_44_dot = P_dot(4,4);
    P_45_dot = P_dot(4,5);
    P_46_dot = P_dot(4,6);
    P_55_dot = P_dot(5,5);
    P_56_dot = P_dot(5,6);
    P_66_dot = P_dot(6,6);
    
    
    
end

