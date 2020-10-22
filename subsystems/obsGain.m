function [K] = obsGain(P, H, R)
    % EKF - Covariance Predict-Update
    
%     K = P * H' * R;                              
    K = P * H' / R;                 
    
end

