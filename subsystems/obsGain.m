function [K] = obsGain(P, H, R)
    % EKF - Covariance Predict-Update
    
    K = P * H' * R^(-1) * 0;                                                 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
end

