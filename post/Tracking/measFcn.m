function [measurements] = measFcn(states, scaling)
%MEAS Summary of this function goes here
%   Detailed explanation goes here
%   stateVec = [x y z u v w];
%   measurements = [azimuth;elevation]; 
    
    x = states(1) / scaling;
    y = states(2) / scaling;
    z = states(3) / scaling;

    measurements = [
        atan2(y,x)
        atan2(-z,sqrt(x^2 + y^2))
    ];

%     measurements = [atan2(y,x)];                                          %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

end

