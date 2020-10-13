function [measurements] = measFcn(stateVec,controlVec)
%MEAS Summary of this function goes here
%   Detailed explanation goes here
%   stateVec = [x y z u v w];
%   measurements = [azimuth;elevation];

    x = stateVec(1);
    y = stateVec(2);
    z = stateVec(3);

    measurements = [atan2(y,x)
                    atan2(-z,sqrt(x^2 + y^2))];

%     measurements = [atan2(y,x)];                                          %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

end

