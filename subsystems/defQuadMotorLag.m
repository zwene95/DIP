function [ w1_dot, w2_dot, w3_dot, w4_dot] = defQuadMotorLag(   w1, w1_cmd,...
                                                                w2, w2_cmd,...
                                                                w3, w3_cmd,...
                                                                w4, w4_cmd,...
                                                                MotorTC)
%
% Target cartesian position propagation.
%
% Inputs:
%   - 
%
% Outputs:
%   - 

    w1_dot = (w1_cmd - w1) / MotorTC;
    w2_dot = (w2_cmd - w2) / MotorTC;
    w3_dot = (w3_cmd - w3) / MotorTC;
    w4_dot = (w4_cmd - w4) / MotorTC;

end

