function [stateJacobian_x] = stateJac_x(dt)
%STATEFCN Summary of this function goes here
%   Detailed explanation goes here    
%   StateVec = [x y z u v w];
                    
    stateJacobian_x = [ 1 , 0 , 0 , dt ,  0 , 0
                        0 , 1 , 0 ,  0 , dt , 0
                        0 , 0 , 1 ,  0 ,  0 , dt
                        0 , 0 , 0 ,  1 ,  0 , 0
                        0 , 0 , 0 ,  0 ,  1 , 0
                        0 , 0 , 0 ,  0 ,  0 , 1];

    
end

