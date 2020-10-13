function [stateVec_dot] = f(stateVec,controlVec,dt)
%STATEFCN Summary of this function goes here
%   Detailed explanation goes here    
%   stateVec = [x y z u v w];
%   controlVec = [ax ay az];
%   w ... process noise
                    
    stateVec_dot = [    0 , 0 , 0 ,  1 ,  0 , 0
                        0 , 0 , 0 ,  0 ,  1 , 0
                        0 , 0 , 0 ,  0 ,  0 , 1
                        0 , 0 , 0 ,  0 ,  0 , 0
                        0 , 0 , 0 ,  0 ,  0 , 0
                        0 , 0 , 0 ,  0 ,  0 , 0
                    ] * stateVec...
                    - ...
                    [   0 , 0 , 0
                        0 , 0 , 0
                        0 , 0 , 0
                        1 , 0 , 0
                        0 , 1 , 0
                        0 , 0 , 1
                    ] * controlVec;
                    
                
                
                
    
end

