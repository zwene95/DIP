function [updateStateVec] = stateFcn(stateVec,controlVec,dt)
%STATEFCN Summary of this function goes here
%   Detailed explanation goes here    
%   stateVec = [x y z u v w];
%   controlVec = [ax ay az];
%   w ... process noise
                    
    updateStateVec = [  1 , 0 , 0 , dt ,  0 , 0
                        0 , 1 , 0 ,  0 , dt , 0
                        0 , 0 , 1 ,  0 ,  0 , dt
                        0 , 0 , 0 ,  1 ,  0 , 0
                        0 , 0 , 0 ,  0 ,  1 , 0
                        0 , 0 , 0 ,  0 ,  0 , 1
                    ] * stateVec...
                    - ...
                    [   0.5*dt^2 , 0        , 0
                        0        , 0.5*dt^2 , 0
                        0        , 0        , 0.5*dt^2
                        dt       , 0        , 0
                        0        , dt       , 0
                        0        , 0        , dt
                    ] * controlVec...
                    ;%+ w;
                    
                
                
                
    
end

