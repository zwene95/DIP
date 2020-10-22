function [stateJacobian_w] = stateJac_w(dt)
%STATEFCN Summary of this function goes here
%   Detailed explanation goes here    
%   StateVec = [x y z u v w];


    stateJacobian_w = [ 0.5*dt^2, 0         , 0 
                        0       , 0.5*dt^2  , 0 
                        0       , 0         , 0.5*dt^2
                        dt      , 0         , 0
                        0       , dt        , 0
                        0       , 0         , dt];
                    
%     stateJacobian_w = [ dt      , 0         , 0   , 0.5*dt^2, 0         , 0 
%                         0       , dt        , 0   , 0       , 0.5*dt^2  , 0 
%                         0       , 0         , dt  , 0       , 0         , 0.5*dt^2
%                         0       , 0         , 0   , dt      , 0         , 0
%                         0       , 0         , 0   , 0       , dt        , 0
%                         0       , 0         , 0   , 0       , 0         , dt];
                        

    
end

