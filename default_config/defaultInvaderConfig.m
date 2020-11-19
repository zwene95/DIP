function [ invaderConfig ] = defaultInvaderConfig()

    invaderConfig = struct();    
    
%     invaderConfig.mI =                     2;                             % invader mass
    invaderConfig.T2W_max       = 2;                               % invader thrust to weight ratio    
    invaderConfig.pIOO_0        = [200; 0; -50];           % initial invader position [150; 70; -50] * 1.15
    invaderConfig.vIOO_0        = [0  ;   0 ;    0];               % initial invader velocity, für Quad2    
    invaderConfig.rEscape       = 0;                               % invader evasive maneuver parameter (maneuver at relative defender/invader distance) 
    invaderConfig.vI_abs_max    = 10;                              % maximum velocity
%     invaderOptions.r0_min =                 1000;                         % minimum initial invader distance 
%     invaderOptions.r0_max =                 1000;                         % minimum initial invader distance
%     invaderOptions.aI_abs_max =             20;                           % maximum acceleration capability

end
