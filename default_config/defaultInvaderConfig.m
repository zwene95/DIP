function [ InvaderConfig ] = defaultInvaderConfig()

    InvaderConfig = struct();    
    
%     invaderConfig.mI =                     2;                             % invader mass
    InvaderConfig.T2W_max       = 2;                               % invader thrust to weight ratio    
    InvaderConfig.pIOO_0        = [190; 50; -50];           % initial invader position [150; 70; -50] * 1.15 [170; 100; -50]
    InvaderConfig.vIOO_0        = [0  ;   0 ;    0];               % initial invader velocity, für Quad2    
    InvaderConfig.rEscape       = 0;                               % invader evasive maneuver parameter (maneuver at relative defender/invader distance) 
    InvaderConfig.vI_abs_max    = 10;                              % maximum velocity
%     invaderOptions.r0_min =                 1000;                         % minimum initial invader distance 
%     invaderOptions.r0_max =                 1000;                         % minimum initial invader distance
%     invaderOptions.aI_abs_max =             20;                           % maximum acceleration capability

end
