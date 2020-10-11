function [ targetConfig ] = defaultTargetConfig()

    targetConfig = struct();    
    targetConfig.Type =                'Cylinder';                         % 'Dome' / 'Cylinder' / 'Circle'
    targetConfig.Random =              true;                               % if false, target will be initialized at [0;0;0]
    targetConfig.seed =                1;                                  % Seed for random number generator
    targetConfig.pTOO =                [0;0;0];
    targetConfig.rT_max =              100;                                % maximum radius of target area (for circle, cylinder and dome)
    targetConfig.rT_min =              100;                                % minimum radius of target area (for circle, cylinder and dome)
    targetConfig.hT_max =              50;                                 % maximum height over ground of target
    targetConfig.hT_min =              0;                                  % minimum height over ground of target  
    targetConfig.targetConstraint =    true;                               % target violation constraint
    
end


