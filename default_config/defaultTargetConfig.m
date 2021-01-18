function [ TargetConfig ] = defaultTargetConfig()

    TargetConfig = struct();    
    TargetConfig.Type                   = 'Cylinder';                       % 'Dome' / 'Cylinder' / 'Circle'
    TargetConfig.Random                 = true;                             % if false, target will be initialized at [0;0;0]
    TargetConfig.Seed                   = 2;                                % Seed for random number generator
    TargetConfig.pTOO                   = [20;-100;0]; %[0;0;0];
    TargetConfig.rT_max                 = 100;                              % maximum radius of target area (for circle, cylinder and dome)
    TargetConfig.rT_min                 = 20;                               % minimum radius of target area (for circle, cylinder and dome)
    TargetConfig.hT_max                 = 50;                               % maximum height over ground of target
    TargetConfig.hT_min                 = 0;                                % minimum height over ground of target      
    TargetConfig.TargetViolationCost    = 0;                                % Target violation cost 
    
end


