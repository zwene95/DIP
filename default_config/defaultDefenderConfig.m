function [ defenderConfig ] = defaultDefenderConfig()

    defenderConfig = struct();        
    
%     defenderConfig.m =                     2;                              % Defender mass
    defenderConfig.T2W_max                  = 2;                              % Defender thrust to weight ratio        
    defenderConfig.V_abs_max                = inf;
    defenderConfig.FovConstraint            = true;                       % true
    defenderConfig.FoV                      = [120,60];                       % [elevation, azimuth] in[°] .. maximum Seeker + Gimbal FoV
    defenderConfig.pDOO_0                   = [0; 0; 0];
    defenderConfig.vDOO_0                   = [0; 0; 0];
    defenderConfig.MotorTC                  = 20e-3;                          % Motor time constant [ms]
    defenderConfig.HitConstraint            = false;                         % true if hitconstraint, false if missdistance as const function

end
