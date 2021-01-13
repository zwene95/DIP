function [ DefenderConfig ] = defaultDefenderConfig()

    DefenderConfig = struct();        
    
%     defenderConfig.m =                     2;                              % Defender mass
    DefenderConfig.T2W_max                  = 2;                             % Defender thrust to weight ratio        
    DefenderConfig.V_abs_max                = inf;
    DefenderConfig.FovConstraint            = true;                         % true
    DefenderConfig.FoV                      = [120,180];                     % [elevation, azimuth] in[°] .. maximum Seeker + Gimbal FoV
    DefenderConfig.pDOO_0                   = [0; 0; 0];
    DefenderConfig.vDOO_0                   = [0; 0; 0];
    DefenderConfig.MotorTC                  = 20e-3;                        % Motor time constant [ms]
    DefenderConfig.HitConstraint            = false;                        % true if hitconstraint, false if missdistance as const function
    DefenderConfig.ThrustConstraint         = true;                        

end
