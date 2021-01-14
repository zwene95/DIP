function [ DefenderConfig ] = defaultDefenderConfig()

    DefenderConfig = struct();        
    
%     defenderConfig.m =                     2;                             % Defender mass
    DefenderConfig.T2W_max                  = 2;                            % Defender thrust to weight ratio        
    DefenderConfig.V_abs_max                = inf;                          % Maximum defender velocity (mostly relevant for non-aero implemented)
    DefenderConfig.FovConstraint            = true;                         % Field of View constraint
    DefenderConfig.FoV                      = [120,180];                    % Maximum defender FoV [Elevation, Azimuth] in[°] .. maximum Seeker + Gimbal FoV
    DefenderConfig.pDOO_0                   = [0; 0; 0];                    % Initial defender position
    DefenderConfig.vDOO_0                   = [0; 0; 0];                    % Initial defender velocity
    DefenderConfig.MotorTC                  = 20e-3;                        % Motor time constant [ms]
    
end
