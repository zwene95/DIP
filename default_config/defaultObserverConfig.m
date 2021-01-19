function [ ObserverConfig ] = defaultObserverConfig()

    ObserverConfig = struct();      
    
    ObserverConfig.StepTime     = 10e-03;                                   % Maximum observer step time
    ObserverConfig.Std_r0       = 20;                                       % Initial radius standard deviation (seeker detection range)
    ObserverConfig.Std_r0_dot   = 50;                                       % Initial radius rate standard deviation (max Invader velocity)
    ObserverConfig.Std_Qw       = 10;                                       % Process covariance standard deviation
    ObserverConfig.Std_Rv       = 5e-2;                                     % Measurement covariance standard deviation
    ObserverConfig.Std_w        = 0;                                        % Process noise standard deviation
    ObserverConfig.Std_v        = 0;                                        % Measurement noise standard deviation
    ObserverConfig.Seed         = 2018;                                     % (3571) Seed for random number generator for initial state vector bias
    ObserverConfig.Spr          = 1e-4;                                  	% Singularity prevention term for jacobians                      
end
