function [ ObserverConfig ] = defaultObserverConfig()

    ObserverConfig = struct();      
    
    ObserverConfig.TimeStep = 10e-03;                                       % Maximum observer time step
    ObserverConfig.P0       = diag([1e2,1e2,1e2,5e2,5e2,5e2])   * 1e-0;     % Initial convariance matrix    
    ObserverConfig.Q        = diag([1e2,1e2,1e2])               * 1e-0;     % Process noise covariance matrix
    ObserverConfig.R        = diag([1e-2,1e-2])                 * 1e+0;     % Measurement noise covariance matrix    
    ObserverConfig.Sigma_x0 = 10; %10                                       % Standard deviation for initial position state vector bias      
    ObserverConfig.Sigma_w  = 0;                                            % Process noise standard deviation
    ObserverConfig.Sigma_v  = 0;                                            % Measurement noise standard deviation
    ObserverConfig.Seed     = 2018;                                         % (3571) Seed for random number generator for initial state vector bias
    ObserverConfig.Spr      = 1e-4;                                         % Singularity prevention term for jacobians                      
end
