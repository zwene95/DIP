function [ observerConfig ] = defaultObserverConfig()

    observerConfig = struct();      
    
    observerConfig.P0       = diag([1e2,1e2,1e2,5e2,5e2,5e2])   * 1e-0;     % Initial convariance matrix    
    observerConfig.Q        = diag([1e2,1e2,1e2])               * 1e-0;     % Process noise covariance matrix
    observerConfig.R        = diag([1e-2,1e-2])                 * 1e+0;     % Measurement noise covariance matrix    
    observerConfig.Sigma_x0 = 10; %10                                       % Standard deviation for initial position state vector bias      
    observerConfig.Sigma_w  = 0;                                            % Process noise standard deviation
    observerConfig.Sigma_v  = 0;                                            % Measurement noise standard deviation
    observerConfig.Seed     = 2018;                                         % (3571) Seed for random number generator for initial state vector bias
    observerConfig.spr      = 1e-4;                                         % Singularity prevention term for jacobians                      
end
