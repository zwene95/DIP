function [ observerConfig ] = defaultObserverConfig()

    observerConfig = struct();    
    
    observerConfig.seed       = 3571;                                     % Seed for random number generator for initial state vector bias  
    observerConfig.std_pos    = 0; %10                                      % Standard deviation for initial position state vector bias  
    observerConfig.std_vel    = 0; %10                                       % Standard deviation for initial velocity state vector bias  
    observerConfig.P_0        = diag([1e2,1e2,1e2,1e2,1e2,1e2]);          % Initial convariance matrix    
    observerConfig.Q          = diag([1e2,1e2,1e2]);                      % Process noise covariance matrix
    observerConfig.R          = diag([1e-2,1e-2]);                        % Measurement noise covariance matrix
    
end
