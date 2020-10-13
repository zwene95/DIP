function [ ekf ] = initEKF(setup)
% Initialize EKF
% [ ekf ] = initEKF(setup)
    
    ekf = struct();
    
    
    
    % Get true initial filter state
    x0_true = [setup.scenario.pDIO_0; setup.scenario.vDIO_0];
    
    % Setup bias in initial filter state x0
    rng(setup.observerConfig.seed);   
    mu_x0       = 0;
    std_x0_pos  = setup.observerConfig.std_pos;
    std_x0_vel  = setup.observerConfig.std_vel;
    b_x0_pos    = normrnd(mu_x0,std_x0_pos,[3 1]);
    b_x0_vel    = normrnd(mu_x0,std_x0_vel,[3 1]);
    b_x0        = [b_x0_pos; b_x0_vel];  
    
    % Initial biased observer state vector
    x0 = x0_true + b_x0;
    
    % Uppder and lower boundary of final state vector
    xf_lw = [-inf; -inf; -inf; -inf; -inf; -inf;];
    xf_up = [+inf; +inf; +inf; +inf; +inf; +inf;];
    
    
    % Initializes covariance matrix
%     P_0     = reshape(setup.observerConfig.P_0(1)',[36,1]);
%     P_lw    = zeros(36,1);
%     P_up    = Inf(36,1);

    P0     = [
        setup.observerConfig.P_0(1,1); zeros(5,1)
        setup.observerConfig.P_0(2,2); zeros(4,1) 
        setup.observerConfig.P_0(3,3); zeros(3,1) 
        setup.observerConfig.P_0(4,4); zeros(2,1) 
        setup.observerConfig.P_0(5,5); zeros(1,1) 
        setup.observerConfig.P_0(6,6)
    ];
    P_lw    = -Inf(21,1);
    P_up    = Inf(21,1);
            
    
    %% Set initial and final boundaries
    ekf.initBoundaries =       [x0      ; P0];
    ekf.finalBoundaries_lw =   [xf_lw   ; P_lw];
    ekf.finalBoundaries_up =   [xf_up   ; P_up];
    

end
