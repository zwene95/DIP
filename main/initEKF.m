function [ ekf ] = initEKF(setup)
% Initialize EKF
% [ ekf ] = initEKF(setup)
    
    ekf = struct();    
    
    % Get true initial filter state
    x0_true = [setup.scenario.pDIO_0; setup.scenario.vDIO_0];
    
    % Setup bias in initial filter state x0
    rng(setup.observerConfig.Seed);   
    mu_x0       = 0;
    std_x0_pos  = setup.observerConfig.StdPos;
%     std_x0_vel  = setup.observerConfig.StdVel;
    b_x0_pos    = normrnd(mu_x0,std_x0_pos,[3 1]);
%     b_x0_vel    = normrnd(mu_x0,std_x0_vel,[3 1]);
    b_x0_vel    = -x0_true(4:6);
    b_x0        = [b_x0_pos; b_x0_vel];  
    
    % Initial biased observer state vector
    x0_est = x0_true + b_x0;
    
    % Uppder and lower boundary of final state vector
    xf_lw_est = [-inf; -inf; -inf; -inf; -inf; -inf;];
    xf_up_est = [+inf; +inf; +inf; +inf; +inf; +inf;];
    
    
    % Initializes covariance matrix
%     P0      = reshape(setup.observerConfig.P_0,[36,1]);
%     P_lw    = -Inf(36,1);
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
    ekf.initBoundaries      = [x0_est       ; P0];
    ekf.finalBoundaries_lw  = [xf_lw_est    ; P_lw];
    ekf.finalBoundaries_up  = [xf_up_est    ; P_up];
    

end
