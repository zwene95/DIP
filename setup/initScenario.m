function [ scenario ] = initScenario(setup)
% Set boundary values for given scenario
    
    scenario = struct();
    
    %% Time
    if setup.modelOptions.timeState
        T_0 =   0;
        T_lw =  0;
        T_up =  +inf;
    else
        T_0 =   [];
        T_lw =  [];
        T_up =  [];
    end    
    
    %% Target
    if ~isfield(setup,'scenario')
        if setup.targetConfig.Random
            rng(setup.targetConfig.seed)
            r = setup.targetConfig.rT_min + (setup.targetConfig.rT_max - setup.targetConfig.rT_min) * rand;
            r_base = [-1;-1] + diag([2;2]) * rand(2,1);
            r_base = r_base / norm(r_base) * r;
            h = - setup.targetConfig.hT_min - (setup.targetConfig.hT_max - setup.targetConfig.hT_min) * rand;
            switch setup.targetConfig.Type                
                case 'Circle'
                    scenario.pTOO = [r_base; 0];
                otherwise
                    scenario.pTOO = [r_base; h];
            end    
        else
            scenario.pTOO = setup.targetConfig.pTOO;
        end
    else
        scenario = setup.scenario;
    end
    
    %% Defender
    
    % Defender translational state boundaries
    pDOO_0 =    setup.defenderConfig.pDOO_0;
    vDOO_0 =    setup.defenderConfig.vDOO_0;
    pDOO_lw =   [-inf; -inf; -inf];
    pDOO_up =   [+inf; +inf;    0];
    vDOO_lw =   [-inf; -inf; -inf];
    vDOO_up =   [+inf; +inf; +inf];
    
    % Defender rotational state boundaries
    if setup.modelOptions.defender.SixDoF
        switch setup.modelOptions.defender.Attitude
            case 'Euler'
                att_0 =     [   0;    0;    0];
                wDOB_0 =    [   0;    0;    0];
                att_lw =    [-inf; -inf; -inf];
                att_up =    [+inf; +inf; +inf];
                wDOB_lw =   [-inf; -inf; -inf];
                wDOB_up =   [+inf; +inf; +inf];
            otherwise
                cfcn = dbstack;
                error('Error in %s.\n Attitude option not supported.\n', cfcn.name);  
        end
    else
        att_0 =     [];
        att_lw =    [];
        att_up =    [];
        wDOB_0 =    [];
        wDOB_lw =   [];
        wDOB_up =   [];
    end
    
    % Defender motor state boundaries
    if setup.modelOptions.defender.MotorLag
        motor_init = [1; 1; 1; 1] * 5000;
        motor_lw = [1; 1; 1; 1] * 0;
        motor_up = [1; 1; 1; 1] * 10000;
    else
        motor_init =    [];
        motor_lw =      [];
        motor_up =      [];
    end
    
    %% Invader
    
    % Invader boundaries
    if setup.modelOptions.invader.SixDoF
        error('Error invader 6DoF model not supported');         
    else
        switch setup.modelOptions.invader.Type
            case 'Quad1'
                pIOO_0  = setup.invaderConfig.pIOO_0;                
                pIOO_lw = [-inf; -inf; -inf];
                pIOO_up = [+inf; +inf;    0];
                vIOO_0  = [];
                vIOO_lw = [];
                vIOO_up = [];
                % Compute invader velocity
                pITO = scenario.pTOO - setup.invaderConfig.pIOO_0;
                scenario.vIOO = pITO / norm(pITO) * setup.invaderConfig.vI_abs_max;
            case 'Quad2'
                pIOO_0          = setup.invaderConfig.pIOO_0;                
                pIOO_lw         = [-inf; -inf; -inf];
                pIOO_up         = [+inf; +inf;    0];
                vIOO_0          = setup.invaderConfig.vIOO_0;
                vIOO_lw         = [-inf; -inf; -inf]; 
                vIOO_up         = [+inf; +inf; +inf];
                scenario.vDIO_0 = vIOO_0 - vDOO_0;
            otherwise
                cfcn = dbstack;
                error('Error in %s.\n Scenario option not supported.\n', cfcn.name);  
        end         
    end
            
    %% Get relative states for estimator initialization
    scenario.pDIO_0     = pIOO_0 - pDOO_0;
    scenario.vDIO_0     = scenario.vIOO - vDOO_0;   
    
    
    %% Set initial and final boundaries
    scenario.initBoundaries =       [pDOO_0     ; vDOO_0    ; pIOO_0    ; vIOO_0    ; att_0     ; wDOB_0    ; motor_init    ; T_0];
    scenario.finalBoundaries_lw =   [pDOO_lw    ; vDOO_lw   ; pIOO_lw   ; vIOO_lw   ; att_lw    ; wDOB_lw   ; motor_lw      ; T_lw];
    scenario.finalBoundaries_up =   [pDOO_up    ; vDOO_up   ; pIOO_up   ; vIOO_up   ; att_up    ; wDOB_up   ; motor_up      ; T_up];    
    

end
