function [Scenario] = initScenario(Setup)
% Set boundary values for given scenario
    
    Scenario = struct();
    
    %% Time
    if Setup.ModelOptions.TimeState
        T_0 =   0;
        T_lw =  0;
        T_up =  +inf;
    else
        T_0 =   [];
        T_lw =  [];
        T_up =  [];
    end    
    
    %% Target
    if ~isfield(Setup,'Scenario')
        if Setup.TargetConfig.Random
            rng(Setup.TargetConfig.Seed)
            r = Setup.TargetConfig.rT_min + (Setup.TargetConfig.rT_max - Setup.TargetConfig.rT_min) * rand;
            r_base = [-1;-1] + diag([2;2]) * rand(2,1);
            r_base = r_base / norm(r_base) * r;
            h = - Setup.TargetConfig.hT_min - (Setup.TargetConfig.hT_max - Setup.TargetConfig.hT_min) * rand;
            switch Setup.TargetConfig.Type                
                case 'Circle'
                    Scenario.pTOO = [r_base; 0];
                otherwise
                    Scenario.pTOO = [r_base; h];
            end    
        else
            Scenario.pTOO = Setup.TargetConfig.pTOO;
        end
    else
        Scenario = Setup.Scenario;
    end
    
    %% Defender
    
    % Defender translational state boundaries
    pDOO_0 =    Setup.DefenderConfig.pDOO_0;
    vDOO_0 =    Setup.DefenderConfig.vDOO_0;
    pDOO_lw =   [-inf; -inf; -inf]; 
    pDOO_up =   [+inf; +inf; +inf]; 
    vDOO_lw =   [-inf; -inf; -inf];
    vDOO_up =   [+inf; +inf; +inf];
    
    % Defender rotational state boundaries
    if Setup.ModelOptions.Defender.SixDoF
        switch Setup.ModelOptions.Defender.Attitude
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
    if Setup.ModelOptions.Defender.MotorLag
        motor_init = [1; 1; 1; 1] * 5000;
        motor_lw = [1; 1; 1; 1] * 0;
        motor_up = [+inf; +inf; +inf; +inf];
    else
        motor_init =    [];
        motor_lw =      [];
        motor_up =      [];
    end
    
    %% Invader
    
    % Invader boundaries
    if Setup.ModelOptions.Invader.SixDoF
        error('Error invader 6DoF model not supported');         
    else
        switch Setup.ModelOptions.Invader.Type
            case 'Quad1'
                pIOO_0  = Setup.InvaderConfig.pIOO_0;                
                pIOO_lw = [-inf; -inf; -inf];
                pIOO_up = [+inf; +inf; +inf];
                vIOO_0  = [];
                vIOO_lw = [];
                vIOO_up = [];
                % Compute invader velocity
                pITO = Scenario.pTOO - Setup.InvaderConfig.pIOO_0;
                Scenario.vIOO = pITO / norm(pITO) * Setup.InvaderConfig.vI_abs_max;
            case 'Quad2'
                pIOO_0          = Setup.InvaderConfig.pIOO_0;                
                pIOO_lw         = [-inf; -inf; -inf];
                pIOO_up         = [+inf; +inf; +inf];
                vIOO_0          = Setup.InvaderConfig.vIOO_0;
                vIOO_lw         = [-inf; -inf; -inf]; 
                vIOO_up         = [+inf; +inf; +inf];
                Scenario.vDIO_0 = vIOO_0 - vDOO_0;
            otherwise
                cfcn = dbstack;
                error('Error in %s.\n Scenario option not supported.\n', cfcn.name);  
        end         
    end
            
    %% Get relative states for estimator initialization
    Scenario.pDIO_0     = pIOO_0 - pDOO_0;
    Scenario.vDIO_0     = Scenario.vIOO - vDOO_0;   
    
    
    %% Set initial and final boundaries
    Scenario.InitBoundaries =       [pDOO_0     ; vDOO_0    ; pIOO_0    ; vIOO_0    ; att_0     ; wDOB_0    ; motor_init    ; T_0];
    Scenario.FinalBoundaries_lw =   [pDOO_lw    ; vDOO_lw   ; pIOO_lw   ; vIOO_lw   ; att_lw    ; wDOB_lw   ; motor_lw      ; T_lw];
    Scenario.FinalBoundaries_up =   [pDOO_up    ; vDOO_up   ; pIOO_up   ; vIOO_up   ; att_up    ; wDOB_up   ; motor_up      ; T_up];    
    

end
