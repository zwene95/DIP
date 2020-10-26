% Run DefenderInvaderProblem
function [] = DIP_run(setup_src, varargin)

    clear;clc;
    
    if nargin == 1 
        % Rerun setup file
        setup = setup_src;
    else    
        
        %% Set modeloptions and configs
        setup                              	= struct();
        
        % Force modelbuild
        setup.forceBuild                    = 0;                        % Force build process of the model        
        
        % Load default options and configs
        setup.modelOptions                  = defaultModelOptions();
        setup.defenderConfig                = defaultDefenderConfig();
        setup.invaderConfig                 = defaultInvaderConfig();
        setup.targetConfig                  = defaultTargetConfig();
        setup.observerConfig                = defaultObserverConfig();
        setup.postOptions                   = defaultPostOptions('Test3_obs');
        
        % Modify default modeloptions        
        setup.modelOptions.observer             = 0;        
        setup.modelOptions.defender.MotorLag    = 1;
        setup.modelOptions.defender.Aero        = 1;
        setup.modelOptions.defender.SixDoF      = 1;
        
        % Modify default postOptions
        setup.postOptions.save                  = 0;
     
        % Modify default config 
        % Defender                
        setup.defenderConfig.FovConstraint      = 0; 
        setup.defenderConfig.HitConstraint      = 1;
        % Invader        
        setup.invaderConfig.vI_abs_max          = 0;                           % 20                        
        % Target
        setup.targetConfig.targetConstraint     = 0;                      
        % Observer
        setup.observerConfig.std_pos            = 0;
        setup.observerConfig.std_vel            = 0;
        setup.observerConfig.spr                = 1e-2;                         % 1e-?        
        % Solver options
        setup.solver.gridSize                   = 50;                           % 200
        setup.solver.maxIter                    = 500;                         % 500           
        
        % Retrieve modelname 
        setup.modelName = getModelName(setup.modelOptions);   
        
    end
         
    % Initialize problem
    [setup , problem] = DIP_init(setup);
    % Prepare for solving
    problem.Bake();    
    problem.setMajorIterLimit(setup.solver.maxIter);
    % Solve problem
    problem.Solve();
    % Post process
    DIP_post(setup, problem);
   
%EoF    
end
    