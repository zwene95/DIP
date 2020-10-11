%% Run DefenderInvaderProblem
function [] = DIP_run(setup_src, varargin)
    
    if nargin == 1 
        % Rerun setup file
        setup = setup_src;
%         setup.forceBuild =          true;
    else    
        
        % Set model, post processing and target options
        setup                              	= struct();
        
        % Force modelbuild
        setup.forceBuild                    = false;                    % Force build process of the model        
        
        % Load default options and configs
        setup.modelOptions                  = defaultModelOptions();
        setup.defenderConfig                = defaultDefenderConfig();
        setup.invaderConfig                 = defaultInvaderConfig(); 
        setup.targetConfig                  = defaultTargetConfig();
        setup.observerConfig                = defaultObserverConfig();
        setup.postOptions                   = defaultPostOptions('Test107_est'); 
%         setup.environmentConfig                 = defaultEnvironmentConfig();


        % Modify default options        
        setup.modelOptions.observer                 = true;
        setup.modelOptions.modelName                = 'ModelTest_est';
        setup.modelOptions.target.targetConstraint  = false;
        
     
        %% Config adaption
        % Defender        
        setup.defenderConfig.MotorTC           = 20e-3;                      % 20e-3        
        setup.defenderConfig.FoV               = [120,60]; 
        setup.defenderConfig.V_abs_max         = 90;                         % 30        
        setup.defenderConfig.pDOO_0            = [0; 0; 0];                  % 
        
        % Invader
        setup.invaderConfig.pIOO_0             = [150; 70; -50]*1.15;        % [150; 70; -50]*1.15  
        setup.invaderConfig.vI_abs_max         = 15;                         % 20        
        setup.invaderConfig.rEscape            = 0;                          % 0
        
        % Target
        setup.targetConfig.rT_max              = 100;                        % 100                               
        setup.targetConfig.rT_min              = 20;        
        setup.targetConfig.seed                = 2;
        
        
    end
    
    % Set solver options
    setup.gridSize 	= 200;                                                  % 200
    setup.maxIter	= 500;                                                  % 500    
         
    % Initialize problem
    [setup , problem] = DIP_init(setup);
    % Prepare for solving
    problem.Bake();    
    problem.setMajorIterLimit(setup.maxIter);
    % Solve problem
    problem.Solve();
    % Post process
    DIP_post(setup, problem);
   
%EoF    
end
    