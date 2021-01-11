% Run DefenderInvaderProblem
function [] = DIP_run(setup_src, varargin)

clc;

if nargin == 1
    % Rerun setup file
    setup = setup_src;
else
    
    %% Set modeloptions and configs
    setup = struct();
    
    % Force modelbuild
    setup.forceBuild = 0;                                                   % Force build process of the model
    
    % Load default options and configs
    setup.modelOptions              = defaultModelOptions();
    setup.defenderConfig            = defaultDefenderConfig();
    setup.invaderConfig             = defaultInvaderConfig();
    setup.targetConfig              = defaultTargetConfig();
    setup.observerConfig            = defaultObserverConfig();
    setup.postOptions               = defaultPostOptions('Test_Sim');
    setup.CCConfig                  = defaultCCConfig();             
    setup.Solver                    = defaultSolverConfig();
    
    % Model options
    setup.modelOptions.defender.SixDoF      = 1; 
    setup.modelOptions.defender.MotorLag    = 0;
    setup.modelOptions.defender.Aero        = 0;       
    setup.modelOptions.observabilityCostFcn = 0;
    % Post options
    setup.postOptions.Save                  = 1;
    % Target configuration
    setup.targetConfig.Random               = 0;
    setup.targetConfig.pTOO                 = [0;-100;-50];
    % Invader configuration
    setup.invaderConfig.vI_abs_max          = 15;                           % 20  
    setup.invaderConfig.pIOO_0              = [200;0;-50];
    % Cost configuration
    setup.CCConfig.Missdistance.Cost        = 1;
    setup.CCConfig.Time.Cost                = 1;
    setup.CCConfig.TargetViolation.Cost     = 1;
    % Constraint configuration
    setup.CCConfig.Thrust.Constraint        = 0;
    setup.CCConfig.Hit.Constraint           = 0;
    setup.CCConfig.FoV.Constraint           = 0;
    % Scaling configuration
    setup.CCConfig.Missdistance.Scaling     = .1e+00;%50e-04;                       % 50e-04
    setup.CCConfig.Time.Scaling             = 1e+00;%55e-02;                       % 50e-02
    setup.CCConfig.ObserverCov.Scaling      = 50e-06;                       % 50e-06
    setup.CCConfig.ObserverRMSE.Scaling     = 0e-04;                        % 10e-05
    setup.CCConfig.TargetViolation.Scaling  = 10e+01;                       % 10e+01
    setup.CCConfig.Hit.Scaling              = 10e-01;                       % 10e-01
    % Solver configuration
    setup.Solver.GridSize                   = 2000;                          % 200
    setup.Solver.MaxIter                    = 500;                         % 500
    setup.Solver.BackwarEuler               = 0;
    setup.Solver.Parallel                   = 1;
    
    %% PN FÜR AMDC
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     setup.modelOptions.observabilityCostFcn = 0;
%     setup.Solver.CostScalingTime = 1;%0.5000;
%     setup.Solver.CostScalingMiss = 1;%0.0050;
%     setup.Solver.GridSize = 200;
%     setup.defenderConfig.pDOO_0 = [0;0;0];
%     setup.invaderConfig.pIOO_0 = [250; 0; -50]/2;
%     setup.invaderConfig.vIOO_0 = [0 -1 0]/sqrt(1)*15;
%     setup.targetConfig.pTOO = [250;-1000;-50]/2;
%     setup.invaderConfig.vI_abs_max = 15;
%     setup.targetConfig.Random = 0;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
    %%
    
    
    % Check modelOptions for contradictions and revise modelOptions
    setup = checkModelOptions(setup);
    
    % Retrieve modelname
    setup.modelName = getModelName(setup.modelOptions);
    
end

% Initialize problem
[setup , problem] = DIP_init(setup);
% Prepare for solving
problem.Bake();
problem.setMajorIterLimit(setup.Solver.MaxIter);
% Solve problem
problem.Solve();
% Post process
DIP_post(setup, problem);

if ~setup.postOptions.Save
    warning('Results NOT saved, Save-Flag was not set!');
end

problem.Simulate;
problem.PlotGUI;

%EoF
end
