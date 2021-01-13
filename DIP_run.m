% Run DefenderInvaderProblem
function [] = DIP_run(SetupSrc, varargin)

clc;

if nargin == 1
    % Rerun setup file
    Setup = SetupSrc;
else
    
    %% Set modeloptions and configs
    Setup = struct();
    
    % Force modelbuild
    Setup.forceBuild = 0;                                                   % Force build process of the model
    
    % Load default options and configs
    Setup.ModelOptions              = defaultModelOptions();
    Setup.DefenderConfig            = defaultDefenderConfig();
    Setup.InvaderConfig             = defaultInvaderConfig();
    Setup.TargetConfig              = defaultTargetConfig();
    Setup.ObserverConfig            = defaultObserverConfig();
    Setup.PostOptions               = defaultPostOptions('PursuitGuidance');
    Setup.CCConfig                  = defaultCCConfig();             
    Setup.Solver                    = defaultSolverConfig();
    
    % Model options
    Setup.ModelOptions.Defender.SixDoF      = 1; 
    Setup.ModelOptions.Defender.MotorLag    = 0;
    Setup.ModelOptions.Defender.Aero        = 0;       
    Setup.ModelOptions.ObservabilityCostFcn = 0;
    % Post options
    Setup.postOptions.Save                  = 1;
    % Target configuration
    Setup.targetConfig.Random               = 0;
    Setup.targetConfig.pTOO                 = [0;-100;-50];
    % Invader configuration
    Setup.invaderConfig.vI_abs_max          = 15;                           % 15/20  
    Setup.invaderConfig.pIOO_0              = [200;0;-50];
    % Cost configuration
    Setup.CCConfig.Missdistance.Cost        = 1;
    Setup.CCConfig.Time.Cost                = 0;
    Setup.CCConfig.TargetViolation.Cost     = 0;
    % Constraint configuration
    Setup.CCConfig.Thrust.Constraint        = 0;
    Setup.CCConfig.Hit.Constraint           = 0;
    Setup.CCConfig.FoV.Constraint           = 0;
    % Scaling configuration
    Setup.CCConfig.Missdistance.Scaling     = .1e+00;%50e-04;                       % 50e-04
    Setup.CCConfig.Time.Scaling             = 1e+00;%55e-02;                       % 50e-02
    Setup.CCConfig.ObserverCov.Scaling      = 50e-06;                       % 50e-06
    Setup.CCConfig.ObserverRMSE.Scaling     = 0e-04;                        % 10e-05
    Setup.CCConfig.TargetViolation.Scaling  = 10e+01;                       % 10e+01
    Setup.CCConfig.Hit.Scaling              = 10e-01;                       % 10e-01
    % Solver configuration
    Setup.Solver.GridSize                   = 2000;                          % 200
    Setup.Solver.MaxIter                    = 500;                         % 500
    Setup.Solver.BackwarEuler               = 0;
    Setup.Solver.Parallel                   = 1;
    
    %% PN FÜR AMDC
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     setup.ModelOptions.observabilityCostFcn = 0;
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
    
    
    % Check ModelOptions for contradictions and revise ModelOptions
    Setup = checkModelOptions(Setup);
    
    % Retrieve modelname
    Setup.modelName = getModelName(Setup.ModelOptions);
    
end

% Initialize problem
[Setup , problem] = DIP_init(Setup);
% Prepare for solving
problem.Bake();
problem.setMajorIterLimit(Setup.Solver.MaxIter);
% Solve problem
problem.Solve();
% Post process
DIP_post(Setup, problem);

if ~Setup.PostOptions.Save
    warning('Results NOT saved, Save-Flag was not set!');
end

problem.Simulate;
problem.PlotGUI;

%EoF
end
