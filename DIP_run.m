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
    Setup.PostOptions               = defaultPostOptions('Test_LosRate');
    Setup.CCConfig                  = defaultCCConfig();             
    Setup.Solver                    = defaultSolverConfig();
    
    % Model options
    Setup.ModelOptions.Defender.SixDoF      = 1; 
    Setup.ModelOptions.Defender.MotorLag    = 0;
    Setup.ModelOptions.Defender.Aero        = 1;       
    Setup.ModelOptions.ObservabilityCostFcn = 0;    
    % Target configuration
    Setup.TargetConfig.Random               = 0;
    Setup.TargetConfig.pTOO                 = [20;-150;0];                  %[0;-100;-50];
    % Invader configuration
    Setup.InvaderConfig.vI_abs_max          = 15;                           % 15/20  
    Setup.InvaderConfig.pIOO_0              = [200;0;-50];
    % Cost configuration
    Setup.CCConfig.Cost.Missdistance        = 1;
    Setup.CCConfig.Cost.Time                = 0;
    Setup.CCConfig.Cost.TargetViolation     = 0;
    Setup.CCConfig.Cost.LosRate             = 1;
    % Constraint configuration
    Setup.CCConfig.Constraint.FoV           = 0;
    % Scaling configuration
    Setup.CCConfig.Scaling.Missdistance     = .1e+00;%50e-04;                       % 50e-04
    Setup.CCConfig.Scaling.Time             = 1e00;%55e-02;                       % 50e-02
    Setup.CCConfig.Scaling.ObserverCov      = 50e-06;                       % 50e-06
    Setup.CCConfig.Scaling.ObserverRMSE     = 0e-04;                        % 10e-05
    Setup.CCConfig.Scaling.TargetViolation  = 10e+01;                       % 10e+01    
    Setup.CCConfig.Scaling.LosRate          = 1e-06;
    % Solver configuration
    Setup.Solver.GridSize                   = 1000;                         % 200
    Setup.Solver.MaxIter                    = 500;                          % 500
    Setup.Solver.BackwarEuler               = 0;
    Setup.Solver.Parallel                   = 1;
    % Post options
    Setup.PostOptions.Save                  = 1;
    
    % Retrieve modelname
    Setup.ModelName = getModelName(Setup.ModelOptions);    
    
end

% Build scenario
Setup.Scenario = buildScenario(Setup);
% Build Optimal Control Problem
Problem = buildOCP(Setup);
% Prepare for solving
Problem.Bake();
Problem.setMajorIterLimit(Setup.Solver.MaxIter);
% Solve problem
Problem.Solve();
% Post process
DIP_post(Setup, Problem);

if ~Setup.PostOptions.Save
    warning('Results NOT saved, Save-Flag was not set!');
end

Problem.Simulate;
Problem.PlotGUI;

%EoF
end
