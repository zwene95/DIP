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
    Setup.PostOptions               = defaultPostOptions('OG_vI20_Obs_Stat4');
    Setup.CCConfig                  = defaultCCConfig();             
    Setup.Solver                    = defaultSolverConfig();
    
    % Model options
    Setup.ModelOptions.Defender.SixDoF      = 1; 
    Setup.ModelOptions.Defender.MotorLag    = 0;
    Setup.ModelOptions.Defender.Aero        = 1;
    Setup.ModelOptions.ObservabilityCostFcn = 1;
    % Target configuration    
    % Invader configuration
    Setup.InvaderConfig.vI_abs_max          = 0;
    % Cost configuration
    Setup.CCConfig.Cost.Missdistance        = 1;
    Setup.CCConfig.Cost.Time                = 1;
    Setup.CCConfig.Cost.TargetViolation     = 0;
    % Constraint configuration
    Setup.CCConfig.Constraint.FoV           = 1;
    % Scaling configuration
    Setup.CCConfig.Scaling.Overall          = 1;
    Setup.CCConfig.Scaling.Missdistance     = 2e-2;                         %1e-1% 50e-04
    Setup.CCConfig.Scaling.Time             = 1e-0;                         %1e00%55e-02
    Setup.CCConfig.Scaling.TargetViolation  = 1e0; 
    Setup.CCConfig.Scaling.ObserverCov      = 10e-6;                        % 1e-6/50e-06
    Setup.CCConfig.Scaling.ObserverRMSE     = 10e-2;                        % 1e-2
    Setup.CCConfig.Scaling.TargetViolation  = 1e-4;                         % 10e+01        
    % Solver configuration
    Setup.Solver.GridSize                   = 200;                           % 200
    Setup.Solver.MaxIter                    = 1000;                          % 500
    Setup.Solver.BackwarEuler               = 0;
    Setup.Solver.Parallel                   = 1;
    % Post options
    Setup.PostOptions.Save                  = 1;%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
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
