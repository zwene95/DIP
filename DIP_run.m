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
    setup.postOptions               = defaultPostOptions('Test3_TracePos_NEES');
    setup.Solver                    = defaultSolverConfig();
    
    % Modify default modeloptions
    setup.modelOptions.defender.MotorLag    = 0;
    setup.modelOptions.defender.Aero        = 0;
    setup.modelOptions.defender.SixDoF      = 0;
    setup.modelOptions.observer             = 0;
    setup.modelOptions.observabilityCostFcn = 1;
    
    % Modify default postOptions
    setup.postOptions.Save                  = 1;
    
    % Modify default config
    % Defender
    setup.defenderConfig.FovConstraint      = 0;
    setup.defenderConfig.HitConstraint      = 0;
    setup.defenderConfig.ThrustConstraint   = 0;
    % Invader
    setup.invaderConfig.vI_abs_max          = 0;                            % 20    
    % Target
    setup.targetConfig.targetConstraint     = 0;
    % Observer
    setup.observerConfig.std_pos            = 0;
    setup.observerConfig.std_vel            = 0;
    setup.observerConfig.spr                = 1e-2;                         % 1e-?
    % Solver options
    setup.Solver.gridSize                   = 50;                           % 200
    setup.Solver.maxIter                    = 50;                           % 500
    setup.Solver.CostScaling                = 1e-0;                         % 1e-0
    setup.Solver.TimeCostScaling            = 0e-0;                         % 1e-0
    setup.Solver.ObsCostScaling             = 1e-4;                         % 1e-4/1e-5    
    setup.Solver.Parallel                   = 1;
    setup.Solver.GPU                        = 0;
    
    % Check modelOptions for contradictions and revise modelOptions
    setup = checkModelOptions(setup);
    
    % Retrieve modelname
    setup.modelName = getModelName(setup.modelOptions);
    
end

% Initialize problem
[setup , problem] = DIP_init(setup);
% Prepare for solving
problem.Bake();
problem.setMajorIterLimit(setup.Solver.maxIter);
% Solve problem
problem.Solve();
% Post process
DIP_post(setup, problem);

if ~setup.postOptions.Save
    warning('Results NOT saved, Save-Flag was not set!');
end

%EoF
end
