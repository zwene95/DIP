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
    setup.postOptions               = defaultPostOptions('TracePos_errPosVec_Time_6DoF');      % Test3_TracePos_errPosVec
    setup.Solver                    = defaultSolverConfig();
    
    % Modify default modeloptions
    setup.modelOptions.defender.MotorLag    = 1;
    setup.modelOptions.defender.Aero        = 1;
    setup.modelOptions.defender.SixDoF      = 1;
    setup.modelOptions.observer             = 0;
    setup.modelOptions.observabilityCostFcn = 1;
    
    % Modify default postOptions
    setup.postOptions.Save                  = 1;
    
    % Modify default config
    % Defender
    setup.defenderConfig.FovConstraint      = 1;
    setup.defenderConfig.HitConstraint      = 0;
    setup.defenderConfig.ThrustConstraint   = 0;
    % Invader
    setup.invaderConfig.vI_abs_max          = 0;                            % 20
    % Target
    setup.targetConfig.targetConstraint     = 0;
    % Observer
%     setup.observerConfig.std_pos            = 0;
%     setup.observerConfig.std_vel            = 0;
%     setup.observerConfig.spr                = 1e-2;                       % 1e-?
    % Solver options
    setup.Solver.GridSize                   = 50;                           % 200
    setup.Solver.MaxIter                    = 50;                           % 500
    setup.Solver.Parallel                   = 1;
    setup.Solver.GPU                        = 0;        
    setup.Solver.CostWeightTime             = 1e-2;%1;%5e-1;                      % 10e-1/ 5e-1
    setup.Solver.CostWeightMiss             = 1;%100;%5e-3;                    % 5e-3/1e-2/5e-2
    setup.Solver.CostWeightCov              = 1e-0;                         % 5e-5/1e-4/
    setup.Solver.CostWeightRMSE             = 1e-0;                         % 0e-0/1e-6        
    
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

%EoF
end
