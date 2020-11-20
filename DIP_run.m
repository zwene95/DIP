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
    setup.postOptions               = defaultPostOptions('AMDC_Praesi');      % TracePos_errPosVec_Time_6DoF
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
    setup.invaderConfig.vI_abs_max          = 10;                            % 20    
    % Target
    setup.targetConfig.targetConstraint     = 0;
    % Observer
%     setup.observerConfig.std_pos            = 0;
%     setup.observerConfig.std_vel            = 0;
%     setup.observerConfig.spr                = 1e-2;                         % 1e-?
    % Solver options
    setup.Solver.GridSize                   = 50;                           % 200
    setup.Solver.MaxIter                    = 150;                           % 500
    setup.Solver.Parallel                   = 1;
    setup.Solver.GPU                        = 0;        
    setup.Solver.CostScalingTime            = 7.5e-1;                         % 5e-1
    setup.Solver.CostScalingMiss            = 5e-3;                         % 1e-2/5e-2
    setup.Solver.CostScalingCov             = 5e-5;                         % 5e-5/
    setup.Solver.CostScalingRMSE            = 0e-4;                         % 1e-4
    
    
    %% PN FÜR AMDC
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    setup.modelOptions.observabilityCostFcn = 0;
    setup.Solver.CostScalingTime = 1;%0.5000;
    setup.Solver.CostScalingMiss = 1;%0.0050;
    setup.Solver.GridSize = 200;
    setup.defenderConfig.pDOO_0 = [0;0;0];
    setup.invaderConfig.pIOO_0 = [250; 0; -50]/2;
    setup.invaderConfig.vIOO_0 = [0 -1 0]/sqrt(1)*15;
    setup.targetConfig.pTOO = [250;-1000;-50]/2;
    setup.invaderConfig.vI_abs_max = 15;
    setup.targetConfig.Random = 0;

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    
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
