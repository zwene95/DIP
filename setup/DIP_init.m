function [Setup, Problem ] = DIP_init(Setup)
%% Initialize DefenderInvaderProblem

% Setup scenario if not provided
Setup.Scenario  = initScenario(Setup);

% Model variables
variables = createDataTypes(Setup.modelOptions);

% Final time initialization
tf = falcon.Parameter('FinalTime', 5, 0, 15, 1e-0);                     % 1e-1

% Build model if not yet built
model = functions(str2func(Setup.modelName));
if isempty(model.file) || Setup.forceBuild
    disp('INFO: Building model');
    buildModel(Setup.modelOptions,Setup.modelName);
end
disp('INFO: Loading model');
model_fh = str2func(Setup.modelName);

%% Create Problem
if Setup.modelOptions.uncertainty
    Problem = falcon.gPCColloc.gPCProblem(Setup.postOptions.ScenarioName);
else
    Problem = falcon.Problem(Setup.postOptions.ScenarioName, 'UseHessian', false);
end

% Discretization
if Setup.Solver.BackwarEuler 
    Problem.setDiscretizationMethod(falcon.discretization.BackwardEuler);
end
tau = linspace(0,1,( Setup.Solver.GridSize + 1) );

% Add new phase
phase = Problem.addNewPhase(model_fh, variables.states, tau, 0, tf);

% Add control grid
phase.addNewControlGrid(variables.controls,tau);

% Add outputs
phase.Model.setModelOutputs(variables.outputs);

% Add model constants
phase.Model.addModelConstants(Setup.observerConfig.spr);


% Set initial and final boundary conditions
    phase.setInitialBoundaries(Setup.scenario.initBoundaries);
    phase.setFinalBoundaries(...
        Setup.scenario.finalBoundaries_lw,...
        Setup.scenario.finalBoundaries_up);

%% Set Model Parameters
switch Setup.modelOptions.optimize
    case 'def'
        if Setup.modelOptions.uncertainty
            modelParameters = [
                falcon.DistrParameter('vIOO_x'  , Setup.scenario.vIOO(1)                    , 'Fixed', true, 'Uncertain', false)
                falcon.DistrParameter('vIOO_y'  , Setup.scenario.vIOO(2)                    , 'Fixed', true, 'Uncertain', false)
                falcon.DistrParameter('vIOO_z'  , Setup.scenario.vIOO(3), -20, 20, 1e-2     , 'Fixed', true, 'DistType','Gaussian','DistVals',[Setup.scenario.vIOO(3),1,0,0]);
                falcon.DistrParameter('T2W_max' , Setup.defenderConfig.T2W_max              , 'Fixed', true, 'Uncertain', false)        % max thrust to weight ratio
                falcon.DistrParameter('rEscape' , Setup.invaderConfig.rEscape               , 'Fixed', true, 'Uncertain', false)        % parameter for invader escape maneuver
                ];
        else
            modelParameters = [
                falcon.Parameter('pTOO_x'    , Setup.scenario.pTOO(1)           , 'Fixed', true)
                falcon.Parameter('pTOO_y'    , Setup.scenario.pTOO(2)           , 'Fixed', true)
                falcon.Parameter('pTOO_z'    , Setup.scenario.pTOO(3)           , 'Fixed', true)
                falcon.Parameter('vIOO_x'    , Setup.scenario.vIOO(1)           , 'Fixed', true)
                falcon.Parameter('vIOO_y'    , Setup.scenario.vIOO(2)           , 'Fixed', true)
                falcon.Parameter('vIOO_z'    , Setup.scenario.vIOO(3)           , 'Fixed', true)
                falcon.Parameter('vI_abs_max', Setup.invaderConfig.vI_abs_max   , 'Fixed', true)
                falcon.Parameter('T2W_max'   , Setup.defenderConfig.T2W_max     , 'Fixed', true)
                falcon.Parameter('MotorTC'   , Setup.defenderConfig.MotorTC     , 'Fixed', true)
                falcon.Parameter('rEscape'   , Setup.invaderConfig.rEscape      , 'Fixed', true)
                ];
            
        end
        
    case 'inv'
        modelParameters = [
            falcon.Parameter('vI_abs_max'   , Setup.invaderConfig.vI_abs_max   , 'Fixed', true)
            falcon.Parameter('T2W_max_inv'  , Setup.invaderConfig.T2W_max      , 'Fixed', true)
            ];
end

parameters = [
    modelParameters
    ];

% Set model parameters
phase.Model.setModelParameters(parameters);

%% Constraints

% Hitconstraint
if Setup.CCConfig.Hit.Constraint
    hitConstraint = falcon.Constraint('hitCon',...
        -inf, 0.09, Setup.CCConfig.Hit.Scaling);
    phase.addNewPathConstraint(@hitConFcn, hitConstraint, 1);
end

% Seeker and thrust constraints
if Setup.modelOptions.defender.SixDoF
    
    if Setup.CCConfig.FoV.Constraint
        % Add defender seeker FOV constraint
        el_max_half = Setup.defenderConfig.FoV(1) / 2 * pi/180;
        az_max_half = Setup.defenderConfig.FoV(2) / 2 * pi/180;
        fovConstraint = [
            falcon.Constraint('elevationConstraint' , -el_max_half  , el_max_half)
            falcon.Constraint('azimuthConstraint'   , -az_max_half  , az_max_half)
            ];
        phase.addNewPathConstraint(@fovConstraintFcn, fovConstraint ,tau);
    end
else
    % Add thrust constraint
    if Setup.CCConfig.Thrust.Constraint
        thrustConstraint = falcon.Constraint('defThrustConstraint', 0, 1, 1e-0);
        error('Thrust Constraing sqrt(3) berücksichtigen!!!');
        phase.addNewPathConstraint(@defThrustConFcn, thrustConstraint ,tau);
    end
end

% Velocity Cons% velocityConstraint = falcon.Constraint('defVelocityConstraint', 0, 25^2, 1e-2);
% phase.addNewPathConstraint(@defVelConFcn, velocityConstraint ,tau);


%% Costs
% Observability cost
if Setup.modelOptions.observabilityCostFcn
    myObj = CostObject;
    myObj.Problem = Problem;
    myObj.Setup   = Setup;
    pcon =  Problem.addNewMayerCost(...
        @myObj.ObservabilityCostFcn,...
        falcon.Cost('observability'),...
        phase, tau);
    pcon.setParameters([phase.StartTime; phase.FinalTime]);
end

% Target violation cost
if Setup.CCConfig.TargetViolation.Cost
    targetVioCostObj = phase.addNewLagrangeCost(...
        @targetViolationCostFcn,...
        falcon.Cost('targetVioCost',...
        Setup.CCConfig.TargetViolation.Scaling), tau);                                % 1e+0, 1e+5, 1e+7,
    targetVioCostObj.setParameters(...
        falcon.Parameter('captureRadius',...
        Setup.targetConfig.rT_max, 'fixed', true));
end


if Setup.CCConfig.Missdistance.Cost
    % Missdistance cost
    Problem.addNewMayerCost(...
        @missDistanceCostFcn,...
        falcon.Cost('MissDistance', Setup.CCConfig.Missdistance.Scaling),...   % 1e-2/1e-4/1e-1
        phase, 1);
    %             missDistanceCostObj.setParameters(...
    %                                 falcon.Parameter('maxMissDistance',...
    %                                 setup.maxMissDistance, 'fixed', true));
end




% Time cost function
%     problem.addNewParameterCost(tf, 'min', 'Scaling', 1e-0);   % 1e-1
if Setup.CCConfig.Time.Cost
    Problem.addNewParameterCost(tf, 'min',...
        'Scaling', Setup.CCConfig.Time.Scaling);
end


%% gpC Collocation
% if setup.modelOptions.uncertainty
%     problem.setExpGridType('tensor')
%
%
%     % Number of nodes
%     problem.setExpNodes(2);
%
%     % States setting
%     states_mat      = zeros(10,4,1);
%     states_mat(:,1) = -10;
%     states_mat(:,2) = 10;
%     states_mat(:,3) = 1e0;
%     states_mat(:,4) = 0;
%     problem.setExpStates_LBUBOffScal(states_mat);
%
%     % Controls setting
%     ctrls_mat      = zeros(10,4,1);
%     ctrls_mat(:,1) = -10;
%     ctrls_mat(:,2) = 10;
%     ctrls_mat(:,3) = 1e0;
%     ctrls_mat(:,4) = 0;
%     problem.setExpCtrls_LBUBOffScal(ctrls_mat);
%
%     % Parameters setting
%     param_mat      = zeros(10,4,1);
%     param_mat(:,1) = -10;
%     param_mat(:,2) = 10;
%     param_mat(:,3) = 1e0;
%     param_mat(:,4) = 0;
%     problem.setExpParams_LBUBOffScal(param_mat);
%
%     % Initial guess
%     problem.Phases(1,1).setExpStatesInitGuess(zeros(10,length(tau),length(variables.states)));
%     problem.Phases(1,1).setExpCtrlsInitGuess(zeros(10,length(tau),length(variables.controls)));
%     problem.setExpParamsInitGuess(zeros(10,1,length(parameters)));
%
% end


% EoF
end


