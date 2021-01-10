function [ setup, problem ] = DIP_init(setup)
%% Initialize DefenderInvaderProblem

% Setup scenario if not provided
setup.scenario  = initScenario(setup);

% Model variables
variables = createDataTypes(setup.modelOptions);

% Final time initialization
tf = falcon.Parameter('FinalTime', 5, 0, 15, 1e-0);                     % 1e-1

% Build model if not yet built
model = functions(str2func(setup.modelName));
if isempty(model.file) || setup.forceBuild
    disp('INFO: Building model');
    buildModel(setup.modelOptions,setup.modelName);
end
disp('INFO: Loading model');
model_fh = str2func(setup.modelName);

%% Create Problem
if setup.modelOptions.uncertainty
    problem = falcon.gPCColloc.gPCProblem(setup.postOptions.ScenarioName);
else
    problem = falcon.Problem(setup.postOptions.ScenarioName, 'UseHessian', false);
end

% Discretization
if setup.Solver.BackwarEuler 
    problem.setDiscretizationMethod(falcon.discretization.BackwardEuler);
end
tau = linspace(0,1,( setup.Solver.GridSize + 1) );

% Add new phase
phase = problem.addNewPhase(model_fh, variables.states, tau, 0, tf);

% Add control grid
phase.addNewControlGrid(variables.controls,tau);

% Add outputs
phase.Model.setModelOutputs(variables.outputs);

% Add model constants
phase.Model.addModelConstants(setup.observerConfig.spr);


% Set initial and final boundary conditions
    phase.setInitialBoundaries(setup.scenario.initBoundaries);
    phase.setFinalBoundaries(...
        setup.scenario.finalBoundaries_lw,...
        setup.scenario.finalBoundaries_up);

%% Set Model Parameters
switch setup.modelOptions.optimize
    case 'def'
        if setup.modelOptions.uncertainty
            modelParameters = [
                falcon.DistrParameter('vIOO_x'  , setup.scenario.vIOO(1)                    , 'Fixed', true, 'Uncertain', false)
                falcon.DistrParameter('vIOO_y'  , setup.scenario.vIOO(2)                    , 'Fixed', true, 'Uncertain', false)
                falcon.DistrParameter('vIOO_z'  , setup.scenario.vIOO(3), -20, 20, 1e-2     , 'Fixed', true, 'DistType','Gaussian','DistVals',[setup.scenario.vIOO(3),1,0,0]);
                falcon.DistrParameter('T2W_max' , setup.defenderConfig.T2W_max              , 'Fixed', true, 'Uncertain', false)        % max thrust to weight ratio
                falcon.DistrParameter('rEscape' , setup.invaderConfig.rEscape               , 'Fixed', true, 'Uncertain', false)        % parameter for invader escape maneuver
                ];
        else
            modelParameters = [
                falcon.Parameter('pTOO_x'    , setup.scenario.pTOO(1)           , 'Fixed', true)
                falcon.Parameter('pTOO_y'    , setup.scenario.pTOO(2)           , 'Fixed', true)
                falcon.Parameter('pTOO_z'    , setup.scenario.pTOO(3)           , 'Fixed', true)
                falcon.Parameter('vIOO_x'    , setup.scenario.vIOO(1)           , 'Fixed', true)
                falcon.Parameter('vIOO_y'    , setup.scenario.vIOO(2)           , 'Fixed', true)
                falcon.Parameter('vIOO_z'    , setup.scenario.vIOO(3)           , 'Fixed', true)
                falcon.Parameter('vI_abs_max', setup.invaderConfig.vI_abs_max   , 'Fixed', true)
                falcon.Parameter('T2W_max'   , setup.defenderConfig.T2W_max     , 'Fixed', true)
                falcon.Parameter('MotorTC'   , setup.defenderConfig.MotorTC     , 'Fixed', true)
                falcon.Parameter('rEscape'   , setup.invaderConfig.rEscape      , 'Fixed', true)
                ];
            
        end
        
    case 'inv'
        modelParameters = [
            falcon.Parameter('vI_abs_max'   , setup.invaderConfig.vI_abs_max   , 'Fixed', true)
            falcon.Parameter('T2W_max_inv'  , setup.invaderConfig.T2W_max      , 'Fixed', true)
            ];
end

parameters = [
    modelParameters
    ];

% Set model parameters
phase.Model.setModelParameters(parameters);

%% Constraints

% Hitconstraint
if setup.CCConfig.Hit.Constraint
    hitConstraint = falcon.Constraint('hitCon',...
        -inf, 0.09, setup.CCConfig.Hit.Scaling);
    phase.addNewPathConstraint(@hitConFcn, hitConstraint, 1);
end

% Seeker and thrust constraints
if setup.modelOptions.defender.SixDoF
    
    if setup.CCConfig.FoV.Constraint
        % Add defender seeker FOV constraint
        el_max_half = setup.defenderConfig.FoV(1) / 2 * pi/180;
        az_max_half = setup.defenderConfig.FoV(2) / 2 * pi/180;
        fovConstraint = [
            falcon.Constraint('elevationConstraint' , -el_max_half  , el_max_half)
            falcon.Constraint('azimuthConstraint'   , -az_max_half  , az_max_half)
            ];
        phase.addNewPathConstraint(@fovConstraintFcn, fovConstraint ,tau);
    end
else
    % Add thrust constraint
    if setup.CCConfig.Thrust.Constraint
        thrustConstraint = falcon.Constraint('defThrustConstraint', 0, 1, 1e-0);
        error('Thrust Constraing sqrt(3) berücksichtigen!!!');
        phase.addNewPathConstraint(@defThrustConFcn, thrustConstraint ,tau);
    end
end



% Velocity Cons% velocityConstraint = falcon.Constraint('defVelocityConstraint', 0, 25^2, 1e-2);
% phase.addNewPathConstraint(@defVelConFcn, velocityConstraint ,tau);


%% Costs
% Observability cost
if setup.modelOptions.observabilityCostFcn
    myObj = CostObject;
    myObj.Problem = problem;
    myObj.Setup   = setup;
    pcon =  problem.addNewMayerCost(...
        @myObj.ObservabilityCostFcn,...
        falcon.Cost('observability'),...
        phase, tau);
    pcon.setParameters([phase.StartTime; phase.FinalTime]);
end

% Target violation cost
if setup.CCConfig.TargetViolation.Cost
    targetVioCostObj = phase.addNewLagrangeCost(...
        @targetViolationCostFcn,...
        falcon.Cost('targetVioCost',...
        setup.CCConfig.TargetViolation.Scaling), tau);                                % 1e+0, 1e+5, 1e+7,
    targetVioCostObj.setParameters(...
        falcon.Parameter('captureRadius',...
        setup.targetConfig.rT_max, 'fixed', true));
end


if setup.CCConfig.Missdistance.Cost
    % Missdistance cost
    problem.addNewMayerCost(...
        @missDistanceCostFcn,...
        falcon.Cost('MissDistance', setup.CCConfig.Missdistance.Scaling),...   % 1e-2/1e-4/1e-1
        phase, 1);
    %             missDistanceCostObj.setParameters(...
    %                                 falcon.Parameter('maxMissDistance',...
    %                                 setup.maxMissDistance, 'fixed', true));
end




% Time cost function
%     problem.addNewParameterCost(tf, 'min', 'Scaling', 1e-0);   % 1e-1
if setup.CCConfig.Time.Cost
    problem.addNewParameterCost(tf, 'min',...
        'Scaling', setup.CCConfig.Time.Scaling);
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


