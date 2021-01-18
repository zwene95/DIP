function [Problem] = buildOCP(Setup)
%% Initialize DefenderInvaderProblem

% Model variables
Variables = buildDataTypes(Setup.ModelOptions);

% Final time initialization
tf = falcon.Parameter('FinalTime', 5, 0, 20, 1e-0);                     % 1e-1

% Build model if not yet built
model = functions(str2func(Setup.ModelName));
if isempty(model.file) || Setup.forceBuild
    disp('INFO: Building model');
    buildModel(Setup.ModelOptions,Setup.ModelName);
end
disp('INFO: Loading model');
model_fh = str2func(Setup.ModelName);

%% Create Problem
if Setup.ModelOptions.Uncertainty
    Problem = falcon.gPCColloc.gPCProblem(Setup.PostOptions.ScenarioName);
else
    Problem = falcon.Problem(Setup.PostOptions.ScenarioName, 'UseHessian', false);
end

% Discretization
if Setup.Solver.BackwarEuler
    Problem.setDiscretizationMethod(falcon.discretization.BackwardEuler);
end
Tau = linspace(0,1,( Setup.Solver.GridSize + 1) );

% Add new phase
Phase = Problem.addNewPhase(model_fh, Variables.States, Tau, 0, tf);

% Add control grid
Phase.addNewControlGrid(Variables.Controls,Tau);

% Add outputs
Phase.Model.setModelOutputs(Variables.Outputs);

% Add model constants
Phase.Model.addModelConstants(Setup.ObserverConfig.Spr);


% Set initial and final boundary conditions
Phase.setInitialBoundaries(Setup.Scenario.InitBoundaries);
Phase.setFinalBoundaries(...
    Setup.Scenario.FinalBoundaries_lw,...
    Setup.Scenario.FinalBoundaries_up);

%% Set Model Parameters
switch Setup.ModelOptions.Optimize
    case 'def'
        if Setup.ModelOptions.Uncertainty
            ModelParameters = [
                falcon.DistrParameter('vIOO_x'  , Setup.Scenario.vIOO(1)                    , 'Fixed', true, 'Uncertain', false)
                falcon.DistrParameter('vIOO_y'  , Setup.Scenario.vIOO(2)                    , 'Fixed', true, 'Uncertain', false)
                falcon.DistrParameter('vIOO_z'  , Setup.Scenario.vIOO(3), -20, 20, 1e-2     , 'Fixed', true, 'DistType','Gaussian','DistVals',[Setup.Scenario.vIOO(3),1,0,0]);
                falcon.DistrParameter('T2W_max' , Setup.DefenderConfig.T2W_max              , 'Fixed', true, 'Uncertain', false)        % max thrust to weight ratio
                falcon.DistrParameter('rEscape' , Setup.InvaderConfig.rEscape               , 'Fixed', true, 'Uncertain', false)        % parameter for invader escape maneuver
                ];
        else
            ModelParameters = [
                falcon.Parameter('pTOO_x'    , Setup.Scenario.pTOO(1)           , 'Fixed', true)
                falcon.Parameter('pTOO_y'    , Setup.Scenario.pTOO(2)           , 'Fixed', true)
                falcon.Parameter('pTOO_z'    , Setup.Scenario.pTOO(3)           , 'Fixed', true)
                falcon.Parameter('vIOO_x'    , Setup.Scenario.vIOO(1)           , 'Fixed', true)
                falcon.Parameter('vIOO_y'    , Setup.Scenario.vIOO(2)           , 'Fixed', true)
                falcon.Parameter('vIOO_z'    , Setup.Scenario.vIOO(3)           , 'Fixed', true)
                falcon.Parameter('vI_abs_max', Setup.InvaderConfig.vI_abs_max   , 'Fixed', true)
                falcon.Parameter('T2W_max'   , Setup.DefenderConfig.T2W_max     , 'Fixed', true)
                falcon.Parameter('MotorTC'   , Setup.DefenderConfig.MotorTC     , 'Fixed', true)
                falcon.Parameter('rEscape'   , Setup.InvaderConfig.rEscape      , 'Fixed', true)
                ];
        end
        
    case 'inv'
        ModelParameters = [
            falcon.Parameter('vI_abs_max'   , Setup.InvaderConfig.vI_abs_max   , 'Fixed', true)
            falcon.Parameter('T2W_max_inv'  , Setup.InvaderConfig.T2W_max      , 'Fixed', true)
            ];
end

Parameters = [
    ModelParameters
    ];

% Set model parameters
Phase.Model.setModelParameters(Parameters);

%% Constraints

% Hitconstraint
if Setup.CCConfig.Constraint.Hit
    HitConstraint = falcon.Constraint('HitCon',...
        -inf, 0.09, Setup.CCConfig.Scaling.Hit);
    Phase.addNewPathConstraint(@hitConFcn, HitConstraint, 1);
end

% Seeker and thrust constraints
if Setup.ModelOptions.Defender.SixDoF
    
    if Setup.CCConfig.Constraint.FoV
        % Add defender seeker FOV constraint
        El_max_half = Setup.DefenderConfig.FoV(1) / 2 * pi/180;
        Az_max_half = Setup.DefenderConfig.FoV(2) / 2 * pi/180;
        FovConstraint = [
            falcon.Constraint('ElevationConstraint' , -El_max_half  , El_max_half)
            falcon.Constraint('AzimuthConstraint'   , -Az_max_half  , Az_max_half)
            ];
        Phase.addNewPathConstraint(@fovConstraintFcn, FovConstraint ,Tau);
    end
else
    % Add thrust constraint
    if Setup.CCConfig.Constraint.Thrust
        ThrustConstraint = falcon.Constraint('ThrustConstraint', 0, 1, 1e-0);
        Phase.addNewPathConstraint(@defThrustConFcn, ThrustConstraint ,Tau);
        error('Thrust Constraing sqrt(3) berücksichtigen!!!');
    end
end

% Velocity Cons% velocityConstraint = falcon.Constraint('defVelocityConstraint', 0, 25^2, 1e-2);
% phase.addNewPathConstraint(@defVelConFcn, velocityConstraint ,tau);


%% Costs
% Observability cost
% Create EKF object to retrieve initial state estimate and covariance
myEKF = EKF_Object;
myEKF.ObserverConfig = Setup.ObserverConfig;
% Extract true initial state vector
x0_true = [...
    Setup.InvaderConfig.pIOO_0 - Setup.DefenderConfig.pDOO_0
    Setup.InvaderConfig.vIOO_0 - Setup.DefenderConfig.vDOO_0];
[x0,P0] = myEKF.initialEstimate(x0_true);
if Setup.ModelOptions.ObservabilityCostFcn
    myObj = CostObject;
    myObj.Problem   = Problem;
    myObj.Setup     = Setup;
    myObj.x0        = x0;
    myObj.P0        = P0;
    myObj.Qw        = eye(3) * Setup.ObserverConfig.Std_Qw;
    myObj.Rv        = eye(2) * Setup.ObserverConfig.Std_Rv;
    pcon =  Problem.addNewMayerCost(...
        @myObj.ObservabilityCostFcn,...
        falcon.Cost('Observability'),...
        Phase, Tau);
    pcon.setParameters([Phase.StartTime; Phase.FinalTime]);
end

% Target violation cost
if Setup.CCConfig.Cost.TargetViolation
    TargetVioCostObj = Phase.addNewLagrangeCost(...
        @targetViolationCostFcn,...
        falcon.Cost('TargetVioCost',...
        Setup.CCConfig.Scaling.TargetViolation), Tau);                                % 1e+0, 1e+5, 1e+7,
    TargetVioCostObj.setParameters(...
        falcon.Parameter('CaptureRadius',...
        Setup.TargetConfig.rT_max, 'fixed', true));
end


if Setup.CCConfig.Cost.Missdistance
    % Missdistance cost
    Problem.addNewMayerCost(...
        @missDistanceCostFcn,...
        falcon.Cost('MissDistance', Setup.CCConfig.Scaling.Missdistance),...   % 1e-2/1e-4/1e-1
        Phase, 1);
    %             missDistanceCostObj.setParameters(...
    %                                 falcon.Parameter('maxMissDistance',...
    %                                 setup.maxMissDistance, 'fixed', true));
end

% Time cost function
%     problem.addNewParameterCost(tf, 'min', 'Scaling', 1e-0);   % 1e-1
if Setup.CCConfig.Cost.Time
    Problem.addNewParameterCost(tf, 'min',...
        'Scaling', Setup.CCConfig.Scaling.Time);
end

% Set overall cost scaling
Problem.setCostScaling(Setup.CCConfig.Scaling.Overall);


%% gpC Collocation
% if setup.ModelOptions.uncertainty
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


