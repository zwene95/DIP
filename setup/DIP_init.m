function [ setup, problem ] = DIP_init(setup)
%% Initialize DefenderInvaderProblem
    
    % Setup scenario if not provided    
    setup.scenario  = initScenario(setup);    
    setup.ekf       = initEKF(setup);
    
    % Model variables
    variables = createDataTypes(setup.modelOptions);
    
    % Final time initialization
    tf = falcon.Parameter('FinalTime', 15, 0, 20, 1e-0);                     % 1e-1
    
    % Build model if not yet built
    model = functions(str2func(setup.modelOptions.modelName));
    if isempty(model.file) || setup.forceBuild
        disp('INFO: Building model');
        buildModel(setup.modelOptions);
    end    
    disp('INFO: Loading model');
    model_fh = str2func(setup.modelOptions.modelName);
        
    %% Create Problem
    if setup.modelOptions.uncertainty
        problem = falcon.gPCColloc.gPCProblem(setup.postOptions.ScenarioName);    
    else
        problem = falcon.Problem(setup.postOptions.ScenarioName, 'UseHessian', false);    
    end
    
    % Discretization    
%     problem.setDiscretizationMethod(falcon.discretization.BackwardEuler);
    tau = linspace(0,1,( setup.gridSize + 1) );
    
    % Add new phase
    phase = problem.addNewPhase(model_fh, variables.states, tau, 0, tf);
    
    % Add control grid
    phase.addNewControlGrid(variables.controls,tau);
        
    % Add outputs
    phase.Model.setModelOutputs(variables.outputs);
    
    % Set initial and final boundary conditions
    if setup.modelOptions.observer        
        phase.setInitialBoundaries([setup.scenario.initBoundaries
                                    setup.ekf.initBoundaries]);
        phase.setFinalBoundaries(  [setup.scenario.finalBoundaries_lw
                                    setup.ekf.finalBoundaries_lw],...
                                   [setup.scenario.finalBoundaries_up
                                    setup.ekf.finalBoundaries_up]);
    else
        phase.setInitialBoundaries( setup.scenario.initBoundaries);
        phase.setFinalBoundaries(   setup.scenario.finalBoundaries_lw,...
                                    setup.scenario.finalBoundaries_up);
    end
    
    % Add terminal point constraint
%     HitConstraints = [  falcon.Constraint('hitCon_x', -0.05, 0.05, 1e+2)
%                         falcon.Constraint('hitCon_y', -0.05, 0.05, 1e+2)
%                         falcon.Constraint('hitCon_z', -0.05, 0.05, 1e+2) ];
%     phase.addNewPathConstraint(@hitConstraintFunction, HitConstraints, 1);
    
    %% Set model parameters
    switch setup.modelOptions.optimize
        case 'def'            
            if setup.modelOptions.uncertainty
                modelParameters = [
                    falcon.DistrParameter('vIOO_x'  , setup.scenario.vIOO(1)                    , 'Fixed', true, 'Uncertain', false)
                    falcon.DistrParameter('vIOO_y'  , setup.scenario.vIOO(2)                    , 'Fixed', true, 'Uncertain', false)
                    falcon.DistrParameter('vIOO_z'  , setup.scenario.vIOO(3), -20, 20, 1e-2     , 'Fixed', true, 'DistType','Gaussian','DistVals',[setup.scenario.vIOO(3),1,0,0]);
                    falcon.DistrParameter('T2W_max' , setup.defenderConfig.T2W_max             , 'Fixed', true, 'Uncertain', false)        % max thrust to weight ratio
                    falcon.DistrParameter('rEscape' , setup.invaderConfig.rEscape              , 'Fixed', true, 'Uncertain', false)        % parameter for invader escape maneuver
                ];
            else
                modelParameters = [
                    falcon.Parameter('pTOO_x'    , setup.scenario.pTOO(1)           , 'Fixed', true)   
                    falcon.Parameter('pTOO_y'    , setup.scenario.pTOO(2)           , 'Fixed', true)
                    falcon.Parameter('pTOO_z'    , setup.scenario.pTOO(3)           , 'Fixed', true)
                    falcon.Parameter('vIOO_x'    , setup.scenario.vIOO(1)           , 'Fixed', true)
                    falcon.Parameter('vIOO_y'    , setup.scenario.vIOO(2)           , 'Fixed', true)
                    falcon.Parameter('vIOO_z'    , setup.scenario.vIOO(3)           , 'Fixed', true)
                    falcon.Parameter('vI_abs_max', setup.invaderConfig.vI_abs_max  , 'Fixed', true)
                    falcon.Parameter('T2W_max'   , setup.defenderConfig.T2W_max    , 'Fixed', true)
                    falcon.Parameter('MotorTC'   , setup.defenderConfig.MotorTC    , 'Fixed', true)                    
                    falcon.Parameter('rEscape'   , setup.invaderConfig.rEscape     , 'Fixed', true)
                ];
                
            end
            
        case 'inv'            
            modelParameters = [
                falcon.Parameter('vI_abs_max'   , setup.invaderConfig.vI_abs_max   , 'Fixed', true)
                falcon.Parameter('T2W_max_inv'  , setup.invaderConfig.T2W_max      , 'Fixed', true)
            ];
    end
    
    if setup.modelOptions.observer
        observerParameters = [                   
            falcon.Parameter('Q_11'  , setup.observerConfig.Q(1,1) , 'fixed', true, 'Scaling', 1e-2)
            falcon.Parameter('Q_12'  , setup.observerConfig.Q(1,2) , 'fixed', true)
            falcon.Parameter('Q_13'  , setup.observerConfig.Q(1,3) , 'fixed', true)
            falcon.Parameter('Q_21'  , setup.observerConfig.Q(2,1) , 'fixed', true)
            falcon.Parameter('Q_22'  , setup.observerConfig.Q(2,2) , 'fixed', true, 'Scaling', 1e-2)
            falcon.Parameter('Q_23'  , setup.observerConfig.Q(2,3) , 'fixed', true)
            falcon.Parameter('Q_31'  , setup.observerConfig.Q(3,1) , 'fixed', true)
            falcon.Parameter('Q_32'  , setup.observerConfig.Q(3,2) , 'fixed', true)
            falcon.Parameter('Q_33'  , setup.observerConfig.Q(3,3) , 'fixed', true, 'Scaling', 1e-2)            
            falcon.Parameter('R_11'  , setup.observerConfig.R(1,1) , 'fixed', true, 'Scaling', 1e+2)  
            falcon.Parameter('R_12'  , setup.observerConfig.R(1,2) , 'fixed', true)  
            falcon.Parameter('R_21'  , setup.observerConfig.R(2,1) , 'fixed', true)  
            falcon.Parameter('R_22'  , setup.observerConfig.R(2,2) , 'fixed', true, 'Scaling', 1e+2)  
        ];        
    
    else
        observerParameters = falcon.Parameter.empty();
    end
    
    parameters = [
        modelParameters
        observerParameters
    ];

    % Set model parameters
    phase.Model.setModelParameters(parameters);

    %% Constraints and costs
    
    switch setup.modelOptions.optimize
        case 'def'
            
            % Missdistance - Mayer cost           
            missDistanceCostObj = problem.addNewMayerCost(...
                                    @missDistanceCostFcn,...
                                    falcon.Cost('missDistance', 1e-3),...   % 1e-4/1e-2
                                    phase, 1);  
%             missDistanceCostObj.setParameters(...
%                                 falcon.Parameter('maxMissDistance',...
%                                 setup.maxMissDistance, 'fixed', true));
            
            % Target violation cost
            if setup.modelOptions.target.targetConstraint
                targetVioCostObj = phase.addNewLagrangeCost(...
                                    @targetViolationCostFcn,...
                                    falcon.Cost('targetVioCost',...
                                    1e-4), tau);                                % 1e+0, 1e+5, 1e+7, 
                targetVioCostObj.setParameters(...
                                falcon.Parameter('captureRadius',...
                                setup.targetConfig.rT_max, 'fixed', true));
            end


            
            % Hitconstraint
%             hitConstraint = falcon.Constraint('hitCon', -inf, 0.09, 1e+2);
%             phase.addNewPathConstraint(@hitConFcn, hitConstraint, 1);

            % Softconstraint for successful target protection
%             defenceConstraint = falcon.Constraint('defenceCon', setup.targetConfig.rT_max^2, +inf, 1e-2);
%             phase.addNewPathConstraint(@defenceConFcn,defenceConstraint, 1);
            
%             defenceCost = falcon.Cost('defenceCost', 1e-4);
%             problem.addNewMayerCost(@defenceCostFcn, defenceCost, phase, 1);
            
            
            % Defender velocity constraint
%             velocityConstraint = falcon.Constraint('defVelocityConstraint', 0, setup.defenderConfig.V_abs_max^2, 1e-2);
%             phase.addNewPathConstraint(@defVelConFcn, velocityConstraint ,tau); 

            % Add defender cost
%             defConObj = phase.addNewPathConstraint(@defConFcn, falcon.Constraint('defCon', -inf, 0, 1e+2),tau);
%             defConObj.setParameters([   falcon.Parameter('T2W' , setup.defenderConfig.T2W_max)
%                                         falcon.Parameter('g'   , setup.environmentOptions.g)
%                                         falcon.Parameter('s'   , setup.sigma)
%                                         ]);
                                    
            
            % Seeker and thrust constraints
            if setup.modelOptions.defender.SixDoF
                
                if setup.modelOptions.defender.FovConstraint
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
                thrustConstraint = falcon.Constraint('defThrustConstraint', 0, 1);
                phase.addNewPathConstraint(@defThrustConFcn, thrustConstraint ,tau); 
            end
                        
            % Proximity parameter constraint for invader maneuver parametrization
%             proximityConstraint = falcon.Constraint('proxConstraint_x', -inf, +inf);
            % Point constraint
%             proxCon = phase.addNewPathConstraint(@proxConstraintFcn, proximityConstraint ,tau); 
%             proxCon.setParameters(pProxCon);
            
            
        case 'inv'         
            
%             % Add mayer cost for invader objective
%             invaderMayerCosts = falcon.Cost('targetDistance');
%             problem.addNewMayerCost(@invaderMayerCostFunction, invaderMayerCosts, phase, 1);   
%             
%             % Add invader thrust velocity constraint
%             if strcmp(setup.invaderConfig.Type,'Quad2')
%                 invaderConstraints = [  falcon.Constraint('invaderVelocityConstraint'   , 0, setup.invaderConfig.vI_abs_max^2  , 1e-2)
%                                         falcon.Constraint('invaderThrustConstraint'     , -inf, 0)];
%                 phase.addNewPathConstraint(@invaderConstraintFunction, invaderConstraints ,tau); 
%             end   
%             
%             % Add evasion constraint
%             evasionConstraint = falcon.Constraint('evasionConstraint', 1  , +inf  , 1e+0);
%             phase.addNewPathConstraint(@invEvasionConstraintFcn, evasionConstraint);            
            
    end
    
    
    % Put time in cost function
    problem.addNewParameterCost(tf, 'min', 'Scaling', 1e-0);   % 1e-1
%     problem.addNewParameterCost(tf);
    
    
    %% gpC Collocation
    if setup.modelOptions.uncertainty
        problem.setExpGridType('tensor')


        % Number of nodes
        problem.setExpNodes(2);

        % States setting
        states_mat      = zeros(10,4,1);
        states_mat(:,1) =-10;
        states_mat(:,2) = 10;
        states_mat(:,3) = 1e0;
        states_mat(:,4) = 0;
        problem.setExpStates_LBUBOffScal(states_mat);

        % Controls setting
        ctrls_mat      = zeros(10,4,1);
        ctrls_mat(:,1) =-10;
        ctrls_mat(:,2) = 10;
        ctrls_mat(:,3) = 1e0;
        ctrls_mat(:,4) = 0;
        problem.setExpCtrls_LBUBOffScal(ctrls_mat);

        % Parameters setting
        param_mat      = zeros(10,4,1);
        param_mat(:,1) =-10;
        param_mat(:,2) = 10;
        param_mat(:,3) = 1e0;
        param_mat(:,4) = 0;
        problem.setExpParams_LBUBOffScal(param_mat);

        % Initial guess
        problem.Phases(1,1).setExpStatesInitGuess(zeros(10,length(tau),length(variables.states)));
        problem.Phases(1,1).setExpCtrlsInitGuess(zeros(10,length(tau),length(variables.controls)));
        problem.setExpParamsInitGuess(zeros(10,1,length(parameters)));
        
    end  
    
    
% EoF
end


