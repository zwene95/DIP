% -------------------------------------------------------------------------
%                                FALCON.m
% Copyright (c) 2014-2018 Institute of Flight System Dynamics, TUM, Munich
% Matthias Bittner, Matthias Rieck, Maximilian Richter,
% Benedikt Grueter, Johannes Diepolder, Florian Schwaiger,
% Patrick Piprek, and Florian Holzapfel
% -------------------------------------------------------------------------
% DefenderInvaderProblem
% -------------------------------------------------------------------------

function [] = buildModel(modelOptions,modelName)

    defenderOptions = modelOptions.defender;
    invaderOptions  = modelOptions.invader;

%     addpath(fullfile(fileparts(mfilename('fullpath')), 'subsystems'));
    fprintf('\n\n%s\n# Building 6-DoF model %s ...\n', repmat('#', 1, 50), modelName);

    %% create the model variables
    variables = createDataTypes(modelOptions);

    %% 1 Create the falcon.SimulationModelBuilder Object
    builder = falcon.SimulationModelBuilder(modelName,...
                                            variables.states,...
                                            variables.controls,...
                                            variables.parameters);
    % Add constants to model
    addModelConstants(builder, modelOptions);

    %% 2 Add Subsystems

    %% 2.1 Environment
    
    % Time propagation
    if modelOptions.timeState
        builder.addSubsystem(@dipTime);
    end
    

    %% 2.2 Attitude
    if defenderOptions.SixDoF
        switch defenderOptions.Attitude
        case 'Euler'
            builder.addSubsystem(@defAttitudeEuler);
        case 'Quaternion'
        otherwise
            error('Unsupported attitude option: %s', defenderOptions.Attitude);
        end
    end
    
    
    %% 2.3 Propulsion   
    
    switch modelOptions.optimize
        case 'def'
            switch defenderOptions.Type
                case 'Quad'
                    if defenderOptions.SixDoF
                        if defenderOptions.MotorTable
                            builder.addSubsystem(@sysPropulsionData);
                            error('Not implemented');  % TODO ==> PWM Mapping        
                        else
                            % Forces and Moments 6DoF
                            builder.addSubsystem(@defQuadPropulsionModel6DoF);                
                        end 
                    else
                        % Forces and Moments 3DoF
                        builder.addSubsystem(@defQuadPropulsionModel3DoF);                
                    end
                otherwise
                        error('Defender type not implemented');
            end
        case 'inv'
            switch invaderOptions.Type
                case 'Quad2'
                    % Invader forces 3DoF
                    builder.addSubsystem(@invQuadPropulsionModel3DoF);
                otherwise
                    error('Invader type not implemented');
            end
        otherwise
            error('Check setup.optimize')
    end
    
    
    %% 2.4 Aerodynamics    
    if defenderOptions.Aero        
        switch defenderOptions.Type
            case 'Quad'        
                if defenderOptions.SixDoF
                    builder.addSubsystem(@defQuadAero6DoF);
                    builder.SplitVariable('FDAD',{'FDAD_x'; 'FDAD_y'; 'FDAD_z'});                   
                else
                    error('Defender 3DoF Aerodynamics not implemented yet');
                end
            otherwise
                error('Defender type not implemented');
        end
    else
        if defenderOptions.SixDoF
            builder.addConstant('FDAD', [0; 0; 0]);      
%             builder.addSubsystem(@(x) x, 'Inputs', {'theta'}  , 'Outputs', {'Void'});             -----> to be removed
        else
            builder.addConstant('FDAO', [0; 0; 0]);
        end        
    end
    
    
               
    %% 2.5 Total Forces and Moments
    
    switch modelOptions.optimize
        case 'def'
            % Defender
            if defenderOptions.SixDoF
                builder.addSubsystem(@defForces6DoF);
                builder.addSubsystem(@defMoments);
            else
                builder.addSubsystem(@defForces3DoF);
            end
            % Invader
            if modelOptions.timeState
                builder.addSubsystem(@invPursuitGuidanceTime);
            else
                if invaderOptions.Escape
                    builder.CombineVariables('pTOO', {'pTOO_x', 'pTOO_y', 'pTOO_z'}');
                    builder.addSubsystem(@invPursuitEscapeGuidance);
                else
                    builder.CombineVariables('vIOO', {'vIOO_x', 'vIOO_y', 'vIOO_z'}');
                    builder.addSubsystem(@invPursuitGuidance);
                end
            end
        case 'inv'
            % Defender
            switch defenderOptions.Guidance
                case 'PN'
%                     builder.CombineVariables('vDOO', {'u', 'v', 'w'}');
%                     builder.addSubsystem(@defSeekerLOS);
                    builder.addSubsystem(@defPN);
                case 'APN'
                    error('APN guidance not implemented')
                otherwise
                    error('Unsupported defender guidance option: %s', defenderOptions.Guidance);
            end
            % Invader
            builder.addSubsystem(@invForces);
    end

    %% 2.6 Equations of Motion
    
    % EOM - Defender Position
    builder.addSubsystem(@defEOMPosition);
    builder.CombineVariables('pDOO', {'x', 'y', 'z'}');

    % EOM - Defender Translation
    builder.addSubsystem(@defEOMTranslation);
    
    % EOM - Defender Attitude
    if defenderOptions.SixDoF        
        switch defenderOptions.Attitude
        case 'Euler'
            builder.addSubsystem(@defEOMAttitudeEuler);
        case 'Quaternion'
            % Add the `sysAttitudeQuaternion` subsystem, which outputs the
            % `quat_dot` vector.  Split `quat_dot` into `q0_dot` etc.
            error('Not implemented');  % TODO
        otherwise
            error('Unsupported attitude option: %s', defenderOptions.Attitude);
        end
    end    

    % EOM - Defender Rotation
    if defenderOptions.SixDoF
        builder.CombineVariables('wDOD', {'p', 'q', 'r'}');
        builder.addSubsystem(@defEOMRotation);    
    end

    % EOM -  Defender Actuators
    if defenderOptions.MotorLag && defenderOptions.SixDoF
        switch defenderOptions.Type
            case 'Quad'
                builder.addSubsystem(@defQuadMotorLag);
            otherwise
                error('Defender type not implemented!');
        end
    end
    
    % EOM - Invader       
    switch modelOptions.optimize
        case 'inv'
            if invaderOptions.SixDoF
                error('Invader 6DoF not implemented!');
            else        
                switch invaderOptions.Type
                    case 'Quad1'
                        builder.addSubsystem(@invEOMPosition1stO);
                    case 'Quad2'
                        builder.addSubsystem(@invEOMPosition);
                        builder.addSubsystem(@invEOMTranslation);
                    otherwise
                        error('Invader type not implemented');
                end  
            end
        case 'def'
            builder.addSubsystem(@invEOMPosition);
        otherwise
            error('Check setup.optimize!');
    end
    
    
    %% 2.7 State Observer
    if modelOptions.observer  
        
        % Process covariance matrix
        builder.CombineVariables('Q', {
            'Q_11'  , 'Q_12', 'Q_13'
            'Q_12'  , 'Q_22', 'Q_23'
            'Q_13'  , 'Q_23', 'Q_33'            
        });    
        
        % Measurement covariance matrix
        builder.CombineVariables('R', {
            'R_11'  , 'R_12' 
            'R_12'  , 'R_22'                         
        }); 
        
%         % State covariance matrix
%         builder.CombineVariables('P', {
%             'P_11'  , 'P_12', 'P_13', 'P_14', 'P_15', 'P_16'
%             'P_21'  , 'P_22', 'P_23', 'P_24', 'P_25', 'P_26'
%             'P_31'  , 'P_32', 'P_33', 'P_34', 'P_35', 'P_36'
%             'P_41'  , 'P_42', 'P_43', 'P_44', 'P_45', 'P_46'
%             'P_51'  , 'P_52', 'P_53', 'P_54', 'P_55', 'P_56'
%             'P_61'  , 'P_62', 'P_63', 'P_64', 'P_65', 'P_66'
%         });       

        % State covariance matrix
        builder.CombineVariables('P', {
            'P_11'  , 'P_12', 'P_13', 'P_14', 'P_15', 'P_16'
            'P_12'  , 'P_22', 'P_23', 'P_24', 'P_25', 'P_26'
            'P_13'  , 'P_23', 'P_33', 'P_34', 'P_35', 'P_36'
            'P_14'  , 'P_24', 'P_34', 'P_44', 'P_45', 'P_46'
            'P_15'  , 'P_25', 'P_35', 'P_45', 'P_55', 'P_56'
            'P_16'  , 'P_26', 'P_36', 'P_46', 'P_56', 'P_66'
        });     
                                    
        % Measurement Jacobian (State Jacobian in constants)
%         builder.addSubsystem(@obsMeasurementJacobian);                     
        builder.addDerivativeSubsystem(@obsMeasurementJacobianDS,...
            'Outputs', {'H'},...
            'Inputs' , {'x', 'y', 'z', 'x_inv', 'y_inv', 'z_inv'});

                                    
        % Measurements
        builder.addDerivativeSubsystem(@obsTrueMeasurementDS,...
            'Outputs', {'meas_true'},...
            'Inputs' , {'x', 'y', 'z', 'x_inv', 'y_inv', 'z_inv'});
        
        builder.addDerivativeSubsystem(@obsEstMeasurementDS,...
            'Outputs', {'meas_est'},...
            'Inputs' , {'x_est', 'y_est', 'z_est'});
        
%         addSubsystem(@obsTrueMeasurement);
%         builder.addSubsystem(@obsEstMeasurement);
        
        % Update Gain
        builder.addSubsystem(@obsGain);                                     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%                                  
        
        % Predict-Update Step        
        builder.addSubsystem(@obsStateUpdate);
        
        builder.addSubsystem(@obsCovUpdate);                                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%              
        
%         builder.SplitVariable('P_dot', {
%             'P_11_dot', 'P_12_dot', 'P_13_dot', 'P_14_dot', 'P_15_dot', 'P_16_dot'
%             'P_21_dot', 'P_22_dot', 'P_23_dot', 'P_24_dot', 'P_25_dot', 'P_26_dot'
%             'P_31_dot', 'P_32_dot', 'P_33_dot', 'P_34_dot', 'P_35_dot', 'P_36_dot'
%             'P_41_dot', 'P_42_dot', 'P_43_dot', 'P_44_dot', 'P_45_dot', 'P_46_dot'
%             'P_51_dot', 'P_52_dot', 'P_53_dot', 'P_54_dot', 'P_55_dot', 'P_56_dot'
%             'P_61_dot', 'P_62_dot', 'P_63_dot', 'P_64_dot', 'P_65_dot', 'P_66_dot'
%         });
    end
    

    %% 2.3 Set Outputs and State Derivatives and Build the Model
    % Define Outputs / (Constraint): Total Load Factor in the negative z_B Direction
    % Split the load factor in the Body Fixed Frame for the Output/Constraint first:
    builder.setOutputs(variables.outputs);

    builder.setStateDerivativeNames(variables.stateDerivativeNames);
   
    % Build the Model and check the generated derivatives
    builder.Build();    
    
    %% 2.4 Build Path Constraints
%     VelCon = falcon.PathConstraintBuilder('Vel_con', 0, variables.states,...
%                                           0, 0, @pathConstraints);
%     VelCon.Build();

% %     %% 2.4 CREATE POINT Constraint
%     Hit = falcon.PointConstraintBuilder('HitCon', @pointConstraint);
%     Hit.Build();


end