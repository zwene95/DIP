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
if modelOptions.defender.SixDoF
    fprintf('\n\n%s\n# Building 6-DoF model %s ...\n', repmat('#', 1, 50), modelName);
else
    fprintf('\n\n%s\n# Building 3-DoF model %s ...\n', repmat('#', 1, 50), modelName);
end

%% create the model variables
variables = createDataTypes(modelOptions);

%% 1 Create the falcon.SimulationModelBuilder Object
builder = falcon.SimulationModelBuilder(modelName,...
    variables.states,...
    variables.controls,...
    variables.parameters);
% Add constants to model
defaultModelConstants(builder, modelOptions);

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

%% 2.7 Observability Cost Function

if modelOptions.observabilityCostFcn
    % Pseudo defender controls
    builder.addSubsystem(@(u_dot) u_dot,...
        'Inputs', {'u_dot'},...
        'Outputs', {'u1'});
    builder.addSubsystem(@(v_dot) v_dot,...
        'Inputs', {'v_dot'},...
        'Outputs', {'u2'});
    builder.addSubsystem(@(w_dot) w_dot,...
        'Inputs', {'w_dot'},...
        'Outputs', {'u3'});
    
    % Invader velocity
    builder.addSubsystem(@(x_inv_dot) x_inv_dot,...
        'Inputs', {'x_inv_dot'},...
        'Outputs', {'u_inv_out'});
    builder.addSubsystem(@(y_inv_dot) y_inv_dot,...
        'Inputs', {'y_inv_dot'},...
        'Outputs', {'v_inv_out'});
    builder.addSubsystem(@(z_inv_dot) z_inv_dot,...
        'Inputs', {'z_inv_dot'},...
        'Outputs', {'w_inv_out'});
    

end

%% 3 Set Outputs and State Derivatives and Build the Model
% Define Outputs / (Constraint): Total Load Factor in the negative z_B Direction
% Split the load factor in the Body Fixed Frame for the Output/Constraint first:

% Check if outputs are empty
if strcmp({variables.outputs.Name},'Void')
    builder.addSubsystem(@(x) 0, 'Inputs', {'x'}  , 'Outputs', {'Void'});
end


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