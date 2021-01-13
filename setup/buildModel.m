% -------------------------------------------------------------------------
%                                FALCON.m
% Copyright (c) 2014-2018 Institute of Flight System Dynamics, TUM, Munich
% Matthias Bittner, Matthias Rieck, Maximilian Richter,
% Benedikt Grueter, Johannes Diepolder, Florian Schwaiger,
% Patrick Piprek, and Florian Holzapfel
% -------------------------------------------------------------------------
% DefenderInvaderProblem
% -------------------------------------------------------------------------

function [] = buildModel(ModelOptions,ModelName)

DefenderOptions = ModelOptions.Defender;
InvaderOptions  = ModelOptions.Invader;

%     addpath(fullfile(fileparts(mfilename('fullpath')), 'subsystems'));
if ModelOptions.Defender.SixDoF
    fprintf('\n\n%s\n# Building 6-DoF model %s ...\n', repmat('#', 1, 50), ModelName);
else
    fprintf('\n\n%s\n# Building 3-DoF model %s ...\n', repmat('#', 1, 50), ModelName);
end

%% create the model variables
Variables = createDataTypes(ModelOptions);

%% 1 Create the falcon.SimulationModelBuilder Object
Builder = falcon.SimulationModelBuilder(ModelName,...
    Variables.States,...
    Variables.Controls,...
    Variables.Parameters);
% Add constants to model
defaultModelConstants(Builder, ModelOptions);

%% 2 Add Subsystems

%% 2.1 Environment

% Time propagation
if ModelOptions.timeState
    Builder.addSubsystem(@dipTime);
end

%% 2.2 Attitude
if DefenderOptions.SixDoF
    switch DefenderOptions.Attitude
        case 'Euler'
            Builder.addSubsystem(@defAttitudeEuler);
        case 'Quaternion'
        otherwise
            error('Unsupported attitude option: %s', DefenderOptions.Attitude);
    end
end

%% 2.3 Propulsion

switch ModelOptions.optimize
    case 'def'
        switch DefenderOptions.Type
            case 'Quad'
                if DefenderOptions.SixDoF
                    if DefenderOptions.MotorTable
                        Builder.addSubsystem(@sysPropulsionData);
                        error('Not implemented');  % TODO ==> PWM Mapping
                    else
                        % Forces and Moments 6DoF
                        Builder.addSubsystem(@defQuadPropulsionModel6DoF);
                    end
                else
                    % Forces and Moments 3DoF
                    Builder.addSubsystem(@defQuadPropulsionModel3DoF);
                end
            otherwise
                error('Defender type not implemented');
        end
    case 'inv'
        switch InvaderOptions.Type
            case 'Quad2'
                % Invader forces 3DoF
                Builder.addSubsystem(@invQuadPropulsionModel3DoF);
            otherwise
                error('Invader type not implemented');
        end
    otherwise
        error('Check setup.optimize')
end

%% 2.4 Aerodynamics
if DefenderOptions.Aero
    switch DefenderOptions.Type
        case 'Quad'
            if DefenderOptions.SixDoF
                Builder.addSubsystem(@defQuadAero6DoF);
                Builder.SplitVariable('FDAD',{'FDAD_x'; 'FDAD_y'; 'FDAD_z'});
            else
                error('Defender 3DoF Aerodynamics not implemented yet');
            end
        otherwise
            error('Defender type not implemented');
    end
else
    if DefenderOptions.SixDoF
        Builder.addConstant('FDAD', [0; 0; 0]);
    else
        Builder.addConstant('FDAO', [0; 0; 0]);
    end
end

%% 2.5 Total Forces and Moments

switch ModelOptions.Optimize
    case 'def'
        % Defender
        if DefenderOptions.SixDoF
            Builder.addSubsystem(@defForces6DoF);
            Builder.addSubsystem(@defMoments);
        else
            Builder.addSubsystem(@defForces3DoF);
        end
        % Invader
        if ModelOptions.timeState
            Builder.addSubsystem(@invPursuitGuidanceTime);
        else
            if InvaderOptions.Escape
                Builder.CombineVariables('pTOO', {'pTOO_x', 'pTOO_y', 'pTOO_z'}');
                Builder.addSubsystem(@invPursuitEscapeGuidance);
            else
                Builder.CombineVariables('vIOO', {'vIOO_x', 'vIOO_y', 'vIOO_z'}');
                Builder.addSubsystem(@invPursuitGuidance);
            end
        end
    case 'inv'
        % Defender
        switch DefenderOptions.Guidance
            case 'PN'
                %                     builder.CombineVariables('vDOO', {'u', 'v', 'w'}');
                %                     builder.addSubsystem(@defSeekerLOS);
                Builder.addSubsystem(@defPN);
            case 'APN'
                error('APN guidance not implemented')
            otherwise
                error('Unsupported defender guidance option: %s', DefenderOptions.Guidance);
        end
        % Invader
        Builder.addSubsystem(@invForces);
end

%% 2.6 Equations of Motion

% EOM - Defender Position
Builder.addSubsystem(@defEOMPosition);
Builder.CombineVariables('pDOO', {'x', 'y', 'z'}');

% EOM - Defender Translation
Builder.addSubsystem(@defEOMTranslation);

% EOM - Defender Attitude
if DefenderOptions.SixDoF
    switch DefenderOptions.Attitude
        case 'Euler'
            Builder.addSubsystem(@defEOMAttitudeEuler);
        case 'Quaternion'
            % Add the `sysAttitudeQuaternion` subsystem, which outputs the
            % `quat_dot` vector.  Split `quat_dot` into `q0_dot` etc.
            error('Not implemented');  % TODO
        otherwise
            error('Unsupported attitude option: %s', DefenderOptions.Attitude);
    end
end

% EOM - Defender Rotation
if DefenderOptions.SixDoF
    Builder.CombineVariables('wDOD', {'p', 'q', 'r'}');
    Builder.addSubsystem(@defEOMRotation);
end

% EOM -  Defender Actuators
if DefenderOptions.MotorLag && DefenderOptions.SixDoF
    switch DefenderOptions.Type
        case 'Quad'
            Builder.addSubsystem(@defQuadMotorLag);
        otherwise
            error('Defender type not implemented!');
    end
end

% EOM - Invader
switch ModelOptions.Optimize
    case 'inv'
        if InvaderOptions.SixDoF
            error('Invader 6DoF not implemented!');
        else
            switch InvaderOptions.Type
                case 'Quad1'
                    Builder.addSubsystem(@invEOMPosition1stO);
                case 'Quad2'
                    Builder.addSubsystem(@invEOMPosition);
                    Builder.addSubsystem(@invEOMTranslation);
                otherwise
                    error('Invader type not implemented');
            end
        end
    case 'def'
        Builder.addSubsystem(@invEOMPosition);
    otherwise
        error('Check setup.optimize!');
end

%% 2.7 Observability Cost Function

if ModelOptions.observabilityCostFcn
    % Pseudo defender controls
    Builder.addSubsystem(@(u_dot) u_dot,...
        'Inputs', {'u_dot'},...
        'Outputs', {'u1'});
    Builder.addSubsystem(@(v_dot) v_dot,...
        'Inputs', {'v_dot'},...
        'Outputs', {'u2'});
    Builder.addSubsystem(@(w_dot) w_dot,...
        'Inputs', {'w_dot'},...
        'Outputs', {'u3'});
    
    % Invader velocity
    Builder.addSubsystem(@(x_inv_dot) x_inv_dot,...
        'Inputs', {'x_inv_dot'},...
        'Outputs', {'u_inv_out'});
    Builder.addSubsystem(@(y_inv_dot) y_inv_dot,...
        'Inputs', {'y_inv_dot'},...
        'Outputs', {'v_inv_out'});
    Builder.addSubsystem(@(z_inv_dot) z_inv_dot,...
        'Inputs', {'z_inv_dot'},...
        'Outputs', {'w_inv_out'});
    

end

%% 3 Set Outputs and State Derivatives and Build the Model
% Define Outputs / (Constraint): Total Load Factor in the negative z_B Direction
% Split the load factor in the Body Fixed Frame for the Output/Constraint first:

% Check if outputs are empty
if strcmp({Variables.Outputs.Name},'Void')
    Builder.addSubsystem(@(x) 0, 'Inputs', {'x'}  , 'Outputs', {'Void'});
end


Builder.setOutputs(Variables.Outputs);

Builder.setStateDerivativeNames(Variables.StateDerivativeNames);

% Build the Model and check the generated derivatives
Builder.Build();

%% 2.4 Build Path Constraints
%     VelCon = falcon.PathConstraintBuilder('Vel_con', 0, variables.states,...
%                                           0, 0, @pathConstraints);
%     VelCon.Build();

% %     %% 2.4 CREATE POINT Constraint
%     Hit = falcon.PointConstraintBuilder('HitCon', @pointConstraint);
%     Hit.Build();


end