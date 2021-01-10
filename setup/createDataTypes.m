
% -------------------------------------------------------------------------
%                                FALCON.m
% Copyright (c) 2014-2018 Institute of Flight System Dynamics, TUM, Munich
% Matthias Bittner, Matthias Rieck, Maximilian Richter,
% Benedikt Grueter, Johannes Diepolder, Florian Schwaiger,
% Patrick Piprek, and Florian Holzapfel
% -------------------------------------------------------------------------
% DefenderInvaderProblem Data Types
% -------------------------------------------------------------------------

function [ variables ] = createDataTypes(modelOptions)

    defenderOptions =   modelOptions.defender;
    invaderOptions =    modelOptions.invader;
    RPM_max =           modelOptions.defender.RPM_max;
    
%% States    

    % Zeit als State einbinden
            %            Name    , LowerBound, UpperBound, Scaling
    if modelOptions.timeState
        timeState =  falcon.State('T'     , 0         , +inf      , 1e+0);
    else
        timeState =  falcon.State.empty();
    end

    % Defender states
    defenderPositionStates = [
        %            Name           , LowerBound, UpperBound, Scaling
        falcon.State('x'            , -inf      , +inf      , 1e-0)         % 1e-1    
        falcon.State('y'            , -inf      , +inf      , 1e-0)         % 1e-1    
        falcon.State('z'            , -inf      ,   +0      , 1e-0)         % 1e-1    
    ];

    defenderTranslationStates = [
        %            Name           , LowerBound, UpperBound, Scaling
        falcon.State('u'            , -inf      , +inf      , 1e-0)         % 1e-1    
        falcon.State('v'            , -inf      , +inf      , 1e-0)         % 1e-1    
        falcon.State('w'            , -inf      , +inf      , 1e-0)         % 1e-1    
    ];

    if defenderOptions.SixDoF
        switch defenderOptions.Attitude
            case 'Euler'
                defenderAttitudeStates = [
                    %            Name           , LowerBound, UpperBound, Scaling
                    falcon.State('phi'          , -pi       , +pi       , 1e+0);
                    falcon.State('theta'        , -pi/2     , +pi/2     , 1e+0);
                    falcon.State('psi'          , -pi       , +pi       , 1e+0);
                ];
            case 'Quaternion'
                defenderAttitudeStates = [
                    %            Name           , LowerBound, UpperBound, Scaling
                    error('Not implemented');  % TODO
                ];
            otherwise
                error('Unsupported attitude option: %s', defenderOptions.Attitude);
        end
        
        defenderRotationStates = [
            %            Name           , LowerBound, UpperBound, Scaling
            falcon.State('p'            , -4*pi     , +4*pi     , 1e+0)
            falcon.State('q'            , -4*pi     , +4*pi     , 1e+0)
            falcon.State('r'            , -4*pi    	, +4*pi    	, 1e+0)
        ];
    else
        defenderAttitudeStates = falcon.State.empty();
        defenderRotationStates = falcon.State.empty();
    end

    if defenderOptions.MotorLag && defenderOptions.SixDoF
        switch defenderOptions.Type
            case 'Quad'
                defenderMotorStates = [            
                    %            Name         , LowerBound, UpperBound  , Scaling
                    falcon.State('w1'         , 0         , RPM_max     , 1/RPM_max)
                    falcon.State('w2'         , 0         , RPM_max     , 1/RPM_max)
                    falcon.State('w3'         , 0         , RPM_max     , 1/RPM_max)
                    falcon.State('w4'         , 0         , RPM_max     , 1/RPM_max)
                ];
            otherwise
                error('Defender type not implemented!');
        end
    else
        defenderMotorStates = falcon.State.empty();
    end
    
    % Invader states
    if invaderOptions.SixDoF
        error('Invader 6DoF Model not implemented');
    else
        switch modelOptions.invader.Type
            case 'Quad1'
                invaderStates = [
                    %            Name           , LowerBound, UpperBound, Scaling
                    falcon.State('x_inv'        , -inf      , +inf      , 1e-0) %1e+0
                    falcon.State('y_inv'        , -inf      , +inf      , 1e-0) %1e+0
                    falcon.State('z_inv'        , -inf      , +0        , 1e-0) %1e+0
                ];                
            case 'Quad2'
                invaderStates = [
                    %            Name           , LowerBound, UpperBound, Scaling
                    falcon.State('x_inv'        , -inf      , +inf      , 1e-0)     % 1e-1    
                    falcon.State('y_inv'        , -inf      , +inf      , 1e-0)     % 1e-1    
                    falcon.State('z_inv'        , -inf      , +0        , 1e-0)     % 1e-1    
                    falcon.State('u_inv'        , -inf      , +inf      , 1e-0)     % 1e-1    
                    falcon.State('v_inv'        , -inf      , +inf      , 1e-0)     % 1e-1    
                    falcon.State('w_inv'        , -inf      , +inf      , 1e-0)     % 1e-1    
                ];
        end
    end
    
    states = [
        defenderPositionStates
        defenderTranslationStates
        invaderStates
        defenderAttitudeStates        
        defenderRotationStates        
        defenderMotorStates
        timeState        
    ];

    stateDerivativeNames = ...
        cellfun(@(x) strcat(x, '_dot'), {states.Name}, 'UniformOutput', false);

%% Controls

    % Defender control    
    switch defenderOptions.Type
        case 'Quad'
            if defenderOptions.SixDoF        
                if defenderOptions.MotorLag
                    defenderControls = [
                        %              Name         , LowerBound, UpperBound, Scaling
                        falcon.Control('w1_cmd'     , 0         , RPM_max   , 1/RPM_max)    % 1e-4    
                        falcon.Control('w2_cmd'     , 0         , RPM_max   , 1/RPM_max)    % 1e-4    
                        falcon.Control('w3_cmd'     , 0         , RPM_max   , 1/RPM_max)    % 1e-4    
                        falcon.Control('w4_cmd'     , 0         , RPM_max   , 1/RPM_max)    % 1e-4    
                    ];
                else
                    defenderControls = [
                        %              Name         , LowerBound, UpperBound, Scaling
                        falcon.Control('w1'         , 0         , RPM_max   , 1/RPM_max)    % 1e-4    
                        falcon.Control('w2'         , 0         , RPM_max   , 1/RPM_max)    % 1e-4    
                        falcon.Control('w3'         , 0         , RPM_max   , 1/RPM_max)    % 1e-4    
                        falcon.Control('w4'         , 0         , RPM_max   , 1/RPM_max)    % 1e-4    
                    ];
                end
            else
%                 defenderControls = [
%                     %              Name         , LowerBound, UpperBound, Scaling
%                     falcon.Control('delta_c'    , 0         , 1         , 1e+1)
%                     falcon.Control('theta_c'    , -1        , +1        , 1e+1)
%                     falcon.Control('chi_c'      , -pi       , +pi       , 1e+0)
%                 ];
                defenderControls = [
                    %              Name         , LowerBound, UpperBound, Scaling
                    falcon.Control('T_x'        , -1        , +1        , 1e+0) %1e+1
                    falcon.Control('T_y'        , -1        , +1        , 1e+0) %1e+1
                    falcon.Control('T_z'        , -1        , +1        , 1e+0) %1e+1
                ];
            end  
        otherwise
                error('Defender type not implemented!');
    end
                      
    
    % Invader control
    if invaderOptions.SixDoF
        error('Invader 6DoF model not implemented');
    else
        switch invaderOptions.Type
            case 'Quad1'
                invaderControls = [
                        %              Name     , LowerBound, UpperBound    , Scaling
                        falcon.Control('vI_x'   , -1        , +1            , 1e+0)
                        falcon.Control('vI_y'   , -1        , +1            , 1e+0)
                        falcon.Control('vI_z'   , -1        , +1            , 1e+0)
                    ];
%                     invaderControls = [
%                         %              Name         , LowerBound, UpperBound    , Scaling
%                         falcon.Control('vI_abs'     , 0         , vI_abs_max    , 1e+0)
%                         falcon.Control('vI_theta'   , -pi/2     , +pi/2         , 1e+0)
%                         falcon.Control('vI_chi'     , -pi       , +pi           , 1e+0)
%                     ];  
%             case 'Quad2'
%                     invaderControls = [
%                         %              Name         , LowerBound, UpperBound    , Scaling
%                         falcon.Control('delta_Ic'   , 0         , T2W_max_inv   , 1e+0)
%                         falcon.Control('theta_Ic'   , 0         , +pi           , 1e+0)
%                         falcon.Control('chi_Ic'     , -pi       , +pi           , 1e+0)
%                     ];  
            case 'Quad2'
                invaderControls = [
                    %              Name         , LowerBound, UpperBound    , Scaling
                    falcon.Control('T_x_inv'    , -1        , +1            , 1e+1)
                    falcon.Control('T_y_inv'    , -1        , +1            , 1e+1)
                    falcon.Control('T_z_inv'    , -1        , +1            , 1e+1)
                ];
            otherwise
                error('Invader type not supported');        
        end
    end
    
    % Choose control depending on optimized player
    switch modelOptions.optimize
        case 'def'
            controls = defenderControls;
        case 'inv'
            controls = invaderControls;
        otherwise
            error('Check setup.optimize')
    end

%% Parameters
    switch modelOptions.optimize
        case 'def'
            if modelOptions.uncertainty
                modelParameters = [
                    falcon.DistrParameter('vIOO_x')
                    falcon.DistrParameter('vIOO_y')
                    falcon.DistrParameter('vIOO_z')
                    falcon.DistrParameter('T2W_max')
                    falcon.DistrParameter('rEscape')
                ];
            else
                modelParameters = [
                    falcon.Parameter('pTOO_x')
                    falcon.Parameter('pTOO_y')
                    falcon.Parameter('pTOO_z')
                    falcon.Parameter('vIOO_x')
                    falcon.Parameter('vIOO_y')
                    falcon.Parameter('vIOO_z')
                    falcon.Parameter('vI_abs_max')
                    falcon.Parameter('T2W_max')
                    falcon.Parameter('MotorTC')
                    falcon.Parameter('rEscape')
                ];
            end
        case 'inv'
            modelParameters = [
                falcon.Parameter('vI_abs_max')
                falcon.Parameter('T2W_max_inv')
            ];
        otherwise
            error('Check setup.optimize')
    end
        
    parameters = [
        modelParameters        
    ];                
    
%% Outputs

switch modelOptions.optimize
    case 'def'
        
        % Observability cost funciton
        if modelOptions.observabilityCostFcn
            outputsObservability = [                
                falcon.Output('u_inv_out')
                falcon.Output('v_inv_out')
                falcon.Output('w_inv_out')
                falcon.Output('u1')
                falcon.Output('u2')
                falcon.Output('u3')
            ];
            
        else
            outputsObservability = falcon.Output.empty();
        end
        
        % Aerodynamics
%         if defenderOptions.Aero
%             outputsAero = [
%                 %                     %             Name
%                 falcon.Output('FDAD_x')
%                 falcon.Output('FDAD_y')
%                 falcon.Output('FDAD_z')
%                 %                     falcon.Output('azimuth_true')
%                 %                     falcon.Output('elevation_true')
%                 %                     falcon.Output('azimuth_meas')
%                 %                     falcon.Output('elevation_meas')
%             ];
%         else
%             outputsAero = falcon.Output.empty();
%         end
        
        outputsAero = falcon.Output.empty();
        
        outputs = [
            outputsObservability
            outputsAero
        ];
        
        if isempty(outputs)
            outputs = falcon.Output('Void');
        end
        
    case 'inv'
        outputs = falcon.Output.empty();
end

variables = struct();
variables.states = states;
variables.stateDerivativeNames = stateDerivativeNames;
variables.controls = controls;
variables.outputs = outputs;
variables.parameters = parameters;

end