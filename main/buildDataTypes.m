
% -------------------------------------------------------------------------
%                                FALCON.m
% Copyright (c) 2014-2018 Institute of Flight System Dynamics, TUM, Munich
% Matthias Bittner, Matthias Rieck, Maximilian Richter,
% Benedikt Grueter, Johannes Diepolder, Florian Schwaiger,
% Patrick Piprek, and Florian Holzapfel
% -------------------------------------------------------------------------
% DefenderInvaderProblem Data Types
% -------------------------------------------------------------------------

function [ Variables ] = buildDataTypes(ModelOptions)

    DefenderOptions =   ModelOptions.Defender;
    InvaderOptions =    ModelOptions.Invader;
    RPM_max =           ModelOptions.Defender.RPM_max;
    
%% States    

    % Zeit als State einbinden
            %            Name    , LowerBound, UpperBound, Scaling
    if ModelOptions.TimeState
        TimeState =  falcon.State('T'     , 0         , +inf      , 1e+0);
    else
        TimeState =  falcon.State.empty();
    end

    % Defender states
    DefenderPositionStates = [
        %            Name           , LowerBound, UpperBound, Scaling
        falcon.State('x'            , -inf      , +inf      , 1e-0)         % 1e-1    
        falcon.State('y'            , -inf      , +inf      , 1e-0)         % 1e-1    
        falcon.State('z'            , -inf      ,   +0      , 1e-0)         % 1e-1    
    ];

    DefenderTranslationStates = [
        %            Name           , LowerBound, UpperBound, Scaling
        falcon.State('u'            , -inf      , +inf      , 1e-0)         % 1e-1    
        falcon.State('v'            , -inf      , +inf      , 1e-0)         % 1e-1    
        falcon.State('w'            , -inf      , +inf      , 1e-0)         % 1e-1    
    ];

    if DefenderOptions.SixDoF
        switch DefenderOptions.Attitude
            case 'Euler'
                DefenderAttitudeStates = [
                    %            Name           , LowerBound, UpperBound, Scaling
                    falcon.State('phi'          , -pi       , +pi       , 1e+0);
                    falcon.State('theta'        , -pi/2     , +pi/2     , 1e+0);
                    falcon.State('psi'          , -pi       , +pi       , 1e+0);
                ];
            case 'Quaternion'
                DefenderAttitudeStates = [
                    %            Name           , LowerBound, UpperBound, Scaling
                    error('Not implemented');  % TODO
                ];
            otherwise
                error('Unsupported attitude option: %s', DefenderOptions.Attitude);
        end
        
        DefenderRotationStates = [
            %            Name           , LowerBound, UpperBound, Scaling
            falcon.State('p'            , -4*pi     , +4*pi     , 1e+0)
            falcon.State('q'            , -4*pi     , +4*pi     , 1e+0)
            falcon.State('r'            , -4*pi    	, +4*pi    	, 1e+0)
        ];
    else
        DefenderAttitudeStates = falcon.State.empty();
        DefenderRotationStates = falcon.State.empty();
    end

    if DefenderOptions.MotorLag && DefenderOptions.SixDoF
        switch DefenderOptions.Type
            case 'Quad'
                DefenderMotorStates = [            
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
        DefenderMotorStates = falcon.State.empty();
    end
    
    % Invader states
    if InvaderOptions.SixDoF
        error('Invader 6DoF Model not implemented');
    else
        switch ModelOptions.Invader.Type
            case 'Quad1'
                InvaderStates = [
                    %            Name           , LowerBound, UpperBound, Scaling
                    falcon.State('x_inv'        , -inf      , +inf      , 1e-0) %1e+0
                    falcon.State('y_inv'        , -inf      , +inf      , 1e-0) %1e+0
                    falcon.State('z_inv'        , -inf      , +0        , 1e-0) %1e+0
                ];                
            case 'Quad2'
                InvaderStates = [
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
    
    States = [
        DefenderPositionStates
        DefenderTranslationStates
        InvaderStates
        DefenderAttitudeStates        
        DefenderRotationStates        
        DefenderMotorStates
        TimeState
    ];

    StateDerivativeNames = ...
        cellfun(@(x) strcat(x, '_dot'), {States.Name}, 'UniformOutput', false);

%% Controls

    % Defender control    
    switch DefenderOptions.Type
        case 'Quad'
            if DefenderOptions.SixDoF        
                if DefenderOptions.MotorLag
                    DefenderControls = [
                        %              Name         , LowerBound, UpperBound, Scaling
                        falcon.Control('w1_cmd'     , 0         , RPM_max   , 1/RPM_max)    % 1e-4    
                        falcon.Control('w2_cmd'     , 0         , RPM_max   , 1/RPM_max)    % 1e-4    
                        falcon.Control('w3_cmd'     , 0         , RPM_max   , 1/RPM_max)    % 1e-4    
                        falcon.Control('w4_cmd'     , 0         , RPM_max   , 1/RPM_max)    % 1e-4    
                    ];
                else
                    DefenderControls = [
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
                DefenderControls = [
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
    if InvaderOptions.SixDoF
        error('Invader 6DoF model not implemented');
    else
        switch InvaderOptions.Type
            case 'Quad1'
                InvaderControls = [
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
                InvaderControls = [
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
    switch ModelOptions.Optimize
        case 'def'
            Controls = DefenderControls;
        case 'inv'
            Controls = InvaderControls;
        otherwise
            error('Check setup.optimize')
    end

%% Parameters
    switch ModelOptions.Optimize
        case 'def'
            if ModelOptions.Uncertainty
                ModelParameters = [
                    falcon.DistrParameter('vIOO_x')
                    falcon.DistrParameter('vIOO_y')
                    falcon.DistrParameter('vIOO_z')
                    falcon.DistrParameter('T2W_max')
                    falcon.DistrParameter('rEscape')
                ];
            else
                ModelParameters = [
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
            ModelParameters = [
                falcon.Parameter('vI_abs_max')
                falcon.Parameter('T2W_max_inv')
            ];
        otherwise
            error('Check setup.optimize')
    end
    
    Parameters = [
        ModelParameters        
    ];                
    
%% Outputs

switch ModelOptions.Optimize
    case 'def'        
        % Defender acceleration for ObservabilityCostFcn 
        if ModelOptions.ObservabilityCostFcn
            OutputsObservability = [
                falcon.Output('u1')
                falcon.Output('u2')
                falcon.Output('u3')];        
        else
            OutputsObservability = falcon.Output.empty();
        end
        % Invader velocity
        OutputsInvader = [
            falcon.Output('u_inv_out')
            falcon.Output('v_inv_out')
            falcon.Output('w_inv_out')];
        
        Outputs = [
            OutputsInvader
            OutputsObservability];
        
        if isempty(Outputs)
            Outputs = falcon.Output('Void');
        end
        
    case 'inv'
        Outputs = falcon.Output.empty();
end

Variables = struct();
Variables.States = States;
Variables.StateDerivativeNames = StateDerivativeNames;
Variables.Controls = Controls;
Variables.Outputs = Outputs;
Variables.Parameters = Parameters;

end