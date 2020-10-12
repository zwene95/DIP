
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
        falcon.State('z'            , -inf      ,    0      , 1e-0)         % 1e-1    
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
            falcon.State('p'            , -10      , +10        , 1e+0)
            falcon.State('q'            , -10      , +10        , 1e+0)
            falcon.State('r'            , -10      , +10        , 1e+0)
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
                    falcon.State('z_inv'        , -inf      , +inf      , 1e-0) %1e+0
                ];                
            case 'Quad2'
                invaderStates = [
                    %            Name           , LowerBound, UpperBound, Scaling
                    falcon.State('x_inv'        , -inf      , +inf      , 1e-0)     % 1e-1    
                    falcon.State('y_inv'        , -inf      , +inf      , 1e-0)     % 1e-1    
                    falcon.State('z_inv'        , -inf      , +inf      , 1e-0)     % 1e-1    
                    falcon.State('u_inv'        , -inf      , +inf      , 1e-0)     % 1e-1    
                    falcon.State('v_inv'        , -inf      , +inf      , 1e-0)     % 1e-1    
                    falcon.State('w_inv'        , -inf      , +inf      , 1e-0)     % 1e-1    
                ];
        end
    end
    
    % Observer states
    if modelOptions.observer
        observerStates = [
             %            Name          , LowerBound, UpperBound, Scaling
             % States
            falcon.State('x_est'        , -inf      , +inf      , 1e-0)     % 
            falcon.State('y_est'        , -inf      , +inf      , 1e-0)     % 
            falcon.State('z_est'        , -inf      , +inf      , 1e-0)     % 
            falcon.State('u_est'        , -inf      , +inf      , 1e-0)     % 
            falcon.State('v_est'        , -inf      , +inf      , 1e-0)     % 
            falcon.State('w_est'        , -inf      , +inf      , 1e-0)     %   
            % Covariance
            falcon.State('P_11'         , -inf         , +inf      , 1e-2)     % 
            falcon.State('P_12'         , -inf         , +inf      , 1e-2)     % 
            falcon.State('P_13'         , -inf         , +inf      , 1e-2)     % 
            falcon.State('P_14'         , -inf         , +inf      , 1e-2)     % 
            falcon.State('P_15'         , -inf         , +inf      , 1e-2)     % 
            falcon.State('P_16'         , -inf         , +inf      , 1e-2)     %
%             falcon.State('P_21'         , 0         , +inf      , 1e-0)     % 
            falcon.State('P_22'         , -inf         , +inf      , 1e-2)     %
            falcon.State('P_23'         , -inf         , +inf      , 1e-2)     %
            falcon.State('P_24'         , -inf         , +inf      , 1e-2)     %
            falcon.State('P_25'         , -inf         , +inf      , 1e-2)     %
            falcon.State('P_26'         , -inf         , +inf      , 1e-2)     %
%             falcon.State('P_31'         , 0         , +inf      , 1e-0)     % 
%             falcon.State('P_32'         , 0         , +inf      , 1e-0)     %
            falcon.State('P_33'         , -inf         , +inf      , 1e-2)     %
            falcon.State('P_34'         , -inf         , +inf      , 1e-2)     %
            falcon.State('P_35'         , -inf         , +inf      , 1e-2)     %
            falcon.State('P_36'         , -inf         , +inf      , 1e-2)     %
%             falcon.State('P_41'         , 0         , +inf      , 1e-0)     % 
%             falcon.State('P_42'         , 0         , +inf      , 1e-0)     %
%             falcon.State('P_43'         , 0         , +inf      , 1e-0)     %
            falcon.State('P_44'         , -inf         , +inf      , 1e-2)     %
            falcon.State('P_45'         , -inf         , +inf      , 1e-2)     %
            falcon.State('P_46'         , -inf         , +inf      , 1e-2)     %
%             falcon.State('P_51'         , 0         , +inf      , 1e-0)     % 
%             falcon.State('P_52'         , 0         , +inf      , 1e-0)     %
%             falcon.State('P_53'         , 0         , +inf      , 1e-0)     %
%             falcon.State('P_54'         , 0         , +inf      , 1e-0)     %
            falcon.State('P_55'         , -inf         , +inf      , 1e-2)     %
            falcon.State('P_56'         , -inf         , +inf      , 1e-2)     %
%             falcon.State('P_61'         , 0         , +inf      , 1e-0)     % 
% %             falcon.State('P_62'         , 0         , +inf      , 1e-0)     %
%             falcon.State('P_63'         , 0         , +inf      , 1e-0)     %
%             falcon.State('P_64'         , 0         , +inf      , 1e-0)     %
%             falcon.State('P_65'         , 0         , +inf      , 1e-0)     %
            falcon.State('P_66'         , -inf         , +inf      , 1e-2)     %
        ];
    else
        observerStates = falcon.State.empty();
    
    end
    
    states = [
        defenderPositionStates
        defenderTranslationStates
        invaderStates
        defenderAttitudeStates        
        defenderRotationStates        
        defenderMotorStates
        timeState
        observerStates
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
    
    if modelOptions.observer
        observerParameters = [                 
            falcon.Parameter('Q_11', 'fixed', true)
            falcon.Parameter('Q_12')
            falcon.Parameter('Q_13')
            falcon.Parameter('Q_21')
            falcon.Parameter('Q_22', 'fixed', true)
            falcon.Parameter('Q_23')
            falcon.Parameter('Q_31')
            falcon.Parameter('Q_32')
            falcon.Parameter('Q_33', 'fixed', true)
            falcon.Parameter('R_11', 'fixed', true)
            falcon.Parameter('R_12')
            falcon.Parameter('R_21')
            falcon.Parameter('R_22', 'fixed', true)
        ];
    else
        observerParameters = falcon.Parameter.empty();
    end
    
    parameters = [
        modelParameters
        observerParameters
    ];
    
            
    
    %% Outputs
    
    switch modelOptions.optimize
        case 'def'
            if defenderOptions.Aero
                outputs = [
%                     %             Name    
                    falcon.Output('FDAD_x')
                    falcon.Output('FDAD_y')
                    falcon.Output('FDAD_z')
                ];
            else                
                outputs = falcon.Output('Void');
%                 outputs = falcon.Output.empty();    
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