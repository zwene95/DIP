function [ builder ] = defaultModelConstants(builder, modelOptions)
% Adds constants to given model    

    %% Environment
    builder.addConstant('g', 9.80665);
    builder.addConstant('rho', 1.225);


    %% Attitude
    switch modelOptions.defender.Attitude
    case 'Euler'
    case 'Quaternion'
        builder.addConstant('k_quat_normalization', 1e-6);
    otherwise
        error('Unsupported attitude option: %s', modelOptions.defender.Attitude);
    end

    %% Defender

    % Airframe configuration
    builder.addConstant('m'     , modelOptions.defender.m);                % kg
    builder.addConstant('Inertia', diag([0.019, 0.019, 0.03]));           % kg*m^2  default: diag([0.0187, 0.0144, 0.03]
    builder.addConstant('d', 0.25);                                         % m
    L_lever = 0.15;
    builder.addConstant('lever', L_lever);                                  % m
    builder.addConstant('r1', [1; 1; 0] * L_lever/sqrt(2));                 % m
    builder.addConstant('r2', [-1; 1; 0] * L_lever/sqrt(2));                % m
    builder.addConstant('r3', [-1; -1; 0] * L_lever/sqrt(2));               % m
    builder.addConstant('r4', [1; -1; 0] * L_lever/sqrt(2));                % m
    
    % Propulsion characteristics
%     builder.addConstant('cf', 18e-8);                                     % N/rpm^2
    builder.addConstant('RPM_max', modelOptions.defender.RPM_max);          % rpm
    builder.addConstant('cm', 35e-10);                                      % N/rpm^2

    % Aerodynamics 
    builder.addConstant('cD',   0.1);                                       % C_D: Drag    

    %% Invader
    builder.addConstant('mI' , 2);                                          % kg
    
    %% Observer
    % State jacobian for x
    builder.addConstant('F_x',[ 0 , 0 , 0 , 1 , 0 , 0
                                0 , 0 , 0 , 0 , 1 , 0
                                0 , 0 , 0 , 0 , 0 , 1
                                0 , 0 , 0 , 0 , 0 , 0
                                0 , 0 , 0 , 0 , 0 , 0
                                0 , 0 , 0 , 0 , 0 , 0]);
    % State jacobian for w
    builder.addConstant('F_w',[ 0 , 0 , 0 
                                0 , 0 , 0 
                                0 , 0 , 0 
                                1 , 0 , 0 
                                0 , 1 , 0 
                                0 , 0 , 1]);
                            
    builder.addConstant('spr', 1e-4);                                       % Singularity prevention term for jacobians  (1e-4)

end