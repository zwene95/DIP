function [ modelOptions ] = defaultModelOptions()

    modelOptions = struct();        
    
    % General    
    modelOptions.optimize                   = 'def';
    modelOptions.timeState                  = false;        
    modelOptions.uncertainty                = false; 
    modelOptions.observer                   = false;
    modelOptions.observabilityCost          = true;
    
    % Defender
    modelOptions.defender.Type              = 'Quad';
    modelOptions.defender.SixDoF            = true;
    modelOptions.defender.Attitude          = 'Euler';
    modelOptions.defender.MotorTable        = false;
    modelOptions.defender.MotorLag          = true;                       % true    
    modelOptions.defender.Aero              = true;                       % true   
    modelOptions.defender.m                 = 2;
    modelOptions.defender.RPM_max           = 1e4;
    
    % Invader
    modelOptions.invader.Type               = 'Quad1';                    % quadrotor order of control (1: velocity, 2: acceleration control)
    modelOptions.invader.SixDoF             = false;
    modelOptions.invader.Escape             = false;                      % false    
    
end
