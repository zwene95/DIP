function [ ModelOptions ] = defaultModelOptions()

    ModelOptions = struct();        
    
    % General    
    ModelOptions.Optimize                   = 'def';
    ModelOptions.TimeState                  = false;        
    ModelOptions.Uncertainty                = false;     
    ModelOptions.ObservabilityCostFcn       = true;
    
    % Defender
    ModelOptions.Defender.Type              = 'Quad';
    ModelOptions.Defender.SixDoF            = true;
    ModelOptions.Defender.Attitude          = 'Euler';
    ModelOptions.Defender.MotorTable        = false;
    ModelOptions.Defender.MotorLag          = true;                       % true    
    ModelOptions.Defender.Aero              = true;                       % true   
    ModelOptions.Defender.m                 = 2;
    ModelOptions.Defender.RPM_max           = 1e4;
    
    % Invader
    ModelOptions.Invader.Type               = 'Quad1';                    % quadrotor order of control (1: velocity, 2: acceleration control)
    ModelOptions.Invader.SixDoF             = false;
    ModelOptions.Invader.Escape             = false;                      % false    
    
end
