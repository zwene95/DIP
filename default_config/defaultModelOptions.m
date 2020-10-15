function [ modelOptions ] = defaultModelOptions()

    modelOptions = struct();        
    
    % General
    
    modelOptions.optimize                           = 'def';
    modelOptions.timeState                          = false;        
    modelOptions.uncertainty                        = false;   
    
    % Defender
    modelOptions.defender.Type                      = 'Quad';
    modelOptions.defender.SixDoF                    = true;
    modelOptions.defender.Attitude                  = 'Euler';
    modelOptions.defender.MotorTable                = false;
    modelOptions.defender.MotorLag                  = true;                       % true    
    modelOptions.defender.Aero                      = true;                       % true   
    modelOptions.defender.m                         = 2;
    modelOptions.defender.RPM_max                   = 10000;
    
    % Invader
    modelOptions.invader.Type                       = 'Quad1';                    % quadrotor order of control (1: velocity, 2: acceleration control)
    modelOptions.invader.SixDoF                     = false;
    modelOptions.invader.Escape                     = false;                      % false    
    
    % Target
    modelOptions.target.targetConstraint            = true;                     % target violation constraint
    
    % Observer
    modelOptions.observer                           = false;
    
    
    modelOptions.modelName                          = ;
        
end
