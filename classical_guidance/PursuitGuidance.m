function [Setup, Results] = PursuitGuidance(SetupSrc, varargin)
%PURSUITGUIDANCE Implemented Pursuit Guidance 
%   Pursuit guidance law implementaion for benchmarking observability

if nargin == 1
    % Rerun setup file
    Setup = SetupSrc;
else

    Setup.DefenderConfig    = defaultDefenderConfig;
    Setup.InvaderConfig     = defaultInvaderConfig;
    Setup.TargetConfig      = defaultTargetConfig;
    Setup.PostOptions       = defaultPostOptions('PursuitGuidance');
    Setup.ObserverConfig    = defaultObserverConfig;    
end

% Get parameters
rTOO    = [200;-150;0];%setup.targetConfig.pTOO;
Setup.Scenario.pTOO = rTOO;
vD_abs  = 30;
vI_abs  = 10;

% Get initial state vector
x_0 = [
    Setup.DefenderConfig.pDOO_0(1)
    Setup.DefenderConfig.pDOO_0(2)
    Setup.DefenderConfig.pDOO_0(3)
    Setup.InvaderConfig.pIOO_0(1)
    Setup.InvaderConfig.pIOO_0(2)
    Setup.InvaderConfig.pIOO_0(3)];

% Simulation time step
dt = 10e-03;

% Init simulation
hit = 0;
x_sim = x_0;
t = 0;

% Run simulation
k = 1;
while ~hit
    
    x_k = x_sim(:,k);
    
    f1 = f(x_k              ,rTOO , vD_abs , vI_abs);
    f2 = f(x_k + 0.5*dt*f1  ,rTOO , vD_abs , vI_abs);
    f3 = f(x_k + 0.5*dt*f2  ,rTOO , vD_abs , vI_abs);
    f4 = f(x_k              ,rTOO , vD_abs , vI_abs);
    
    x_sim(:,k+1) = x_k + dt/6*(f1 + 2*f2 + 2*f3 + f4);
    
    t(k+1) = t(k) + dt;
    k = k + 1;
    
    norm(x_k(4:6)-x_k(1:3));
    if norm(x_k(4:6)-x_k(1:3))<1
        hit = 1;
    end
    
end

Results.Time = t;
Results.Defender.States.Pos = x_sim(1:3,:);
Results.Defender.States.Vel = gradient(Results.Defender.States.Pos);
Results.Defender.States.Acc = gradient(Results.Defender.States.Vel);
Results.Invader.States.Pos  = x_sim(4:6,:);
Results.Invader.States.Vel  = gradient(Results.Invader.States.Pos);

%% Post processing
% Create path so save results
if Setup.PostOptions.Save
    mkdir(Setup.PostOptions.Path);
    if Setup.PostOptions.Jpg
        Setup.PostOptions.PathJpg = [Setup.PostOptions.Path, 'JPG'];
        mkdir(Setup.PostOptions.PathJpg);
    end
    if Setup.PostOptions.Fig
        Setup.PostOptions.PathFig = [Setup.PostOptions.Path, 'FIG'];
        mkdir(Setup.PostOptions.PathFig);
    end
end

% Post process results
PostObservabilityAnalysis(Setup,Results);
    
end

% Compute state derivatives
function x_dot = f(x,rTOO,vD_abs,vI_abs)
    % State vector mapping
    rDOO = x(1:3);
    rIOO = x(4:6);
    % Relative position defender invader
    rDIO    = rIOO - rDOO;
    rDIO_n  = rDIO/norm(rDIO);
    vDOO    = rDIO_n * vD_abs;
    % Relative position invader target
    rITO    = rTOO - rIOO;
    rITO_n  = rITO/norm(rITO);
    vIOO    = rITO_n * vI_abs;
    
    x_dot = [vDOO;vIOO];

end
