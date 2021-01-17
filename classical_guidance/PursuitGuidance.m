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
rTOO    = [20;-150;0];%setup.targetConfig.pTOO;
Setup.Scenario.pTOO = rTOO;
vD_abs  = 30;
vI_abs  = 15;

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
hit     = 0;
t_max   = 20;
x_sim   = x_0;
t       = 0;

% Run simulation
k = 1;
while ~((t(end)>t_max)||hit)
    
    x_k = x_sim(:,k);
    
    f1 = f(t        ,x_k              ,rTOO , vD_abs , vI_abs);
    f2 = f(t+0.5*dt , x_k + 0.5*dt*f1  ,rTOO , vD_abs , vI_abs);
    f3 = f(t+0.5*dt ,x_k + 0.5*dt*f2  ,rTOO , vD_abs , vI_abs);
    f4 = f(t+dt     ,x_k              ,rTOO , vD_abs , vI_abs);
    
    x_sim(:,k+1) = x_k + dt/6*(f1 + 2*f2 + 2*f3 + f4);
    
    % Hit critaria (miss distance below 1)
    norm(x_k(4:6)-x_k(1:3));
    if norm(x_k(4:6)-x_k(1:3))<1
        hit = 1;
    end
    
    % Increment time and step
    t(k+1) = t(k) + dt;
    k = k + 1;
end

Setup.Solver.GridSize = k;

Results.Time = t;
Results.Defender.States.Pos = x_sim(1:3,:);
Results.Defender.States.Vel = gradient(Results.Defender.States.Pos) /dt;
Results.Defender.States.Acc = gradient(Results.Defender.States.Vel) /dt;
Results.Invader.States.Pos  = x_sim(4:6,:);
Results.Invader.States.Vel  = gradient(Results.Invader.States.Pos) /dt;

%% Post processing
% Create path so save results
if Setup.PostOptions.Save
    % Save Jpg
    mkdir(Setup.PostOptions.Path);
    if Setup.PostOptions.Jpg
        Setup.PostOptions.PathJpg = [Setup.PostOptions.Path, 'JPG'];
        mkdir(Setup.PostOptions.PathJpg);
    end
    % Save Fig
    if Setup.PostOptions.Fig
        Setup.PostOptions.PathFig = [Setup.PostOptions.Path, 'FIG'];
        mkdir(Setup.PostOptions.PathFig);
    end
    
    % Save Setup and Results to mat file
    save([Setup.PostOptions.Path,'Setup.mat'], 'Setup');
    save([Setup.PostOptions.Path,'Results.mat'], 'Results');
    disp(['Results saved in ',Setup.PostOptions.Path]);
end

% Post process results
PostObservabilityAnalysis(Setup,Results);
Plot_Intercept_LOS(Setup,Results);
plotDef_vBOOabs(Setup, Results);
plotDef_aBOOabs(Setup, Results);

end

% Compute state derivatives
function x_dot = f(t,x,rTOO,vD_abs,vI_abs)

% State vector mapping
rDOO = x(1:3);
rIOO = x(4:6);
% Additional maneuvering
dV = [
    8 * sin(2*pi*t(end)/1)
    0 * sin(2*pi*t(end)/1)
    5 * sin(2*pi*t(end)/2)];
% Relative position defender invader
rDIO    = rIOO - rDOO;
rDIO_n  = rDIO/norm(rDIO);
vDOO    = rDIO_n * vD_abs;
% Relative position invader target
rITO    = rTOO - rIOO;
rITO_n  = rITO/norm(rITO);
vIOO = rITO_n * vI_abs;

% State vector dot
x_dot = [vDOO;vIOO];

end
