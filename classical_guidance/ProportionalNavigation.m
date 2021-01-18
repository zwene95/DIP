function [Setup, Results] = ProportionalNavigation(SetupSrc, varargin)
%ProportionalNavigation Implemented Poportional Navigation Guidance 
%   Poportional Navigation Guidance implementaion for benchmarking observability

if nargin == 1
    % Rerun setup file
    Setup = SetupSrc;
else
    Setup.DefenderConfig    = defaultDefenderConfig;
    Setup.InvaderConfig     = defaultInvaderConfig;
    Setup.TargetConfig      = defaultTargetConfig;
    Setup.PostOptions       = defaultPostOptions('PN');
    Setup.ObserverConfig    = defaultObserverConfig;    
end

% Get parameters
rTOO = Setup.TargetConfig.pTOO;
Setup.Scenario.pTOO = rTOO; % required for post processing
vD_abs = 30;
vI_abs = Setup.InvaderConfig.vI_abs_max;

% Define proportional navigation Navigation constant N
N = 20;

% Get initial positions
x0_pos = [
    Setup.DefenderConfig.pDOO_0(1)
    Setup.DefenderConfig.pDOO_0(2)
    Setup.DefenderConfig.pDOO_0(3)    
    Setup.InvaderConfig.pIOO_0(1)
    Setup.InvaderConfig.pIOO_0(2)
    Setup.InvaderConfig.pIOO_0(3)
    ];

% Compute initial velocities
% Defender
rDOO_0  = x0_pos(1:3,:);
rIOO_0  = x0_pos(4:6,:);
rDIO    = rIOO_0 - rDOO_0;
rDIO_n  = rDIO/norm(rDIO);
vDOO_0  = rDIO_n * vD_abs;
% Invader
rITO    = rTOO - rIOO_0;
rITO_n  = rITO/norm(rITO);
vIOO_0  = rITO_n * vI_abs;

x0_vel = [vDOO_0;vIOO_0];

% Initial state vector
x0 = [x0_pos;x0_vel];

% Simulation time step
dt = 10e-03;

% Init simulation
hit = 0;
x_sim = x0;
t = 0;

% Run simulation
k = 1;
% ODE4 step
while ~hit
    
    x_k = x_sim(:,k);
    
    f1 = f(x_k              ,N);
    f2 = f(x_k + 0.5*dt*f1  ,N);
    f3 = f(x_k + 0.5*dt*f2  ,N);
    f4 = f(x_k              ,N);
    
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
Results.Defender.States.Vel = gradient(Results.Defender.States.Pos)/dt;
Results.Defender.States.Acc = gradient(Results.Defender.States.Vel)/dt;
Results.Invader.States.Pos  = x_sim(4:6,:);
Results.Invader.States.Vel  = gradient(Results.Invader.States.Pos)/dt;


%%  Post processing
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

PostObservabilityAnalysis(Setup,Results);
Plot_Intercept_LOS(Setup,Results);
plotDef_vBOOabs(Setup, Results);
plotDef_aBOOabs(Setup, Results);
    
end

function x_dot = f(x,N)
% PPN Guidance Law
    
    % State vector mapping
    rDOO = x(1:3);
    rIOO = x(4:6); 
    vDOO = x(7:9);
    vIOO = x(10:12);
    
    % Relative states
    rDIO = rIOO - rDOO;    
    vDIO = vIOO - vDOO;
    
    % Position propagation
    x_dot(1:3,:) = vDOO;
    x_dot(4:6,:) = vIOO;
    
    % LOS rate
    OmegaLOS = cross(rDIO,vDIO)/(rDIO'*rDIO);    
%     rDIO_n   = rDIO/norm(rDIO);
    
    % Compute defender acceleration
    aDOO = N * cross(OmegaLOS,vDOO);
    
    % Velocity propagation
    x_dot(7:9,:)    = aDOO;
    x_dot(10:12,:)  = zeros(3,1);

end
