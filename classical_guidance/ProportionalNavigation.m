function [setup, results] = ProportionalNavigation(setup_src, varargin)
%ProportionalNavigation Implemented Poportional Navigation Guidance 
%   Poportional Navigation Guidance implementaion for benchmarking observability

if nargin == 1
    % Rerun setup file
    setup = setup_src;
else
    setup.defenderConfig    = defaultDefenderConfig;
    setup.invaderConfig     = defaultInvaderConfig;
    setup.targetConfig      = defaultTargetConfig;
    setup.postOptions       = defaultPostOptions('PN');
    setup.observerConfig    = defaultObserverConfig;    
end

% Get parameters
rTOO_0    = [20;-150;0];%setup.targetConfig.pTOO;
setup.scenario.pTOO = rTOO_0; % required for post processing
vD_abs  = 30;
vI_abs  = 10;

% Define proportional navigation Navigation constant N
N = 20;

% Get initial positions
x0_pos = [
    setup.defenderConfig.pDOO_0(1)
    setup.defenderConfig.pDOO_0(2)
    setup.defenderConfig.pDOO_0(3)    
    setup.invaderConfig.pIOO_0(1)
    setup.invaderConfig.pIOO_0(2)
    setup.invaderConfig.pIOO_0(3)
    ];

% Compute initial velocities
% Defender
rDOO_0  = x0_pos(1:3,:);
rIOO_0  = x0_pos(4:6,:);
rDIO    = rIOO_0 - rDOO_0;
rDIO_n  = rDIO/norm(rDIO);
vDOO_0  = rDIO_n * vD_abs;
% Invader
rITO    = rTOO_0 - rIOO_0;
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
    
    t(k+1) = t(k) + dt;
    k = k + 1;
    
    % Hit critaria (miss distance below 1)
    norm(x_k(4:6)-x_k(1:3));
    if norm(x_k(4:6)-x_k(1:3))<1
        hit = 1;
    end
    
end

results.time = t;
results.defender.states.pos = x_sim(1:3,:);
results.defender.states.vel = gradient(results.defender.states.pos);
results.defender.states.acc = gradient(results.defender.states.vel);
results.invader.states.pos  = x_sim(4:6,:);
results.invader.states.vel  = gradient(results.invader.states.pos);


%%  Post processing
% Create path so save results
if setup.postOptions.Save
    mkdir(setup.postOptions.Path);
    if setup.postOptions.Jpg
        setup.postOptions.PathJpg = [setup.postOptions.Path, 'JPG'];
        mkdir(setup.postOptions.PathJpg);
    end
    if setup.postOptions.Fig
        setup.postOptions.PathFig = [setup.postOptions.Path, 'FIG'];
        mkdir(setup.postOptions.PathFig);
    end
end

PostObservabilityAnalysis(setup,results);
% Plot_Intercept_LOS(setup, problem, c);
    
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
