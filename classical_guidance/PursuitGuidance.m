function [setup, results] = PursuitGuidance(setup_src, varargin)
%PURSUITGUIDANCE Implemented Pursuit Guidance 
%   Pursuit guidance law implementaion for benchmarking observability

if nargin == 1
    % Rerun setup file
    setup = setup_src;
else

    setup.defenderConfig    = defaultDefenderConfig;
    setup.invaderConfig     = defaultInvaderConfig;
    setup.targetConfig      = defaultTargetConfig;
    setup.postOptions       = defaultPostOptions('PursuitGuidance');
    setup.observerConfig    = defaultObserverConfig;    
end

% Get parameters
rTOO    = [200;-150;0];%setup.targetConfig.pTOO;
setup.scenario.pTOO = rTOO;
vD_abs  = 30;
vI_abs  = 10;

% Get initial values
x_0 = [
    setup.defenderConfig.pDOO_0(1)
    setup.defenderConfig.pDOO_0(2)
    setup.defenderConfig.pDOO_0(3)
    setup.invaderConfig.pIOO_0(1)
    setup.invaderConfig.pIOO_0(2)
    setup.invaderConfig.pIOO_0(3)];

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
    f3 = f(x_k + 0.5*dt*f2              ,rTOO , vD_abs , vI_abs);
    f4 = f(x_k              ,rTOO , vD_abs , vI_abs);
    
    x_sim(:,k+1) = x_k + dt/6*(f1 + 2*f2 + 2*f3 + f4);
    
    t(k+1) = t(k) + dt;
    k = k + 1;
    
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

% Post process results
PostObservabilityAnalysis(setup,results);
    
end

% Compute state derivatives
function x_dot = f(x,rTOO,vD_abs,vI_abs)
    rDOO = x(1:3);
    rIOO = x(4:6);    
    rDIO    = rIOO - rDOO;
    rDIO_n  = rDIO/norm(rDIO);
    vDOO    = rDIO_n * vD_abs;
    
    rITO    = rTOO - rIOO;
    rITO_n  = rITO/norm(rITO);
    vIOO    = rITO_n * vI_abs; 
    
    x_dot = [vDOO;vIOO];

end
