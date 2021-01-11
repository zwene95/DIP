function [SimulatedStates] = DIP_Simulate(setup, problem, results)
%DIP_SIMULATE simulates the results of the DIP optimization
%   ODE4 Runge Kutta based simulation of states


% Get model name
f = str2func(setup.modelName);
% Extract time t, states x, controls u, parameters p and constants c
t   = problem.RealTime;
x   = problem.StateValues;
u   = problem.ControlValues;
p   = [problem.Parameters(3:end).Value]';
c   = problem.Phases(1).Model.ModelConstants{:};

x_sim = nan(size(x));
x_sim(:,1) = x(:,1);
% Numerically integration (ODE4)
dt = diff(t(1:2));
for k=1:(length(x)-1)
    
    x_k     = x_sim(:,k);
    u_k     = u(:,k);
    u_kp1   = u(:,k+1);
    
    f1 = f(x_k              , u_k               , p, c);
    f2 = f(x_k + 0.5*dt*f1  , 0.5*(u_k + u_kp1) , p, c);
    f3 = f(x_k + 0.5*dt*f2  , 0.5*(u_k + u_kp1) , p, c);
    f4 = f(x_k + dt*f3      , u_kp1             , p, c);
    
    x_sim(:,k+1) = x_k + dt/6*(f1 + 2*f2 + 2*f3 + f4);

end

SimulatedStates = x_sim;



end