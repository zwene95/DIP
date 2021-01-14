function [costs] = LosRateCostFcn(outputs, states, controls, parameters) %#codegen
% cost function interface created by falcon.m

% Extract outputs
u_inv_out = outputs(1);
v_inv_out = outputs(2);
w_inv_out = outputs(3);

% Extract states
x         = states(1);
y         = states(2);
z         = states(3);
u         = states(4);
v         = states(5);
w         = states(6);
x_inv     = states(7);
y_inv     = states(8);
z_inv     = states(9);
phi       = states(10);
theta     = states(11);
psi       = states(12);
p         = states(13);
q         = states(14);
r         = states(15);

% Extract controls
w1        = controls(1);
w2        = controls(2);
w3        = controls(3);
w4        = controls(4);

% Extract parameters
Thresh = parameters(1);

% Relative defender invader kinematics
x_R = x_inv - x;
y_R = y_inv - y;
z_R = z_inv - z;
u_R = u_inv_out - u;
v_R = v_inv_out - v;
w_R = w_inv_out - w;
% Azimuth rate
AzimuthRate = (x_R * v_R - y_R * u_R); % / (x_R^2 + y_R^2 + sqrt(eps));
% Elevation rate
ElevationRate = (x_R * z_R * u_R + y_R * z_R * v_R - w_R * (x_R^2 + y_R^2));%...
%             / (sqrt(x_R^2 + y_R^2) * (x_R^2 + y_R^2 + z_R^2) + sqrt(eps));
        
costs = [AzimuthRate^2; ElevationRate^2];
% EoF

end