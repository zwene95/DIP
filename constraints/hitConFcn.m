function [Constraints] = hitConFcn(Outputs, States, Controls) %#codegen
% constraint interface created by falcon.m

% Extract states
x     = States(1);
y     = States(2);
z     = States(3);
% u     = states(4);
% v     = states(5);
% w     = states(6);
x_inv = States(7);
y_inv = States(8);
z_inv = States(9);
% phi   = states(10);
% theta = states(11);
% psi   = states(12);
% p     = states(13);
% q     = states(14);
% r     = states(15);

% Extract controls
% w1    = controls(1);
% w2    = controls(2);
% w3    = controls(3);
% w4    = controls(4);

% ----------------------------- %
% implement the constraint function here %
% ----------------------------- %

% implement constraint values here
HitCon =   ((x_inv - x)^2 + ...
            (y_inv - y)^2 + ...
            (z_inv - z)^2);
        
Constraints = [HitCon];
% EoF
end