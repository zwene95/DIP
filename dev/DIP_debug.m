% Debug Function for Interceptor_v04_model
% States
x     = 100;
y     = 0;
z     = -50;
phi   = 0;
theta = 0;
psi   = 0;
u     = 0;
v     = 0;
w     = 0;
p     = 0;
q     = 0;
r     = 0;

% Controls
w1    = 5000;
w2    = 1000;
w3    = 5000;
w4    = 1000;
states = [x; y; z; phi; theta; psi; u; v; w; p; q; r];
controls = [w1; w2; w3; w4]*0;
sol_v04 = Interceptor_v04_model(states,controls);
pos_dot_v04 = sol_v04(1:3);
att_dot_v04 = sol_v04(4:6);
vel_dot_v04 = sol_v04(7:9);
pqr_dot_v04 = sol_v04(10:12);