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
x_inv = 0;
y_inv = 0;
z_inv = 0;
w1    = 0;
w2    = 0;
w3    = 0;
w4    = 0;
% Controls
w1_cmd    = 5000;
w2_cmd    = 1000;
w3_cmd    = 5000;
w4_cmd    = 1000;
% Parameters
vT_abs_max = 30;
x_tgt_f    = 0;
y_tgt_f    = 0;
z_tgt_f    = 0;

states = [x; y; z; phi; theta; psi; u; v; w; p; q; r; x_inv; y_inv; z_inv; w1; w2; w3; w4];
controls = [w1_cmd; w2_cmd; w3_cmd; w4_cmd];
parameters = [vT_abs_max; x_tgt_f; y_tgt_f; z_tgt_f];
sol_v04 = DIP_6DoF_DefOpt_stop(states,controls,parameters);
pos_dot_v04 = sol_v04(1:3);
att_dot_v04 = sol_v04(4:6);
vel_dot_v04 = sol_v04(7:9);
pqr_dot_v04 = sol_v04(10:12);
w_dot_v04   = sol_v04(16:19);