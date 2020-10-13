% Debug Function 

% States
x           = 0;
y           = 0;
z           = 0;
u           = 0;
v           = 0;
w           = 0;
x_inv       = 0;
y_inv       = 0;
z_inv       = 0;
phi         = 0;
theta       = 0;
psi         = 0;
p           = 0;
q           = 0;
r           = 0;
w1          = 5e3;
w2          = 5e3;
w3          = 5e3;
w4          = 5e3;
x_est       = 0;
y_est       = 0;
z_est       = 0;
u_est       = 0;
v_est       = 0;
w_est       = 0;
P           = diag([1e2 1e2 1e2 1e2 1e2 1e2]);
P_vec       = reshape(P,[numel(P) 1]);
states      = [x; y; z; u; v; w; x_inv; y_inv; z_inv; phi; theta; psi; p; q; r; w1; w2; w3; w4; x_est; y_est; z_est; u_est; v_est; w_est; P_vec];

% Controls
w1_cmd      = 5000;
w2_cmd      = 5000;
w3_cmd      = 5000;
w4_cmd      = 5000;
controls    = [w1_cmd; w2_cmd; w3_cmd; w4_cmd];

% Parameters
pTOO_x      = 0;
pTOO_y      = 10;
pTOO_z      = -50;
vTOO_x      = 5;
vTOO_y      = -20;
vTOO_z      = -15;
vI_abs_max  = 20;
T2W_max     = 2;
MotorTC     = 2e-2;
rEscape     = 0;
Q           = diag([1e2 1e2 1e2]);
Q_vec       = reshape(Q,[numel(Q) 1]);
R           = diag([1e-2 1e-2]);
R_vec       = reshape(R,[numel(R) 1]);
parameters  = [pTOO_x; pTOO_y; pTOO_z; vTOO_x; vTOO_y; vTOO_z; vI_abs_max; T2W_max; MotorTC; rEscape; Q_vec; R_vec];    

% Solve
sol         = Modelx_est(states,controls,parameters);
pos_dot     = sol(1:3);
vel_dot     = sol(4:6);
inv_pos_dot = sol(7:9);
att_dot     = sol(10:12);
pqr_dot     = sol(13:15);
w_dot_v04   = sol(16:19);
pos_est_dot = sol(20:25)
cov_dot     = sol(26:61);