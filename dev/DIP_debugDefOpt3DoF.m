% Debug Function
% States
x     = 0;
y     = 0;
z     = 0;
u     = 0;
v     = 0;
w     = 0;
x_inv = 100;
y_inv = 100;
z_inv = -100;

% Controls
T_x = 0;
T_y = 0;
T_z = 0;

% Parameters
x_tgt = 0;
y_tgt = 0;
z_tgt = 0;
vI_abs_max = 10;
T2W_max = 3;

states = [x; y; z; u; v; w; x_inv; y_inv; z_inv];
controls = [T_x; T_y; T_z]*0;
parameters = [x_tgt; y_tgt; z_tgt; vI_abs_max; T2W_max];
sol = DIP3DoF_DefOpt(states,controls,parameters)
vIOO = sol(7:9)
