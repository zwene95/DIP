% Debug Function for Interceptor_v04_model
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
u_inv = 0;
v_inv = 10;
w_inv =  0;

% Controls
T_x = 0;
T_y = 0;
T_z = 0;

states = [x; y; z; u; v; w; x_inv; y_inv; z_inv; u_inv; v_inv; w_inv;];
controls = [T_x; T_y; T_z]*0;
sol = DIP3DoF(states,controls)

PN_cmd = sol(4:6);
