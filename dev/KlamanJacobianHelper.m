
function [j, jac] = KlamanJacobianHelper()
%% Kalman derivative helper for cost function jacobian

% Sequence of derivation: [dj_dout; dj_dx; dj_du; dj_dt];
% 3DoF defender (no MotorLag)
clear j;clc;

% Parameter vector sizes
N   = 2;                                                                    % number of timesteps
n_x = 9;                                                                    % number of states
n_u = 3;                                                                    % number of controls
n_y = 8;                                                                    % number of outputs
n_t = 2;                                                                    % number of timeparameters
% syms dt real;                                                               % time step
dt = 1;

% DIP outputs, states, controls, timeparameter

% Outputs
% [u1 u2 u3 u_inv_out v_inv_out w_inv_out az_true el_true]
outputs = sym('y',[n_y,N],'real');

% States
% [x y z u v w x_inv y_inv z_inv]
states = sym('x',[n_x,1],'real');

% Controls
% [T_x T_y T_z]
controls = sym('u',[n_u,N],'real');

% Timeparameters
% [t0 tf]
% timepar = sym('t',[n_t,N],'real');

% Vector of all parameters required for derivation [1 x N]
% par = reshape([
%     outputs    
%     controls
%     ],1,[]);

% Filter sizes
n_x = 6;
n_u = 3;
n_z = 2;

% Filter states
% x_obs = [
%     states(7,:)     - states(1,:)
%     states(8,:)     - states(2,:)
%     states(9,:)     - states(3,:)
%     outputs(4,:)    - states(4,:)
%     outputs(5,:)    - states(5,:)
%     outputs(6,:)    - states(6,:)
% ];
x_obs = [
    states(1,:)
    states(2,:)
    states(3,:)
    states(7,:)
    states(8,:)
    states(9,:)
    ];

% Filter controls
u_obs = [
    outputs(1,:)
    outputs(2,:)
    outputs(3,:)
    ];
% Filter measurements
z_obs = [
    outputs(7,:)
    outputs(8,:)
    ];

% Initialize covarianc matrixes
Q_diag  = sym('Q_', [n_u,1], 'real');
R_diag  = sym('R_', [n_z,1], 'real');
Q  = diag(Q_diag);
R  = diag(R_diag);

% State jacobians
F_x = stateJac_x(dt);
F_w = stateJac_w(dt);

% Allocate filter variables
% x_k_km1 = sym('x_k_km1_', [n_x,1], 'real');
% P_k_km1 = sym('P_k_km1_', [n_x,n_x], 'real');
P_k_k   = sym('P_k_k_'  , [n_x,n_x], 'real');

% EKF step    
% Prediction step
k = 2;        
x_k_km1 =  stateFcn(x_obs(:,1),u_obs(:,k-1),dt);
P_k_km1 = F_x * P_k_k * F_x' + F_w * Q * F_w';
% Prediction measurement
% y_k_km1         = measFcn(x_k_km1,1);
% Gain
H       = measJac(x_k_km1);
K       = P_k_km1 * H' / (H * P_k_km1 * H' + R);
% Update step
% x_k_k(:)        = x_k_km1 + K * (z_obs(:,k) - y_k_km1);        
P_k_k   = (eye(n_x) - K * H) * P_k_km1;

% Cost function variables
P_trace_pos = trace(P_k_k(1:3,1:3));

% Cost function
j = sum(P_trace_pos);

jac = jacobian(j,[reshape([outputs;controls],1,[]),states']);

end








