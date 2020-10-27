%% Kalman derivative helper for cost function jacobian

% Sequence of derivation: [dj_dout; dj_dx; dj_du; dj_dt];
% 3DoF defender (no MotorLag)
clear;clc;

% Parameter vector sizes
N   = 2;                                                                    % number of timesteps
n_x = 9;                                                                    % number of states
n_u = 3;                                                                    % number of controls
n_y = 8;                                                                    % number of outputs
n_t = 2;                                                                    % number of timeparameters
syms dt real;                                                               % time step
% dt = 1;

% DIP outputs, states, controls, timeparameter

% Outputs 
% [u1 u2 u3 u_inv_out v_inv_out w_inv_out az_true el_true]
outputs = sym('y',[n_y,N],'real');

% States
% [x y z u v w x_inv y_inv z_inv]
states = sym('x',[n_x,N],'real');

% Controls
% [T_x T_y T_z]
controls = sym('u',[n_u,N],'real');

% Timeparameters
% [t0 tf]
timepar = sym('t',[n_t,N],'real');

% Vector of all parameters required for derivation
par = [    
    outputs
    states
    controls
    timepar    
];

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
    states(4,:)     
    states(5,:)     
    states(6,:)    
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
P0_diag = sym('P0_',[n_x,1],'real');
P0 = diag(P0_diag);
Q   = sym('Q_', [n_u,n_u], 'real');
R   = sym('R_', [n_z,n_z], 'real');

% State jacobians
F_x = stateJac_x(dt);
F_w = stateJac_w(dt);

% Allocate filter variables
x_k_km1 = sym('x_k_km1_', [n_x,N], 'real');
x_k_k   = sym('x_k_k_'  , [n_x,N], 'real');
P_k_km1 = sym('P_k_km1_', [n_x,n_x,N], 'real'); 
P_k_k   = sym('P_k_k_'  , [n_x,n_x,N], 'real'); 
% Initialize filter variables
x_k_km1(:,1)    = x_obs(:,1);
P_k_km1(:,:,1)  = P0;
x_k_k(:,1)      = x_obs(:,1);
P_k_k(:,:,1)    = P0;

% Allocate cost function variables
P_trace_pos = sym('P_tr_pos_', [1,N], 'real');
% Initialize cost function variables
P_trace_pos(1) = trace(P0(1:3,1:3));

% EKF
for k=2:N
    % Prediction step
    x_k_km1(:,k)   = stateFcn(x_k_k(:,k-1),u_obs(:,k-1),dt);
    P_k_km1(:,:,k) = F_x * P_k_k(:,:,k-1) * F_x' ;% + F_w * Q * F_w';
    % Prediction measurement
    y_k_km1 = measFcn(x_k_km1(:,k),1);
    % Gain
    H   =   measJac(x_k_km1(:,k));
%     K1 = P_k_km1(:,:,k) * H';
%     K2 = (H * P_k_km1(:,:,k) * H' + R);
%     K = K1 / K2;
%     K   =   P_k_km1(1,:,k) * H(1,:)' / (H * P_k_km1(:,:,k) * H' + R);
    K   =   P_k_km1(:,:,k) * H' / (H * P_k_km1(:,:,k) * H');% + R);
    % Update step
    x_k_k(:,k)      =   x_k_km1(:,k) + K * (z_obs(:,k) - y_k_km1);
    P_k_k(:,:,k)    =   (eye(n_x) - K * H) * P_k_km1(:,:,k);    
    
    % Cost function variables
    P_trace_pos(k) = trace(P_k_k(1:3,1:3,k));
    
end

% Cost function
j = sum(P_trace_pos);








