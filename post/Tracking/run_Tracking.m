%% Target Tracking / Observability Analysis

%% Pre Processing
% Load Trajectory
clc; clear f;
%     load('D:\GoogleDrive\UNI\Master\Masterarbeit\DIP_git\Results\preRestart\Test3_obs\results.mat');
%     load('D:\GoogleDrive\UNI\Master\Masterarbeit\DIP_git\Results\preRestart\Test_est2\results.mat');
%     load('D:\GoogleDrive\UNI\Master\Masterarbeit\DIP_git\Results\preRestart\PN\results.mat');

load('D:\GoogleDrive\UNI\Master\Masterarbeit\DIP_git\Results\pre_ObsvObject\results.mat');
% load('D:\GoogleDrive\UNI\Master\Masterarbeit\DIP_git\Results\pre_ObsvObject_stat\results.mat');


%% Constants
c = struct();
c.Interpreter   = 'latex';
c.FS_title      = 14;
c.FS_Legend     = 12;

%% Pre Processing Results
nDat    = length(results.time) - 0;
iDat    = linspace(1,nDat,nDat);
dtFil  = 10e-3; 
nFil    = ceil(results.time(end)/dtFil);
iFil    = linspace(1,nDat,nFil);
time    = results.time(1:nDat);


%   State dynamics: x_dot = A*x + B*u + w(t), x[6x1], u [3x1], A [6x3], B [6x3]

%   Get states, structure as follows: x = [x y z u v w];
x_true = [
    results.invader.states.pos(1,:) - results.defender.states.pos(1,:)
    results.invader.states.pos(2,:) - results.defender.states.pos(2,:)
    results.invader.states.pos(3,:) - results.defender.states.pos(3,:)
    results.invader.states.vel(1,:) - results.defender.states.vel(1,:)
    results.invader.states.vel(2,:) - results.defender.states.vel(2,:)
    results.invader.states.vel(3,:) - results.defender.states.vel(3,:)
    ];

% Get pseudo-controls [ax ay az]
u_true  = results.defender.states.acc(:,:);

% Process noise
mu_w    =  0;   % mean
Sigma_w =  0;   % standard deviation (10)
rng(2019);
w = normrnd(mu_w,Sigma_w,size(u_true));

u = u_true + w;

% Measurements z = h(x(t)) + v(t);
z_true = [
    results.LOS.azimuth(:,:)
    results.LOS.elevation(:,:)
    ];

myEKF               = EKF_Object;
myEKF.Time          = results.time;
myEKF.Controls      = u_true;
myEKF.Interpolate   = true;
myEKF.Measurements  = z_true;
myEKF.Sigma_P0_pos  = 10;
myEKF.Sigma_P0_vel  = sqrt(5e2);
myEKF.Sigma_Q       = sqrt(1e2);
myEKF.Sigma_R       = sqrt(1e-2);
myEKF.Sigma_v       = 0e-02;
myEKF.Sigma_w       = 0;
myEKF.Sigma_x0      = 10;
myEKF.States        = x_true;
myEKF.StepTime      = dtFil;

% Measurement noise
mu_v    =  0;       % mean
Sigma_v =  0e-2;    % standard deviation (2e-2)
rng(2020);
v = normrnd(mu_v,Sigma_v,size(z_true));
z = z_true + v;

% Intercpolate Data
if ~isequal(nDat,nFil)
    time    = interp1(iDat,time,iFil);
%     time    = interp1(iDat,time,iFil,'pchip',time(end));
    x_true  = interp1(iDat,x_true',iFil)';    
    z_true  = interp1(iDat,z_true',iFil)';
    u       = interp1(iDat,u',iFil)';
    z       = interp1(iDat,z',iFil)';
end
% Time differences
dt = diff(time(1:2));

%% EKF init
% State jacobians (linear time invariant)
F_x = stateJac_x(dt);
F_w = stateJac_w(dt);

% Initial bias
mu_x0           = 0;
sigma_x0_pos    = 10;   %10
% std_x0_vel  = 10;   %10
rng(9999);
% rng shuffle;
b_x0_pos    = normrnd(mu_x0, sigma_x0_pos, [3 1]);
% b_x0_vel    = normrnd(mu_x0, std_x0_vel, [3 1]);
% b_x0        = [b_x0_pos; b_x0_vel];
b_x0        = [b_x0_pos; -x_true(4:6,1)] * 1;


% Allocate state and state covariance matrix
x_0     = 	x_true(:,1) + b_x0;                                             % [eye(3),zeros(3); zeros(3,6)]
P_0     =   diag([1e2,1e2,1e2,5e2,5e2,5e2]);                                % diag([1e2,1e2,1e2,5e2,5e2,5e2])
n_x     =   length(x_0);
n_y     =	length(measFcn(x_0,1));
% Setup variables for states and state covariance matrix
x_k_km1 =   nan(n_x,nFil);
x_k_k   =   nan(n_x,nFil);
P_k_km1 =   nan(n_x,n_x,nFil);
P_k_k   =   nan(n_x,n_x,nFil);
x_k_km1_ode4 =   nan(n_x,nFil);                                                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initialize variables for states and state covariance matrix
x_k_km1(:,1)    = x_0;
x_k_km1_ode4(:,1)    = x_0;                                                 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
P_k_km1(:,:,1)  = P_0;
x_k_k(:,1)      = x_0;
P_k_k(:,:,1)    = P_0;

% Process noise variance matrix
Q = diag([1e2, 1e2, 1e2]);
% Measurement noise variance matrix
R = diag([1e-2 1e-2]);
% R = 1e-2;                                                      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Setup variables for post processing
MeasurementsFil =   nan(2,nFil);
Sigma           =   nan(n_x,nFil);
ErrorFil        =   nan(n_x,nFil);
NEES            =   nan(1,nFil);                                               % Normalized Estimation Error Squared combined
NEES_pos        =   nan(1,nFil);                                               % Normalized Estimation Error Squared position only
NEES_vel        =   nan(1,nFil);                                               % Normalized Estimation Error Squared velocity only
P_trace         =   nan(1,nFil);
P_trace_pos     =   nan(1,nFil);
P_trace_vel     =   nan(1,nFil);
K_norm          =   nan(1,nFil-1);
P_norm          =   nan(1,nFil-1);
H_norm          =   nan(1,nFil-1);
I_norm          =   nan(1,nFil-1);
dz_vec          =   nan(2,nFil-1);
K_vec           =   nan(n_x,n_y,nFil-1);
H_linPseudo     =   nan(n_y,n_x,nFil-1);
H_linPseudo_t   =   nan(n_y,n_x,nFil-1);
H_det           =   nan(1,nFil-1);
% Init variables for post processing
Sigma(:,1)      =   sqrt(diag(P_0));
ErrorFil(:,1)   =   x_true(:,1) - x_0;
P_trace(1)      =   trace(P_0);
P_trace_pos(1)  =   trace(P_0(1:3,1:3));
P_trace_vel(1)  =   trace(P_0(4:6,4:6));

scaling = norm(x_true(1:3,1));

%% EKF run
tic
for k=2:nFil    
    % Prediction
    
    % State propagation
    x_k_km1(:,k)    =   stateFcn(x_k_k(:,k-1),u(:,k-1),dt);           
    
    % Covariance propagation
    P_k_km1(:,:,k)  =   F_x * P_k_k(:,:,k-1) * F_x' + F_w * Q * F_w';
    
    % Predicted measurement
    y_k_km1 = measFcn(x_k_km1(:,k), scaling);
    
    % Gain
    H   =   measJac(x_k_km1(:,k));
    K   =   P_k_km1(:,:,k) * H' / (H * P_k_km1(:,:,k) * H' + R);
    
    % Correction Step
    x_k_k(:,k)      =   x_k_km1(:,k) + K * ( z(:,k) - y_k_km1);
    P_k_k(:,:,k)    =   (eye(n_x) - K * H) * P_k_km1(:,:,k);    
    
    % Debug and post process functions
    MeasurementsFil(:,k)=   y_k_km1;
    Sigma(:,k)      	=   sqrt(diag(P_k_k(:,:,k)));
    err_x               =   x_true(:,k) - x_k_k(:,k);
    ErrorFil(:,k)       =   err_x;
    NEES(k)             =   err_x' * P_k_k(:,:,k) * err_x;
    NEES_pos(k)         =   err_x(1:3)' * P_k_k(1:3,1:3,k) * err_x(1:3);
    NEES_vel(k)         =   err_x(4:6)' * P_k_k(4:6,4:6,k) * err_x(4:6);
    P_trace(k)          =   trace(P_k_k(:,:,k));
    P_trace_pos(k)      =   trace(P_k_k(1:3,1:3,k));
    P_trace_vel(k)      =   trace(P_k_k(4:6,4:6,k));
    K_vec(:,:,k-1)      =   K;
    P_norm(k-1)         =   norm(P_k_k(:,:,k));
    dz_vec(:,k-1)       =   z(:,k) - y_k_km1;    
end
toc

%% Post processing
close all;

% Limit number of error bars
nErrBar     = 100;
iErrBar     = round(linspace(1,nFil,nErrBar));

%% Plot true and estimated position
figure(); hold on;
ax1             = subplot(3,1,1); hold on; grid on;
ptrue           = plot(time,x_true(1,:),'g','LineWidth',2);
pest            = plot(time,x_k_k(1,:),'.r','LineWidth',2);
pstd            = errorbar(time(iErrBar),x_k_k(1,iErrBar),Sigma(1,iErrBar),'.r','LineWidth',.1);
xlabel('T','Interpreter',c.Interpreter);
ylabel('X','Interpreter',c.Interpreter);
ax2 = subplot(3,1,2); hold on; grid on;
plot(time,x_true(2,:),'g','LineWidth',2);
plot(time,x_k_k(2,:),'.r','LineWidth',2);
errorbar(time(iErrBar),x_k_k(2,iErrBar),Sigma(2,iErrBar),'.r','LineWidth',.1);
xlabel('T','Interpreter',c.Interpreter);
ylabel('Y','Interpreter',c.Interpreter);
ax3 = subplot(3,1,3); hold on; grid on;
plot(time,x_true(3,:),'g','LineWidth',2);
plot(time,x_k_k(3,:),'.r','LineWidth',2);
errorbar(time(iErrBar),x_k_k(3,iErrBar),Sigma(3,iErrBar),'.r','LineWidth',.1);
xlabel('T','Interpreter',c.Interpreter);
ylabel('Z','Interpreter',c.Interpreter);
% ylabel('Z','FontSize',c.FS_axes, 'Interpreter',c.Interpreter);
linkaxes([ax1,ax2, ax3],'x');
set(gca,'TickLabelInterpreter',c.Interpreter)
sgtitle('True and Estimated Relative Position','FontWeight','bold','FontSize',c.FS_title, 'Interpreter',c.Interpreter);
legend([ptrue(1) pest(1) pstd], {'True Position','Estimated Position','Standard Deviation'},'FontSize',c.FS_Legend,'Interpreter',c.Interpreter);


%% Plot true and estimated velocity
figure(); hold on;
ax1             = subplot(3,1,1); hold on; grid on;
ptrue       =   plot(time,x_true(4,:),'g','LineWidth',2);
pest        =   plot(time,x_k_k(4,:),'.r','LineWidth',2);
pstd        =   errorbar(time(iErrBar),x_k_k(4,iErrBar),Sigma(4,iErrBar),'.r','LineWidth',.1);
xlabel('T','Interpreter',c.Interpreter);
ylabel('u','Interpreter',c.Interpreter);
ax2 = subplot(3,1,2); hold on; grid on;
plot(time,x_true(5,:),'g','LineWidth',2);
plot(time,x_k_k(5,:),'.r','LineWidth',2);
errorbar(time(iErrBar),x_k_k(5,iErrBar),Sigma(5,iErrBar),'.r','LineWidth',.1);
xlabel('T','Interpreter',c.Interpreter);
ylabel('v','Interpreter',c.Interpreter);
ax3 = subplot(3,1,3); hold on; grid on;
plot(time,x_true(6,:),'g','LineWidth',2);
plot(time,x_k_k(6,:),'.r','LineWidth',2);
errorbar(time(iErrBar),x_k_k(6,iErrBar),Sigma(6,iErrBar),'.r','LineWidth',.1);
xlabel('T','Interpreter',c.Interpreter);
ylabel('w','Interpreter',c.Interpreter);
linkaxes([ax1,ax2, ax3],'x');
set(gca,'TickLabelInterpreter',c.Interpreter)
sgtitle('True and Estimated Relative Velocity','FontWeight','bold','FontSize',c.FS_title, 'Interpreter',c.Interpreter);
legend([ptrue(1) pest(1) pstd], {'True Velocity','Estimated Velocity','Standard Deviation'},'FontSize',c.FS_Legend,'Interpreter',c.Interpreter);

%% Plot true, measured and estimated LOS angles
figure();
ax1 = subplot(2,1,1); hold on; grid on;
ptrue   =   plot(time,z_true(1,:),'-g','LineWidth',2);
pnoise  =   plot(time,z(1,:),'-.b','LineWidth',1);
pest    =   plot(time,MeasurementsFil(1,:),'--r','LineWidth',2);
title('Azimuth');
ax2 = subplot(2,1,2); hold on; grid on;
plot(time,z_true(2,:),'-g','LineWidth',2); grid on;
plot(time,z(2,:),'-.b','LineWidth',1); grid on;
plot(time,MeasurementsFil(2,:),'--r','LineWidth',2);
title('Elevation');
legend([ptrue(1) pnoise(1) pest(1)], {'True', 'Measurement', 'Estimation'});
sgtitle('True and Estimated Measurements');
linkaxes([ax1,ax2],'x');

%% Plot observability indices
figure(); hold on;
ax1 = subplot(3,3,1);
plot(time(2:end),vecnorm(ErrorFil(1:6,2:end)),'g','LineWidth',2);grid on;
title('RMSE Combined');ylabel('[-]');
ax2 = subplot(3,3,2);
plot(time(2:end),vecnorm(ErrorFil(1:3,2:end)),'g','LineWidth',2);grid on;
title('RMSE Position');ylabel('[m]');
ax3 = subplot(3,3,3);
plot(time(2:end),vecnorm(ErrorFil(4:6,2:end)),'g','LineWidth',2);grid on;
title('RMSE Velocity');ylabel('[m/s]');
ax4 = subplot(3,3,4);
plot(time(2:end),P_trace(2:end),'g','LineWidth',2);grid on;
title('Covariance Trace Combined');
ax5 = subplot(3,3,5);
plot(time(2:end),P_trace_pos(2:end),'g','LineWidth',2);grid on;
title('Covariance Trace Position');
ax6 = subplot(3,3,6);
plot(time(2:end),P_trace_vel(2:end),'g','LineWidth',2);grid on;
title('Covariance Trace Velocity');
ax7 = subplot(3,3,7);
plot(time(2:end),NEES(2:end),'g','LineWidth',2);grid on;
title('NEES Combined');
ax8 = subplot(3,3,8);
plot(time(2:end),NEES_pos(2:end),'g','LineWidth',2);grid on;
title('NEES Position');
ax9 = subplot(3,3,9);
plot(time(2:end),NEES_vel(2:end),'g','LineWidth',2);grid on;
title('NEES Velocity');

sgtitle('Observability Indices');
linkaxes([ax1,ax2,ax3,ax4,ax5,ax6,ax7,ax8,ax9],'x');


%% 3D Plots
options = 'cylinder';
%     options = 'cylinder';
%     options = 'sphere';
n3D = 200;                                                                  % number of spheres/cylinders along trajectory
i3D = linspace(1,nDat,n3D);

% Create figure
figure(5); hold on;

% Plot Defender
pDOO_x = results.defender.states.pos(1,iDat);
pDOO_y = results.defender.states.pos(2,iDat);
pDOO_z = results.defender.states.pos(3,iDat);
pD = plot3(pDOO_x,pDOO_y,-pDOO_z,'-g','LineWidth',2);

% Plot invader true position
pIOO_x  = results.invader.states.pos(1,iDat);
pIOO_y  = results.invader.states.pos(2,iDat);
pIOO_z  = results.invader.states.pos(3,iDat);
pI_true = plot3(pIOO_x,pIOO_y,-pIOO_z,'-.r','LineWidth',1);
plot3(pIOO_x(1),pIOO_y(1),-pIOO_z(1),'or','LineWidth',1);
plot3(pIOO_x(end),pIOO_y(end),-pIOO_z(end),'xr','LineWidth',1);

% Plot invader estimated position
pIOO_x_e = x_k_k(1,:) + interp1(iDat,pDOO_x,iFil);
pIOO_y_e = x_k_k(2,:) + interp1(iDat,pDOO_y,iFil);
pIOO_z_e = x_k_k(3,:) + interp1(iDat,pDOO_z,iFil);
pI = plot3(pIOO_x_e,pIOO_y_e,-pIOO_z_e,'--b','LineWidth',2);
plot3(pIOO_x_e(1),pIOO_y_e(1),-pIOO_z_e(1),'ob','LineWidth',2);
plot3(pIOO_x_e(end),pIOO_y_e(end),-pIOO_z_e(end),'xb','LineWidth',2);

% Interpolate data
pIOO_x_e    = interp1(iFil,pIOO_x_e,i3D);
pIOO_y_e    = interp1(iFil,pIOO_y_e,i3D);
pIOO_z_e    = interp1(iFil,pIOO_z_e,i3D);

% Get directions and distance between two points
pIOO_e  = [pIOO_x_e;pIOO_y_e;pIOO_z_e];
dir_vec = gradient(pIOO_e);
dr_vec  = vecnorm(gradient(pIOO_e));


% Extract standard deviation from covariance matrix
Sigma_r = interp1(iFil,vecnorm(Sigma([1 2 3],:)/2),i3D);

% Plot standard deviation of estimation
for j=1:n3D
    
    % Sphere/cylinder radius
    r_j = Sigma_r(j)^(1/2);
    c_x = pIOO_x_e(j);
    c_y = pIOO_y_e(j);
    c_z = -pIOO_z_e(j);
    
    % Plot sphere/cylinder
    switch options
        case 'sphere'
            [x_n,y_n,z_n] = sphere;
            x = x_n * r_j;
            y = y_n * r_j;
            z = z_n * r_j;
            j_sphere = surf(c_x + x, c_y + y, c_z + z,'FaceColor','b','FaceAlpha',0.05, 'EdgeColor', 'none');
        case 'cylinder'
            [x,y,z] = cylinder(r_j);
            j_cylinder = surf(c_x + x, c_y + y, c_z - z*dr_vec(j), 'FaceColor','b', 'FaceAlpha',.1, 'EdgeColor', 'none');
            dir = dir_vec(:,j) / norm(dir_vec(:,j));
            % Rotate cylinder
            r = vrrotvec([0 0 1],dir);
            rotate(j_cylinder,r(1:3),-r(4)*180/pi,[c_x,c_y,c_z]);
    end
    
end

title('3D Scenario');
axis image;
grid on;
view(45,45);
lgd = legend([pD pI pI_true],...
    {'Defender','Invader Estimated','Invader True'});
tmp = sprintf('RMSE_{pos} = %.2fm\n\x03c3_{pos} = %.2fm\n\x03A3_{pos} = %.2fm',...
    norm(ErrorFil(1:3,end)),norm(Sigma(1:3,end)),sum(vecnorm(Sigma(1:3,:)))/nFil);
annotation('textbox',lgd.Position - [0 .1 0 0],...
    'String',tmp,'FitBoxToText','on','BackgroundColor','w');
xlabel('X');ylabel('Y');zlabel('Z');

%     legend(sprintf('Final position standard  deviation = %.3fm',sqrt())));
% Ploz standard deviation of 3D position estimation








