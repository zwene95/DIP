function PostObservabilityAnalysis(setup, results)

c = setup.postOptions.c;

%% Pre Processing Results
N = length(results.time) - 0;
time = results.time(1:N);
dt = diff(time(1:2));

%   State dynamics                                                          x_dot = A*x + B*u + w(t), x[6x1], u [3x1], A [6x3], B [6x3]

%   Get States, structure as follows: [x y z u v w];
x_true = [
    results.invader.states.pos(1,1:N) - results.defender.states.pos(1,1:N)
    results.invader.states.pos(2,1:N) - results.defender.states.pos(2,1:N)
    results.invader.states.pos(3,1:N) - results.defender.states.pos(3,1:N)
    results.invader.states.vel(1,1:N) - results.defender.states.vel(1,1:N)
    results.invader.states.vel(2,1:N) - results.defender.states.vel(2,1:N)
    results.invader.states.vel(3,1:N) - results.defender.states.vel(3,1:N)
    ];

% Get pseudo-controls [ax ay az]
u_true  = results.defender.states.acc(:,1:N);

% Process noise
mu_p    =  0;   % mean
std_p   =  0;   % standard deviation (10)
rng(2019);
w = normrnd(mu_p,std_p,size(u_true));

u = u_true + w;

% Measurement dynamics                                                      % z = h(x(t)) + v(t);
z_true = [
    results.LOS.azimuth(:,1:N)
    results.LOS.elevation(:,1:N)
    ];

% Measurement noise
mu_m    =  0;       % mean
std_m   =  0e-2;    % standard deviation (2e-2)
rng(2020);
v = normrnd(mu_m,std_m,size(z_true));
z = z_true + v;

%% EKF init

% State jacobians (linear time invariant)
F_x = stateJac_x(dt);
F_w = stateJac_w(dt);

% Initial bias
mu_x0       = 0;
std_x0_pos  = 10;   %10
% std_x0_vel  = 10;   %10
rng(2018);
b_x0_pos    = normrnd(mu_x0, std_x0_pos, [3 1]);
% b_x0_vel    = normrnd(mu_x0, std_x0_vel, [3 1]);
% b_x0        = [b_x0_pos; b_x0_vel];
b_x0        = [b_x0_pos; -x_true(4:6,1)] * 1;


% Allocate state and state covariance matrix
x_0     = 	x_true(:,1) + b_x0;                                             % [eye(3),zeros(3); zeros(3,6)]
P_0     =   diag([1e2,1e2,1e2,5e2,5e2,5e2]);                                % diag([1e2,1e2,1e2,1e2,1e2,1e2])
n_x     =   length(x_0);
n_y     =	length(measFcn(x_0,1));
% Setup variables for states and state covariance matrix
x_k_km1 =   nan(n_x,N);
x_k_k   =   nan(n_x,N);
P_k_km1 =   nan(n_x,n_x,N);
P_k_k   =   nan(n_x,n_x,N);
% Initialize variables for states and state covariance matrix
x_k_km1(:,1)    = x_0;
P_k_km1(:,:,1)  = P_0;
x_k_k(:,1)      = x_0;
P_k_k(:,:,1)    = P_0;

% Process noise variance matrix
Q = diag([1e2, 1e2, 1e2]);
% Measurement noise variance matrix
R = diag([1e-2 1e-2]);

%% Setup variables for post processing
measurements    =   nan(2,N);
std             =   nan(n_x,N);
err_vec         =   nan(n_x,N);
NEES            =   nan(1,N);                                               % Normalized Estimation Error Squared combined
NEES_pos        =   nan(1,N);                                               % Normalized Estimation Error Squared position only
NEES_vel        =   nan(1,N);                                               % Normalized Estimation Error Squared velocity only
P_trace         =   nan(1,N);
P_trace_pos     =   nan(1,N);
P_trace_vel     =   nan(1,N);
% Init variables for post processing
std(:,1)        =   sqrt(diag(P_0));
err_vec(:,1)    =   x_true(:,1) - x_0;
P_trace(1)      =   trace(P_0);
P_trace_pos(1)  =   trace(P_0(1:3,1:3));
P_trace_vel(1)  =   trace(P_0(4:6,4:6));

scaling = norm(x_true(1:3,1));

%% EKF run
for k=2:N
    
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
    
    % Update
    x_k_k(:,k)      =   x_k_km1(:,k) + K * ( z(:,k) - y_k_km1);
    P_k_k(:,:,k)    =   (eye(n_x) - K * H) * P_k_km1(:,:,k);
    
    % Debug and post process functions
    measurements(:,k)   =   y_k_km1;
    std(:,k)            =   sqrt(diag(P_k_k(:,:,k)));
    err_x               =   x_true(:,k) - x_k_k(:,k);
    err_vec(:,k)        =   err_x;
    NEES(k)             =   err_x' * P_k_k(:,:,k) * err_x;
    NEES_pos(k)         =   err_x(1:3)' * P_k_k(1:3,1:3,k) * err_x(1:3);
    NEES_vel(k)         =   err_x(4:6)' * P_k_k(4:6,4:6,k) * err_x(4:6);
    P_trace(k)          =   trace(P_k_k(:,:,k));
    P_trace_pos(k)      =   trace(P_k_k(1:3,1:3,k));
    P_trace_vel(k)      =   trace(P_k_k(4:6,4:6,k));
    
end

%% Plot Results
figures     = zeros(1,5);
fignames    = strings(size(figures));

% Plot true and estimated position
idx             = 1;
fignames(idx)   = 'True and Estimated Relative Position';
figures(idx)    = figure('Tag',fignames(idx),'name', fignames(idx),'Position', c.Pos_Groesse_SVGA);
ax1             = subplot(3,1,1); hold on; grid on;
ptrue           =   plot(time,x_true(1,:),'g','LineWidth',2);
pest            =   plot(time,x_k_k(1,:),'.r','LineWidth',2);
pstd            =   errorbar(time,x_k_k(1,:),std(1,:),'.r','LineWidth',.1);
xlabel('T','Interpreter',c.Interpreter);
ylabel('X','Interpreter',c.Interpreter);
ax2 = subplot(3,1,2); hold on; grid on;
plot(time,x_true(2,:),'g','LineWidth',2);
plot(time,x_k_k(2,:),'.r','LineWidth',2);
errorbar(time,x_k_k(2,:),std(2,:),'.r','LineWidth',.1);
xlabel('T','Interpreter',c.Interpreter);
ylabel('Y','Interpreter',c.Interpreter);
ax3 = subplot(3,1,3); hold on; grid on;
plot(time,x_true(3,:),'g','LineWidth',2);
plot(time,x_k_k(3,:),'.r','LineWidth',2);
errorbar(time,x_k_k(3,:),std(3,:),'.r','LineWidth',.1);
xlabel('T','Interpreter',c.Interpreter);
ylabel('Z','Interpreter',c.Interpreter);
% ylabel('Z','FontSize',c.FS_axes, 'Interpreter',c.Interpreter);
linkaxes([ax1,ax2, ax3],'x');
set(gca,'TickLabelInterpreter',c.Interpreter)
sgtitle(fignames(idx),'FontWeight','bold','FontSize',c.FS_title, 'Interpreter',c.Interpreter);
legend([ptrue(1) pest(1) pstd], {'True Position','Estimated Position','Standard Deviation'},'FontSize',c.FS_Legend,'Interpreter',c.Interpreter);


% Plot true and estimated velocity
idx             = 2;
fignames(idx)   = 'True and Estimated  Relative Velocity';
figures(idx)    = figure('Tag',fignames(idx),'name', fignames(idx),'Position', c.Pos_Groesse_SVGA);
ax1             = subplot(3,1,1); hold on; grid on;
ptrue       =   plot(time,x_true(4,:),'g','LineWidth',2);
pest        =   plot(time,x_k_k(4,:),'.r','LineWidth',2);
pstd        =   errorbar(time,x_k_k(4,:),std(4,:),'.r','LineWidth',.1);
xlabel('T','Interpreter',c.Interpreter);
ylabel('v','Interpreter',c.Interpreter);
ax2 = subplot(3,1,2); hold on; grid on;
plot(time,x_true(5,:),'g','LineWidth',2);
plot(time,x_k_k(5,:),'.r','LineWidth',2);
errorbar(time,x_k_k(5,:),std(5,:),'.r','LineWidth',.1);
xlabel('T','Interpreter',c.Interpreter);
ylabel('v','Interpreter',c.Interpreter);
ax3 = subplot(3,1,3); hold on; grid on;
plot(time,x_true(6,:),'g','LineWidth',2);
plot(time,x_k_k(6,:),'.r','LineWidth',2);
errorbar(time,x_k_k(6,:),std(6,:),'.r','LineWidth',.1);
xlabel('T','Interpreter',c.Interpreter);
ylabel('w','Interpreter',c.Interpreter);
linkaxes([ax1,ax2, ax3],'x');
set(gca,'TickLabelInterpreter',c.Interpreter)
sgtitle(fignames(idx),'FontWeight','bold','FontSize',c.FS_title, 'Interpreter',c.Interpreter);
legend([ptrue(1) pest(1) pstd], {'True Velocity','Estimated Velocity','Standard Deviation'},'FontSize',c.FS_Legend,'Interpreter',c.Interpreter);

% Plot true, measured and estimated LOS angles
idx             = 3;
fignames(idx)   = 'True and Estimated Measurements';
figures(3)      = figure('Tag',fignames(idx),'name', fignames(idx),'Position', c.Pos_Groesse_SVGA);
ax1             = subplot(2,1,2); hold on; grid on;
ptrue           =   plot(time,z_true(1,:),'-g','LineWidth',2);
pnoise          =   plot(time,z(1,:),'-.b','LineWidth',1);
pest            =   plot(time,measurements(1,:),'--r','LineWidth',2);
title('Azimuth','FontWeight','bold','FontSize',c.FS_subtitle, 'Interpreter',c.Interpreter);
ax2 = subplot(2,1,1); hold on; grid on;
plot(time,z_true(2,:),'-g','LineWidth',2); grid on;
plot(time,z(2,:),'-.b','LineWidth',1); grid on;
plot(time,measurements(2,:),'--r','LineWidth',2);
title('Elevation','FontWeight','bold','FontSize',c.FS_subtitle, 'Interpreter',c.Interpreter);
set(gca,'TickLabelInterpreter',c.Interpreter);
legend([ptrue(1) pnoise(1) pest(1)], {'True', 'Measurement', 'Estimation'},'FontSize',c.FS_Legend,'Interpreter',c.Interpreter);
sgtitle(fignames(idx),'FontWeight','bold','FontSize',c.FS_title, 'Interpreter',c.Interpreter);
linkaxes([ax1,ax2],'x');

% Plot observability
idx             = 4;
fignames(idx)   = 'Observability Indices';
figures(idx)    = figure('Tag',fignames(idx),'name', fignames(idx),'Position', c.Pos_Groesse_SVGA);
hold on;
ax1 = subplot(3,3,1);
plot(time(2:end),vecnorm(err_vec(1:6,2:end)),'g','LineWidth',2);grid on;
title('RMSE Combined','FontWeight','bold','FontSize',c.FS_subtitle, 'Interpreter',c.Interpreter)
ylabel('[-]','Interpreter',c.Interpreter);
ax2 = subplot(3,3,2);
plot(time(2:end),vecnorm(err_vec(1:3,2:end)),'g','LineWidth',2);grid on;
title('RMSE Position','FontWeight','bold','FontSize',c.FS_subtitle, 'Interpreter',c.Interpreter)
ylabel('[m]','Interpreter',c.Interpreter);
ax3 = subplot(3,3,3);
plot(time(2:end),vecnorm(err_vec(4:6,2:end)),'g','LineWidth',2);grid on;
title('RMSE Velocity','FontWeight','bold','FontSize',c.FS_subtitle, 'Interpreter',c.Interpreter)
ylabel('[m/s]','Interpreter',c.Interpreter);
ax4 = subplot(3,3,4);
plot(time(2:end),P_trace(2:end),'g','LineWidth',2);grid on;
title('Covariance Trace Combined','FontWeight','bold','FontSize',c.FS_subtitle, 'Interpreter',c.Interpreter);
ax5 = subplot(3,3,5);
plot(time(2:end),P_trace_pos(2:end),'g','LineWidth',2);grid on;
title('Covariance Trace Position','FontWeight','bold','FontSize',c.FS_subtitle, 'Interpreter',c.Interpreter);
ax6 = subplot(3,3,6);
plot(time(2:end),P_trace_vel(2:end),'g','LineWidth',2);grid on;
title('Covariance Trace Velocity','FontWeight','bold','FontSize',c.FS_subtitle, 'Interpreter',c.Interpreter);
ax7 = subplot(3,3,7);
plot(time(2:end),NEES(2:end),'g','LineWidth',2);grid on;
title('NEES Combined','FontWeight','bold','FontSize',c.FS_subtitle, 'Interpreter',c.Interpreter);
ax8 = subplot(3,3,8);
plot(time(2:end),NEES_pos(2:end),'g','LineWidth',2);grid on;
title('NEES Position','FontWeight','bold','FontSize',c.FS_subtitle, 'Interpreter',c.Interpreter);
ax9 = subplot(3,3,9);
plot(time(2:end),NEES_vel(2:end),'g','LineWidth',2);grid on;
set(gca,'TickLabelInterpreter',c.Interpreter)
title('NEES Velocity','FontWeight','bold','FontSize',c.FS_subtitle, 'Interpreter',c.Interpreter);
sgtitle(fignames(idx),'FontWeight','bold','FontSize',c.FS_title, 'Interpreter',c.Interpreter);
linkaxes([ax1,ax2,ax3,ax4,ax5,ax6,ax7,ax8,ax9],'x');


% 3D Plot
options.animated = 1;
options.CItype = 'cylinder';                                                   % Confidence interval type
%     options.CItype = 'cylinder';
%     options.CItype = 'sphere';

% Create figure
idx             = 5;
fignames(idx)   = 'Intercept Animated with Observability';
figures(5)      = figure('Tag',fignames(idx),'name', fignames(idx),'Position', c.Pos_Groesse_SVGA);
hold on;

% Plot Defender
pDOO_x = results.defender.states.pos(1,1:N);
pDOO_y = results.defender.states.pos(2,1:N);
pDOO_z = results.defender.states.pos(3,1:N);
if options.animated
    pD = animatedline('Color','green','LineStyle','-','LineWidth',2);
else
    pD = plot3(pDOO_x,pDOO_y,-pDOO_z,'-g','LineWidth',2);
end
% Plot invader true position
pIOO_x  = results.invader.states.pos(1,1:N);
pIOO_y  = results.invader.states.pos(2,1:N);
pIOO_z  = results.invader.states.pos(3,1:N);
if options.animated    
    pI_true = animatedline('Color','blue','LineStyle','-.','LineWidth',2);
    plot3(pIOO_x(1),pIOO_y(1),-pIOO_z(1),'xb','LineWidth',2);
    
else
    pI_true = plot3(pIOO_x,pIOO_y,-pIOO_z,'-.b','LineWidth',1);
    plot3(pIOO_x(1),pIOO_y(1),-pIOO_z(1),'xb','LineWidth',1);
end

% Plot invader estimated position
pIOO_x_e = x_k_k(1,1:N) + pDOO_x;
pIOO_y_e = x_k_k(2,1:N) + pDOO_y;
pIOO_z_e = x_k_k(3,1:N) + pDOO_z;
if options.animated
    pI = animatedline('Color','red','LineStyle','--','LineWidth',2);    
    plot3(pIOO_x_e(1),pIOO_y_e(1),-pIOO_z_e(1),'xr','LineWidth',2);
    plot3(pIOO_x_e(end),pIOO_y_e(end),-pIOO_z_e(end),'or','LineWidth',2);
else
    pI = plot3(pIOO_x_e,pIOO_y_e,-pIOO_z_e,'--r','LineWidth',2);
    plot3(pIOO_x_e(1),pIOO_y_e(1),-pIOO_z_e(1),'xr','LineWidth',2);
    plot3(pIOO_x_e(end),pIOO_y_e(end),-pIOO_z_e(end),'or','LineWidth',2);
end

%     pI = quiver3(pIOO_x_e,pIOO_y_e,-pIOO_z_e,vIOO_x_e,vIOO_y_e,-vIOO_z_e);

xlabel('X','Interpreter',c.Interpreter);
ylabel('Y','Interpreter',c.Interpreter);
zlabel('Z','Interpreter',c.Interpreter);
title(fignames(idx),'FontWeight','bold','FontSize',c.FS_title, 'Interpreter',c.Interpreter);
axis image;
grid on;
view(45,45);
set(gca,'TickLabelInterpreter',c.Interpreter)
lgd = legend([pD pI pI_true], {'Defender','Invader Estimated','Invader True'},'FontSize',c.FS_Legend,'Interpreter',c.Interpreter);
tmp = sprintf('pos_{RMSE} = %.3fm \n\x03c3_{RMSE} = %.3fm',norm(err_vec(1:3,end)),norm(std(1:3,end)));
annotation('textbox',lgd.Position - [0 .1 0 0],'String',tmp,'FitBoxToText','on','BackgroundColor','w');

if options.animated
    % Animate Trajectories
    N_max   = 200;                                                              % number of spheres along trajectory
    i_data  = linspace(1,N,N);
    i_query = linspace(1,N,N_max);
    pDOO_x      = interp1(i_data,pDOO_x,i_query);
    pDOO_y      = interp1(i_data,pDOO_y,i_query);
    pDOO_z      = interp1(i_data,pDOO_z,i_query);
    pIOO_x      = interp1(i_data,pIOO_x,i_query);
    pIOO_y      = interp1(i_data,pIOO_y,i_query);
    pIOO_z      = interp1(i_data,pIOO_z,i_query);
    pIOO_x_e_ip = interp1(i_data,pIOO_x_e,i_query);
    pIOO_y_e_ip = interp1(i_data,pIOO_y_e,i_query);
    pIOO_z_e_ip = interp1(i_data,pIOO_z_e,i_query);
    % a = tic;
    for n = 1 : N_max
        addpoints(pD, pDOO_x(n), pDOO_y(n), -pDOO_z(n));
        addpoints(pI_true, pIOO_x(n), pIOO_y(n), -pIOO_z(n));
        addpoints(pI, pIOO_x_e_ip(n), pIOO_y_e_ip(n), -pIOO_z_e_ip(n));
        drawnow
    end
    % drawnow
end


% Plot confidence interval
% Get invader estimated velocity
vIOO_x_e = gradient(pIOO_x_e);
vIOO_y_e = gradient(pIOO_y_e);
vIOO_z_e = gradient(pIOO_z_e);
directions = [vIOO_x_e;vIOO_y_e;vIOO_z_e];
% Interpolate data
j_max   = 2 * setup.Solver.gridSize;                                                              % number of spheres/cylinders along trajectory
i_data  = linspace(1,N,N);
i_query = linspace(1,N,j_max);

pIOO_x_e_ip    = interp1(i_data,pIOO_x_e,i_query);
pIOO_y_e_ip    = interp1(i_data,pIOO_y_e,i_query);
pIOO_z_e_ip    = interp1(i_data,pIOO_z_e,i_query);
directions  = interp1(i_data,directions',i_query)';
spacing     = vecnorm(directions);

% Extract standard deviation from covariance matrix
r_vec = interp1(i_data,vecnorm(std([1 2 3],:)/2),i_query);

% Plot standard deviation of estimation
for j=1:j_max
    
    % Sphere/cylinder radius
    r_j = r_vec(j)^(1/2);
    c_x = pIOO_x_e_ip(j);
    c_y = pIOO_y_e_ip(j);
    c_z = -pIOO_z_e_ip(j);
    
    % Plot sphere/cylinder
    switch options.CItype
        case 'sphere'
            [x_n,y_n,z_n] = sphere;
            x = x_n * r_j;
            y = y_n * r_j;
            z = z_n * r_j;
            j_sphere = surf(c_x + x, c_y + y, c_z + z,'FaceColor','r','FaceAlpha',0.05, 'EdgeColor', 'none');
        case 'cylinder'
            [x,y,z] = cylinder(r_j);
            j_cylinder = surf(c_x + x, c_y + y, c_z + z*spacing(j)*.55, 'FaceColor','r', 'FaceAlpha',.1, 'EdgeColor', 'none');
            dir = directions(:,j) / norm(directions(:,j));
            % Rotate cylinder
            r = vrrotvec([0 0 1],dir);
            rotate(j_cylinder,r(1:3),-r(4)*180/pi,[c_x,c_y,c_z]);
%             java.lang.Thread.sleep(100)
    end        
end
legend([pD pI pI_true], {'Defender','Invader Estimated','Invader True'},'FontSize',c.FS_Legend,'Interpreter',c.Interpreter);



% Save plot
if setup.postOptions.Save
    if setup.postOptions.Jpg
        for i=1:numel(figures)
            saveas(figures(i),fullfile(setup.postOptions.PathJpg,fignames(i)),'jpg');
        end
    end
    if setup.postOptions.Fig
        savefig(figures,fullfile(setup.postOptions.PathFig,'PostObservability'));
    end
end

end