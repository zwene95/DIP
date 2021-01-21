function PostObservabilityAnalysis(Setup, Results)

c = Setup.PostOptions.c;

%% Pre Processing Results

%   Get States, structure as follows: x_true = [x y z u v w];
x_true = [
    Results.Invader.States.Pos(1,:) - Results.Defender.States.Pos(1,:)
    Results.Invader.States.Pos(2,:) - Results.Defender.States.Pos(2,:)
    Results.Invader.States.Pos(3,:) - Results.Defender.States.Pos(3,:)
    Results.Invader.States.Vel(1,:) - Results.Defender.States.Vel(1,:)
    Results.Invader.States.Vel(2,:) - Results.Defender.States.Vel(2,:)
    Results.Invader.States.Vel(3,:) - Results.Defender.States.Vel(3,:)
    ];

% Get pseudo-controls [ax ay az]
u_true  = Results.Defender.States.Acc(:,:);

% Setup extended Kalman filter
myEKF = EKF_Object;
myEKF.TimeHistory       = Results.Time;
myEKF.StateHistory      = x_true;
myEKF.ControlHistory    = u_true;
myEKF.ObserverConfig    = Setup.ObserverConfig;
% Run tracking
% EKF = myEKF.runEKF;
EKF = myEKF.runEKF;

%% Pre process results for plotting
% Defender position
pDOO_x = interp1(EKF.iDat,Results.Defender.States.Pos(1,:),EKF.iEKF);
pDOO_y = interp1(EKF.iDat,Results.Defender.States.Pos(2,:),EKF.iEKF);
pDOO_z = interp1(EKF.iDat,Results.Defender.States.Pos(3,:),EKF.iEKF);
% Invader position
pIOO_x  = interp1(EKF.iDat,Results.Invader.States.Pos(1,:),EKF.iEKF);
pIOO_y  = interp1(EKF.iDat,Results.Invader.States.Pos(2,:),EKF.iEKF);
pIOO_z  = interp1(EKF.iDat,Results.Invader.States.Pos(3,:),EKF.iEKF);

figures     = zeros(1,7);
fignames    = strings(size(figures));
nErrBar     = 100;
iErrBar     = round(linspace(1,EKF.nEKF,nErrBar));

%% Plot true and estimated position
idx             = 1;
fignames(idx)   = 'True and Estimated Relative Position';
figures(idx)    = figure('Tag',fignames(idx),'name', fignames(idx),'Position', c.Pos_Groesse_SVGA);
ax1             = subplot(3,1,1); hold on; grid on;
set(gca,c.Axes{:});
ptrue           = plot(EKF.Time,EKF.x_true(1,:),'g','LineWidth',2);
pest            = plot(EKF.Time,EKF.x_k_k(1,:),'--r','LineWidth',2);
pstd            = errorbar(EKF.Time(iErrBar),EKF.x_k_k(1,iErrBar),EKF.Std(1,iErrBar),'.r','LineWidth',.1);
xlabel('T [s]',c.Label{:});
ylabel('X [m]',c.Label{:});
ax2 = subplot(3,1,2); hold on; grid on;
set(gca,c.Axes{:});
plot(EKF.Time,EKF.x_true(2,:),'g','LineWidth',2);
plot(EKF.Time,EKF.x_k_k(2,:),'--r','LineWidth',2);
errorbar(EKF.Time(iErrBar),EKF.x_k_k(2,iErrBar),EKF.Std(2,iErrBar),'.r','LineWidth',.1);
xlabel('T [s]',c.Label{:});
ylabel('Y [m]',c.Label{:});
ax3 = subplot(3,1,3); hold on; grid on;
set(gca,c.Axes{:});
plot(EKF.Time,EKF.x_true(3,:),'g','LineWidth',2);
plot(EKF.Time,EKF.x_k_k(3,:),'--r','LineWidth',2);
errorbar(EKF.Time(iErrBar),EKF.x_k_k(3,iErrBar),EKF.Std(3,iErrBar),'.r','LineWidth',.1);
xlabel('T [s]',c.Label{:});
ylabel('Z [m]',c.Label{:});
linkaxes([ax1,ax2, ax3],'x');
sgtitle(fignames(idx),c.Title{:});
legend([ptrue(1) pest(1) pstd], {'True Position','Estimated Position','Standard Deviation'},c.Legend{:});

%% Plot true and estimated velocity
idx             = 2;
fignames(idx)   = 'True and Estimated  Relative Velocity';
figures(idx)    = figure('Tag',fignames(idx),'name', fignames(idx),'Position', c.Pos_Groesse_SVGA);
ax1             = subplot(3,1,1); hold on; grid on;
set(gca,c.Axes{:});
ptrue       =   plot(EKF.Time,EKF.x_true(4,:),'g','LineWidth',2);
pest        =   plot(EKF.Time,EKF.x_k_k(4,:),'--r','LineWidth',2);
pstd        =   errorbar(EKF.Time(iErrBar),EKF.x_k_k(4,iErrBar),EKF.Std(4,iErrBar),'.r','LineWidth',.1);
xlabel('T [s]',c.Label{:});
ylabel('U [m/s]',c.Label{:});
ax2 = subplot(3,1,2); hold on; grid on;
set(gca,c.Axes{:});
plot(EKF.Time,EKF.x_true(5,:),'g','LineWidth',2);
plot(EKF.Time,EKF.x_k_k(5,:),'--r','LineWidth',2);
errorbar(EKF.Time(iErrBar),EKF.x_k_k(5,iErrBar),EKF.Std(5,iErrBar),'.r','LineWidth',.1);
xlabel('T [s]',c.Label{:});
ylabel('V [m/s]',c.Label{:});
ax3 = subplot(3,1,3); hold on; grid on;
set(gca,c.Axes{:});
plot(EKF.Time,EKF.x_true(6,:),'g','LineWidth',2);
plot(EKF.Time,EKF.x_k_k(6,:),'--r','LineWidth',2);
errorbar(EKF.Time(iErrBar),EKF.x_k_k(6,iErrBar),EKF.Std(6,iErrBar),'.r','LineWidth',.1);
xlabel('T [s]',c.Label{:});
ylabel('W [m/s]',c.Label{:});
linkaxes([ax1,ax2, ax3],'x');
sgtitle(fignames(idx),c.Title{:});
legend([ptrue(1) pest(1) pstd], {'True Velocity','Estimated Velocity','Standard Deviation'},c.Legend{:});

%% Plot true and estimated position spherical
idx             = 3;
fignames(idx)   = 'True and Estimated Relative Spherical Position';
figures(idx)    = figure('Tag',fignames(idx),'name', fignames(idx),'Position', c.Pos_Groesse_SVGA);
ax1             = subplot(3,1,1); hold on; grid on;
set(gca,c.Axes{:});
ptrue           = plot(EKF.Time,EKF.x_S_true(1,:),'g','LineWidth',2);
pest            = plot(EKF.Time,EKF.x_S_est(1,:),'--r','LineWidth',2);
pstd            = errorbar(EKF.Time(iErrBar),EKF.x_S_est(1,iErrBar),EKF.Std_S(1,iErrBar),'.r','LineWidth',.1);
xlabel('T [s]',c.Label{:});
ylabel('R [m]',c.Label{:});
ax2 = subplot(3,1,2); hold on; grid on;
set(gca,c.Axes{:});
plot(EKF.Time,EKF.x_S_true(2,:),'g','LineWidth',2);
plot(EKF.Time,EKF.x_S_est(2,:),'--r','LineWidth',2);
errorbar(EKF.Time(iErrBar),EKF.x_S_est(2,iErrBar),EKF.Std_S(2,iErrBar),'.r','LineWidth',.1);
xlabel('T [s]',c.Label{:});
ylabel('$$\beta$$ [rad]',c.Label{:});
ax3 = subplot(3,1,3); hold on; grid on;
set(gca,c.Axes{:});
plot(EKF.Time,EKF.x_S_true(3,:),'g','LineWidth',2);
plot(EKF.Time,EKF.x_S_est(3,:),'--r','LineWidth',2);
errorbar(EKF.Time(iErrBar),EKF.x_S_est(3,iErrBar),EKF.Std_S(3,iErrBar),'.r','LineWidth',.1);
xlabel('T [s]',c.Label{:});
ylabel('$$\epsilon$$ [rad]',c.Label{:});
linkaxes([ax1,ax2, ax3],'x');
sgtitle(fignames(idx),c.Title{:});
legend([ptrue(1) pest(1) pstd], {'True Spherical Position','Estimated Spherical Position','Standard Deviation'},c.Legend{:});

%% Plot true and estimated velocity spherical
idx             = 4;
fignames(idx)   = 'True and Estimated Relative Spherical Velocity';
figures(idx)    = figure('Tag',fignames(idx),'name', fignames(idx),'Position', c.Pos_Groesse_SVGA);
ax1             = subplot(3,1,1); hold on; grid on;
set(gca,c.Axes{:});
ptrue           = plot(EKF.Time,EKF.x_S_true(4,:),'g','LineWidth',2);
pest            = plot(EKF.Time,EKF.x_S_est(4,:),'--r','LineWidth',2);
pstd            = errorbar(EKF.Time(iErrBar),EKF.x_S_est(4,iErrBar),EKF.Std_S(4,iErrBar),'.r','LineWidth',.1);
xlabel('T [s]',c.Label{:});
ylabel('$$\dot R$$ [m/s]',c.Label{:});
ax2 = subplot(3,1,2); hold on; grid on;
set(gca,c.Axes{:});
plot(EKF.Time,EKF.x_S_true(5,:),'g','LineWidth',2);
plot(EKF.Time,EKF.x_S_est(5,:),'--r','LineWidth',2);
errorbar(EKF.Time(iErrBar),EKF.x_S_est(5,iErrBar),EKF.Std_S(5,iErrBar),'.r','LineWidth',.1);
xlabel('T [s]',c.Label{:});
ylabel('$$\dot \beta$$ [rad/s]',c.Label{:});
ylim([-1,1]);
ax3 = subplot(3,1,3); hold on; grid on;
set(gca,c.Axes{:});
plot(EKF.Time,EKF.x_S_true(6,:),'g','LineWidth',2);
plot(EKF.Time,EKF.x_S_est(6,:),'--r','LineWidth',2);
errorbar(EKF.Time(iErrBar),EKF.x_S_est(6,iErrBar),EKF.Std_S(6,iErrBar),'.r','LineWidth',.1);
xlabel('T [s]',c.Label{:});
ylabel('$$\dot \epsilon$$ [rad/s]',c.Label{:});
ylim([-1,1]);
linkaxes([ax1,ax2, ax3],'x');
sgtitle(fignames(idx),c.Title{:});
legend([ptrue(1) pest(1) pstd], {'True Spherical Position','Estimated Spherical Position','Standard Deviation'},c.Legend{:});

%% Plot true, measured and estimated LOS angles
idx             = 5;
fignames(idx)   = 'True and Estimated Measurements';
figures(idx)    = figure('Tag',fignames(idx),'name', fignames(idx),'Position', c.Pos_Groesse_SVGA);
ax1             = subplot(2,1,2); hold on; grid on;
set(gca,c.Axes{:});
plot(EKF.Time,EKF.z_true(2,:)*c.rad2deg,'-g','LineWidth',2); grid on;
plot(EKF.Time,EKF.z(2,:)*c.rad2deg,'-.b','LineWidth',1); grid on;
plot(EKF.Time,EKF.Measurements(2,:)*c.rad2deg,'--r','LineWidth',2);
title('Elevation',c.Subtitle{:});
xlabel('T [s]',c.Label{:});
ylabel('$$\epsilon$$ [deg]',c.Label{:});
ax2 = subplot(2,1,1); hold on; grid on;
set(gca,c.Axes{:});
ptrue           =   plot(EKF.Time,EKF.z_true(1,:)*c.rad2deg,'-g','LineWidth',2);
pnoise          =   plot(EKF.Time,EKF.z(1,:)*c.rad2deg,'-.b','LineWidth',1);
pest            =   plot(EKF.Time,EKF.Measurements(1,:)*c.rad2deg,'--r','LineWidth',2);
title('Azimuth',c.Subtitle{:});
xlabel('T [s]',c.Label{:});
ylabel('$$\beta$$ [deg]',c.Label{:});
linkaxes([ax1,ax2],'x');
sgtitle(fignames(idx),c.Title{:});
legend([ptrue(1) pnoise(1) pest(1)], {'True', 'Measurement', 'Estimation'},c.Legend{:});

%% Plot observability indices
idx             = 6;
fignames(idx)   = 'Observability Indices';
figures(idx)    = figure('Tag',fignames(idx),'name', fignames(idx),'Position', c.Pos_Groesse_SVGA);
hold on;
ax1 = subplot(2,2,1);
plot(EKF.Time,sqrt(EKF.SE_pos),'g','LineWidth',2);grid on;
set(gca,c.Axes{:});
title('Position Error',c.Subtitle{:})
xlabel('T [s]',c.Label{:});
ylabel('[$$m^{2}$$]',c.Label{:});
ylim([0,inf]);
ax2 = subplot(2,2,2);
plot(EKF.Time,sqrt(EKF.SE_vel),'g','LineWidth',2);grid on;
set(gca,c.Axes{:});
title('Velocity Error',c.Subtitle{:})
xlabel('T [s]',c.Label{:});
ylabel('[$$m^{2}/s^{2}$$]',c.Label{:});
ylim([0,inf]);
ax3 = subplot(2,2,3);
plot(EKF.Time,EKF.P_trace_pos,'g','LineWidth',2);grid on;
set(gca,c.Axes{:});
title('Position Covariace Trace',c.Subtitle{:})
xlabel('T [s]',c.Label{:});
ylabel('[$$m^{2}$$]',c.Label{:});
ylim([0,inf]);
ax4 = subplot(2,2,4);
plot(EKF.Time,EKF.P_trace_vel,'g','LineWidth',2);grid on;
set(gca,c.Axes{:});
title('Velocity Covariace Trace',c.Subtitle{:});
xlabel('T [s]',c.Label{:});
ylabel('[$$m^{2}/s^{2}$$]',c.Label{:});
ylim([0,inf]);
sgtitle(fignames(idx),c.Title{:});
linkaxes([ax1,ax2,ax3,ax4],'x');

%% 3D Plot
options.animated = 1;
options.CItype = 'cylinder';                                                   % Confidence interval type
%     options.CItype = 'cylinder';
%     options.CItype = 'sphere';

% Create figure
idx = 7;
fignames(idx) = 'DIP with Target Observability Information';
figures(idx)  = figure('Tag',fignames(idx),'name', fignames(idx),'Position', c.Pos_Groesse_SVGA);
hold on;

% Plot defender position
if options.animated
    pD = animatedline('Color','green','LineStyle','-','LineWidth',2);
    plot3(pDOO_x(1),pDOO_y(1),-pDOO_z(1),'-gO','LineWidth',2);
else
    pD = plot3(pDOO_x,pDOO_y,-pDOO_z,'-g','LineWidth',2);
    plot3(pDOO_x(1),pDOO_y(1),-pDOO_z(1),'-gO','LineWidth',2);
end

% Plot invader position
if options.animated
    pI_true = animatedline('Color','red','LineStyle','-.','LineWidth',2);
    plot3(pIOO_x(1),pIOO_y(1),-pIOO_z(1),'or','LineWidth',2);
else
    pI_true = plot3(pIOO_x,pIOO_y,-pIOO_z,'-.r','LineWidth',1);
    plot3(pIOO_x(1),pIOO_y(1),-pIOO_z(1),'or','LineWidth',1);
end

% Plot invader estimated position
pIOO_x_e = EKF.x_k_k(1,:) + pDOO_x;
pIOO_y_e = EKF.x_k_k(2,:) + pDOO_y;
pIOO_z_e = EKF.x_k_k(3,:) + pDOO_z;
if options.animated
    pI = animatedline('Color','blue','LineStyle','--','LineWidth',2);
    plot3(pIOO_x_e(1),pIOO_y_e(1),-pIOO_z_e(1),'ob','LineWidth',2);
    plot3(pIOO_x_e(end),pIOO_y_e(end),-pIOO_z_e(end),'xb','LineWidth',2);
else
    pI = plot3(pIOO_x_e,pIOO_y_e,-pIOO_z_e,'--b','LineWidth',2);
    plot3(pIOO_x_e(1),pIOO_y_e(1),-pIOO_z_e(1),'ob','LineWidth',2);
    plot3(pIOO_x_e(end),pIOO_y_e(end),-pIOO_z_e(end),'xb','LineWidth',2);
end

xlabel('X [m]',c.Label{:});
ylabel('Y [m]',c.Label{:});
zlabel('Z [m]',c.Label{:});
title(fignames(idx),c.Title{:});
axis image;
grid on;
% view(-60,20);
view(-93.5,18.5);
set(gca,c.Axes{:});
lgd = legend([pD pI pI_true],...
    {'Defender','Invader Estimated','Invader True'},...
    c.Legend{:});
tmp = sprintf('RMSE_{Pos} = %.2fm\nRMSE_{Vel} = %.2fm\n\x03A3 Cov_{Pos} = %.0fm^2',...
    EKF.RMSE_pos,EKF.RMSE_vel,sum(EKF.P_trace_pos)/EKF.nEKF);
annotation('textbox',lgd.Position - [0 .1 0 0],'String',tmp,'FitBoxToText','on','BackgroundColor','w');

if options.animated
    % Animate Trajectories
    nAnim       = 200;                                                        % number of animation points
    iAnim       = linspace(1,EKF.nDat,nAnim);
    pDOO_x      = interp1(EKF.iEKF,pDOO_x,iAnim);
    pDOO_y      = interp1(EKF.iEKF,pDOO_y,iAnim);
    pDOO_z      = interp1(EKF.iEKF,pDOO_z,iAnim);
    pIOO_x      = interp1(EKF.iEKF,pIOO_x,iAnim);
    pIOO_y      = interp1(EKF.iEKF,pIOO_y,iAnim);
    pIOO_z      = interp1(EKF.iEKF,pIOO_z,iAnim);
    pIOO_x_e_ip = interp1(EKF.iEKF,pIOO_x_e,iAnim);
    pIOO_y_e_ip = interp1(EKF.iEKF,pIOO_y_e,iAnim);
    pIOO_z_e_ip = interp1(EKF.iEKF,pIOO_z_e,iAnim);
    % a = tic;
    for n = 1 : nAnim
        addpoints(pD, pDOO_x(n), pDOO_y(n), -pDOO_z(n));
        addpoints(pI_true, pIOO_x(n), pIOO_y(n), -pIOO_z(n));
        addpoints(pI, pIOO_x_e_ip(n), pIOO_y_e_ip(n), -pIOO_z_e_ip(n));
        drawnow
    end
end

% Plot confidence interval
% Interpolate data
n3D   = 200;                                                              % number of spheres/cylinder along trajectory
i3D = linspace(1,EKF.nDat,n3D);

pIOO_x_e_ip = interp1(EKF.iEKF,pIOO_x_e,i3D);
pIOO_y_e_ip = interp1(EKF.iEKF,pIOO_y_e,i3D);
pIOO_z_e_ip = interp1(EKF.iEKF,pIOO_z_e,i3D);
pIOO_e_ip   = [pIOO_x_e_ip;pIOO_y_e_ip;pIOO_z_e_ip];
dir_vec = gradient(pIOO_e_ip);
dr_vec  = vecnorm(gradient(pIOO_e_ip));

% Extract standard deviation from covariance matrix
Std_r = interp1(EKF.iEKF,vecnorm(EKF.Std([1 2 3],:)/1),i3D); % /2

% Plot standard deviation of estimation
for j=1:n3D
    
    % Sphere/cylinder radius
    r_j = Std_r(j)^(1/2);
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
            j_sphere = surf(c_x + x, c_y + y, c_z + z,'FaceColor','b','FaceAlpha',0.05, 'EdgeColor', 'none');
        case 'cylinder'
            [x,y,z] = cylinder(r_j);
            j_cylinder = surf(c_x + x, c_y + y, c_z - z*dr_vec(j), 'FaceColor','b', 'FaceAlpha',.1, 'EdgeColor', 'none');
            dir = dir_vec(:,j) / norm(dir_vec(:,j));
            % Rotate cylinder
            r = vrrotvec([0 0 1],dir);
            rotate(j_cylinder,r(1:3),-r(4)*180/pi,[c_x,c_y,c_z]);
            %             java.lang.Thread.sleep(100)
    end
end

% Plot target area
switch Setup.TargetConfig.Type
    case 'Dome'
        [x,y,z] = sphere(Setup.targetOptions.rT_max);
        xEast  = Setup.targetOptions.rT_max * x;
        yNorth = Setup.targetOptions.rT_max * y;
        zUp    = Setup.targetOptions.rT_max * z;
        zUp(zUp < 0) = 0;
        pT = surf(xEast, yNorth, zUp,'FaceColor','y','FaceAlpha',0.3, 'EdgeColor', 'None');
    case 'Cylinder'
        [x,y,z] = cylinder(Setup.TargetConfig.rT_max);
        xEast  = x;
        yNorth = y;
        zUp    = Setup.TargetConfig.hT_max * z;
        zUp(zUp < 0) = 0;
        pT = surf(xEast, yNorth, zUp,'FaceColor','y','FaceAlpha',0.3, 'EdgeColor', 'None');
    case 'Circle'
        n = linspace(0,2*pi);
        x = cos(n) * Setup.TargetConfig.rT_max;
        y = sin(n) * Setup.TargetConfig.rT_max;
        pT = plot(x,y,'-y','LineWidth',2);
    otherwise
        error('Target options are not supported!');
end

% Plot target point
pTOO = Setup.Scenario.pTOO;
plot3(pTOO(1), pTOO(2), -pTOO(3),'yX');


% Plot LOS line only for stationary scenario, thus manual switch
if 1
    % Plot initial LOS line
    % LOS coordinates
    X = [pDOO_x(1);pIOO_x_e(end)];
    Y = [pDOO_y(1);pIOO_y_e(end)];
    Z = [-pDOO_z(1);-pIOO_z_e(end)];
    % Plot LOS lines
    pLOS = plot3(X,Y,Z,'-k','LineWidth',0.1);
    
    legend([pD pI pI_true pT pLOS],...
    {'Defender','Invader Estimated','Invader True', 'Defended Area',...
    'LOS'}, c.Legend{:});
else
    legend([pD pI pI_true pT],...
    {'Defender','Invader Estimated','Invader True', 'Defended Area',},...
    c.Legend{:});
end



% Save plot
if Setup.PostOptions.Save
    if Setup.PostOptions.Jpg
        for i=1:numel(figures)
            saveas(figures(i),fullfile(Setup.PostOptions.PathJpg,fignames(i)),'jpg');
        end
    end
    if Setup.PostOptions.Fig
        savefig(figures,fullfile(Setup.PostOptions.PathFig,'PostObservability'));
    end
end

end