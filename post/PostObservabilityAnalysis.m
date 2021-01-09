function PostObservabilityAnalysis(setup, results)

c = setup.postOptions.c;

%% Pre Processing Results

%   Get States, structure as follows: x_true = [x y z u v w];
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

% Measurements
z_true = [
    results.LOS.azimuth(:,:)
    results.LOS.elevation(:,:)
    ];

% Setup extended Kalman filter
myEKF = EKF_Object;
myEKF.Time          = results.time;
myEKF.States        = x_true;
myEKF.Controls      = u_true;
myEKF.Measurements  = z_true;
myEKF.Interpolate   = true;
myEKF.StepTime      = 10e-03;
myEKF.Sigma_Q       = sqrt(1e2);
myEKF.Sigma_R       = sqrt(1e-2);
myEKF.Sigma_v       = 0e-02;
myEKF.Sigma_w       = 0;
myEKF.Sigma_x0      = 10;
myEKF.Sigma_P0_pos  = 10;
myEKF.Sigma_P0_vel  = sqrt(5e2);

EKF = myEKF.Results;

%% Plot Results
% Preprocessing
% Defender position
nDat = length(results.time);
iDat = linspace(1,nDat,nDat);
nEKF = length(EKF.Time);
iEKF = linspace(1,nDat,nEKF);
pDOO_x = interp1(iDat,results.defender.states.pos(1,:),iEKF);
pDOO_y = interp1(iDat,results.defender.states.pos(2,:),iEKF);
pDOO_z = interp1(iDat,results.defender.states.pos(3,:),iEKF);
% Invader position
pIOO_x  = interp1(iDat,results.invader.states.pos(1,:),iEKF);
pIOO_y  = interp1(iDat,results.invader.states.pos(2,:),iEKF);
pIOO_z  = interp1(iDat,results.invader.states.pos(3,:),iEKF);

figures     = zeros(1,5);
fignames    = strings(size(figures));
nErrBar     = 100;
iErrBar     = round(linspace(1,nEKF,nErrBar));

% Plot true and estimated position
idx             = 1;
fignames(idx)   = 'True and Estimated Relative Position';
figures(idx)    = figure('Tag',fignames(idx),'name', fignames(idx),'Position', c.Pos_Groesse_SVGA);
ax1             = subplot(3,1,1); hold on; grid on;
ptrue           = plot(EKF.Time,EKF.x_true(1,:),'g','LineWidth',2);
pest            = plot(EKF.Time,EKF.x_k_k(1,:),'.r','LineWidth',2);
pstd            = errorbar(EKF.Time(iErrBar),EKF.x_k_k(1,iErrBar),EKF.Sigma(1,iErrBar),'.r','LineWidth',.1);
xlabel('T','Interpreter',c.Interpreter);
ylabel('X','Interpreter',c.Interpreter);
ax2 = subplot(3,1,2); hold on; grid on;
plot(EKF.Time,EKF.x_true(2,:),'g','LineWidth',2);
plot(EKF.Time,EKF.x_k_k(2,:),'.r','LineWidth',2);
errorbar(EKF.Time(iErrBar),EKF.x_k_k(2,iErrBar),EKF.Sigma(2,iErrBar),'.r','LineWidth',.1);
xlabel('T','Interpreter',c.Interpreter);
ylabel('Y','Interpreter',c.Interpreter);
ax3 = subplot(3,1,3); hold on; grid on;
plot(EKF.Time,EKF.x_true(3,:),'g','LineWidth',2);
plot(EKF.Time,EKF.x_k_k(3,:),'.r','LineWidth',2);
errorbar(EKF.Time(iErrBar),EKF.x_k_k(3,iErrBar),EKF.Sigma(3,iErrBar),'.r','LineWidth',.1);
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
ptrue       =   plot(EKF.Time,EKF.x_true(4,:),'g','LineWidth',2);
pest        =   plot(EKF.Time,EKF.x_k_k(4,:),'.r','LineWidth',2);
pstd        =   errorbar(EKF.Time(iErrBar),EKF.x_k_k(4,iErrBar),EKF.Sigma(4,iErrBar),'.r','LineWidth',.1);
xlabel('T','Interpreter',c.Interpreter);
ylabel('u','Interpreter',c.Interpreter);
ax2 = subplot(3,1,2); hold on; grid on;
plot(EKF.Time,EKF.x_true(5,:),'g','LineWidth',2);
plot(EKF.Time,EKF.x_k_k(5,:),'.r','LineWidth',2);
errorbar(EKF.Time(iErrBar),EKF.x_k_k(5,iErrBar),EKF.Sigma(5,iErrBar),'.r','LineWidth',.1);
xlabel('T','Interpreter',c.Interpreter);
ylabel('v','Interpreter',c.Interpreter);
ax3 = subplot(3,1,3); hold on; grid on;
plot(EKF.Time,EKF.x_true(6,:),'g','LineWidth',2);
plot(EKF.Time,EKF.x_k_k(6,:),'.r','LineWidth',2);
errorbar(EKF.Time(iErrBar),EKF.x_k_k(6,iErrBar),EKF.Sigma(6,iErrBar),'.r','LineWidth',.1);
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
ptrue           =   plot(EKF.Time,EKF.z_true(1,:),'-g','LineWidth',2);
pnoise          =   plot(EKF.Time,EKF.z(1,:),'-.b','LineWidth',1);
pest            =   plot(EKF.Time,EKF.Measurements(1,:),'--r','LineWidth',2);
title('Azimuth','FontWeight','bold','FontSize',c.FS_subtitle, 'Interpreter',c.Interpreter);
ax2 = subplot(2,1,1); hold on; grid on;
plot(EKF.Time,EKF.z_true(2,:),'-g','LineWidth',2); grid on;
plot(EKF.Time,EKF.z(2,:),'-.b','LineWidth',1); grid on;
plot(EKF.Time,EKF.Measurements(2,:),'--r','LineWidth',2);
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
plot(EKF.Time(2:end),vecnorm(EKF.Error(1:6,2:end)),'g','LineWidth',2);grid on;
title('RMSE Combined','FontWeight','bold','FontSize',c.FS_subtitle, 'Interpreter',c.Interpreter)
ylabel('[-]','Interpreter',c.Interpreter);
ax2 = subplot(3,3,2);
plot(EKF.Time(2:end),vecnorm(EKF.Error(1:3,2:end)),'g','LineWidth',2);grid on;
title('RMSE Position','FontWeight','bold','FontSize',c.FS_subtitle, 'Interpreter',c.Interpreter)
ylabel('[m]','Interpreter',c.Interpreter);
ax3 = subplot(3,3,3);
plot(EKF.Time(2:end),vecnorm(EKF.Error(4:6,2:end)),'g','LineWidth',2);grid on;
title('RMSE Velocity','FontWeight','bold','FontSize',c.FS_subtitle, 'Interpreter',c.Interpreter)
ylabel('[m/s]','Interpreter',c.Interpreter);
ax4 = subplot(3,3,4);
plot(EKF.Time(2:end),EKF.P_trace(2:end),'g','LineWidth',2);grid on;
title('Covariance Trace Combined','FontWeight','bold','FontSize',c.FS_subtitle, 'Interpreter',c.Interpreter);
ax5 = subplot(3,3,5);
plot(EKF.Time(2:end),EKF.P_trace_pos(2:end),'g','LineWidth',2);grid on;
title('Covariance Trace Position','FontWeight','bold','FontSize',c.FS_subtitle, 'Interpreter',c.Interpreter);
ax6 = subplot(3,3,6);
plot(EKF.Time(2:end),EKF.P_trace_vel(2:end),'g','LineWidth',2);grid on;
title('Covariance Trace Velocity','FontWeight','bold','FontSize',c.FS_subtitle, 'Interpreter',c.Interpreter);
ax7 = subplot(3,3,7);
plot(EKF.Time(2:end),EKF.NEES(2:end),'g','LineWidth',2);grid on;
title('NEES Combined','FontWeight','bold','FontSize',c.FS_subtitle, 'Interpreter',c.Interpreter);
ax8 = subplot(3,3,8);
plot(EKF.Time(2:end),EKF.NEES_pos(2:end),'g','LineWidth',2);grid on;
title('NEES Position','FontWeight','bold','FontSize',c.FS_subtitle, 'Interpreter',c.Interpreter);
ax9 = subplot(3,3,9);
plot(EKF.Time(2:end),EKF.NEES_vel(2:end),'g','LineWidth',2);grid on;
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

% Plot defender position
if options.animated
    pD = animatedline('Color','green','LineStyle','-','LineWidth',2);
else
    pD = plot3(pDOO_x,pDOO_y,-pDOO_z,'-g','LineWidth',2);
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

xlabel('X','Interpreter',c.Interpreter);
ylabel('Y','Interpreter',c.Interpreter);
zlabel('Z','Interpreter',c.Interpreter);
title(fignames(idx),...
    'FontWeight','bold','FontSize',c.FS_title, 'Interpreter',c.Interpreter);
axis image;
grid on;
view(45,45);
set(gca,'TickLabelInterpreter',c.Interpreter)
lgd = legend([pD pI pI_true],...
    {'Defender','Invader Estimated','Invader True'},...
    'FontSize',c.FS_Legend,'Interpreter',c.Interpreter);
tmp = sprintf('RMSE_{pos} = %.2fm\n\x03c3_{pos} = %.2fm\n\x03A3_{pos} = %.2fm',...
    norm(EKF.Error(1:3,end)),norm(EKF.Sigma(1:3,end)),sum(vecnorm(EKF.Sigma(1:3,:)))/nEKF);
annotation('textbox',lgd.Position - [0 .1 0 0],'String',tmp,'FitBoxToText','on','BackgroundColor','w');

if options.animated
    % Animate Trajectories
    nAnim       = 200;                                                        % number of animation points
    iAnim       = linspace(1,nDat,nAnim);
    pDOO_x      = interp1(iEKF,pDOO_x,iAnim);
    pDOO_y      = interp1(iEKF,pDOO_y,iAnim);
    pDOO_z      = interp1(iEKF,pDOO_z,iAnim);
    pIOO_x      = interp1(iEKF,pIOO_x,iAnim);
    pIOO_y      = interp1(iEKF,pIOO_y,iAnim);
    pIOO_z      = interp1(iEKF,pIOO_z,iAnim);
    pIOO_x_e_ip = interp1(iEKF,pIOO_x_e,iAnim);
    pIOO_y_e_ip = interp1(iEKF,pIOO_y_e,iAnim);
    pIOO_z_e_ip = interp1(iEKF,pIOO_z_e,iAnim);
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
i3D = linspace(1,nDat,n3D);

pIOO_x_e_ip = interp1(iEKF,pIOO_x_e,i3D);
pIOO_y_e_ip = interp1(iEKF,pIOO_y_e,i3D);
pIOO_z_e_ip = interp1(iEKF,pIOO_z_e,i3D);
pIOO_e_ip   = [pIOO_x_e_ip;pIOO_y_e_ip;pIOO_z_e_ip];
dir_vec = gradient(pIOO_e_ip);
dr_vec  = vecnorm(gradient(pIOO_e_ip));

% Extract standard deviation from covariance matrix
Sigma_r = interp1(iEKF,vecnorm(EKF.Sigma([1 2 3],:)/2),i3D);

% Plot standard deviation of estimation
for j=1:n3D
    
    % Sphere/cylinder radius
    r_j = Sigma_r(j)^(1/2);
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
    switch setup.targetConfig.Type
        case 'Dome'
            [x,y,z] = sphere(setup.targetOptions.rT_max);
            xEast  = setup.targetOptions.rT_max * x;
            yNorth = setup.targetOptions.rT_max * y;
            zUp    = setup.targetOptions.rT_max * z;
            zUp(zUp < 0) = 0;
            pT = surf(xEast, yNorth, zUp,'FaceColor','y','FaceAlpha',0.3, 'EdgeColor', 'None');
        case 'Cylinder'
            [x,y,z] = cylinder(setup.targetConfig.rT_max);
            xEast  = x;
            yNorth = y;
            zUp    = setup.targetConfig.hT_max * z;
            zUp(zUp < 0) = 0;
            pT = surf(xEast, yNorth, zUp,'FaceColor','y','FaceAlpha',0.3, 'EdgeColor', 'None');
        case 'Circle'
            n = linspace(0,2*pi);
            x = cos(n) * setup.targetConfig.rT_max;
            y = sin(n) * setup.targetConfig.rT_max;
            pT = plot(x,y,'-y','LineWidth',2);
        otherwise
            error('Target options are not supported!');
    end
    
    % Plot target point
    pTOO = setup.scenario.pTOO;
    plot3(pTOO(1), pTOO(2), -pTOO(3),'yX');


legend([pD pI pI_true, pT],...
    {'Defender','Invader Estimated','Invader True', 'Defended Area'},...
    'FontSize',c.FS_Legend,'Interpreter',c.Interpreter);



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