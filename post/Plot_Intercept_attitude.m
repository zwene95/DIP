function Plot_Intercept_attitude(Setup, Results)

c = Setup.PostOptions.c;

N = 50;                                                                 % number of attitude vectors to be displayed
step = Setup.Solver.GridSize / N;                                              % stepsize for attitude plot
N_LOS = 10;                                                             % number of attitude vectors LOS to be displayed
step_LOS = Setup.Solver.GridSize / N_LOS;                                          % stepsize for LOS plot

% Position
pDOO =  Results.Defender.States.Pos;
pIOO =  Results.Invader.States.Pos;

% LOS coordinates
X = [pDOO(1,1:step_LOS:end);pIOO(1,1:step_LOS:end)];
Y = [pDOO(2,1:step_LOS:end);pIOO(2,1:step_LOS:end)];
Z = [pDOO(3,1:step_LOS:end);pIOO(3,1:step_LOS:end)];



% Get attitude
switch Setup.ModelOptions.Optimize
    case 'inv'
        T_x = Results.Invader.Controls.T_x_inv(1:step:end);
        T_y = Results.Invader.Controls.T_y_inv(1:step:end);
        T_z = -Results.Invader.Controls.T_z_inv(1:step:end);
    case 'def'
        if Setup.ModelOptions.Defender.SixDoF
            f_pDOD_x = @(theta,psi) [cos(psi).*cos(theta); sin(psi).*cos(theta); -sin(theta)] * 10;
            Theta = Results.Defender.States.Att(2,1:step:end);
            Psi = Results.Defender.States.Att(3,1:step:end);
            pDOD_x = f_pDOD_x(Theta,Psi);
            T_x = pDOD_x(1,:);
            T_y = pDOD_x(2,:);
            T_z = - pDOD_x(3,:);
        else
            T_x = Results.Defender.Controls.Tx(1:step:end);
            T_y = Results.Defender.Controls.Ty(1:step:end);
            T_z = - Results.Defender.Controls.Tz(1:step:end);
        end
end

Figname = 'DIP with Attitude Information';


% Plot invader
figure('Tag',Figname,'name', Figname,'Position', c.Pos_Groesse_SVGA);
hold on
plot3(pIOO(1,1),pIOO(2,1),-pIOO(3,1),'bO','LineWidth',2.0);             % Initial invader position
plot3(pIOO(1,end),pIOO(2,end),-pIOO(3,end),'bX','LineWidth',2.0)        % Final invader position
if strcmp(Setup.ModelOptions.Optimize,'inv')
    p1 = quiver3(pIOO(1,:),pIOO(2,:),-pIOO(3,:),T_x,T_y,T_z,'r',...
        'Linewidth',2.0);                                               % 3D invader trajectory with attitude
else
    p1 = plot3(pIOO(1,:),pIOO(2,:),-pIOO(3,:),'r','Linewidth',2.0);     % 3D invader trajectory
end

% Plot defender
plot3(pDOO(1,1),pDOO(2,1),-pDOO(3,1),'gO','LineWidth',2.0)              % Initial defender position
plot3(pDOO(1,end),pDOO(2,end),-pDOO(3,end),'gX','LineWidth',2.0)        % Final defender position
if strcmp(Setup.ModelOptions.Optimize,'def')
    p2 = plot3(pDOO(1,:),pDOO(2,:),-pDOO(3,:),'g','Linewidth',2.0);
    p2a = quiver3(pDOO(1,1:step:end),pDOO(2,1:step:end),...
        -pDOO(3,1:step:end),T_x,T_y,T_z,'g','Linewidth',2.0);           % 3D invader trajectory with attitude
else
    p2 = plot3(pDOO(1,:),pDOO(2,:),-pDOO(3,:),'g','Linewidth',2.0);     % 3D defender trajectory
end


% Plot Target area
%     switch setup.targetOptions.Type
%         case 'Dome'
%             [x,y,z] = sphere(setup.targetOptions.rT_max);
%             xEast  = setup.targetOptions.rT_max * x;
%             yNorth = setup.targetOptions.rT_max * y;
%             zUp    = setup.targetOptions.rT_max * z;
%             zUp(zUp < 0) = 0;
%             p3 = surf(xEast, yNorth, zUp,'FaceColor','magenta','FaceAlpha',0.3, 'EdgeColor', 'None');
%         case 'Cylinder'
%             [x,y,z] = cylinder(setup.targetOptions.rT_max);
%             xEast  = x;
%             yNorth = y;
%             zUp    = setup.targetOptions.rT_max * z;
%             zUp(zUp < 0) = 0;
%             p3 = surf(xEast, yNorth, zUp,'FaceColor','magenta','FaceAlpha',0.4, 'EdgeColor', 'None');
%         otherwise
%             error('Target options are not supported!');
%     end

% Plot target
pTOO = Setup.Scenario.pTOO;
p3 = plot3(pTOO(1), pTOO(2), -pTOO(3),'yO','LineWidth',2.0);

% Plot LOS lines
p4 = plot3(X,Y,-Z,'-k','LineWidth',0.5);

hold off

xlabel('X',c.Label{:});
ylabel('Y',c.Label{:});
zlabel('Z',c.Label{:});

legend([p1 p2 p2a p3 p4(1)],'Invader', 'Defender',...
    'Defender Attitude' , 'Target', 'LOS',c.Legend{:})

grid on;
set(gca,'XMinorTick','on');
set(gca,'YMinorTick','on');
set(gca,c.Axes{:});
title([Figname,newline],c.Title{:});
axis image;
view(45,45);

% Save plots
if Setup.PostOptions.Save
    if Setup.PostOptions.Jpg
        saveas(gcf,fullfile(Setup.PostOptions.PathJpg,Figname),'svg');
    end
    if Setup.PostOptions.Fig
        savefig(fullfile(Setup.PostOptions.PathFig,Figname));
    end
end

end