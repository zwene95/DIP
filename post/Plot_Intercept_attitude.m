function Plot_Intercept_attitude(setup, problem, c)

    N = 50;                                                                 % number of attitude vectors to be displayed
    step = setup.solver.gridSize / N;                                              % stepsize for attitude plot
    N_LOS = 10;                                                             % number of attitude vectors LOS to be displayed
    step_LOS = setup.solver.gridSize / N_LOS;                                          % stepsize for LOS plot

    % Position
    pDOO =  [   
        problem.StateValues(find(ismember(problem.StateNames,'x'),1),:)
        problem.StateValues(find(ismember(problem.StateNames,'y'),1),:)
        - problem.StateValues(find(ismember(problem.StateNames,'z'),1),:)
    ];
        
    pIOO =  [   
        problem.StateValues(find(ismember(problem.StateNames,'x_inv'),1),:)
        problem.StateValues(find(ismember(problem.StateNames,'y_inv'),1),:)
        - problem.StateValues(find(ismember(problem.StateNames,'z_inv'),1),:)
    ];
        
    % LOS coordinates    
    X = [pDOO(1,1:step_LOS:end);pIOO(1,1:step_LOS:end)];
    Y = [pDOO(2,1:step_LOS:end);pIOO(2,1:step_LOS:end)];
    Z = [pDOO(3,1:step_LOS:end);pIOO(3,1:step_LOS:end)];   
        
        
        
    % Get attitude    
    switch setup.modelOptions.optimize
        case 'inv'
            T_x = problem.ControlValues(find(ismember(problem.ControlNames,'T_x_inv'),1),1:step:end);
            T_y = problem.ControlValues(find(ismember(problem.ControlNames,'T_y_inv'),1),1:step:end);
            T_z = - problem.ControlValues(find(ismember(problem.ControlNames,'T_z_inv'),1),1:step:end);
        case 'def'
            if setup.modelOptions.defender.SixDoF
                f_pDOD_x = @(theta,psi) [cos(psi).*cos(theta); sin(psi).*cos(theta); -sin(theta)] * 10;
                theta = problem.StateValues(find(ismember(problem.StateNames,'theta'),1),1:step:end);
                psi = problem.StateValues(find(ismember(problem.StateNames,'psi'),1),1:step:end);
                pDOD_x = f_pDOD_x(theta,psi);
                T_x = pDOD_x(1,:);
                T_y = pDOD_x(2,:);
                T_z = - pDOD_x(3,:);
            else
                T_x = problem.ControlValues(find(ismember(problem.ControlNames,'T_x'),1),1:step:end);
                T_y = problem.ControlValues(find(ismember(problem.ControlNames,'T_y'),1),1:step:end);
                T_z = - problem.ControlValues(find(ismember(problem.ControlNames,'T_z'),1),1:step:end);
            end
    end    

    figname = 'Intercept + Attitude';
    
    
    % Plot invader
    figure('Tag',figname,'name', figname,'Position', c.Pos_Groesse_SVGA);
    hold on
    plot3(pIOO(1,1),pIOO(2,1),pIOO(3,1),'rX');                                          % Initial invader position        
    if strcmp(setup.modelOptions.optimize,'inv')
        p1 = quiver3(pIOO(1,:),pIOO(2,:),pIOO(3,:),T_x,T_y,T_z,'r','Linewidth',1.5);    % 3D invader trajectory with attitude
    else
        p1 = plot3(pIOO(1,:),pIOO(2,:),pIOO(3,:),'r','Linewidth',1.5);                  % 3D invader trajectory
    end
    
    % Plot defender
    plot3(pDOO(1,1),pDOO(2,1),pDOO(3,1),'gX')                                           % Initial defender position        
    if strcmp(setup.modelOptions.optimize,'def')
        p2 = plot3(pDOO(1,:),pDOO(2,:),pDOO(3,:),'g','Linewidth',1.5);
        p2a = quiver3(pDOO(1,1:step:end),pDOO(2,1:step:end),pDOO(3,1:step:end),T_x,T_y,T_z,'g','Linewidth',1.5);    % 3D invader trajectory with attitude
    else
        p2 = plot3(pDOO(1,:),pDOO(2,:),pDOO(3,:),'g','Linewidth',1.5);                  % 3D defender trajectory
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
    pTOO = setup.scenario.pTOO;
    p3 = plot3(pTOO(1), pTOO(2), -pTOO(3),'mX');
    
    % Plot LOS lines
    p4 = plot3(X,Y,Z,'-k','LineWidth',0.1);

    hold off

    xlabel('X');
    ylabel('Y');
    zlabel('Z');

    legend([p1 p2 p2a p3 p4(1)],'Invader', 'Defender', 'Defender Attitude' , 'Target', 'LOS')

    grid on;
    set(gca,'XMinorTick','on');
    set(gca,'YMinorTick','on');
    set(gca,'FontSize',c.FS_plot);
    title([figname,newline],'FontWeight','bold','FontSize',c.FS_title);
    axis image;
    view(45,45);

    % Save plots
    if setup.postOptions.Save
        if setup.postOptions.Jpg
            saveas(gcf,fullfile(setup.postOptions.PathJpg,figname),'jpg');
        end
        if setup.postOptions.Fig
            savefig(fullfile(setup.postOptions.PathFig,figname));
        end
    end

end