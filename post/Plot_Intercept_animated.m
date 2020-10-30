function Plot_Intercept_animated(setup, problem, c)

   
    pDOO =  [   problem.StateValues(find(ismember(problem.StateNames,'x'),1),:)
                problem.StateValues(find(ismember(problem.StateNames,'y'),1),:)
                - problem.StateValues(find(ismember(problem.StateNames,'z'),1),:)
            ];
        
    pIOO =  [   problem.StateValues(find(ismember(problem.StateNames,'x_inv'),1),:)
                problem.StateValues(find(ismember(problem.StateNames,'y_inv'),1),:)
                - problem.StateValues(find(ismember(problem.StateNames,'z_inv'),1),:)
            ];
    

    figname = 'Intercept Animated';
    figure('Tag',figname,'name', figname,'Position', c.Pos_Groesse_SVGA,'Renderer','opengl');
    title([figname,newline],'FontWeight','bold','FontSize',c.FS_title);
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    grid on;
    hold on;
    set(gca,'XMinorTick','on');
    set(gca,'YMinorTick','on');
    set(gca,'FontSize',c.FS_plot);    
    axis image;
    trajectoryInterceptor = animatedline('LineWidth',2,'Color','green');
    plot3(pDOO(1,1),pDOO(2,1),pDOO(3,1),'gX','LineWidth',2);
    trajectoryInvader =     animatedline('LineWidth',2,'Color','red');
    plot3(pIOO(1,1),pIOO(2,1),pIOO(3,1),'rX','LineWidth',2);
    view(-45,30);
    
    
% Plot target area    
    switch setup.targetConfig.Type
        case 'Dome'
            [x,y,z] = sphere(setup.targetOptions.rT_max);
            xEast  = setup.targetOptions.rT_max * x;
            yNorth = setup.targetOptions.rT_max * y;
            zUp    = setup.targetOptions.rT_max * z;
            zUp(zUp < 0) = 0;
            target = surf(xEast, yNorth, zUp,'FaceColor','y','FaceAlpha',0.3, 'EdgeColor', 'None');
        case 'Cylinder'
            [x,y,z] = cylinder(setup.targetConfig.rT_max);
            xEast  = x;
            yNorth = y;
            zUp    = setup.targetConfig.hT_max * z;
            zUp(zUp < 0) = 0;
            target = surf(xEast, yNorth, zUp,'FaceColor','y','FaceAlpha',0.3, 'EdgeColor', 'None');
        case 'Circle'
            n = linspace(0,2*pi);
            x = cos(n) * setup.targetConfig.rT_max;
            y = sin(n) * setup.targetConfig.rT_max;
            target = plot(x,y,'-y','LineWidth',2);
        otherwise
            error('Target options are not supported!');
    end
    
    % Plot target point
    pTOO = setup.scenario.pTOO;
    plot3(pTOO(1), pTOO(2), -pTOO(3),'yX');
    
    legend([trajectoryInvader, trajectoryInterceptor, target],'Invader', 'Interceptor', 'Target')
    
    
    % Animate Trajectories
    for n = 1 : length(pDOO)
        addpoints(trajectoryInterceptor, pDOO(1,n), pDOO(2,n), pDOO(3,n));
        addpoints(trajectoryInvader, pIOO(1,n), pIOO(2,n), pIOO(3,n));
        drawnow;
    end
    
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